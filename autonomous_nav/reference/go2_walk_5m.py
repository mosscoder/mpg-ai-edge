"""
Go2 Walk Forward 5 Meters

Uses: unitree_webrtc_connect (legion1581)
https://github.com/legion1581/unitree_webrtc_connect

Install:
    pip install unitree_webrtc_connect

================================================================================
CONNECTION MODES
================================================================================

1. LocalAP - Direct connection to robot's hotspot
   - Connect your laptop to Go2's WiFi (e.g., "GO2-XXXXXX")
   - No IP needed, robot is always at 192.168.12.1
   - Laptop loses internet access
   - Single client only

2. LocalSTA - Robot and laptop on same WiFi network  
   - Configure via Unitree Go app: Bind robot → Wi-Fi connection mode
   - Need to find robot's IP on your network (see below)
   - Laptop keeps internet access
   - Better range (depends on your WiFi)

================================================================================
FINDING THE ROBOT'S IP (for LocalSTA mode)
================================================================================

Option 1 - nmap scan (replace with your subnet):
    
    nmap -sn 192.168.1.0/24 | grep -i -B2 "unitree\|go2"
    
Option 2 - arp scan:

    sudo arp-scan --localnet | grep -i unitree

Option 3 - Check router's DHCP leases:
    
    - Login to router admin page (usually 192.168.1.1)
    - Look for connected devices / DHCP leases
    - Hostname contains serial number (e.g., "B42D2000XXXXXXXX")

Option 4 - Multicast discovery (built into the driver):
    
    # If you know serial number but not IP, the driver can find it:
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalSTA, 
        serialNumber="B42D2000XXXXXXXX"
    )

================================================================================
TROUBLESHOOTING
================================================================================

"Robot API not responding":
    - Close the Unitree app on your phone (only one client at a time without token)
    - Verify IP address is correct
    - Check firmware version (supported: 1.0.19-1.0.25, 1.1.1-1.1.11)
    - Ensure robot is powered on and standing

"Connection refused":
    - Wrong IP or robot not on network
    - Try LocalAP mode to verify robot is working

================================================================================
"""

import asyncio
import json
import logging
import sys
import time

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ==============================================================================
# CONFIGURATION - EDIT THESE
# ==============================================================================

# Connection mode: "LocalAP" or "LocalSTA"
CONNECTION_MODE = "LocalAP"

# For LocalSTA only - robot's IP on your network
ROBOT_IP = "192.168.1.105"

# For LocalSTA only - alternative: use serial number (driver will find IP)
# Set to None to use ROBOT_IP instead
ROBOT_SERIAL = None  # e.g., "B42D2000XXXXXXXX"

# Movement
DISTANCE_METERS = 5.0
VELOCITY_MPS = 0.5  # m/s (max ~1.5 in normal mode)
RAMP_TIME = 0.5     # seconds to ramp up/down

# Timeouts
API_READY_TIMEOUT = 30
MODE_SWITCH_WAIT = 5


# ==============================================================================
# API HELPERS
# ==============================================================================

async def wait_for_robot_api(conn, timeout: float = API_READY_TIMEOUT) -> bool:
    """Poll MOTION_SWITCHER until robot responds."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            resp = await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {"api_id": 1001}
            )
            if resp["data"]["header"]["status"]["code"] == 0:
                logger.info("Robot API ready")
                return True
        except Exception as e:
            logger.debug(f"API not ready: {e}")
        await asyncio.sleep(0.5)
    
    logger.error(f"Timeout after {timeout}s waiting for robot API")
    return False


async def get_motion_mode(conn) -> str:
    """Get current motion mode name."""
    resp = await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["MOTION_SWITCHER"],
        {"api_id": 1001}
    )
    if resp["data"]["header"]["status"]["code"] != 0:
        raise RuntimeError(f"Failed to get motion mode: {resp}")
    return json.loads(resp["data"]["data"]).get("name", "unknown")


async def set_motion_mode(conn, mode: str):
    """Set motion mode ('normal', 'ai', 'advanced')."""
    logger.info(f"Setting motion mode to '{mode}'...")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["MOTION_SWITCHER"],
        {"api_id": 1002, "parameter": {"name": mode}}
    )
    await asyncio.sleep(MODE_SWITCH_WAIT)


async def ensure_normal_mode(conn):
    """Ensure robot is in 'normal' mode for walking."""
    mode = await get_motion_mode(conn)
    logger.info(f"Current motion mode: {mode}")
    
    if mode != "normal":
        await set_motion_mode(conn, "normal")
        logger.info("Switched to normal mode")


async def balance_stand(conn):
    """Issue BalanceStand to unlock joints."""
    logger.info("BalanceStand...")
    resp = await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        {"api_id": SPORT_CMD["BalanceStand"]}
    )
    if resp["data"]["header"]["status"]["code"] != 0:
        logger.warning("BalanceStand returned non-zero status")
    return resp


async def send_velocity(conn, x: float = 0, y: float = 0, z: float = 0):
    """Send velocity command (x=forward, y=strafe left, z=rotate CCW)."""
    return await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        {
            "api_id": SPORT_CMD["Move"],
            "parameter": {"x": x, "y": y, "z": z}
        }
    )


async def stop_move(conn):
    """Explicit stop command (API 1003)."""
    logger.info("Stopping...")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        {"api_id": SPORT_CMD["StopMove"]}
    )


async def move_distance(conn, distance: float, velocity: float, ramp_time: float = 0.5):
    """
    Move forward with velocity ramping.
    
    Note: Dead reckoning only - no odometry feedback via WebRTC.
    Actual distance varies with terrain, battery, etc.
    """
    ramp_distance = velocity * ramp_time
    cruise_distance = max(0, distance - ramp_distance)
    cruise_time = cruise_distance / velocity if velocity > 0 else 0
    total_time = 2 * ramp_time + cruise_time
    
    logger.info(f"Moving {distance}m at {velocity}m/s")
    logger.info(f"  Ramp: {ramp_time}s | Cruise: {cruise_time:.1f}s | Total: {total_time:.1f}s")
    
    # Ramp up
    steps = 10
    for i in range(steps):
        v = velocity * (i + 1) / steps
        await send_velocity(conn, x=v)
        await asyncio.sleep(ramp_time / steps)
    
    # Cruise
    await send_velocity(conn, x=velocity)
    await asyncio.sleep(cruise_time)
    
    # Ramp down
    for i in range(steps):
        v = velocity * (steps - i - 1) / steps
        await send_velocity(conn, x=v)
        await asyncio.sleep(ramp_time / steps)
    
    await stop_move(conn)


def create_connection():
    """Create connection based on configuration."""
    if CONNECTION_MODE == "LocalAP":
        logger.info("Using LocalAP mode (connect laptop to robot's hotspot)")
        return UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    
    elif CONNECTION_MODE == "LocalSTA":
        if ROBOT_SERIAL:
            logger.info(f"Using LocalSTA mode (serial: {ROBOT_SERIAL}, auto-discover IP)")
            return UnitreeWebRTCConnection(
                WebRTCConnectionMethod.LocalSTA, 
                serialNumber=ROBOT_SERIAL
            )
        else:
            logger.info(f"Using LocalSTA mode (IP: {ROBOT_IP})")
            return UnitreeWebRTCConnection(
                WebRTCConnectionMethod.LocalSTA, 
                ip=ROBOT_IP
            )
    else:
        raise ValueError(f"Unknown CONNECTION_MODE: {CONNECTION_MODE}")


# ==============================================================================
# MAIN
# ==============================================================================

async def main():
    logger.info("=" * 60)
    logger.info("Go2 Walk Forward")
    logger.info(f"Distance: {DISTANCE_METERS}m | Velocity: {VELOCITY_MPS}m/s")
    logger.info("=" * 60)
    
    conn = create_connection()
    
    try:
        await conn.connect()
        logger.info("WebRTC connected")
        
        if not await wait_for_robot_api(conn):
            raise RuntimeError(
                "Robot API not responding. Check:\n"
                "  - Close Unitree app on phone (single client limit)\n"
                "  - Verify IP address / connection mode\n"
                "  - Supported firmware: 1.0.19-1.0.25 or 1.1.1-1.1.11"
            )
        
        await ensure_normal_mode(conn)
        await balance_stand(conn)
        await asyncio.sleep(1)
        
        await move_distance(conn, DISTANCE_METERS, VELOCITY_MPS, RAMP_TIME)
        
        logger.info("=" * 60)
        logger.info(f"Done - traveled ~{DISTANCE_METERS}m (dead reckoning)")
        logger.info("=" * 60)
        
        await asyncio.sleep(2)
        
    except Exception as e:
        logger.error(f"Error: {e}")
        try:
            await stop_move(conn)
        except:
            pass
        raise


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(0)
