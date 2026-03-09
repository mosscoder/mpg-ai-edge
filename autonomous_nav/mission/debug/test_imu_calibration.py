#!/usr/bin/env python3
"""
Test IMU Calibration & Walk North

Debug script that uses the nav_utils navigation stack to:
1. Connect GPS, wait for RTK fix
2. Calibrate the IMU by walking forward (~1.5m displacement)
3. Turn to face north (0° heading)
4. Walk north for 10 seconds at 0.5 m/s (dead reckoning, no heading correction)

Logs GPS position at 2 Hz during the walk for post-analysis.

Environment Variables:
    ROBOT_IP         - Go2's IP address on local network (required for LocalSTA)
    GPS_PORT         - GPS serial port (default: /dev/ttyACM0)
    GPS_BAUD         - GPS baud rate (default: 38400)
    EMLID_USERNAME   - NTRIP username
    EMLID_PASSWORD   - NTRIP password
    EMLID_MOUNTPOINT - NTRIP mountpoint (default: MP1979)

Usage:
    python autonomous_nav/mission/debug/test_imu_calibration.py
"""

import asyncio
import logging
import os
import sys
import time

# Add repo root to path for imports
sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
)

from autonomous_nav.nav_utils import (
    GPSManager,
    Go2Robot,
    NTRIPConfig,
    Waypoint,
    WaypointNavigator,
    _log_banner,
    haversine_distance,
    normalize_angle,
)

# =============================================================================
# CONFIGURATION
# =============================================================================

GPS_PORT = os.getenv("GPS_PORT", "/dev/ttyACM0")
GPS_BAUD = int(os.getenv("GPS_BAUD", "38400"))

NTRIP_CONFIG = NTRIPConfig(
    host=os.getenv("EMLID_NTRIP_HOST", "caster.emlid.com"),
    port=int(os.getenv("EMLID_NTRIP_PORT", "2101")),
    mountpoint=os.getenv("EMLID_MOUNTPOINT", "MP1979"),
    username=os.getenv("EMLID_USERNAME", ""),
    password=os.getenv("EMLID_PASSWORD", ""),
)

CONNECTION_MODE = os.getenv("CONNECTION_MODE", "LocalSTA")
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.1.105")
ROBOT_SERIAL = os.getenv("ROBOT_SERIAL", None)

CALIBRATION_WALK_SPEED = 0.3  # m/s during IMU calibration
WALK_NORTH_SPEED = 0.5  # m/s during the northward walk
WALK_NORTH_DURATION = 10.0  # seconds
GPS_LOG_INTERVAL = 0.5  # seconds between GPS logs during walk
HEADING_TOLERANCE = 5.0  # degrees — close enough to north
TURN_RATE = 0.3  # rad/s rotation speed
MIN_FIX_TYPE = 5  # RTK Float or better
GPS_FIX_TIMEOUT = 300  # seconds


# =============================================================================
# LOGGING SETUP
# =============================================================================


def setup_logging():
    log_dir = os.path.join(os.path.dirname(__file__), "logs")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    logfile = os.path.join(log_dir, f"test_imu_calibration_{timestamp}.log")

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(logfile),
        ],
    )
    logging.getLogger("autonomous_nav.nav_utils").setLevel(logging.DEBUG)
    return logfile


# =============================================================================
# MAIN
# =============================================================================


async def run():
    logger = logging.getLogger(__name__)

    print("=" * 60)
    print("Test IMU Calibration & Walk North")
    print("=" * 60)

    gps = GPSManager(port=GPS_PORT, baudrate=GPS_BAUD, ntrip_config=NTRIP_CONFIG)
    robot = Go2Robot(
        connection_mode=CONNECTION_MODE,
        robot_ip=ROBOT_IP,
        robot_serial=ROBOT_SERIAL,
    )

    try:
        # =================================================================
        # Phase 1: Setup — GPS + Robot
        # =================================================================
        _log_banner("PHASE 1: SETUP")

        print("\n[1/4] Connecting to GPS...")
        if not gps.connect(use_ntrip=bool(NTRIP_CONFIG.username)):
            print("ERROR: Failed to connect to GPS")
            return False
        print("GPS connected")

        print(f"Waiting for GPS fix (min type {MIN_FIX_TYPE})...")
        if not gps.wait_for_fix(timeout=GPS_FIX_TIMEOUT, min_fix_type=MIN_FIX_TYPE):
            print("ERROR: GPS fix timeout")
            return False

        start_pos = gps.get_position()
        if start_pos:
            print(
                f"Start position: ({start_pos.latitude:.8f}, {start_pos.longitude:.8f}) "
                f"hAcc: {start_pos.accuracy_horizontal:.3f}m"
            )

        print("\n[2/4] Connecting to robot...")
        if not await robot.connect():
            print("ERROR: Failed to connect to robot")
            return False
        await robot.prepare_for_navigation()
        print("Robot ready")

        # =================================================================
        # Phase 2: IMU Calibration
        # =================================================================
        _log_banner("PHASE 2: IMU CALIBRATION")

        # WaypointNavigator gives us calibration + heading infrastructure.
        # We pass a dummy waypoint — we won't use navigate_to().
        navigator = WaypointNavigator(
            gps=gps,
            robot=robot,
            max_velocity=CALIBRATION_WALK_SPEED,
        )

        print("\nWalking forward to calibrate IMU...")
        calibrated = False
        cal_start = time.time()

        while not calibrated:
            pos = gps.get_position()
            imu_yaw = robot.get_yaw_degrees()

            if pos is None or imu_yaw is None:
                await asyncio.sleep(0.2)
                continue

            calibrated = navigator._calibrate_imu(pos, imu_yaw)

            # Keep walking forward during calibration
            if not calibrated:
                await robot.send_velocity(x=CALIBRATION_WALK_SPEED)

            await asyncio.sleep(0.2)

        await robot.stop()
        await asyncio.sleep(0.5)

        logger.info(
            f"North offset: {navigator._imu_north_offset:.1f}°  |  "
            f"Current heading: {navigator.get_calibrated_heading():.1f}°"
        )

        # =================================================================
        # Phase 3: Turn to Face North
        # =================================================================
        _log_banner("PHASE 3: TURN TO NORTH")

        print("\nRotating to face north (0°)...")
        aligned = False
        while not aligned:
            heading = navigator.get_calibrated_heading()
            if heading is None:
                await asyncio.sleep(0.1)
                continue

            # Error to 0° (north)
            error = normalize_angle(0 - heading)

            if abs(error) < HEADING_TOLERANCE:
                aligned = True
                await robot.stop()
                logger.info(f"Aligned to north — heading: {heading:.1f}° (error: {error:.1f}°)")
                break

            # Rotate toward north
            direction = 1.0 if error > 0 else -1.0
            await robot.send_velocity(z=direction * TURN_RATE)
            await asyncio.sleep(0.1)

        await asyncio.sleep(0.5)
        final_heading = navigator.get_calibrated_heading()
        print(f"Heading after alignment: {final_heading:.1f}°")

        # =================================================================
        # Phase 4: Walk North for 10s (dead reckoning)
        # =================================================================
        _log_banner("PHASE 4: WALK NORTH 10s @ 0.5 m/s")

        walk_start_pos = gps.get_position()
        print(f"\nWalking north for {WALK_NORTH_DURATION}s at {WALK_NORTH_SPEED} m/s...")

        walk_start = time.time()
        last_log = 0.0

        while time.time() - walk_start < WALK_NORTH_DURATION:
            await robot.send_velocity(x=WALK_NORTH_SPEED)

            now = time.time()
            if now - last_log >= GPS_LOG_INTERVAL:
                pos = gps.get_position()
                if pos:
                    elapsed = now - walk_start
                    logger.info(
                        f"[walk {elapsed:5.1f}s] "
                        f"({pos.latitude:.8f}, {pos.longitude:.8f}) "
                        f"fix={pos.fix_type} hAcc={pos.accuracy_horizontal:.3f}m"
                    )
                last_log = now

            await asyncio.sleep(0.1)

        await robot.stop()
        logger.info("Walk complete — robot stopped")

        # Final report
        final_pos = gps.get_position()
        if final_pos and walk_start_pos:
            total_dist = haversine_distance(
                walk_start_pos.latitude, walk_start_pos.longitude,
                final_pos.latitude, final_pos.longitude,
            )
            logger.info(
                f"Final position: ({final_pos.latitude:.8f}, {final_pos.longitude:.8f})"
            )
            logger.info(f"Distance covered (GPS): {total_dist:.2f}m")

        if final_pos and start_pos:
            total_from_origin = haversine_distance(
                start_pos.latitude, start_pos.longitude,
                final_pos.latitude, final_pos.longitude,
            )
            logger.info(f"Total distance from origin: {total_from_origin:.2f}m")

        _log_banner("TEST COMPLETE")
        return True

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        logger.info("Interrupted by KeyboardInterrupt")
        try:
            await robot.stop()
        except Exception:
            pass
        return False

    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        print(f"\nERROR: {e}")
        try:
            await robot.stop()
        except Exception:
            pass
        return False

    finally:
        print("\nCleaning up...")
        try:
            await robot.stop()
        except Exception:
            pass
        gps.disconnect()
        print("Done")


def main():
    logfile = setup_logging()
    print(f"Logging to: {logfile}")
    try:
        success = asyncio.run(run())
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(1)


if __name__ == "__main__":
    main()
