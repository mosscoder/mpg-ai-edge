#!/usr/bin/env python3
"""
Mission 01: Multi-Waypoint Navigation

Navigates the Go2 robot to all waypoints in tennis_court_points.geojson
sequentially (point 0 -> point 1 -> stop).

Requirements:
- SparkFun ZED-F9P GPS connected to Jetson (default: /dev/ttyACM0)
- Go2 and Jetson on same WiFi network (LocalSTA mode)
- NTRIP credentials set via environment variables

Environment Variables:
    ROBOT_IP         - Go2's IP address on local network (required)
    GPS_PORT         - GPS serial port (default: /dev/ttyACM0)
    GPS_BAUD         - GPS baud rate (default: 38400)
    EMLID_USERNAME   - NTRIP username
    EMLID_PASSWORD   - NTRIP password
    EMLID_MOUNTPOINT - NTRIP mountpoint (default: MP1979)

Usage:
    python autonomous_nav/mission/mission_01.py
"""

import asyncio
import logging
import os
import sys
import time

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from autonomous_nav.nav_utils import (
    GPSManager,
    Go2Robot,
    NTRIPConfig,
    WaypointNavigator,
    haversine_distance,
    load_waypoints,
)

# =============================================================================
# CONFIGURATION
# =============================================================================

# GPS Configuration
GPS_PORT = os.getenv("GPS_PORT", "/dev/ttyACM0")
GPS_BAUD = int(os.getenv("GPS_BAUD", "38400"))

# NTRIP Configuration
NTRIP_CONFIG = NTRIPConfig(
    host=os.getenv("EMLID_NTRIP_HOST", "caster.emlid.com"),
    port=int(os.getenv("EMLID_NTRIP_PORT", "2101")),
    mountpoint=os.getenv("EMLID_MOUNTPOINT", "MP1979"),
    username=os.getenv("EMLID_USERNAME", ""),
    password=os.getenv("EMLID_PASSWORD", ""),
)

# Robot Configuration
CONNECTION_MODE = os.getenv("CONNECTION_MODE", "LocalSTA")
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.1.105")
ROBOT_SERIAL = os.getenv("ROBOT_SERIAL", None)

# Navigation Configuration
WAYPOINT_FILE = os.path.join(
    os.path.dirname(__file__), "..", "..", "data", "vector", "tennis_court_points.geojson"
)
ARRIVAL_TOLERANCE = 0.2  # meters - stop when within this distance
MAX_VELOCITY = 0.3  # m/s - conservative for precision
ROTATION_RATE = 0.3  # rad/s - rotation speed when turning
GPS_FIX_TIMEOUT = 300  # seconds to wait for GPS fix
MIN_FIX_TYPE = 5  # Minimum fix quality (5=RTK Float or better)

# =============================================================================
# LOGGING SETUP
# =============================================================================


def setup_logging():
    """Configure logging to console and file."""
    log_dir = os.path.join(os.path.dirname(__file__), "logs")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    logfile = os.path.join(log_dir, f"mission_01_{timestamp}.log")

    # Root logger config
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(logfile),
        ],
    )

    # Set nav_utils to DEBUG for detailed navigation logs
    logging.getLogger("autonomous_nav.nav_utils").setLevel(logging.DEBUG)

    return logfile


# =============================================================================
# MAIN MISSION
# =============================================================================


async def run_mission():
    """Execute the multi-waypoint navigation mission."""
    logger = logging.getLogger(__name__)

    print("=" * 60)
    print("RTK Mission 01: Multi-Waypoint Navigation")
    print("=" * 60)
    print(f"Target tolerance: {ARRIVAL_TOLERANCE}m")
    print(f"Max velocity: {MAX_VELOCITY} m/s")
    print(f"Min fix type: {MIN_FIX_TYPE} (RTK Float or better)")
    print(f"GPS Port: {GPS_PORT}")
    print(f"NTRIP: {NTRIP_CONFIG.host}:{NTRIP_CONFIG.port}/{NTRIP_CONFIG.mountpoint}")
    print("=" * 60)

    # Initialize components
    gps = GPSManager(
        port=GPS_PORT,
        baudrate=GPS_BAUD,
        ntrip_config=NTRIP_CONFIG,
    )
    robot = Go2Robot(
        connection_mode=CONNECTION_MODE,
        robot_ip=ROBOT_IP,
        robot_serial=ROBOT_SERIAL,
    )

    try:
        # --- Step 1: Connect to GPS ---
        print("\n[1/4] Connecting to GPS...")
        if not gps.connect(use_ntrip=bool(NTRIP_CONFIG.username)):
            print("ERROR: Failed to connect to GPS")
            return False
        print("GPS connected")

        # --- Step 2: Wait for GPS fix ---
        print(f"\n[2/4] Waiting for GPS fix (min type {MIN_FIX_TYPE})...")
        if not gps.wait_for_fix(timeout=GPS_FIX_TIMEOUT, min_fix_type=MIN_FIX_TYPE):
            print("ERROR: GPS fix timeout")
            return False

        # Get initial position
        initial_pos = gps.get_position()
        if initial_pos:
            print(f"Initial position: ({initial_pos.latitude:.8f}, {initial_pos.longitude:.8f})")
            print(f"Accuracy: {initial_pos.accuracy_horizontal:.3f}m horizontal")

        # --- Step 3: Connect to robot ---
        print("\n[3/4] Connecting to Go2 robot...")
        if not await robot.connect():
            print("ERROR: Failed to connect to robot")
            print("  - Ensure laptop is connected to Go2's WiFi hotspot")
            print("  - Close Unitree app on phone (single client limit)")
            return False
        print("Robot connected")

        # Prepare robot for walking
        await robot.prepare_for_navigation()
        print("Robot ready for navigation")

        # --- Step 4: Load waypoints and navigate ---
        print("\n[4/4] Loading waypoints and starting navigation...")
        waypoints = load_waypoints(WAYPOINT_FILE)
        if not waypoints:
            print(f"ERROR: No waypoints found in {WAYPOINT_FILE}")
            return False

        print(f"Loaded {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            print(f"  {i+1}. {wp.name}: ({wp.latitude:.8f}, {wp.longitude:.8f})")

        # Create navigator
        navigator = WaypointNavigator(
            gps=gps,
            robot=robot,
            max_velocity=MAX_VELOCITY,
            rotation_rate=ROTATION_RATE,
            arrival_tolerance=ARRIVAL_TOLERANCE,
            min_fix_type=MIN_FIX_TYPE,
        )

        print("\nStarting navigation (Ctrl+C to abort)...")

        # Navigate to each waypoint in order
        for i, target in enumerate(waypoints):
            print("\n" + "=" * 60)
            print(f"WAYPOINT {i+1}/{len(waypoints)}: {target.name}")
            print(f"Target: ({target.latitude:.8f}, {target.longitude:.8f})")
            print("=" * 60)

            # Calculate distance to this waypoint
            current_pos = gps.get_position()
            if current_pos:
                distance = haversine_distance(
                    current_pos.latitude, current_pos.longitude,
                    target.latitude, target.longitude
                )
                print(f"Distance to waypoint: {distance:.2f}m")

            print("-" * 60)
            success = await navigator.navigate_to(target)
            print("-" * 60)

            if not success:
                print(f"\nFailed to reach waypoint {i+1}: {target.name}")
                return False

            print(f"Reached waypoint {i+1}: {target.name}")

        # All waypoints reached
        print("\n" + "=" * 60)
        print("MISSION COMPLETE - All waypoints reached!")
        print("=" * 60)

        final_pos = gps.get_position()
        if final_pos:
            print(f"Final position: ({final_pos.latitude:.8f}, {final_pos.longitude:.8f})")

        return True

    except KeyboardInterrupt:
        print("\n\nMission interrupted by user")
        logger.info("Mission interrupted by KeyboardInterrupt")
        try:
            await robot.stop()
        except Exception:
            pass
        return False

    except Exception as e:
        logger.error(f"Mission error: {e}", exc_info=True)
        print(f"\nERROR: {e}")
        try:
            await robot.stop()
        except Exception:
            pass
        return False

    finally:
        # Cleanup
        print("\nCleaning up...")
        try:
            await robot.stop()
        except Exception:
            pass
        gps.disconnect()
        print("Connections closed")


def main():
    logfile = setup_logging()
    print(f"Logging to: {logfile}")

    try:
        success = asyncio.run(run_mission())
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(1)


if __name__ == "__main__":
    main()
