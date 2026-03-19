#!/usr/bin/env python3
"""
Test SparkFun F9R IMU Heading & Walk North

Debug script that uses the F9R's sensor-fused heading (headVeh) instead of
the Go2 robot IMU + GPS calibration walk. Includes diagnostic probes to
understand the F9R module's current state before attempting navigation.

Phases:
  0. Diagnostic Probes (GPS-only, no robot)
     A. Is F9R sensor fusion (HPS) active?
     B. Where are corrections coming from?
     C. Is headVeh valid?
  1. Setup — GPS + Robot connection
  2. SparkFun Heading Acquisition — wait for headVeh or walk to trigger it
  3. Turn to North — using F9R heading
  4. Walk North for 10s — logging both F9R and robot IMU headings

Environment Variables:
    ROBOT_IP         - Go2's IP address on local network (required for LocalSTA)
    GPS_PORT         - GPS serial port (default: /dev/ttyACM0)
    GPS_BAUD         - GPS baud rate (default: 38400)
    EMLID_USERNAME   - NTRIP username
    EMLID_PASSWORD   - NTRIP password
    EMLID_MOUNTPOINT - NTRIP mountpoint (default: MP15774)

Usage:
    python autonomous_nav/mission/debug/test_sparkfun_imu.py
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
    mountpoint=os.getenv("EMLID_MOUNTPOINT", "MP15774"),
    username=os.getenv("EMLID_USERNAME", ""),
    password=os.getenv("EMLID_PASSWORD", ""),
)

CONNECTION_MODE = os.getenv("CONNECTION_MODE", "LocalSTA")
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.1.105")
ROBOT_SERIAL = os.getenv("ROBOT_SERIAL", None)

CALIBRATION_WALK_SPEED = 0.3  # m/s during F9R heading acquisition walk
WALK_NORTH_SPEED = 0.5  # m/s during the northward walk
WALK_NORTH_DURATION = 10.0  # seconds
GPS_LOG_INTERVAL = 0.5  # seconds between GPS logs during walk
HEADING_TOLERANCE = 5.0  # degrees — close enough to north
TURN_RATE = 0.8  # rad/s rotation speed
TURN_TIMEOUT = 60.0  # seconds — abort rotation if not aligned
CALIBRATION_TIMEOUT = 90.0  # seconds — Phase 2 timeout for headVeh acquisition
MIN_FIX_TYPE = 4  # GNSS+DR or better
MAX_HACC = 1.0  # meters — max horizontal accuracy
GPS_FIX_TIMEOUT = 300  # seconds


# =============================================================================
# LOGGING SETUP
# =============================================================================


def setup_logging():
    log_dir = os.path.join(os.path.dirname(__file__), "logs")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    logfile = os.path.join(log_dir, f"test_sparkfun_imu_{timestamp}.log")

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
    print("Test SparkFun F9R IMU Heading & Walk North")
    print("=" * 60)

    gps = GPSManager(port=GPS_PORT, baudrate=GPS_BAUD, ntrip_config=NTRIP_CONFIG)
    robot = Go2Robot(
        connection_mode=CONNECTION_MODE,
        robot_ip=ROBOT_IP,
        robot_serial=ROBOT_SERIAL,
    )

    # Track whether headVeh is valid from Phase 0
    head_veh_valid_at_start = False

    try:
        # =================================================================
        # Phase 0: Diagnostic Probes (GPS-only)
        # =================================================================
        _log_banner("PHASE 0: DIAGNOSTIC PROBES")

        print("\n[0/4] Connecting to GPS for diagnostics...")
        if not gps.connect(use_ntrip=bool(NTRIP_CONFIG.username)):
            print("ERROR: Failed to connect to GPS")
            return False
        print("GPS connected")

        print(f"Waiting for GPS fix (min type {MIN_FIX_TYPE})...")
        if not gps.wait_for_fix(timeout=GPS_FIX_TIMEOUT, min_fix_type=MIN_FIX_TYPE):
            print("ERROR: GPS fix timeout")
            return False

        # --- Test A: Is F9R sensor fusion (HPS) active? ---
        _log_banner("TEST A: F9R SENSOR FUSION STATUS", char="-")
        esf = gps.gps.poll_esf_status()
        if esf:
            logger.info(f"Fusion mode: {esf['fusionModeLabel']} ({esf['fusionMode']})")
            logger.info(f"Number of sensors: {esf['numSensors']}")
            for s in esf["sensors"]:
                logger.info(
                    f"  {s['type']:12s} | used={s['used']} ready={s['ready']} "
                    f"calib={s['calibration']} time_status={s['time_status']}"
                )
            _log_banner(f"F9R SENSOR FUSION: {esf['fusionModeLabel']}")
        else:
            logger.warning("ESF-STATUS not available — module may not support sensor fusion")
            _log_banner("F9R SENSOR FUSION: NOT AVAILABLE", level="warning", char="!")

        # --- Test B: Where are corrections coming from? ---
        _log_banner("TEST B: CORRECTION SOURCE", char="-")
        pvt = gps.gps.poll_nav_pvt()
        if pvt:
            logger.info(
                f"carrSoln={pvt['carrSoln']} diffSoln={pvt['diffSoln']} "
                f"fixType={pvt['fixType']} numSV={pvt['numSV']} "
                f"hAcc={pvt['hAcc']:.3f}m"
            )
            if pvt["carrSoln"] > 0 and not NTRIP_CONFIG.username:
                logger.info("NOTE: carrSoln > 0 but NTRIP not configured — corrections from another source")
            _log_banner(
                f"CORRECTION SOURCE: carrSoln={pvt['carrSoln']} "
                f"diffSoln={pvt['diffSoln']} fixType={pvt['fixType']} "
                f"hAcc={pvt['hAcc']:.3f}m"
            )
        else:
            logger.warning("NAV-PVT poll failed")
            _log_banner("CORRECTION SOURCE: POLL FAILED", level="warning", char="!")

        # --- Test C: Is headVeh valid? ---
        _log_banner("TEST C: HEAD_VEH STATUS", char="-")
        pvt = gps.gps.poll_nav_pvt()
        if pvt:
            if pvt["headVehValid"]:
                head_veh_deg = pvt["headVeh"] * 1e-5
                head_acc_deg = pvt["headAcc"] * 1e-5
                logger.info(f"headVeh VALID: heading={head_veh_deg:.1f}° accuracy={head_acc_deg:.1f}°")
                head_veh_valid_at_start = True
                _log_banner(f"HEAD_VEH: VALID heading={head_veh_deg:.1f}° accuracy={head_acc_deg:.1f}°")
            else:
                head_veh_deg = pvt["headVeh"] * 1e-5
                head_acc_deg = pvt["headAcc"] * 1e-5
                logger.info(
                    f"headVeh NOT VALID — HPS hasn't initialized yet "
                    f"(raw heading={head_veh_deg:.1f}° accuracy={head_acc_deg:.1f}°)"
                )
                _log_banner("HEAD_VEH: INVALID — HPS not initialized", level="warning", char="!")
        else:
            logger.warning("NAV-PVT poll failed for headVeh check")
            _log_banner("HEAD_VEH: POLL FAILED", level="warning", char="!")

        # =================================================================
        # Phase 1: Setup — Robot Connection
        # =================================================================
        _log_banner("PHASE 1: SETUP")

        start_pos = gps.get_position()
        if start_pos:
            print(
                f"Start position: ({start_pos.latitude:.8f}, {start_pos.longitude:.8f}) "
                f"hAcc: {start_pos.accuracy_horizontal:.3f}m"
            )

        print("\n[1/4] Connecting to robot...")
        if not await robot.connect():
            print("ERROR: Failed to connect to robot")
            return False
        await robot.prepare_for_navigation()
        print("Robot ready")

        # Create navigator for robot IMU comparison logging
        navigator = WaypointNavigator(
            gps=gps,
            robot=robot,
            max_velocity=CALIBRATION_WALK_SPEED,
            min_fix_type=MIN_FIX_TYPE,
            max_hacc=MAX_HACC,
        )

        # =================================================================
        # Phase 2: SparkFun Heading Acquisition
        # =================================================================
        _log_banner("PHASE 2: F9R HEADING ACQUISITION")

        if head_veh_valid_at_start:
            pos = gps.get_position()
            if pos and pos.head_vehicle is not None:
                logger.info(
                    f"headVeh already valid from Phase 0 — skipping calibration walk. "
                    f"heading={pos.head_vehicle:.1f}° accuracy={pos.head_vehicle_accuracy:.1f}°"
                )
                _log_banner(
                    f"F9R HEADING ACQUIRED | heading={pos.head_vehicle:.1f}° "
                    f"accuracy={pos.head_vehicle_accuracy:.1f}°"
                )
            else:
                # Re-check failed, fall through to walk
                head_veh_valid_at_start = False

        if not head_veh_valid_at_start:
            print("\nWalking forward to trigger F9R auto-calibration...")
            cal_start = time.time()
            cal_last_log = 0.0
            acquired = False

            while not acquired:
                cal_elapsed = time.time() - cal_start
                if cal_elapsed > CALIBRATION_TIMEOUT:
                    logger.error(
                        f"Phase 2 timeout after {cal_elapsed:.0f}s — "
                        f"headVeh never became valid"
                    )
                    await robot.stop()
                    return False

                pos = gps.get_position()
                imu_yaw = robot.get_yaw_degrees()

                if pos is None:
                    await asyncio.sleep(0.2)
                    continue

                # Also run robot IMU calibration in parallel for comparison
                if imu_yaw is not None and navigator._imu_north_offset is None:
                    navigator._calibrate_imu(pos, imu_yaw)

                # Periodic logging
                now = time.time()
                if now - cal_last_log >= 2.0:
                    displacement = 0.0
                    if start_pos:
                        displacement = haversine_distance(
                            start_pos.latitude, start_pos.longitude,
                            pos.latitude, pos.longitude,
                        )
                    robot_hdg_str = "?"
                    robot_hdg = navigator.get_calibrated_heading()
                    if robot_hdg is not None:
                        robot_hdg_str = f"{robot_hdg:.1f}°"

                    f9r_str = "invalid"
                    if pos.head_vehicle is not None:
                        f9r_str = f"{pos.head_vehicle:.1f}°"

                    logger.info(
                        f"[cal walk] ({pos.latitude:.8f}, {pos.longitude:.8f}) "
                        f"fix={pos.fix_type} hAcc={pos.accuracy_horizontal:.3f}m "
                        f"disp={displacement:.2f}m | "
                        f"f9r_hdg={f9r_str} robot_hdg={robot_hdg_str}"
                    )
                    cal_last_log = now

                # Check if headVeh is now valid
                if pos.head_vehicle is not None:
                    acquired = True
                    await robot.stop()
                    logger.info(
                        f"F9R heading acquired after {cal_elapsed:.1f}s: "
                        f"heading={pos.head_vehicle:.1f}° "
                        f"accuracy={pos.head_vehicle_accuracy:.1f}°"
                    )
                    _log_banner(
                        f"F9R HEADING ACQUIRED | heading={pos.head_vehicle:.1f}° "
                        f"accuracy={pos.head_vehicle_accuracy:.1f}°"
                    )
                    break

                # Keep walking forward
                await robot.send_velocity(x=CALIBRATION_WALK_SPEED)
                await asyncio.sleep(0.2)

            await robot.stop()
            await asyncio.sleep(0.5)

        # Log comparison of both heading sources
        pos = gps.get_position()
        robot_hdg = navigator.get_calibrated_heading()
        if pos and pos.head_vehicle is not None:
            logger.info(
                f"Heading comparison — F9R: {pos.head_vehicle:.1f}° | "
                f"Robot IMU: {robot_hdg:.1f}°" if robot_hdg is not None
                else f"Heading comparison — F9R: {pos.head_vehicle:.1f}° | Robot IMU: not calibrated"
            )

        # =================================================================
        # Phase 3: Turn to Face North (using F9R heading)
        # =================================================================
        _log_banner("PHASE 3: TURN TO NORTH (F9R)")

        await robot.balance_stand()
        await asyncio.sleep(1.0)

        print("\nRotating to face north (0°) using F9R heading...")
        aligned = False
        turn_start = time.time()
        turn_last_log = 0.0
        while not aligned:
            if time.time() - turn_start > TURN_TIMEOUT:
                pos = gps.get_position()
                hdg_str = f"{pos.head_vehicle:.1f}°" if pos and pos.head_vehicle is not None else "?"
                logger.error(
                    f"Turn timeout after {TURN_TIMEOUT:.0f}s — "
                    f"F9R heading: {hdg_str}, aborting rotation"
                )
                await robot.stop()
                return False

            pos = gps.get_position()
            if pos is None or pos.head_vehicle is None:
                await asyncio.sleep(0.1)
                continue

            heading = pos.head_vehicle
            error = normalize_angle(0 - heading)

            # Periodic logging
            now = time.time()
            if now - turn_last_log >= 2.0:
                robot_hdg = navigator.get_calibrated_heading()
                robot_hdg_str = f"{robot_hdg:.1f}°" if robot_hdg is not None else "?"
                logger.info(
                    f"[turn] f9r_hdg={heading:.1f}° err={error:.1f}° "
                    f"robot_hdg={robot_hdg_str} | "
                    f"({pos.latitude:.8f}, {pos.longitude:.8f}) "
                    f"fix={pos.fix_type} hAcc={pos.accuracy_horizontal:.3f}m"
                )
                turn_last_log = now

            if abs(error) < HEADING_TOLERANCE:
                aligned = True
                await robot.stop()
                logger.info(f"Aligned to north — F9R heading: {heading:.1f}° (error: {error:.1f}°)")
                break

            direction = 1.0 if error > 0 else -1.0
            await robot.send_velocity(z=direction * TURN_RATE)
            await asyncio.sleep(0.1)

        await asyncio.sleep(0.5)
        pos = gps.get_position()
        if pos and pos.head_vehicle is not None:
            print(f"F9R heading after alignment: {pos.head_vehicle:.1f}°")

        # =================================================================
        # Phase 4: Walk North for 10s
        # =================================================================
        _log_banner("PHASE 4: WALK NORTH 10s @ 0.5 m/s (F9R)")

        await robot.balance_stand()
        await asyncio.sleep(1.0)

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
                    f9r_str = f"{pos.head_vehicle:.1f}°" if pos.head_vehicle is not None else "?"
                    robot_hdg = navigator.get_calibrated_heading()
                    robot_hdg_str = f"{robot_hdg:.1f}°" if robot_hdg is not None else "?"
                    logger.info(
                        f"[walk {elapsed:5.1f}s] ({pos.latitude:.8f}, {pos.longitude:.8f}) "
                        f"fix={pos.fix_type} hAcc={pos.accuracy_horizontal:.3f}m "
                        f"f9r_hdg={f9r_str} robot_hdg={robot_hdg_str}"
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
