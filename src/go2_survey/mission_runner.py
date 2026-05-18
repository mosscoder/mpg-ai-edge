"""Shared mission executor.

Loads mission.toml + waypoints.geojson from a mission directory, spins
up GPS + robot + navigator, iterates the waypoints, and calls any hooks
attached to the MissionRunner. Every mission runs through this function
— per-mission Python files should not re-implement setup/teardown.
"""

import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Awaitable, Callable

from go2_survey.capture import CaptureContext, build_strategy
from go2_survey.config import MissionSettings, load_mission_config
from go2_survey.discovery import find_robot_ips
from go2_survey.gps import GPSManager, RTKPosition
from go2_survey.logging_utils import log_banner, set_teardown_in_progress
from go2_survey.navigator import WaypointNavigator
from go2_survey.probes import run_f9r_probe
from go2_survey.robot import Go2Robot
from go2_survey.waypoints import Waypoint, load_waypoints

logger = logging.getLogger(__name__)

# Discovery retry budget. Robot can take a moment to associate with
# the Pixel hotspot after power-on; the 2026-05-14 session had two
# discovery failures before a third call ten minutes later succeeded.
# 4 attempts * 10s sleep ≈ 30s of patience before declaring "no Go2".
ROBOT_DISCOVERY_ATTEMPTS = 4
ROBOT_DISCOVERY_DELAY_S = 10.0


WaypointHook = Callable[[Waypoint, RTKPosition], Awaitable[None]]
PositionHook = Callable[[RTKPosition], Awaitable[None]]


def _discover_robot_ip_with_retry() -> str | None:
    """Auto-discover the Go2 on the local network, retrying on empty result.

    A transient empty result (no host on ports 8081/9991) usually means
    the robot is still booting or its wifi hasn't associated yet.
    Re-scanning after a short wait typically recovers without operator
    intervention. A `RuntimeError` from the discovery layer (e.g. no
    `ip route` output to figure out the subnet) is a config problem,
    not transient — fails fast.

    Returns the first IP found, or None if every attempt was empty.
    Caller is responsible for emitting the surrounding phase banner and
    the final error log on `None`.
    """
    for attempt in range(1, ROBOT_DISCOVERY_ATTEMPTS + 1):
        logger.info(
            f"Discovery attempt {attempt}/{ROBOT_DISCOVERY_ATTEMPTS}: "
            f"scanning for Go2 on ports 8081/9991..."
        )
        ips = find_robot_ips()
        if ips:
            chosen = ips[0]
            if len(ips) > 1:
                logger.warning(
                    f"Discovery attempt {attempt}: multiple candidates "
                    f"({ips}); using {chosen}"
                )
            else:
                logger.info(
                    f"Discovery attempt {attempt}: found Go2 at {chosen}"
                )
            return chosen
        if attempt < ROBOT_DISCOVERY_ATTEMPTS:
            logger.warning(
                f"Discovery attempt {attempt} found no Go2; "
                f"waiting {ROBOT_DISCOVERY_DELAY_S:.0f}s before retry "
                f"(robot may still be booting / associating with hotspot)..."
            )
            time.sleep(ROBOT_DISCOVERY_DELAY_S)
        else:
            logger.warning(
                f"Discovery attempt {attempt} found no Go2 — retry budget exhausted"
            )
    return None


@dataclass
class MissionRunner:
    """Everything a mission invocation needs.

    Extend with additional fields (e.g. a `capture_images` flag that
    turns into a `_default_frame_hook`) as features land.
    """

    mission_dir: Path
    run_dir: Path
    dry_run: bool = False
    on_waypoint_reached: WaypointHook | None = None
    on_gps_update: PositionHook | None = None


async def run_mission(runner: MissionRunner) -> bool:
    """Execute a mission end-to-end. Returns True on success."""
    mission_dir = runner.mission_dir

    settings = load_mission_config(mission_dir)
    log_banner(f"MISSION: {settings.name or mission_dir.name}", logger=logger)
    if settings.description:
        logger.info(settings.description)
    log_banner(f"MODE: {settings.mode}", char="-", logger=logger)

    if settings.mode == "probe_gps":
        return await _run_probe_gps(runner, settings)

    if settings.mode in ("static_camera", "static_geotag"):
        return await _run_static(runner, settings)

    waypoints_path = mission_dir / "waypoints.geojson"
    if not waypoints_path.exists():
        logger.error(f"waypoints.geojson not found in {mission_dir}")
        return False
    waypoints = load_waypoints(waypoints_path)
    if not waypoints:
        logger.error("No waypoints loaded; aborting")
        return False

    logger.info(f"Loaded {len(waypoints)} waypoint(s):")
    for i, wp in enumerate(waypoints, start=1):
        logger.info(
            f"  {i}. {wp.name}: ({wp.latitude:.8f}, {wp.longitude:.8f})"
        )

    logger.info(
        f"GPS: {settings.gps.port} @ {settings.gps.baud} | "
        f"NTRIP: {settings.ntrip.host}:{settings.ntrip.port} "
        f"mountpoints={settings.ntrip.mountpoints} | "
        f"Robot: {settings.robot.connection_mode}"
        + (f" ip={settings.robot.ip}" if settings.robot.ip else "")
    )
    logger.info(
        f"Nav: tol={settings.navigation.arrival_tolerance}m "
        f"vmax={settings.navigation.max_velocity}m/s "
        f"rot={settings.navigation.rotation_rate}rad/s "
        f"minFix={settings.navigation.min_fix_type} "
        f"maxHacc={settings.navigation.max_hacc}m"
    )

    if runner.dry_run:
        log_banner("DRY RUN — not connecting to hardware", logger=logger)
        return True

    gps = GPSManager(
        port=settings.gps.port,
        baudrate=settings.gps.baud,
        ntrip_settings=settings.ntrip,
        max_rtcm_age_s=settings.navigation.max_rtcm_age_s,
    )

    robot_ip = settings.robot.ip
    if (
        robot_ip is None
        and settings.robot.serial is None
        and settings.robot.connection_mode == "LocalSTA"
    ):
        log_banner(
            f"AUTO-DISCOVERING ROBOT IP "
            f"(up to {ROBOT_DISCOVERY_ATTEMPTS} attempts, "
            f"{ROBOT_DISCOVERY_DELAY_S:.0f}s between)",
            char="-",
            logger=logger,
        )
        logger.info(
            "robot.ip not set in mission.toml and ROBOT_IP env var not set; "
            "scanning local network for a Go2 (nmap ports 8081/9991)..."
        )
        try:
            robot_ip = _discover_robot_ip_with_retry()
        except RuntimeError as e:
            logger.error(f"Robot IP auto-discovery failed: {e}")
            logger.error(
                "Set [robot] ip in mission.toml, set ROBOT_IP in the "
                "environment, or run `go2-survey discover-ip` to diagnose."
            )
            return False
        if robot_ip is None:
            logger.error(
                f"Robot IP auto-discovery found no Go2 after "
                f"{ROBOT_DISCOVERY_ATTEMPTS} attempts. "
                f"Set [robot] ip in mission.toml, set ROBOT_IP in the "
                f"environment, or run `go2-survey discover-ip` to diagnose."
            )
            return False
        log_banner(
            f"ROBOT IP DISCOVERED | {robot_ip}",
            char="-",
            logger=logger,
        )

    robot = Go2Robot(
        connection_mode=settings.robot.connection_mode,
        robot_ip=robot_ip,
        robot_serial=settings.robot.serial,
    )
    navigator = WaypointNavigator(
        gps=gps,
        robot=robot,
        max_velocity=settings.navigation.max_velocity,
        rotation_rate=settings.navigation.rotation_rate,
        arrival_tolerance=settings.navigation.arrival_tolerance,
        min_fix_type=settings.navigation.min_fix_type,
        max_hacc=settings.navigation.max_hacc,
        gps_timeout=settings.navigation.mid_mission_fix_timeout,
        imu_recalibrate_on_arrival=settings.navigation.imu_recalibrate_on_arrival,
    )

    try:
        log_banner("PHASE 1: GPS", char="-", logger=logger)
        if not gps.connect(use_ntrip=bool(settings.ntrip.mountpoints)):
            logger.error("Failed to connect to GPS")
            return False

        if not gps.wait_for_fix(
            timeout=settings.navigation.gps_fix_timeout,
            min_fix_type=settings.navigation.min_fix_type,
        ):
            logger.error("GPS fix timeout")
            return False

        initial_pos = gps.get_position()
        if initial_pos:
            logger.info(
                f"Start position: ({initial_pos.latitude:.8f}, "
                f"{initial_pos.longitude:.8f}) "
                f"hAcc: {initial_pos.accuracy_horizontal:.3f}m"
            )

        log_banner(
            f"PHASE 2: GPS STABILIZATION ({settings.navigation.stabilization_period_s}s)",
            char="-",
            logger=logger,
        )
        gps.stabilization_dwell(
            settings.navigation.stabilization_period_s,
            early_exit_s=settings.navigation.stabilization_early_exit_s,
        )

        log_banner("PHASE 3: ROBOT", char="-", logger=logger)
        if not await robot.connect():
            logger.error("Failed to connect to robot")
            return False
        await robot.prepare_for_navigation()

        capture_strategy = build_strategy(settings.capture)
        if settings.capture.strategy != "none":
            log_banner(
                f"CAPTURE STRATEGY: {settings.capture.strategy}",
                char="-",
                logger=logger,
            )
            await robot.enable_video()

        log_banner(
            f"PHASE 4: NAVIGATE {len(waypoints)} WAYPOINT(S)",
            char="-",
            logger=logger,
        )
        for i, wp in enumerate(waypoints, start=1):
            log_banner(
                f"WAYPOINT {i}/{len(waypoints)}: {wp.name}", logger=logger
            )
            reached = await navigator.navigate_to(wp)
            if not reached:
                logger.error(f"Failed to reach waypoint {i}: {wp.name}")
                return False

            arrival_pos = gps.get_position()

            if settings.capture.strategy != "none":
                ctx = CaptureContext(
                    mission_name=settings.name or mission_dir.name,
                    mission_dir=mission_dir,
                    run_dir=runner.run_dir,
                    robot=robot,
                    gps=gps,
                    navigator=navigator,
                    settings=settings.capture,
                    waypoint=wp,
                    arrival_position=arrival_pos,
                )
                await capture_strategy.execute(ctx)

            if runner.on_waypoint_reached is not None and arrival_pos is not None:
                await runner.on_waypoint_reached(wp, arrival_pos)

        log_banner("MISSION COMPLETE", logger=logger)
        final = gps.get_position()
        if final:
            logger.info(
                f"Final position: ({final.latitude:.8f}, {final.longitude:.8f})"
            )
        return True

    except KeyboardInterrupt:
        logger.warning("Mission interrupted by user")
        return False
    except Exception as e:
        logger.error(f"Mission error: {e}", exc_info=True)
        return False
    finally:
        # Suppress library-side stream-end noise from the WebRTC
        # teardown that follows. Mid-mission instances of the same
        # patterns still surface — the filter only activates from here.
        set_teardown_in_progress(True)
        try:
            await robot.disable_video()
        except Exception:
            logger.debug("disable_video raised during teardown", exc_info=True)
        try:
            await robot.stop()
        except Exception:
            logger.debug("robot.stop raised during teardown", exc_info=True)
        try:
            await robot.close()
        except Exception:
            logger.debug("robot.close raised during teardown", exc_info=True)
        try:
            gps.disconnect()
        except Exception:
            logger.debug("gps.disconnect raised during teardown", exc_info=True)
        logger.info("Connections closed")


async def _run_static(runner: MissionRunner, settings: MissionSettings) -> bool:
    """Static capture modes: robot in place, no navigation.

    - `static_camera` — robot only, video only, single capture. No GPS,
      no RTK, no waypoints. Lab-bench plumbing test.
    - `static_geotag` — GPS + robot + video + sidecar. Robot stands
      (or pivots, per strategy) at one location; RTK fix required.
    """
    mission_dir = runner.mission_dir
    mode = settings.mode

    logger.info(
        f"Static mode: {mode} | capture strategy: {settings.capture.strategy}"
    )

    if runner.dry_run:
        log_banner("DRY RUN — not connecting to hardware", logger=logger)
        return True

    use_gps = mode == "static_geotag"
    gps: GPSManager | None = None
    robot: Go2Robot | None = None

    try:
        if use_gps:
            gps = GPSManager(
                port=settings.gps.port,
                baudrate=settings.gps.baud,
                ntrip_settings=settings.ntrip,
            )
            log_banner("PHASE 1: GPS", char="-", logger=logger)
            if not gps.connect(use_ntrip=bool(settings.ntrip.mountpoints)):
                logger.error("Failed to connect to GPS")
                return False
            if not gps.wait_for_fix(
                timeout=settings.navigation.gps_fix_timeout,
                min_fix_type=settings.navigation.min_fix_type,
            ):
                logger.error("GPS fix timeout")
                return False
            log_banner(
                f"PHASE 2: GPS STABILIZATION ({settings.navigation.stabilization_period_s}s)",
                char="-",
                logger=logger,
            )
            gps.stabilization_dwell(
                settings.navigation.stabilization_period_s,
                early_exit_s=settings.navigation.stabilization_early_exit_s,
            )

        robot_ip = settings.robot.ip
        if (
            robot_ip is None
            and settings.robot.serial is None
            and settings.robot.connection_mode == "LocalSTA"
        ):
            log_banner(
                f"AUTO-DISCOVERING ROBOT IP "
                f"(up to {ROBOT_DISCOVERY_ATTEMPTS} attempts, "
                f"{ROBOT_DISCOVERY_DELAY_S:.0f}s between)",
                char="-",
                logger=logger,
            )
            try:
                robot_ip = _discover_robot_ip_with_retry()
            except RuntimeError as e:
                logger.error(f"Robot IP auto-discovery failed: {e}")
                return False
            if robot_ip is None:
                logger.error(
                    f"Robot IP auto-discovery found no Go2 after "
                    f"{ROBOT_DISCOVERY_ATTEMPTS} attempts"
                )
                return False
            log_banner(
                f"ROBOT IP DISCOVERED | {robot_ip}",
                char="-",
                logger=logger,
            )

        robot = Go2Robot(
            connection_mode=settings.robot.connection_mode,
            robot_ip=robot_ip,
            robot_serial=settings.robot.serial,
        )
        # Phase numbering: if GPS was used we inserted a stabilization phase,
        # so ROBOT is PHASE 3 and CAPTURE is PHASE 4. Otherwise (no GPS) keep
        # the legacy PHASE 2 / PHASE 3 numbering.
        robot_phase = "PHASE 3" if use_gps else "PHASE 2"
        capture_phase = "PHASE 4" if use_gps else "PHASE 3"
        log_banner(f"{robot_phase}: ROBOT", char="-", logger=logger)
        if not await robot.connect():
            logger.error("Failed to connect to robot")
            return False

        await robot.enable_video()

        capture_strategy = build_strategy(settings.capture)
        log_banner(
            f"{capture_phase}: STATIC CAPTURE ({settings.capture.strategy})",
            char="-",
            logger=logger,
        )

        arrival_pos = gps.get_position() if gps is not None else None
        ctx = CaptureContext(
            mission_name=settings.name or mission_dir.name,
            mission_dir=mission_dir,
            run_dir=runner.run_dir,
            robot=robot,
            gps=gps,
            navigator=None,  # no rotation support for static
            settings=settings.capture,
            waypoint=None,
            arrival_position=arrival_pos,
        )
        n_frames = await capture_strategy.execute(ctx)
        log_banner(
            f"STATIC CAPTURE DONE | {n_frames} frames", logger=logger
        )
        return True

    except KeyboardInterrupt:
        logger.warning("Static capture interrupted by user")
        return False
    except Exception as e:
        logger.error(f"Static capture error: {e}", exc_info=True)
        return False
    finally:
        set_teardown_in_progress(True)
        if robot is not None:
            try:
                await robot.disable_video()
            except Exception:
                logger.debug("disable_video raised during teardown", exc_info=True)
            try:
                await robot.stop()
            except Exception:
                logger.debug("robot.stop raised during teardown", exc_info=True)
            try:
                await robot.close()
            except Exception:
                logger.debug("robot.close raised during teardown", exc_info=True)
        if gps is not None:
            try:
                gps.disconnect()
            except Exception:
                logger.debug("gps.disconnect raised during teardown", exc_info=True)
        logger.info("Connections closed")


async def _run_probe_gps(runner: MissionRunner, settings: MissionSettings) -> bool:
    """GPS-only diagnostic probe. No robot, no navigation, no waypoints."""
    mission_dir = runner.mission_dir

    logger.info(
        f"GPS: {settings.gps.port} @ {settings.gps.baud} | "
        f"NTRIP: {settings.ntrip.host}:{settings.ntrip.port} "
        f"mountpoints={settings.ntrip.mountpoints}"
    )
    logger.info(
        f"Probe: duration={settings.probe.duration_sec:.0f}s "
        f"rate={settings.probe.sample_rate_hz:.1f}Hz "
        f"wait_for_fix={settings.probe.wait_for_fix}"
    )

    if runner.dry_run:
        log_banner("DRY RUN — not connecting to hardware", logger=logger)
        return True

    gps = GPSManager(
        port=settings.gps.port,
        baudrate=settings.gps.baud,
        ntrip_settings=settings.ntrip,
        max_rtcm_age_s=settings.navigation.max_rtcm_age_s,
    )

    try:
        log_banner("PHASE 1: GPS", char="-", logger=logger)
        if not gps.connect(use_ntrip=bool(settings.ntrip.mountpoints)):
            logger.error("Failed to connect to GPS")
            return False

        if settings.probe.wait_for_fix:
            if not gps.wait_for_fix(
                timeout=settings.navigation.gps_fix_timeout,
                min_fix_type=settings.navigation.min_fix_type,
            ):
                logger.warning(
                    "Did not reach desired fix before timeout; "
                    "probing anyway to capture degraded-state diagnostics"
                )

        await run_f9r_probe(
            gps=gps,
            out_dir=mission_dir,
            duration_sec=settings.probe.duration_sec,
            sample_rate_hz=settings.probe.sample_rate_hz,
        )
        log_banner("PROBE COMPLETE", logger=logger)
        return True

    except KeyboardInterrupt:
        logger.warning("Probe interrupted by user")
        return False
    except Exception as e:
        logger.error(f"Probe error: {e}", exc_info=True)
        return False
    finally:
        try:
            gps.disconnect()
        except Exception:
            logger.debug("gps.disconnect raised during teardown", exc_info=True)
        logger.info("GPS disconnected")
