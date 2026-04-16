"""Shared mission executor.

Loads mission.toml + waypoints.geojson from a mission directory, spins
up GPS + robot + navigator, iterates the waypoints, and calls any hooks
attached to the MissionRunner. Every mission runs through this function
— per-mission Python files should not re-implement setup/teardown.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Awaitable, Callable, Optional

from go2_survey.capture import CaptureContext, build_strategy
from go2_survey.config import MissionSettings, load_mission_config
from go2_survey.discovery import find_robot_ips
from go2_survey.gps import GPSManager, RTKPosition
from go2_survey.logging_utils import log_banner
from go2_survey.navigator import WaypointNavigator
from go2_survey.ntrip import NTRIPConfig
from go2_survey.probes import run_f9r_probe
from go2_survey.robot import Go2Robot
from go2_survey.waypoints import Waypoint, load_waypoints

logger = logging.getLogger(__name__)


WaypointHook = Callable[[Waypoint, RTKPosition], Awaitable[None]]
PositionHook = Callable[[RTKPosition], Awaitable[None]]


@dataclass
class MissionRunner:
    """Everything a mission invocation needs.

    Extend with additional fields (e.g. a `capture_images` flag that
    turns into a `_default_frame_hook`) as features land.
    """

    mission_dir: Path
    dry_run: bool = False
    on_waypoint_reached: Optional[WaypointHook] = None
    on_gps_update: Optional[PositionHook] = None


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
        f"NTRIP: {settings.ntrip.host}:{settings.ntrip.port}/{settings.ntrip.mountpoint} | "
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

    ntrip_cfg = NTRIPConfig(
        host=settings.ntrip.host,
        port=settings.ntrip.port,
        mountpoint=settings.ntrip.mountpoint,
        username=settings.ntrip.username,
        password=settings.ntrip.password,
    )
    gps = GPSManager(
        port=settings.gps.port,
        baudrate=settings.gps.baud,
        ntrip_config=ntrip_cfg,
    )

    robot_ip = settings.robot.ip
    if (
        robot_ip is None
        and settings.robot.serial is None
        and settings.robot.connection_mode == "LocalSTA"
    ):
        log_banner("AUTO-DISCOVERING ROBOT IP", char="-", logger=logger)
        logger.info(
            "robot.ip not set in mission.toml and ROBOT_IP env var not set; "
            "scanning local network for a Go2 (nmap ports 8081/9991)..."
        )
        try:
            ips = find_robot_ips()
        except RuntimeError as e:
            logger.error(f"Robot IP auto-discovery failed: {e}")
            logger.error(
                "Set [robot] ip in mission.toml, set ROBOT_IP in the "
                "environment, or run `go2-survey discover-ip` to diagnose."
            )
            return False
        if not ips:
            logger.error(
                "Robot IP auto-discovery found no Go2 on the local network. "
                "Set [robot] ip in mission.toml, set ROBOT_IP in the "
                "environment, or run `go2-survey discover-ip` to diagnose."
            )
            return False
        robot_ip = ips[0]
        if len(ips) > 1:
            logger.warning(
                f"Multiple candidates found; using {robot_ip} (others: {ips[1:]})"
            )
        else:
            logger.info(f"Auto-discovered Go2 at {robot_ip}")

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
    )

    try:
        log_banner("PHASE 1: GPS", char="-", logger=logger)
        if not gps.connect(use_ntrip=bool(settings.ntrip.username)):
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

        log_banner("PHASE 2: ROBOT", char="-", logger=logger)
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
            f"PHASE 3: NAVIGATE {len(waypoints)} WAYPOINT(S)",
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
        try:
            await robot.disable_video()
        except Exception:
            pass
        try:
            await robot.stop()
        except Exception:
            pass
        try:
            gps.disconnect()
        except Exception:
            pass
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
    gps: Optional[GPSManager] = None
    robot: Optional[Go2Robot] = None

    try:
        if use_gps:
            ntrip_cfg = NTRIPConfig(
                host=settings.ntrip.host,
                port=settings.ntrip.port,
                mountpoint=settings.ntrip.mountpoint,
                username=settings.ntrip.username,
                password=settings.ntrip.password,
            )
            gps = GPSManager(
                port=settings.gps.port,
                baudrate=settings.gps.baud,
                ntrip_config=ntrip_cfg,
            )
            log_banner("PHASE 1: GPS", char="-", logger=logger)
            if not gps.connect(use_ntrip=bool(settings.ntrip.username)):
                logger.error("Failed to connect to GPS")
                return False
            if not gps.wait_for_fix(
                timeout=settings.navigation.gps_fix_timeout,
                min_fix_type=settings.navigation.min_fix_type,
            ):
                logger.error("GPS fix timeout")
                return False

        robot_ip = settings.robot.ip
        if (
            robot_ip is None
            and settings.robot.serial is None
            and settings.robot.connection_mode == "LocalSTA"
        ):
            log_banner("AUTO-DISCOVERING ROBOT IP", char="-", logger=logger)
            try:
                ips = find_robot_ips()
            except RuntimeError as e:
                logger.error(f"Robot IP auto-discovery failed: {e}")
                return False
            if not ips:
                logger.error("Robot IP auto-discovery found no Go2")
                return False
            robot_ip = ips[0]
            logger.info(f"Auto-discovered Go2 at {robot_ip}")

        robot = Go2Robot(
            connection_mode=settings.robot.connection_mode,
            robot_ip=robot_ip,
            robot_serial=settings.robot.serial,
        )
        log_banner("PHASE 2: ROBOT", char="-", logger=logger)
        if not await robot.connect():
            logger.error("Failed to connect to robot")
            return False

        await robot.enable_video()

        capture_strategy = build_strategy(settings.capture)
        log_banner(
            f"PHASE 3: STATIC CAPTURE ({settings.capture.strategy})",
            char="-",
            logger=logger,
        )

        arrival_pos = gps.get_position() if gps is not None else None
        ctx = CaptureContext(
            mission_name=settings.name or mission_dir.name,
            mission_dir=mission_dir,
            robot=robot,
            gps=gps,  # type: ignore[arg-type]
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
        if robot is not None:
            try:
                await robot.disable_video()
            except Exception:
                pass
        if gps is not None:
            try:
                gps.disconnect()
            except Exception:
                pass
        logger.info("Connections closed")


async def _run_probe_gps(runner: MissionRunner, settings: MissionSettings) -> bool:
    """GPS-only diagnostic probe. No robot, no navigation, no waypoints."""
    mission_dir = runner.mission_dir

    logger.info(
        f"GPS: {settings.gps.port} @ {settings.gps.baud} | "
        f"NTRIP: {settings.ntrip.host}:{settings.ntrip.port}/{settings.ntrip.mountpoint}"
    )
    logger.info(
        f"Probe: duration={settings.probe.duration_sec:.0f}s "
        f"rate={settings.probe.sample_rate_hz:.1f}Hz "
        f"wait_for_fix={settings.probe.wait_for_fix}"
    )

    if runner.dry_run:
        log_banner("DRY RUN — not connecting to hardware", logger=logger)
        return True

    ntrip_cfg = NTRIPConfig(
        host=settings.ntrip.host,
        port=settings.ntrip.port,
        mountpoint=settings.ntrip.mountpoint,
        username=settings.ntrip.username,
        password=settings.ntrip.password,
    )
    gps = GPSManager(
        port=settings.gps.port,
        baudrate=settings.gps.baud,
        ntrip_config=ntrip_cfg,
    )

    try:
        log_banner("PHASE 1: GPS", char="-", logger=logger)
        if not gps.connect(use_ntrip=bool(settings.ntrip.username)):
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
            pass
        logger.info("GPS disconnected")
