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

from go2_survey.config import load_mission_config
from go2_survey.gps import GPSManager, RTKPosition
from go2_survey.logging_utils import log_banner
from go2_survey.navigator import WaypointNavigator
from go2_survey.ntrip import NTRIPConfig
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
    robot = Go2Robot(
        connection_mode=settings.robot.connection_mode,
        robot_ip=settings.robot.ip,
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

            if runner.on_waypoint_reached is not None:
                pos = gps.get_position()
                if pos is not None:
                    await runner.on_waypoint_reached(wp, pos)

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
            await robot.stop()
        except Exception:
            pass
        try:
            gps.disconnect()
        except Exception:
            pass
        logger.info("Connections closed")
