"""Shared mission executor.

Loads mission.toml + waypoints.geojson from a mission directory, spins
up GPS + robot + navigator, iterates the waypoints, and calls any hooks
attached to the MissionRunner. Every mission runs through this function
— per-mission Python files should not re-implement setup/teardown.
"""

import asyncio
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Awaitable, Callable

from go2_survey import battery, health
from go2_survey.capture import (
    CaptureContext,
    LineSurveyStrategy,
    build_strategy,
    write_captures_manifest,
    write_interval_capture,
)
from go2_survey.bearings import finalize_bearings
from go2_survey.config import MissionSettings, load_mission_config
from go2_survey.discovery import find_robot_ips
from go2_survey.gps import GPSManager, RTKPosition
from go2_survey.logging_utils import log_banner, set_teardown_in_progress
from go2_survey.navigator import WaypointNavigator
from go2_survey.probes import run_f9r_probe, run_lidar_probe
from go2_survey.resume import ResumeAnchor
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
    # Set by `run --resume`: the anchor (last good mark) the line survey
    # continues from. `run_dir` then points at that partial run's directory so
    # the resumed captures merge into one complete survey. See :mod:`resume`.
    resume_anchor: ResumeAnchor | None = None


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

    if settings.mode == "probe_lidar":
        return await _run_probe_lidar(runner, settings)

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
        cooling_settings=settings.cooling,
    )

    health_task: asyncio.Task | None = None
    start_soc: int | None = None
    health_monitor = health.HealthMonitor(
        caution_c=settings.navigation.motor_caution_temp_c,
        danger_c=settings.navigation.motor_danger_temp_c,
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

        # Health: wait briefly for the first lowstate sample, banner the
        # starting battery + thigh temps (+ seed battery.log), then emit the
        # consolidated HEALTH line + thermal alerts every 10 s.
        start_bs = None
        for _ in range(20):  # ~4 s for the first lowstate frame
            start_bs = robot.get_battery_state()
            if start_bs is not None:
                break
            await asyncio.sleep(0.2)
        health_monitor.emit_start(robot)
        if start_bs is not None:
            battery.telemetry_logger.info(battery.format_telemetry(start_bs))
            start_soc = start_bs.soc
        health_task = asyncio.create_task(
            health.run_health_logger(robot, health_monitor)
        )

        if settings.mode == "cooling_test":
            log_banner("PHASE 4: COOLING TEST", char="-", logger=logger)
            success = await _run_cooling_test_route(
                settings=settings,
                waypoints=waypoints,
                robot=robot,
                navigator=navigator,
                health_monitor=health_monitor,
            )
            if not success:
                return False
            log_banner("MISSION COMPLETE", logger=logger)
            return True

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

        if settings.capture.strategy == LineSurveyStrategy.name:
            # Line survey runs the route end-to-end inside the navigator:
            # drive straight legs between corner waypoints, capture every
            # capture_interval_m of along-track travel via the callback.
            async def _interval_capture_cb(
                leg_label, mark_index, t_mark, along_m, position, leg_bearing
            ):
                ctx = CaptureContext(
                    mission_name=settings.name or mission_dir.name,
                    mission_dir=mission_dir,
                    run_dir=runner.run_dir,
                    robot=robot,
                    gps=gps,
                    navigator=navigator,
                    settings=settings.capture,
                    waypoint=None,
                    arrival_position=position,
                )
                return await write_interval_capture(
                    ctx,
                    leg_label=leg_label,
                    mark_index=mark_index,
                    target_time=t_mark,
                    along_track_m=along_m,
                    fallback_position=position,
                    target_bearing=leg_bearing,
                )

            # Resume: anchor the survey at the last good mark's position (a
            # synthetic start waypoint) followed by the remaining corners, and
            # continue the legNN/mMMM numbering into this (the partial's) run
            # dir so it ends up as one complete survey.
            route = waypoints
            resume_kwargs: dict = {}
            if runner.resume_anchor is not None:
                a = runner.resume_anchor
                anchor_wp = Waypoint(
                    latitude=a.latitude, longitude=a.longitude,
                    name=f"resume_leg{a.leg:02d}_m{a.mark:03d}",
                )
                route = [anchor_wp, *waypoints[a.leg:]]
                resume_kwargs = dict(
                    leg_number_start=a.leg,
                    first_leg_mark_offset=a.mark,
                    first_leg_skip_start_capture=True,
                    n_captured_start=a.n_captured,
                )
                log_banner(
                    f"RESUMING leg{a.leg:02d}/m{a.mark:03d} | {a.n_captured} "
                    f"prior frame(s) | {len(route) - 1} leg(s) remain -> "
                    f"approaching ({a.latitude:.8f}, {a.longitude:.8f})",
                    logger=logger,
                )

            try:
                success = await navigator.navigate_legs(
                    waypoints=route,
                    on_capture_cb=_interval_capture_cb,
                    interval_m=settings.capture.capture_interval_m,
                    speed=settings.navigation.max_velocity,
                    turn_tolerance_deg=settings.capture.turn_tolerance_deg,
                    leg_steering=settings.capture.leg_steering,
                    lookahead_m=settings.capture.lookahead_m,
                    course_lookback_m=settings.capture.course_lookback_m,
                    health=health_monitor,
                    **resume_kwargs,
                )
            finally:
                # Finalize EXIF bearings from each leg's full RTK track
                # (centered look-back+forward), then (re)build the manifest
                # from sidecars — both run even if the route aborted mid-run.
                captures_dir = runner.run_dir / settings.capture.output_subdir
                try:
                    finalize_bearings(
                        captures_dir, settings.capture.course_lookback_m
                    )
                except Exception as e:
                    logger.warning(f"Bearing finalization failed: {e}")
                try:
                    write_captures_manifest(captures_dir)
                except Exception as e:
                    logger.warning(f"Capture manifest build failed: {e}")
            if not success:
                logger.error("Line-survey route failed")
                return False
        else:
            for i, wp in enumerate(waypoints, start=1):
                log_banner(
                    f"WAYPOINT {i}/{len(waypoints)}: {wp.name}", logger=logger
                )
                reached = await navigator.navigate_to(wp)
                if not reached:
                    logger.error(f"Failed to reach waypoint {i}: {wp.name}")
                    return False

                health_monitor.emit_leg(robot, i, len(waypoints))

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
        # Stop the health logger and banner the ending battery + peak motor
        # temps while the WebRTC connection (and its lowstate stream) is alive.
        if health_task is not None:
            health_task.cancel()
            try:
                await health_task
            except asyncio.CancelledError:
                pass
            health_monitor.emit_end(robot, start_soc)
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


async def _run_cooling_test_route(
    *,
    settings: MissionSettings,
    waypoints: list[Waypoint],
    robot: Go2Robot,
    navigator: WaypointNavigator,
    health_monitor: health.HealthMonitor,
) -> bool:
    """Two-waypoint thermal-interlock exercise.

    Alternates between the tennis-court waypoint pair and performs the old
    quadrat-style cardinal turns without enabling video or writing frames. Once
    the cooling interlock has fired at least once, it visits the other waypoint
    one final time and then stays in crouched cooling mode until interrupted.
    """
    if len(waypoints) != 2:
        logger.error("cooling_test mode requires exactly two waypoints")
        return False
    if not settings.cooling.enabled:
        logger.warning(
            "cooling_test mode is running with [cooling] enabled=false; "
            "it will alternate forever unless cooling is enabled"
        )

    target_idx = 0
    visit_count = 0
    work_bearings = (0.0, 90.0, 180.0, 270.0)

    log_banner(
        "COOLING TEST | alternate two waypoints until one cooldown",
        char="-",
        logger=logger,
    )

    while navigator.cooldown_count < 1:
        before = navigator.cooldown_count
        wp = waypoints[target_idx]
        log_banner(
            f"COOLING TEST WAYPOINT {visit_count + 1}: {wp.name}",
            logger=logger,
        )
        if not await navigator.navigate_to(wp):
            logger.error(f"Cooling test failed to reach {wp.name}")
            return False

        visit_count += 1
        health_monitor.emit_leg(robot, visit_count, visit_count)
        if navigator.cooldown_count > before:
            break

        cooled_during_work = await _run_cooling_test_work(
            navigator=navigator,
            bearings=work_bearings,
            settle_time_s=settings.capture.settle_time,
            turn_tolerance_deg=settings.capture.turn_tolerance_deg,
        )
        if cooled_during_work is None:
            return False
        if cooled_during_work:
            break

        target_idx = 1 - target_idx

    final_idx = 1 - target_idx
    final_wp = waypoints[final_idx]
    log_banner(
        f"COOLING TEST FINAL WAYPOINT: {final_wp.name}",
        char="-",
        logger=logger,
    )
    if not await navigator.navigate_to(final_wp):
        logger.error(f"Cooling test failed to reach final waypoint {final_wp.name}")
        return False

    visit_count += 1
    health_monitor.emit_leg(robot, visit_count, visit_count)

    log_banner(
        "COOLING TEST FINAL HOLD | crouched cooldown until interrupted",
        level="warning",
        char="!",
        logger=logger,
    )
    c = settings.cooling
    await robot.stop_and_cool(
        resume_temp_c=c.resume_temp_c,
        poll_interval_s=c.poll_interval_s,
        log_interval_s=c.log_interval_s,
        telemetry_max_age_s=c.telemetry_max_age_s,
        lock_wait_s=c.lock_wait_s,
        crouch_wait_s=c.crouch_wait_s,
        stand_wait_s=c.stand_wait_s,
        stand_on_resume=False,
        hold_forever=True,
    )
    return True


async def _run_cooling_test_work(
    *,
    navigator: WaypointNavigator,
    bearings: tuple[float, ...],
    settle_time_s: float,
    turn_tolerance_deg: float,
) -> bool | None:
    """Run rotating-quadrat-style work without image capture.

    Returns True when cooldown fired during the work, False when the work
    completed without cooldown, and None on turn failure.
    """
    before = navigator.cooldown_count
    log_banner("COOLING TEST WORK | cardinal turns, no capture", char="-", logger=logger)
    for bearing in bearings:
        if not await navigator.turn_to_bearing(
            bearing,
            tolerance_deg=turn_tolerance_deg,
            timeout=20.0,
        ):
            logger.error(f"Cooling test turn to {bearing:.0f} deg failed")
            return None
        if navigator.cooldown_count > before:
            return True
        if settle_time_s > 0:
            await asyncio.sleep(settle_time_s)
    return navigator.cooldown_count > before


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
            out_dir=runner.run_dir,
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


async def _run_probe_lidar(runner: MissionRunner, settings: MissionSettings) -> bool:
    """Lidar diagnostic probe. Robot connection only — no GPS, no navigation.

    Verifies the documented ``rt/utlidar/voxel_map_compressed`` stream
    works end-to-end and emits a per-frame log + first-frame .npy +
    three quick-look raster PNGs into the run directory.
    """
    logger.info(
        f"Lidar probe: duration={settings.probe.duration_sec:.0f}s | "
        f"Robot: {settings.robot.connection_mode}"
        + (f" ip={settings.robot.ip}" if settings.robot.ip else "")
    )

    if runner.dry_run:
        log_banner("DRY RUN — not connecting to hardware", logger=logger)
        return True

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

    try:
        log_banner("PHASE 1: ROBOT", char="-", logger=logger)
        if not await robot.connect():
            logger.error("Failed to connect to robot")
            return False

        log_banner(
            f"PHASE 2: LIDAR PROBE ({settings.probe.duration_sec:.0f}s)",
            char="-",
            logger=logger,
        )
        await run_lidar_probe(
            robot=robot,
            out_dir=runner.run_dir,
            duration_sec=settings.probe.duration_sec,
        )
        log_banner("LIDAR PROBE COMPLETE", logger=logger)
        return True

    except KeyboardInterrupt:
        logger.warning("Lidar probe interrupted by user")
        return False
    except Exception as e:
        logger.error(f"Lidar probe error: {e}", exc_info=True)
        return False
    finally:
        set_teardown_in_progress(True)
        try:
            await robot.close()
        except Exception:
            logger.debug("robot.close raised during teardown", exc_info=True)
        logger.info("Robot disconnected")
