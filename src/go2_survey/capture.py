"""Capture strategies — per-waypoint image capture sequences.

A capture strategy is an async object with `execute(context) -> int`
that returns the number of frames written. Invoked by the mission
runner at each waypoint after the navigator reports arrival.

The strategy owns the sequence of actions at a single capture site:
settle, average GPS, optionally rotate, capture, write. Bearing
decisions use the robot's body IMU with the navigator's calibrated
offset — F9R `headVeh` is recorded in sidecar metadata only.

Strategy registry lives at the bottom of this module. To add a new
strategy, define a subclass of `CaptureStrategy`, implement `execute`,
and register it in `STRATEGIES`. Missions select a strategy by name
via `[capture] strategy = "..."` in mission.toml.
"""

import asyncio
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from go2_survey.gps import GPSManager, RTKPosition
from go2_survey.logging_utils import log_banner
from go2_survey.robot import Go2Robot
from go2_survey.vision.frames import capture_frame
from go2_survey.vision.geotag import write_frame_only_jpeg, write_geotagged_jpeg

if TYPE_CHECKING:
    from go2_survey.config import CaptureSettings
    from go2_survey.navigator import WaypointNavigator
    from go2_survey.waypoints import Waypoint

logger = logging.getLogger(__name__)


@dataclass
class CaptureContext:
    """Everything a strategy needs to do its job at a waypoint."""

    mission_name: str
    mission_dir: Path
    run_dir: Path  # per-run directory under <mission>/runs/; captures land in run_dir/<output_subdir>/<wp>/
    robot: Go2Robot
    gps: GPSManager | None  # None for static_camera mode
    navigator: "WaypointNavigator | None"  # None for static modes
    settings: "CaptureSettings"
    waypoint: "Waypoint | None"  # None for static modes
    arrival_position: RTKPosition | None


class CaptureStrategy:
    """Base class. Override `execute`."""

    name: str = "base"

    async def execute(self, ctx: CaptureContext) -> int:
        raise NotImplementedError


class NoOpStrategy(CaptureStrategy):
    """Default strategy — does nothing. Preserves pre-capture behavior
    for missions that don't configure a `[capture]` section."""

    name = "none"

    async def execute(self, ctx: CaptureContext) -> int:
        return 0


class FrameOnlyStrategy(CaptureStrategy):
    """Settle → capture one frame → write JPEG + minimal sidecar.

    No GPS, no bearing, no rotation. For lab-bench smoke tests
    (`mode = "static_camera"`) where the WebRTC video pipeline can
    be exercised without sky view, RTK, or a navigator. Sidecar
    carries `position: null` and `heading.source: "none"`.
    """

    name = "frame_only"

    async def execute(self, ctx: CaptureContext) -> int:
        s = ctx.settings
        wp_name = ctx.waypoint.name if ctx.waypoint else "static"

        log_banner(f"CAPTURE @ {wp_name} | {self.name}", char="-", logger=logger)

        logger.info(f"Settling for {s.settle_time:.1f}s")
        await asyncio.sleep(s.settle_time)

        frame_result = await capture_frame(
            ctx.robot,
            max_age=s.frame_max_age,
            wait_timeout=s.frame_wait_timeout,
            prefer_clean=s.prefer_clean_frame,
        )
        if frame_result is None:
            logger.error(f"No fresh frame at {wp_name}; skipping")
            return 0

        out_path = _capture_output_path(ctx, wp_name, bearing=None)
        write_frame_only_jpeg(
            frame=frame_result,
            out_path=out_path,
            mission_context=_mission_context(ctx, wp_name, self.name, bearing=None),
        )
        log_banner(
            f"CAPTURE DONE @ {wp_name} | 1 frame (no geotag)",
            char="-",
            logger=logger,
        )
        return 1


class WaypointForwardStrategy(CaptureStrategy):
    """Settle → average GPS → capture one forward-facing frame.

    No rotation. The bearing written into EXIF + sidecar is whatever
    the robot is currently pointing (from body IMU + navigator
    calibration). Simplest possible capture — good first field test
    of the nav+capture composition.
    """

    name = "waypoint_forward"

    async def execute(self, ctx: CaptureContext) -> int:
        s = ctx.settings
        wp_name = ctx.waypoint.name if ctx.waypoint else "static"

        log_banner(f"CAPTURE @ {wp_name} | {self.name}", char="-", logger=logger)

        # Settle after stop so IMU stabilizes and gait oscillation dies.
        logger.info(f"Settling for {s.settle_time:.1f}s")
        await asyncio.sleep(s.settle_time)

        position = await _sample_position(ctx)
        if position is None:
            logger.error(f"No GPS at capture point {wp_name}; skipping")
            return 0

        # waypoint_forward has no bearing TARGET — camera faces whatever
        # direction the robot ended up in. The achieved heading from the
        # IMU is what the photo shows; bearing stays None.
        achieved_heading, heading_source = _current_bearing(ctx)

        frame_result = await capture_frame(
            ctx.robot,
            max_age=s.frame_max_age,
            wait_timeout=s.frame_wait_timeout,
            prefer_clean=s.prefer_clean_frame,
        )
        if frame_result is None:
            logger.error(f"No fresh frame at {wp_name}; skipping")
            return 0

        out_path = _capture_output_path(ctx, wp_name, None)
        write_geotagged_jpeg(
            frame=frame_result,
            position=position,
            bearing=None,
            achieved_heading=achieved_heading,
            out_path=out_path,
            heading_source=heading_source,
            mission_context=_mission_context(ctx, wp_name, self.name, None),
        )
        log_banner(
            f"CAPTURE DONE @ {wp_name} | 1 frame", char="-", logger=logger
        )
        return 1


class DriveByStrategy(CaptureStrategy):
    """In-motion capture: single forward frame triggered as the dog passes
    the waypoint, while continuing through the route without stopping.

    Distinct from the other strategies, this one doesn't run *at* a
    waypoint — the whole route runs as one continuous motion in the
    navigator (`navigate_through`), with a callback firing the per-wp
    shutter at closest approach. The per-wp work (frame + position +
    sidecar) lives in `write_drive_by_capture()` below; this class's
    `execute()` is intentionally never called — the strategy name is
    just the dispatch marker mission_runner uses to take the drive-by
    code path.

    Velocity profile, valley radius, and sharp-turn handling are all
    configured via [capture] fields read in `navigator.navigate_through`.
    """

    name = "drive_by"

    async def execute(self, ctx: CaptureContext) -> int:
        raise RuntimeError(
            "DriveByStrategy.execute() should never be called per-waypoint; "
            "the drive-by route runs end-to-end inside "
            "navigator.navigate_through(). Check mission_runner dispatch."
        )


class RotatingQuadratStrategy(CaptureStrategy):
    """Rotate to each absolute bearing in the list, capturing at each.

    Bearings are interpreted as true-north absolute (0 = N, 90 = E,
    etc.). Rotation uses the navigator's calibrated-heading primitive
    (robot IMU + GPS-derived offset established during approach). If
    the navigator is unavailable (static mode), falls back to the
    robot's raw IMU yaw with no true-north calibration and logs a
    warning.

    This is the botanical-quadrat pattern: 4 shots × 90° = a
    spot-sampled "view from this point" per waypoint. Per-bearing
    error is independent (each rotation is to an absolute target, not
    relative to the previous capture) — so a single bad rotation
    doesn't poison subsequent ones.
    """

    name = "rotating_quadrat"

    async def execute(self, ctx: CaptureContext) -> int:
        s = ctx.settings
        wp_name = ctx.waypoint.name if ctx.waypoint else "static"

        log_banner(
            f"CAPTURE @ {wp_name} | {self.name} × {len(s.bearings)}",
            char="-",
            logger=logger,
        )

        n_written = 0
        for bearing in s.bearings:
            success = await _turn_to_bearing(ctx, bearing)
            if not success:
                logger.warning(
                    f"Turn to bearing {bearing:.0f}° failed; "
                    f"continuing with remaining bearings"
                )

            logger.info(f"Settling for {s.settle_time:.1f}s at {bearing:.0f}°")
            await asyncio.sleep(s.settle_time)

            position = await _sample_position(ctx)
            if position is None:
                logger.error(f"No GPS at {wp_name}/{bearing:.0f}°; skipping")
                continue

            frame_result = await capture_frame(
                ctx.robot,
                max_age=s.frame_max_age,
                wait_timeout=s.frame_wait_timeout,
                prefer_clean=s.prefer_clean_frame,
            )
            if frame_result is None:
                logger.error(f"No frame at {wp_name}/{bearing:.0f}°; skipping")
                continue

            achieved_heading, heading_source = _current_bearing(ctx)
            out_path = _capture_output_path(ctx, wp_name, bearing)
            write_geotagged_jpeg(
                frame=frame_result,
                position=position,
                bearing=bearing,
                achieved_heading=achieved_heading,
                out_path=out_path,
                heading_source=heading_source,
                mission_context=_mission_context(ctx, wp_name, self.name, bearing),
            )
            n_written += 1
            log_banner(
                f"FRAME {n_written}/{len(s.bearings)} @ {bearing:.0f}°",
                char="-",
                logger=logger,
            )

        log_banner(
            f"CAPTURE DONE @ {wp_name} | {n_written} frames",
            char="-",
            logger=logger,
        )
        return n_written


# ---- helpers shared by strategies -----------------------------------------


async def _sample_position(ctx: CaptureContext) -> RTKPosition | None:
    """Average GPS for the configured window, else single-shot.

    Returns None when no GPS manager is attached (static_camera mode).
    Strategies that require a position should treat None as "skip
    capture"; strategies that don't need GPS shouldn't call this.
    """
    if ctx.gps is None:
        return None
    s = ctx.settings
    if s.gps_avg_sec > 0:
        return ctx.gps.average_position(s.gps_avg_sec)
    return ctx.gps.get_position()


def _current_bearing(ctx: CaptureContext):
    """Return (bearing_degrees_true, heading_source_label).

    Prefers navigator's calibrated-heading (robot IMU + GPS offset) if
    available. Falls back to raw robot IMU yaw (no calibration,
    relative-to-power-on reference) with a warning.
    """
    nav = ctx.navigator
    if nav is not None:
        calibrated = nav.get_calibrated_heading()
        if calibrated is not None:
            return calibrated, "robot_imu_calibrated"
    yaw = ctx.robot.get_yaw_degrees()
    if yaw is not None:
        logger.warning("Using raw robot IMU yaw — bearing is uncalibrated")
        return yaw, "robot_imu_raw"
    return None, "none"


async def _turn_to_bearing(ctx: CaptureContext, target_bearing_deg: float) -> bool:
    """Command the robot to face `target_bearing_deg` (true-north degrees).

    Uses the navigator's `turn_to_bearing` primitive when available.
    Returns True on success. No-ops with a warning when no navigator
    (static mode with rotating quadrat).
    """
    nav = ctx.navigator
    if nav is None:
        logger.warning(
            "No navigator available; skipping rotation. Captures will "
            "reuse the current robot pose."
        )
        return False
    log_banner(
        f"TURN TO {target_bearing_deg:.0f}° (true)", char="-", logger=logger
    )
    try:
        return await nav.turn_to_bearing(
            target_bearing_deg,
            tolerance_deg=ctx.settings.turn_tolerance_deg,
            timeout=ctx.settings.turn_timeout_sec,
            kp=ctx.settings.turn_kp,
            min_rate_rad_s=ctx.settings.turn_min_rate_rad_s,
        )
    except AttributeError:
        logger.error(
            "Navigator has no turn_to_bearing method; capture rotation "
            "cannot proceed without nav support."
        )
        return False


def _capture_output_path(
    ctx: CaptureContext, wp_name: str, bearing: float | None
) -> Path:
    """Build `<run_dir>/<output_subdir>/<wp>/<bearing>_<ts>.jpg` path.

    Captures live under the per-run directory so a single mission run's
    logs and captures are co-located. Per-waypoint subdirs keep the
    layout legible when waypoint counts grow.

    Timestamp stays in the filename to disambiguate within-run retries
    (e.g. if a rotating_quadrat bearing is captured twice in one run).
    Omits the bearing component when it's None (e.g. static_camera
    mode with no heading data).
    """
    ts = time.strftime("%Y-%m-%dT%H-%M-%S")
    bearing_tag = f"b{int(bearing) % 360:03d}" if bearing is not None else "nobrg"
    safe_wp = wp_name.replace("/", "_").replace(" ", "_")
    fname = f"{bearing_tag}_{ts}.jpg"
    wp_dir = ctx.run_dir / ctx.settings.output_subdir / safe_wp
    wp_dir.mkdir(parents=True, exist_ok=True)
    return wp_dir / fname


def _mission_context(
    ctx: CaptureContext,
    wp_name: str,
    strategy_name: str,
    bearing: float | None,
) -> dict:
    """Sidecar mission context block."""
    return {
        "mission_name": ctx.mission_name,
        "mission_dir": str(ctx.mission_dir),
        "waypoint_name": wp_name,
        "strategy": strategy_name,
        "target_bearing_deg_true": bearing,
    }


async def write_drive_by_capture(
    ctx: CaptureContext,
    wp_name: str,
    position: RTKPosition,
    distance_at_trigger_m: float,
    commanded_speed_m_s: float,
) -> int:
    """In-motion shutter + write. Called from navigator.navigate_through()
    at the closest-pass moment of each waypoint.

    The frame is whatever's freshest in the WebRTC cache at the trigger
    instant; the geotag is the GPS sample passed in (already the
    closest-pass position). No settle, no GPS averaging, no rotation —
    the dog is moving the whole time. Drive-by-specific telemetry
    (distance at trigger, commanded forward velocity) lands in the
    sidecar `extra` block so post-hoc analysis can correlate frame
    quality against motion state.
    """
    frame_result = await capture_frame(
        ctx.robot,
        max_age=ctx.settings.frame_max_age,
        wait_timeout=ctx.settings.frame_wait_timeout,
        target_time=position.timestamp,
        prefer_clean=ctx.settings.prefer_clean_frame,
    )
    if frame_result is None:
        logger.error(f"No fresh frame at {wp_name} (drive_by); skipping")
        return 0

    # Geotag by the chosen frame's own time: interpolate the RTK position
    # to frame_result.timestamp rather than using the trigger position, so
    # an older (cleaner) frame still gets the position it was actually
    # taken at. Falls back to the trigger position if history is too sparse.
    interp = ctx.gps.position_at(frame_result.timestamp) if ctx.gps else None
    geo_pos = interp or position
    position_interpolated = interp is not None

    achieved_heading, heading_source = _current_bearing(ctx)
    out_path = _capture_output_path(ctx, wp_name, bearing=None)
    write_geotagged_jpeg(
        frame=frame_result,
        position=geo_pos,
        bearing=None,
        achieved_heading=achieved_heading,
        out_path=out_path,
        heading_source=heading_source,
        position_interpolated=position_interpolated,
        mission_context=_mission_context(ctx, wp_name, DriveByStrategy.name, None),
        extra={
            "drive_by": {
                "distance_at_trigger_m": distance_at_trigger_m,
                "commanded_speed_m_s": commanded_speed_m_s,
                "frame_offset_from_trigger_s": frame_result.timestamp
                - position.timestamp,
            }
        },
    )
    return 1


STRATEGIES = {
    NoOpStrategy.name: NoOpStrategy,
    FrameOnlyStrategy.name: FrameOnlyStrategy,
    WaypointForwardStrategy.name: WaypointForwardStrategy,
    RotatingQuadratStrategy.name: RotatingQuadratStrategy,
    DriveByStrategy.name: DriveByStrategy,
}


def build_strategy(settings: "CaptureSettings") -> CaptureStrategy:
    """Instantiate the strategy named in `settings.strategy`."""
    cls = STRATEGIES.get(settings.strategy)
    if cls is None:
        raise ValueError(
            f"Unknown capture strategy {settings.strategy!r}; "
            f"valid options: {sorted(STRATEGIES)}"
        )
    return cls()
