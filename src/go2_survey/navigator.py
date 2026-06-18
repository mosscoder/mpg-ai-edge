"""Waypoint navigator: state machine (calibrate -> turn -> walk)."""

import asyncio
import logging
import math
import time
from collections import deque
from typing import NamedTuple

from go2_survey.geometry import (
    calculate_bearing,
    haversine_distance,
    normalize_angle,
    project_along_leg,
)
from go2_survey.gps import GPSManager, RTKPosition
from go2_survey.logging_utils import log_banner
from go2_survey.robot import Go2Robot
from go2_survey.waypoints import Waypoint

logger = logging.getLogger(__name__)

class _TrajSample(NamedTuple):
    """One (time, position, IMU yaw) sample collected while walking. Feeds
    the running-COG recal (line survey) and the endpoint recal (navigate_to).
    """

    t: float
    lat: float
    lon: float
    imu_yaw: float


class WaypointNavigator:
    """Navigation controller with two drive modes:

    * `navigate_legs` — the lawnmower line survey (the primary path): drive
      straight legs between corner waypoints, steering each leg by GPS course
      (`cross_track`) or the calibrated IMU heading (`point_seek`). The IMU
      offset self-seeds and tracks continuously from the survey motion via the
      running COG recal (`_update_running_recal`); no cal walk runs.
    * `navigate_to` — point-to-point to a single waypoint: a three-phase state
      machine (cal walk to seed `_imu_north_offset` → turn to face → walk with
      proportional steering), with the offset refreshed at arrival from the
      leg's GPS+IMU samples (`_recalibrate_from_buffer`).
    """

    # Per-waypoint recalibration tunables. Constants rather than init
    # params: only the on/off switch is exposed via config.
    RECAL_MIN_SAMPLES = 10
    RECAL_MIN_BASELINE_M = 5.0
    RECAL_MAX_DELTA_DEG = 30.0
    RECAL_DROP_RECENT_SEC = 1.0      # ignore samples from arrival deceleration
    RECAL_STRAIGHT_VZ_THRESHOLD = 0.05  # rad/s — only buffer when going straight
    RECAL_BUFFER_MAXLEN = 400        # ~40s at 10Hz — holds a long survey leg's straight body
    # Initial IMU cal-walk baseline for navigate_to (point-to-point). 5 m so the
    # seed heading offset is ~1-2°, not the ~6-18° a 1.5 m chord gave (gait
    # wobble ~0.15 m / baseline sets the floor — see the 2026-05-21 track
    # replay). The line survey self-seeds from the running COG recal below and
    # skips the cal walk entirely.
    CALIBRATION_BASELINE_M = 5.0

    # Running COG recal: a continuous, per-tick circular-EMA of the IMU offset
    # over straight centered-COG samples, self-seeding (no cal walk) — the line
    # survey's calibration. See _update_running_recal. Validated on the 09c replay
    # (heading error mean 7.9->1.9°, corner pre-aim 7.1->3.4°, max jump 26.5->3.8°).
    RUNNING_RECAL_TAU_S = 30.0       # EMA time constant (s)
    RUNNING_RECAL_LOOKBACK_M = 1.5   # centered half-chord each side (m)
    RUNNING_RECAL_GATE_DEG = 25.0    # soft per-sample outlier reject
    # Fold-time measured-straightness gate: reject a centered chord whose IMU
    # yaw turned more than this end-to-end. Replaces the commanded-vz buffer
    # gate for survey legs — cross_track's constant small steering corrections
    # kept |vz| above RECAL_STRAIGHT_VZ_THRESHOLD nearly every tick, starving
    # the estimator (2 folds in the whole 2026-06-11 site_1_strip_3 run). 8°
    # admits gait wobble (±3°/endpoint) and on-line corrections, rejects
    # corner-entry recovery arcs.
    RUNNING_RECAL_MAX_TURN_DEG = 8.0

    def __init__(
        self,
        gps: GPSManager,
        robot: Go2Robot,
        max_velocity: float = 0.3,
        rotation_rate: float = 0.3,
        arrival_tolerance: float = 0.2,
        min_fix_type: int = 5,
        max_hacc: float = 1.0,
        gps_timeout: float = 300.0,
        calibration_timeout: float = 30.0,
        calibration_hacc: float = 0.1,
        imu_recalibrate_on_arrival: bool = True,
    ):
        self.gps = gps
        self.robot = robot
        self.max_velocity = max_velocity
        self.rotation_rate = rotation_rate
        self.arrival_tolerance = arrival_tolerance
        self.min_fix_type = min_fix_type
        self.max_hacc = max_hacc
        self.gps_timeout = gps_timeout
        self.calibration_timeout = calibration_timeout
        self.calibration_hacc = calibration_hacc
        self.imu_recalibrate_on_arrival = imu_recalibrate_on_arrival

        self._running = False
        self._paused = False
        self._pause_start: float | None = None
        self._pause_last_progress: float | None = None
        self._nav_last_status: float | None = None

        self._imu_north_offset: float | None = None
        self._recal_last_t: float | None = None  # running_cog: last folded sample t
        self._running_recal_logged: float = 0.0   # running_cog: log throttle
        self._calibration_start_pos: RTKPosition | None = None
        self._calibration_start_time: float | None = None
        self._calibration_last_progress: float | None = None

        # Trajectory buffer feeding _recalibrate_from_buffer at arrival.
        # Cleared at the start of every navigate_to() so leg N's recal
        # uses only leg N's samples.
        self._traj_buf: deque = deque(maxlen=self.RECAL_BUFFER_MAXLEN)

    def _is_quality_acceptable(
        self, pos: RTKPosition | None, corrections_active: bool
    ) -> bool:
        """Quality gate that doesn't trust receiver float-coast.

        Combines the bare receiver check (fix_type, hAcc) with NTRIP
        corrections freshness: a reported RTK fix (Float / Fixed) is
        only acceptable when corrections are actually flowing. This
        catches the F9P/F9R holding Float for 30-60s after RTCM stops,
        which the system would otherwise navigate on.

        For low-tier fixes (type <= 4 / GNSS-only), corrections are
        not expected and the receiver gate alone applies.
        """
        if pos is None:
            return False
        if pos.fix_type < self.min_fix_type:
            return False
        if pos.accuracy_horizontal > self.max_hacc:
            return False
        # RTK-tier reading without corrections: float-coast, reject.
        if pos.fix_type >= 5 and not corrections_active:
            return False
        return True

    def _fix_lost_diagnostics(
        self, pos: RTKPosition | None, corrections_active: bool
    ) -> tuple[str, str]:
        """Return (fix_info, cause) strings for the GPS FIX LOST banner.

        `fix_info` summarizes the receiver state. `cause` attributes
        the loss to either stale corrections, degraded receiver
        signal, or absent position — so post-mortems can tell whether
        the robot was waiting on cellular network or sky view.
        """
        if pos is None:
            return "no position", "receiver returned no fix"
        age = (
            self.gps.ntrip.seconds_since_last_rtcm()
            if self.gps.ntrip
            else None
        )
        age_str = f" rtcm_age={age:.1f}s" if age is not None else ""
        fix_info = (
            f"type {pos.fix_type} hAcc={pos.accuracy_horizontal:.3f}m{age_str}"
        )
        if pos.fix_type >= 5 and not corrections_active:
            ntrip = self.gps.ntrip
            if ntrip is not None and not ntrip.connection_alive:
                cause = (
                    f"NTRIP stream died (worker exited; state={self.gps.state}); "
                    f"receiver coasting on type {pos.fix_type}"
                )
            else:
                age_repr = f"{age:.1f}s" if age is not None else "∞"
                cause = (
                    f"RTCM stale (age={age_repr} > max="
                    f"{self.gps.max_rtcm_age_s:.1f}s, state={self.gps.state}); "
                    f"receiver coasting on type {pos.fix_type}"
                )
        elif pos.fix_type < self.min_fix_type:
            cause = (
                f"receiver fix degraded to type {pos.fix_type} "
                f"(min {self.min_fix_type})"
            )
        elif pos.accuracy_horizontal > self.max_hacc:
            cause = (
                f"hAcc {pos.accuracy_horizontal:.3f}m exceeds limit "
                f"{self.max_hacc:.3f}m"
            )
        else:
            cause = "see receiver state above"
        return fix_info, cause

    async def _wait_for_fix_resume(self) -> bool:
        """Block until the receiver returns to acceptable quality, or the
        mid-mission GPS timeout expires. Returns True on resume, False on
        timeout — caller is responsible for aborting on False.
        """
        pause_start = time.time()
        last_log = time.time()
        while self._running:
            if time.time() - pause_start > self.gps_timeout:
                logger.error(
                    f"GPS not restored in drive-by within {self.gps_timeout:.0f}s"
                )
                return False
            pos = self.gps.get_position()
            corrections_active = self.gps.has_active_corrections()
            if self._is_quality_acceptable(pos, corrections_active):
                logger.info(
                    f"GPS restored after {time.time() - pause_start:.0f}s; "
                    f"resuming drive-by"
                )
                await self.robot.balance_stand()
                await asyncio.sleep(1.0)
                return True
            now = time.time()
            if now - last_log >= 15.0:
                logger.info(
                    f"GPS still degraded… "
                    f"({now - pause_start:.0f}s/{self.gps_timeout:.0f}s)"
                )
                last_log = now
            await asyncio.sleep(0.5)
        return False

    async def _run_cal_walk(self, timeout: float) -> bool:
        """Walk straight forward (vz=0) until the IMU<->north offset is set.

        Extracted phase-1 cal walk, shared by navigate_to and the line
        survey. Idempotent: a no-op returning True when _imu_north_offset is
        already set (a navigator calibrates once). Buffers each clean (vz=0)
        sample, calls _calibrate_imu() each tick until it returns True
        (displacement >= CALIBRATION_BASELINE_M at hAcc <= calibration_hacc),
        then stops + balance-stands. Returns False on timeout or if stopped.
        The caller reads the cal-endpoint fix via self.gps.get_position().
        """
        if self._imu_north_offset is not None:
            return True
        log_banner("CALIBRATING IMU", char="-", logger=logger)
        cal_start = time.time()
        while self._running:
            if time.time() - cal_start > timeout:
                logger.error("Timeout during IMU calibration")
                log_banner(
                    "CALIBRATION TIMEOUT", level="error", char="!", logger=logger
                )
                await self.robot.stop()
                return False

            pos = self.gps.get_position()
            if not pos:
                await asyncio.sleep(0.2)
                continue

            imu_yaw = self.robot.get_yaw_degrees()
            if imu_yaw is not None:
                # Cal walk is pure forward motion (vz=0) — the cleanest
                # samples for the buffer recal at arrival.
                self._maybe_buffer_sample(pos, imu_yaw, vz=0.0)
                if self._calibrate_imu(pos, imu_yaw):
                    await self.robot.stop()
                    await self.robot.balance_stand()
                    await asyncio.sleep(1.0)
                    return True

            await self.robot.send_velocity(x=self.max_velocity)
            await asyncio.sleep(0.2)
        return False

    async def _drive_leg(
        self,
        start: Waypoint,
        end: Waypoint,
        *,
        leg_label: str,
        interval_m: float,
        speed: float,
        turn_tolerance_deg: float,
        leg_steering: str,
        lookahead_m: float,
        course_lookback_m: float,
        do_capture: bool,
        timeout: float,
        on_capture_cb=None,
        n_captured_in: int = 0,
        mark_offset: int = 0,
        capture_start_corner: bool = True,
    ) -> tuple[bool, int]:
        """Turn to the leg bearing, then drive start->end on the configured
        steering law (cross_track | point_seek), optionally firing interval +
        corner captures, and recal the IMU offset from the leg buffer on
        arrival. Shared by the survey legs (do_capture=True) and the
        cal-endpoint -> first-corner approach (do_capture=False,
        on_capture_cb=None). Returns (ok, n_captured_total).

        ``mark_offset`` is added to every mark index passed to ``on_capture_cb``
        (and the capture filenames it writes); ``capture_start_corner=False``
        skips the leg-start shot. Both are set on a resume's first (partial)
        leg so it continues the original ``legNN_m<MMM>`` numbering from the
        anchor mark instead of re-shooting it (see :mod:`resume`).
        """
        leg_bearing = calculate_bearing(
            start.latitude, start.longitude, end.latitude, end.longitude
        )
        leg_len = haversine_distance(
            start.latitude, start.longitude, end.latitude, end.longitude
        )
        n_captured = n_captured_in

        # Corner turn (no capture). Tolerance matches the rotating-quadrat
        # strategy so the leg-start corner capture is tightly aligned. Skipped
        # while uncalibrated (running_cog self-seeds on the approach) — the turn
        # needs the offset; cross_track converges onto the line without it.
        await self.robot.stop()
        if self._imu_north_offset is not None:
            await self.turn_to_bearing(
                leg_bearing, tolerance_deg=turn_tolerance_deg, timeout=20.0
            )
        else:
            logger.info(
                f"{leg_label}: uncalibrated start — cross_track converges onto "
                f"the line"
            )

        # Recompute the IMU<->north offset from each leg's straight-line
        # GPS+IMU samples (a long baseline => low-noise recal). running_cog
        # consumes this buffer per-tick — restart its pointer each leg so the
        # centered window stays leg-clamped.
        self._traj_buf.clear()
        self._recal_last_t = None
        pos_hist: deque = deque(maxlen=64)  # recent (lat, lon) for GPS course
        next_mark = interval_m
        prev_along = 0.0
        prev_t = time.time()
        mark_index = 0
        leg_start = time.time()
        last_status = 0.0
        last_fired_along = -interval_m  # for end-of-leg de-dup

        # Capture at the leg-start corner (heading just settled from the
        # turn). Each interior corner thus gets two shots: the previous
        # leg's end capture (incoming heading) and this start (outgoing).
        if do_capture and capture_start_corner:
            start_pos = self.gps.get_position()
            if start_pos is not None and self._is_quality_acceptable(
                start_pos, self.gps.has_active_corrections()
            ):
                n_written = await on_capture_cb(
                    leg_label, mark_index + mark_offset, time.time(), 0.0,
                    start_pos, leg_bearing,
                )
                n_captured += n_written
                last_fired_along = 0.0
                log_banner(
                    f"CAPTURE {leg_label} start corner | n={n_written}",
                    char="-", logger=logger,
                )

        while self._running:
            if time.time() - leg_start > timeout:
                logger.error(
                    f"Line-survey timeout on {leg_label} after {timeout:.0f}s"
                )
                await self.robot.stop()
                return False, n_captured

            pos = self.gps.get_position()
            corrections_active = self.gps.has_active_corrections()
            if not self._is_quality_acceptable(pos, corrections_active):
                fix_info, cause = self._fix_lost_diagnostics(
                    pos, corrections_active
                )
                logger.warning(
                    f"GPS lost mid-line-survey ({fix_info}); pausing | {cause}"
                )
                await self.robot.stop()
                if not await self._wait_for_fix_resume():
                    return False, n_captured
                leg_start = time.time()
                prev_t = time.time()
                continue

            now = time.time()
            along, cross = project_along_leg(
                start.latitude, start.longitude,
                end.latitude, end.longitude,
                pos.latitude, pos.longitude,
            )

            # Fire a capture at each interval mark crossed this tick.
            if do_capture:
                while next_mark <= min(along, leg_len):
                    frac = (
                        (next_mark - prev_along) / (along - prev_along)
                        if along > prev_along else 1.0
                    )
                    t_mark = prev_t + frac * (now - prev_t)
                    mark_index += 1
                    n_written = await on_capture_cb(
                        leg_label, mark_index + mark_offset, t_mark, next_mark,
                        pos, leg_bearing,
                    )
                    n_captured += n_written
                    last_fired_along = next_mark
                    log_banner(
                        f"CAPTURE {leg_label} mark {mark_index + mark_offset} @ "
                        f"{next_mark:.1f}m | n={n_written}",
                        char="-",
                        logger=logger,
                    )
                    next_mark += interval_m

            # Arrival at the end corner.
            dist_to_end = haversine_distance(
                pos.latitude, pos.longitude, end.latitude, end.longitude
            )
            if (
                along >= leg_len - self.arrival_tolerance
                or dist_to_end < self.arrival_tolerance
            ):
                # Capture at the leg-end corner before turning away —
                # unless an interval mark already landed within 0.5 m of it.
                if do_capture and along - last_fired_along > 0.5:
                    mark_index += 1
                    n_written = await on_capture_cb(
                        leg_label, mark_index + mark_offset, time.time(), along,
                        pos, leg_bearing,
                    )
                    n_captured += n_written
                    log_banner(
                        f"CAPTURE {leg_label} end corner @ {along:.1f}m | "
                        f"n={n_written}",
                        char="-", logger=logger,
                    )
                break

            # Leg steering — two laws (config [capture] leg_steering):
            #   point_seek (default): re-aim at the end corner each tick on
            #     the calibrated IMU heading. A small residual offset curves
            #     the path toward the corner and converges; a drifted offset
            #     makes the dog bow off-line confidently (hdg_err~0 while
            #     cross grows) and, at speed, never recover (see v0.26.0).
            #   cross_track: pure-pursuit a near carrot on the leg line,
            #     steering on GPS-derived course (offset-free) instead of the
            #     IMU. Immune to IMU-offset drift; tracks the line. Same
            #     proportional law/sign: +error -> CCW-negative z on the Go2.
            if leg_steering == "cross_track":
                pos_hist.append((pos.latitude, pos.longitude))
                cog = self._gps_course(pos_hist, course_lookback_m)
                if cog is None:
                    # Too little travel for a clean course baseline yet —
                    # drive straight (doubles as a per-leg cal-stretch).
                    heading_error = 0.0
                else:
                    carrot_along = min(along + lookahead_m, leg_len)
                    frac = carrot_along / leg_len if leg_len > 0 else 1.0
                    carrot_lat = start.latitude + (end.latitude - start.latitude) * frac
                    carrot_lon = start.longitude + (end.longitude - start.longitude) * frac
                    carrot_brg = calculate_bearing(
                        pos.latitude, pos.longitude, carrot_lat, carrot_lon
                    )
                    heading_error = normalize_angle(carrot_brg - cog)
            else:
                cog = None
                bearing = calculate_bearing(
                    pos.latitude, pos.longitude, end.latitude, end.longitude
                )
                heading = self.get_calibrated_heading()
                heading_error = (
                    normalize_angle(bearing - heading)
                    if heading is not None else 0.0
                )
            vz = max(
                -self.rotation_rate,
                min(self.rotation_rate, heading_error * -0.015),
            )

            imu_yaw_now = self.robot.get_yaw_degrees()
            if imu_yaw_now is not None:
                # Legs are straight by construction — buffer every quality
                # sample; the running recal gates on measured chord turn.
                self._maybe_buffer_sample(
                    pos, imu_yaw_now, vz=vz, enforce_straight=False
                )
                self._update_running_recal()

            await self.robot.send_velocity(x=speed, z=vz)

            if now - last_status >= 2.0:
                cog_str = f"{cog:.0f}°" if cog is not None else "—"
                logger.info(
                    f"NAV [{leg_steering}] {leg_label} | along={along:.1f}/"
                    f"{leg_len:.1f}m cross={cross:.2f}m "
                    f"hdg_err={heading_error:.1f}° cog={cog_str} "
                    f"fix={pos.fix_type}"
                )
                last_status = now

            prev_along, prev_t = along, now
            await asyncio.sleep(0.1)

        await self.robot.stop()
        # The line survey calibrates continuously via _update_running_recal
        # (per tick, above) — no per-arrival recompute needed.
        return True, n_captured

    async def navigate_legs(
        self,
        waypoints: list[Waypoint],
        on_capture_cb,
        interval_m: float,
        speed: float,
        turn_tolerance_deg: float = 2.0,
        timeout_per_leg: float = 300.0,
        leg_steering: str = "point_seek",
        lookahead_m: float = 4.0,
        course_lookback_m: float = 1.0,
        health=None,
        leg_number_start: int = 1,
        first_leg_mark_offset: int = 0,
        first_leg_skip_start_capture: bool = False,
        n_captured_start: int = 0,
    ) -> bool:
        """Lawnmower line survey: drive straight legs between consecutive
        waypoints (the leg corners), turning in place at each corner, and
        fire `on_capture_cb` every `interval_m` of along-track travel.

        The first corner is reached via the same `_drive_leg` as a survey leg
        (captures off); the IMU offset self-seeds from that approach motion via
        the running COG recal, so no cal walk runs. Per leg: turn to the leg
        bearing, then steer the end corner (`leg_steering`), firing a capture
        each time along-track distance crosses the next interval mark. Captures
        fire only while driving a leg, never during the corner turns — so on a
        straight leg the camera points along-track and coverage is even by real
        distance.

        `on_capture_cb(leg_label, mark_index, t_mark, along_m, position,
        leg_bearing) -> int` (frames written). Returns True on success,
        False on GPS-loss or per-leg timeout.

        Resume (see :mod:`resume`): when picking up an interrupted survey,
        `waypoints[0]` is the anchor mark's position and `waypoints[1:]` the
        remaining corners; `leg_number_start` continues the `legNN` numbering
        (= the anchor leg), `first_leg_mark_offset` continues `m<MMM>` from the
        anchor mark, and `first_leg_skip_start_capture` avoids re-shooting the
        anchor. `n_captured_start` seeds the completion tally. All default to a
        fresh run (start at leg01/m000, capture the first corner).
        """
        if len(waypoints) < 2:
            logger.error("navigate_legs needs at least 2 waypoints (1 leg)")
            return False

        log_banner(
            f"LINE SURVEY | {len(waypoints)} corners / {len(waypoints) - 1} "
            f"legs | {speed:.2f}m/s | capture every {interval_m:.1f}m",
            logger=logger,
        )

        self._running = True

        # The running COG recal self-seeds the IMU offset from the approach
        # motion, so the line survey needs no dedicated cal walk. Drive the
        # start fix -> first-corner approach on the SAME cross_track _drive_leg
        # the survey legs use (captures off) while the offset self-seeds.
        start_pos = self.gps.get_position()
        if start_pos is None or not self._is_quality_acceptable(
            start_pos, self.gps.has_active_corrections()
        ):
            logger.error("No quality GPS fix at line-survey start")
            return False
        approach_start = Waypoint(
            latitude=start_pos.latitude, longitude=start_pos.longitude, name="start"
        )
        if (
            haversine_distance(
                approach_start.latitude, approach_start.longitude,
                waypoints[0].latitude, waypoints[0].longitude,
            )
            > self.arrival_tolerance
        ):
            ok, _ = await self._drive_leg(
                approach_start, waypoints[0],
                leg_label="approach", on_capture_cb=None,
                interval_m=interval_m, speed=speed,
                turn_tolerance_deg=turn_tolerance_deg, leg_steering=leg_steering,
                lookahead_m=lookahead_m, course_lookback_m=course_lookback_m,
                do_capture=False, timeout=timeout_per_leg,
            )
            if not ok:
                logger.error("Failed to reach line-survey start corner")
                return False

        n_captured = n_captured_start
        for i in range(len(waypoints) - 1):
            if not self._running:
                break
            s, e = waypoints[i], waypoints[i + 1]
            leg_label = f"leg{leg_number_start + i:02d}"
            leg_bearing = calculate_bearing(
                s.latitude, s.longitude, e.latitude, e.longitude
            )
            leg_len = haversine_distance(
                s.latitude, s.longitude, e.latitude, e.longitude
            )
            log_banner(
                f"LINE SURVEY {leg_label} ({i + 1}/{len(waypoints) - 1}) -> "
                f"{e.name} | brg={leg_bearing:.0f}° len={leg_len:.1f}m",
                char="-",
                logger=logger,
            )
            ok, n_captured = await self._drive_leg(
                s, e, leg_label=leg_label, on_capture_cb=on_capture_cb,
                interval_m=interval_m, speed=speed,
                turn_tolerance_deg=turn_tolerance_deg, leg_steering=leg_steering,
                lookahead_m=lookahead_m, course_lookback_m=course_lookback_m,
                do_capture=True, timeout=timeout_per_leg, n_captured_in=n_captured,
                mark_offset=(first_leg_mark_offset if i == 0 else 0),
                capture_start_corner=not (first_leg_skip_start_capture and i == 0),
            )
            if not ok:
                return False
            # Per-leg HEALTH banner at the corner just reached (waypoint
            # boundary): battery + thigh temps + per-leg rise + trend.
            if health is not None:
                health.emit_leg(self.robot, i + 1, len(waypoints) - 1)

        await self.robot.stop()
        await self.robot.balance_stand()
        log_banner(f"LINE SURVEY COMPLETE | {n_captured} frame(s)", logger=logger)
        return True

    async def navigate_to(self, waypoint: Waypoint, timeout: float = 300.0) -> bool:
        """Drive the robot to a single waypoint. Returns True on success."""
        logger.info(
            f"Navigating to {waypoint.name}: "
            f"({waypoint.latitude:.8f}, {waypoint.longitude:.8f})"
        )
        log_banner(f"NAVIGATING TO {waypoint.name}", logger=logger)

        self._running = True
        self._paused = False
        self._nav_last_status = None
        self._traj_buf.clear()
        nav_start = time.time()

        await self.robot.balance_stand()
        await asyncio.sleep(1.0)

        # Phase 1: Calibrate IMU (once per navigator) — shared cal walk.
        if not await self._run_cal_walk(timeout):
            return False

        # Phase 2: Turn to face waypoint
        pos = self.gps.get_position()
        if pos:
            bearing = calculate_bearing(
                pos.latitude, pos.longitude, waypoint.latitude, waypoint.longitude
            )
            current_heading = self.get_calibrated_heading()
            if current_heading is not None:
                error = normalize_angle(bearing - current_heading)
                if abs(error) > 30:
                    log_banner("TURNING TO WAYPOINT", char="-", logger=logger)
                    direction = -1.0 if error > 0 else 1.0
                    logger.info(
                        f"Turn direction: {'CCW' if direction > 0 else 'CW'} "
                        f"(heading={current_heading:.1f}° "
                        f"bearing={bearing:.1f}° error={error:.1f}°)"
                    )
                    await self.robot.balance_stand()
                    await asyncio.sleep(1.0)

                    turn_last_log = 0.0
                    while self._running:
                        if time.time() - nav_start > timeout:
                            logger.error(
                                f"Timeout during turn for {waypoint.name}"
                            )
                            await self.robot.stop()
                            return False

                        current_heading = self.get_calibrated_heading()
                        if current_heading is None:
                            await asyncio.sleep(0.1)
                            continue

                        pos = self.gps.get_position()
                        bearing = (
                            calculate_bearing(
                                pos.latitude,
                                pos.longitude,
                                waypoint.latitude,
                                waypoint.longitude,
                            )
                            if pos
                            else bearing
                        )
                        error = normalize_angle(bearing - current_heading)

                        now = time.time()
                        if now - turn_last_log >= 2.0:
                            logger.info(
                                f"[turn] hdg={current_heading:.1f}° "
                                f"brg={bearing:.1f}° err={error:.1f}°"
                            )
                            turn_last_log = now

                        if abs(error) < 30:
                            await self.robot.stop()
                            logger.info(
                                f"Aligned to waypoint — "
                                f"heading: {current_heading:.1f}° "
                                f"(error: {error:.1f}°)"
                            )
                            await self.robot.balance_stand()
                            await asyncio.sleep(1.0)
                            break

                        await self.robot.send_velocity(
                            z=direction * self.rotation_rate
                        )
                        await asyncio.sleep(0.1)

        # Phase 3: Walk to waypoint with proportional steering
        log_banner("WALKING TO WAYPOINT", char="-", logger=logger)
        while self._running:
            if time.time() - nav_start > timeout:
                pos_check = self.gps.get_position()
                remaining = ""
                if pos_check:
                    dist = haversine_distance(
                        pos_check.latitude,
                        pos_check.longitude,
                        waypoint.latitude,
                        waypoint.longitude,
                    )
                    remaining = f" | {dist:.1f}m remaining"
                logger.error(
                    f"Navigation timeout after {timeout:.0f}s for waypoint "
                    f"{waypoint.name}{remaining}"
                )
                log_banner(
                    f"NAVIGATION TIMEOUT | {waypoint.name} | {timeout:.0f}s",
                    level="error",
                    char="!",
                    logger=logger,
                )
                await self.robot.stop()
                return False

            pos = self.gps.get_position()
            corrections_active = self.gps.has_active_corrections()

            if not self._is_quality_acceptable(pos, corrections_active):
                fix_info, cause = self._fix_lost_diagnostics(pos, corrections_active)
                if not self._paused:
                    logger.warning(
                        f"GPS fix lost or degraded ({fix_info}), pausing robot..."
                    )
                    log_banner(
                        f"GPS FIX LOST | {fix_info} | "
                        f"Cause: {cause} | "
                        f"robot paused | mid-mission timeout {self.gps_timeout:.0f}s",
                        level="warning",
                        char="!",
                        logger=logger,
                    )
                    await self.robot.stop()
                    self._paused = True
                    self._pause_start = time.time()
                    self._pause_last_progress = time.time()
                    # Kick off async NTRIP reconnect when corrections
                    # are the cause and the manager has a stream to
                    # repair (i.e. NTRIP was previously connected).
                    if (
                        not corrections_active
                        and self.gps.state in ("connected", "degraded", "lost")
                    ):
                        self.gps.reconnect_ntrip_async()
                elif time.time() - self._pause_start > self.gps_timeout:
                    pause_elapsed = time.time() - self._pause_start
                    logger.error(
                        f"GPS fix not restored after {pause_elapsed:.0f}s "
                        f"(limit: {self.gps_timeout:.0f}s), "
                        f"aborting navigation to {waypoint.name}"
                    )
                    log_banner(
                        f"GPS PAUSE TIMEOUT | {pause_elapsed:.0f}s exhausted | "
                        f"Cause: {cause} | "
                        f"Aborting navigation to {waypoint.name}",
                        level="error",
                        char="!",
                        logger=logger,
                    )
                    await self.robot.stop()
                    return False
                else:
                    now = time.time()
                    if now - self._pause_last_progress >= 15.0:
                        pause_elapsed = now - self._pause_start
                        logger.info(
                            f"GPS fix lost -- still waiting... "
                            f"({pause_elapsed:.0f}s/{self.gps_timeout:.0f}s | "
                            f"NTRIP state={self.gps.state})"
                        )
                        self._pause_last_progress = now
                await asyncio.sleep(0.5)
                continue

            if self._paused:
                pause_duration = time.time() - self._pause_start
                age = (
                    self.gps.ntrip.seconds_since_last_rtcm()
                    if self.gps.ntrip
                    else None
                )
                age_str = (
                    f" rtcm_age={age:.1f}s" if age is not None else ""
                )
                logger.info(
                    f"GPS fix restored (type {pos.fix_type}), resuming navigation"
                )
                log_banner(
                    f"GPS FIX RESTORED | type {pos.fix_type} "
                    f"hAcc {pos.accuracy_horizontal:.3f}m | "
                    f"paused {pause_duration:.0f}s | "
                    f"NTRIP state={self.gps.state}{age_str}",
                    logger=logger,
                )
                self._paused = False
                self._pause_start = None
                self._pause_last_progress = None
                await self.robot.balance_stand()
                await asyncio.sleep(1.0)

            distance = haversine_distance(
                pos.latitude, pos.longitude, waypoint.latitude, waypoint.longitude
            )
            bearing = calculate_bearing(
                pos.latitude, pos.longitude, waypoint.latitude, waypoint.longitude
            )

            if distance < self.arrival_tolerance:
                elapsed = time.time() - nav_start
                logger.info(f"ARRIVED at {waypoint.name}! Distance: {distance:.3f}m")
                log_banner(
                    f"ARRIVED at {waypoint.name} | {distance:.3f}m | {elapsed:.0f}s",
                    logger=logger,
                )
                await self.robot.stop()
                self._recalibrate_from_buffer()
                return True

            current_heading = self.get_calibrated_heading()
            heading_error = (
                normalize_angle(bearing - current_heading)
                if current_heading is not None
                else 0
            )

            vx, vz = self._compute_velocity(distance, heading_error)

            # Buffer this sample BEFORE sending the new velocity so the
            # vz filter reflects the steering load for this tick — only
            # samples taken during near-straight motion contribute.
            imu_yaw_now = self.robot.get_yaw_degrees()
            if imu_yaw_now is not None:
                self._maybe_buffer_sample(pos, imu_yaw_now, vz=vz)

            await self.robot.send_velocity(x=vx, z=vz)

            calibrated = "yes" if self._imu_north_offset is not None else "no"
            logger.debug(
                f"Pos: ({pos.latitude:.8f}, {pos.longitude:.8f}) | "
                f"Dist: {distance:.2f}m | Fix: {pos.fix_type} | "
                f"hAcc: {pos.accuracy_horizontal:.3f}m | "
                f"Bearing: {bearing:.1f}° | Heading: {current_heading or '?'}° | "
                f"Error: {heading_error:.1f}° | Vel: x={vx:.2f}, z={vz:.2f} | "
                f"IMU cal: {calibrated}"
            )

            now = time.time()
            if self._nav_last_status is None or now - self._nav_last_status >= 5.0:
                heading_str = (
                    f"{current_heading:.1f}°" if current_heading is not None else "?"
                )
                logger.info(
                    f"NAV | ({pos.latitude:.8f}, {pos.longitude:.8f}) | "
                    f"fix={pos.fix_type} hAcc={pos.accuracy_horizontal:.3f}m | "
                    f"dist={distance:.2f}m | hdg={heading_str} | "
                    f"IMU cal: {calibrated}"
                )
                self._nav_last_status = now

            await asyncio.sleep(0.2)

        await self.robot.stop()
        return False

    def _calibrate_imu(self, pos: RTKPosition, imu_yaw: float) -> bool:
        """Compute IMU yaw -> true-heading offset from GPS bearing over displacement.

        The Go2 IMU reports yaw CCW-positive while GPS bearing is CW-positive,
        so the offset is computed as normalize_angle(gps_bearing + imu_yaw).
        """
        if self._calibration_start_pos is None:
            if pos.accuracy_horizontal > self.calibration_hacc:
                return False
            self._calibration_start_pos = pos
            self._calibration_start_time = time.time()
            self._calibration_last_progress = time.time()
            logger.info(
                f"IMU calibration started (hAcc: {pos.accuracy_horizontal:.3f}m) "
                f"- walking forward to calibrate..."
            )
            log_banner("IMU CALIBRATION STARTED", char="-", logger=logger)
            return False

        displacement = haversine_distance(
            self._calibration_start_pos.latitude,
            self._calibration_start_pos.longitude,
            pos.latitude,
            pos.longitude,
        )

        elapsed = time.time() - self._calibration_start_time
        if elapsed > self.calibration_timeout:
            logger.error(
                f"IMU calibration timeout after {self.calibration_timeout:.0f}s "
                f"(displacement: {displacement:.2f}m / "
                f"{self.CALIBRATION_BASELINE_M:.2f}m needed). "
                f"Resetting to allow retry."
            )
            log_banner(
                f"IMU CALIBRATION TIMEOUT after {self.calibration_timeout:.0f}s",
                level="error",
                char="!",
                logger=logger,
            )
            self._calibration_start_pos = None
            self._calibration_start_time = None
            self._calibration_last_progress = None
            return False

        now = time.time()
        if (
            self._calibration_last_progress is not None
            and now - self._calibration_last_progress >= 5.0
        ):
            logger.info(
                f"IMU calibrating... displacement: {displacement:.2f}m / "
                f"{self.CALIBRATION_BASELINE_M:.2f}m needed "
                f"hAcc: {pos.accuracy_horizontal:.3f}m ({elapsed:.0f}s)"
            )
            self._calibration_last_progress = now

        if (
            displacement >= self.CALIBRATION_BASELINE_M
            and pos.accuracy_horizontal <= self.calibration_hacc
        ):
            gps_bearing = calculate_bearing(
                self._calibration_start_pos.latitude,
                self._calibration_start_pos.longitude,
                pos.latitude,
                pos.longitude,
            )
            self._imu_north_offset = normalize_angle(gps_bearing + imu_yaw)
            logger.info(
                f"IMU calibrated! Offset: {self._imu_north_offset:.1f}° "
                f"(GPS bearing: {gps_bearing:.1f}°, IMU yaw: {imu_yaw:.1f}°, "
                f"hAcc: {pos.accuracy_horizontal:.3f}m, disp: {displacement:.2f}m)"
            )
            log_banner(
                f"IMU CALIBRATED | Offset: {self._imu_north_offset:.1f} deg",
                logger=logger,
            )
            return True

        return False

    def _gps_course(self, hist: deque, lookback_m: float) -> float | None:
        """GPS course-over-ground (compass deg) from the position history:
        bearing from the most recent fix that is >= `lookback_m` behind, to the
        newest fix. Returns None until that baseline exists (start of a leg).

        This is the offset-free heading reference for cross_track steering — it
        measures where the dog is actually *moving*, immune to IMU-yaw drift.
        Reliable at survey speed: the ~1.4 cm per-fix RTK noise over a ~1 m
        baseline is <1° of course noise (the low-speed COG problem that drove
        the IMU approach is a sub-0.3 m/s effect; legs run at 0.5-1.0 m/s).
        """
        if len(hist) < 2:
            return None
        lat_now, lon_now = hist[-1]
        for j in range(len(hist) - 2, -1, -1):
            lat_b, lon_b = hist[j]
            if haversine_distance(lat_b, lon_b, lat_now, lon_now) >= lookback_m:
                return calculate_bearing(lat_b, lon_b, lat_now, lon_now)
        return None

    def _maybe_buffer_sample(
        self,
        pos: RTKPosition,
        imu_yaw: float,
        vz: float,
        enforce_straight: bool = True,
    ) -> None:
        """Append a trajectory sample if it passes quality + straight-line filters.

        Quality gate matches the navigation gate (`min_fix_type`,
        `max_hacc`) so degraded GPS samples never feed the recal.
        Straight-line gate (`|vz| <= RECAL_STRAIGHT_VZ_THRESHOLD`) keeps
        only samples taken during near-zero commanded rotation, so the
        chord-vs-curve geometric error in `calculate_bearing(A, B)`
        stays small. Survey-leg drives pass ``enforce_straight=False``:
        legs are straight by construction (the buffer clears at each
        corner) and cross_track's constant small vz corrections would
        otherwise starve the buffer — the running recal applies its own
        measured-straightness gate at fold time instead
        (`RUNNING_RECAL_MAX_TURN_DEG`).
        """
        if pos.fix_type < self.min_fix_type:
            return
        if pos.accuracy_horizontal > self.max_hacc:
            return
        if enforce_straight and abs(vz) > self.RECAL_STRAIGHT_VZ_THRESHOLD:
            return
        self._traj_buf.append(
            _TrajSample(
                t=time.time(),
                lat=pos.latitude,
                lon=pos.longitude,
                imu_yaw=imu_yaw,
            )
        )

    def _update_running_recal(self) -> None:
        """Continuously refine `_imu_north_offset` from straight centered-COG
        samples. Self-seeding: sets the offset from the first centered-complete
        sample (no cal walk needed), then folds later samples with a
        time-constant circular EMA. Called once per drive tick from
        `_drive_leg`, right after `_maybe_buffer_sample`.

        Each "fresh" estimate is `centered_cog + imu_yaw` (the `_calibrate_imu`
        convention): the bearing of the chord from a buffered fix
        >= RUNNING_RECAL_LOOKBACK_M behind a sample to one >= that ahead, plus
        that sample's IMU yaw. `_traj_buf` holds the current leg's quality
        samples (cleared at each leg start, so a centered chord never straddles
        a corner); straightness is enforced HERE, on measured geometry — a
        chord whose IMU yaw turned more than RUNNING_RECAL_MAX_TURN_DEG
        end-to-end (a corner-entry recovery arc, not gait wobble) is skipped.
        """
        buf = self._traj_buf
        if len(buf) < 3:
            return
        lb = self.RUNNING_RECAL_LOOKBACK_M
        newest = buf[-1]
        for c in buf:
            if self._recal_last_t is not None and c.t <= self._recal_last_t:
                continue
            if haversine_distance(c.lat, c.lon, newest.lat, newest.lon) < lb:
                break  # c (and all later) lack a full forward window yet
            back = None
            for p in buf:
                if p.t >= c.t:
                    break
                if haversine_distance(p.lat, p.lon, c.lat, c.lon) >= lb:
                    back = p  # nearest sample >= lb behind c
            if back is None:
                continue  # no full backward window yet (leg start)
            fwd = None
            for p in reversed(buf):
                if p.t <= c.t:
                    break
                if haversine_distance(p.lat, p.lon, c.lat, c.lon) >= lb:
                    fwd = p  # nearest sample >= lb ahead of c
            if fwd is None:
                continue
            if (
                abs(normalize_angle(fwd.imu_yaw - back.imu_yaw))
                > self.RUNNING_RECAL_MAX_TURN_DEG
            ):
                # Body turned across the chord (corner-entry recovery arc) —
                # the chord bearing wouldn't represent the body heading.
                self._recal_last_t = c.t  # consume; this chord never improves
                continue
            centered_cog = calculate_bearing(back.lat, back.lon, fwd.lat, fwd.lon)
            fresh = normalize_angle(centered_cog + c.imu_yaw)
            self._fold_offset(fresh, c.t)
            self._recal_last_t = c.t

    def _fold_offset(self, fresh: float, t: float) -> None:
        """Blend `fresh` into `_imu_north_offset` with a time-constant circular
        EMA. Self-seeds when the offset is None; soft-rejects a lone outlier
        (never a whole leg, the failure mode of the endpoint guardrail)."""
        off = self._imu_north_offset
        if off is None:
            self._imu_north_offset = fresh % 360.0
            logger.info(f"IMU RUNNING RECAL | seed offset {fresh % 360.0:.1f}°")
            return
        if abs(normalize_angle(fresh - off)) > self.RUNNING_RECAL_GATE_DEG:
            return
        dt = t - self._recal_last_t if self._recal_last_t is not None else 1.0
        a = 1.0 - math.exp(-max(dt, 0.05) / self.RUNNING_RECAL_TAU_S)
        x = (1 - a) * math.cos(math.radians(off)) + a * math.cos(math.radians(fresh))
        y = (1 - a) * math.sin(math.radians(off)) + a * math.sin(math.radians(fresh))
        new = math.degrees(math.atan2(y, x)) % 360.0
        self._imu_north_offset = new
        now = time.time()
        if now - self._running_recal_logged >= 5.0:
            logger.info(
                f"IMU RUNNING RECAL | offset {new:.1f}° "
                f"(fresh {fresh:.1f}°, Δ {normalize_angle(new - off):+.2f}°)"
            )
            self._running_recal_logged = now

    def _recalibrate_from_buffer(self) -> None:
        """Refresh the IMU offset from the leg's GPS+IMU buffer on arrival
        (navigate_to / point-to-point). Skips when disabled or before the
        first-leg cal walk has set the offset. See `_recalibrate_endpoint`.
        """
        if not self.imu_recalibrate_on_arrival:
            return
        if self._imu_north_offset is None:
            # First-leg cal walk hasn't completed; nothing to refine.
            return
        self._recalibrate_endpoint()

    def _recalibrate_endpoint(self) -> None:
        """Refresh `_imu_north_offset` from the leg's GPS endpoints.

        Picks the longest available straight-line baseline in the buffer:
        A = earliest sample, B = most recent sample (after dropping the
        last `RECAL_DROP_RECENT_SEC` of arrival deceleration noise).
        Computes `gps_bearing = bearing(A → B)` and sets
        `new_offset = normalize_angle(gps_bearing + B.imu_yaw)` —
        same sign convention as the first-leg cal walk in
        `_calibrate_imu`.

        Skips with a warning if any of:
          - too few samples ever buffered
          - too few samples remain after dropping the deceleration zone
          - resulting baseline shorter than `RECAL_MIN_BASELINE_M`
          - proposed offset shift exceeds `RECAL_MAX_DELTA_DEG` (likely
            a corrupt sample, not real drift in a single leg)
        """
        n = len(self._traj_buf)
        if n < self.RECAL_MIN_SAMPLES:
            logger.warning(
                f"IMU recal [endpoint]: only {n}/{self.RECAL_MIN_SAMPLES} "
                f"samples in buffer; keeping offset "
                f"{self._imu_north_offset:.1f}°"
            )
            return

        cutoff = time.time() - self.RECAL_DROP_RECENT_SEC
        candidates = [s for s in self._traj_buf if s.t <= cutoff]
        if len(candidates) < 2:
            logger.warning(
                f"IMU recal [endpoint]: only {len(candidates)} samples "
                f"remain after dropping last {self.RECAL_DROP_RECENT_SEC:.1f}s; "
                f"keeping offset {self._imu_north_offset:.1f}°"
            )
            return

        a = candidates[0]
        b = candidates[-1]
        baseline = haversine_distance(a.lat, a.lon, b.lat, b.lon)
        if baseline < self.RECAL_MIN_BASELINE_M:
            logger.warning(
                f"IMU recal [endpoint]: baseline {baseline:.2f}m < "
                f"{self.RECAL_MIN_BASELINE_M:.1f}m minimum across "
                f"{len(candidates)} samples; keeping offset "
                f"{self._imu_north_offset:.1f}°"
            )
            return

        gps_bearing = calculate_bearing(a.lat, a.lon, b.lat, b.lon)
        new_offset = normalize_angle(gps_bearing + b.imu_yaw)
        delta = normalize_angle(new_offset - self._imu_north_offset)

        if abs(delta) > self.RECAL_MAX_DELTA_DEG:
            logger.warning(
                f"IMU recal [endpoint]: proposed shift {delta:+.1f}° "
                f"exceeds {self.RECAL_MAX_DELTA_DEG:.0f}° guardrail "
                f"(bsl={baseline:.2f}m, n={len(candidates)}); "
                f"keeping offset {self._imu_north_offset:.1f}°"
            )
            return

        old = self._imu_north_offset
        self._imu_north_offset = new_offset
        log_banner(
            f"IMU RECAL [endpoint] | {old:.1f}° → {new_offset:.1f}° "
            f"(Δ {delta:+.1f}°) | n={len(candidates)} bsl={baseline:.2f}m",
            char="-",
            logger=logger,
        )

    def get_calibrated_heading(self) -> float | None:
        """Convert the robot's IMU yaw to a true heading (0=north, 90=east)."""
        imu_yaw = self.robot.get_yaw_degrees()
        if imu_yaw is None or self._imu_north_offset is None:
            return None
        return (-imu_yaw + self._imu_north_offset) % 360

    async def turn_to_bearing(
        self,
        target_bearing_deg: float,
        tolerance_deg: float = 2.0,
        timeout: float = 15.0,
        kp: float = 0.04,
        min_rate_rad_s: float = 0.45,
    ) -> bool:
        """Rotate in place to face `target_bearing_deg` (true-north).

        P-controller: rate = clamp(|kp * error|, min_rate_rad_s,
        self.rotation_rate). Decelerates as it approaches target so the
        stop command lands before overshoot — enables sub-2° alignment
        without the hunting that pure bang-bang produces at the same
        tolerance. `min_rate_rad_s` floors the command above the motor
        stiction threshold; a pure P-controller would stall sub-threshold
        near zero error and never converge.

        Uses `get_calibrated_heading()` as the heading source —
        requires the IMU-to-GPS offset to have been established by a
        prior `_calibrate_imu()` call (done during the first waypoint
        approach). Returns True when aligned within `tolerance_deg`,
        False on timeout or if heading is unavailable.

        Intended use: capture strategies that need the robot facing a
        specific absolute bearing before snapping a frame.
        """
        if self._imu_north_offset is None:
            logger.error(
                "turn_to_bearing: IMU offset not yet calibrated; "
                "run at least one nav leg before calling this."
            )
            return False

        target = target_bearing_deg % 360.0
        t_start = time.time()
        last_log = 0.0

        await self.robot.balance_stand()
        await asyncio.sleep(0.5)

        while self._running:
            if time.time() - t_start > timeout:
                logger.error(
                    f"turn_to_bearing({target:.1f}°) timeout after {timeout:.1f}s"
                )
                await self.robot.stop()
                return False

            heading = self.get_calibrated_heading()
            if heading is None:
                await asyncio.sleep(0.1)
                continue

            error = normalize_angle(target - heading)

            if abs(error) <= tolerance_deg:
                await self.robot.stop()
                logger.info(
                    f"Aligned to {target:.1f}° (heading={heading:.1f}°, "
                    f"err={error:.1f}°)"
                )
                await self.robot.balance_stand()
                await asyncio.sleep(0.5)
                return True

            # P-controller with dead-band floor. The min-rate clamp is
            # critical: below the Go2's motor stiction threshold the
            # body doesn't actually rotate, so a pure P controller
            # would stall on the last few degrees and never converge.
            rate_mag = min(self.rotation_rate, max(abs(kp * error), min_rate_rad_s))
            # Same sign convention as navigate_to(): positive error →
            # need to rotate CW in the GPS frame, which is CCW-negative
            # on the Go2 IMU. direction = -1 for CCW (positive z-rate).
            direction = -1.0 if error > 0 else 1.0

            now = time.time()
            if now - last_log >= 1.0:
                logger.info(
                    f"[turn→{target:.0f}°] hdg={heading:.1f}° err={error:.1f}° "
                    f"rate={rate_mag:.2f}rad/s"
                )
                last_log = now

            await self.robot.send_velocity(z=direction * rate_mag)
            await asyncio.sleep(0.1)

        await self.robot.stop()
        return False

    def _compute_velocity(
        self, distance: float, heading_error: float
    ) -> tuple[float, float]:
        """Compute (forward, rotational) velocities for the walk phase."""
        vx = min(self.max_velocity, distance * 0.5)
        vx = max(0.2, vx)

        vz = heading_error * -0.015
        vz = max(-self.rotation_rate, min(self.rotation_rate, vz))

        return vx, vz

    def stop(self) -> None:
        """Signal navigation to stop at the next control tick."""
        self._running = False
