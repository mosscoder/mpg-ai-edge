"""Waypoint navigator: state machine (calibrate -> turn -> walk)."""

import asyncio
import logging
import math
import statistics
import time
from collections import deque
from typing import NamedTuple

from go2_survey.geometry import calculate_bearing, haversine_distance, normalize_angle
from go2_survey.gps import GPSManager, RTKPosition
from go2_survey.logging_utils import log_banner
from go2_survey.robot import Go2Robot
from go2_survey.waypoints import Waypoint

logger = logging.getLogger(__name__)

# COG samples below this speed have too much per-sample angular noise to
# contribute usefully to the inverse-variance-weighted circular mean.
# At 0.2 m/s with 100ms tick and ~2cm GPS noise, single-sample bearing
# uncertainty is already ~atan(0.02/0.02) ≈ 45° — saturating the noise
# model. Above 0.2 m/s noise drops below ~30° per sample, which the
# weighting + averaging can usefully reduce.
COG_FUSION_MIN_SPEED_M_S = 0.2


class _TrajSample(NamedTuple):
    """One (time, position, IMU yaw, GPS COG, GPS speed) sample collected
    while walking. `cog`/`speed` may be None — endpoint recompute only
    uses the position fields, so older code paths remain valid; cog_fusion
    requires both.
    """

    t: float
    lat: float
    lon: float
    imu_yaw: float
    cog: float | None = None
    speed: float | None = None


class WaypointNavigator:
    """Navigation controller for waypoint following.

    Three-phase state machine per waypoint:
      1. Calibrate IMU (walk forward until GPS-bearing-vs-IMU-yaw offset is known)
      2. Turn to face waypoint (committed direction, no oscillation)
      3. Walk to waypoint (proportional steering)

    The first-leg IMU calibration walk seeds `_imu_north_offset`. With
    `imu_recalibrate_on_arrival = True` (default), the offset is then
    refreshed at every waypoint arrival from the GPS+IMU samples
    collected during that leg's walk — see `_recalibrate_from_buffer`.
    """

    # Per-waypoint recalibration tunables. Constants rather than init
    # params: only the on/off switch is exposed via config.
    RECAL_MIN_SAMPLES = 10
    RECAL_MIN_BASELINE_M = 1.5
    RECAL_MAX_DELTA_DEG = 30.0
    RECAL_DROP_RECENT_SEC = 1.0      # ignore samples from arrival deceleration
    RECAL_STRAIGHT_VZ_THRESHOLD = 0.05  # rad/s — only buffer when going straight
    RECAL_BUFFER_MAXLEN = 60         # ~12s at 5Hz

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
        bearing_method: str = "endpoint",
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
        if bearing_method not in ("endpoint", "cog_fusion"):
            raise ValueError(
                f"bearing_method must be 'endpoint' or 'cog_fusion', "
                f"got {bearing_method!r}"
            )
        self.bearing_method = bearing_method

        self._running = False
        self._paused = False
        self._pause_start: float | None = None
        self._pause_last_progress: float | None = None
        self._nav_last_status: float | None = None

        self._imu_north_offset: float | None = None
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

    async def navigate_through(
        self,
        waypoints: list[Waypoint],
        on_capture_cb,
        cruise_speed: float,
        valley_speed: float,
        valley_radius_m: float,
        sharp_turn_deg: float,
        timeout_per_wp: float = 300.0,
    ) -> bool:
        """Continuous-motion drive-by route through all waypoints.

        Velocity profile per leg:
          * distance to current target > valley_radius_m: cruise_speed
          * within valley_radius_m: linear ramp toward valley_speed (V-shape)
          * at arrival: fire on_capture_cb(wp, position, dist, vx)

        At each capture, the heading error to the *next* waypoint is
        measured. If it exceeds sharp_turn_deg, the dog brakes to a
        stop, calls `turn_to_bearing` to rotate in place, then resumes
        cruise on the next leg. Otherwise it continues smoothly with
        no pause.

        Returns True on success after the final waypoint's capture.
        Returns False on GPS-loss timeout, IMU-calibration timeout, or
        per-waypoint navigation timeout.
        """
        if not waypoints:
            return True

        log_banner(
            f"DRIVE-BY ROUTE | {len(waypoints)} wp(s) | "
            f"cruise={cruise_speed:.2f}m/s valley={valley_speed:.2f}m/s "
            f"radius={valley_radius_m:.1f}m sharp_turn>{sharp_turn_deg:.0f}°",
            logger=logger,
        )

        self._running = True
        await self.robot.balance_stand()
        await asyncio.sleep(1.0)

        # Initial IMU calibration walk if not yet calibrated.
        if self._imu_north_offset is None:
            log_banner("CALIBRATING IMU (drive-by)", char="-", logger=logger)
            cal_start = time.time()
            while self._running:
                if time.time() - cal_start > timeout_per_wp:
                    logger.error("Timeout during initial IMU calibration")
                    await self.robot.stop()
                    return False
                pos = self.gps.get_position()
                if not pos:
                    await asyncio.sleep(0.2)
                    continue
                imu_yaw = self.robot.get_yaw_degrees()
                if imu_yaw is not None:
                    self._maybe_buffer_sample(pos, imu_yaw, vz=0.0)
                    if self._calibrate_imu(pos, imu_yaw):
                        # Don't stop — flow straight into the route. The
                        # cal walk happens while heading toward wp1, so
                        # the dog is already in roughly the right direction.
                        break
                await self.robot.send_velocity(x=cruise_speed)
                await asyncio.sleep(0.2)

        n_captured = 0
        for i, wp in enumerate(waypoints):
            is_last = i == len(waypoints) - 1
            next_wp = None if is_last else waypoints[i + 1]
            log_banner(
                f"DRIVE-BY WP {i + 1}/{len(waypoints)}: {wp.name}",
                char="-",
                logger=logger,
            )

            self._traj_buf.clear()
            leg_start = time.time()
            prev_distance = float("inf")
            captured = False
            last_status = 0.0

            while self._running:
                if time.time() - leg_start > timeout_per_wp:
                    logger.error(
                        f"Drive-by timeout on leg to {wp.name} "
                        f"after {timeout_per_wp:.0f}s"
                    )
                    await self.robot.stop()
                    return False

                pos = self.gps.get_position()
                corrections_active = self.gps.has_active_corrections()
                if not self._is_quality_acceptable(pos, corrections_active):
                    # Drive-by has no graceful "pause and wait" — if GPS
                    # degrades mid-route we stop and let the existing
                    # nav-style fix-lost loop handle it.
                    fix_info, cause = self._fix_lost_diagnostics(
                        pos, corrections_active
                    )
                    logger.warning(
                        f"GPS lost mid-drive-by ({fix_info}); pausing | {cause}"
                    )
                    await self.robot.stop()
                    if not await self._wait_for_fix_resume():
                        return False
                    leg_start = time.time()  # reset leg timeout after fix
                    continue

                distance = haversine_distance(
                    pos.latitude, pos.longitude,
                    wp.latitude, wp.longitude,
                )

                # Capture trigger: inside arrival_tolerance OR distance
                # just turned around (closest-pass detection). The
                # closest-pass clause is gated on prev_distance being
                # "near" the waypoint so we don't trigger on noise far
                # from the target.
                near_radius = max(2 * valley_radius_m, 0.5)
                closest_pass = (
                    distance > prev_distance and prev_distance < near_radius
                )
                if not captured and (
                    distance < self.arrival_tolerance or closest_pass
                ):
                    trigger_speed = self._drive_by_speed(
                        prev_distance, cruise_speed, valley_speed, valley_radius_m
                    )
                    log_banner(
                        f"CAPTURE @ {wp.name} | drive_by | "
                        f"d={prev_distance:.2f}m v_cmd={trigger_speed:.2f}m/s",
                        char="-",
                        logger=logger,
                    )
                    n_written = await on_capture_cb(
                        wp, pos, prev_distance, trigger_speed
                    )
                    n_captured += n_written
                    captured = True
                    self._recalibrate_from_buffer()

                    if is_last:
                        await self.robot.stop()
                        await self.robot.balance_stand()
                        log_banner(
                            f"DRIVE-BY ROUTE COMPLETE | {n_captured} frame(s)",
                            logger=logger,
                        )
                        return True

                    # Sharp-turn check toward next waypoint.
                    next_bearing = calculate_bearing(
                        pos.latitude, pos.longitude,
                        next_wp.latitude, next_wp.longitude,
                    )
                    heading_now = self.get_calibrated_heading()
                    if heading_now is not None:
                        turn_error = abs(
                            normalize_angle(next_bearing - heading_now)
                        )
                        if turn_error > sharp_turn_deg:
                            log_banner(
                                f"SHARP TURN | {turn_error:.0f}° to next wp | "
                                f"pausing to rotate",
                                char="-",
                                logger=logger,
                            )
                            await self.robot.stop()
                            await self.turn_to_bearing(
                                next_bearing,
                                tolerance_deg=10.0,
                                timeout=15.0,
                            )
                    break  # advance to next waypoint

                # Speed + heading control.
                desired_speed = self._drive_by_speed(
                    distance, cruise_speed, valley_speed, valley_radius_m
                )
                bearing = calculate_bearing(
                    pos.latitude, pos.longitude,
                    wp.latitude, wp.longitude,
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
                    self._maybe_buffer_sample(pos, imu_yaw_now, vz=vz)

                await self.robot.send_velocity(x=desired_speed, z=vz)

                now = time.time()
                if now - last_status >= 2.0:
                    logger.info(
                        f"NAV [drive_by] | d={distance:.2f}m v={desired_speed:.2f}m/s "
                        f"hdg_err={heading_error:.1f}° fix={pos.fix_type}"
                    )
                    last_status = now

                prev_distance = distance
                await asyncio.sleep(0.1)

        await self.robot.stop()
        return True

    @staticmethod
    def _drive_by_speed(
        distance: float,
        cruise: float,
        valley: float,
        radius: float,
    ) -> float:
        """Linear V-shape: cruise outside the valley, ramp to valley speed
        at the waypoint. Symmetric on both sides since `distance` is
        always non-negative — the dog approaches the waypoint from one
        side, fires, and the *next* waypoint's distance takes over.
        """
        if distance >= radius:
            return cruise
        # d ∈ [0, radius]: linear from valley (at d=0) to cruise (at d=radius)
        return valley + (cruise - valley) * (distance / radius)

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

        # Phase 1: Calibrate IMU (once per navigator)
        if self._imu_north_offset is None:
            log_banner("CALIBRATING IMU", char="-", logger=logger)
            while self._running:
                if time.time() - nav_start > timeout:
                    logger.error(
                        f"Timeout during IMU calibration for {waypoint.name}"
                    )
                    log_banner(
                        f"CALIBRATION TIMEOUT | {waypoint.name}",
                        level="error",
                        char="!",
                        logger=logger,
                    )
                    await self.robot.stop()
                    return False

                pos = self.gps.get_position()
                if not pos:
                    await asyncio.sleep(0.2)
                    continue

                imu_yaw = self.robot.get_yaw_degrees()
                if imu_yaw is not None:
                    # Cal walk is pure forward motion (vz=0) — these are
                    # the cleanest samples in the whole leg for the
                    # buffer recal at arrival.
                    self._maybe_buffer_sample(pos, imu_yaw, vz=0.0)
                    if self._calibrate_imu(pos, imu_yaw):
                        await self.robot.stop()
                        await self.robot.balance_stand()
                        await asyncio.sleep(1.0)
                        break

                await self.robot.send_velocity(x=self.max_velocity)
                await asyncio.sleep(0.2)

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
                f"(displacement: {displacement:.2f}m / 1.50m needed). "
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
                f"IMU calibrating... displacement: {displacement:.2f}m / 1.50m needed "
                f"hAcc: {pos.accuracy_horizontal:.3f}m ({elapsed:.0f}s)"
            )
            self._calibration_last_progress = now

        if (
            displacement >= 1.5
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

    def _maybe_buffer_sample(
        self, pos: RTKPosition, imu_yaw: float, vz: float
    ) -> None:
        """Append a trajectory sample if it passes quality + straight-line filters.

        Quality gate matches the navigation gate (`min_fix_type`,
        `max_hacc`) so degraded GPS samples never feed the recal.
        Straight-line gate (`|vz| <= RECAL_STRAIGHT_VZ_THRESHOLD`) keeps
        only samples taken during near-zero commanded rotation, so the
        chord-vs-curve geometric error in `calculate_bearing(A, B)`
        stays small.
        """
        if pos.fix_type < self.min_fix_type:
            return
        if pos.accuracy_horizontal > self.max_hacc:
            return
        if abs(vz) > self.RECAL_STRAIGHT_VZ_THRESHOLD:
            return
        self._traj_buf.append(
            _TrajSample(
                t=time.time(),
                lat=pos.latitude,
                lon=pos.longitude,
                imu_yaw=imu_yaw,
                cog=pos.course_over_ground,
                speed=pos.speed_over_ground,
            )
        )

    def _recalibrate_from_buffer(self) -> None:
        """Dispatch to the configured per-arrival recompute algorithm.

        Both backends share the early-skip guards (feature disabled,
        first-leg cal not yet complete) before branching. See
        `_recalibrate_endpoint` and `_recalibrate_cog_fusion` for the
        per-algorithm logic and log-banner shape.
        """
        if not self.imu_recalibrate_on_arrival:
            return
        if self._imu_north_offset is None:
            # First-leg cal walk hasn't completed; nothing to refine.
            return

        if self.bearing_method == "cog_fusion":
            self._recalibrate_cog_fusion()
        else:
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

    def _recalibrate_cog_fusion(self) -> None:
        """Refresh `_imu_north_offset` via inverse-variance-weighted
        circular mean of GPS course-over-ground samples.

        Each per-tick COG sample has angular noise σ_θ ≈ σ_pos / (v·Δt),
        so variance scales as 1/v². Optimal inverse-variance weighting
        therefore weights each sample by v². The vector-sum form of the
        circular mean (`atan2(Σw·sin, Σw·cos)`) handles the 0°/360°
        wrap-around correctly.

        Falls back to the same `RECAL_MAX_DELTA_DEG` guardrail as the
        endpoint method. Skips with a warning if:
          - fewer than `RECAL_MIN_SAMPLES` qualify (after speed gate)
          - the weighted vector sum is degenerate (samples cancel out)
          - proposed offset shift exceeds the delta guardrail

        Note: the per-sample speed/cog gate (`s.speed > MIN_SPEED`,
        `s.cog is not None`) is on top of the `_maybe_buffer_sample`
        gating that already requires `min_fix_type`, `max_hacc`, and
        small `|vz|`. The remaining samples are good-quality, nearly-
        straight motion above the noise-floor speed.
        """
        qualifying = [
            s for s in self._traj_buf
            if s.cog is not None
            and s.speed is not None
            and s.speed > COG_FUSION_MIN_SPEED_M_S
        ]
        if len(qualifying) < self.RECAL_MIN_SAMPLES:
            logger.warning(
                f"IMU recal [cog_fusion]: only {len(qualifying)}/"
                f"{self.RECAL_MIN_SAMPLES} qualifying samples (need cog + "
                f"speed > {COG_FUSION_MIN_SPEED_M_S}m/s); keeping offset "
                f"{self._imu_north_offset:.1f}°"
            )
            return

        # Inverse-variance weighting (weight = v²) + vector-sum circular
        # mean. cog is in degrees; convert to radians for the trig.
        xs = sum(
            (s.speed ** 2) * math.cos(math.radians(s.cog))
            for s in qualifying
        )
        ys = sum(
            (s.speed ** 2) * math.sin(math.radians(s.cog))
            for s in qualifying
        )
        if xs * xs + ys * ys < 1e-9:
            logger.warning(
                f"IMU recal [cog_fusion]: degenerate vector sum "
                f"(n={len(qualifying)} samples cancelled out); keeping "
                f"offset {self._imu_north_offset:.1f}°"
            )
            return

        fused_bearing_deg = math.degrees(math.atan2(ys, xs)) % 360.0
        # Anchor to the most-recent qualifying sample's IMU yaw, mirroring
        # the endpoint method's use of B.imu_yaw.
        new_offset = normalize_angle(fused_bearing_deg + qualifying[-1].imu_yaw)
        delta = normalize_angle(new_offset - self._imu_north_offset)

        if abs(delta) > self.RECAL_MAX_DELTA_DEG:
            logger.warning(
                f"IMU recal [cog_fusion]: proposed shift {delta:+.1f}° "
                f"exceeds {self.RECAL_MAX_DELTA_DEG:.0f}° guardrail "
                f"(n={len(qualifying)}); keeping offset "
                f"{self._imu_north_offset:.1f}°"
            )
            return

        old = self._imu_north_offset
        self._imu_north_offset = new_offset
        avg_speed = statistics.mean(s.speed for s in qualifying)
        log_banner(
            f"IMU RECAL [cog_fusion] | {old:.1f}° → {new_offset:.1f}° "
            f"(Δ {delta:+.1f}°) | n={len(qualifying)} "
            f"avg_speed={avg_speed:.2f}m/s",
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
