"""Waypoint navigator: state machine (calibrate -> turn -> walk)."""

import asyncio
import logging
import time
from collections import deque
from typing import NamedTuple

from go2_survey.geometry import calculate_bearing, haversine_distance, normalize_angle
from go2_survey.gps import GPSManager, RTKPosition
from go2_survey.logging_utils import log_banner
from go2_survey.robot import Go2Robot
from go2_survey.waypoints import Waypoint

logger = logging.getLogger(__name__)


class _TrajSample(NamedTuple):
    """One (time, position, IMU yaw) sample collected while walking."""

    t: float
    lat: float
    lon: float
    imu_yaw: float


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
        self._calibration_start_pos: RTKPosition | None = None
        self._calibration_start_time: float | None = None
        self._calibration_last_progress: float | None = None

        # Trajectory buffer feeding _recalibrate_from_buffer at arrival.
        # Cleared at the start of every navigate_to() so leg N's recal
        # uses only leg N's samples.
        self._traj_buf: deque = deque(maxlen=self.RECAL_BUFFER_MAXLEN)

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

            if (
                not pos
                or pos.fix_type < self.min_fix_type
                or pos.accuracy_horizontal > self.max_hacc
            ):
                if not self._paused:
                    fix_info = (
                        f"type {pos.fix_type}, hAcc {pos.accuracy_horizontal:.3f}m"
                        if pos
                        else "no position"
                    )
                    logger.warning(
                        f"GPS fix lost or degraded ({fix_info}), pausing robot..."
                    )
                    log_banner(
                        f"GPS FIX LOST | {fix_info} | robot paused",
                        level="warning",
                        char="!",
                        logger=logger,
                    )
                    await self.robot.stop()
                    self._paused = True
                    self._pause_start = time.time()
                    self._pause_last_progress = time.time()
                elif time.time() - self._pause_start > self.gps_timeout:
                    pause_elapsed = time.time() - self._pause_start
                    logger.error(
                        f"GPS fix not restored after {pause_elapsed:.0f}s "
                        f"(limit: {self.gps_timeout:.0f}s), "
                        f"aborting navigation to {waypoint.name}"
                    )
                    log_banner(
                        f"GPS PAUSE TIMEOUT | {pause_elapsed:.0f}s | aborting",
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
                            f"({pause_elapsed:.0f}s / {self.gps_timeout:.0f}s timeout)"
                        )
                        self._pause_last_progress = now
                await asyncio.sleep(0.5)
                continue

            if self._paused:
                pause_duration = time.time() - self._pause_start
                logger.info(
                    f"GPS fix restored (type {pos.fix_type}), resuming navigation"
                )
                log_banner(
                    f"GPS FIX RESTORED | type {pos.fix_type} "
                    f"| paused {pause_duration:.0f}s",
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
            )
        )

    def _recalibrate_from_buffer(self) -> None:
        """Refresh `_imu_north_offset` at arrival from buffered samples.

        Picks the longest available straight-line baseline in the buffer:
        A = earliest sample, B = most recent sample (after dropping the
        last `RECAL_DROP_RECENT_SEC` of arrival deceleration noise).
        Computes `gps_bearing = bearing(A → B)` and sets
        `new_offset = normalize_angle(gps_bearing + B.imu_yaw)` —
        same sign convention as the first-leg cal walk in
        `_calibrate_imu`.

        Skips with a warning if any of:
          - feature disabled in config
          - too few samples ever buffered
          - too few samples remain after dropping the deceleration zone
          - resulting baseline shorter than `RECAL_MIN_BASELINE_M`
          - proposed offset shift exceeds `RECAL_MAX_DELTA_DEG` (likely
            a corrupt sample, not real drift in a single leg)
        """
        if not self.imu_recalibrate_on_arrival:
            return

        if self._imu_north_offset is None:
            # First leg's cal walk hasn't completed; nothing to refine.
            return

        n = len(self._traj_buf)
        if n < self.RECAL_MIN_SAMPLES:
            logger.warning(
                f"IMU recal: only {n}/{self.RECAL_MIN_SAMPLES} samples in "
                f"buffer; keeping offset {self._imu_north_offset:.1f}°"
            )
            return

        cutoff = time.time() - self.RECAL_DROP_RECENT_SEC
        candidates = [s for s in self._traj_buf if s.t <= cutoff]
        if len(candidates) < 2:
            logger.warning(
                f"IMU recal: only {len(candidates)} samples remain after "
                f"dropping last {self.RECAL_DROP_RECENT_SEC:.1f}s; "
                f"keeping offset {self._imu_north_offset:.1f}°"
            )
            return

        a = candidates[0]
        b = candidates[-1]
        baseline = haversine_distance(a.lat, a.lon, b.lat, b.lon)
        if baseline < self.RECAL_MIN_BASELINE_M:
            logger.warning(
                f"IMU recal: baseline {baseline:.2f}m < "
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
                f"IMU recal: proposed shift {delta:+.1f}° exceeds "
                f"{self.RECAL_MAX_DELTA_DEG:.0f}° guardrail "
                f"(bsl={baseline:.2f}m, n={len(candidates)}); "
                f"keeping offset {self._imu_north_offset:.1f}°"
            )
            return

        old = self._imu_north_offset
        self._imu_north_offset = new_offset
        log_banner(
            f"IMU RECAL | {old:.1f}° → {new_offset:.1f}° "
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
        tolerance_deg: float = 5.0,
        timeout: float = 15.0,
    ) -> bool:
        """Rotate in place to face `target_bearing_deg` (true-north).

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

            now = time.time()
            if now - last_log >= 1.0:
                logger.info(
                    f"[turn→{target:.0f}°] hdg={heading:.1f}° err={error:.1f}°"
                )
                last_log = now

            if abs(error) <= tolerance_deg:
                await self.robot.stop()
                logger.info(
                    f"Aligned to {target:.1f}° (heading={heading:.1f}°, "
                    f"err={error:.1f}°)"
                )
                await self.robot.balance_stand()
                await asyncio.sleep(0.5)
                return True

            # Same sign convention as navigate_to(): positive error →
            # need to rotate CW in the GPS frame, which is CCW-negative
            # on the Go2 IMU. direction = -1 for CCW (positive z-rate).
            direction = -1.0 if error > 0 else 1.0
            await self.robot.send_velocity(z=direction * self.rotation_rate)
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
