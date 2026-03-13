# Navigation Changelog

## 2026-03-10: Fix Rotation Failure, Relax GPS Constraints, Add GPS Visibility

### Problem

First field test of `test_imu_calibration.py` failed. The robot walked forward (calibration succeeded), but Phase 3 rotation did nothing for 25s until techs killed it. Root causes:

1. **BalanceStand not re-issued** — after the calibration walk+stop, the gait controller was not re-primed, so `send_velocity(z=...)` had no effect
2. **Mode switch failure (code 7004)** — the switch to "normal" mode had failed silently
3. **TURN_RATE too low** — field data showed ~7% command-to-actual ratio at 0.3 rad/s
4. **GPS constraints too tight** — requiring RTK Float (fix_type 5) caused unnecessary stalls; GNSS+DR (fix_type 4) with hAcc gating is sufficient
5. **GPS status logging insufficient** — no visibility into hAcc, fix_type, or position during calibration walk or rotation phases
6. **No overall Phase 2 timeout** — the calibration walk loop had no exit condition other than success; inner `_calibrate_imu` 30s timeout just reset and retried, so the robot could walk forward indefinitely
7. **Navigator config inconsistent** — `min_fix_type` and `max_hacc` not passed to the `WaypointNavigator` constructor in the calibration script

### Changes

**`nav_utils.py` — WaypointNavigator:**
- Added `max_hacc` parameter (default 1.0m) to `__init__()`
- `navigate_to()` now issues `balance_stand()` + 1s sleep before the nav loop, ensuring the gait controller is ready for every waypoint (including between waypoints in multi-waypoint missions)
- GPS degradation check now also pauses when `hAcc > max_hacc`, with hAcc included in the log message
- Added periodic INFO-level nav status every 5s: position, hAcc, fix_type, distance to target, heading, calibration status

**`test_imu_calibration.py`:**
- `MIN_FIX_TYPE`: 5 → 4 (GNSS+DR or better)
- Added `MAX_HACC = 1.0` — skip noisy positions during calibration (don't feed to `_calibrate_imu`)
- `TURN_RATE`: 0.3 → 0.8 rad/s
- Added `TURN_TIMEOUT = 60.0` — abort rotation if not aligned within this time
- Added `CALIBRATION_TIMEOUT = 90.0` — overall Phase 2 timeout; stops robot and aborts if calibration doesn't complete (including inner retries)
- Phase 2: added GPS logging every 2s (position, hAcc, fix_type, displacement)
- Phase 3: re-issue `balance_stand()` before rotation loop; added 2s periodic logging (heading, error, GPS status)
- Phase 4: re-issue `balance_stand()` before northward walk
- Navigator constructor now passes `min_fix_type=MIN_FIX_TYPE` and `max_hacc=MAX_HACC` for consistency

**`mission_00.py` and `mission_01.py`:**
- `MIN_FIX_TYPE`: 5 → 4
- All nav_utils improvements (BalanceStand per waypoint, hAcc gating, INFO logging) flow through automatically

### Verification

- `python -m py_compile` passes for all four files
- No API changes visible to mission scripts — all new parameters have defaults

---

## 2026-02-12: Human-Readable Mission Epoch Logging

### Problem

Log output is a wall of text — major state transitions (GPS fix achieved, IMU calibration complete, arrival at waypoint) are visually indistinguishable from routine 5 Hz telemetry lines. During long waits (GPS fix, GPS pause, IMU calibration), logs go silent for minutes with no indication of progress.

### Solution: Banner Logging + Progress Messages

A `_log_banner()` helper produces 3-line bordered banners at every major state transition, making them easy to spot in scrolling output or log files:

```
============================================================
========= GPS RTK Fixed ACHIEVED | hAcc: 0.014m ============
============================================================
```

Error/warning banners use `!!!` borders:

```
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!! GPS FIX TIMEOUT after 300s !!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
```

### Banner Locations

| Event | Level | Border |
|-------|-------|--------|
| GPS fix achieved | info | `===` |
| GPS fix timeout | error | `!!!` |
| IMU calibration started | info | `---` |
| IMU calibrated (offset computed) | info | `===` |
| IMU calibration timeout | error | `!!!` |
| Navigation started (per waypoint) | info | `===` |
| Arrived at waypoint | info | `===` |
| Navigation timeout | error | `!!!` |
| GPS fix lost (pausing) | warning | `!!!` |
| GPS fix restored (resuming) | info | `===` |
| GPS pause timeout | error | `!!!` |

### Progress Messages During Waits

- **GPS fix wait:** Status every 15 s with elapsed time, current fix type, satellite count
- **GPS pause (fix lost):** Status every 15 s with elapsed time and timeout limit
- **IMU calibration:** Status every 5 s with displacement progress (e.g. `0.73m / 1.50m needed`)

### Enhanced Failure Messages

- Navigation timeout includes remaining distance to waypoint
- GPS pause timeout includes actual elapsed time and which waypoint was targeted
- IMU calibration timeout includes displacement achieved vs. required

### Verification

- `python -m py_compile autonomous_nav/nav_utils.py` passes
- No function signatures changed — existing mission scripts run unmodified

---

## 2026-02-12: IMU Robustness and Timeout Failure Modes

### Problem

Several open-ended waits could hang forever if conditions didn't improve:

1. **No navigation timeout** — if the robot couldn't reach a waypoint (obstacle, drift), `navigate_to()` would run indefinitely
2. **No GPS pause timeout** — if GPS fix was lost mid-navigation, the robot would wait forever for restoration
3. **No IMU calibration timeout** — if the robot was blocked and couldn't walk 1.5 m, calibration would never complete
4. **Stale IMU data undetected** — if the sport-mode state topic stopped publishing, the last IMU reading would be used silently

### Solution

**IMU staleness detection:**
- `Go2Robot._imu_timestamp` tracks when the last IMU reading arrived
- `get_yaw_degrees(max_age=1.0)` returns `None` if data is older than `max_age` seconds
- When heading is stale and calibration was previously complete, a warning is logged and the robot falls back to forward motion

**Navigation timeout:**
- `navigate_to()` accepts a `timeout` parameter (default 300 s)
- If the waypoint isn't reached within `timeout` seconds, navigation aborts with an error

**GPS pause timeout:**
- `WaypointNavigator.__init__()` accepts `gps_timeout` (default 300 s)
- If GPS fix isn't restored within `gps_timeout` seconds after loss, navigation aborts

**IMU calibration timeout:**
- `WaypointNavigator.__init__()` accepts `calibration_timeout` (default 30 s)
- If displacement doesn't reach 1.5 m within the timeout, calibration state is reset and retried automatically

**Rotation rate clamping:**
- `_compute_velocity()` now clamps `vz` to `[-rotation_rate, rotation_rate]`, preventing runaway spinning from large proportional heading errors

### Code Changes

**New parameters:**
- `Go2Robot.get_yaw_degrees(max_age: float = 1.0)`
- `WaypointNavigator.__init__(..., gps_timeout=300.0, calibration_timeout=30.0)`
- `WaypointNavigator.navigate_to(waypoint, timeout=300.0)`

**New attributes:**
- `Go2Robot._imu_timestamp: float`
- `WaypointNavigator._pause_start: Optional[float]`
- `WaypointNavigator._calibration_start_time: Optional[float]`

### Verification

- No API changes visible to mission scripts — all new parameters have defaults
- `python -m py_compile autonomous_nav/nav_utils.py` passes

---

## 2026-01-29: IMU-Based Heading with GPS Calibration

### Problem

Position-based heading estimation had significant limitations:

1. **Requires 1.5m displacement** - Robot must walk ~5 seconds forward before knowing its heading
2. **Only works during forward motion** - Can't update heading during in-place rotation
3. **Delayed response** - Heading lags behind actual orientation
4. **Cold start problem** - After GPS fix restore, heading is unknown until robot moves
5. **State complexity** - Tracks `_prev_pos`, `_last_heading`, `_is_moving_forward`

### Solution: IMU Yaw with GPS Calibration

Use the Go2's onboard IMU for instant heading, calibrated against GPS bearing:

1. **Subscribe to sport mode state** - Get IMU data via `RTC_TOPIC['LF_SPORT_MOD_STATE']`
2. **One-time calibration** - Walk forward 1.5m, compute offset between IMU yaw and GPS bearing
3. **Instant heading** - After calibration, IMU provides true heading every loop iteration

### Sensor Fusion Strategy

| Sensor | Purpose |
|--------|---------|
| RTK GPS | Absolute position, distance/bearing to waypoint |
| IMU | Current heading (calibrated to true north) |

### Code Changes

**Removed (vestigial):**
- `_estimate_heading()` method
- `_prev_pos`, `_last_heading`, `_is_moving_forward` state variables
- Motion tracking logic
- Heading reset on GPS fix restore

**Added:**
- IMU subscription in `Go2Robot.connect()`
- `Go2Robot._on_sport_state()` callback for IMU data
- `Go2Robot.get_yaw_degrees()` method
- `WaypointNavigator._calibrate_imu()` for one-time GPS alignment
- `WaypointNavigator.get_calibrated_heading()` for true heading
- `_imu_north_offset`, `_calibration_start_pos`, `_calibration_start_yaw` state variables

### Benefits

- **Instant heading** - No warm-up walk after calibration
- **Works during rotation** - Know heading while turning in place
- **Simpler code** - ~40 lines of complex position-tracking logic removed
- **Better GPS recovery** - IMU heading survives GPS outages

### Verification

1. Robot calibrates during first ~1.5m of forward motion
2. Heading appears immediately in logs after calibration
3. Navigation completes faster than position-based approach

---

## 2026-01-29: Position-Based Heading Estimation

### Problem

GPS course-over-ground (COG) is unreliable at low walking speeds. During field testing on the tennis court, the robot exhibited erratic heading behavior because:

1. **COG requires significant velocity** - GPS COG is only accurate when moving >0.5 m/s, but the robot walks at ~0.3 m/s
2. **COG updates lag** - Even when valid, COG reflects past motion, not current orientation
3. **Stationary COG is meaningless** - When stopped or turning in place, COG provides no useful heading information

### Root Cause Analysis

From the 2026-01-29 test logs:
- Robot would frequently oscillate or spin because GPS COG was either `None` or stale
- After GPS fix recovery, robot would resume with old heading values, causing incorrect corrections
- Heading error calculations were based on unreliable COG data

### Solution: Position-Based Heading

Instead of relying on GPS COG, heading is now computed from actual position changes:

1. **Track previous position** - Store the last RTK position when heading was updated
2. **Compute displacement** - Calculate distance traveled using haversine formula
3. **Update heading on sufficient movement** - Only update heading after moving >0.1m forward
4. **Forward motion only** - Only update heading when `vx > 0` (walking forward), not during rotation

### Algorithm

```
if moving_forward AND displacement > 1.5m:
    heading = bearing(prev_pos, current_pos)
    prev_pos = current_pos
```

The 1.5m threshold ensures the displacement is well above GPS noise (RTK Float can have 10-50cm accuracy).

### Other Changes

- **Reset heading on fix restore** - After GPS outage, heading and prev_pos are reset to `None`, forcing the robot to walk forward briefly to re-establish heading
- **Enhanced logging** - Navigation loop now logs fix type and horizontal accuracy every iteration for easier debugging
- **Moved logs directory** - Logs now stored in `autonomous_nav/mission/logs/` (previously `autonomous_nav/mission_logs/`)

### Verification

1. Robot should walk forward initially when heading is unknown
2. After ~1.5m of forward movement (~5 seconds), heading stabilizes
3. Robot turns toward waypoint and navigates
4. Logs show consistent heading values during forward motion
