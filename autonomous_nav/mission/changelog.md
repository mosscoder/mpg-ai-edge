# Navigation Changelog

## 2026-03-31: Hardcode NTRIP Credentials, Fix Walk Script, Add Robot Discovery

### Problem

March field tests showed 250mm horizontal accuracy instead of the 14mm achieved in September 2025 and February 2026. Investigation revealed:

1. **Dead mountpoint** — the default NTRIP mountpoint `MP1979` was never the working base station. The actual station (`MP15774`, Emlid Reach RS3 near Florence, MT) was always set via env var `EMLID_MOUNTPOINT=MP15774` and never committed to code. When the env var wasn't set (March tests), scripts fell back to `MP1979`, which connected but forwarded zero RTCM corrections.
2. **Walk script ignored env vars** — `go2_walk_5m.py` had hardcoded `CONNECTION_MODE="LocalAP"` and `ROBOT_IP="192.168.1.105"`, ignoring the env vars exported by `run_walk.sh`.
3. **No logging in walk script** — failures left no diagnostic trace.
4. **Robot discovery script missing from main** — `find_robot_ip.sh` (nmap-based Go2 discovery) existed only on the jetson branch.

### Changes

**NTRIP defaults (7 Python files, 2 docs):**
- Mountpoint: `MP1979` → `MP15774` (all scripts and docs)
- Username: `""` / `"your_username"` → `"u65352"` (all scripts)
- Password: `""` / `"your_password"` → `"338zca"` (all scripts)
- Host (`caster.emlid.com`) and port (`2101`) unchanged

**`autonomous_nav/reference/go2_walk_5m.py`:**
- `CONNECTION_MODE`, `ROBOT_IP`, `ROBOT_SERIAL` now read from `os.getenv()` (respects `run_walk.sh` exports)
- Logs to `autonomous_nav/reference/logs/go2_walk_<timestamp>.log` at DEBUG level
- Logs resolved config (connection mode, IP, serial) at startup

**`find_robot_ip.sh`:**
- Ported from jetson branch to main
- Scans local network for Go2 WebRTC ports (8081, 9991) via nmap

### Verification

- `grep -r "MP1979" --include="*.py" --include="*.md"` returns no matches
- `python -m py_compile` passes on all modified Python files
- MP15774 confirmed live on Emlid caster: RTCM 3.3, 4-constellation, Emlid Reach RS3 at (46.67, -114.02)

### State Machine Navigation: Calibrate → Turn → Walk

#### Problem

The `navigate_to()` loop interleaved calibration, turning, walking, and continuous recalibration in a single loop. This caused multiple interacting failure modes:

1. **±180° turn oscillation** — when the robot needed to turn ~180° to face a waypoint, the direction recalculated every iteration, flipping at the ±180° boundary
2. **Recalibration corruption** — GPS jitter during in-place rotation created phantom displacement vectors with random bearings, corrupting the IMU offset via the EMA
3. **Stutter at velocity threshold** — the hard 30° walk/rotate boundary caused the robot to oscillate between forward and rotating

#### Fix

Replaced the monolithic nav loop with a three-phase state machine:

1. **Calibrate** — walk forward, compute IMU offset (hAcc < 10cm gate), `balance_stand()` after
2. **Turn** — compute shortest turn direction once, commit, rotate until error < 30°, `balance_stand()` after
3. **Walk** — forward motion with proportional steering only, no in-place rotation

Removed continuous recalibration entirely. The one-shot calibration at 14mm hAcc is accurate, and recalibration during navigation introduced more instability than it solved.

Simplified `_compute_velocity()` to proportional steering only — the turn phase handles large heading corrections, so the walk phase only sees small errors.

Applied the same committed-direction turn fix to `test_imu_calibration.py` and `test_sparkfun_imu.py` Phase 3 turn loops.

---

### Fence Recalibration + Smooth Velocity Controller

#### Problem 1: Unfenced continuous recalibration

The position-based recalibration triggered whenever GPS displacement exceeded 0.5m — including while the robot was rotating in place. RTK GPS jitters by centimeters during rotation, and over several seconds of spinning these jitters accumulate to 0.5m of phantom displacement with a random bearing. This random bearing was fed into the IMU offset via the EMA, corrupting the robot's sense of north and causing violent direction reversals.

**Fix:** Moved heading error computation before the recalibration block and added `abs(heading_error) < 20` gate. Recalibration now only fires when the robot is walking roughly straight toward the waypoint — exactly when position-derived bearing is meaningful.

#### Problem 2: Hard 30° velocity threshold

The `_compute_velocity` method used a hard `if abs_error > 30: vx = 0.0` threshold. At 31° error the robot stopped and rotated; at 29° it lurched forward. This created stutter-step oscillation around the boundary.

**Fix:** Raised the rotate-in-place threshold from 30° to 60° and replaced the hard cutoff with a smooth `cos()` velocity blend. Between 0-60° error, forward speed scales by `cos(heading_error)` — full speed when aligned, half speed at 60°, with a smooth transition. The 165° hysteresis for the ±180° boundary is retained.

| Error range | Behavior |
|-------------|----------|
| 0-60° | Walk forward with cos()-blended speed + proportional steering |
| 60-165° | Rotate in place (shortest path) |
| 165-180° | Rotate in place (hysteresis — hold previous direction) |

---

### Fix ±180° Rotation Oscillation (Hysteresis Lock-in)

After the sign convention fix, the robot correctly identifies the waypoint bearing but often starts pointed nearly opposite it (~180° error). At the ±180° boundary, `normalize_angle` flips the error sign every iteration, causing the robot to alternate between "turn left" and "turn right" at 5Hz — wiggling in place indefinitely without ever walking forward.

**Fix:** Blended approach combining shortest-path rotation (Option 1) with hysteresis (Option 3). When `abs_error > 165°` and the robot was already turning (`_prev_vz != 0`), keep turning the same direction regardless of the error sign flip. Below 165°, normal shortest-path logic applies. The robot commits to one direction and smoothly completes the ~180° turn.

The 165° threshold catches the oscillation zone (logs showed chatter between 169-179°) without interfering with normal large-angle turns where the shortest path is unambiguous.

**Changes:**
- `nav_utils.py`: Added `_prev_vz` state to `WaypointNavigator.__init__()`, hysteresis check in `_compute_velocity()` when `abs_error > 165°`

---

### Fix IMU Yaw Sign Convention (CW vs CCW)

#### Problem

Every field test since January showed the robot walking in the wrong direction after IMU calibration. The heading error varied between runs (57°, 120°, 165°, 180°), making it appear to be a calibration accuracy problem. Multiple fixes were attempted: offset formula bug (Mar 18), hAcc gating, balance_stand re-priming, continuous recalibration — none solved the core issue.

#### Root Cause

**Coordinate system mismatch between GPS bearing and Go2 IMU yaw:**
- GPS bearing is **CW-positive** (navigational): 0°=North, 90°=East, increases clockwise
- Go2 IMU yaw is **CCW-positive** (right-hand rule): turning left increases yaw, turning right decreases yaw

The calibration formula `offset = gps_bearing - imu_yaw` produces the correct heading at the moment of calibration (tautology), but after any turn the heading drifts by exactly 2× the turn angle. This is because a CW physical turn decreases IMU yaw, but should increase compass heading — the formula moves both in the same direction instead of opposite.

**Mathematical proof:** After turning CW by angle θ from calibration:
- IMU yaw decreases by θ: `new_yaw = yaw_cal - θ`
- Reported heading: `(yaw_cal - θ) + (gps_bearing - yaw_cal) = gps_bearing - θ` (WRONG — should be +θ)
- True heading: `gps_bearing + θ`
- Error: `2θ`, proportional to turn angle from calibration

**Verification across all historical runs:**
- Mar 31 11:56 (sign bug only): predicted true heading 132°, actual walk 120.6° — 11° error (GPS noise at 0.33m hAcc)
- Mar 18 (sign bug + delta bug): 103° residual matches `start_yaw ≈ 103.5°` from the delta bug exactly
- Mar 31 mission_02 runs: variable error (57°, 169°, 180°) all explained by different turn angles from calibration

#### Fix

Negate `imu_yaw` in three places in `nav_utils.py`:

1. **`_calibrate_imu()`:** `offset = normalize_angle(gps_bearing + imu_yaw)` (was `- imu_yaw`)
2. **`get_calibrated_heading()`:** `return (-imu_yaw + offset) % 360` (was `imu_yaw + offset`)
3. **Continuous recalibration:** `new_offset = normalize_angle(pos_bearing + imu_yaw)` (was `- imu_yaw`)

### NTRIP Password Fix

Initial NTRIP tests returned `HTTP/1.1 400 BAD REQUEST "Protocol parsing error"` with password `338ca`. Every authenticated request to any mountpoint got 400; wrong credentials got 401. The password was a typo — correct password is `338zca`. With the fix, caster returns `ICY 200 OK` and hAcc drops to **14mm** within seconds.

### Mission Rotation Stall Fix

First mission_02 field test: robot calibrated IMU successfully, then needed to rotate ~125° to face waypoint 1. It rotated extremely slowly (~1°/s) and appeared to stall. Two causes:

1. **`ROTATION_RATE` too low in mission scripts** — missions 00/01/02 used 0.3 rad/s (the original default), while debug test scripts had already been bumped to 0.8 after the March 10 field test. At 0.3 with the Go2's ~7% command-to-actual ratio, the robot turns at ~1.2°/s — a 125° turn would take ~104 seconds.

2. **No `balance_stand()` after IMU calibration walk** — `navigate_to()` issues `balance_stand()` at the start, but the IMU calibration walk happens after that. Once calibration completes and the robot needs to rotate toward the waypoint, the gait controller is no longer primed. This is the same root cause as the March 10 rotation failure, but in the `navigate_to()` code path rather than the test script.

**Changes:**

- `nav_utils.py`: `navigate_to()` now calls `stop()` + `balance_stand()` + 1s sleep immediately after `_calibrate_imu()` returns True
- `mission_00.py`, `mission_01.py`, `mission_02.py`: `ROTATION_RATE` 0.3 → 0.8 rad/s

### IMU Calibration: Require hAcc < 10cm

Second mission_02 field test: NTRIP working (14mm hAcc), robot turned and walked confidently — but in the wrong direction (~65° off). The IMU calibration offset was computed while hAcc was still 72-78mm (NTRIP converging). At 78mm error over a 1.5m calibration walk, the GPS bearing can be off by several degrees — and the start position captured at even worse accuracy compounds the error.

**Fix:** `_calibrate_imu()` now gates on `pos.accuracy_horizontal <= 0.1m` in two places:
1. Won't capture the calibration start position until hAcc < 10cm
2. Won't complete calibration (compute offset) until hAcc < 10cm

Also added hAcc to calibration progress and completion log messages for visibility.

### Continuous IMU Recalibration via Position-Derived Bearing

The IMU offset was computed once during the calibration walk and never updated. Any IMU drift accumulated as permanent heading error for the rest of the mission.

Initial attempt used `headMot` (GPS heading-of-motion from NAV-PVT) for continuous recalibration. This failed because the F9R's sensor fusion (HPS) corrupts `headMot` with its own uncalibrated internal IMU — `headMot` on the F9R is not a pure GPS velocity-derived COG. The recalibration was actively overwriting the good initial offset with bad data, causing the robot to walk in the wrong direction within seconds.

**Fix:** Recalibration now uses **position-derived bearing** — the same method as the initial calibration. Tracks a reference position and recomputes bearing when the robot has moved >= 0.5m from it (enough displacement for a clean bearing at 14mm hAcc). Only the SparkFun's RTK position is used; `headMot` and all F9R sensor fusion outputs are ignored. Uses exponential smoothing (alpha=0.3) to blend updates. The 0.5m threshold means updates arrive roughly every 1.5s at walking speed (0.3 m/s).

### New: Mission 02

- `mission_02.py`: Two hardcoded waypoints (46.86164631, -113.99780057) → (46.86154957, -113.99796955)
- `run_mission_02.sh`: Shell wrapper with robot discovery and env setup

---

## 2026-03-19: SparkFun F9R Sensor-Fused Heading (headVeh) Support

### Discovery

The SparkFun module is a **ZED-F9R** (not ZED-F9P) — a sensor fusion GNSS receiver with a built-in IMU. Its High Precision Sensor fusion (HPS) engine continuously fuses IMU with GNSS data to produce `headVeh` (vehicle heading referenced to true north) at 0.2° accuracy. This eliminates the need for the GPS calibration walk used by the robot IMU approach.

### Changes

**`nav_utils.py` — RTKPosition & UBloxRTKGPS:**
- Added `head_vehicle` and `head_vehicle_accuracy` optional fields to `RTKPosition` (default `None` — no breaking changes)
- `poll_nav_pvt()` now extracts `headVeh` (offset 84), `headAcc` (offset 88), and `headVehValid` (flags bit 5) from NAV-PVT
- `get_position()` passes through F9R heading to RTKPosition when `headVehValid` is set
- New `poll_esf_status()` method polls UBX-ESF-STATUS for fusion mode and per-sensor calibration status

**`test_sparkfun_imu.py` — new debug script:**
- Phase 0: Diagnostic probes (GPS-only) — tests F9R sensor fusion status, correction source, and headVeh validity
- Phase 1: GPS + robot connection (same as existing script)
- Phase 2: F9R heading acquisition — skips calibration walk if headVeh already valid, otherwise walks to trigger HPS auto-calibration; logs both F9R and robot IMU headings for comparison
- Phase 3: Turn to north using `pos.head_vehicle` instead of `navigator.get_calibrated_heading()`
- Phase 4: Walk north logging both heading sources side-by-side

### Key Differences from Robot IMU Approach

| Aspect | Robot IMU | F9R headVeh |
|--------|-----------|-------------|
| Calibration | 1.5m walk required | Auto-calibrates during motion |
| Reference | Relative to power-on | True north |
| Accuracy | Unknown + drift | 0.2° (datasheet) |
| Code complexity | offset formula, calibration state machine | Single field extraction |

### Verification

- `python -m py_compile autonomous_nav/nav_utils.py` passes
- `python -m py_compile autonomous_nav/mission/debug/test_sparkfun_imu.py` passes
- Existing `test_imu_calibration.py` still compiles (no breaking changes)
- `RTKPosition` new fields have defaults — existing constructors unaffected

---

## 2026-03-18: Fix IMU Calibration Offset Bug

### Problem

March 18 field test: robot calibrated IMU, aligned to "north" (Phase 3 succeeded), then walked **southwest at ~231°** instead of north. Replicated on second attempt.

### Root Cause

The offset formula in `_calibrate_imu()` used IMU yaw *delta* (current minus start) instead of the *absolute* IMU yaw at calibration end:

```python
# BUG:
imu_delta = imu_yaw - self._calibration_start_yaw
self._imu_north_offset = normalize_angle(gps_bearing - imu_delta)
```

Math trace showing the error:

```
heading = imu_yaw + offset
        = imu_yaw + (gps_bearing - (imu_yaw - start_yaw))
        = gps_bearing + start_yaw   ← WRONG (includes start_yaw)
```

The bug was always present but masked in March 10 tests where the starting yaw was small (~-22°). On March 18, the starting yaw was ~103.5°, producing a heading error of ~104°.

### Fix

One-line change — use absolute IMU yaw instead of delta:

```python
self._imu_north_offset = normalize_angle(gps_bearing - imu_yaw)
```

Corrected math:

```
heading = imu_yaw + offset
        = imu_yaw + (gps_bearing - imu_yaw)
        = gps_bearing   ✓
```

Also removed the now-unused `_calibration_start_yaw` variable from `__init__()`, the calibration start block, and the timeout reset block.

### Verification

- `python -m py_compile autonomous_nav/nav_utils.py` passes
- No remaining references to `_calibration_start_yaw`

---

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
