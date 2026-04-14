# Navigation Changelog

## 2026-04-14: Repo Refactor — `src/go2_survey/` Package, Data-Driven Missions, Jetson 3.8 Compatibility

### Scope

Rework the repo from a flat collection of scripts under `autonomous_nav/` into an installable Python package (`src/go2_survey/`) with a single `go2-survey` CLI entry point. Missions become **data**, not code — each mission is a directory containing a `mission.toml` config and a `waypoints.geojson`, run via `go2-survey run <name|path>`. The shell-script wrappers (`run_mission_*.sh`, `find_robot_ip.sh`, per-mission `run.sh` files) are retired; all invocations go through the CLI. The Python version pin walked back from 3.11 to 3.8 so the package installs on the current Jetson Orin Nano image.

This is a structural refactor, not a behavioral change. The navigator state machine (calibrate → turn → walk), the GPS/NTRIP/IMU logic, and the sign-corrected heading math from the April 1 successful run all carry over unchanged. Same nmap invocation, same RTCM handling, same three-phase navigation.

### New package layout

```
mpg-ai-edge/
├── src/go2_survey/          # installable package
│   ├── ntrip.py             # NTRIPConfig, EmlidNTRIPClient
│   ├── gps.py               # RTKPosition, UBloxRTKGPS, GPSManager
│   ├── robot.py             # Go2Robot (WebRTC + IMU + motion commands)
│   ├── navigator.py         # WaypointNavigator state machine
│   ├── waypoints.py         # Waypoint, load_waypoints (GeoJSON)
│   ├── geometry.py          # haversine, bearing, normalize_angle
│   ├── discovery.py         # find_robot_ips subprocess wrapper over nmap
│   ├── mission_runner.py    # shared run_mission() + MissionRunner + hooks
│   ├── cli.py               # argparse, run/list/discover-ip
│   ├── config.py            # mission.toml loader + dataclasses
│   ├── logging_utils.py     # log_banner
│   └── vision/              # TODO: frames.py + geotag.py placeholders
├── dev/
│   ├── missions/
│   │   ├── _template/       # copy-as-starting-point
│   │   ├── mission_00/      # parking lot circuit (first successful run)
│   │   └── mission_01/      # tennis court circuit
│   ├── changelog.md         # (this file)
│   ├── archive/             # retired debug scripts + historical logs
│   └── webrtc_docs/
├── setup/                   # install.md + Jetson platform notes
├── pyproject.toml           # single source of truth for deps
└── README.md
```

The 1192-line `autonomous_nav/nav_utils.py` monolith split into 8 focused modules. The entire `autonomous_nav/` tree was deleted along with every top-level inference-pipeline script and legacy data directory.

### `go2-survey` CLI

Three subcommands, all documented in `setup/install.md` under the CLI reference section:

- **`go2-survey run <mission>`** — load config + waypoints, connect GPS, wait for RTK fix, **auto-discover the robot IP** via nmap if `[robot] ip` is unset in mission.toml, connect the robot, iterate waypoints through the state machine. `<mission>` accepts a bare name (resolved under `<repo>/dev/missions/<name>`) or any path to a directory containing `mission.toml` + `waypoints.geojson`. Repo root is auto-detected from `pyproject.toml` so invocation works from any directory inside the repo. Flags: `--dry-run`, `-v`/`--verbose`, `--capture-images` (placeholder).
- **`go2-survey list`** — recursively walks `<repo>/dev/missions/` and prints each mission folder as its relative path (e.g. `mission_00`, `tennis_court/wp_set_a`). Stops recursing at a `mission.toml` (missions are leaves, not containers for nested missions). Any directory whose name starts with `_` is skipped, including its subtree, so `_template/` stays hidden and whole experimental subtrees can be hidden by prefixing a parent with `_`.
- **`go2-survey discover-ip [--cidr CIDR]`** — standalone diagnostic that shells out to `nmap -n -sT -p 8081,9991 --open -Pn <CIDR>`. The same function (`go2_survey.discovery.find_robot_ips`) powers `run`'s auto-discovery path.

### Data-driven missions

Each mission is a folder with two load-bearing files:

```
dev/missions/mission_00/
├── mission.toml         # gps/ntrip/robot/navigation settings
├── waypoints.geojson    # FeatureCollection of Point features
└── logs/                # co-located per-mission run history
```

`mission.toml` carries `[gps]`, `[ntrip]`, `[robot]`, and `[navigation]` sections plus top-level `name` and `description`. Config precedence is **dataclass defaults → mission.toml → env vars** (via `_apply_env_overrides` in `config.py`). Python 3.11+ uses stdlib `tomllib`; 3.8–3.10 uses the `tomli` backport, pulled in via a conditional PEP 508 marker in `pyproject.toml` (`tomli >= 1.1.0 ; python_version < '3.11'`).

Creating a new mission is one `cp -r` away:

```bash
cp -r dev/missions/_template dev/missions/my_new_mission
# edit mission.toml (name, description) and replace waypoints.geojson
go2-survey run my_new_mission
```

Mission lineup after restructure:

- **`mission_00`** (parking lot) — promoted from the old `mission_02`, which was the mission that produced the first successful autonomous run on 2026-04-01. Tuned settings: `arrival_tolerance=0.5`, `max_velocity=0.5`, `rotation_rate=0.8`.
- **`mission_01`** (tennis court) — same two-waypoint layout as before, now running with the same tuned values (was `0.2`/`0.3`/`0.8` pre-tuning). Mission content and waypoints unchanged.
- The old single-waypoint tennis-court `mission_00` was retired; its 5 historical logs moved to `dev/archive/logs/retired_mission_00/`.

### Mission runner + extension points

`src/go2_survey/mission_runner.py` owns the shared execution flow: load config, connect GPS, (auto-discover robot IP if needed,) connect robot, iterate waypoints. The `MissionRunner` dataclass exposes two hook slots — `on_waypoint_reached(waypoint, position)` and `on_gps_update(position)` — so future features like frame capture and live geotagging can plug in without touching any mission folder or the runner's core loop.

### Robot IP discovery

`find_robot_ip.sh` (the old nmap-based shell script) ported to `src/go2_survey/discovery.py` as a **subprocess wrapper**, not a pure-Python reimplementation. Same binary dependency (`nmap`), same Linux-only `ip route` + `ip addr` CIDR detection, same parsing, same exit codes. Wired into `mission_runner.run_mission()` so that any `go2-survey run` with `robot.ip` unset and `robot.serial` unset and `connection_mode == "LocalSTA"` automatically runs discovery before constructing `Go2Robot`. Fails fast with an actionable error message if nmap is missing or returns nothing.

### Deletions / archival

- **`autonomous_nav/`** — entire tree retired. `nav_utils.py` is now the 8 split modules; `mission/mission_*.py` and `mission/run_mission_*.sh` are replaced by data-driven folders; `rtk/rtk_with_logs.py` (a duplicate of `GPSManager`) was deleted; `reference/` moved to `dev/archive/reference/`.
- **Top-level `00_`–`04_.py`** — unused Jetson inference pipeline stubs, deleted.
- **`data/`, `subject/`, `results/`, `notes/`** — root-level dumping grounds cleaned out. `notes/` content moved to `setup/` with snake_cased filenames. The one load-bearing geojson (`tennis_court_points.geojson`) was extracted into `mission_01/waypoints.geojson` before deleting `data/`.
- **`environment.yml`, `environment-jetson.yml`, `requirements-jetson.txt`** — superseded by `pyproject.toml`.
- **`scripts/find_robot_ip.sh`** — deleted. The Python port is now the only implementation; anything that used to call the shell script either uses `go2-survey discover-ip` directly or goes through `go2-survey run`'s auto-discovery path.
- **Per-mission `run.sh` wrappers** — byte-identical one-liners execing the CLI, retired to remove the shell/CLI duality. All invocation now goes through `go2-survey run <name|path>`.
- **`.claude/settings.local.json`** — was tracked and mutated every session, cluttering `git status`. Untracked and added to `.gitignore`.
- **Debug test harnesses** (`test_imu_calibration.py`, `test_sparkfun_imu.py`) — moved unchanged to `dev/archive/debug_tests/`. They import from the old `autonomous_nav.nav_utils` path and will not run against the new package; kept for historical reference only.

### Follow-ups before merging to `main`

- **Python 3.8 install on Jetson.** The package declares `requires-python = ">=3.8"` and the `tomli` backport should be pulled in via the PEP 508 marker on 3.8–3.10, but the full `pip install -e .` + live run has only been exercised on Python 3.12 locally.
- **Auto-discovery on real hardware.** The nmap subprocess wrapper was unit-checked via `_parse_nmap_output` against synthetic output, and the shell original was production-proven, but a live end-to-end `go2-survey run mission_00` against a real Go2 has not been run since the port.
- **Newer Python versions.** 3.8 is the lower bound, not a preference. Upcoming CV work (frame capture, inference) will likely need 3.10+; should verify the full stack still installs and runs on 3.10/3.11/3.12 before committing to a wider version range.

---

## 2026-04-01: First Successful Autonomous Two-Waypoint Mission

### Result

Mission 02 completed: robot navigated from start position to waypoint 1 (8.4m, 30s), then waypoint 2 (15.5m, 41s). Total mission ~71 seconds. Both arrivals within 0.5m. Heading error during cruise settled to 1-3°. hAcc steady at 14mm throughout.

### What it took to get here

The path from "robot walks confidently in the wrong direction" to "robot navigates two waypoints" required fixing a chain of interdependent bugs discovered over March 31 and April 1 field tests:

**1. NTRIP password typo** — `338ca` → `338zca`. The Emlid caster returned `400 BAD REQUEST` on every authenticated request. Without corrections, hAcc was 250-470mm instead of 14mm, making GPS bearing unreliable for IMU calibration.

**2. IMU yaw sign convention (CW vs CCW)** — The Go2 IMU yaw is CCW-positive (right-hand rule), but GPS bearing is CW-positive (navigational). The original formula `offset = gps_bearing - imu_yaw` produced correct heading at calibration but drifted by 2× the turn angle afterward. Fix: negate imu_yaw in offset computation and heading calculation.

**3. Steering direction inversion** — The sign fix on heading also flipped what "positive error" means. Positive error changed from "target is left, turn left" to "target is right, turn right," but the proportional controller and turn logic weren't updated. The robot steered away from the waypoint in a positive feedback loop. Fix: negate the proportional gain and turn direction.

**4. ±180° turn oscillation** — After correcting the sign, the robot often started pointed ~180° from the waypoint. The nav loop recalculated turn direction every 200ms, flipping at the ±180° boundary. Fix: restructured `navigate_to()` as a state machine (calibrate → turn → walk) where the turn phase picks direction once and commits.

**5. Recalibration corruption** — Continuous IMU recalibration using position-derived bearing was corrupted by GPS jitter during in-place rotation, feeding random bearings into the offset. Fix: removed continuous recalibration. One-shot calibration at 14mm hAcc is accurate enough.

**6. Arrival tolerance and walk speed** — Robot reached 0.36m from waypoint but couldn't close the last 16cm at minimum commanded velocity. Fix: tolerance 0.2→0.5m, walk speed 0.3→0.5 m/s, minimum forward speed 0.1→0.2 m/s.

### GNSS Antenna Stabilization

Moved GNSS antenna to a more stable mounting position on the robot's back. Repeated mission 02 to compare. hAcc remained at 14mm for both runs — RTK Fixed accuracy is dominated by the base station corrections, not antenna placement. Heading stability during cruise was comparable (1-6° error). The primary benefit is mechanical reliability: a stable mount reduces the risk of antenna shift during rough terrain traversal, which would corrupt the calibration bearing. No code changes.

### Architecture

The working navigation is a three-phase state machine per waypoint:

1. **Calibrate** — walk forward, compute IMU offset when hAcc < 10cm and displacement ≥ 1.5m. Offset = `gps_bearing + imu_yaw` (sign-corrected). `balance_stand()` after. Runs once per mission; preserved between waypoints.
2. **Turn** — compute shortest turn direction once, commit, rotate at 0.8 rad/s until error < 30°. `balance_stand()` after. No direction recalculation, no oscillation possible.
3. **Walk** — forward motion with proportional steering (`vz = error * -0.015`). GPS pause/resume on fix degradation. Arrive when within 0.5m.

---

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

### Tune Arrival Tolerance and Walk Speed

First successful waypoint approach: robot walked 8m to within 0.36m of waypoint 1 with 14mm hAcc, steady heading (~34-45°), and 1-3° steering error. But it plateaued at 0.36m — the 0.2m arrival tolerance was tighter than the robot could achieve at minimum commanded velocity. The robot hovered just outside the threshold for 90s until killed.

**Changes:**
- `mission_02.py`: `ARRIVAL_TOLERANCE` 0.2m → 0.5m, `MAX_VELOCITY` 0.3 → 0.5 m/s
- `nav_utils.py`: Minimum forward speed in `_compute_velocity()` raised from 0.1 → 0.2 m/s

---

### Fix Steering Direction After Sign Convention Change

The IMU yaw sign fix (negating `imu_yaw` in heading calculation) also flipped the meaning of positive heading error. Previously, `error > 0` meant "target is to the left, turn left (CCW)." After the fix, `error > 0` means "target is to the right, turn right (CW)" — but the steering commands weren't updated, causing the robot to steer away from the waypoint in a positive feedback loop (heading spins while walking).

**Fix:** Negated steering in three places:
- `_compute_velocity()`: `vz = heading_error * -0.015` (was `0.015`)
- `navigate_to()` turn phase: `direction = -1.0 if error > 0 else 1.0` (was `1.0 if error > 0`)
- `test_imu_calibration.py` and `test_sparkfun_imu.py` turn loops: same direction flip

---

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
