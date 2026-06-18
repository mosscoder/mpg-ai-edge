# Navigation Changelog

## 2026-06-18: v0.35.0 — `run --resume`: pick an interrupted line survey back up from the last good mark

A mid-mission stop (GPS loss, Ctrl-C, a crash) no longer means re-walking the
whole strip. `go2-survey run DIR --resume` inspects the most recent run,
finds the **last cleanly-captured mark**, and continues from there.

- **Anchor from sidecars, no checkpoint file.** `resume.py::find_resume_anchor`
  scans the newest run dir (by timestamp) and returns the highest-`(leg, mark)`
  capture whose RTK position passes the mission fix gate (`fix_type ≥ min_fix`,
  `hAcc ≤ max_hacc`) — so a GPS-degraded *tail* of bad fixes is discarded and
  the anchor is the last **good** mark. The per-capture sidecar JSON is the
  durable record (written every frame), so the anchor survives any abrupt stop.
  Refuses if the newest run already logged `LINE SURVEY COMPLETE`; falls through
  0-capture false-starts to the newest run that has real captures.
- **Treats the anchor as the starting waypoint.** The remaining route is
  `[anchor P, cL, c(L+1), …, cN]`: `navigate_legs`' existing first-corner
  approach drives the dog to `P` — **self-seeding the IMU offset from that
  motion and lining up on the leg bearing** — then continues the unfinished
  tail of leg L and the remaining legs. No cal walk; same cross_track machinery.
- **One complete survey.** Captures merge **into the original partial run dir**
  (the file logs append; `leg_number_start`/`first_leg_mark_offset` continue the
  `legNN_m<MMM>` numbering from the anchor without re-shooting it), so leg L ends
  up whole and `finalize_bearings` + the manifest run over the full dir. The
  completed run keeps the partial's dir timestamp (one survey, dated from when
  it began).
- **SOP:** place the dog ~5 m back **along the leg**, facing the anchor (room to
  self-calibrate + a clean lineup); the run banner prints `RESUMING legL/mM @
  (lat, lon)`. Purely additive — all new `navigate_legs`/`_drive_leg` params
  default to a fresh run, so non-resume behavior is unchanged.



Makes the thermal failure mode (2026-06-16 strip_2 collapse at 83 °C) *visible
during a run*. Until now motor temperature appeared only in the raw `rt/lf/lowstate`
JSON the WebRTC library dumps to the console at ~1 Hz — never parsed — so tracking
it meant hand-reading heartbeats; battery was one easily-missed line every 10 s.
This adds a consolidated **HEALTH** view that combines battery + the 12 motor
temps, foregrounding the **thigh ("shoulder") motors** that overheat.

- **Per-leg HEALTH CHECK banner** (the headline): a boxed block at every corner —
  the waypoint→waypoint boundary — with battery (SOC, current, runtime, Δ since
  last leg), all four thigh temps with the hottest marked + the hot diagonal,
  the hottest thigh's **rise this leg** and **°C/min trend**, the 8 other motors'
  max, and a status line (`ok` / `[!]` caution / `[STOP]` danger). Emitted from
  `navigate_legs` at each corner (and from the nav-mode waypoint loop).
- **HEALTH AT START / END** banners: start flags a **warm start** (the 12:57
  collapse began at 72 °C with no cooldown — now caught before launch); end
  reports the battery delta + **peak thigh** reached over the run.
- **Periodic HEALTH line** every 10 s between corners (replaces the bare
  `BATTERY …` line): `HEALTH batt 90% -4.8A ~38min · motors 83°C max RR_thigh ▲+3.4°/min [STOP]`.
- **THERMAL alert** the instant a thigh crosses a threshold (≤10 s latency): a
  loud box at WARNING (caution) / ERROR (danger), independent of the banner
  cadence. Crossings re-arm on cool-down.
- **`motor.log` sidecar + `LowStateFilter`**: the raw ~1 Hz lowstate frames are
  now dropped from console/main.log (readability) and routed to a dedicated
  `motor.log` (mirrors imu.log/gps.log/battery.log) — full 12-motor + BMS detail
  preserved for post-hoc analysis, lossless.
- Thresholds in `[navigation]` (`motor_caution_temp_c=70`, `motor_danger_temp_c=78`;
  anchored on the 83 °C cut-out with margin), so they're tunable per mission.

New modules `motors.py` (MotorState + `parse_motors` + `MotorTempTrend`) and
`health.py` (HealthMonitor + banner formatters + the `run_health_logger` task,
which supersedes `battery.run_battery_logger`); `robot.get_motor_state()` mirrors
`get_battery_state()`. Battery banner formatters moved out of `battery.py` (now
just BMS state + the detailed battery.log line + `RuntimeEstimator`).

**Replay proof** — fed the 12:57 collapse's real frames through the emit path:
HEALTH AT START flags `[!] WARM START RR_thigh 72°C`, a `[STOP]` DANGER alert
fires when FL crosses 78 °C, and every subsequent leg banner reads `[STOP] … COOL
DOWN` — **three legs of escalating warning** before the locomotion cut-out,
instead of a surprise red light. Markers (`▲ ✓ [!] [STOP]` …) are module-level
constants for easy ASCII swap.

## 2026-06-16: Field Findings — Shoulder (thigh) motor temperatures (no code change)

A day of site_1 strips surfaced a motor-thermal failure mode and a persistent
shoulder-temperature asymmetry. Recorded here for the field record; no software
change. Data source: the Go2 `rt/lf/lowstate` `motor_state[].temperature` (12
motors = FR/FL/RR/RL × hip/thigh/calf; the **thigh** motors, indices 1/4/7/10,
are the "shoulders"), already in `main.log` since the v0.32.0 battery work.

**What we observed**
- **Persistent diagonal differential.** The **front-left + rear-right thighs run
  ~10 °C hotter** than the front-right + rear-left pair, under load, in *every*
  run measured (strip_4 clean, strip_5, both strip_2 attempts). Rear-right is
  usually the single hottest, and its **foot force drops over a run** (most
  offloaded — 70→20 on the 15:06 strip_2), i.e. the RR leg works hardest yet
  bears least weight.
- **It's load-dependent, not a sensor offset.** At a cool start all four thighs
  read within ~1 °C; the differential opens to ~+10 °C only as they do work — so
  it's genuine extra mechanical work on that diagonal, not a miscalibrated
  thermistor.
- **It only collapses the robot without thermal headroom.** strip_4 (cool start
  31 °C → peak 61 °C) and the 15:06 strip_2 (cool 37 °C → peak 65 °C) finished
  fine. The 12:57 strip_2 **collapsed (red light, shoulder hot to touch)**
  because it started **hot (64–72 °C, no cooldown from prior runs)** and the same
  diagonal hit the **~83 °C** thermal-protection ceiling, which kills the
  locomotion controller (sportmodestate freezes while lowstate keeps reporting →
  legs go limp). Distinct from the earlier strip_5 collapse, which was **battery**
  (42% SOC sag under a corner-load surge), not thermal — motors were only 59 °C.

**Leading hypotheses for the asymmetry**
1. **Boot-up joint-calibration (now favored).** Unitree advises powering the dog
   on **flat ground with the legs/joints in the prescribed alignment** so the
   joint-angle zeros calibrate correctly. A bad startup posture biases the
   joint-angle reference → the controller commands slightly-off angles → a
   standing torque/load imbalance on one diagonal → those motors run hotter. This
   would produce exactly a *persistent, load-dependent, non-damage* differential.
   **Testable:** re-boot carefully on flat ground with joints aligned; if the
   FL/RR–vs–FR/RL gap shrinks, it's calibration, not mechanics.
2. **Mechanical friction/bind** in the FL & RR thigh joints, and/or an off-center
   payload (the loose backpack) twisting load onto that diagonal.

**Operational guidance (until resolved)**
- **Start every strip cool** — never begin with the thighs much above ~40–45 °C;
  give real cooldown between strips (the thigh temps are in `main.log` for a
  pre-flight check). This alone prevented the repeat collapse.
- **Start above ~50% SOC** (separate failure mode — strip_5).
- **Inspect the rear-right leg** (hottest + most offloaded + highest motor
  comm-loss count), and **test the boot-up hypothesis** before assuming a
  mechanical fault.

## 2026-06-03: Design Notes (speculative) — GPS-Course Bearing, Calibration Retirement, Transect Buffering

**Status: speculative / forward-looking.** Captured from discussion after the
09c `cross_track` run (v0.26.0) for later distillation into an exact plan. No
code change, no version bump — directions, not decisions.

### Bearing: the IMU/GPS-course decoupling

09c made explicit that three bearing signals have come apart, and which one we
trust has flipped:

- **IMU calibrated heading** — where the body points (drifts; ~30° off by leg 9
  on 09b; still what EXIF `GPSImgDirection` records).
- **GPS course-over-ground** — where the robot moves (offset-free; what
  `cross_track` already steers on; what the post-hoc bearing fix recovers).
- **Leg / transect bearing** — the prescribed line.

Under `cross_track`'s 0.10 m on-leg MAD these converge (body ≈ course ≈ leg), so
**GPS course is an offset-free proxy for camera facing.** Two caveats:

- *Crab.* Course is the travel direction; the camera looks along the body. The
  sideslip is tiny on the straight legs (where imagery matters), larger on the
  5 m connectors / through corners.
- *Noise floor.* Heading-from-track error ≈ `atan(0.15 m / baseline)` — gait
  wobble (~15 cm), not RTK (1.4 cm), is the floor. 1 m baseline ≈ 8°, 2 m ≈ 4°,
  4 m ≈ 2°; per-leg averaging beats it to ~1°. A per-*image* bearing therefore
  wants a few-metre baseline.

### Post-hoc → on-the-fly bearing

The post-hoc recovery (centered neighbour-to-neighbour RTK bearing, ±2 m, ~1°)
is the gold standard because it looks both ways; live, only the past is
available. Two routes:

- *Cheapest:* `cross_track` computes `_gps_course(pos_hist, lookback)` every
  tick — thread that `cog` into the capture sidecar/EXIF (cross_track-only,
  1 m lookback).
- *General:* add `gps.course_at(frame.timestamp, lookback)` mirroring the v0.18
  `position_at(frame.timestamp)`; compute from the same fix history,
  mode-independent.

Design notes: use a **~3–4 m lookback** (the steering loop filters 1 m noise; a
single geotag does not), accept the small lag (harmless on a straight leg), and
**fall back to the known leg bearing on the connectors/turns.** Record course
live as the trusted heading, keep the IMU `achieved_heading` as a provenance
field, and keep the RTK position in the sidecar so the geotag stays
post-hoc-refinable to the centered ~1° version.

### Calibration: largely obsolete for the moving survey

With steering and geotag both on GPS course, the precise IMU offset — the whole
point of the 5 m cal walk + per-leg recal — is load-bearing for almost nothing
on the line survey:

- Steering → course. Geotag → course.
- The only consumer left is the in-place corner turn (`turn_to_bearing`), and it
  is *coarse* — it just has to start the leg roughly aimed; `cross_track` pulls
  onto the line in the first few metres via course feedback.

Direction:

- **Collapse calibration to one coarse offset** (±10–15° suffices for the turns)
  and **retire the per-leg recal** — it has been rejected every leg, it is a
  recurring rabbit hole, and nothing depends on it anymore.
- *Enabling fact:* the Jan-2026 IMU-over-COG choice was made because COG is
  unreliable at 0.3 m/s; the survey now runs 0.5–1.0 m/s where course is
  reliable (~1° on 09c). Faster survey speed retroactively obsoletes the reason
  the calibration machinery exists here.
- *Endpoint:* replace the in-place turn with **drive-through reorientation**
  (start toward the carrot, let `cross_track` yaw the body onto the line) →
  removes the last offset consumer → the line survey needs **no IMU calibration
  at all**; the GPS track self-provides heading whenever moving. Cost: the first
  few metres of each leg curve in (off-bearing); sharp 180° serpentine reversals
  may still want a pivot.
- *Caveat that keeps calibration alive:* the stationary strategies
  (`rotating_quadrat`, `waypoint_forward`) shoot while stopped — no course — so
  they still need the calibrated IMU. Calibration becomes **optional for the
  moving line survey, required only for the stationary missions.**

### Transects, not waypoints — buffer the ends, discard the turns

The legs *are* botanical survey transects; the corner turns are repositioning,
not data. So target a transect operationally by **over-walking it**: for a 50 m
transect, define the leg as 60 m (50 m + a 5 m buffer at each end), walk the
full length, and **use only the captures from the central 50 m.** The buffers
absorb the leg-start `cross_track` convergence transient, the end decel/turn,
and any first-corner approach weirdness — it is exactly the ±5 m turn buffer used
to measure the 0.10 m steady-state MAD, made operational.

This **moots most of the remaining precision concerns** and reinforces the
calibration direction above: the first-corner stall, corner-turn bearing
accuracy, and connector noise stop mattering because their captures are cropped
out. Steering only has to be clean in the middle of each transect — which 09c
showed it is.

### To distill later (exact plan TBD)

- Coarse one-time cal vs. full drive-through reorientation (does the 180°
  serpentine reversal still need an in-place pivot?).
- Transect buffer length (5 m each end?) and how buffer captures get tagged /
  cropped (by along-track distance, already in the sidecar `extra.line_survey`).

*(Realized since: post-hoc bearing corrector in-pipeline (v0.27.0); the per-leg
recal + the line-survey cal walk replaced by the self-seeding running COG recal
(v0.28.0); the live-course route dropped in favour of the post-hoc one. Transect
buffering remains the open item.)*

## 2026-06-16: v0.33.0 — Background GPS reader: position reads no longer block the event loop

Fixes the mid-mission "no position" pauses (strip_4 2026-06-16: 13 pauses on the
late legs, all `receiver returned no fix`). Root cause was **event-loop
saturation**, not a cable or the antenna: under mission load (capture JPEG/disk
writes + nav + NTRIP + video), the single asyncio loop fell behind — the GPS
sample rate halved (~50→26/min) exactly when the pauses spiked — and the
**synchronous 0.5 s `poll_nav_pvt` serial read timed out → None → pause**. The
GPS dropouts co-occurred with WiFi video frame-skips (12/13 within ±2 s, 10×
chance), proving a shared *Jetson-side* cause (USB-serial and WiFi don't share a
cable). Same mechanism behind the 06-15 "RTCM stale" pauses — different symptom,
one disease.

- `GPSManager` now runs a **dedicated background reader thread** that polls the
  receiver ~10 Hz and caches the latest fix; `get_position()` returns that cache
  instantly (verified: ~5 ms vs a 600 ms blocked poll) instead of blocking the
  caller's event loop. The blocking serial read releases the GIL, so the reader
  keeps up even when the loop is saturated. Position reads are now immune to
  event-loop load — the direct fix for the pausing.
- `get_position()` reports None only when the cache is older than
  `POSITION_MAX_AGE_S` (1.0 s) — a genuine receiver outage still pauses; a busy
  loop no longer does. `position_at()` history is now filled by the reader at a
  steady rate (maxlen 64→128) regardless of loop load.
- Reader starts in `GPSManager.connect()` (sole poller; all callers already go
  through `get_position()`), stops cleanly in `disconnect()` before the port
  closes.

Remaining lever (not in this release): the loop *slowdown* itself still drops
video frames during busy stretches — offloading capture JPEG/disk writes off the
asyncio hot path would address that. The 1 m interval doubles that write load
vs 2 m.

## 2026-06-16: v0.32.0 — Battery telemetry: per-run battery.log + start/end banners + 10 s SOC lines

The Go2 publishes battery state on the `rt/lf/lowstate` data-channel topic
(`bms_state` = soc / current / cycle / temps, plus top-level `power_v`). This
release subscribes to it and surfaces charge throughout a run.

- **New `battery.log` sidecar** alongside main/imu/gps.log (always on). Carries
  the detailed per-interval line: `SOC 82% | 28.91V | -4.6A (discharging) |
  batt 34°C | mcu 36°C | cycles 42`, sampled every 10 s.
- **`main.log` battery banners:** `BATTERY AT START: 84% | 29.4V | 31°C` in the
  robot phase, a concise `BATTERY 82% | -4.6A | ~16 min left @ current draw`
  line every 10 s through navigation, and `BATTERY AT END: 68% | 28.1V | 39°C
  (−16% over run)` at teardown (emitted before the WebRTC stream closes, so it
  lands even on abort).
- **Runtime estimate** from a rolling 3-minute SOC-drain rate (`RuntimeEstimator`);
  reads "estimating…" until there's a measurable drop over ≥60 s.
- New `go2_survey.battery` module (`BatteryState`, `parse_bms`, formatting,
  `run_battery_logger`); `Go2Robot` subscribes to `LOW_STATE` and exposes
  `get_battery_state()` mirroring `get_yaw_degrees()`. Logging split mirrors
  the GPS one: `go2_survey.battery.telemetry` → battery.log only,
  `go2_survey.battery` → main.log/console (`BatteryTelemetry*Filter`).
- Wired into the nav `run_mission` path (the surveys); a concurrent task ticks
  every 10 s and is cancelled at teardown. No mission.toml change required.

## 2026-06-11: v0.31.0 — Line-survey running recal: buffer all leg samples, gate straightness at fold time

The 2026-06-11 `site_1_strip_3` run — the first field test of the running
recal — completed cleanly (no-nudge first corner, self-seed on the approach,
all 7 corner turns ≤1.9°, 312/312 clean captures), but the estimator folded
only **twice** in 11 minutes: the commanded-vz buffer gate
(`|vz| ≤ RECAL_STRAIGHT_VZ_THRESHOLD = 0.05 rad/s`) rejected nearly every leg
tick, because cross_track constantly issues small steering corrections.
"Continuous" calibration was effectively "seed once" — harmless at 10–15 min
(nothing critical consumes the offset but corner pre-aim), increasingly stale
on longer missions (corner pre-aims drift, leg-entry recovery arcs grow).

- `_drive_leg` now buffers **every** quality sample
  (`_maybe_buffer_sample(..., enforce_straight=False)`): legs are straight by
  construction — the buffer clears at each corner.
- `_update_running_recal` enforces straightness on **measured geometry**
  instead: a centered chord whose IMU yaw turned more than
  `RUNNING_RECAL_MAX_TURN_DEG` (8° — above gait wobble, below a corner-entry
  recovery arc) end-to-end is consumed without folding.
- `navigate_to` / cal-walk paths keep the commanded-vz gate (endpoint recal
  unchanged).
- Synthetic check: under constant steering vz = 0.12 rad/s (which starved the
  old gate to zero) the estimator folds every tick and converges on the true
  offset with ±2.5° gait wobble injected; a 2°/sample arc never seeds.

## 2026-06-10: v0.30.0 — External output root (`output_dir`) + exact polygon survey areas

Two independent quality-of-life features around the line-survey core; the nav
path itself is untouched.

### Output root — keep field data out of the repo

- New top-level mission.toml key **`output_dir`** (default unset). Unset, runs
  keep landing at `<mission_dir>/runs/<name>_<TS>/` exactly as before. Set,
  the run dir becomes `<output_dir>/<mission_name>/<name>_<TS>/` — same run-dir
  naming, different parent — and **everything** already anchored to `run_dir`
  follows for free: main/imu/gps logs, captures + sidecars, `captures.geojson`,
  `finalize_bearings`, lidar-probe artifacts. `~` and `$ENV_VARS` are expanded;
  a relative path resolves against the mission dir. Env override:
  `GO2_SURVEY_OUTPUT_DIR` (mission.toml is committed; output disks are
  per-machine). `go2-survey finalize-bearings` already takes any path.
- Capacity only for now (user choice): no keeper mission sets it yet, so
  default behavior is unchanged everywhere. Future runs stop growing the repo
  (~336 MB of run data is tracked today, kept as history) once a toml opts in.
- `cmd_run` now loads the mission config *before* logging starts (to parent
  the run dir) — a malformed mission.toml gets a clean one-line error instead
  of a traceback.
- Consistency fix: `probe_gps` was the one mode writing artifacts into the
  mission dir; it now writes to `run_dir` like every other mode.
- Provenance fix: the log banner's `git <sha>` is now anchored to the
  go2-survey package's own checkout instead of the CWD — running a by-path
  mission from inside another repo (e.g. multimodal_survey on the field
  datastick) used to stamp *that* repo's SHA into the run log.

### Polygon survey areas — `make-waypoints` upgrade

Polygon input (GeoJSON/KML) existed but was too crude to field: leg endpoints
snapped to a `leg_space_m` sampling raster (up to a full spacing short of the
boundary, and the half-spacing tolerance let endpoints sit *outside* the
polygon), concave notches were silently spanned, and a polygon+pin KML
collapsed to centroid-point mode. Now:

- **Boxify.** The polygon is reduced to its oriented bounding box: long axis
  along the legs, short axis rounded UP to a whole number of `--leg-space-m`
  swaths (15 m short axis at 5 m spacing → exactly 3 legs). The box is swept
  by `width / spacing` **identical** legs — every leg the full long-axis
  length, exact spacing throughout, each inset half a spacing from the box
  edge so the swath bands tile the box. A perfect rectangle, 100% coverage of
  the polygon; legs overrun the boundary wherever the polygon is narrower
  than its box (angled ends, concave notches), and the layout plot draws both
  so the overrun is eyeballable.
- **Auto bearing** (user choice): `--bearing-deg` unset now aligns the legs to
  the polygon's **longest edge** (longest legs, fewest corner turns — the slow
  part of a survey). Explicit `--bearing-deg` overrides; point input keeps E–W.
  The resolved bearing + `bearing_source: "auto_longest_edge"` are recorded in
  the `generation` metadata.
- **`--start-corner south|north|east|west`** (default **south**): which compass
  corner of the grid wp_001 sits on. Same legs, same spacing — the serpentine
  is just walked from that corner (one of its four equivalent traversals; ties,
  e.g. exactly E–W legs, break west then south). Applies to point/seed grids
  too; recorded in the `generation` metadata.
- **Multi-feature files:** exactly one polygon among the features wins (stray
  Google Earth pins ignored, with an info line); >1 polygon errors; all-point
  inputs keep the centroid behavior.
- **Projected-CRS GeoJSON:** a legacy `crs` member declaring a non-4326 EPSG
  (QGIS exports projected layers this way — first hit: `site_1_strip3`'s
  `seed_poly.geojson` in EPSG:6514 meters) is honored: coordinates are
  transformed to WGS-84 on read instead of being misread as lon/lat.
- **Layout shows the boundary:** the source ring is embedded in
  `waypoints.geojson` `generation.source_polygon`, and `mission_layout.png`
  (incl. `plot-waypoints` re-runs) draws it, so clipping is eyeballable before
  a field day. Outputs are otherwise identical to the seed-point flow
  (`waypoints.geojson` + `mission_layout.png` next to the input).
- Dropped the now-unused ray-casting helpers (`_point_in_polygon`,
  `_point_segment_distance`, `_point_in_polygon_with_tol`, `_polygon_centroid`).

Also refreshed `dev/missions/_template/mission.toml`, which still advertised
the removed `rotating_quadrat` strategy and its dead knobs — it now shows a
`line_survey`/`cross_track` capture block and the `output_dir` example.

## 2026-06-04: v0.29.0 — Prune to the 09c line-survey core (drop drive-by, cog_fusion, rotating-quadrat)

The 09c line survey (`line_survey` + `cross_track` + self-seeding `running_cog` +
post-hoc EXIF bearing) is the proven way to gather landscape imagery with the
hardware on hand. This release removes the superseded and unused paths around it,
trimming ~640 net LOC from the nav/capture core with **no change to the 09c path
or to point-to-point `navigate_to`** (`navigator.py` 1614→1255, `capture.py`
637→433; 696 deletions / 60 insertions).

### Removed

- **Drive-by (`navigate_through`)** — the closest-pass predecessor that
  `line_survey` replaced; no mission used it. Gone: `navigate_through`,
  `_drive_by_speed`, `DriveByStrategy`, `write_drive_by_capture`, the dispatch
  branch, and the `cruise_speed`/`valley_speed`/`valley_radius_m`/`sharp_turn_deg`
  config.
- **`cog_fusion` bearing method** — CLI-only (`--cog-fusion`), never set by any
  mission, and resting on the F9R course/heading the pipeline already distrusts.
  Gone: `_recalibrate_cog_fusion`, `COG_FUSION_MIN_SPEED_M_S`, the `cog`/`speed`
  fields on `_TrajSample`, the `--cog-fusion` flag.
- **`rotating_quadrat` capture** — too slow and bearing-error-prone (the fix is a
  second GNSS antenna, a long way out). Gone: `RotatingQuadratStrategy`, the
  `_turn_to_bearing` capture helper, and the `bearings` / `turn_kp` /
  `turn_min_rate_rad_s` / `turn_timeout_sec` config.
- **`bearing_method` parameter** — with both alternatives gone, the line survey is
  always `running_cog` (self-seeds, no cal walk). The now-unreachable
  `!= "running_cog"` branches in `navigate_legs` / `_drive_leg` and the param
  threading through the navigator, `MissionRunner`, and the CLI are removed.

### Kept

Point-to-point `navigate_to` (cal walk → turn → walk → arrival endpoint recal) and
its `waypoint_forward` / `frame_only` captures; the `static_camera` /
`static_geotag` / `probe_lidar` modes; and the `point_seek` leg-steering option.
`turn_tolerance_deg` stays — the line-survey corner turn uses it.

### Missions: pruned to the active set

With the code down to the 09c path, the mission tree was tidied to match
(`go2-survey list` only scans `dev/missions/`, so archiving hides a mission while
keeping it in git):

- **Renamed to clean slots** — the a/b/c suffixes only disambiguated dev
  iterations: `09c_field_test → 09_field_test` (the validated cross_track
  reference survey) and `08b_walk_farm_row → 08_walk_farm_row` (updated to
  `cross_track` to match). Their historical run dirs keep their original
  `09c_…` / `08b_…` timestamped names.
- **Archived** to `dev/archive/missions/` (in git, hidden from `list`):
  `01_tennis_court`, `04_tennis_single_forward`, the now-broken
  `05_tennis_single_quadrat` / `06_tennis_quadrat_pair` (the only
  `rotating_quadrat` users — they would error with `Unknown capture strategy`),
  and the superseded `08_walk_farm_row` / `09_field_test` / `09b_field_test`
  line-survey steps.
- **Deleted**: `10_farm_rainyday` / `10b_farm_rainyday` — synthetic CLI grids,
  regenerable via `make-waypoints`.

Active set: `00` (bare-nav smoke), `02` (camera bench), `03` (static geotag),
`07` (lidar probe), `08` (farm-row survey), `09` (field survey — the reference).

## 2026-06-03: v0.28.0 — Line-Survey: running COG IMU recal (self-seeding) replaces the cal walk

### Continuous calibration from motion

The per-leg endpoint recal was noisy — on the 09c log it bounced the IMU offset
across an 86°↔132° (46°) range with 26° single steps, holding bad offsets for
whole legs, while the true IMU-yaw drift was a smooth ~18°/13 min (~1.4°/min,
temp-driven). New `bearing_method="running_cog"` (now the default) replaces it
with a continuous, per-tick **circular-EMA over straight, centered-COG samples**:
each tick `_update_running_recal` folds any centered-complete buffered sample
(`centered_cog + imu_yaw`, the `_calibrate_imu` convention) into the offset with a
τ≈30 s time constant and a *soft* per-sample outlier gate (never the whole-leg
reject that broke the endpoint guardrail). Replaying the in-code estimator on the
09c log: it self-seeds, tracks the drift, and caps single-update jumps at 6.9°
(continuous; tighter once leg-clamped) vs the endpoint recal's 26.5°. The earlier
standalone sim measured calibrated-heading error mean 7.9°→1.9° and
`turn_to_bearing` corner pre-aim 7.1°→3.4°.

Because v0.27.0 put steering on `cross_track` and the EXIF bearing post-hoc, the
offset's only live consumer is `turn_to_bearing`, so this is a pure **observer**
swap — it cannot destabilize the path or the data.

### The cal walk is gone for the line survey

`running_cog` **self-seeds** the offset from the first centered-complete sample
during the cal-endpoint→first-corner approach, so the line survey no longer does
a dedicated cal walk: `navigate_legs` skips `_run_cal_walk` under `running_cog`,
and `_drive_leg` skips the in-place `turn_to_bearing` while uncalibrated (the turn
needs the offset; `cross_track` converges onto the line without it). Field SOP:
**place the dog 5 m+ from the first waypoint, roughly facing it** — it calibrates
from its own motion en route and continuously thereafter. Cold-start caveat:
leg 1 gets the coarsest offset (only the ~5 m approach has fed the estimator);
`cross_track` converges its first ~2 m and it locks in by leg 2.

The cal walk and the endpoint/cog_fusion recals are retained for the stationary
strategies (`rotating_quadrat`, `waypoint_forward` via `navigate_to` /
`navigate_through`) — no motion to self-calibrate from — and remain selectable
(`--cog-fusion`, or `bearing_method` on the navigator).

## 2026-06-03: v0.27.0 — Line-Survey: cross_track first-corner approach + post-hoc centered EXIF bearing

### First-corner approach on cross_track (no more stall)

09c completed all 21 legs on `cross_track` but **froze at the first corner**: the
survey legs steer with `cross_track`, but the first corner was reached via
`navigate_to()`'s phase-3 walk — `point_seek` on the (drifted) IMU heading plus
`_compute_velocity()` whose `vx = max(0.2, …)` floor drops below the Go2
gait-translation threshold near the target, so the dog stalled and had to be
nudged. Refactor: extract `_run_cal_walk` (the phase-1 IMU cal walk, now shared
with `navigate_to`) and `_drive_leg` (turn-to-bearing + `cross_track`/`point_seek`
drive + interval/corner captures + arrival recal) out of `navigate_legs`. The
cal-endpoint → first-corner approach now drives through the **same `_drive_leg`**
as a survey leg, with captures suppressed — reaching the first corner on
offset-free `cross_track` at constant speed, no point_seek/`_compute_velocity`
stall. `navigate_to` keeps its turn + point_seek walk for the per-waypoint
(quadrat) path. The 5 m cal walk is unchanged this release.

### Post-hoc centered EXIF bearing

New `bearings.finalize_bearings()` runs at mission end (in the mission_runner
`finally`, before the manifest, so a partial run still gets it): for each
line-survey capture it computes a **centered** (look-back + look-forward,
leg-clamped) RTK-track course from the sidecar positions — the in-pipeline port
of the offline corrector that produced the aerial movie's `cor_true` — and writes
it to the sidecar (`heading.course_degrees_true` + `exif_direction_source`,
schema 3; the IMU `achieved_degrees_true` is kept as provenance) and to the JPEG
EXIF `GPSImgDirection`. The EXIF rewrite is **lossless** via `piexif` (no pixel
re-encode; a float→rational sanitize works around Pillow writing
`GPSHPositioningError` as a float, which otherwise breaks `piexif.dump`).
Validated on the 09c run: matches `cor_true` to mean 0.026°, 0 pixels changed.
Also exposed as `go2-survey finalize-bearings <run>` to re-apply to existing runs.
New dependency: `piexif`.

## 2026-06-02: v0.26.0 — Line-Survey: Cross-Track Leg Steering (GPS-Course Pure Pursuit), opt-in via `09c`

### Scope

The first 1 m/s field run (`09b_field_test`, 2026-06-02) followed the right
lawnmower order but failed to *track* the legs: it bowed 12–14 m off each 50 m
survey leg while `hdg_err` read ~0–4° (the "steering confidently on a bad
heading" signature), the cross-track displacement compounded leg-over-leg, and
by leg 13 the dog was 55 m off-grid and the run ended. Forensics: every corner
was at RTK Fixed (~1.4 cm), so the dog always knew its position — this is a
heading/steering failure, not GPS. The per-leg IMU recal that should keep
heading honest was rejected on **every** survey leg (proposed 30–85° shifts vs
the 30° guardrail) in both the 1 m/s and the good 0.5 m/s run, so the offset
stayed frozen at the cal-walk value while the IMU yaw drifted (~30° by leg 9 —
turn-induced, far past the ~0.37°/min static rate). The 0.5 m/s run survived
only because point-seek has ~2× the cross-track authority at half speed (turn
radius ∝ speed) and recovered each leg; at 1 m/s that margin is gone.

Root cause of both: point-seek steers on the **IMU heading**, which had drifted,
and re-aims at the **far corner**, so cross-track correction is weak early in a
leg. This commit adds a leg-steering law that depends on neither.

### Change

- **New `cross_track` leg-steering law in `navigate_legs`** (opt-in;
  `point_seek` stays the default, so `09`/`09b` are unchanged). It pure-pursues
  a carrot `lookahead_m` ahead on the leg **line** (not the corner) and steers
  on **GPS-derived course** — the bearing of the RTK track over the last
  `course_lookback_m` — instead of the calibrated IMU heading. GPS course is
  offset-free, so the steering is immune to the IMU-yaw drift that sank `09b`;
  the near carrot tracks the line rather than cutting to the corner. Same
  proportional law and sign as point-seek (`vz = heading_error * -0.015`), so
  the only changes are the *target* (carrot vs corner) and the *heading source*
  (GPS course vs IMU). This is v0.24 pure-pursuit minus the IMU heading that
  made it veer. The first ~1 m of each leg has no course baseline yet and drives
  straight — which doubles as a per-leg cal-stretch.
- `navigator._gps_course(hist, lookback_m)` — bearing from the most recent fix
  ≥ `lookback_m` behind to the newest; `None` until that baseline exists.
- `[capture]` gains `leg_steering` (`"point_seek"`|`"cross_track"`),
  `lookahead_m` (4.0), `course_lookback_m` (1.0), threaded
  config → `mission_runner` → `navigate_legs`. The per-leg NAV log line now
  shows the active law and `cog=` so a field run is self-diagnosing.
- **New mission `09c_field_test`** — copy of `09b` (fast, 1 m/s, identical
  waypoints) with `leg_steering = "cross_track"`. The A/B against `09b`.

### Verification offline

- AST parse; version 0.26.0; `09c` parses with `leg_steering=cross_track`,
  `point_seek` still default for `09`/`09b`.
- Closed-loop kinematic sim of a 50 m leg (proven `*-0.015` law, modeled with
  the Go2's CCW-positive yaw convention, RTK noise 1.4 cm, 0.7 command factor):
  `point_seek` with a fixed IMU offset ε reproduces the field bow (ε=15° →
  4.9 m, ε=30° → 10.3 m, recovering partway — matches `09b` leg 9's ~14→9 m).
  `cross_track` is identical for ε=0 and ε=30 (offset-immune) and converges from
  a 5 m / 30°-misheaded worst case to ~0 m end-cross with no overshoot, stable
  across lookahead 3–5 m and lookback 0.5–1.5 m.

### Hardware validation pending

First field run of `cross_track`. Run `09c` and confirm: `cross` stays small
through each 50 m leg (no 12–14 m bow), `cog` tracks the leg bearing, the
per-corner positions stay locked to the waypoints leg-over-leg (no compounding
drift like `09b`), and it completes all 21 legs. If it holds, this is the
1 m/s fix and `cross_track` should become the default; if it bows or hunts,
tighten `lookahead_m` (3 m) or `course_lookback_m`. The endpoint recal is still
non-functional and unused for steering here — fixing it (or replacing it with a
cross-track offset trim) is the separate follow-up.

## 2026-06-01: v0.25.0 — Line-Survey: Point-Seek the Corner + 5 m Cal Walk + Per-Survey-Leg Recal

### Scope

The v0.24.0 pure-pursuit line survey veered into the plant rows on the
2026-05-28 farm runs (cross-track bow ~0.9 m mid-leg). Forensics on those logs
plus replays of the good 2026-05-21 field test traced it to a wrong IMU heading
offset (ε ≈ 18°), not the capture path: the legs bowed while `hdg_err` read ~0,
i.e. the dog steered confidently on a bad heading reference. ε came from (a) a
1.5 m initial cal walk (seed error ~6–18°) and (b) sparse, noisy per-leg recal
that swung ε ±20° — short-baseline recal noise exceeds the ~0.37°/min drift it
corrects (the v0.12.0 finding, re-confirmed). The replays showed
heading-from-GPS-track error ≈ `atan(0.15 m / baseline)` — gait wobble (~15 cm),
not GPS (~1.4 cm), is the floor — so a clean offset needs a long baseline.

### Change (`e3077cd`)

- **Steering: pure pursuit → point-seek the end corner.** `navigate_legs`
  re-aims at the leg's end corner every tick from the current RTK position
  (proportional `vz = heading_error * -0.015`, the same law
  `navigate_to`/`navigate_through` use) instead of chasing a `lookahead_m`
  carrot. A residual ε now curves the path toward the corner and converges,
  rather than holding a fixed off-line bow. Forward speed stays constant for
  even 2 m capture spacing. `lookahead_m` removed (param, `CaptureSettings`,
  `mission_runner`, the four line-survey tomls).
- **Initial cal walk 1.5 m → 5 m** (`CALIBRATION_BASELINE_M`), shrinking the
  seed ε from ~6–18° to ~1–2°. Shared by every nav mission via
  `navigate_to`/`navigate_through`.
- **Recal once per *survey* leg, gated** (`RECAL_MIN_LEG_M = 10`): the 2 m/5 m
  crossovers no longer recal (a short baseline only adds noise). Plus
  `RECAL_MIN_BASELINE_M` 1.5 → 5.0 (secondary quality guard) and
  `RECAL_BUFFER_MAXLEN` 60 → 400 so a long row recals off its long straight
  body (~20 m → ~0.4°) rather than the last ~6 m near the decel.
- The 2 m interval capture + `captures.geojson` manifest path is unchanged.

### Verification offline

- AST parse; version 0.25.0; all four line-survey tomls parse with
  `lookahead_m` gone, `strategy=line_survey`, `capture_interval_m=2.0`.
  `navigate_legs` steering aims at the end corner with
  `vz = heading_error * -0.015`, no carrot/lookahead references; recal guarded
  by `leg_len >= RECAL_MIN_LEG_M`; constants set. The empirical basis
  (heading-from-track `atan(0.15/baseline)`, drift 0.37°/min) came from the
  2026-05-21 RTK-track replay.

### Hardware validation pending

First field run of point-seek line steering. Confirm: legs don't veer into the
rows (`cross` stays small through each leg, no parallel bow); corner turns only
at corners (no mid-leg stop-rotates); `IMU RECAL` fires only on the 15 m/50 m
survey legs (skipped on the 2 m/5 m crossovers) with small Δ; the 5 m cal walk
completes on the approach to the first corner; captures land ~every 2 m with
sidecars `position_interpolated: true`, mostly `frame_corrupt: false`.

## 2026-05-22: v0.24.0 — Line-Survey: Pure-Pursuit Leg Steering

### Scope

`navigate_legs` steered toward the far end corner, so cross-track error was
only weakly corrected early in a leg (correction strength ∝ 1/distance-
remaining) and legs bowed — undermining the even-swath coverage the lawnmower
pattern is meant to guarantee. The cross-track offset was already computed
(`project_along_leg`) and logged, but never fed back into steering.

### Change (`2d26d2d`)

- Leg steering is now pure pursuit: aim at a carrot `lookahead_m` ahead on the
  leg line (interpolated between the leg corners) rather than at the end
  corner, so cross-track error converges with a constant gain. Yaw rate uses
  the curvature form `vz = -speed · 2·sin(α) / lookahead_m` (α = heading error
  to the carrot), which makes the path shape speed-independent; the sign
  matches `navigate_to()`. The carrot parks on the end corner for the final
  `lookahead_m`, so arrival and end-corner capture are unchanged.
- New `[capture] lookahead_m` knob (default 4.0 m), threaded
  config → `mission_runner` → `navigate_legs` and set explicitly to 4.0 in the
  08/09 line-survey missions.

### Verification offline

- AST parse; version 0.24.0; both line-survey tomls parse with
  `lookahead_m = 4.0`. Closed-loop kinematic sim of a 50 m leg starting 0.50 m
  off-line converges to 0.000 m with no overshoot at both 4 m and 2 m
  lookahead; the new `vz` keeps the same sign as the old controller across
  ±error and is 0 when aligned.

### Hardware validation pending

First field run of pure-pursuit steering (and still the first of
`navigate_legs` overall). Start at `lookahead_m = 4.0` and watch the `cross`
readout (logged every 2 s): it should settle toward ~0 within the first
~10-15 m of a 50 m leg. Tighten to 3 → 2 m if legs still bow; raise if the dog
hunts on GPS noise.

## 2026-05-22: v0.22.0 — Line-Survey: Corner-Turn Tolerance Matches Quadrat

### Change (`3adb632`)

The corner turn in `navigate_legs` was using an 8° tolerance, so leg-start
corner captures could be up to 8° off the leg bearing. It now uses
`settings.capture.turn_tolerance_deg` (default 2°, the same source the
rotating-quadrat strategy uses), threaded from `mission_runner`. Corner
shots are now aligned as tightly as quadrat captures, and the tolerance is
tunable per mission via `[capture] turn_tolerance_deg`.

## 2026-05-22: v0.23.0 — Line-Survey: Flat Capture Folder + GeoJSON Manifest

### Scope

Line-survey captures were nesting one subfolder per leg, which is awkward to
feed to SfM/QGIS. Flatten the layout and add a single manifest.

### Change (`7c40b6e`)

- `write_interval_capture` now writes all of a run's photos flat in
  `<run>/<output_subdir>/` as `<leg>_m<NNN>_b<BBB>_<ts>.jpg` (m000 = leg
  start corner, highest m = end corner; the mark index also makes filenames
  unique within a second, fixing a latent collision). Per-image JSON
  sidecars + EXIF geotag are unchanged.
- New `write_captures_manifest(captures_dir)` scans the run's sidecars and
  writes `captures.geojson` — one Point feature per photo at its actual
  (interpolated) lon/lat, properties `{file, leg, mark_index, along_track_m,
  achieved_heading, target_bearing, fix_type, hacc_m, corrupt,
  position_interpolated, captured_at_utc}`. Written atomically (temp +
  rename); derived from the sidecars so it is rebuildable and an interrupted
  run loses nothing.
- `mission_runner` builds the manifest after the line-survey route in a
  `finally`, so even an aborted field run gets one.

### Verification offline

- AST parse; version 0.23.0. `write_captures_manifest` on synthetic
  sidecars → valid FeatureCollection, `[lon,lat]` geometry, all properties
  carried, frame-only (position-null) sidecars skipped, idempotent re-run,
  temp file cleaned.

## 2026-05-22: v0.21.0 — Line-Survey: Capture at Leg Corners

### Scope

The v0.20.0 line survey anchored its 2 m capture grid 2 m in from each leg
start and stopped short of the end corner, so the leg endpoints — the
monumented waypoints — were never imaged. This adds a capture at both the
start and end corner of every leg.

### Change (`20bec87`)

`navigate_legs` now fires a capture at the leg-start corner (right after the
turn settles — outgoing heading) and at the leg-end corner (at arrival,
before turning away — incoming heading), in addition to the 2 m along-track
marks. The end capture is skipped if an interval mark already landed within
0.5 m of the corner (de-dup). Each interior corner therefore gets two shots
(previous leg's end + next leg's start, i.e. both headings); the route's
first and last corners get one each. The start capture is GPS-quality gated.

### Verification offline

- AST parse; version 0.21.0. Per-leg capture sets: 49.89 m row → 26 (start +
  24 marks + end), 5 m connector → 4, 1.9 m farm connector → 2, 15.36 m farm
  leg → 9. Totals ~326 field-test / ~20 farm.

### Hardware validation pending

Still unrun on the robot (see v0.20.0). Also confirm the start capture isn't
smeared by residual turn settle and that interior corners show the two
expected headings.

## 2026-05-22: v0.20.0 — Line-Survey (Lawnmower) Strategy + Mission Restructure

### Scope

Mission-10 analysis showed the per-waypoint drive-by model often shot
while the dog wasn't aimed at the next waypoint: the shutter fires at
closest pass to the *current* waypoint, before the turn to the next, so
~22% of frames were >45° off and the dog stop-rotated at ~a third of the
waypoints. The fix is the drone "lawnmower" pattern, terrestrially: drive
straight legs between sparse corner waypoints and capture at a fixed
along-track interval while moving, so on-leg frames point along-track and
coverage is even by real distance traveled.

### Change (`2293399`)

**New `line_survey` strategy.** Waypoints are demoted to leg corners; the
route runs end-to-end in the navigator (like `drive_by`), not per-waypoint.

- `navigator.navigate_legs(waypoints, on_capture_cb, interval_m, speed)`:
  reach the first corner + calibrate IMU via `navigate_to` (no capture),
  then per leg — turn in place to the leg bearing (no capture), drive
  toward the end corner with proportional steering, and fire a capture each
  time along-track distance crosses the next `interval_m` mark
  (edge-triggered on absolute marks ⇒ no drift; the crossing time is
  interpolated and handed to the frame selector). Per-leg IMU recal from
  each leg's straight-line samples (long baseline ⇒ low noise).
- `geometry.project_along_leg()`: along/cross-track meters via a local
  equirectangular projection — monotonic with forward progress, immune to
  cross-track wobble.
- `capture.LineSurveyStrategy` (dispatch marker) + `write_interval_capture()`:
  per mark, select the cleanest frame near the crossing time
  (`get_best_frame_near`) and geotag by interpolating the RTK position to
  that frame's own timestamp (`position_at`) — reuses the v0.18.0 path.
  Sidecar `extra.line_survey` carries leg, mark_index, along_track_m,
  interval_m, frame_corrupt, frame_offset_from_mark_s. Captures land under
  `<run>/captures/<legNN>/`.
- `config.CaptureSettings.capture_interval_m` (default 2.0); leg speed from
  `[navigation] max_velocity`.
- `mission_runner` dispatches `line_survey` to `navigate_legs` (mirrors the
  `drive_by` branch).

**Mission restructure.** Renamed `08_farm_const_speed` → `08_walk_farm_row`
and `10_field_test_const_speed` → `09_field_test`; both tomls set
`strategy = "line_survey"`, `capture_interval_m = 2.0` (drive-by keys
dropped), `max_velocity = 0.5`. Deleted the two decel A/B missions
(`09_farm_decel_img`, `11_field_test_decel_img`) and archived each kept
mission's prior runs under `archive/`.

### Verification offline

- AST parse; version 0.20.0; both tomls parse with `strategy=line_survey`,
  `capture_interval_m=2.0`, `max_velocity=0.5`.
- `project_along_leg` recovers along/cross to mm; the mark-trigger logic
  yields 24 marks on a 49.89 m leg, 2 on a 5 m connector, 0 on a 1.9 m
  connector (~284 field-test / ~14 farm captures).
- `build_strategy("line_survey")` returns the marker; `load_waypoints` →
  4 (farm) / 22 (field test).

### Hardware validation pending

`navigate_legs` is a new drive loop and has NOT been run on the robot.
Field shakeout required before trusting it: corner turns only at corners
(no mid-leg stop-rotates), even ~2 m capture spacing, on-leg headings ≈
leg bearing, sidecars showing `position_interpolated: true` and mostly
`frame_corrupt: false`.

## 2026-05-22: v0.19.0 — make-waypoints Emits Lawnmower Leg Endpoints

### Scope

The line-survey strategy needs the route as leg corners, not a dense grid —
captures are a run-time interval behavior geotagged per-image, so they
don't belong in the geojson. The waypoint generator is repurposed to emit
corners.

### Change (`0a8c880`)

- Renamed CLI `get-waypoints` → `make-waypoints`; `--grid-m` →
  `--leg-space-m` (now the spacing between parallel legs / swath).
- `waypoint_gen` emits each serpentine row's two endpoints (the leg
  corners) instead of every node — `_grid_around_center` / `_grid_in_polygon`
  reduced to endpoints; `grid_m` renamed `leg_space_m` throughout (params,
  `generation` metadata, plot subtitle). Input parsing, projection, geojson
  writer, and the auto-plot are unchanged — the plot's arrows now trace the
  legs.
- Field test regenerated from its seed (`--side-len-m 50 --leg-space-m 5
  --epsg 6514 --bearing-deg 0`) → 22 corners; farm reduced in place to its
  4 measured leg-endpoint corners (no CLI — preserves real coords).

### Verification offline

- AST parse; `_grid_around_center(50, 5, 0)` → 22 points, 2 per leg,
  serpentine, 49.89 m legs + 5 m connectors; field-test `wp_001` matches
  the prior grid's start exactly.

## 2026-05-21: v0.18.0 — Corruption-Aware Frame Selection + GPS-Time Interpolation

### Scope

The Go2 WebRTC H264 video stream drops ~1 packet/s (136 decode-fail
warnings in a ~2.5 min run), leaving some captured frames partially
corrupt — stale macroblocks / visible smear (e.g. wp10 in
`08_farm_const_speed_2026-05-21_11-33-52`). The capture path kept only
the single freshest frame and geotagged it with the trigger-time GPS,
so a frame up to `frame_max_age` (0.5 s) stale was paired with a newer
position — a ~16 cm image/position lag at 0.5 m/s.

This commit picks, per waypoint, the frame nearest the closest pass
that is *not* corrupt, and geotags it by interpolating the RTK
position to that frame's own timestamp — so choosing an older clean
frame costs nothing geospatially and the lag is removed.

### Change (`d0d2127`)

**Corruption signal**: PyAV's per-frame `frame.is_corrupt` (FFmpeg
`decode_error_flags` / `AV_FRAME_FLAG_CORRUPT`) — the same decode-error
signal aiortc logs as `H264Decoder() failed to decode`, read directly
per output frame (no log parsing). It is a property, not a method.

**Frame ring buffer** (`robot.py`): the single latest-frame slot
becomes a 16-deep deque of `(timestamp, image, is_corrupt)` (~1 s at
15 fps). `is_corrupt` is read off the `av.VideoFrame` before
`to_ndarray()` discards it. New `get_best_frame_near(target_time,
max_age, prefer_clean)` returns the nearest non-corrupt frame in the
window, or the nearest frame with its corrupt flag if all are corrupt.

**Selection** (`frames.py`): `capture_frame` gains `target_time` and
`prefer_clean`; `FrameResult` gains `corrupt`. Drive-by passes
`target_time = position.timestamp` (the closest-pass instant); all
stationary strategies pass `prefer_clean` (→ latest clean frame).

**GPS-time interpolation** (`gps.py`): `GPSManager` keeps a 64-deep fix
history (fed in `get_position`). `position_at(t)` linear-interpolates
lat/lon/alt between the two fixes bracketing `t` (discrete fields from
the nearer sample); <2 samples or out-of-range → None and the caller
falls back to the trigger position. Sub-cm vs the RTK noise floor at
0.5 m/s.

**Wiring** (`capture.py`): drive-by selects the clean frame, then
geotags with `position_at(frame.timestamp)`, falling back to the
trigger position if history is too sparse.

**Behavior** (user decisions): applies to all strategies; if no clean
frame is in the window, use the nearest and flag it; search window =
`frame_max_age` (~0.5 s / ~25 cm). Toggle `prefer_clean_frame`
(`[capture]`, default true).

**Sidecar** bumped to `schema_version: 2` with `frame.corrupt`,
`position_interpolated`, and `frame_position_time_delta_s`. After
interpolation `position.timestamp == frame.timestamp`, so EXIF
`DateTime` and `GPSTimeStamp` agree.

### Files changed

- `src/go2_survey/robot.py` — frame ring buffer + `is_corrupt` read +
  `get_best_frame_near`
- `src/go2_survey/vision/frames.py` — `FrameResult.corrupt`;
  `capture_frame` `target_time` + `prefer_clean`
- `src/go2_survey/gps.py` — `GPSManager` fix history + `position_at`
- `src/go2_survey/capture.py` — drive-by selection + interpolation;
  stationary strategies route through `prefer_clean`
- `src/go2_survey/vision/geotag.py` — sidecar schema 2 + new fields
- `src/go2_survey/config.py` — `prefer_clean_frame` (default true)

### Verification offline

- AST parse on all changed files; version 0.18.0; missions
  06/08/09/10/11/_template parse unchanged
- Unit checks: `get_best_frame_near` (window filter, prefer-clean,
  all-corrupt → nearest + flag) and `position_at` (bracketed interp,
  out-of-range → None, just-past-newest reuse)
- `is_corrupt` confirmed a getset property (not a method) in PyAV 16.1.0

### Hardware validation pending

- Run `08_farm_const_speed` (and a tennis quadrat for the
  all-strategies path); confirm sidecars show `frame_corrupt: false`
  on most, `position_interpolated: true` for drive-by, and EXIF
  `DateTime == GPSTimeStamp`. Compare captures vs the `11-33-52`
  baseline (wp10 smear) — corruption should be gone or flagged.

## 2026-05-18: v0.12.0 — `--cog-fusion` CLI Flag (Experimental Recompute Backend)

### Scope

Per-arrival IMU recompute today computes a GPS bearing from the
endpoints (A → B) of the per-leg trajectory buffer. The forensic
analysis of the 5/15 quadrat runs showed this recompute is dominated
by GPS noise at typical baseline lengths: ±10-15° of recompute noise
on 5-10 m baselines, which exceeds the actual IMU drift it's trying
to correct. Tightly-spaced waypoints in upcoming surveys make the
baseline shorter and the problem worse.

This commit adds an opt-in alternative that fuses every qualifying
GPS course-over-ground (COG) sample buffered during the leg via an
inverse-variance-weighted circular mean (weight = speed²). Precision
scales with sample count rather than baseline length, so even short
legs (2-3 m) get useful recomputes from the ~10 Hz GPS stream.

### Change (`4a5bb50`)

**Flag**: `--cog-fusion` on `go2-survey run`. Default off — the
existing endpoint method runs unchanged. No TOML schema changes; the
flag is CLI-only because the experimental phase wants run-time A/B
without editing mission files. A future commit may promote the
better-performing method to a TOML setting with CLI override.

**Mission-start log line** (in both modes, before the dry-run exit):
`Bearing recompute method: endpoint` or `Bearing recompute method:
cog_fusion`. The choice is also tagged on every per-arrival recal
banner:

```
- IMU RECAL [endpoint] | 125.4° → 115.0° (Δ -10.4°) | n=42 bsl=10.07m -
- IMU RECAL [cog_fusion] | 125.4° → 117.8° (Δ -7.6°) | n=58 avg_speed=0.61m/s -
```

post-hoc analysis can `grep` either tag to see which method produced
which recal.

**Algorithm** (`navigator.py::_recalibrate_cog_fusion`):

Per-tick COG samples have angular noise σ_θ ≈ σ_pos / (v · Δt), so
variance scales as 1/v². Optimal inverse-variance weighting
therefore weights each sample by v². The vector-sum form of the
circular mean handles 0°/360° wrap-around correctly (averaging 359°
and 1° gives ~0°, not 180°):

```python
xs = Σ (vᵢ² · cos(cogᵢ))
ys = Σ (vᵢ² · sin(cogᵢ))
fused_bearing_deg = atan2(ys, xs) (mod 360°)
new_offset = normalize(fused_bearing_deg + last_imu_yaw)
```

The math is the closed-form solution to a 1D Kalman filter with
process noise Q = 0 (heading constant during the straight-walk
phase, which the existing `RECAL_STRAIGHT_VZ_THRESHOLD` gate
enforces). It's also the standard "mean of circular quantities"
recipe from directional statistics (Mardia & Jupp 1999).

**Sample gating** (additive to the existing endpoint-method gates):
- `speed > 0.2 m/s` — below this, single-sample noise saturates
- `cog is not None` — receiver only emits COG above ~0.1 m/s anyway

**Guardrails inherited from endpoint method**:
- `RECAL_MIN_SAMPLES = 10` (qualifying-sample minimum, not total)
- `RECAL_MAX_DELTA_DEG = 30°` (still rejects gross shifts)
- Same `imu_recalibrate_on_arrival` master switch

### Files changed

- `src/go2_survey/cli.py` — `--cog-fusion` flag, bearing_method string
- `src/go2_survey/mission_runner.py` — bearing_method field on
  MissionRunner, threaded into WaypointNavigator, mission-start log line
- `src/go2_survey/navigator.py` — _TrajSample extended with optional
  cog+speed; _maybe_buffer_sample captures them unconditionally;
  __init__ gains bearing_method kwarg with validation;
  _recalibrate_from_buffer refactored into dispatcher +
  _recalibrate_endpoint + _recalibrate_cog_fusion;
  COG_FUSION_MIN_SPEED_M_S constant
- `src/go2_survey/gps.py` — RTKPosition gains speed_over_ground field,
  populated in get_position() from existing pvt["gSpeed"] parse

### Verification offline

- Syntax check on all four edited files
- All existing missions parse with no TOML changes
- Algorithm sanity tests:
  - All-north samples → fused 0° ✓
  - 359°/1° wrap test → ~0° (wrap-around correct, NOT 180°) ✓
  - Speed-weighted (10×1° at 1m/s vs 10×91° at 0.1m/s) → 1.57°
    (variance weighting prefers faster samples, NOT 46° midpoint) ✓
- Dry-run with flag off logs `endpoint`; with `--cog-fusion` logs
  `cog_fusion`; `run --help` shows the flag

### Hardware validation pending

The two-back-to-back A/B comparison:
1. Run `06_tennis_quadrat_pair` with no flag (endpoint baseline).
2. Run `06_tennis_quadrat_pair --cog-fusion` within 5 min, same
   conditions (battery state, sky, temperature similar).
3. Compare:
   - `IMU RECAL [endpoint]` Δ magnitudes vs `IMU RECAL [cog_fusion]` Δ
     magnitudes across the legs. Expect cog_fusion Δs to be tighter
     (sub-degree corrections, not the ±15° swings seen on 5/15).
   - Sidecar `residual_degrees` distributions across the 8 captures
     per run. If cog_fusion offset is tighter AND the P-controller's
     tolerance isn't the binding constraint, residuals should also
     tighten.
   - Sample count tags in the banner — endpoint uses N=2 effectively
     (just A and B); cog_fusion typically uses N≈40-100 per leg.

If cog_fusion residuals are equal-or-worse, the flag stays opt-in
and the endpoint method retains its default position. If tighter, a
future version promotes cog_fusion to default and adds a TOML
setting for opt-out.

Bumps `0.11.0 → 0.12.0`.

---

## 2026-05-18: v0.11.0 — Lidar Probe Mission

### Scope

Stage 1 of the lidar-bitmap-companion plan. The earlier `docs/webrtc/
README.md` rewrite revealed the library has a complete (but
undocumented-in-the-README) lidar stack: 13 lidar-shaped topics in
`RTC_TOPIC` and two decoder backends. Before building any capture-
time integration on top of that surface, this commit lands a probe
mission that verifies the documented plumbing actually delivers
frames end-to-end on real hardware. Follows the project's "probe
hardware unknowns before building on them" guideline.

### Change (`ce32c30`)

**New mission**: `dev/missions/07_probe_lidar/` with `mode = "probe_lidar"`.
Robot-only — no GPS, no navigation, no waypoints.

**Probe logic** (`src/go2_survey/probes.py::run_lidar_probe`):

1. `datachannel.set_decoder("native")` — switch from the default
   `libvoxel` WASM mesh backend (returns Three.js-style mesh) to the
   `native` lz4-to-numpy backend (returns `{"points": ndarray(N, 3)}`).
2. Subscribe to `RTC_TOPIC["ULIDAR_ARRAY"]` with a counting callback.
   Auto-decode plumbing in `webrtc_datachannel.py` routes the binary
   payload through the active decoder before the callback fires.
3. Settle for 2 s and count frames received **before**
   `disableTrafficSaving(True)`. The library author's inline comment
   flags this gate as required for utlidar topics; this measurement
   confirms whether it's truly required or just recommended.
4. Open the gate, then sample for `[probe] duration_sec` (default 30 s)
   with 1 Hz progress logging: frames, rate, last-frame point count,
   XYZ extents, error count.
5. Restore the default traffic-saving state at the end.
6. Dump the first valid `(N, 3)` frame to `lidar_first_frame.npy` so
   the rasterizer can be developed and tuned offline without holding
   a live robot.
7. Generate three quick-look 480×480 grayscale PNGs from that frame:
   BEV (X-Y density), side elevation (X-Z), front elevation (Y-Z).
   These are diagnostic — the goal is "yes, the room is in there"
   not metric accuracy.
8. Write `lidar_probe_summary.json` with frame count, mean rate,
   point-count stats, and first-frame metadata.

**Dispatcher** (`src/go2_survey/mission_runner.py::_run_probe_lidar`):
mirrors `_run_probe_gps` shape — IP discovery with retry, robot
connect, teardown noise suppression. Phases logged as PHASE 1: ROBOT
then PHASE 2: LIDAR PROBE.

**Config** (`src/go2_survey/config.py`): `MissionSettings.mode`
docstring gains `"probe_lidar"`. Reuses existing `[probe]
duration_sec` field — no new TOML schema.

### Hardware validation pending

This entire feature is the validation step for the previous commit's
docs claims. Expected on first run:

- Frames begin arriving within ~3 s of subscribe + gate open
- Mean rate stable across the 30 s window (no dropouts)
- Point count per frame in the thousands (L1 typically produces tens
  of thousands of valid voxels in indoor scenes)
- BEV PNG shows recognizable room geometry; side/front show vertical
  structure

Anti-expectations (any of these would change Stage 2 design):
- Zero frames received → topic doesn't fire without `ULIDAR_SWITCH`
  toggle first, or `OBSTACLES_AVOID` controls the lidar power state
- Frames arrive before `disableTrafficSaving(True)` → gate is
  optional, simpler Stage 2 plumbing
- Decoder error or shape mismatch → library version drift; rebuild
  against documented `webrtc_datachannel.deal_array_buffer_for_normal`
  pathway

### Deferred — Stage 2

Capture-time companion bitmap (separate PNG saved next to each JPG
with sidecar entries documenting projection method + lidar timestamp
delta). Intentionally not built here. The probe's first-frame `.npy`
+ three quick-looks will inform projection choice, raster size, and
encoding before any capture-path code is touched.

Bumps `0.10.0 → 0.11.0` (new sensor surface).

---

## 2026-05-18: v0.10.0 — Captures Co-located with Logs (runs/ Layout)

### Scope

Per-run forensics used to require manual timestamp correlation between
two sibling directories: logs lived at `<mission>/logs/<TS>/{main,imu,
gps}.log` while captures landed in a flat shared `<mission>/captures/`
that mixed every run's output. From v0.10.0 every artifact for a
single mission run shares one directory under `<mission>/runs/<TS>/`.

**Breaking layout change.** External tooling that globs
`<mission>/captures/**/*.jpg` needs to switch to
`<mission>/runs/*/captures/**/*.jpg`. The migration script handles
legacy data in this repo; downstream consumers update once.

### Change (`7b5441a`)

**Code:**
- `cli.py`: `setup_logging` writes to `runs/` (was `logs/`).
- `mission_runner.py`: `MissionRunner` gains `run_dir` field; threaded
  into both `CaptureContext` construction sites (waypoint + static).
- `capture.py`: `CaptureContext` gains `run_dir`; `_capture_output_path`
  builds `<run_dir>/<output_subdir>/<wp>/<bearing>_<TS>.jpg` and
  mkdirs the per-waypoint subdir.
- `_template/mission.toml`: documents the new layout in `[capture]`
  block comments.
- `.gitignore`: scopes the legacy `runs/` rule to `/runs/` (root only).
  Without this scope fix the new per-mission `runs/` subtrees would
  have been silently ignored.

**Filename keeps the timestamp.** `b000_2026-05-15T14-01-50.jpg` —
redundant with the run-dir TS by 1-2 minutes, but disambiguates
within-run retries (rotating_quadrat re-capturing the same bearing
on a misalignment, for example).

**Per-waypoint subdir.** `<run>/captures/<wp>/<bearing>_<TS>.jpg`
rather than flat — keeps coverage spot-checks easy once waypoint
counts grow past a handful.

### Migration

`dev/scripts/migrate_to_runs_layout.py`:
- Renames each `<mission>/logs/` → `<mission>/runs/`.
- Promotes the oldest flat `<run>.log` files (pre per-run sidecar
  split) into `<run>/main.log` per-run dirs, so every run has a
  uniform shape regardless of vintage.
- Moves each capture (+ matching `.json` sidecar) into the matching
  run dir based on embedded timestamp (latest run whose TS ≤ capture
  TS). Orphans (no matching run window) land under `runs/_orphans/`
  so nothing is silently dropped.
- Idempotent; `--dry-run` flag previews moves.

All seven existing missions migrated in this commit (0 orphans, 0
skipped). Flat `captures/` and `logs/` dirs are gone repo-wide.

### Hardware validation pending

Next field run should write directly to
`<mission>/runs/<mission>_<TS>/{main,imu,gps}.log` plus
`captures/<wp>/<bearing>_<TS>.{jpg,json}` — no flat dirs anywhere.

Bumps `0.9.3 → 0.10.0`.

---

## 2026-05-18: v0.9.3 — wait_for_fix Self-Rescue + RTCM-Stalled Diagnosis

### Scope

Closes both carry-overs from the 2026-05-15 13:22 failure post-mortem
in a single commit. That failure exposed a missing path: NTRIP socket
nominally connected, msgs counter at 4, RTCM flow died silently
after the initial handshake — rtcm_age climbed monotonically from
13s → 287s while the receiver sat at type 3. The dwell aborted with
'see NTRIP attempts above' (unhelpful) and the mission had no
self-rescue path. Mid-mission the navigator's stale-detection would
have fired `reconnect_ntrip_async()`, but PHASE 1 `wait_for_fix` had
no equivalent.

### Change (`da542ff`)

Two coupled fixes in `gps.py:wait_for_fix`:

**(1) Mid-wait self-rescue.** When `state == 'connected'` AND
`has_active_corrections()` is False, the loop fires
`reconnect_ntrip_async()`. Idempotent — the manager's reconnect_lock
prevents re-entry while one is in flight. Stays fire-once-per-stall-
incident via a local flag; re-arms when corrections return so a
second stall in the same wait window also gets a rescue attempt.

**(2) Fifth cause-attribution bucket.** When state ∈ ('connected',
'reconnecting') at timeout AND `not has_active_corrections()`, the
new branch reads:

```
NTRIP connected, last RTCM 287.0s ago, forwarded 4 msgs |
Likely cause: NTRIP connected but RTCM stalled
(caster delivery issue / upstream cellular)
```

Ordering matters: the new bucket goes between the 'healthy' branch
and the 'disabled'/'gnss_only' branches, so the path that previously
fell through to 'see NTRIP attempts above' now gets the diagnosis.

Unit-tested:
- Reconnect fires once per stall (Test 1)
- Re-arms on recovery and fires again on second stall (Test 2)
- Timeout banner contains the new cause text instead of the unhelpful
  default (Test 3)

The 5/15 13:22 scenario would now likely self-recover during PHASE 1
instead of aborting. Worst case (caster truly dead, retry budget
exhausted) still hits the 300s timeout, but the banner now names the
actual cause so post-mortem doesn't need log archaeology.

### Hardware validation pending

Field validation: force RTCM stall during PHASE 1 (e.g. kill cell
briefly between NTRIP connect and first fix). Expect to see
'wait_for_fix: RTCM stalled ... firing reconnect_ntrip_async()'
followed by mission recovery. If cell stays dead, expect timeout
banner to read 'NTRIP connected but RTCM stalled' instead of 'see
NTRIP attempts above'.

Bumps `0.9.2 → 0.9.3`. Closes carry-over tasks #15 and #16.

---

## 2026-05-18: v0.9.2 — Stabilization Early Exit

### Scope

The 2026-05-15 `06_tennis_quadrat_pair` run reached RTK Fixed within
seconds of NTRIP connect — and then burned the full 60-second
stabilization dwell standing still anyway, because the dwell duration
was a fixed value, not a quality threshold. The 60s is sized for the
worst-case fresh-boot convergence; with a warm receiver and good sky
it's pure dead time.

### Change (`1d935ca`)

`stabilization_dwell` gains an early-exit predicate. If the receiver
holds **RTK Fixed (type 6)** AND `has_active_corrections()` is True
for **N consecutive seconds** (default 3), the dwell breaks out
immediately and the mission proceeds.

The active-corrections gate is the same anti-coast guard used by
`wait_for_fix` since v0.7.0 — a stale Fixed (receiver hasn't yet
realized RTCM dropped) won't trigger a false-positive early exit.
A trusted-Fixed streak broken mid-accumulation logs one INFO line
and resets the counter, so post-mortem can see the wobble.

New TOML knob:

```toml
[navigation]
stabilization_period_s     = 60    # max duration (unchanged)
stabilization_early_exit_s = 3.0   # NEW — 0 disables early exit
```

Banner reads either `DWELL COMPLETE` (full duration) or
`DWELL EARLY EXIT | ... | early@Xs/Ys (held trusted Fixed Ns)` so
the wallclock saved is visible in the main.log.

Unit-tested with three cases:
- Trusted Fixed held → exits at ~3s (was 60s)
- `early_exit_s=0` → full duration runs
- Alternating corrections-alive state → streak broken, full duration

### Hardware validation pending

Field run on `06_tennis_quadrat_pair` should now exit the dwell in
~3–10s when corrections are fresh and receiver is warm, vs. the
full 60s previously. Worst-case (slow convergence) still respects
the 60s cap and runs to completion exactly as before.

Bumps `0.9.1 → 0.9.2`.

---

## 2026-05-18: v0.9.1 — Tighter Bearing Alignment + Dual Heading Metadata

### Scope

The 2026-05-15 quadrat captures (missions 05, 06) all completed
successfully — every capture landed within ±5° of target, which is
exactly what the bang-bang turn_to_bearing loop guarantees at the
current 0.8 rad/s rotation rate (latency window between "in
tolerance" and stop-command-landing is the alignment floor). The
downstream consumer (SfM, stitching, geo-rectification) wants
tighter alignment AND ground-truth heading metadata even when
residual remains. Two coordinated commits.

### 1. P-controller bearing alignment (`d923109`)

`navigator.turn_to_bearing` is now a proportional controller with a
dead-band floor:

```
rate = clamp(|kp * error|, min_rate_rad_s, self.rotation_rate)
```

`kp = 0.04` commands full `rotation_rate` at error ≥ 20°, linear
decel below. `min_rate_rad_s = 0.15` floors above Go2 motor stiction
— a pure P controller stalls sub-threshold near zero error and
never converges. Stop command lands before overshoot, so 2° alignment
is reachable without hunting.

`CaptureSettings` defaults:
- `turn_tolerance_deg` 5.0 → 2.0
- `turn_kp = 0.04` (new)
- `turn_min_rate_rad_s = 0.15` (new)

All three field-tunable per-mission. The `[turn→N°]` log line now
includes `rate=Xrad/s` so post-mortem can verify the controller is
actually decelerating on approach (expect ~0.8 → ~0.15 across a
turn).

Production missions `05_tennis_single_quadrat` and
`06_tennis_quadrat_pair` updated from explicit
`turn_tolerance_deg = 5.0` to `2.0` to pick up the new default.

### 2. Dual heading (target + achieved) in EXIF + sidecar (`aef59bd`)

**Semantic change to EXIF — flag for downstream consumers**: EXIF
`GPSImgDirection` now holds the *actual achieved heading* at moment
of capture, not the navigation target. Photo metadata describes
what the photo shows, not the intent. Before today, b090 captures
wrote `GPSImgDirection = 90.0` even when the camera actually
pointed at 91.5°. Now it writes `GPSImgDirection = 91.5`.

The target bearing moves to EXIF `ImageDescription` as a parseable
ASCII string:

```
target_bearing_deg=90.00;achieved_heading_deg=91.50;residual_deg=1.50
```

The sidecar JSON `heading` block expands to carry both:

```json
"heading": {
  "degrees_true": 90.0,                 // legacy alias kept for back-compat
  "target_degrees_true": 90.0,
  "achieved_degrees_true": 91.5,
  "residual_degrees": 1.5,              // pre-computed: achieved − target, wrap-safe
  "source": "robot_imu_calibrated"
}
```

`residual_degrees` uses `geometry.normalize_angle` so the
target=355° → achieved=2° edge case correctly produces `+7.0`, not
`-353.0`. Downstream consumers can use this value directly without
their own angle-wrap math.

`WaypointForwardStrategy` (the `nobrg` single-capture mode) also
fixed alongside: it now correctly writes `bearing=None` (no target)
and `achieved_heading=<imu>`, instead of conflating the achieved
value into the `target_bearing_deg_true` mission-context field.

Verified with mock round-trip: three cases (rotating_quadrat with
residual, waypoint_forward with no target, wrap-around 355°→2°)
all produce the expected EXIF + sidecar contents.

### Hardware validation pending

Local unit tests + dry-run config plumbing verified. Field
validation: run `06_tennis_quadrat_pair`; expect tighter alignment
(`err ≤ 2°` consistently), decelerating `rate=X.XXrad/s` in turn
logs, and cross-consistent navigator log ↔ EXIF GPSImgDirection ↔
sidecar `achieved_degrees_true` values per capture.

---

## 2026-05-15: v0.8.3 — Robot IP Discovery Retry

### Scope

The 2026-05-14 session at the parking lot logged two consecutive
"Robot IP auto-discovery found no Go2 on the local network" aborts at
19:41:47 and 19:42:14 — same subnet (`10.123.195.0/24`) that a third
call at 19:52:27 found the robot on in 3 seconds. Diagnostic
post-mortem from the cross-session IP comparison (`.5` always; subnet
varies with hotspot DHCP pool) made clear the robot was just slow to
associate with the Pixel hotspot after power on. The discovery code
was working; the tech was just calling it too early.

This commit lets discovery ride through that transient state instead
of aborting the mission.

### Change (`1b6c4a3`)

New `_discover_robot_ip_with_retry` helper in
`src/go2_survey/mission_runner.py` wraps `find_robot_ips()` in a
retry loop:

- **`ROBOT_DISCOVERY_ATTEMPTS = 4`** total attempts
- **`ROBOT_DISCOVERY_DELAY_S = 10.0`** seconds between attempts
- Worst case ≈ 30 s of sleeps + 4× ~3 s scans = ~42 s before
  declaring "no Go2"

Only an empty result triggers a retry (the transient boot pattern).
`RuntimeError` from the discovery layer (no `ip route` output,
malformed CIDR, etc.) is a config bug and still fails fast — no
retry.

Verbose progress logs at every step so the tech can see what's
happening during the phase:

```
------------------------------------------------------------
-- AUTO-DISCOVERING ROBOT IP (up to 4 attempts, 10s between) --
------------------------------------------------------------
robot.ip not set in mission.toml and ROBOT_IP env var not set;
    scanning local network for a Go2 (nmap ports 8081/9991)...
Discovery attempt 1/4: scanning for Go2 on ports 8081/9991...
Discovery attempt 1 found no Go2; waiting 10s before retry
    (robot may still be booting / associating with hotspot)...
Discovery attempt 2/4: scanning for Go2 on ports 8081/9991...
Discovery attempt 2: found Go2 at 10.123.195.5
------------------------------------------------------------
---------- ROBOT IP DISCOVERED | 10.123.195.5 --------------
------------------------------------------------------------
```

Applied to both `run_mission` and `_run_static` call sites.

Bumps `0.8.2 → 0.8.3`.

### Hardware validation pending

Unit-tested via mock `find_robot_ips`: first-try success, found-on-third
with two sleeps, exhausted-budget returns None, RuntimeError propagates
without retry. Field validation: power on robot and immediately run
`go2-survey run`; expect discovery to ride through the boot-associate
window without aborting.

---

## 2026-05-15: v0.8.2 — GPS Stabilization Phase, Tighter Arrival Tolerance, Cause-Attribution Fix

### Scope

The 2026-05-14 parking-lot run
(`dev/missions/00_parking_lot/logs/00_parking_lot_2026-05-14_19-52-24/`)
was the first v0.7.0 mission to complete cleanly — both waypoints
reached, full reconnect handling exercised on a real `ECONNRESET`,
gps.log captured the entire RTK convergence timeline. Reading that
timeline surfaced three things to tighten:

1. **The robot started moving before RTK Fixed converged.** First
   waypoint was reached in Float (hAcc ~0.07 m); Fixed (hAcc
   0.014 m) only converged 47 s after NTRIP connect, mid-walk to the
   second waypoint. Carrier-phase ambiguity resolution is genuinely
   slow on a fresh F9P boot; the prior pipeline gave it no soak time.
2. **Arrival tolerance was loose for the RTK quality we now get.**
   0.5 m made sense when Float was the realistic best case. With
   sub-cm Fixed reliably available, 0.5 m is twice the underlying
   precision and was leaving fidelity on the table.
3. **Mid-mission cause-attribution banner read as self-contradictory.**
   When the NTRIP worker died on `[Errno 104] Connection reset by
   peer`, the navigator paused the robot within 0.7 s — correctly —
   but the FIX LOST banner reported `"rtcm_age=0.8s ... corrections
   stale"`. The `has_active_corrections()` predicate returned False
   because `connection_alive=False`, not because the age exceeded the
   threshold; but the diagnostic string lumped both cases under
   "stale". Three different failure modes; we should name them
   distinctly.

Three code commits + a docs commit, additive (no breaking schema
changes — the new field has a safe default). End state
`0.7.0 → 0.8.2`.

### 1. PHASE 2: GPS STABILIZATION between fix and motion (`70cb7f2`)

A new phase sits between PHASE 1 (GPS connect + initial fix) and the
former PHASE 2 (ROBOT), dwelling for `[navigation]
stabilization_period_s` (default 60 s) so RTK ambiguity resolution
can complete before the robot moves. All later phases reindex:

```
PHASE 1: GPS                        (unchanged)
PHASE 2: GPS STABILIZATION  (60s)   (new)
PHASE 3: ROBOT                      (was PHASE 2)
PHASE 4: NAVIGATE                   (was PHASE 3)
```

`GPSManager.stabilization_dwell()` polls `get_position()` at 1 Hz so
the existing `gps.log` telemetry captures the dwell for free. It
tracks `best_fix_type`, `min_hacc`, `mean_hacc`, `mean_sats`,
`tt_first_float`, `tt_first_fixed`, and emits banners at start, every
`progress_interval_s` (default 15 s), and end:

```
[stabilization] 15/60s | fix=5 hAcc=0.089m sats=23 | rtcm_age=0.4s
[stabilization] 30/60s | fix=5 hAcc=0.071m sats=24 | rtcm_age=0.5s
[stabilization] 45/60s | fix=6 hAcc=0.014m sats=24 | rtcm_age=0.3s

DWELL COMPLETE | best=RTK Fixed min_hAcc=0.014m |
mean hAcc=0.041m sats=23.4 | TTFloat=9s TTFixed=42s
```

`static_geotag` mode picks up the same insertion (PHASE 2:
STABILIZATION + reindex). `static_camera` (no GPS) is unchanged
because there's no PHASE 1 to anchor against. `probe_gps` is
unchanged because the probe itself is a diagnostic dwell — adding
another 60 s in front would be redundant.

Set `stabilization_period_s = 0` to skip the dwell entirely; the
method short-circuits with no banner.

### 2. Waypoint arrival tolerance 0.5 m → 0.25 m (`59b25bc`)

Single-line edits to `NavigationSettings.arrival_tolerance` and the
five in-tree mission TOMLs that explicitly set it
(`00_parking_lot`, `01_tennis_court`, `04_tennis_single_forward`,
`05_tennis_single_quadrat`, `06_tennis_quadrat_pair`, plus
`_template`). `03_static_geotag` and `02_camera_test` don't navigate,
so they're untouched.

The tighter tolerance is safe because the v0.7.0 stack now delivers
RTK Fixed (~1.4 cm) reliably during walk legs and the new
stabilization phase ensures Fixed is already locked before the robot
moves.

### 3. NTRIP stream-died vs. RTCM-stale (`cbaa37d`)

The cause string in `WaypointNavigator._fix_lost_diagnostics` now
branches on `EmlidNTRIPClient.connection_alive`:

| Sub-case | Trigger | Banner reads |
|---|---|---|
| Stream died | `connection_alive=False` | `NTRIP stream died (worker exited; state=connected); receiver coasting on type 5` |
| RTCM stale | `connection_alive=True` AND `rtcm_age > max_rtcm_age_s` | `RTCM stale (age=7.2s > max=5.0s, state=connected); receiver coasting on type 5` |

The same split applies to the `RTK FLOAT SUSPECT` banner in
`GPSManager.wait_for_fix`, which fires at the initial-fix gate when
the receiver claims Float without active corrections.

Yesterday's run would have read `NTRIP stream died (worker exited)`
instead of `rtcm_age=0.8s ... stale`.

### Hardware validation pending

Local syntax checks + config parsing verified; cause-attribution
split unit-tested via mock NTRIP/GPS objects. End-to-end behavior
(60 s dwell ticks, robot stationary during dwell, 0.25 m arrival,
ECONNRESET attribution on real hardware) requires field hardware.

---

## 2026-05-09: v0.7.0 — RTK Observability (retry, gps.log sidecar, cause-attributed banners)

### Scope

The 2026-05-08 parking-lot run
(`dev/missions/00_parking_lot/logs/00_parking_lot_2026-05-08_18-10-21/`)
exposed three structural gaps in the v0.4.x NTRIP/RTK pipeline:

1. **Single-attempt endpoints.** Each mountpoint got one 10s socket
   try; a transient cellular blip flipped straight to fallback (or
   all the way to GNSS-only) when 30s of retry would have ridden
   through it.
2. **Float-coast trusted as RTK.** The F9P/F9R holds `fix_type=5`
   (Float) for 30-60s after corrections stop. With both NTRIP
   endpoints failed at startup, the receiver still reported Float at
   `hAcc 0.052m`; the system trusted it, started navigation, and
   174 ms after the walking-leg banner the fix dropped to type 3
   with the robot stuck for 139s before the tech Ctrl-C'd.
3. **No mid-mission RTCM-loss signal, no cause attribution.** When
   corrections stop flowing, the prior code only learned about it
   indirectly when `fix_type` finally degraded. A tech reading
   `main.log` couldn't tell whether a paused robot was waiting on
   cell network or sky view.

This release closes all three. Three code commits + a docs commit
land additively (no breaking schema changes — the new TOML fields are
optional with safe defaults). End state: `0.4.1 → 0.7.0`.

### 1. NTRIP retry + RTCM freshness + mid-mission reconnect (`5303095`)

Each endpoint is now retried up to `NTRIP_ATTEMPTS_PER_ENDPOINT` (=3)
times before moving on, so the worst-case startup budget is roughly
`3 attempts × 10s socket × 2 endpoints ≈ 60s` before declaring NTRIP
unavailable. Per-attempt and per-endpoint exhaustion are logged
explicitly:

```
NTRIP: trying primary mountpoint MP22385 (1/2, attempt 1/3)
NTRIP: primary mountpoint MP22385 failed (timed out)
NTRIP: trying primary mountpoint MP22385 (1/2, attempt 2/3)
NTRIP: connected via primary mountpoint MP22385 (took 21s, 2 attempts)
```

`EmlidNTRIPClient` (`src/go2_survey/ntrip.py`) gains two liveness
signals: `last_rtcm_at` (monotonic clock, updated each successful
RTCM frame forward) and `connection_alive` (False when the worker's
`recv()` returns empty). `seconds_since_last_rtcm()` exposes the
freshness signal upstream.

`GPSManager` (`src/go2_survey/gps.py`) grows a `state` enum
(`disabled / connecting / connected / degraded / lost / gnss_only /
aborted / reconnecting`) and a `has_active_corrections()` predicate
that combines liveness with `rtcm_age < max_rtcm_age_s` (default
`5.0s`). `reconnect_ntrip_async()` runs the same retry loop in a
daemon thread so the navigator can keep polling fix during a
mid-mission reconnect — fire-and-forget, idempotent.

When every endpoint exhausts, behavior branches on the new
`[ntrip] on_unavailable` field:

- `"warn_continue"` (default) — log a banner, continue in GNSS-only
  mode with a tightened quality gate (Float treated as suspect).
  Existing missions inherit this without TOML edits.
- `"abort"` — log an ERROR banner and return False so the mission
  aborts before the robot is connected. For production survey runs
  where RTK precision is mandatory.

`[navigation]` gains two new fields:

- `mid_mission_fix_timeout = 60` — separate from `gps_fix_timeout`
  (initial wait, default 300s). A stationary robot mid-leg is a
  worse failure mode than a longer initial wait, so the in-flight
  timeout is shorter. **This also fixes a latent bug** where the
  prior `WaypointNavigator(gps_timeout=...)` was hardcoded to the
  300s constructor default and ignored the TOML.
- `max_rtcm_age_s = 5.0` — beyond this, RTK readings are treated as
  coasting.

### 2. `gps.log` sidecar at 1 Hz (`4c7235e`)

A third per-run log artifact alongside `main.log` and `imu.log`,
mirroring the `imu.log` mechanism committed in `df6eb18`. The
motivating gap: yesterday's post-mortem couldn't reconstruct the
precise `rtcm_age` timeline that would have explained the
float-coast behavior. `main.log` narrated `"fix lost type 3"` but
not `"rtcm went stale 5s before that"`.

Layout becomes:

```
dev/missions/<name>/logs/<name>_<ts>/
├── main.log    # human-readable mission narrative
├── imu.log     # 20 Hz rt/lf/sportmodestate stream (since v0.4.0)
└── gps.log     # 1 Hz GPS+RTK telemetry, JSON-per-line
```

`logging_utils.py` adds `GPSTelemetryFilter` (drops telemetry from
`main.log` / console) and `GPSTelemetryOnlyFilter` (keeps it in
`gps.log`) — same filter-pair pattern as `SportModeStateFilter`.
`cli.setup_logging` adds the `gps.log` `FileHandler` at DEBUG level,
filtered to only the `go2_survey.gps.telemetry` logger. Always on
regardless of `-v`.

`GPSManager.get_position` is wrapped with a monotonic-clock-throttled
emission that decimates to 1 Hz. The navigator polls
`get_position()` at ~5 Hz during navigation and `wait_for_rtk_fix`
loops at 1 Hz during PHASE 1, so the throttle never misses a fix
transition. Each record is one JSON line:

```json
{"t":"2026-05-09T10:00:21.900","fix":6,"hAcc":0.014,"vAcc":0.022,
 "pdop":1.18,"sats":14,"lat":46.86164,"lon":-113.99780,"hMSL":982.02,
 "corr_age_bin":1,
 "ntrip":{"state":"connected","mountpoint":"MP22385",
          "msgs":142,"bytes":18934,"rtcm_age_s":0.4}}
```

`corr_age_bin` is the receiver's quantized view (already on
`RTKPosition`); `rtcm_age_s` is our TCP-socket view. Together they
distinguish a network blip we recovered from (both small) from
float-coast (bin growing, age large) at post-mortem time.

### 3. Float-coast refused; cause-attributed banners (`11baa78`)

`GPSManager.wait_for_fix` is no longer a one-line delegate to the
receiver's gate. It owns the fix-wait loop and accepts only when:

```
pos.fix_type >= min_fix_type
AND (pos.fix_type <= 4  OR  self.has_active_corrections())
```

A reported RTK fix with no active corrections is rejected; a one-shot
`RTK FLOAT SUSPECT` banner fires the first time the receiver claims
Float without RTCM, and the loop keeps waiting. Periodic 15s
progress logs include `rtcm_age` so the tech can see the timeline.
On timeout, the multi-line `GPS FIX TIMEOUT` banner names the likely
cause:

```
GPS FIX TIMEOUT | 300s | never reached type 5 |
Last: type 3 hAcc 0.480m sats 10 |
NTRIP healthy (msgs=872) |
Likely cause: poor sky view / multipath
```

`WaypointNavigator` (`src/go2_survey/navigator.py`) replaces its
bare quality check with `_is_quality_acceptable(pos,
corrections_active)`, which incorporates `has_active_corrections()`
the same way. `_fix_lost_diagnostics` produces a cause-attribution
string ("NTRIP corrections stale" vs. "receiver fix degraded" vs.
"hAcc out of band") that goes into the `GPS FIX LOST` banner. When
the cause is stale corrections AND NTRIP was previously connected,
the navigator fires `gps.reconnect_ntrip_async()` so the manager
retries endpoints in a daemon thread while the navigator keeps
polling.

`GPS FIX RESTORED`, `GPS PAUSE TIMEOUT`, and the periodic
still-waiting log now all carry NTRIP state too. Yesterday's run
would have read `Cause: NTRIP corrections stale (state=gnss_only);
reported 5 but coasting` instead of just `type 3 hAcc 0.064m`.

### Hardware validation pending

Local dry-runs verified config plumbing, log-artifact creation, and
banner formatting. End-to-end RTK behavior (retry budget under real
cellular blip, mid-mission reconnect, `on_unavailable=abort`
short-circuit) requires field hardware on the robot.

---

## 2026-04-27: v0.4.0 — NTRIP Fallback + Version Banner + Split IMU Log

### Scope

Three independent improvements to the mission runner, plus a coordinated
change to the on-disk mountpoint. Field deployment moves from `MP15774`
to the new Emlid base station `MP22385`, with `MP22385a` registered as
an automatic fallback. Every run log now carries a version banner so
post-mortems can pin a captured artifact to a specific build. The 20 Hz
`rt/lf/sportmodestate` IMU stream — previously discarded outside `-v`
runs — is now preserved by default in a separate `imu.log` artifact.

This is the first changelog entry tagged with a package version. From
here, every code commit bumps `__version__` (in
`src/go2_survey/__init__.py` and `pyproject.toml`) so the banner in a
log file uniquely identifies the build that produced it. Docs-only
commits (this one included) do not bump.

### 1. NTRIP fallback via parallel mountpoint/credential lists

The single-mountpoint `[ntrip]` schema is replaced with three parallel
lists. Index 0 is the primary; subsequent indices are tried in order on
failure. Field crews can now register N caster endpoints without code
changes, and an outage on the primary base station no longer drops the
mission to GNSS-only when a working alternative is configured.

```toml
[ntrip]
host        = "caster.emlid.com"
port        = 2101
mountpoints = ["MP22385", "MP22385a"]
usernames   = ["u26787",  "u26787"]
passwords   = ["492utz",  "492utz"]
```

Fallback orchestration lives in `GPSManager._connect_ntrip_with_fallback`
(`src/go2_survey/gps.py`). `EmlidNTRIPClient` itself stays
single-mountpoint — one client = one connection — so each attempt logs
cleanly and the working client is the one that ends up streaming RTCM:

```
NTRIP: trying primary mountpoint MP22385 (1/2)
NTRIP: primary mountpoint MP22385 failed
NTRIP: trying fallback #1 mountpoint MP22385a (2/2)
NTRIP: connected via fallback #1 mountpoint MP22385a
```

If every endpoint fails, `GPSManager` logs `NTRIP not available (all N
endpoint(s) failed), using GNSS-only mode` and continues — the GPS
itself is usable without RTK, so the mission isn't aborted on caster
unavailability.

**Schema is a hard cutover.** `load_mission_config` raises `ValueError`
if a TOML still uses the legacy scalar keys (`mountpoint` / `username` /
`password`), naming the file and the rename. A second validator
(`_validate_ntrip` in `src/go2_survey/config.py`) checks that the three
lists are length-aligned and reports the mismatch with all three
lengths so a wrong-length list isn't silently truncated:

```
NTRIP config invalid in dev/missions/00_parking_lot/mission.toml:
  mountpoints has 2 entries, usernames has 1, passwords has 2.
  Each list must have the same length (one entry per endpoint).
```

The error is printed to stderr (the runtime logger isn't set up at
config-load time) and raised, so the mission aborts before any GPS or
NTRIP work begins.

**Env-var overrides** switch to comma-separated lists:
`EMLID_MOUNTPOINTS="MP22385,MP22385a"`,
`EMLID_USERNAMES="u26787,u26787"`,
`EMLID_PASSWORDS="492utz,492utz"`. Old unsuffixed `EMLID_MOUNTPOINT` /
`EMLID_USERNAME` / `EMLID_PASSWORD` are removed; the same length-mismatch
validator runs after env vars merge in.

All seven in-tree mission TOMLs were migrated in this commit; the
`_template/mission.toml` now ships the new schema with explanatory
comments. `02_camera_test` has no `[ntrip]` section and is unaffected.

Commit: `fb3a72f`.

### 2. Version + git SHA banner at the top of every log

A new helper `_git_short_sha()` in `src/go2_survey/cli.py` runs
`git rev-parse --short HEAD` (subprocess, 2 s timeout) and falls back
to `"unknown"` on failure. `setup_logging` emits the banner as the
first log line:

```
2026-04-27 08:32:43,379 [INFO] go2_survey.cli: go2-survey v0.4.0 |
    git df6eb18 | run dir: dev/missions/00_parking_lot/logs/
    00_parking_lot_2026-04-27_08-32-43
```

This makes log post-mortems self-describing — no cross-referencing
file timestamps against `git log`, and no ambiguity about which build
produced which artifact when running off a tagged release vs. a dirty
working tree.

Commit: `b7d63da`.

### 3. Per-run log directory: `main.log` + `imu.log`

Replaces the flat `logs/<name>_<TIMESTAMP>.log` file with a per-run
directory holding two streams:

```
dev/missions/00_parking_lot/logs/
└── 00_parking_lot_2026-04-27_08-32-43/
    ├── main.log    # mission narrative (was: <name>_<ts>.log)
    └── imu.log     # rt/lf/sportmodestate stream, on by default
```

`main.log` (and the console mirror) drop the 20 Hz `rt/lf/sportmodestate`
flood and the `unitree_webrtc_connect` legacy-SDP-probe errors, same as
before. `imu.log` is new: a dedicated `FileHandler` whose
`SportModeStateOnlyFilter` (`src/go2_survey/logging_utils.py`) is the
inverse of `SportModeStateFilter`. It captures only the IMU/position/
foot stream, runs at DEBUG regardless of root level, and writes whether
or not `-v` is set.

**Why split the artifacts** rather than gating on `-v`: the prior
behavior (set up in commit `ffda4b7` on 2026-04-23) discarded the IMU
stream entirely on default runs, so any post-hoc analysis that wanted
it — calibration drift checks, gait diagnostics, foot-contact timing —
required a verbose re-run. With the split, the data is on disk for
every mission; only the human-readable narrative is filtered.

**`-v`/`--verbose` semantics** narrow as a result: it now controls only
the root log level (`INFO` → `DEBUG`). The IMU stream is no longer
gated by it. Help text in `cli.build_parser()` updated accordingly.

Commit: `df6eb18`.

### Hardware validation pending

The NTRIP fallback can only be exercised end-to-end on the robot in the
field — local dry-runs only confirm config loading and call wiring. The
log split and version banner are fully verified locally.

---

## 2026-04-23: NTRIP Credential Rotation + Clean Mission Exit + Log Hygiene

### Scope

Three independent-but-adjacent cleanup passes landed in one session:
rotate the Emlid NTRIP account and strip live credentials from public
docs, fix the "process hangs after mission completes" failure mode so
the tech doesn't have to Ctrl-C after every run, and quiet the 20 Hz
IMU data-channel flood that was drowning the mission narrative.

### 1. NTRIP credential rotation and source-code sanitization

Moved the live account from `u65352` / `338zca` to `u26787` / `492utz`
across all seven in-tree mission TOMLs. Public example blocks in
`README.md` and `docs/install.md` now carry random placeholders
(`u47193` / `x9kqbw`) shaped like Emlid credentials but intentionally
non-functional, so copy-pasting from the README doesn't leak a working
account.

`src/go2_survey/gps.py:580-581` previously embedded the old credentials
as `os.getenv()` fallback defaults, meaning the source code always
shipped a working real account whether or not a user set env vars.
The fallbacks are now empty strings; the existing
`if use_ntrip and self.ntrip_config.username:` guard at
`gps.py:594` short-circuits NTRIP when unconfigured, so there's no
functional change for missions that provide creds via TOML.

Commit: `e6ade6c`.

### 2. Clean mission process exit (no more Ctrl-C after capture)

**Symptom.** `02_camera_test` hardware logs (the 2026-04-22 runs Erik
pushed) ended with `asyncio: Task was destroyed but it is pending!`
— once 19 s after "Connections closed", once **4 minutes later** in a
second run. The mission's captures were on disk well before the
warning fired, but the Python process refused to exit cleanly and
the tech had to Ctrl-C.

**Root cause.** `robot.py` had no `close()` / `disconnect()` method
at all. `disable_video()` turned the video channel off but never tore
down the `RTCPeerConnection`. aiortc's own finalizer eventually
scheduled a `pc.close()` coroutine; the asyncio event loop couldn't
fully drain it before interpreter shutdown; hence the dangling-task
warning and the exit delay.

**Fix bundle** (commit `90cc1e6`):

- **`Go2Robot.close()`** — new method. Cancels and **awaits** the
  video consumer task (the existing `disable_video()` only called
  `cancel()` without awaiting — same dangling-task shape), then
  calls `UnitreeWebRTCConnection.disconnect()` (the library's own
  wrapper, which internally does `await self.pc.close()` — preferred
  over poking `.pc` directly so we don't bypass the library's track
  and datachannel teardown). Idempotent and safe when `connect()`
  never completed.
- **Wired into both finally blocks** in `mission_runner.py`:
  `run_mission` and `_run_static`. `_run_static` now also sends
  `robot.stop()` for defensive symmetry with nav-mode teardown —
  currently benign (no static strategy moves the robot) but will
  matter once one does.
- **`ntrip.disconnect()` reordered** to shut down the socket
  **before** joining the worker thread. Prior ordering let the
  join time out at 2 s because the blocked `recv()` wouldn't wake
  until its 10 s socket timeout elapsed. The correction worker's
  `except` branch is now aware of the intentional-shutdown case
  (`self.running is False`) and skips the spurious
  `Error in correction worker: ...` log during normal teardown.
- **Teardown exceptions no longer silently swallowed.** The five
  `except Exception: pass` in `mission_runner.py` finally blocks
  became `except Exception: logger.debug("<op> raised during
  teardown", exc_info=True)`. INFO-level runs still see a clean
  teardown; a consistently-failing cleanup step now surfaces under
  `-v`/`--verbose` instead of being invisible.

**Hardware validation pending.** All five changes have clean dry-runs
and unit-tested filters, but the actual "no Ctrl-C needed" test
requires running `02_camera_test` on the Jetson + Go2 and watching
the process exit. Next bench session.

### 3. Log hygiene — two library-noise filters

Two new `logging.Filter` subclasses in
`src/go2_survey/logging_utils.py`, both attached to the root logger's
handlers by `cli.setup_logging` so library records emitted on the
root logger are filtered regardless of origin.

**`WebRTCFallbackNoiseFilter`** — suppresses the
`unitree_webrtc_connect` "old method" fallback error pair:

```
[ERROR] root: An error occurred: HTTPConnectionPool(host='...',
    port=8081): Max retries exceeded with url: /offer ...
[ERROR] root: An error occurred with the old method:
    Failed to receive SDP Answer: No response
```

The library probes a legacy SDP endpoint on port 8081 before falling
back to the canonical path that actually works on our Go2 firmware.
The probe failure is harmless — the library recovers on the next
request — but logging it at ERROR masquerades as a real problem and
makes `grep ERROR` on mission logs useless. Filter matches on two
substring patterns (`"Max retries exceeded with url: /offer"` and
`"An error occurred with the old method:"`) that are distinctive
enough to not false-positive on real library errors. Commit `90cc1e6`.

**`SportModeStateFilter`** — suppresses the 20 Hz
`rt/lf/sportmodestate` data-channel INFO flood. The library logs
every incoming data channel message at INFO on the root logger, and
the sport-mode-state topic alone produces ~1200 lines/minute of
IMU/velocity/foot payloads. Our code already extracts the only field
we consume (`imu_state` for `Go2Robot.get_yaw_degrees`) via
`_on_sport_state`, and the per-waypoint trajectory buffer (2026-04-18
entry) captures the structured slice you'd want for post-hoc IMU
analysis. The raw log lines add nothing in default runs. Commit
`ffda4b7`.

**Conditional on `-v`/`--verbose`.** `SportModeStateFilter` is only
installed when verbose logging is off — `go2-survey run <mission> -v`
still surfaces the full firehose for diagnostic runs where you need
to see what the robot was reporting per-tick.
`WebRTCFallbackNoiseFilter` installs unconditionally: those messages
are never useful, verbose or not.

### What didn't change

- Historical changelog mentions of the old credentials are left
  intact — append-only convention; those describe repo state at the
  time of writing.
- Nav-mode `run_mission()` finally was already correct in structure
  (sent `disable_video` + `stop` + `gps.disconnect`); we added the
  new `close()` step and reworked the except-clauses.
- No mission-config schema changes. Every edit is source-only or
  test-data only; `go2-survey list` and all six navigational
  dry-runs pass without touching mission.toml.

---

## 2026-04-18: Docs Reorg — Single Top-Level `docs/`

### Scope

Reorganized all documentation into one `docs/` tree at the repo root.
The previous split — vendor reference notes under `dev/docs/` and the
install + Jetson notes under `setup/` — had two inconsistent
documentation roots and made the Jetson notes hard to discover. This
change collapses both into a single top-level `docs/` and groups the
three Jetson notes into their own subdirectory with deduplicated
filenames.

### Move map

```
dev/docs/gps/                      → docs/gps/                       (4 PDFs + README)
dev/docs/webrtc/                   → docs/webrtc/                    (README)
setup/install.md                   → docs/install.md
setup/jetson_orin_nano_setup.md    → docs/jetson_orin_nano/jetson_setup.md
setup/jetson_orin_nano_status.md   → docs/jetson_orin_nano/jetson_status.md
setup/orin_nano_ml_setup.md        → docs/jetson_orin_nano/ml_setup.md
```

Old `dev/docs/` and `setup/` directories are gone. The Jetson filenames
drop the `jetson_orin_nano_` prefix since the parent directory already
carries it.

Result:

```
docs/
├── install.md             # end-to-end install + first-run walkthrough
├── gps/                   # u-blox ZED-F9R reference PDFs + config notes
├── webrtc/                # unitree_webrtc_connect protocol notes
└── jetson_orin_nano/      # Jetson hardware/OS setup + ML stack notes
```

### Reference sweep

- `README.md:36` — install link `setup/install.md` → `docs/install.md`.
- `README.md:127-145` — Layout tree restructured; `dev/docs/` and the
  separate `setup/` line are gone, replaced by a single `docs/`
  block listing `install.md`, `gps/`, `webrtc/`, `jetson_orin_nano/`.
- `src/go2_survey/robot.py:47, 142` — comments now point at
  `docs/webrtc/README.md`.
- `docs/install.md:208-209` — Jetson cross-references updated to the
  new path + filenames.
- `docs/gps/README.md:378` — internal `mount/` self-reference now
  points at `docs/gps/mount/`.

### What didn't change

- Historical changelog entries that mention `dev/docs/`,
  `dev/gps_docs/`, `dev/webrtc_docs/`, or `setup/install.md` are left
  intact — those paths were correct at the time those entries were
  written. Append-only convention.
- All renames went through `git mv`-equivalent staging (the user did
  the moves manually; `git add -A` detected them as renames at 100%
  similarity), so `git log --follow` keeps working on every PDF and
  README.

---

## 2026-04-18: Capture Bug Fix + Per-Waypoint IMU Recalibration

### Scope

Two follow-ups to the 2026-04-16 capture infrastructure: a real
strategy for the previously-broken `static_camera` mode, and a way to
fight Go2 IMU yaw drift across multi-waypoint missions without forcing
a calibration walk at every stop.

### Fix: `FrameOnlyStrategy` for `static_camera` mode

Mission `02_camera_test` was crashing on first hardware run.
`mission_runner._run_static()` constructs a `CaptureContext` with
`gps=None` in `static_camera` mode (no GPS by design — it's a
lab-bench smoke test), but the strategy it dispatched to —
`WaypointForwardStrategy` — called `_sample_position(ctx)`
unconditionally, which dereferenced `ctx.gps.get_position()` on a
`None`. AttributeError before the first frame was ever pulled.

Three changes to fix the underlying design mismatch:

- **New `FrameOnlyStrategy` in `src/go2_survey/capture.py`** — settle,
  capture one frame, write JPEG + minimal sidecar. No GPS, no bearing,
  no rotation. Registered under name `"frame_only"`. The right
  primitive for "prove the WebRTC video pipeline works without RTK
  or sky view."
- **New `write_frame_only_jpeg()` in `vision/geotag.py`** — sibling to
  `write_geotagged_jpeg`; writes basic EXIF (Make/Model/Software/
  DateTime) and a sidecar shaped like the geotagged sidecar but with
  `position: null` and `heading.source: "none"` so a downstream
  consumer can load both flavors uniformly.
- **`CaptureContext.gps` annotation tightened** to `GPSManager | None`
  to match what `_run_static` actually passes; `_sample_position`
  guarded against `ctx.gps is None`; the
  `# type: ignore[arg-type]` on the `_run_static` call site is gone.

`02_camera_test/mission.toml` now points at `strategy = "frame_only"`
and drops the unused `gps_avg_sec` field.

### Per-waypoint IMU recalibration from trajectory buffer

The navigator previously calibrated `_imu_north_offset` exactly once
per mission, on the approach to the first waypoint. On a multi-leg
mission like `06_tennis_quadrat_pair`, the Go2 IMU drifts a few degrees
between waypoints, biasing every captured-frame bearing thereafter.
Forcing another full calibration walk at each waypoint would slow the
mission down and waste the GPS+IMU samples we already collect during
normal walking. So instead: buffer those samples and recompute the
offset at each arrival.

Mechanics:

- **Trajectory buffer** — a `collections.deque(maxlen=60)` (~12 s at
  5 Hz) on `WaypointNavigator`. Cleared at the top of every
  `navigate_to()` so leg N's recal sees only leg N's samples.
- **Sample-admission filter** in `_maybe_buffer_sample()`:
  GPS quality gate (`fix_type ≥ min_fix_type`, `hAcc ≤ max_hacc` —
  same gate the navigator already uses for motion decisions),
  *plus* `|commanded vz| ≤ 0.05 rad/s`. The straight-line gate is
  the load-bearing one: if the robot is actively steering to converge
  on the waypoint, a chord between two GPS points doesn't represent
  the robot's heading at the endpoint, and the recal would replace a
  clean offset with a dirtier one. Cal-walk samples (Phase 1, vz=0
  by construction) always pass — those are the cleanest data in the
  whole leg.
- **`_recalibrate_from_buffer()`** runs on arrival, after
  `robot.stop()` and before `return True`. Drops the last 1.0 s of
  samples (arrival deceleration noise), picks A = earliest remaining
  sample and B = newest remaining sample, requires ≥ 10 buffered
  samples and a ≥ 1.5 m baseline, then computes
  `new_offset = normalize_angle(bearing(A → B) + B.imu_yaw)` —
  same sign convention as the existing `_calibrate_imu`.
- **30° guardrail** — if the proposed shift exceeds
  `RECAL_MAX_DELTA_DEG`, it's rejected as a corrupt sample (real
  drift in a single leg is sub-degree to a few degrees). All
  outcomes log via `log_banner` so per-leg drift is visible in
  mission logs.

Tunables (`RECAL_MIN_SAMPLES = 10`, `RECAL_MIN_BASELINE_M = 1.5`,
`RECAL_MAX_DELTA_DEG = 30`, `RECAL_DROP_RECENT_SEC = 1.0`,
`RECAL_STRAIGHT_VZ_THRESHOLD = 0.05`, `RECAL_BUFFER_MAXLEN = 60`)
are class constants on `WaypointNavigator` — the next person debugging
this will want to twiddle them, but they don't belong in mission TOML.
The on/off switch is the only knob exposed via config:
`NavigationSettings.imu_recalibrate_on_arrival: bool = True`. Setting
it `false` preserves the legacy single-shot calibration.

Runtime cost is negligible — microseconds per tick to buffer, sub-ms
at arrival to recalibrate, no new sleeps or robot commands. Mission
durations are unchanged. The only second-order behavioral effect is
that the Phase 2 turn-in at subsequent waypoints uses a possibly-
shifted offset; for a few-degree drift that's tens of milliseconds
of extra or fewer rotation, well below stopwatch noise.

**Caveat worth flagging for the eventual `06_tennis_quadrat_pair`
post-run review:** this fixes drift accumulated *between* legs, not
*within* a rotating-quadrat sequence. The IMU is driven hard during
those four 90° turns, and the offset set on arrival is already stale
by capture #4. If quadrat bearing error in real data turns out to be
dominated by within-strategy drift, the per-waypoint recal won't
move the needle much — and we'd want a different approach (e.g. an
in-strategy mini-recal, or accepting that absolute bearings need an
external compass).

---

## 2026-04-16: F9R Manual Deep-Read — NAV-PVT Parse Fix + GPS Docs Archive + Mission Renames

### Scope

Sat down with the u-blox ZED-F9R documentation set end-to-end for the
first time. Mirrored the four canonical PDFs locally, wrote a ~450-line
reference README explaining how sensor fusion (HPS) actually works on
this module, and in the process found that our live NAV-PVT parser has
been reading the wrong bytes for several fields. The position itself
was always correct; the "fix quality" story around it was not.

Mission folders renamed for clarity (`mission_00` → `00_parking_lot`,
`mission_01` → `01_tennis_court`). Unused diagnostic probe mission
(`02_probe_sparkfun_data`) deleted before ever running on hardware
once the deep read made the probe's primary question (which byte
offset holds `headAcc`) answerable from the manual directly.

### What we learned from the F9R manual

`dev/gps_docs/` now holds the four canonical u-blox PDFs — Data Sheet,
Integration Manual, F9 HPS 1.30 Interface Description, and FW1.00
HPS1.30 release notes — plus a local `README.md` that indexes them and
explains how to configure the F9R's sensor fusion for reliable
heading output.

**Major finding: the F9R's IMU is NOT currently a usable compass for
us, and making it one is genuine work.** To get trustworthy `headVeh`
values out of the F9R's High Precision Sensor fusion (HPS) engine,
the module needs:

1. **Lever-arm configuration** — the 3D vector in cm from the F9R
   chip's internal IMU reference point to the GNSS antenna's phase
   center, expressed in the installation frame (X = forward,
   Y = left, Z = up). Configured via the `CFG-SFIMU-IMU2ANT_*` keys.
   Currently zero on our unit — so any body rotation introduces phantom
   velocity at the antenna that fusion can't reconcile.
2. **IMU-to-vehicle-frame rotation** — three Euler angles
   (`CFG-SFIMU-IMU_MNTALG_YAW/PITCH/ROLL`) telling the F9R how its
   internal chip axes are rotated relative to the dog's
   forward/left/up. Auto-alignment exists but needs vehicle dynamics
   above 10-30 km/h per the manual; our dog walks at 1.8 km/h, well
   below the validated regime. User-defined angles (tape-measure +
   protractor) are the more reliable path at our speeds.
3. **Speed/odometer data** — the F9R was designed for wheel-tick
   vehicles. Without one, it runs in Degraded HPS mode with position
   drift when stationary. Ideal fix is a small bridge feeding the
   Go2's body velocity (already published in the WebRTC sport-state
   stream we subscribe to for IMU yaw) into the F9R as
   `UBX-ESF-MEAS` speed data over the serial link.
4. **A calibration drive** — stationary warm-up, motion with
   left-and-right turns for IMU-mount alignment, then a straight
   segment for attitude initialization. The automotive/e-scooter
   tables in the manual target speeds we can't hit; convergence at
   dog walking speed is an open question.

Verification path: monitor `UBX-ESF-STATUS.fusionMode = 1:FUSION`
continuously, `UBX-ESF-ALG.status = 3:COARSE` or `4:FINE ALIGNED`,
and `NAV-PVT.headVehValid` bit set. The local `dev/gps_docs/README.md`
walks through this as a punch list of remaining work.

For the current roadmap (capture + geotag missions through
`07_tennis_quadrat_pair`), **none of this F9R tuning is required.**
Bearing decisions continue to come from the Go2's own IMU with the
navigator's GPS-position-derived calibration (field-proven to 1-6°
per the 2026-03-19 entry). F9R heading data is recorded as sidecar
metadata only. If and when sub-degree true-north heading becomes
worth the setup cost — e.g. for repeatable change-detection surveys —
the appendix in `dev/gps_docs/README.md` has the step-by-step.

### Fix: NAV-PVT parser was reading the wrong bytes

Cross-referenced `src/go2_survey/gps.py` against the F9 HPS 1.30
Interface Description (UBX-22010984, §3.15.15.1) and found **two
independent parse bugs**. Both fixed in this commit.

**Bug 1 — `struct.unpack` format misalignment at offsets 12-23.**
The format string `"<IBIBBB"` assigned 1 byte to `nano` (spec says
I4, 4 bytes) and 4 bytes to `fixType` (spec says U1, 1 byte). The
downstream `flags`/`flags2`/`numSV` bytes landed at the right offsets
by coincidence of byte-count totals, but `fixType` as we stored it
was garbage — bytes 17-20 of the payload read as a U4, which packs
the upper three bytes of the real `nano` field in with the real
`fixType` byte. That's the origin of the `fixType=67108757` numbers
that show up in the pre-refactor 2025-09 logs. Corrected format
string: `"<IiBBBB"` (capital I for unsigned tAcc, lowercase i for
signed nano, four single bytes for the rest).

**Bug 2 — `carrSoln` and `diffSoln` reading wrong bits.** Per spec,
the `flags` byte at offset 21 has:
- bit 0: `gnssFixOk`
- bit 1: `diffSoln`
- bit 5: `headVehValid`
- bits 6-7: `carrSoln` (0 = no RTK, 1 = float, 2 = fixed)

Our code was doing `carrSoln = flags2 & 0x03` — reading the low two
bits of `flags2`, a *different* byte, which per spec are reserved.
And `diffSoln = flags & 0x01` — that's bit 0 (gnssFixOk), not bit 1.
So what we called `carrSoln` was effectively always 0, and what we
called `diffSoln` was actually the generic fix-valid flag.

Corrected: `carrSoln = (flags >> 6) & 0x03`,
`diffSoln = bool(flags & 0x02)`. Only `headVehValid` (bit 5) was
extracted correctly before.

**Bug 3 (separate issue, also fixed) — `headAcc` at wrong offset.**
Existing code read offset 88-91 as `headAcc`. Per spec, offset 72-75
is the U4 `headAcc` (applies to both motion and vehicle heading);
offset 88-91 is `magDec` (I2) + `magAcc` (U2) — magnetic declination
fields. Our `RTKPosition.head_vehicle_accuracy` has therefore been
populated with combined magnetic-declination bytes interpreted as
heading accuracy. Corrected to read offset 72.

**Bonus: `flags3` now parsed.** The U2 at offset 78 carries
`invalidLlh` (bit 0) and `lastCorrectionAge` (bits 1-4, 12 quantized
age bins 0-1s up to ≥120s). `lastCorrectionAge` is now surfaced as
`RTKPosition.correction_age_bin` — gives us differential correction
age for free, no extra UBX poll required. Earlier plan had us polling
NAV-STATUS to get this; that's now unnecessary.

### Reliability impact on prior missions — position yes, fix label no

The above findings prompt a clear split in what to trust from prior
runs:

**Reliable retroactively:**
- **Lat / lon values.** Offsets 24-31 were parsed correctly from the
  very beginning. Every waypoint we've ever reached was the waypoint
  on the map.
- **Horizontal accuracy (`hAcc`).** Offset 40-43, correct. The 14 mm
  hAcc claims in archived logs reflect genuine RTK Fixed performance.
- **Ellipsoidal height (`height`).** Offset 32-35, correct. `hMSL`
  was also parsed (offset 36-39) but until the 2026-04-16 surfacing
  commit (this entry) it was silently thrown away before reaching
  `RTKPosition`.
- **`max_hacc = 0.10m` navigation safety gate.** Runs against the
  correctly-parsed `hAcc`, so pauses and aborts on degraded
  accuracy worked as intended.

**NOT reliable retroactively:**
- **Any statement of the form "we achieved RTK Fixed at time T"** —
  based on the broken `carrSoln` read, which was always 0. The fix
  might have been RTK Fixed, RTK Float, or GNSS-only at those
  moments; our code couldn't distinguish.
- **`fix_type` labels in logs** (e.g. "GPS RTK Fixed ACHIEVED" banner
  lines). These reported based on broken parse. The fact that
  missions succeeded is evidence that `hAcc` was 14 mm so fix quality
  *was* good, but we can't cite those log banners as independent
  confirmation.
- **The `min_fix_type = 4` pre-flight gate.** Effectively a no-op
  — the garbage `fixType` U4 was almost always a large number that
  trivially passes `>= 4`. The `max_hacc` check carried the safety
  role instead.

Net: no mission ever went to the wrong coordinates. No position was
ever reported with false precision. The "RTK Fixed" claims in logs
are post-hoc defensible via `hAcc` evidence even if they weren't
technically computed from the correct bits.

### Mission folder renames

`dev/missions/mission_00` → `dev/missions/00_parking_lot` and
`dev/missions/mission_01` → `dev/missions/01_tennis_court`. Folder
names now describe the physical site. `resolve_mission_dir()` is
folder-name-agnostic so this is purely cosmetic — no code changes
required beyond README / install.md / cli.py docstring examples.
Internal `name` fields inside each `mission.toml` and
`waypoints.geojson` updated to match.

### Deleted: `02_probe_sparkfun_data`

The probe mission was originally queued as the first new mission on
the roadmap, explicitly to answer the `headAcc` offset ambiguity via
empirical measurement. With `dev/gps_docs/` mirrored locally, the
offset is answered authoritatively by reading the Interface
Description PDF in 30 seconds — the empirical approach became
weaker evidence than the spec it was probing against. Mission folder
deleted. The underlying infrastructure (`probes.py` module, new UBX
pollers for NAV-HPPOSLLH/NAV-DOP/NAV-SAT/NAV-STATUS/MON-VER,
`probe_gps` mission mode in `mission_runner.py`) is retained as
reusable scaffolding for a future motion-based `headVeh` validation
probe, which would actually answer a question the manual can't (will
F9R HPS converge at dog walking speed?).

### Files touched

- `src/go2_survey/gps.py` — NAV-PVT parser corrected; `RTKPosition`
  gains `correction_age_bin`; fix-achieved banner now logs real
  `carrSoln`-derived fix label, real `pDOP`, real `hMSL`.
- `src/go2_survey/probes.py` — dropped the dual-offset headAcc
  comparison (now settled by spec); simplified to single-value
  reporting.
- `dev/missions/00_parking_lot/` — renamed from `mission_00`.
- `dev/missions/01_tennis_court/` — renamed from `mission_01`.
- `dev/missions/02_probe_sparkfun_data/` — deleted.
- `dev/gps_docs/` — new folder, four u-blox PDFs + 450-line README.
- `README.md`, `setup/install.md`, `src/go2_survey/cli.py` — example
  mission names updated to new folder names.

### Verification

```bash
go2-survey list
# 00_parking_lot
# 01_tennis_court

go2-survey run 00_parking_lot --dry-run
go2-survey run 01_tennis_court --dry-run
# both pass
```

Next real run (post-fix) will log genuine `carrSoln` values for the
first time. Expect "RTK Fixed" banner lines to correspond to actual
carrSoln=2 rather than rubber-stamp nearly-always-true old behavior.

### Phase C: Capture infrastructure + five new missions

Same day, separate track of work: built the end-to-end plumbing for
camera capture + geotagging, plus five new missions that exercise it
in a layered hardware bring-up sequence. All landed in two commits
(`9dcfaee` infrastructure, `2a07212` missions) on top of the F9R
parser fix.

**Bearing-source decision baked into capture code.** After the F9R
deep-read made clear that HPS calibration is expensive and the speed
regime may not converge at 1.8 km/h, the `RotatingQuadratStrategy`
uses the Go2's own body IMU plus the navigator's GPS-derived
calibration offset (field-proven to 1-6° per the 2026-03-19 entry).
F9R `headVeh` and `headAcc` are recorded in sidecar metadata only —
never drive control flow. No F9R config is required for any of the
camera missions. The configuration appendix in `dev/gps_docs/README.md`
remains a future upgrade path if sub-degree heading ever becomes a
blocker.

**Infrastructure (`9dcfaee`):**

- `src/go2_survey/robot.py` — `Go2Robot.enable_video()` subscribes
  the robot's WebRTC video track via `unitree_webrtc_connect` (see
  `dev/webrtc_docs/video_frame_captures.md` for the library quirks).
  A long-running consumer task pulls `av.VideoFrame` instances with
  `track.recv()`, converts to numpy BGR with `to_ndarray("bgr24")`,
  and caches the latest frame + timestamp. `get_latest_frame(max_age)`
  mirrors the existing IMU-caching API.
- `src/go2_survey/vision/frames.py` — replaces stub. `FrameResult`
  dataclass + async `capture_frame(robot, max_age, wait_timeout)`
  that polls the robot's cache until a fresh frame arrives and
  returns a defensive copy.
- `src/go2_survey/vision/geotag.py` — replaces stub.
  `write_geotagged_jpeg()` encodes BGR→RGB→JPEG via Pillow and writes
  EXIF GPSInfo (lat/lon DMS, hMSL as GPSAltitude per EXIF convention,
  hAcc as GPSHPositioningError, bearing as GPSImgDirection 'T' for
  true north, UTC timestamp, WGS-84 datum). A sidecar JSON next to
  the JPEG carries everything EXIF can't cleanly represent: full
  `RTKPosition` dict (ellipsoidal height, pDOP, numSV,
  correction_age_bin, fix_type), heading source label, mission
  context, frame metadata. Smoke-tested end-to-end — EXIF round-trips
  through PIL without loss.
- `src/go2_survey/gps.py` — `GPSManager.average_position(duration,
  min_samples)` polls samples over a window and returns a
  synthetic `RTKPosition` with mean lat/lon/alt/hMSL and worst-case
  hAcc/vAcc/pDOP. Used by capture strategies at waypoint stops to
  beat down short-term RTK noise before stamping the frame.
- `src/go2_survey/navigator.py` — new
  `WaypointNavigator.turn_to_bearing(target, tolerance, timeout)`.
  Reuses the existing calibrated-heading machinery (robot IMU +
  GPS-derived offset from the approach leg) to pivot in place to an
  absolute true-north bearing. Required for rotating quadrat.
- `src/go2_survey/capture.py` (new) — `CaptureStrategy` base +
  concrete `NoOpStrategy`, `WaypointForwardStrategy`,
  `RotatingQuadratStrategy`. Registry dict `STRATEGIES` +
  `build_strategy(settings)` factory. The rotating quadrat uses
  absolute-bearing semantics (each rotation to an absolute target
  via `turn_to_bearing`, not relative to the previous capture) so a
  single bad rotation doesn't poison subsequent ones.
- `src/go2_survey/config.py` — `CaptureSettings` dataclass
  (strategy, settle_time, gps_avg_sec, bearings list, output_subdir,
  frame_max_age, frame_wait_timeout, turn_tolerance_deg,
  turn_timeout_sec). Wired through the TOML loader. Defaults
  (`strategy = "none"`) preserve pre-capture behavior for existing
  nav missions.
- `src/go2_survey/mission_runner.py` — nav mode now enables video
  when `capture.strategy != "none"` and invokes the strategy between
  `navigate_to()` and the existing `on_waypoint_reached` hook. Two
  new modes land alongside `probe_gps`:
    - `static_camera` — robot only, no GPS, no navigation. Plain
      JPEG captures for lab-bench camera smoke tests.
    - `static_geotag` — GPS + RTK + camera, robot in place. Full
      EXIF + sidecar pipeline test without navigation.

**New missions (`2a07212`):**

| Mission | Mode | Strategy | Requires | Produces |
|---|---|---|---|---|
| `02_camera_test` | `static_camera` | `waypoint_forward` | robot | 1 JPEG |
| `03_static_geotag` | `static_geotag` | `waypoint_forward` | robot + RTK + NTRIP | 1 JPEG + 1 sidecar |
| `04_tennis_single_forward` | `nav` | `waypoint_forward` | robot + RTK + nav | 1 JPEG + 1 sidecar |
| `05_tennis_single_quadrat` | `nav` | `rotating_quadrat` | robot + RTK + nav | 4 JPEGs + 4 sidecars |
| `06_tennis_quadrat_pair` | `nav` | `rotating_quadrat` | robot + RTK + nav | 8 JPEGs + 8 sidecars |

Missions 04-06 use tennis-court waypoints from `01_tennis_court`
(wp1 only for 04/05, both waypoints for 06). Missions 02 and 03
are static and need no `waypoints.geojson` — the runner's static
dispatch skips that load.

Each mission isolates one new capability on top of the previous:
camera path (02) → metadata pipeline (03) → nav+capture composition
(04) → rotation + multi-capture (05) → multi-waypoint survey
shape (06). A failure at any level localizes to that commit.

**Artifact layout per mission:**

```
dev/missions/<name>/
├── mission.toml
├── waypoints.geojson              # nav modes only
├── logs/<name>_<ts>.log           # existing convention, unchanged
└── captures/                      # new — created on first capture
    ├── <waypoint>_b<bearing>_<ts>.jpg
    └── <waypoint>_b<bearing>_<ts>.json    # sidecar
```

Verification: all seven missions show in `go2-survey list` and
dry-run clean. Live hardware runs deferred to a real bring-up
session — the bottom-up mission ladder (02 → 03 → 04 → 05 → 06) is
designed for exactly that.

**Botanical field site missions remain deferred** — `06` is the
furthest-right mission in this phase per the plan. Site-specific
surveys are a later increment once the pipeline is shaken out on
the tennis court.

### Follow-up: Python 3.8 legacy stripped repo-wide

The 2026-04-14 Jetson-compat work walked `requires-python` back to
`>=3.8` to fit the old Jetson Orin Nano image. The current deployment
target is Jetpack 6.2 / Python 3.10, so those 3.8-era compromises are
now load-bearing only for a platform we don't support. Removed
everything tied to 3.8 and migrated the annotation style to the
syntax 3.10 can evaluate natively.

**Packaging + docs**

- `pyproject.toml:10` — `requires-python = ">=3.8"` → `">=3.10"`.
- `README.md` — dropped the "(Python 3.8+)" parenthetical from the
  install example; deleted the Roadmap bullets about verifying 3.8
  deployment and testing Python > 3.8 (both superseded by the move).
- `setup/install.md:7` — prerequisite rewritten to "Python 3.10+"
  with the accurate explanation that the `tomli` backport is still
  used on 3.10 and stdlib `tomllib` on 3.11+.

**Typing syntax — every module in `src/go2_survey/`**

Mechanical rewrite across 15 files (discovery, gps, robot, navigator,
mission_runner, cli, capture, ntrip, probes, config, waypoints,
geometry, logging_utils, vision/frames, vision/geotag):

- `Optional[X]` → `X | None` and `Union[A, B]` → `A | B` everywhere
  (PEP 604, ~60 sites). The `Optional` / `Union` imports drop out of
  every `from typing import ...` line as a consequence.
- `List[X]`, `Dict[K, V]`, `Tuple[...]`, `Set[X]` → lowercase builtin
  generics `list[X]`, `dict[K, V]`, `tuple[...]`, `set[X]`
  (PEP 585, ~40 sites). Same imports drop out.
- `from __future__ import annotations` removed from every module.
  Not strictly a 3.8 workaround, but the only reason it had been
  added uniformly was to smooth PEP 604/585 syntax on older
  interpreters. On 3.10+ the native syntax evaluates eagerly and
  correctly without it.
- Edge case preserved: `capture.py::CaptureContext` uses
  `TYPE_CHECKING`-gated imports for `WaypointNavigator`,
  `CaptureSettings`, and `Waypoint`. Those annotations are kept as
  string literals (`"WaypointNavigator | None"`, etc.) so the
  deferred imports still work as type-check-only references.

**What didn't change**

- `tomli` fallback in `config.py:19-22` and its dependency line in
  `pyproject.toml:16` (`python_version < '3.11'` marker). `tomllib`
  only entered stdlib in 3.11, so on 3.10 the backport is still
  required. Comment retitled from "Python 3.8-3.10 backport" to
  "Python <3.11 backport" to reflect the current floor.
- Historical changelog entries mentioning 3.8 compatibility (e.g.
  the 2026-04-14 entry) are left as written — they're accurate for
  the state of the repo at the time.

Regression coverage: every module imports cleanly on 3.10, and both
`go2-survey run 00_parking_lot --dry-run` and `01_tennis_court
--dry-run` complete as before.

### Follow-up: docs reorganized under `dev/docs/`

Both vendor-reference doc piles had sat at the top of `dev/` with
inconsistent shape (`gps_docs/` was one README + four PDFs;
`webrtc_docs/` was three overlapping `.md` files). Consolidated into
a common parent that future doc folders (Jetson platform notes, etc.)
can slot into cleanly.

- `git mv dev/gps_docs/` → `dev/docs/gps/` — preserves rename
  history on the 450-line README and the four u-blox PDFs.
- `dev/webrtc_docs/{imu_data,obstacle_avoidance,video_frame_captures}.md`
  merged into a single `dev/docs/webrtc/README.md`, deduped and
  reordered in library-primitive order (connection → IMU → video →
  obstacle avoidance). Dropped the stale "Design implications for
  `vision/frames.py`" speculation (now implemented in
  `src/go2_survey/{robot.py, vision/frames.py}`). Fixed the stale
  `autonomous_nav/nav_utils.py` / `autonomous_nav/mission/changelog.md`
  paths in the old IMU notes during merge.
- Reference sweep: `README.md:140` tree diagram updated;
  `src/go2_survey/robot.py:47, 142` doc-comment pointers repointed
  at `dev/docs/webrtc/README.md`; `dev/docs/gps/README.md:378`
  internal reference to `dev/gps_docs/mount/` → `dev/docs/gps/mount/`.

Historical entries above (and below) this one still reference
`dev/gps_docs/` and `dev/webrtc_docs/` — left untouched per the
append-only convention; those paths were correct at the time those
entries were written.

---

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
