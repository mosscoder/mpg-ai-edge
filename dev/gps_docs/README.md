# GPS docs for ZED-F9R-03B-00

Official u-blox + SparkFun reference material for the SparkFun
GPS-RTK Dead Reckoning Breakout (SMA variant) with the
**ZED-F9R-03B-00** module running firmware **FW1.00 HPS1.30**.

Downloaded 2026-04-16 from official content.u-blox.com and SparkFun
GitHub mirrors.

## Local PDFs

| File | u-blox Doc # | What's inside | Use case |
|---|---|---|---|
| `ZED-F9R-03B_DataSheet_UBX-22024085.pdf` | UBX-22024085 | Electrical specs, pinout, performance, physical dimensions, ordering info | Hardware reference: absolute accuracy claims (e.g. "heading accuracy 0.2° post-init"), current draw, mechanical drawing |
| `ZED-F9R_IntegrationManual_UBX-20039643.pdf` | UBX-20039643 | **Most important for us.** Mounting guidance, IMU axes, lever arm + vehicle-frame rotation config, antenna placement, calibration procedures, RTCM setup, sensor fusion tuning | Configuring `CFG-SFIMU-IMU_TO_VEH_*` and `CFG-SFIMU-IMU_INT_LEVERARM_*` if we ever enable HPS for heading |
| `u-blox-F9-HPS-1.30_InterfaceDescription_UBX-22010984.pdf` | UBX-22010984 | Complete UBX protocol spec for HPS 1.30: every message definition, byte offsets, flag bits, config keys | **Ground truth for the headAcc offset question** (probe reference). Also definitive source for NAV-PVT / NAV-HPPOSLLH / NAV-DOP / NAV-SAT / NAV-STATUS / MON-VER / ESF-STATUS payload layouts used by `gps.py` |
| `ZED-F9R-03B_FW1.00HPS1.30_ReleaseNotes_UBX-22035201.pdf` | UBX-22035201 | Firmware-specific changes, known issues, config defaults shipped with the module | Check before upgrading firmware or diagnosing odd behavior |

## Web resources not mirrored here

These are HTML pages or CAD source files, more useful as live links
than static snapshots.

- **SparkFun hookup guide:** [learn.sparkfun.com — SparkFun GPS-RTK Dead Reckoning ZED-F9R Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-gps-rtk-dead-reckoning-zed-f9r-hookup-guide)
  Covers board jumpers, I²C/UART pin assignments, antenna connectors,
  and the default config shipped by SparkFun. Has an example calibration walk.
- **SparkFun SMA breakout hardware (Eagle CAD):** [GitHub — sparkfun/SparkFun_GPS_Dead_Reckoning_ZED-F9R/Hardware/SMA](https://github.com/sparkfun/SparkFun_GPS_Dead_Reckoning_ZED-F9R/tree/master/Hardware/SMA)
  `SparkFun_ZED-F9R_v11.sch` + `.brd`. No rendered PDF schematic in the
  repo — you need Autodesk Eagle (or KiCad with an import) to open.
- **u-blox product page:** [u-blox.com — ZED-F9R module](https://www.u-blox.com/en/product/zed-f9r-module)
  Evergreen link to the current datasheet / integration manual / firmware.
- **u-center 2 (Windows config tool):** [u-blox.com — u-center 2](https://www.u-blox.com/en/product/u-center)
  For one-time configuration of lever arm + vehicle-frame rotation if
  we ever enable F9R HPS. Alternative: send UBX CFG-VALSET messages
  from Python.

## Quick-reference index: which doc answers which question

| Question | Primary doc | Section |
|---|---|---|
| What byte offset is `headAcc` at in NAV-PVT? | Interface Description | UBX-NAV-PVT message definition |
| Does the F9R fill magDec/magAcc bytes on our firmware? | Interface Description + Release Notes | NAV-PVT payload + firmware changelog |
| What's the NAV-HPPOSLLH layout? | Interface Description | UBX-NAV-HPPOSLLH |
| How do I configure the antenna lever arm? | Integration Manual | "Sensor fusion" / "IMU mounting and calibration" chapter |
| Which IMU axes correspond to the chip package? | Integration Manual + Datasheet | Mechanical drawings + IMU axis diagram |
| How do I tell the F9R which direction is "forward"? | Integration Manual | "IMU-to-vehicle-frame" / `CFG-SFIMU-IMU_TO_VEH_*` |
| What auto-alignment modes exist, and what motion do they need? | Integration Manual | Sensor fusion initialization |
| What's the expected headVeh accuracy? | Datasheet | Performance specs table |
| What messages are enabled by default? | Release Notes + Integration Manual | Default message output config |
| What's the maximum message rate we can push NAV-PVT? | Integration Manual | Navigation rate config (`CFG-RATE`) |

## Key facts surfaced during download (cross-reference)

From the SparkFun hookup guide summary:

- **IMU orientation on the SparkFun breakout** (as mounted): X = North
  (vehicle front), Y = East (vehicle right), Z = Down. Right-handed
  NED frame.
- **Default I²C address:** 0x42.
- **UART1:** host communication. **UART2:** RTCM correction input.
- **RTK horizontal accuracy claim:** 0.01 m + 1 ppm (when RTK Fixed).
- **Heading / pitch / roll accuracy claim (post-init with HPS
  active):** 0.2° / 0.3° / 0.5°. These assume correctly-configured
  lever arm and vehicle frame.
- **Hot start time:** 2 s.

## Firmware confirmation

The probe mission (`02_probe_sparkfun_data`) polls `MON-VER` at
startup — cross-reference its output against the
`ZED-F9R-03B_FW1.00HPS1.30_ReleaseNotes_UBX-22035201.pdf` to confirm
the unit on our dog is actually on this firmware. If it reports a
different HPS revision, the interface description here may not
perfectly match (most differences are additive — new messages, not
changed layouts — but worth verifying).

---

# Configuring the F9R for sensor fusion (HPS)

Reference: **Integration Manual §3.2.4 – §3.2.7** (pages 25–45).
This section is a distilled how-to pulled from the manual; when in
doubt, the manual wins.

## Overview: what has to be true before `headVeh` becomes trustworthy

1. **Physical mount must be rigid** — antenna and F9R both firmly
   attached, no flex under motion. Our changelog already notes we did
   this pass.
2. **Configure lever arm** — tell the F9R where the antenna is
   relative to its internal IMU, in cm, in the **installation
   frame**.
3. **Configure IMU-mount alignment** — tell the F9R how the F9R chip
   is rotated relative to the vehicle's "forward" direction. Either
   manual (3 Euler angles) or automatic (requires a calibration
   drive).
4. **Supply speed/odometer data** OR accept **Degraded HPS mode**.
   The F9R was designed for vehicles with wheel-tick sensors. On the
   Go2 we don't have those; our options are (a) feed the Go2's body
   velocity to the F9R as `UBX-ESF-MEAS` speed data over the existing
   USB serial link, or (b) run without, in which case fusion enters
   Degraded HPS mode — fusion still works, but position can drift
   slightly when stationary.
5. **Run a calibration drive** — stationary warm-up, then motion with
   left-and-right turns for IMU-mount alignment, then a straight
   segment for attitude initialization.
6. **Verify via `UBX-ESF-STATUS` and `UBX-ESF-ALG`** — fusion mode =
   1 (FUSION), alignment status = 3 (COARSE ALIGNED) or 4 (FINE
   ALIGNED), per-sensor calibStatus = 2/3 (CALIBRATED). Only then is
   `NAV-PVT.headVeh` the 0.2°-datasheet-claim value.

## Reference frames — get this right or nothing else works

Three frames to keep straight:

| Frame | Axes | Used for |
|---|---|---|
| **IMU frame** | Native axes of the F9R chip's internal accelerometer/gyroscope. Orientation shown on the F9R package (see Fig 2, Integration Manual p. 26). | Internal to the receiver — we never need to work in this frame directly. |
| **Installation frame** | X = forward, **Y = left**, **Z = up**. Right-handed. Origin = IRP (center of IMU chip). | **This is the frame lever arm values and Euler alignment angles are expressed in.** |
| **Navigation vehicle frame** | X = forward, **Y = right**, **Z = down**. Right-handed. Origin = VRP. | Used only for attitude output in `UBX-NAV-ATT` — not for input configuration. |

**Critical gotcha:** installation frame is **Y-left / Z-up**, but the
vehicle attitude output frame is **Y-right / Z-down**. It's easy to
conflate them. When entering lever arm or IMU-mount alignment angles,
you are working in installation frame (Y-left, Z-up).

## Reference points — what the lever arm is "to" and "from"

- **IRP (IMU Reference Point)** — origin for lever arm. Center of the
  F9R chip package (see Fig 2, Integration Manual p. 26, which shows
  the IRP location 15 mm × 3 mm offset from the module's lower-right
  corner).
- **ARP (Antenna Reference Point)** — the antenna phase center, not
  the physical center of the antenna housing. For SparkFun's helical
  antenna this is within a few mm of geometric center of the radiating
  element; for sub-cm precision consult the antenna's own datasheet.
- **VRP (Vehicle Reference Point)** — rear-axle-center convention
  (used for automotive wheel-tick ingestion). Not directly meaningful
  for the dog; can be left at default or pointed at a useful body
  point.
- **CRP (Configurable Reference Point)** — optional output origin for
  the navigation solution. Lets you make the published position be at
  e.g. the camera rather than the IRP.

## Step 1: measure the lever arm

What you need:

- Tape measure with cm marks.
- Access to inspect (a) the F9R chip on the SparkFun board, and (b)
  the antenna as mounted on the dog.

Procedure:

1. Orient the dog to a "neutral forward" pose — standing square, nose
   forward, body centered over legs.
2. Identify the **center of the F9R chip** on the SparkFun PCB. It's
   the IC labeled `ZED-F9R-03B-00`. Target the geometric center of
   the package, not the PCB or the USB port.
3. Identify the **antenna phase center** — for a helical antenna, the
   geometric center of the element inside the housing; for a patch
   antenna, center of the top face.
4. With the dog in neutral pose, measure the three offsets from the
   F9R chip to the antenna phase center in **installation frame**
   (X = fwd, Y = left, Z = up), in cm:
   - `LA_X`: how far **forward** of the F9R the antenna is (negative
     if antenna is behind).
   - `LA_Y`: how far to the **left** (negative if to the right).
   - `LA_Z`: how far **up** (negative if below).

Example: antenna mounted 25 cm behind the F9R chip, centered
left/right, 18 cm above:
```
LA_X = -25
LA_Y =   0
LA_Z =  18
```

Resolution: 1 cm is the F9R's granularity. Tape measure precision is
sufficient. Manual says "at least 1 cm resolution" for consistency of
the positioning solution.

## Step 2: determine IMU-mount alignment

If the SparkFun board is mounted flat with the F9R chip's printed
**X arrow pointing toward the dog's nose**, the chip's IMU frame
already lines up with installation frame. All three misalignment
angles are 0.

If the board is rotated (chip's X arrow points to the dog's right, or
the board is mounted upside-down, etc.), you need three Euler angles
describing the rotation from installation frame to IMU frame, applied
in the order **YAW → PITCH → ROLL** (around z, then y', then x''):

- `CFG-SFIMU-IMU_MNTALG_YAW` — rotation about installation z-axis.
  Example: chip X-arrow pointing to dog's right instead of nose →
  YAW = 90°. (Positive rotation follows right-hand rule with thumb
  along +z = up.)
- `CFG-SFIMU-IMU_MNTALG_PITCH` — rotation about installation y-axis
  after the yaw rotation.
- `CFG-SFIMU-IMU_MNTALG_ROLL` — rotation about installation x-axis
  after yaw + pitch.

Required accuracy per manual: **< 5°** ideal, < 10° at absolute worst
(with `CFG-SFIMU-IMU_MNTALG_TOLERANCE = 1` for HIGH tolerance).

**Easier alternative: automatic alignment.** Set
`CFG-SFIMU-AUTO_MNTALG_ENA = 1` and the F9R estimates all three
angles on its own during a calibration drive (requires
left-and-right turns while moving). The catch: automotive HPS tables
specify ≥30 km/h for auto-alignment, e-scooter tables specify
≥10 km/h. Our dog walks at 0.5 m/s (1.8 km/h). We're well below the
speeds the manual validates auto-alignment against, so **user-defined
alignment is the safer path for us**.

## Step 3: the UBX config keys to actually set

All keys live in the F9 HPS 1.30 Interface Description (local PDF,
§ "Configuration items reference"). The keys below are the ones we
care about; the Interface Description has their exact numeric IDs
(0x40...) and data types (U1/U2/U4/I1/I2/I4).

### Lever arm

| Key | Description | Units | Notes |
|---|---|---|---|
| `CFG-SFIMU-IMU2ANT_X` | IRP → antenna, X component | cm (signed) | Installation frame |
| `CFG-SFIMU-IMU2ANT_Y` | IRP → antenna, Y | cm (signed) | " |
| `CFG-SFIMU-IMU2ANT_Z` | IRP → antenna, Z | cm (signed) | " |
| `CFG-SFODO-IMU2VRP_X/Y/Z` | IRP → VRP | cm (signed) | Leave at defaults unless we feed wheel ticks |
| `CFG-SFCORE-IMU2CRP_X/Y/Z` | IRP → configurable reference point | cm (signed) | Optional — use if we want NAV output referenced to the camera instead of the IRP |

### IMU mount alignment

| Key | Description | Units |
|---|---|---|
| `CFG-SFIMU-IMU_MNTALG_YAW` | Yaw from installation → IMU frame | deg × 1e-2 (typ.) |
| `CFG-SFIMU-IMU_MNTALG_PITCH` | Pitch | deg × 1e-2 |
| `CFG-SFIMU-IMU_MNTALG_ROLL` | Roll | deg × 1e-2 |
| `CFG-SFIMU-IMU_MNTALG_TOLERANCE` | 0 = LOW (< 2°), 1 = HIGH (< 10°) | — |
| `CFG-SFIMU-AUTO_MNTALG_ENA` | 1 = enable automatic alignment | bool |

### Fusion enable + related

| Key | Description |
|---|---|
| `CFG-SFCORE-USE_SF` | Master enable for HPS fusion (default on) |
| `CFG-RATE-NAV` | Navigation solution update rate (default 1 Hz; we set 5 Hz via `_configure_navigation_rate(200)` in `gps.py:71`) |
| `CFG-MOT-IMU_FILT_WINDOW` | IMU filter window (100–200 recommended for vibration-prone mounts — dog gait may qualify) |
| `CFG-SFODO-DIS_DIR_INFO` | 1 = directionless odometer mode (lets us feed speed without direction sign if we ever do) |

### Persistence layers

When sending `CFG-VALSET`, the `layers` bitmask determines where the
value lives:
- Bit 0 (RAM) — active immediately, lost on reset.
- Bit 1 (BBR) — battery-backed RAM, survives reset but not power loss.
- Bit 2 (Flash) — persists forever.

For one-time mount configuration: set all three (layers = 0x07) so the
value survives every state.

### Sending from Python

We already have UBX TX plumbing in `gps.py` via
`_send_ubx_message(msg_class, msg_id, payload)`. A `CFG-VALSET`
message is class `0x06`, ID `0x8A`. Payload format is documented in
the Interface Description. Rough sketch:

```python
# One CFG-VALSET can pack multiple key/value pairs.
# Header: version (1 B) | layers (1 B) | reserved (2 B)
# Body:   repeating <key_id (4 B, little-endian)><value (key-sized)>
header = struct.pack("<BBBB", 0, 0x07, 0, 0)  # version 0, all layers
body = struct.pack(
    "<I b",                   # CFG-SFIMU-IMU2ANT_X is I1 (signed byte)
    0x40010000 | KEY_ID,      # lookup from Interface Description
    -25,                      # value
)
# ... pack more key/value pairs ...
self._send_ubx_message(0x06, 0x8A, header + body)
```

The exact key IDs and value types come from the Interface Description
PDF. Until we implement this, u-center 2 (u-blox's GUI tool) is the
standard way to do one-time config — connects over USB, lets you set
keys by name, and persists to Flash with one click.

## Step 4: calibrate (the dog-specific part)

The manual's procedures target automotive (30 km/h+) and e-scooter
(10 km/h+) use cases. **A Go2 walks at 0.5 m/s = 1.8 km/h**, which is
slower than anything the manual validates. There are three
implications:

1. **IMU init** (stationary under good sky for 3 min): fine on any
   platform. Just stand still.
2. **INS init (position + velocity)**: fine — needs good GNSS fix,
   which is platform-agnostic.
3. **IMU-mount alignment init** (needs ≥30 km/h automotive or
   ≥10 km/h e-scooter with left/right turns): **this is where the dog
   may not work.** We can try, but the HPS alignment engine may never
   converge at 1.8 km/h. Mitigation: user-defined (manual) alignment
   via `CFG-SFIMU-IMU_MNTALG_*` instead of automatic. Measure the
   board orientation once with a protractor (within 5°) and skip this
   phase.
4. **Wheel tick initialization** (needs velocity data source): we
   don't have wheel ticks. Options:
   - **Feed Go2 body velocity as UBX-ESF-MEAS speed data (type 11)**
     over USB serial. The Go2 publishes body velocity in its WebRTC
     sport-state stream; we'd need to bridge that to the F9R's serial
     input. Non-trivial but possible.
   - **Degraded HPS mode**: don't feed any odometer data. Fusion
     still runs, but position can drift when stationary. May still
     give usable `headVeh` during motion.
   - **Directionless odometer mode** (`CFG-SFODO-DIS_DIR_INFO=1`): if
     we feed speed without direction, the F9R deals with it.
5. **INS attitude init** (100 m straight at ≥40 km/h automotive or
   ≥10 km/h e-scooter): also too fast for us. Same concern as #3 —
   may fail to converge.

**Practical calibration drive for the dog:**

```
1. Stand dog in open sky for 3+ minutes (IMU init).
2. Confirm RTK Fixed (INS position+velocity init).
3. Walk dog in a figure-eight or zigzag pattern for 2–5 minutes,
   including several ~90° turns in each direction.
4. Walk dog in a straight line for ≥20 m.
5. Check UBX-ESF-STATUS:
   - fusionMode                  should → 1 (FUSION)
   - imuInitStatus               2 (INITIALIZED)
   - insInitStatus               2 (INITIALIZED)
   - mntAlgStatus                2 (INITIALIZED) — or skip via user-defined
   - calibStatus (per sensor)    2 or 3 (CALIBRATED)
6. Check UBX-ESF-ALG.status      3 (COARSE ALIGNED) or 4 (FINE ALIGNED)
7. Check UBX-NAV-PVT.headVehValid bit — should be set.
```

If steps 3–4 at walking speed fail to initialize alignment after a
few minutes, fall back to user-defined misalignment angles.

## Step 5: verify

After calibration, during motion:

- `UBX-NAV-PVT.headVeh` should track the dog's true direction of
  motion within a couple degrees (post-calibration: ~0.2° per
  datasheet, more realistically 1–3° at our speeds).
- `UBX-NAV-PVT.headVehValid` flag bit 5 should be set continuously.
- `UBX-ESF-STATUS` should show `fusionMode=1:FUSION` continuously.
- `UBX-ESF-ALG.status` should be 3 or 4.

If any of these degrade during operation (e.g. alignment status drops
to 1 again), the mount may have shifted or RTK may have been lost for
long enough that fusion reset. Per manual: a cold start or mount
change triggers calibration reset.

---

# What remains to be done

Everything below is open work. The goal is to go from "we have a
stationary probe that reads raw F9R output" to "we can trust
`headVeh` as a bearing source and `NAV-PVT` velocity as a motion
signal during a mission."

## Hardware / measurement

- **Measure lever arm.** Tape-measure job. Need the dog in neutral
  pose, access to the F9R chip location, access to the antenna phase
  center. Record `LA_X`, `LA_Y`, `LA_Z` in installation frame. 15
  minutes of actual work.
- **Document the SparkFun board's orientation on the dog.** Take a
  photograph from above, note which direction the F9R chip's X arrow
  points relative to the dog's nose. If not perfectly forward,
  calculate `IMU_MNTALG_YAW/PITCH/ROLL`.
- **Photograph the mount** and save to `dev/gps_docs/mount/` so any
  future re-mount knows what "the correct geometry" looked like.
  Without this, every remount would require re-measuring from
  scratch.

## Software — config writer

- **Extend `gps.py` with a UBX CFG-VALSET helper.** Wraps
  `_send_ubx_message(0x06, 0x8A, ...)` with a clean API:
  `gps.set_config_keys({KEY_CFG_SFIMU_IMU2ANT_X: -25, ...}, layers="all")`.
- **Build a key-ID table** from the Interface Description PDF (pages
  list every key with numeric ID and data type). Ship as a Python
  module `ubx_keys.py` so the config helper has typed access.
- **Add a one-shot `go2-survey configure-f9r` CLI command** that
  takes lever arm + mount angles from `mission.toml` or command-line
  args and writes them to RAM+BBR+Flash, then reads back to confirm.

## Software — speed data bridge (optional but recommended)

- **Wire Go2 body velocity into F9R via `UBX-ESF-MEAS`.** The Go2
  publishes `LF_SPORT_MOD_STATE` over WebRTC (we already subscribe in
  `robot.py:70-73` for IMU yaw). The same message carries body
  velocity. We'd write a small bridge that pulls this velocity, packs
  it as `UBX-ESF-MEAS` data type 11 (speed, m/s × 1e-3), and sends it
  to the F9R's serial port at ≥5 Hz. With this, the F9R has full
  fusion data and exits Degraded HPS mode.
- Alternatively, skip this and live with Degraded HPS — see whether
  `headVeh` is usable without it first.

## Software — validation mission

- **`03_probe_headveh_motion` (suggested new mission).** After config
  + calibration drive, run a mission that walks a known path (e.g. a
  square), logging `NAV-PVT.headVeh`, `headVehValid`, `headAcc_72`,
  `ESF-STATUS.fusionMode`, `ESF-ALG.status` throughout. Compare
  `headVeh` against ground-truth bearing derived from the walked
  path. This is the empirical proof that F9R fusion works on our
  setup.
- Output: an "is `headVeh` trustworthy?" answer, plus the
  magnitude of heading error under normal walk dynamics.

## Software — integration

- **After validation:** flip the `RotatingQuadratStrategy` bearing
  source from robot IMU yaw to F9R `headVeh` (with robot IMU as
  fallback when `headVehValid` drops). One-line change in the
  strategy, conditional on an opt-in setting so existing capture
  missions aren't disrupted.
- **Sidecar JSON schema:** include `heading_source = "robot_imu" |
  "f9r_headveh"` alongside `heading_degrees` so downstream consumers
  know which pipeline produced the bearing in each image.

## Why any of this matters (and when it doesn't)

For the current roadmap (probe + camera missions through
`07_tennis_quadrat_pair`), **none of this config work is required.**
Robot IMU yaw with the existing navigator calibration is the bearing
source, and that's field-proven to 1–6°. The F9R stuff is a future
upgrade path that could get us to sub-degree heading accuracy and
velocity measurements independent of the Go2's IMU.

It becomes worth doing when:

- We want **repeatable sub-degree bearing** for change-detection
  surveys (same plant, same angle, different dates).
- The Go2 IMU calibration walk becomes a workflow annoyance (e.g.
  when we want to start a mission from an arbitrary pose without a
  1.5 m calibration walk).
- We need **true-north-referenced heading** directly, without
  post-hoc correction from GPS-position-derived bearing.

Until one of those triggers, the F9R stays in metadata-only mode and
this whole appendix is parked as a "when we're ready" reference.
