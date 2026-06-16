# Go2 WebRTC notes

Reference notes for the `unitree_webrtc_connect` library we use to
talk to the Go2 — IMU state, video frame capture, and the control
surface for peripheral features like obstacle avoidance. Distilled
from reading the installed library source at
`site-packages/unitree_webrtc_connect/`; these are descriptive (what
the library does), not a design document.

**Library:** <https://github.com/legion1581/unitree_webrtc_connect>
**Install:** `pip install unitree_webrtc_connect` (transitive dep of
this project).

## Connection model

Our wrapper (`src/go2_survey/robot.py::Go2Robot`) owns one
`UnitreeWebRTCConnection` instance at `self.conn`. After
`await self.conn.connect()` returns, the library has already wired
up:

```
self.conn.pc            # aiortc RTCPeerConnection
self.conn.datachannel   # control-plane pub/sub
self.conn.audio         # we don't use this
self.conn.video         # WebRTCVideoChannel; recvonly video transceiver
```

Control commands and sensor streams go through
`self.conn.datachannel.pub_sub`; video frames come through
`self.conn.video` once the channel is turned on. Topic constants
live in `unitree_webrtc_connect.constants.RTC_TOPIC` — always use
these rather than hardcoding topic strings.

## IMU access

### Available topics

| Topic constant | Topic string | Notes |
|---|---|---|
| `LOW_STATE` | `rt/lf/lowstate` | Low-level state + motor states + battery |
| `SPORT_MOD_STATE` | `rt/sportmodestate` | Sport mode state + nav data |
| `LF_SPORT_MOD_STATE` | `rt/lf/sportmodestate` | **Recommended** — good rate, IMU + locomotion |

### IMU payload

`message['data']['imu_state']` carries:

| Field | Type | Units | Description |
|---|---|---|---|
| `quaternion` | list[4] | — | Orientation `[w, x, y, z]` |
| `gyroscope` | list[3] | rad/s | Angular velocity `[x, y, z]` |
| `accelerometer` | list[3] | m/s² | Linear acceleration `[x, y, z]` |
| `rpy` | list[3] | rad | Roll, pitch, yaw Euler angles |
| `temperature` | float | °C | Sensor temperature |

### Basic subscription

```python
import asyncio
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC

async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.1.105")
    await conn.connect()

    def imu_callback(message):
        imu = message['data']['imu_state']
        print(f"RPY: {imu['rpy']} rad | temp: {imu['temperature']} °C")

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], imu_callback)
    await asyncio.sleep(3600)

asyncio.run(main())
```

### Other fields in sport-mode-state

The same message payload also carries `position`, `velocity`,
`yaw_speed`, `body_height`, `foot_force`, `foot_position_body`,
`foot_speed_body`, `gait_type`, and `mode`. We don't currently
consume any of these but they're there if a future mission needs
them.

## Battery access (`LOW_STATE` / `rt/lf/lowstate`)

Subscribe to `RTC_TOPIC['LOW_STATE']`; the message carries battery state in
`message['data']['bms_state']`, with pack voltage alongside it at
`message['data']['power_v']`. Consumed by `go2_survey.battery` (parsed into a
`BatteryState` and logged to `battery.log` + the main-log banners); see also
`Go2Robot.get_battery_state()`.

| Field | Path | Units | Description |
|---|---|---|---|
| `soc` | `bms_state['soc']` | % | **State of charge, 0–100** |
| `current` | `bms_state['current']` | mA | Pack current (negative = discharging) |
| `cycle` | `bms_state['cycle']` | — | Charge-cycle count (battery health) |
| `bq_ntc` | `bms_state['bq_ntc']` | °C | Battery NTC temps `[t1, t2]` |
| `mcu_ntc` | `bms_state['mcu_ntc']` | °C | MCU NTC temps `[t1, t2]` |
| `power_v` | `data['power_v']` | V | Pack voltage (sibling of `bms_state`) |

The full lowstate payload also includes `motor_state[]`, `foot_force`,
`temperature_ntc1`, and `imu_state`. Field names confirmed against the
library's `examples/go2/data_channel/lowstate/lowstate.py`.

### Integration in this repo

Implemented in `src/go2_survey/robot.py`:

- `Go2Robot._on_sport_state` subscribes to `LF_SPORT_MOD_STATE`
  during `connect()` and caches the latest `imu_state` dict.
- `Go2Robot.get_yaw_degrees(max_age=...)` returns yaw in degrees
  (Go2 IMU is CCW-positive, relative to power-on orientation).
- `WaypointNavigator.get_calibrated_heading()` in
  `src/go2_survey/navigator.py` converts that raw yaw to a
  true-north heading using the GPS-derived calibration offset
  established during the first walk leg.

See the 2026-03-19 entry in `dev/changelog.md` for the
IMU-GPS sensor-fusion strategy that drives this.

### Callback notes

- `subscribe()` and its callback are **synchronous**; the callback
  runs on the data channel's thread. If multiple threads touch the
  cache, add a lock.
- `await conn.connect()` must complete before `subscribe()`.

## Video frame capture

### What the library sets up

From `webrtc_driver.py::init_webrtc()`:

```python
self.pc = RTCPeerConnection(configuration)
self.datachannel = WebRTCDataChannel(self, self.pc)
self.audio = WebRTCAudioChannel(self.pc, self.datachannel)
self.video = WebRTCVideoChannel(self.pc, self.datachannel)
```

And `WebRTCVideoChannel.__init__`:

```python
def __init__(self, pc, datachannel):
    self.pc = pc
    self.pc.addTransceiver("video", direction="recvonly")
    self.datachannel = datachannel
    self.track_callbacks = []
```

By the time `Go2Robot.connect()` returns:

- Peer connection has a `recvonly` video transceiver negotiated.
- `self.conn.video` exists and accepts callbacks.
- **No frames are flowing yet** — the robot doesn't push until told.

### Three moving parts needed to receive a frame

**1. Register a track callback** (`webrtc_video.py::add_track_callback`):

```python
def add_track_callback(self, callback):
    if callable(callback):
        self.track_callbacks.append(callback)
```

The callback must be `async def callback(track): ...` — the library
awaits it. Register it **before** `conn.connect()` so it's in place
when the library's internal `on_track` handler fires.

**2. The library's `@pc.on("track")` handler**
(`webrtc_driver.py`):

```python
@self.pc.on("track")
async def on_track(track):
    if track.kind == "video":
        frame = await track.recv()          # <-- library consumes one frame
        await self.video.track_handler(track)
```

And `track_handler` (`webrtc_video.py`):

```python
async def track_handler(self, track):
    for callback in self.track_callbacks:
        await callback(track)
```

**Important quirk.** The library's `on_track` awaits one frame
itself before calling our callback. That first frame is discarded;
our callback receives the `track` *after* one frame has been pulled
off. Start your own `recv()` loop; the *next* frame is the first one
your code sees.

Also: `track_handler` is called **once per track**, not per frame.
The callback owns its own `while True: await track.recv()` loop —
if it returns without looping, no more frames are delivered.

**3. Turn the track on** (`webrtc_datachannel.py::switchVideoChannel`):

```python
def switchVideoChannel(self, switch: bool):
    self.pub_sub.publish_without_callback(
        "", "on" if switch else "off", DATA_CHANNEL_TYPE["VID"],
    )
```

Fire-and-forget — there's no confirmation. You know it worked
because frames start arriving. Either of these invocations works:

```python
conn.video.switchVideoChannel(True)
conn.datachannel.switchVideoChannel(True)
```

### End-to-end sequence

```python
async def on_video_track(track):
    while True:
        frame = await track.recv()                        # av.VideoFrame
        img = frame.to_ndarray(format="bgr24")            # numpy BGR, OpenCV-ready
        # keep latest, or push onto a queue, etc.

conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=robot_ip)
conn.video.add_track_callback(on_video_track)             # BEFORE connect()
await conn.connect()                                      # SDP negotiation; on_track fires
conn.video.switchVideoChannel(True)                       # robot starts pushing frames
# ... frames now flow into on_video_track ...
conn.video.switchVideoChannel(False)                      # stop when done
```

Registering the callback must happen before `connect()`. The library
registers its `@pc.on("track")` handler during construction, and the
track can arrive any time after the SDP exchange — registering after
`connect()` returns leaves a race where frames fire into an empty
callback list.

### Integration in this repo

Implemented in `src/go2_survey/robot.py::Go2Robot`:

- `enable_video()` registers a track callback and toggles the channel
  on. The callback spawns a long-running task that pulls frames,
  converts to BGR ndarray via `frame.to_ndarray(format="bgr24")`, and
  caches the latest frame + wall-clock timestamp.
- `get_latest_frame(max_age=...)` mirrors the IMU-cache freshness
  check — returns `None` if the cached frame is older than `max_age`
  seconds.
- `disable_video()` flips the channel off and clears the cache.

Higher-level helper at `src/go2_survey/vision/frames.py::capture_frame`
waits for a fresh-enough frame and bundles it with metadata for the
geotagger.

### Transitive dependencies (already installed)

Pulled in by `unitree_webrtc_connect`:

- **`aiortc`** — Python WebRTC stack. `RTCPeerConnection`,
  track abstractions; `track.recv()` is async, returns
  `av.VideoFrame`.
- **`av`** — FFmpeg bindings. `VideoFrame.to_ndarray(format=...)`
  converts to numpy. H.264 decode happens inside `av` transparently.
- **`opencv-python`** — numpy interop + JPEG/PNG encode. We don't
  import `cv2` directly (Pillow handles JPEG encode in
  `vision/geotag.py`), but it's in the footprint.

### What the library does NOT provide

- **Camera selection.** Only `self.video` is exposed — no switching
  between front RGB / chin / belly cameras. Whatever the Go2 sends
  on the default video track is what we get.
- **Track metadata.** No resolution / codec / framerate info is
  surfaced — read that off the `av.VideoFrame` itself (`frame.width`,
  `frame.height`, `frame.time_base`, `frame.pts`).
- **Frame timestamps.** `av.VideoFrame.pts` is a monotonic encoder
  presentation timestamp, not wall-clock. For geotagging we use
  `time.time()` at the moment `recv()` returns — that's what
  correlates with the GPS timeline.
- **Buffering / keyframe logic.** First frames after
  `switchVideoChannel(True)` may be several seconds delayed while
  aiortc waits for a keyframe. Our approach is continuous video
  across the mission + latest-frame cache, so the keyframe wait
  amortizes over the whole run.

### Unknowns still to verify on hardware

- Actual resolution / framerate / codec of the default video track.
- Latency from `switchVideoChannel(True)` to first decoded frame
  (keyframe wait).
- Whether the track stops cleanly on `switchVideoChannel(False)` or
  whether additional teardown is needed.
- Whether enabling video affects control-plane WebRTC stability
  (unlikely but worth watching on the first capture mission).
- Whether `pc.on("track")` fires again after toggling off + on, or
  whether the track is persistent across cycles.

## Lidar data stream

The Go2's 4D LiDAR L1 (360° × 90° FOV, 0.05 m minimum detection
distance) is **not just a hardware feature behind obstacle avoidance**
— `unitree_webrtc_connect` exposes a full set of lidar topics plus a
built-in decoder. The driving discovery: `webrtc_datachannel.py`
treats any data-channel message whose topic contains `"utlidar"` as
a lidar frame and auto-runs it through the decoder before invoking
subscribers, so once you subscribe + flip the right switch, frames
arrive already in numpy/3D form.

Everything in this section is **read from library source** at
`site-packages/unitree_webrtc_connect/`; nothing here has been
validated on the actual Go2 yet. Verify before relying.

### Topics

Direct lidar (raw / encoded voxel data):

| Constant | Topic string | Notes |
|---|---|---|
| `ULIDAR` | `rt/utlidar/voxel_map` | Voxel map (uncompressed) |
| `ULIDAR_ARRAY` | `rt/utlidar/voxel_map_compressed` | Compressed voxel map — preferred over uncompressed |
| `ULIDAR_STATE` | `rt/utlidar/lidar_state` | Sensor health/status |
| `ULIDAR_SWITCH` | `rt/utlidar/switch` | Enable/disable the sensor itself |
| `ROBOTODOM` | `rt/utlidar/robot_pose` | Lidar-derived robot pose |

uSLAM (lidar-based SLAM stack):

| Constant | Topic string | Notes |
|---|---|---|
| `LIDAR_LOCALIZATION_CLOUD_POINT` | `rt/uslam/localization/cloud_world` | World-frame point cloud (localization) |
| `LIDAR_LOCALIZATION_ODOM` | `rt/uslam/localization/odom` | Localization odometry |
| `LIDAR_MAPPING_CLOUD_POINT` | `rt/uslam/frontend/cloud_world_ds` | World-frame downsampled mapping cloud |
| `LIDAR_MAPPING_ODOM` | `rt/uslam/frontend/odom` | Mapping odometry |
| `LIDAR_MAPPING_PCD_FILE` | `rt/uslam/cloud_map` | Persistent PCD-shaped map |
| `LIDAR_MAPPING_CMD` | `rt/uslam/client_command` | Mapping control commands |
| `LIDAR_MAPPING_SERVER_LOG` | `rt/uslam/server_log` | Mapping daemon log stream |
| `LIDAR_NAVIGATION_GLOBAL_PATH` | `rt/uslam/navigation/global_path` | Planned path through the map |

Two map-shaped (already 2D, "bitmap-friendly") topics:

| Constant | Topic string | Notes |
|---|---|---|
| `GRID_MAP` | `rt/mapping/grid_map` | 2D occupancy grid |
| `SLAM_PC_TO_IMAGE_LOCAL` | `rt/pctoimage_local` | Point cloud projected to image (local frame) |

(Discovered by scanning `unitree_webrtc_connect.constants.RTC_TOPIC`
for `lidar`/`scan`/`point`/`cloud`/`voxel`/`utlidar` substrings —
nothing else matched.)

### Built-in decoder

`unitree_webrtc_connect.lidar.lidar_decoder_unified.UnifiedLidarDecoder`
wraps two backends; default is `libvoxel`. Pick via
`datachannel.set_decoder("libvoxel" | "native")`.

**`libvoxel` (default — WASM, mesh-shaped output)**

- Decoder file: `lidar/lidar_decoder_libvoxel.py`
- Engine: `wasmtime` running `libvoxel.wasm` (shipped in the package)
- `decode(compressed, metadata)` returns:
  ```python
  {
      "point_count": int,
      "face_count":  int,
      "positions":   np.ndarray(uint8, faces*12 bytes),   # Three.js layout
      "uvs":         np.ndarray(uint8, faces*8 bytes),
      "indices":     np.ndarray(uint32, faces*6),
  }
  ```
- Mesh-rendering shape, not a bare point cloud — built for the
  Unitree web UI's 3D visualizer.

**`native` (pure Python + lz4 — point-cloud-shaped output)**

- Decoder file: `lidar/lidar_decoder_native.py`
- Dependency: `lz4.block` (transitive)
- `decode(compressed, metadata)` returns:
  ```python
  {"points": np.ndarray(float64, (N, 3))}   # world-frame (x, y, z) in meters
  ```
- Pipeline: `lz4.block.decompress` → unpack 16-byte-wide voxel
  bitfield → mask nonzero bits → `bits * resolution + origin` →
  numpy points.
- **This is the obvious choice for a "lidar bitmap at capture
  time"** use case — top-down rasterization of an (N, 3) numpy
  array is one `np.histogram2d` call.

Metadata fields the decoder consumes from the message payload:

| Field | Used by | Notes |
|---|---|---|
| `origin` | both | `[x, y, z]` world-frame origin of the voxel grid |
| `resolution` | native | Voxel size in meters (`bits_to_points` defaults to 0.05) |
| `src_size` | native | Uncompressed lz4 payload size |

### Auto-decode plumbing

`webrtc_datachannel.WebRTCDataChannel.deal_array_buffer_for_normal`
routes incoming binary messages:

```python
if "utlidar" in topic:
    decoded_data = self.decoder.decode(binary_data, decoded_json['data'])
    decoded_json['data']['data'] = decoded_data
```

Practical consequence: **subscriber callbacks for any `utlidar`
topic receive `message['data']['data']` already decoded** — the
decoder's return dict, not raw bytes. No manual decode step needed
in user code.

(A second binary frame shape — `header_1==2, header_2==0` — has its
own `deal_array_buffer_for_lidar` path that always decodes, but the
normal path is what `ULIDAR_ARRAY` flows through.)

### Traffic-saving gate (likely required)

`webrtc_datachannel.py` includes:

```python
# Should turn it on when subscribed to ulidar topic
async def disableTrafficSaving(self, switch: bool):
    ...
```

The library author's own comment indicates lidar streaming **requires
explicitly disabling traffic saving** — otherwise the data channel
likely throttles/suppresses high-bandwidth topics. So the minimum
sequence to receive a lidar frame is:

```python
from unitree_webrtc_connect.constants import RTC_TOPIC

def lidar_callback(message):
    payload = message["data"]["data"]    # already decoded by the library
    # native decoder: payload["points"] is (N, 3) float64 in world frame
    # libvoxel:       payload["positions"], ["indices"], etc.

conn.datachannel.set_decoder("native")                   # before subscribe
conn.datachannel.pub_sub.subscribe(
    RTC_TOPIC["ULIDAR_ARRAY"], lidar_callback,
)
await conn.datachannel.disableTrafficSaving(True)        # gate the stream open
# ... frames now flow ...
await conn.datachannel.disableTrafficSaving(False)       # stop when done
```

`set_decoder` is wired in `WebRTCDataChannel.__init__` to default to
`libvoxel`; call it again to switch to `native`.

### Unknowns to verify on hardware

Source-reading establishes the API surface. The following need a
probe mission before any feature work depends on them:

- Whether `ULIDAR_ARRAY` actually streams once `disableTrafficSaving(True)`
  is called, or whether `ULIDAR_SWITCH` must also be toggled first.
- The publish rate and per-frame point count of `ULIDAR_ARRAY` (sets
  the bandwidth + post-processing budget for capture-time bitmaps).
- Whether the `native` decoder's `(N, 3)` output is robot-body frame
  or world frame after the `origin + bits * resolution` transform —
  the math suggests world frame keyed off `metadata.origin`, but
  worth confirming against a known scene.
- Whether `GRID_MAP` / `SLAM_PC_TO_IMAGE_LOCAL` actually publish on
  the WebRTC channel (constants exist but no library code wires
  subscribers — they may require uSLAM to be running on the robot).
- Whether `libvoxel` and `native` decoders agree on geometry for the
  same frame (validates that the WASM decoder isn't doing something
  surprising).

A `probe_lidar` mission analogous to `probe_gps` is the natural
shape — subscribe, log frame metadata + first-frame shape, dump one
PNG bitmap and one PCD to disk, exit. Save under
`dev/missions/02_camera_test/runs/<TS>/` alongside log sidecars.

## Obstacle avoidance

Obstacle avoidance is the highest-level consumer of the lidar data
described above. It's **enabled by default** on the Go2.

### Toggling via the RC controller

| Action | Effect |
|---|---|
| X (click) | Avoidance on (default) |
| Y (long press 3 s) | Avoidance off |

You cannot use the Unitree app and the companion remote
simultaneously — pick one control surface.

### Programmatic access

The toggle topic exists but has no high-level wrapper:

| Item | Details |
|---|---|
| Topic constant | `RTC_TOPIC["OBSTACLES_AVOID"]` |
| Topic string | `rt/api/obstacles_avoid/request` |
| High-level API | None |
| Examples | None in the library |
| Documentation | None |

Using it would require low-level experimentation with
`publish_request_new()`. (Note: unlike the lidar topics above, this
control topic has no decoder plumbing — it's purely a command
endpoint.)

### Constraint

Obstacle avoidance **only works when the robot is moving forward.**

## References

- [unitree_webrtc_connect GitHub](https://github.com/legion1581/unitree_webrtc_connect)
- [Library constants](https://github.com/legion1581/unitree_webrtc_connect/blob/main/unitree_webrtc_connect/constants.py)
- [Sport mode state example](https://github.com/legion1581/unitree_webrtc_connect/blob/main/examples/go2/data_channel/sportmodestate/sportmodestate.py)
- [aiortc docs](https://aiortc.readthedocs.io/) — `RTCPeerConnection`,
  track handling, `MediaStreamTrack.recv()` semantics.
- [PyAV docs](https://pyav.org/docs/stable/) — `VideoFrame`,
  `to_ndarray()`.
- [Go2 User Manual](https://static.generation-robots.com/media/Go2-User-Manual.pdf)
- [Go2 Handheld Remote Control Manual](https://static.generation-robots.com/media/Go2-Handheld-Remote-Control.pdf)

**Library source files (installed locally):**

- `site-packages/unitree_webrtc_connect/webrtc_driver.py` — peer
  connection setup and `on_track` handler.
- `site-packages/unitree_webrtc_connect/webrtc_video.py` — short file;
  worth reading in full.
- `site-packages/unitree_webrtc_connect/webrtc_datachannel.py` —
  `switchVideoChannel`, `disableTrafficSaving`, `set_decoder`, and
  the auto-decode plumbing for `utlidar` topics.
- `site-packages/unitree_webrtc_connect/constants.py` — exhaustive
  `RTC_TOPIC` dict. Scan it before assuming a feature is unavailable.
- `site-packages/unitree_webrtc_connect/lidar/lidar_decoder_unified.py`
  — decoder selector (`libvoxel` vs `native`).
- `site-packages/unitree_webrtc_connect/lidar/lidar_decoder_native.py`
  — pure-Python lz4 → numpy (N, 3) point cloud decoder.
- `site-packages/unitree_webrtc_connect/lidar/lidar_decoder_libvoxel.py`
  — WASM wrapper that returns Three.js mesh data.
