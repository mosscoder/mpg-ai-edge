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

## Obstacle avoidance

Obstacle avoidance is **enabled by default** on the Go2.

### Toggling via the RC controller

| Action | Effect |
|---|---|
| X (click) | Avoidance on (default) |
| Y (long press 3 s) | Avoidance off |

You cannot use the Unitree app and the companion remote
simultaneously — pick one control surface.

### Programmatic access

| Item | Details |
|---|---|
| Topic constant | `RTC_TOPIC["OBSTACLES_AVOID"]` |
| Topic string | `rt/api/obstacles_avoid/request` |
| High-level API | **None** |
| Examples | **None** in the library |
| Documentation | **None** |

The topic exists in `constants.py` but nothing calls it. Using it
would require low-level experimentation with `publish_request_new()`.

### Sensor

Obstacle avoidance uses the 4D LiDAR L1 (360° × 90° FOV; 0.05 m
minimum detection distance). It **only works when the robot is moving
forward.**

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
  `switchVideoChannel` lives here.
