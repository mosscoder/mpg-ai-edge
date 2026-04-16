# Video frame capture over WebRTC

Notes on how to pull still frames from the Go2's WebRTC video track,
distilled from reading the installed `unitree_webrtc_connect` source at
`site-packages/unitree_webrtc_connect/`. These notes are descriptive
(what the library does) — they are not a design for our `vision/frames.py`
yet.

## Module map (library side)

```
unitree_webrtc_connect/
├── webrtc_driver.py        # UnitreeWebRTCConnection — the object we already own as self.conn
├── webrtc_video.py         # WebRTCVideoChannel — exposed as self.conn.video
├── webrtc_datachannel.py   # switchVideoChannel() + pub/sub for control plane
└── webrtc_audio.py         # not used by us
```

We already use `UnitreeWebRTCConnection` and its `self.datachannel.pub_sub`
for commands/IMU state (`src/go2_survey/robot.py`). The same connection
object has a `self.video` attribute that has never been touched by our
code.

## What's already set up by `UnitreeWebRTCConnection.connect()`

From `webrtc_driver.py:96-104` (inside `init_webrtc()`):

```python
self.pc = RTCPeerConnection(configuration)
self.datachannel = WebRTCDataChannel(self, self.pc)
self.audio = WebRTCAudioChannel(self.pc, self.datachannel)
self.video = WebRTCVideoChannel(self.pc, self.datachannel)
```

And inside `WebRTCVideoChannel.__init__` (`webrtc_video.py:6-11`):

```python
def __init__(self, pc: RTCPeerConnection, datachannel: WebRTCDataChannel):
    self.pc = pc
    self.pc.addTransceiver("video", direction="recvonly")
    self.datachannel = datachannel
    self.track_callbacks = []
```

So by the time our `Go2Robot.connect()` returns successfully:

- The peer connection has a `recvonly` video transceiver — the SDP
  negotiation already included video.
- `self.conn.video` exists, ready to accept callbacks.
- **No frames are flowing yet** — the Go2 doesn't push video until told.

## The three moving parts needed to receive a frame

### 1. Register a track callback

`webrtc_video.py:16-23`:

```python
def add_track_callback(self, callback):
    if callable(callback):
        self.track_callbacks.append(callback)
    else:
        logging.warning(f"Callback {callback} is not callable.")
```

The callback must be `async def callback(track): ...` — it's awaited by
the library. Register it **before** `conn.connect()` returns, ideally
before we call it, so it's in place when the library's internal
`on_track` handler fires.

### 2. The library's internal `@pc.on("track")` handler

`webrtc_driver.py:156-169`:

```python
@self.pc.on("track")
async def on_track(track):
    logging.info("Track received: %s", track.kind)
    if track.kind == "video":
        # await for the first frame, #ToDo make the code more nicer
        frame = await track.recv()
        await self.video.track_handler(track)
    if track.kind == "audio":
        frame = await track.recv()
        while True:
            frame = await track.recv()
            await self.audio.frame_handler(frame)
```

And `track_handler` (`webrtc_video.py:25-33`):

```python
async def track_handler(self, track):
    logging.info("Receiving video frame")
    for callback in self.track_callbacks:
        try:
            await callback(track)
        except Exception as e:
            logging.error(f"Error in callback {callback}: {e}")
```

**Important quirk.** The library's `on_track` handler `await`s one frame
itself (`frame = await track.recv()`) before calling our callbacks. That
first frame is consumed and discarded — our callback gets the `track`
object *after* one frame has already been pulled off it. Our callback
should start its own `recv()` loop; the *next* frame is the first one
our code sees.

Also note that `track_handler` is called **once** per track, not per
frame. The callback is responsible for its own `while True: await
track.recv()` loop if it wants a continuous stream. If a callback
returns without looping, no more frames are delivered to it.

### 3. Turn the track on

`webrtc_datachannel.py:174-181`:

```python
#Enable/Disable video channel
def switchVideoChannel(self, switch: bool):
    self.pub_sub.publish_without_callback(
        "",
        "on" if switch else "off",
        DATA_CHANNEL_TYPE["VID"],
    )
    print(f"Video channel: {'on' if switch else 'off'}")
```

This is the "tell the robot to start pushing video frames" message. It's
fire-and-forget — `publish_without_callback` — so there's no
confirmation response. We'll know it worked because frames start
arriving at our callback.

Convenience pass-through on the video channel (`webrtc_video.py:13-14`):

```python
def switchVideoChannel(self, switch: bool):
    self.datachannel.switchVideoChannel(switch)
```

So either of these works:

```python
conn.video.switchVideoChannel(True)
conn.datachannel.switchVideoChannel(True)
```

## End-to-end sequence we'd need

```python
async def on_video_track(track):
    while True:
        frame = await track.recv()                         # av.VideoFrame
        img = frame.to_ndarray(format="bgr24")             # numpy BGR, OpenCV-ready
        # keep latest, or push onto a queue, etc.

conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=robot_ip)
conn.video.add_track_callback(on_video_track)              # BEFORE connect()
await conn.connect()                                       # SDP negotiation, on_track fires
conn.video.switchVideoChannel(True)                        # robot starts pushing frames
# ... frames now flow into on_video_track ...
conn.video.switchVideoChannel(False)                       # stop when done
```

The `add_track_callback` must happen before `connect()` because
`init_webrtc()` registers its `@pc.on("track")` handler during
construction and the track can arrive at any point after the SDP
exchange. If we register the callback after `connect()` returns, there's
a race window where the track fires and our callback list is still
empty — we'd miss the handoff.

## Transitive dependencies (already installed)

From `pyproject.toml` of `unitree_webrtc_connect`:

- **`aiortc`** — the Python WebRTC stack. Provides `RTCPeerConnection`,
  video/audio track abstractions. `track.recv()` is async and returns an
  `av.VideoFrame`.
- **`av`** — FFmpeg bindings. `VideoFrame.to_ndarray(format="bgr24")` or
  `format="rgb24"` converts the frame to a numpy array. H.264 decode
  happens inside `av` transparently.
- **`opencv-python`** — numpy array interop + JPEG/PNG encode + any
  downstream image ops (`cv2.imwrite`, `cv2.imencode`).

These come along with `unitree_webrtc_connect` as transitive deps, so
our `pyproject.toml` doesn't need to declare them explicitly. Good to
be aware that our dependency footprint already includes them.

## Things the library does NOT provide

These are our responsibility if we want them:

- **Camera selection.** Only `self.video` is exposed — no way to switch
  between front RGB / chin / belly cameras. Whatever the Go2 sends on
  the default video track is what we get. Almost certainly the front
  RGB (~720p, H.264, 15–30 fps — to be verified on hardware).
- **Track metadata.** No resolution/codec/framerate info is surfaced —
  we'd read that off the `av.VideoFrame` itself (`frame.width`,
  `frame.height`, `frame.time_base`, `frame.pts`).
- **Frame timestamps.** `av.VideoFrame.pts` is a monotonic presentation
  timestamp from the encoder — not wall-clock. For geotagging we'd want
  `time.time()` at the moment `recv()` returned, since that's what
  correlates with the GPS sample timeline.
- **Decoding control.** There's a `set_decoder()` method on the data
  channel (`webrtc_datachannel.py:193`) but it looks audio-related; I
  didn't trace it fully. For video, aiortc/av picks the decoder based
  on the negotiated codec.
- **Buffering / keyframe logic.** First frames after `switchVideoChannel(True)`
  may be several seconds delayed while aiortc waits for a keyframe. If
  we want a "capture one frame right now" API, we should either (a)
  keep the video channel on continuously and cache the latest frame, or
  (b) accept up to a GOP (~1–2 s) of latency on a cold capture.

## Design implications for our `vision/frames.py`

Not a design yet — just things worth deciding before writing code:

1. **Continuous vs on-demand capture.** Continuous (keep video on the
   whole mission, cache latest frame, snapshot on demand) is simpler
   and avoids keyframe-wait latency at waypoints, but burns bandwidth
   + CPU for the full run. On-demand (toggle video on, wait for frame,
   toggle off) is cleaner but adds 1–2 s to every waypoint.
2. **Where the capture callback lives.** `Go2Robot` is the natural
   owner of `self.conn`, so `enable_video()` / `get_latest_frame()`
   probably belong there. `vision/frames.py` would then be the
   higher-level "capture + metadata bundle" step, not the WebRTC
   plumbing.
3. **Frame freshness check.** Analogous to `get_yaw_degrees(max_age=...)`
   in `robot.py`, a `get_latest_frame(max_age=...)` that returns `None`
   on stale data will save us from geotagging a 30-second-old frame if
   the video stalls mid-mission.
4. **Callback threading model.** `aiortc` callbacks run on the
   asyncio event loop. Our mission runner is already async
   (`mission_runner.run_mission`), so no cross-thread concerns — a
   simple `self._latest_frame` attribute on `Go2Robot` works the same
   way `self._latest_imu` does today.

## Unknowns to verify on hardware

- Actual resolution / framerate / codec of the default video track.
- Latency from `switchVideoChannel(True)` to first frame (keyframe wait).
- Whether the track stops cleanly on `switchVideoChannel(False)` or
  whether we need to do anything else to tear it down.
- Whether enabling video affects WebRTC stability of the control plane
  (unlikely, but worth watching on the first test mission).
- Whether `pc.on("track")` fires again if we toggle the channel off and
  back on, or whether the track is persistent across on/off cycles.

## References

- `site-packages/unitree_webrtc_connect/webrtc_driver.py:96-169` — peer
  connection setup, `on_track` handler.
- `site-packages/unitree_webrtc_connect/webrtc_video.py` — the whole
  file, it's short.
- `site-packages/unitree_webrtc_connect/webrtc_datachannel.py:174-181` —
  `switchVideoChannel` implementation.
- [aiortc docs](https://aiortc.readthedocs.io/) — for `RTCPeerConnection`,
  track handling, and `MediaStreamTrack.recv()` semantics.
- [PyAV docs](https://pyav.org/docs/stable/) — for `VideoFrame` and
  `to_ndarray()`.
