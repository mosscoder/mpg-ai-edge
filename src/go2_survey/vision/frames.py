"""Capture still frames from the Go2 WebRTC video stream.

TODO: Implement. The Go2 exposes a video track over the same
RTCPeerConnection used by Go2Robot; aiortc + av + opencv (already pulled
in as transitive deps of unitree_webrtc_connect) can decode frames to
numpy arrays. Expected API roughly:

    async def capture_frame(robot: Go2Robot) -> FrameResult:
        ...

where FrameResult bundles the decoded image, an ISO timestamp, and any
track metadata we want downstream.
"""
