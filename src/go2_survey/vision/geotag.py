"""Geotag captured frames with RTK position and heading.

TODO: Implement. Takes a FrameResult (from frames.capture_frame) plus an
RTKPosition and calibrated heading, and emits either an EXIF-tagged
image or a frame + sidecar JSON pair for downstream inference pipelines.

Expected API roughly:

    def geotag_frame(
        frame: FrameResult,
        position: RTKPosition,
        heading_degrees: float | None,
        out_path: Path,
    ) -> Path:
        ...
"""
