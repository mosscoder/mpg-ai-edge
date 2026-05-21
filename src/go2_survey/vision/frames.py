"""Capture still frames from the Go2 WebRTC video stream.

Thin wrapper over `Go2Robot.get_latest_frame()`. The robot keeps a
rolling cache of the latest decoded frame (see `robot.py::enable_video`
and `_consume_video_track`); this module just waits for a fresh one
and bundles it with metadata for downstream geotagging.
"""

import asyncio
import logging
import time
from dataclasses import dataclass

try:
    import numpy as np
except ImportError:
    np = None  # type: ignore

from go2_survey.robot import Go2Robot

logger = logging.getLogger(__name__)


@dataclass
class FrameResult:
    """A single captured frame + the metadata the geotagger needs."""

    image: "np.ndarray"  # BGR (HxWx3 uint8)
    timestamp: float     # wall clock (time.time()) at frame arrival
    width: int
    height: int
    corrupt: bool = False  # FFmpeg decode_error_flags / AV_FRAME_FLAG_CORRUPT


async def capture_frame(
    robot: Go2Robot,
    max_age: float = 0.5,
    wait_timeout: float = 5.0,
    poll_interval: float = 0.05,
    target_time: float | None = None,
    prefer_clean: bool = True,
) -> FrameResult | None:
    """Pick a frame from the robot's video ring buffer and return it.

    Selects the frame nearest `target_time` (default: now) within
    `max_age` seconds. When `prefer_clean`, a non-corrupt frame is
    preferred over a corrupt one in the window; if every candidate is
    corrupt, the nearest is returned with `FrameResult.corrupt=True` so
    downstream can flag it. `target_time` lets a moving-capture caller
    aim at the closest-pass instant rather than "now".

    - `max_age`: half-width of the time window around `target_time`.
    - `wait_timeout`: how long to block if the buffer has nothing in
      the window yet (e.g. video just started).
    - `poll_interval`: re-check cadence while waiting.

    Returns None on timeout or if video hasn't been enabled.
    """
    if not getattr(robot, "_video_enabled", False):
        logger.error("capture_frame: video channel is not enabled on robot")
        return None

    deadline = time.monotonic() + wait_timeout
    while time.monotonic() < deadline:
        tt = target_time if target_time is not None else time.time()
        best = robot.get_best_frame_near(
            tt, max_age=max_age, prefer_clean=prefer_clean
        )
        if best is not None:
            img, ts, corrupt = best
            h, w = img.shape[:2]
            offset = "" if target_time is None else f" Δtarget={ts - tt:+.3f}s"
            logger.info(
                f"Captured frame | {w}x{h} | age={time.time() - ts:.3f}s | "
                f"corrupt={corrupt}{offset}"
            )
            # Defensive copy so downstream writers don't race with the
            # video consumer overwriting the buffered ndarray.
            return FrameResult(
                image=img.copy() if np is not None else img,
                timestamp=ts,
                width=w,
                height=h,
                corrupt=corrupt,
            )
        await asyncio.sleep(poll_interval)

    logger.warning(
        f"capture_frame: no frame within {wait_timeout:.1f}s "
        f"(max_age={max_age:.2f}s)"
    )
    return None
