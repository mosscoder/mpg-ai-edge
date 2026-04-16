"""Capture still frames from the Go2 WebRTC video stream.

Thin wrapper over `Go2Robot.get_latest_frame()`. The robot keeps a
rolling cache of the latest decoded frame (see `robot.py::enable_video`
and `_consume_video_track`); this module just waits for a fresh one
and bundles it with metadata for downstream geotagging.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from typing import Optional

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


async def capture_frame(
    robot: Go2Robot,
    max_age: float = 0.5,
    wait_timeout: float = 5.0,
    poll_interval: float = 0.05,
) -> Optional[FrameResult]:
    """Wait for a fresh frame from the robot's video cache and return it.

    - `max_age`: frame must be newer than this (seconds). A cached
      frame older than max_age is ignored.
    - `wait_timeout`: how long to block waiting for a fresh frame
      before giving up.
    - `poll_interval`: how often to re-check the cache.

    Returns None on timeout or if video hasn't been enabled. Logs the
    capture event at INFO.
    """
    if not getattr(robot, "_video_enabled", False):
        logger.error("capture_frame: video channel is not enabled on robot")
        return None

    deadline = time.monotonic() + wait_timeout
    while time.monotonic() < deadline:
        img = robot.get_latest_frame(max_age=max_age)
        if img is not None:
            w, h = robot.get_frame_size()
            ts = robot.get_frame_timestamp()
            logger.info(
                f"Captured frame | {w}x{h} | age={time.time() - ts:.3f}s"
            )
            # Defensive copy so downstream writers don't race with the
            # video consumer overwriting the cache slot.
            return FrameResult(
                image=img.copy() if np is not None else img,
                timestamp=ts,
                width=w,
                height=h,
            )
        await asyncio.sleep(poll_interval)

    logger.warning(
        f"capture_frame: no fresh frame within {wait_timeout:.1f}s "
        f"(max_age={max_age:.2f}s)"
    )
    return None
