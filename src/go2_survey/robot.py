"""Unitree Go2 robot interface over WebRTC."""

import asyncio
import json
import logging
import math
import time
from typing import Any

try:
    import numpy as np
except ImportError:
    np = None  # type: ignore

from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

from go2_survey.logging_utils import log_banner

logger = logging.getLogger(__name__)


class Go2Robot:
    """Unitree Go2 robot interface via WebRTC."""

    API_READY_TIMEOUT = 30
    MODE_SWITCH_WAIT = 5

    def __init__(
        self,
        connection_mode: str = "LocalAP",
        robot_ip: str | None = None,
        robot_serial: str | None = None,
    ):
        self.connection_mode = connection_mode
        self.robot_ip = robot_ip
        self.robot_serial = robot_serial
        self.conn: UnitreeWebRTCConnection | None = None
        self._connected = False
        self._latest_imu: dict | None = None
        self._imu_timestamp: float = 0.0
        # Video cache (populated after enable_video()). Shape follows the
        # IMU cache above: latest frame as numpy BGR ndarray + wall-clock
        # timestamp. See docs/webrtc/README.md for the library quirks
        # this works around.
        self._latest_frame: Any | None = None
        self._frame_timestamp: float = 0.0
        self._frame_width: int = 0
        self._frame_height: int = 0
        self._video_enabled: bool = False
        self._video_task: asyncio.Task | None = None
        self._first_frame_logged: bool = False

    async def connect(self) -> bool:
        """Establish WebRTC connection to the robot."""
        try:
            if self.connection_mode == "LocalAP":
                logger.info("Connecting to Go2 via LocalAP (robot hotspot)")
                self.conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
            elif self.connection_mode == "LocalSTA":
                if self.robot_serial:
                    logger.info(
                        f"Connecting via LocalSTA (serial: {self.robot_serial})"
                    )
                    self.conn = UnitreeWebRTCConnection(
                        WebRTCConnectionMethod.LocalSTA,
                        serialNumber=self.robot_serial,
                    )
                else:
                    logger.info(f"Connecting via LocalSTA (IP: {self.robot_ip})")
                    self.conn = UnitreeWebRTCConnection(
                        WebRTCConnectionMethod.LocalSTA, ip=self.robot_ip
                    )
            else:
                raise ValueError(f"Unknown connection mode: {self.connection_mode}")

            await self.conn.connect()
            logger.info("WebRTC connected")

            if not await self._wait_for_api():
                raise RuntimeError("Robot API not responding")

            self.conn.datachannel.pub_sub.subscribe(
                RTC_TOPIC["LF_SPORT_MOD_STATE"], self._on_sport_state
            )
            logger.info("Subscribed to sport mode state (IMU)")

            self._connected = True
            return True
        except Exception as e:
            logger.error(f"Failed to connect to robot: {e}")
            return False

    async def _wait_for_api(self) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.API_READY_TIMEOUT:
            try:
                resp = await self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001}
                )
                if resp["data"]["header"]["status"]["code"] == 0:
                    logger.info("Robot API ready")
                    return True
            except Exception as e:
                logger.debug(f"API not ready: {e}")
            await asyncio.sleep(0.5)
        return False

    def _on_sport_state(self, message: dict) -> None:
        try:
            self._latest_imu = message["data"]["imu_state"]
            self._imu_timestamp = time.time()
        except (KeyError, TypeError):
            pass

    def get_yaw_degrees(self, max_age: float = 1.0) -> float | None:
        """Get current IMU yaw in degrees.

        The Go2 IMU reports yaw CCW-positive relative to power-on
        orientation. Returns None if no IMU sample has arrived within
        max_age seconds.
        """
        if self._latest_imu is None:
            return None
        if time.time() - self._imu_timestamp > max_age:
            return None
        try:
            return math.degrees(self._latest_imu["rpy"][2])
        except (KeyError, TypeError, IndexError):
            return None

    async def enable_video(self) -> None:
        """Subscribe to the Go2 video track and start caching frames.

        Registers an async track callback BEFORE toggling the channel
        on — the library's internal `@pc.on("track")` handler calls
        registered callbacks once per track. We consume frames from the
        track in a long-running task and stash the latest as a numpy
        BGR ndarray. See docs/webrtc/README.md.

        Safe to call more than once; second and later calls are no-ops.
        """
        if not self._connected or self.conn is None:
            raise RuntimeError("Call connect() before enable_video()")
        if self._video_enabled:
            return

        log_banner("ENABLING VIDEO CHANNEL", char="-", logger=logger)
        self.conn.video.add_track_callback(self._on_video_track)
        self.conn.video.switchVideoChannel(True)
        self._video_enabled = True
        logger.info("Video channel turned on; waiting for first frame")

    async def disable_video(self) -> None:
        """Turn the video channel off and clear the cache."""
        if not self._video_enabled or self.conn is None:
            return
        try:
            self.conn.video.switchVideoChannel(False)
        except Exception as e:
            logger.debug(f"switchVideoChannel(False) raised: {e}")
        if self._video_task is not None and not self._video_task.done():
            self._video_task.cancel()
        self._video_enabled = False
        self._latest_frame = None
        self._frame_timestamp = 0.0
        self._first_frame_logged = False
        log_banner("VIDEO CHANNEL OFF", char="-", logger=logger)

    async def close(self) -> None:
        """Tear down the WebRTC peer connection.

        Call from a mission's finally block after disable_video().
        Awaits any lingering video consumer task, then hands off to
        the library's UnitreeWebRTCConnection.disconnect(), which in
        turn awaits pc.close() and releases the underlying
        RTCPeerConnection — the piece missing today that produces the
        "Task was destroyed but it is pending!" warning at process
        exit. Safe to call more than once and when connect() never
        completed.
        """
        if self._video_task is not None:
            if not self._video_task.done():
                self._video_task.cancel()
            try:
                await self._video_task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                logger.debug(f"Video task raised during shutdown: {e}")
            self._video_task = None

        if self.conn is not None:
            try:
                await self.conn.disconnect()
            except Exception as e:
                logger.debug(f"conn.disconnect() raised: {e}")
            self.conn = None

        self._connected = False

    async def _on_video_track(self, track) -> None:
        """Library callback — spawn a consumer task for this track.

        The library calls this once per incoming video track; we must
        start our own loop that pulls frames with `track.recv()`. The
        library's internal handler has already consumed one frame
        before we get here (see webrtc_driver.py on_track handler),
        which we can't avoid — our first recv() returns the *second*
        frame the robot sent. For a cached-latest-frame API this
        doesn't matter.
        """
        logger.info("Video track attached; starting frame consumer")
        self._video_task = asyncio.create_task(self._consume_video_track(track))

    async def _consume_video_track(self, track) -> None:
        """Pull frames off the track in a loop, convert to BGR ndarray."""
        if np is None:
            logger.error("numpy not available; cannot decode video frames")
            return
        try:
            while True:
                frame = await track.recv()
                try:
                    img = frame.to_ndarray(format="bgr24")
                except Exception as e:
                    logger.debug(f"frame.to_ndarray failed: {e}")
                    continue
                self._latest_frame = img
                self._frame_timestamp = time.time()
                self._frame_height, self._frame_width = img.shape[:2]
                if not self._first_frame_logged:
                    logger.info(
                        f"First video frame decoded | "
                        f"{self._frame_width}x{self._frame_height} BGR"
                    )
                    self._first_frame_logged = True
        except asyncio.CancelledError:
            logger.debug("Video consumer task cancelled")
            raise
        except Exception as e:
            logger.error(f"Video consumer task failed: {e}", exc_info=True)

    def get_latest_frame(self, max_age: float = 1.0) -> Any | None:
        """Return the most recent video frame as a numpy BGR ndarray.

        Returns None if no frame has arrived, the cache is older than
        `max_age` seconds, or video is disabled.
        """
        if self._latest_frame is None:
            return None
        if time.time() - self._frame_timestamp > max_age:
            return None
        return self._latest_frame

    def get_frame_timestamp(self) -> float:
        """Wall-clock time (seconds) when the cached frame was received."""
        return self._frame_timestamp

    def get_frame_size(self) -> tuple:
        """(width, height) of the most recent decoded frame. (0, 0) if none yet."""
        return (self._frame_width, self._frame_height)

    async def ensure_normal_mode(self) -> None:
        """Ensure robot is in normal walking mode."""
        resp = await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001}
        )
        if resp["data"]["header"]["status"]["code"] != 0:
            raise RuntimeError(f"Failed to get motion mode: {resp}")

        mode = json.loads(resp["data"]["data"]).get("name", "unknown")
        logger.info(f"Current motion mode: {mode}")

        if mode != "normal":
            logger.info("Switching to normal mode...")
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {"api_id": 1002, "parameter": {"name": "normal"}},
            )
            await asyncio.sleep(self.MODE_SWITCH_WAIT)

    async def balance_stand(self) -> None:
        """Issue BalanceStand to prime the gait controller before motion."""
        logger.info("BalanceStand...")
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]}
        )

    async def send_velocity(self, x: float = 0, y: float = 0, z: float = 0) -> None:
        """Send velocity command.

        x: forward m/s (positive = forward)
        y: lateral m/s (positive = left)
        z: rotational rad/s (positive = CCW)
        """
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Move"], "parameter": {"x": x, "y": y, "z": z}},
        )

    async def stop(self) -> None:
        logger.info("Stopping robot")
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StopMove"]}
        )

    async def prepare_for_navigation(self) -> None:
        """Full preparation sequence before issuing motion commands."""
        await self.ensure_normal_mode()
        await self.balance_stand()
        await asyncio.sleep(1.0)
