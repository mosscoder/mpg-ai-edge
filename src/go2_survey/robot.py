"""Unitree Go2 robot interface over WebRTC."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import time
from typing import Dict, Optional

from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

logger = logging.getLogger(__name__)


class Go2Robot:
    """Unitree Go2 robot interface via WebRTC."""

    API_READY_TIMEOUT = 30
    MODE_SWITCH_WAIT = 5

    def __init__(
        self,
        connection_mode: str = "LocalAP",
        robot_ip: Optional[str] = None,
        robot_serial: Optional[str] = None,
    ):
        self.connection_mode = connection_mode
        self.robot_ip = robot_ip
        self.robot_serial = robot_serial
        self.conn: Optional[UnitreeWebRTCConnection] = None
        self._connected = False
        self._latest_imu: Optional[Dict] = None
        self._imu_timestamp: float = 0.0

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

    def _on_sport_state(self, message: Dict) -> None:
        try:
            self._latest_imu = message["data"]["imu_state"]
            self._imu_timestamp = time.time()
        except (KeyError, TypeError):
            pass

    def get_yaw_degrees(self, max_age: float = 1.0) -> Optional[float]:
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
