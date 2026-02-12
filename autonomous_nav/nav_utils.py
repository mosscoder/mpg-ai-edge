"""
Navigation Utilities for Go2 Waypoint Navigation

Provides:
- GPS: RTK positioning via SparkFun ZED-F9P with Emlid NTRIP corrections
- Robot: Unitree Go2 WebRTC connection and motion control
- Navigation: Coordinate math and waypoint loading
"""

import asyncio
import base64
import json
import logging
import os
import serial
import socket
import struct
import threading
import time
from collections import Counter
import math
from dataclasses import dataclass
from math import asin, atan2, cos, degrees, radians, sin, sqrt
from typing import Dict, List, Optional, Tuple

import pynmea2
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

logger = logging.getLogger(__name__)


def _log_banner(message: str, level: str = "info", char: str = "=", width: int = 60):
    border = char * width
    padded = f" {message} ".center(width, char)
    log_fn = getattr(logger, level)
    log_fn(border)
    log_fn(padded)
    log_fn(border)


# =============================================================================
# DATA STRUCTURES
# =============================================================================


@dataclass
class RTKPosition:
    """GPS position with accuracy metrics."""

    latitude: float
    longitude: float
    altitude: float
    accuracy_horizontal: float
    accuracy_vertical: float
    fix_type: int
    satellites_used: int
    course_over_ground: Optional[float]  # degrees, None if stationary
    timestamp: float


@dataclass
class NTRIPConfig:
    """NTRIP caster connection configuration."""

    host: str
    port: int
    mountpoint: str
    username: str
    password: str


@dataclass
class Waypoint:
    """A navigation target point."""

    latitude: float
    longitude: float
    name: Optional[str] = None


# =============================================================================
# GPS COMPONENTS
# =============================================================================


class EmlidNTRIPClient:
    """NTRIP v1 client for receiving RTCM corrections from Emlid caster."""

    def __init__(self, config: NTRIPConfig):
        self.config = config
        self.socket = None
        self.connected = False
        self.correction_thread = None
        self.running = False
        self.gps_receiver = None
        self.correction_count = 0
        self.bytes_forwarded = 0
        self.rtcm_types = Counter()

    def connect(self) -> bool:
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            logger.info(
                f"Connecting to NTRIP caster at {self.config.host}:{self.config.port}"
            )
            self.socket.connect((self.config.host, self.config.port))

            request = self._build_ntrip_request()
            self.socket.send(request.encode("ascii"))
            response = self.socket.recv(1024).decode("ascii", errors="ignore")

            if "ICY 200 OK" in response or "200 OK" in response:
                logger.info(f"Connected to NTRIP mountpoint: {self.config.mountpoint}")
                self.connected = True
                return True
            else:
                logger.error(f"NTRIP connection failed: {response.strip()}")
                return False
        except Exception as e:
            logger.error(f"Failed to connect to NTRIP caster: {e}")
            return False

    def _build_ntrip_request(self) -> str:
        auth_string = f"{self.config.username}:{self.config.password}"
        auth_header = base64.b64encode(auth_string.encode("ascii")).decode("ascii")
        return (
            f"GET /{self.config.mountpoint} HTTP/1.0\r\n"
            f"Host: {self.config.host}:{self.config.port}\r\n"
            f"User-Agent: Go2NavClient/1.0\r\n"
            f"Authorization: Basic {auth_header}\r\n"
            f"Accept: */*\r\n"
            f"Connection: keep-alive\r\n"
            f"\r\n"
        )

    def start_correction_stream(self, gps_receiver: "UBloxRTKGPS"):
        self.gps_receiver = gps_receiver
        self.running = True
        self.correction_thread = threading.Thread(
            target=self._correction_worker, daemon=True
        )
        self.correction_thread.start()
        logger.info("Started RTCM correction stream")

    @staticmethod
    def _rtcm_msg_type_from_payload(payload: bytes) -> Optional[int]:
        if len(payload) < 2:
            return None
        return ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF

    def _correction_worker(self):
        buffer = b""
        while self.running and self.connected:
            try:
                data = self.socket.recv(4096)
                if not data:
                    logger.warning("NTRIP connection lost")
                    break
                buffer += data

                while len(buffer) >= 3:
                    pre_idx = buffer.find(b"\xD3")
                    if pre_idx == -1:
                        buffer = b""
                        break
                    if pre_idx > 0:
                        buffer = buffer[pre_idx:]
                    if len(buffer) < 3:
                        break

                    length = ((buffer[1] & 0x03) << 8) | buffer[2]
                    total_len = length + 6
                    if len(buffer) < total_len:
                        break

                    rtcm_msg = buffer[:total_len]
                    buffer = buffer[total_len:]

                    payload = rtcm_msg[3:-3]
                    mtype = self._rtcm_msg_type_from_payload(payload)
                    if mtype is not None:
                        self.rtcm_types[mtype] += 1

                    if self.gps_receiver and self.gps_receiver.serial_conn:
                        self.gps_receiver.serial_conn.write(rtcm_msg)
                        self.gps_receiver.serial_conn.flush()
                        self.correction_count += 1
                        self.bytes_forwarded += len(rtcm_msg)

            except Exception as e:
                logger.error(f"Error in correction worker: {e}")
                break
        logger.info("RTCM correction worker stopped")

    def disconnect(self):
        self.running = False
        if self.correction_thread:
            self.correction_thread.join(timeout=2.0)
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            self.socket.close()
        self.connected = False
        logger.info(
            f"Disconnected from NTRIP. Forwarded {self.correction_count} messages."
        )


class UBloxRTKGPS:
    """u-blox ZED-F9P GPS interface using UBX protocol."""

    UBX_SYNC_CHAR1 = 0xB5
    UBX_SYNC_CHAR2 = 0x62
    UBX_NAV_CLASS = 0x01
    UBX_NAV_PVT = 0x07
    UBX_CFG_CLASS = 0x06
    UBX_CFG_RATE = 0x08

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 38400):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.last_pvt: Optional[Dict] = None

    def connect(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            logger.info(f"Connected to GPS on {self.port} @ {self.baudrate} baud")
            self._configure_navigation_rate(200)  # 5Hz for responsive COG
            return True
        except Exception as e:
            logger.error(f"Failed to connect to GPS: {e}")
            return False

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("GPS connection closed")

    def _configure_navigation_rate(self, rate_ms: int):
        payload = struct.pack("<HHH", rate_ms, 1, 1)
        self._send_ubx_message(self.UBX_CFG_CLASS, self.UBX_CFG_RATE, payload)

    def _send_ubx_message(self, msg_class: int, msg_id: int, payload: bytes):
        if not self.serial_conn:
            return
        ca = cb = 0
        header = struct.pack(
            "<BBBBH",
            self.UBX_SYNC_CHAR1,
            self.UBX_SYNC_CHAR2,
            msg_class,
            msg_id,
            len(payload),
        )
        for b in header[2:] + payload:
            ca = (ca + b) & 0xFF
            cb = (cb + ca) & 0xFF
        self.serial_conn.write(header + payload + bytes([ca, cb]))
        self.serial_conn.flush()

    def _read_exact(self, n: int, timeout: float = 1.0) -> bytes:
        end = time.time() + timeout
        buf = bytearray()
        while len(buf) < n and time.time() < end:
            chunk = self.serial_conn.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.005)
        return bytes(buf) if len(buf) == n else b""

    def _read_ubx_message(
        self, timeout: float = 2.0
    ) -> Optional[Tuple[int, int, bytes]]:
        if not self.serial_conn:
            return None
        end = time.time() + timeout
        while time.time() < end:
            b = self.serial_conn.read(1)
            if b != b"\xB5":
                continue
            if self.serial_conn.read(1) != b"\x62":
                continue
            hdr = self._read_exact(4, timeout=0.2)
            if not hdr:
                continue
            msg_class, msg_id, length = struct.unpack("<BBH", hdr)
            payload = self._read_exact(length, timeout=0.5)
            if not payload:
                continue
            ck = self._read_exact(2, timeout=0.2)
            if not ck:
                continue
            ca = cb = 0
            for byte in hdr + payload:
                ca = (ca + byte) & 0xFF
                cb = (cb + ca) & 0xFF
            if ck != bytes([ca, cb]):
                continue
            return msg_class, msg_id, payload
        return None

    def poll_nav_pvt(self) -> Optional[Dict]:
        """Poll UBX-NAV-PVT for position and status."""
        self._send_ubx_message(self.UBX_NAV_CLASS, self.UBX_NAV_PVT, b"")
        msg = self._read_ubx_message(timeout=0.5)
        if not msg:
            return None
        msg_class, msg_id, payload = msg
        if msg_id != self.UBX_NAV_PVT or len(payload) < 92:
            return None

        try:
            (
                iTOW,
                year,
                month,
                day,
                hour,
                minute,
                second,
                valid,
            ) = struct.unpack("<IHBBBBBB", payload[:12])
            tAcc, nano, fixType, flags, flags2, numSV = struct.unpack(
                "<IBIBBB", payload[12:24]
            )
            lon, lat, height, hMSL = struct.unpack("<iiii", payload[24:40])
            hAcc, vAcc = struct.unpack("<II", payload[40:48])

            # Velocity for COG calculation (NED frame, mm/s)
            velN, velE, velD = struct.unpack("<iii", payload[48:60])
            gSpeed = struct.unpack("<i", payload[60:64])[0]  # ground speed mm/s
            headMot = struct.unpack("<i", payload[64:68])[0]  # heading 1e-5 deg

            carrSoln = flags2 & 0x03
            diffSoln = flags & 0x01

            # COG only valid if moving
            cog = None
            if gSpeed > 100:  # > 0.1 m/s
                cog = headMot * 1e-5

            out = {
                "iTOW": iTOW,
                "fixType": fixType,
                "carrSoln": carrSoln,
                "diffSoln": diffSoln,
                "numSV": numSV,
                "lat": lat * 1e-7,
                "lon": lon * 1e-7,
                "height": height * 1e-3,
                "hAcc": hAcc * 1e-3,
                "vAcc": vAcc * 1e-3,
                "gSpeed": gSpeed * 1e-3,  # m/s
                "cog": cog,  # degrees or None
            }
            self.last_pvt = out
            return out
        except Exception as e:
            logger.error(f"Error parsing NAV-PVT: {e}")
            return None

    def get_position(self) -> Optional[RTKPosition]:
        """Get current position as RTKPosition dataclass."""
        pvt = self.poll_nav_pvt()
        if not pvt:
            return None

        # Map carrSoln/fixType to unified fix_type
        if pvt["carrSoln"] == 2:
            fix_type = 6  # RTK Fixed
        elif pvt["carrSoln"] == 1:
            fix_type = 5  # RTK Float
        else:
            fix_type = pvt["fixType"]

        return RTKPosition(
            latitude=pvt["lat"],
            longitude=pvt["lon"],
            altitude=pvt["height"],
            accuracy_horizontal=pvt["hAcc"],
            accuracy_vertical=pvt["vAcc"],
            fix_type=fix_type,
            satellites_used=pvt["numSV"],
            course_over_ground=pvt["cog"],
            timestamp=time.time(),
        )

    def wait_for_rtk_fix(self, timeout: float = 300.0, min_fix_type: int = 4) -> bool:
        """Wait for GPS fix. Returns True if fix achieved."""
        logger.info(f"Waiting for GPS fix (min type {min_fix_type})...")
        start = time.time()
        last_progress = start
        while time.time() - start < timeout:
            pos = self.get_position()
            if pos and pos.fix_type >= min_fix_type:
                fix_names = {
                    3: "3D Fix",
                    4: "GNSS+DR",
                    5: "RTK Float",
                    6: "RTK Fixed",
                }
                fix_label = fix_names.get(pos.fix_type, "Fix")
                logger.info(
                    f"{fix_label} achieved! "
                    f"Lat: {pos.latitude:.8f}, Lon: {pos.longitude:.8f}, "
                    f"hAcc: {pos.accuracy_horizontal:.3f}m"
                )
                _log_banner(f"GPS {fix_label} ACHIEVED | hAcc: {pos.accuracy_horizontal:.3f}m")
                return True
            now = time.time()
            if now - last_progress >= 15.0:
                elapsed = now - start
                fix_type = pos.fix_type if pos else "?"
                num_sv = pos.satellites_used if pos else "?"
                logger.info(f"Waiting for GPS fix... ({elapsed:.0f}s elapsed, type {fix_type}, {num_sv} SVs)")
                last_progress = now
            time.sleep(1.0)
        elapsed = time.time() - start
        logger.error(f"GPS fix timeout after {elapsed:.0f}s")
        _log_banner(f"GPS FIX TIMEOUT after {elapsed:.0f}s", level="error", char="!")
        return False


class GPSManager:
    """High-level GPS manager with NTRIP support."""

    def __init__(
        self,
        port: str = None,
        baudrate: int = None,
        ntrip_config: NTRIPConfig = None,
    ):
        self.port = port or os.getenv("GPS_PORT", "/dev/ttyACM0")
        self.baudrate = baudrate or int(os.getenv("GPS_BAUD", "38400"))

        if ntrip_config is None:
            ntrip_config = NTRIPConfig(
                host=os.getenv("EMLID_NTRIP_HOST", "caster.emlid.com"),
                port=int(os.getenv("EMLID_NTRIP_PORT", "2101")),
                mountpoint=os.getenv("EMLID_MOUNTPOINT", "MP1979"),
                username=os.getenv("EMLID_USERNAME", ""),
                password=os.getenv("EMLID_PASSWORD", ""),
            )
        self.ntrip_config = ntrip_config

        self.gps = UBloxRTKGPS(port=self.port, baudrate=self.baudrate)
        self.ntrip = EmlidNTRIPClient(ntrip_config)
        self._connected = False

    def connect(self, use_ntrip: bool = True) -> bool:
        """Connect to GPS and optionally start NTRIP corrections."""
        if not self.gps.connect():
            return False

        if use_ntrip and self.ntrip_config.username:
            if self.ntrip.connect():
                self.ntrip.start_correction_stream(self.gps)
            else:
                logger.warning("NTRIP not available, using GNSS-only mode")

        self._connected = True
        return True

    def disconnect(self):
        self.ntrip.disconnect()
        self.gps.disconnect()
        self._connected = False

    def get_position(self) -> Optional[RTKPosition]:
        """Get current RTK position."""
        return self.gps.get_position()

    def wait_for_fix(self, timeout: float = 300.0, min_fix_type: int = 4) -> bool:
        """Wait for GPS fix."""
        return self.gps.wait_for_rtk_fix(timeout=timeout, min_fix_type=min_fix_type)


# =============================================================================
# ROBOT COMPONENTS
# =============================================================================


class Go2Robot:
    """Unitree Go2 robot interface via WebRTC."""

    API_READY_TIMEOUT = 30
    MODE_SWITCH_WAIT = 5

    def __init__(
        self,
        connection_mode: str = "LocalAP",
        robot_ip: str = None,
        robot_serial: str = None,
    ):
        self.connection_mode = connection_mode
        self.robot_ip = robot_ip
        self.robot_serial = robot_serial
        self.conn: Optional[UnitreeWebRTCConnection] = None
        self._connected = False
        self._latest_imu: Optional[Dict] = None
        self._imu_timestamp: float = 0.0

    async def connect(self) -> bool:
        """Establish WebRTC connection to robot."""
        try:
            if self.connection_mode == "LocalAP":
                logger.info("Connecting to Go2 via LocalAP (robot hotspot)")
                self.conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
            elif self.connection_mode == "LocalSTA":
                if self.robot_serial:
                    logger.info(f"Connecting via LocalSTA (serial: {self.robot_serial})")
                    self.conn = UnitreeWebRTCConnection(
                        WebRTCConnectionMethod.LocalSTA, serialNumber=self.robot_serial
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

            # Subscribe to sport mode state for IMU data
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
        """Wait for robot API to be ready."""
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

    def _on_sport_state(self, message: Dict):
        """Callback for sport mode state updates containing IMU data."""
        try:
            self._latest_imu = message["data"]["imu_state"]
            self._imu_timestamp = time.time()
        except (KeyError, TypeError):
            pass

    def get_yaw_degrees(self, max_age: float = 1.0) -> Optional[float]:
        """Get current IMU yaw in degrees.

        Args:
            max_age: Maximum age of IMU data in seconds. Returns None if
                data is older than this threshold.

        Returns:
            Yaw angle in degrees, or None if IMU data not available or stale.
            The yaw is relative to the robot's power-on orientation.
        """
        if self._latest_imu is None:
            return None
        if time.time() - self._imu_timestamp > max_age:
            return None
        try:
            return math.degrees(self._latest_imu["rpy"][2])
        except (KeyError, TypeError, IndexError):
            return None

    async def ensure_normal_mode(self):
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

    async def balance_stand(self):
        """Issue BalanceStand command to prepare for walking."""
        logger.info("BalanceStand...")
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]}
        )

    async def send_velocity(self, x: float = 0, y: float = 0, z: float = 0):
        """
        Send velocity command.

        Args:
            x: Forward velocity (m/s, positive = forward)
            y: Lateral velocity (m/s, positive = left)
            z: Rotational velocity (rad/s, positive = CCW)
        """
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Move"], "parameter": {"x": x, "y": y, "z": z}},
        )

    async def stop(self):
        """Stop all movement."""
        logger.info("Stopping robot")
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StopMove"]}
        )

    async def prepare_for_navigation(self):
        """Full preparation sequence for navigation."""
        await self.ensure_normal_mode()
        await self.balance_stand()
        await asyncio.sleep(1.0)


# =============================================================================
# NAVIGATION COMPONENTS
# =============================================================================


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate distance between two coordinates using Haversine formula.

    Args:
        lat1, lon1: First coordinate (degrees)
        lat2, lon2: Second coordinate (degrees)

    Returns:
        Distance in meters
    """
    R = 6371000  # Earth radius in meters
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlam = radians(lon2 - lon1)
    a = sin(dphi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(dlam / 2) ** 2
    return 2 * R * asin(sqrt(a))


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate bearing from point 1 to point 2.

    Args:
        lat1, lon1: Start coordinate (degrees)
        lat2, lon2: End coordinate (degrees)

    Returns:
        Bearing in degrees (0-360, where 0=North, 90=East)
    """
    phi1, phi2 = radians(lat1), radians(lat2)
    dlam = radians(lon2 - lon1)
    x = sin(dlam) * cos(phi2)
    y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dlam)
    return (degrees(atan2(x, y)) + 360) % 360


def normalize_angle(angle: float) -> float:
    """Normalize angle to -180 to 180 range."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def load_waypoints(geojson_path: str) -> List[Waypoint]:
    """
    Load waypoints from a GeoJSON file.

    Supports Point and MultiPoint geometries in a FeatureCollection.

    Args:
        geojson_path: Path to GeoJSON file

    Returns:
        List of Waypoint objects
    """
    with open(geojson_path, "r") as f:
        data = json.load(f)

    waypoints = []
    features = data.get("features", [])

    for i, feature in enumerate(features):
        geom = feature.get("geometry", {})
        props = feature.get("properties", {})
        name = props.get("name", f"waypoint_{i}")

        if geom.get("type") == "Point":
            coords = geom["coordinates"]
            waypoints.append(
                Waypoint(latitude=coords[1], longitude=coords[0], name=name)
            )
        elif geom.get("type") == "MultiPoint":
            for j, coords in enumerate(geom["coordinates"]):
                waypoints.append(
                    Waypoint(
                        latitude=coords[1], longitude=coords[0], name=f"{name}_{j}"
                    )
                )

    logger.info(f"Loaded {len(waypoints)} waypoints from {geojson_path}")
    return waypoints


class WaypointNavigator:
    """Navigation controller for waypoint following."""

    def __init__(
        self,
        gps: GPSManager,
        robot: Go2Robot,
        max_velocity: float = 0.3,
        rotation_rate: float = 0.3,
        arrival_tolerance: float = 0.2,
        min_fix_type: int = 5,
        gps_timeout: float = 300.0,
        calibration_timeout: float = 30.0,
    ):
        self.gps = gps
        self.robot = robot
        self.max_velocity = max_velocity
        self.rotation_rate = rotation_rate
        self.arrival_tolerance = arrival_tolerance
        self.min_fix_type = min_fix_type
        self.gps_timeout = gps_timeout
        self.calibration_timeout = calibration_timeout

        self._running = False
        self._paused = False
        self._pause_start: Optional[float] = None
        self._pause_last_progress: Optional[float] = None

        # IMU calibration state
        self._imu_north_offset: Optional[float] = None  # Degrees to add to IMU yaw
        self._calibration_start_pos: Optional[RTKPosition] = None
        self._calibration_start_yaw: Optional[float] = None
        self._calibration_start_time: Optional[float] = None
        self._calibration_last_progress: Optional[float] = None

    async def navigate_to(self, waypoint: Waypoint, timeout: float = 300.0) -> bool:
        """
        Navigate to a single waypoint.

        Args:
            waypoint: Target waypoint
            timeout: Maximum time in seconds to reach the waypoint

        Returns:
            True if waypoint reached within tolerance
        """
        logger.info(
            f"Navigating to {waypoint.name}: "
            f"({waypoint.latitude:.8f}, {waypoint.longitude:.8f})"
        )
        _log_banner(f"NAVIGATING TO {waypoint.name}")

        self._running = True
        self._paused = False
        nav_start = time.time()

        while self._running:
            if time.time() - nav_start > timeout:
                pos_check = self.gps.get_position()
                remaining = ""
                if pos_check:
                    dist = haversine_distance(pos_check.latitude, pos_check.longitude, waypoint.latitude, waypoint.longitude)
                    remaining = f" | {dist:.1f}m remaining"
                logger.error(
                    f"Navigation timeout after {timeout:.0f}s for waypoint {waypoint.name}{remaining}"
                )
                _log_banner(f"NAVIGATION TIMEOUT | {waypoint.name} | {timeout:.0f}s", level="error", char="!")
                await self.robot.stop()
                return False

            # Get current position
            pos = self.gps.get_position()

            # Check for GPS loss or fix degradation
            if not pos or pos.fix_type < self.min_fix_type:
                if not self._paused:
                    fix_info = f"type {pos.fix_type}" if pos else "no position"
                    logger.warning(f"GPS fix lost or degraded ({fix_info}), pausing robot...")
                    _log_banner(f"GPS FIX LOST | {fix_info} | robot paused", level="warning", char="!")
                    await self.robot.stop()
                    self._paused = True
                    self._pause_start = time.time()
                    self._pause_last_progress = time.time()
                elif time.time() - self._pause_start > self.gps_timeout:
                    pause_elapsed = time.time() - self._pause_start
                    logger.error(
                        f"GPS fix not restored after {pause_elapsed:.0f}s "
                        f"(limit: {self.gps_timeout:.0f}s), aborting navigation to {waypoint.name}"
                    )
                    _log_banner(f"GPS PAUSE TIMEOUT | {pause_elapsed:.0f}s | aborting", level="error", char="!")
                    await self.robot.stop()
                    return False
                else:
                    now = time.time()
                    if now - self._pause_last_progress >= 15.0:
                        pause_elapsed = now - self._pause_start
                        logger.info(f"GPS fix lost -- still waiting... ({pause_elapsed:.0f}s / {self.gps_timeout:.0f}s timeout)")
                        self._pause_last_progress = now
                await asyncio.sleep(0.5)
                continue

            # Fix restored - resume if we were paused
            if self._paused:
                pause_duration = time.time() - self._pause_start
                logger.info(f"GPS fix restored (type {pos.fix_type}), resuming navigation")
                _log_banner(f"GPS FIX RESTORED | type {pos.fix_type} | paused {pause_duration:.0f}s")
                self._paused = False
                self._pause_start = None
                self._pause_last_progress = None
                # IMU heading survives GPS outages - no need to reset calibration

            # Calculate distance and bearing to target
            distance = haversine_distance(
                pos.latitude, pos.longitude, waypoint.latitude, waypoint.longitude
            )
            bearing = calculate_bearing(
                pos.latitude, pos.longitude, waypoint.latitude, waypoint.longitude
            )

            # Check arrival
            if distance < self.arrival_tolerance:
                elapsed = time.time() - nav_start
                logger.info(
                    f"ARRIVED at {waypoint.name}! Distance: {distance:.3f}m"
                )
                _log_banner(f"ARRIVED at {waypoint.name} | {distance:.3f}m | {elapsed:.0f}s")
                await self.robot.stop()
                return True

            # Get IMU yaw (if available)
            imu_yaw = self.robot.get_yaw_degrees()

            # Calibrate IMU if needed (during first forward motion)
            if self._imu_north_offset is None and imu_yaw is not None:
                self._calibrate_imu(pos, imu_yaw)

            # Get calibrated heading
            current_heading = self.get_calibrated_heading()

            # Calculate heading error
            if current_heading is not None:
                heading_error = normalize_angle(bearing - current_heading)
            else:
                if self._imu_north_offset is not None:
                    logger.warning("IMU heading stale - falling back to forward motion")
                heading_error = 0

            # Compute velocity commands
            vx, vz = self._compute_velocity(distance, heading_error)

            # Send command
            await self.robot.send_velocity(x=vx, z=vz)

            # Log status
            calibrated = "yes" if self._imu_north_offset is not None else "no"
            logger.debug(
                f"Pos: ({pos.latitude:.8f}, {pos.longitude:.8f}) | "
                f"Dist: {distance:.2f}m | Fix: {pos.fix_type} | hAcc: {pos.accuracy_horizontal:.3f}m | "
                f"Bearing: {bearing:.1f}° | Heading: {current_heading or '?'}° | "
                f"Error: {heading_error:.1f}° | Vel: x={vx:.2f}, z={vz:.2f} | IMU cal: {calibrated}"
            )

            await asyncio.sleep(0.2)  # 5Hz navigation loop

        await self.robot.stop()
        return False

    def _calibrate_imu(self, pos: RTKPosition, imu_yaw: float) -> bool:
        """Calibrate IMU yaw against GPS bearing during forward motion.

        The IMU provides yaw relative to power-on orientation. This method
        computes an offset to convert IMU yaw to true heading (relative to
        north) by comparing IMU yaw change with GPS-derived bearing during
        a period of forward motion.

        Args:
            pos: Current GPS position
            imu_yaw: Current IMU yaw in degrees

        Returns:
            True when calibration is complete
        """
        if self._calibration_start_pos is None:
            # Start calibration
            self._calibration_start_pos = pos
            self._calibration_start_yaw = imu_yaw
            self._calibration_start_time = time.time()
            self._calibration_last_progress = time.time()
            logger.info("IMU calibration started - walking forward to calibrate...")
            _log_banner("IMU CALIBRATION STARTED", char="-")
            return False

        # Compute displacement first (needed for timeout message and progress)
        displacement = haversine_distance(
            self._calibration_start_pos.latitude,
            self._calibration_start_pos.longitude,
            pos.latitude,
            pos.longitude,
        )

        # Check for calibration timeout
        elapsed = time.time() - self._calibration_start_time
        if elapsed > self.calibration_timeout:
            logger.error(
                f"IMU calibration timeout after {self.calibration_timeout:.0f}s "
                f"(displacement: {displacement:.2f}m / 1.50m needed). Resetting to allow retry."
            )
            _log_banner(f"IMU CALIBRATION TIMEOUT after {self.calibration_timeout:.0f}s", level="error", char="!")
            self._calibration_start_pos = None
            self._calibration_start_yaw = None
            self._calibration_start_time = None
            self._calibration_last_progress = None
            return False

        # Periodic progress
        now = time.time()
        if self._calibration_last_progress is not None and now - self._calibration_last_progress >= 5.0:
            logger.info(f"IMU calibrating... displacement: {displacement:.2f}m / 1.50m needed ({elapsed:.0f}s)")
            self._calibration_last_progress = now

        if displacement >= 1.5:  # Enough movement for reliable GPS bearing
            gps_bearing = calculate_bearing(
                self._calibration_start_pos.latitude,
                self._calibration_start_pos.longitude,
                pos.latitude,
                pos.longitude,
            )
            imu_delta = imu_yaw - self._calibration_start_yaw
            # Offset = GPS bearing - IMU delta (converts IMU yaw to true heading)
            self._imu_north_offset = normalize_angle(gps_bearing - imu_delta)
            logger.info(
                f"IMU calibrated! Offset: {self._imu_north_offset:.1f}° "
                f"(GPS bearing: {gps_bearing:.1f}°, IMU delta: {imu_delta:.1f}°)"
            )
            _log_banner(f"IMU CALIBRATED | Offset: {self._imu_north_offset:.1f} deg")
            return True

        return False

    def get_calibrated_heading(self) -> Optional[float]:
        """Get IMU yaw converted to true heading (relative to north).

        Returns:
            Heading in degrees (0-360, where 0=North, 90=East),
            or None if IMU not calibrated or data unavailable.
        """
        imu_yaw = self.robot.get_yaw_degrees()
        if imu_yaw is None or self._imu_north_offset is None:
            return None
        return (imu_yaw + self._imu_north_offset) % 360

    def _compute_velocity(
        self, distance: float, heading_error: float
    ) -> Tuple[float, float]:
        """
        Compute velocity commands based on distance and heading error.

        Returns:
            (forward_velocity, rotation_velocity)
        """
        abs_error = abs(heading_error)

        if abs_error > 30:
            # Large heading error - rotate in place
            vx = 0.0
            vz = (1 if heading_error > 0 else -1) * self.rotation_rate
        else:
            # Move forward while correcting heading
            # Slow down as we approach target
            vx = min(self.max_velocity, distance * 0.5)
            vx = max(0.1, vx)  # Minimum forward speed

            # Proportional heading correction
            vz = heading_error * 0.015
            vz = max(-self.rotation_rate, min(self.rotation_rate, vz))

        return vx, vz

    def stop(self):
        """Signal navigation to stop."""
        self._running = False
