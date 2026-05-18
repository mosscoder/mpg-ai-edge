"""Emlid NTRIP client for RTCM correction streaming."""

import base64
import logging
import socket
import threading
import time
from collections import Counter
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class NTRIPConfig:
    """NTRIP caster connection configuration."""

    host: str
    port: int
    mountpoint: str
    username: str
    password: str


class EmlidNTRIPClient:
    """NTRIP v1 client that forwards RTCM corrections to an attached GPS receiver."""

    def __init__(self, config: NTRIPConfig):
        self.config = config
        self.socket: socket.socket | None = None
        self.connected = False
        self.correction_thread: threading.Thread | None = None
        self.running = False
        self.gps_receiver = None  # UBloxRTKGPS, typed as Any to avoid circular import
        self.correction_count = 0
        self.bytes_forwarded = 0
        self.rtcm_types: Counter = Counter()
        # Liveness signals for the surrounding system. `connection_alive`
        # flips False when the worker's recv() returns empty (caster
        # closed the socket), independent of `connected`/`running` so
        # GPSManager can detect mid-mission stalls. `last_rtcm_at` is
        # the monotonic clock at the moment the most recent RTCM frame
        # was forwarded — pair it with seconds_since_last_rtcm() to
        # distinguish "corrections flowing" from "receiver float-coast".
        self.connection_alive = False
        self.last_rtcm_at: float | None = None

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
                self.connection_alive = True
                return True
            logger.error(f"NTRIP connection failed: {response.strip()}")
            return False
        except Exception as e:
            logger.error(f"Failed to connect to NTRIP caster: {e}")
            return False

    def seconds_since_last_rtcm(self) -> float | None:
        """How long since we forwarded a frame to the receiver, or None if never."""
        if self.last_rtcm_at is None:
            return None
        return time.monotonic() - self.last_rtcm_at

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

    def start_correction_stream(self, gps_receiver) -> None:
        """Start forwarding RTCM corrections to the given UBloxRTKGPS instance."""
        self.gps_receiver = gps_receiver
        self.running = True
        self.correction_thread = threading.Thread(
            target=self._correction_worker, daemon=True
        )
        self.correction_thread.start()
        logger.info("Started RTCM correction stream")

    @staticmethod
    def _rtcm_msg_type_from_payload(payload: bytes) -> int | None:
        if len(payload) < 2:
            return None
        return ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF

    def _correction_worker(self) -> None:
        buffer = b""
        while self.running and self.connected:
            try:
                data = self.socket.recv(4096)
                if not data:
                    # Don't WARN when the main thread shut down our
                    # socket on purpose (matches the except-branch
                    # treatment below). Mid-mission empty-recv still
                    # logs so the navigator's reconnect path can react.
                    if self.running:
                        logger.warning("NTRIP connection lost")
                    self.connection_alive = False
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
                        self.last_rtcm_at = time.monotonic()
            except Exception as e:
                # During an intentional disconnect() the socket is
                # shut down from the main thread to wake a blocked
                # recv(); don't flag that as an error.
                if self.running:
                    logger.error(f"Error in correction worker: {e}")
                self.connection_alive = False
                break
        logger.info("RTCM correction worker stopped")

    def disconnect(self) -> None:
        self.running = False
        self.connection_alive = False
        # Shutdown the socket BEFORE joining the worker so any blocked
        # recv() wakes up immediately with an error — otherwise the
        # join waits up to `settimeout()` (10s) before the worker
        # notices `running` is False. Worker's except branch is aware
        # of the intentional-shutdown case and skips the error log.
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
        if self.correction_thread:
            self.correction_thread.join(timeout=2.0)
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
        self.connected = False
        logger.info(
            f"Disconnected from NTRIP. Forwarded {self.correction_count} messages."
        )
