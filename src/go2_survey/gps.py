"""u-blox ZED-F9P/F9R GPS interface and high-level manager."""

from __future__ import annotations

import logging
import os
import struct
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import serial

from go2_survey.logging_utils import log_banner
from go2_survey.ntrip import EmlidNTRIPClient, NTRIPConfig

logger = logging.getLogger(__name__)


@dataclass
class RTKPosition:
    """GPS position with accuracy metrics.

    `altitude` is height above the WGS84 ellipsoid (NAV-PVT `height`
    field). `altitude_msl` is height above mean sea level (NAV-PVT
    `hMSL`), using the receiver's internal geoid model.
    """

    latitude: float
    longitude: float
    altitude: float
    accuracy_horizontal: float
    accuracy_vertical: float
    fix_type: int
    satellites_used: int
    course_over_ground: Optional[float]
    timestamp: float
    head_vehicle: Optional[float] = None
    head_vehicle_accuracy: Optional[float] = None
    altitude_msl: Optional[float] = None
    pdop: Optional[float] = None
    correction_age_bin: Optional[int] = None


class UBloxRTKGPS:
    """u-blox ZED-F9P/F9R GPS interface using UBX protocol."""

    UBX_SYNC_CHAR1 = 0xB5
    UBX_SYNC_CHAR2 = 0x62
    UBX_NAV_CLASS = 0x01
    UBX_NAV_PVT = 0x07
    UBX_CFG_CLASS = 0x06
    UBX_CFG_RATE = 0x08

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 38400):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
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

    def disconnect(self) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("GPS connection closed")

    def _configure_navigation_rate(self, rate_ms: int) -> None:
        payload = struct.pack("<HHH", rate_ms, 1, 1)
        self._send_ubx_message(self.UBX_CFG_CLASS, self.UBX_CFG_RATE, payload)

    def _send_ubx_message(self, msg_class: int, msg_id: int, payload: bytes) -> None:
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

    def _read_ubx_message(self, timeout: float = 2.0) -> Optional[Tuple[int, int, bytes]]:
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
        _, msg_id, payload = msg
        if msg_id != self.UBX_NAV_PVT or len(payload) < 92:
            return None

        try:
            # Offsets and types per u-blox F9 HPS 1.30 Interface Description
            # (UBX-22010984), section 3.15.15.1. Payload is 92 bytes.
            (iTOW, year, month, day, hour, minute, second, valid) = struct.unpack(
                "<IHBBBBBB", payload[:12]
            )
            # offsets 12-23: tAcc(U4), nano(I4), fixType(U1), flags(X1),
            # flags2(X1), numSV(U1). Note: signed "i" for nano.
            tAcc, nano, fixType, flags, flags2, numSV = struct.unpack(
                "<IiBBBB", payload[12:24]
            )
            lon, lat, height, hMSL = struct.unpack("<iiii", payload[24:40])
            hAcc, vAcc = struct.unpack("<II", payload[40:48])
            velN, velE, velD = struct.unpack("<iii", payload[48:60])
            gSpeed, headMot, sAcc, headAcc = struct.unpack(
                "<iiII", payload[60:76]
            )
            pDOP = struct.unpack("<H", payload[76:78])[0]
            flags3 = struct.unpack("<H", payload[78:80])[0]
            headVeh = struct.unpack("<i", payload[84:88])[0]

            # flags (X1 at offset 21) — per spec:
            #   bit 0   gnssFixOk
            #   bit 1   diffSoln
            #   bits 2-4 psmState
            #   bit 5   headVehValid
            #   bits 6-7 carrSoln  (0=no, 1=float, 2=fixed)
            gnss_fix_ok = bool(flags & 0x01)
            diffSoln = bool(flags & 0x02)
            head_veh_valid = bool(flags & 0x20)
            carrSoln = (flags >> 6) & 0x03

            # flags3 (X2 at offset 78):
            #   bit 0      invalidLlh
            #   bits 1-4   lastCorrectionAge (12 quantized bins, 0=N/A)
            invalidLlh = bool(flags3 & 0x0001)
            lastCorrectionAge = (flags3 >> 1) & 0x0F

            cog = headMot * 1e-5 if gSpeed > 100 else None

            out = {
                "iTOW": iTOW,
                "fixType": fixType,
                "gnssFixOk": gnss_fix_ok,
                "carrSoln": carrSoln,
                "diffSoln": diffSoln,
                "numSV": numSV,
                "lat": lat * 1e-7,
                "lon": lon * 1e-7,
                "height": height * 1e-3,
                "hMSL": hMSL * 1e-3,
                "hAcc": hAcc * 1e-3,
                "vAcc": vAcc * 1e-3,
                "pDOP": pDOP * 0.01,
                "gSpeed": gSpeed * 1e-3,
                "cog": cog,
                "headMot": headMot,
                "headVeh": headVeh,
                "headAcc": headAcc,
                "headVehValid": head_veh_valid,
                "invalidLlh": invalidLlh,
                "lastCorrectionAge": lastCorrectionAge,
            }
            self.last_pvt = out
            return out
        except Exception as e:
            logger.error(f"Error parsing NAV-PVT: {e}")
            return None

    def poll_esf_status(self) -> Optional[Dict]:
        """Poll UBX-ESF-STATUS for F9R sensor fusion status."""
        self._send_ubx_message(0x10, 0x10, b"")
        msg = self._read_ubx_message(timeout=0.5)
        if not msg:
            return None
        msg_class, msg_id, payload = msg
        if msg_class != 0x10 or msg_id != 0x10 or len(payload) < 16:
            return None

        try:
            iTOW = struct.unpack("<I", payload[0:4])[0]
            version = payload[4]
            fusionMode = payload[12]
            numSens = payload[15]

            fusion_labels = {
                0: "initialization",
                1: "active",
                2: "suspended",
                3: "disabled",
            }

            sensors = []
            for i in range(numSens):
                offset = 16 + i * 4
                if offset + 4 > len(payload):
                    break
                sens_data = struct.unpack("<BBBB", payload[offset:offset + 4])
                sens_type = sens_data[0] & 0x3F
                is_used = bool(sens_data[0] & 0x40)
                is_ready = bool(sens_data[0] & 0x80)
                calib_status = sens_data[1] & 0x03
                time_status = (sens_data[1] >> 2) & 0x03

                type_labels = {
                    0: "none", 1: "gyro_z", 2: "wheel_fl", 3: "wheel_fr",
                    4: "wheel_rl", 5: "wheel_rr", 6: "speed", 7: "gyro_y",
                    8: "gyro_x", 9: "accel_z", 10: "accel_y", 11: "accel_x",
                }
                calib_labels = {
                    0: "not_calibrated",
                    1: "calibrating",
                    2: "calibrated",
                    3: "calibrated",
                }

                sensors.append({
                    "type": type_labels.get(sens_type, f"unknown_{sens_type}"),
                    "used": is_used,
                    "ready": is_ready,
                    "calibration": calib_labels.get(
                        calib_status, f"unknown_{calib_status}"
                    ),
                    "time_status": time_status,
                })

            return {
                "iTOW": iTOW,
                "version": version,
                "fusionMode": fusionMode,
                "fusionModeLabel": fusion_labels.get(fusionMode, f"unknown_{fusionMode}"),
                "numSensors": numSens,
                "sensors": sensors,
            }
        except Exception as e:
            logger.error(f"Error parsing ESF-STATUS: {e}")
            return None

    def poll_nav_hpposllh(self) -> Optional[Dict]:
        """Poll UBX-NAV-HPPOSLLH for high-precision position.

        Returns sub-cm lat/lon/height by adding a small "Hp" byte
        to each standard value. Standard scaling is 1e-7 deg (lat/lon)
        and mm (height); Hp extensions add 1e-9 deg and 0.1 mm.
        """
        self._send_ubx_message(self.UBX_NAV_CLASS, 0x14, b"")
        msg = self._read_ubx_message(timeout=0.5)
        if not msg:
            return None
        msg_class, msg_id, payload = msg
        if msg_class != self.UBX_NAV_CLASS or msg_id != 0x14 or len(payload) < 36:
            return None
        try:
            version = payload[0]
            flags = payload[3]
            iTOW = struct.unpack("<I", payload[4:8])[0]
            lon, lat, height, hMSL = struct.unpack("<iiii", payload[8:24])
            lonHp, latHp, heightHp, hMSLHp = struct.unpack("<bbbb", payload[24:28])
            hAcc, vAcc = struct.unpack("<II", payload[28:36])
            invalidLlh = bool(flags & 0x01)
            lat_deg = lat * 1e-7 + latHp * 1e-9
            lon_deg = lon * 1e-7 + lonHp * 1e-9
            height_m = height * 1e-3 + heightHp * 1e-4
            hMSL_m = hMSL * 1e-3 + hMSLHp * 1e-4
            return {
                "version": version,
                "invalidLlh": invalidLlh,
                "iTOW": iTOW,
                "lat": lat_deg,
                "lon": lon_deg,
                "height": height_m,
                "hMSL": hMSL_m,
                "lat_raw": lat,
                "lon_raw": lon,
                "latHp": latHp,
                "lonHp": lonHp,
                "hAcc": hAcc * 1e-4,  # HPPOSLLH hAcc is in 0.1 mm
                "vAcc": vAcc * 1e-4,
            }
        except Exception as e:
            logger.error(f"Error parsing NAV-HPPOSLLH: {e}")
            return None

    def poll_nav_dop(self) -> Optional[Dict]:
        """Poll UBX-NAV-DOP for dilution-of-precision breakdown."""
        self._send_ubx_message(self.UBX_NAV_CLASS, 0x04, b"")
        msg = self._read_ubx_message(timeout=0.5)
        if not msg:
            return None
        msg_class, msg_id, payload = msg
        if msg_class != self.UBX_NAV_CLASS or msg_id != 0x04 or len(payload) < 18:
            return None
        try:
            iTOW = struct.unpack("<I", payload[0:4])[0]
            gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP = struct.unpack(
                "<HHHHHHH", payload[4:18]
            )
            return {
                "iTOW": iTOW,
                "gDOP": gDOP * 0.01,
                "pDOP": pDOP * 0.01,
                "tDOP": tDOP * 0.01,
                "vDOP": vDOP * 0.01,
                "hDOP": hDOP * 0.01,
                "nDOP": nDOP * 0.01,
                "eDOP": eDOP * 0.01,
            }
        except Exception as e:
            logger.error(f"Error parsing NAV-DOP: {e}")
            return None

    def poll_nav_sat(self) -> Optional[Dict]:
        """Poll UBX-NAV-SAT for per-satellite info (variable length).

        Each 12-byte SV record yields gnssId, svId, cno, elev, azim,
        prRes, flags (including used-in-solution bit).
        """
        self._send_ubx_message(self.UBX_NAV_CLASS, 0x35, b"")
        msg = self._read_ubx_message(timeout=1.0)
        if not msg:
            return None
        msg_class, msg_id, payload = msg
        if msg_class != self.UBX_NAV_CLASS or msg_id != 0x35 or len(payload) < 8:
            return None
        try:
            iTOW = struct.unpack("<I", payload[0:4])[0]
            version = payload[4]
            numSvs = payload[5]
            gnss_labels = {
                0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou",
                4: "IMES", 5: "QZSS", 6: "GLONASS", 7: "NavIC",
            }
            sats = []
            for i in range(numSvs):
                off = 8 + i * 12
                if off + 12 > len(payload):
                    break
                gnssId, svId, cno, elev = struct.unpack("<BBBb", payload[off:off + 4])
                azim, prRes = struct.unpack("<hh", payload[off + 4:off + 8])
                flags = struct.unpack("<I", payload[off + 8:off + 12])[0]
                sats.append({
                    "gnssId": gnssId,
                    "gnss": gnss_labels.get(gnssId, f"unknown_{gnssId}"),
                    "svId": svId,
                    "cno": cno,
                    "elev": elev,
                    "azim": azim,
                    "prRes": prRes * 0.1,
                    "used": bool(flags & 0x08),
                    "qualityInd": flags & 0x07,
                    "health": (flags >> 4) & 0x03,
                })
            return {"iTOW": iTOW, "version": version, "numSvs": numSvs, "sats": sats}
        except Exception as e:
            logger.error(f"Error parsing NAV-SAT: {e}")
            return None

    def poll_nav_status(self) -> Optional[Dict]:
        """Poll UBX-NAV-STATUS for differential age and timing flags.

        `msss` is milliseconds since startup; pair with `ttff` (time
        to first fix) and `flags2` differential-correction-applied bits
        to get correction age.
        """
        self._send_ubx_message(self.UBX_NAV_CLASS, 0x03, b"")
        msg = self._read_ubx_message(timeout=0.5)
        if not msg:
            return None
        msg_class, msg_id, payload = msg
        if msg_class != self.UBX_NAV_CLASS or msg_id != 0x03 or len(payload) < 16:
            return None
        try:
            iTOW = struct.unpack("<I", payload[0:4])[0]
            gpsFix = payload[4]
            flags = payload[5]
            fixStat = payload[6]
            flags2 = payload[7]
            ttff = struct.unpack("<I", payload[8:12])[0]
            msss = struct.unpack("<I", payload[12:16])[0]
            return {
                "iTOW": iTOW,
                "gpsFix": gpsFix,
                "gpsFixOk": bool(flags & 0x01),
                "diffSoln": bool(flags & 0x02),
                "wknSet": bool(flags & 0x04),
                "towSet": bool(flags & 0x08),
                "fixStat": fixStat,
                "psmState": flags2 & 0x03,
                "spoofDetState": (flags2 >> 3) & 0x03,
                "carrSolnStatus": (flags2 >> 6) & 0x03,
                "ttff_ms": ttff,
                "msss_ms": msss,
            }
        except Exception as e:
            logger.error(f"Error parsing NAV-STATUS: {e}")
            return None

    def poll_mon_ver(self) -> Optional[Dict]:
        """Poll UBX-MON-VER for firmware/hardware version strings.

        Variable length: 40 bytes of sw/hw version plus zero or more
        30-byte extension strings (e.g. protocol version, GNSS
        capabilities, firmware build ID).
        """
        self._send_ubx_message(0x0A, 0x04, b"")
        msg = self._read_ubx_message(timeout=0.5)
        if not msg:
            return None
        msg_class, msg_id, payload = msg
        if msg_class != 0x0A or msg_id != 0x04 or len(payload) < 40:
            return None
        try:
            sw = payload[0:30].rstrip(b"\x00").decode("ascii", errors="replace")
            hw = payload[30:40].rstrip(b"\x00").decode("ascii", errors="replace")
            extensions = []
            off = 40
            while off + 30 <= len(payload):
                ext = payload[off:off + 30].rstrip(b"\x00").decode(
                    "ascii", errors="replace"
                )
                if ext:
                    extensions.append(ext)
                off += 30
            return {"sw": sw, "hw": hw, "extensions": extensions}
        except Exception as e:
            logger.error(f"Error parsing MON-VER: {e}")
            return None

    def get_position(self) -> Optional[RTKPosition]:
        """Get current position as RTKPosition."""
        pvt = self.poll_nav_pvt()
        if not pvt:
            return None

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
            head_vehicle=pvt["headVeh"] * 1e-5 if pvt["headVehValid"] else None,
            head_vehicle_accuracy=pvt["headAcc"] * 1e-5,
            altitude_msl=pvt["hMSL"],
            pdop=pvt["pDOP"],
            correction_age_bin=pvt["lastCorrectionAge"],
        )

    def wait_for_rtk_fix(self, timeout: float = 300.0, min_fix_type: int = 4) -> bool:
        """Block until GPS fix is achieved, or timeout."""
        logger.info(f"Waiting for GPS fix (min type {min_fix_type})...")
        start = time.time()
        last_progress = start
        while time.time() - start < timeout:
            pos = self.get_position()
            if pos and pos.fix_type >= min_fix_type:
                fix_names = {3: "3D Fix", 4: "GNSS+DR", 5: "RTK Float", 6: "RTK Fixed"}
                fix_label = fix_names.get(pos.fix_type, "Fix")
                msl_str = (
                    f" hMSL: {pos.altitude_msl:.2f}m" if pos.altitude_msl is not None else ""
                )
                pdop_str = f" pDOP: {pos.pdop:.2f}" if pos.pdop is not None else ""
                logger.info(
                    f"{fix_label} achieved! "
                    f"Lat: {pos.latitude:.8f}, Lon: {pos.longitude:.8f}, "
                    f"h: {pos.altitude:.2f}m,{msl_str} "
                    f"hAcc: {pos.accuracy_horizontal:.3f}m{pdop_str}"
                )
                log_banner(
                    f"GPS {fix_label} ACHIEVED | hAcc: {pos.accuracy_horizontal:.3f}m"
                    f"{pdop_str}",
                    logger=logger,
                )
                return True
            now = time.time()
            if now - last_progress >= 15.0:
                elapsed = now - start
                fix_type = pos.fix_type if pos else "?"
                num_sv = pos.satellites_used if pos else "?"
                logger.info(
                    f"Waiting for GPS fix... ({elapsed:.0f}s elapsed, "
                    f"type {fix_type}, {num_sv} SVs)"
                )
                last_progress = now
            time.sleep(1.0)
        elapsed = time.time() - start
        logger.error(f"GPS fix timeout after {elapsed:.0f}s")
        log_banner(
            f"GPS FIX TIMEOUT after {elapsed:.0f}s",
            level="error",
            char="!",
            logger=logger,
        )
        return False


class GPSManager:
    """High-level GPS manager with NTRIP support.

    Explicit args take precedence. If port/baudrate/ntrip_config are
    omitted, the constructor falls back to GPS_PORT / GPS_BAUD / EMLID_*
    environment variables for backward compatibility.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: Optional[int] = None,
        ntrip_config: Optional[NTRIPConfig] = None,
    ):
        self.port = port or os.getenv("GPS_PORT", "/dev/ttyACM0")
        self.baudrate = baudrate or int(os.getenv("GPS_BAUD", "38400"))

        if ntrip_config is None:
            ntrip_config = NTRIPConfig(
                host=os.getenv("EMLID_NTRIP_HOST", "caster.emlid.com"),
                port=int(os.getenv("EMLID_NTRIP_PORT", "2101")),
                mountpoint=os.getenv("EMLID_MOUNTPOINT", "MP15774"),
                username=os.getenv("EMLID_USERNAME", "u65352"),
                password=os.getenv("EMLID_PASSWORD", "338zca"),
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

    def disconnect(self) -> None:
        self.ntrip.disconnect()
        self.gps.disconnect()
        self._connected = False

    def get_position(self) -> Optional[RTKPosition]:
        return self.gps.get_position()

    def wait_for_fix(self, timeout: float = 300.0, min_fix_type: int = 4) -> bool:
        return self.gps.wait_for_rtk_fix(timeout=timeout, min_fix_type=min_fix_type)
