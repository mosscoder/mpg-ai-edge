#!/usr/bin/env python3
"""
RTK GPS Coordinate Puller using SparkFun u-blox ZED-F9P with Emlid NTRIP Corrections
(Logging-enhanced build)

Adds:
- logs/<timestamp>.log file per run
- Per-second logging of NAV-PVT status (fixType, carrSoln, diffSoln), sats, hAcc/vAcc
- RTCM message type histogram, forwarded byte counter
"""

import time
import sys
import os
import serial, pynmea2
import struct
import socket
import base64
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, Dict
import logging
from collections import Counter

# ---------- Logging setup (console + timestamped file in logs/) ----------
def setup_logging():
    os.makedirs("logs", exist_ok=True)
    start_stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    logfile = os.path.join("logs", f"rtk_{start_stamp}.log")

    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)

    # Avoid duplicate handlers if main() is called twice
    logger.handlers.clear()

    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    ch.setFormatter(fmt)

    fh = logging.FileHandler(logfile)
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)

    logger.addHandler(ch)
    logger.addHandler(fh)

    # Convenience: also capture root logs
    root = logging.getLogger()
    root.handlers.clear()
    root.setLevel(logging.DEBUG)
    root.addHandler(ch)
    root.addHandler(fh)

    return logger, logfile, start_stamp

logger = logging.getLogger(__name__)
# ------------------------------------------------------------------------

@dataclass
class RTKPosition:
    latitude: float
    longitude: float
    altitude: float
    accuracy_horizontal: float
    accuracy_vertical: float
    fix_type: int
    satellites_used: int
    timestamp: float

@dataclass
class NTRIPConfig:
    host: str
    port: int
    mountpoint: str
    username: str
    password: str

class EmlidNTRIPClient:
    """
    NTRIP v1 client. Logs RTCM types and total forwarded bytes.
    """
    def __init__(self, config: NTRIPConfig):
        self.config = config
        self.socket = None
        self.connected = False
        self.correction_thread = None
        self.running = False
        self.gps_receiver = None
        self.correction_count = 0      # number of RTCM messages forwarded
        self.bytes_forwarded = 0       # bytes forwarded to the receiver
        self.rtcm_types = Counter()     # message type histogram
        self._last_types_dump = 0.0

    def connect(self) -> bool:
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            logger.info(f"Connecting to Emlid NTRIP caster at {self.config.host}:{self.config.port}")
            self.socket.connect((self.config.host, self.config.port))
            request = self._build_ntrip_request()
            self.socket.send(request.encode('ascii'))
            response = self.socket.recv(1024).decode('ascii', errors='ignore')
            if 'ICY 200 OK' in response or '200 OK' in response:
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
        auth_header = base64.b64encode(auth_string.encode('ascii')).decode('ascii')
        req = (
            f"GET /{self.config.mountpoint} HTTP/1.0\r\n"
            f"Host: {self.config.host}:{self.config.port}\r\n"
            f"User-Agent: EmlidRTKClient/1.0\r\n"
            f"Authorization: Basic {auth_header}\r\n"
            f"Accept: */*\r\n"
            f"Connection: keep-alive\r\n"
            f"\r\n"
        )
        return req

    def start_correction_stream(self, gps_receiver):
        self.gps_receiver = gps_receiver
        self.running = True
        self.correction_thread = threading.Thread(target=self._correction_worker, daemon=True)
        self.correction_thread.start()
        logger.info("Started RTCM correction stream from Emlid NTRIP caster")

    @staticmethod
    def _rtcm_msg_type_from_payload(payload: bytes) -> Optional[int]:
        # RTCM3: message number = first 12 bits of payload
        if len(payload) < 2:
            return None
        return ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF

    def _correction_worker(self):
        buffer = b''
        last_types_log = time.time()
        while self.running and self.connected:
            try:
                data = self.socket.recv(4096)
                if not data:
                    logger.warning("NTRIP connection lost")
                    break
                buffer += data
                while len(buffer) >= 3:
                    pre_idx = buffer.find(b'\xD3')
                    if pre_idx == -1:
                        buffer = b''
                        break
                    if pre_idx > 0:
                        buffer = buffer[pre_idx:]
                    if len(buffer) < 3:
                        break
                    length = ((buffer[1] & 0x03) << 8) | buffer[2]
                    total_len = length + 6  # 3 hdr + length + 3 CRC
                    if len(buffer) < total_len:
                        break
                    rtcm_msg = buffer[:total_len]
                    buffer = buffer[total_len:]

                    # Track message type & forward
                    payload = rtcm_msg[3:-3]  # after D3/len, before CRC
                    mtype = self._rtcm_msg_type_from_payload(payload)
                    if mtype is not None:
                        self.rtcm_types[mtype] += 1

                    if self.gps_receiver and self.gps_receiver.serial_conn:
                        self.gps_receiver.serial_conn.write(rtcm_msg)
                        self.gps_receiver.serial_conn.flush()
                        self.correction_count += 1
                        self.bytes_forwarded += len(rtcm_msg)

                # Periodically dump a short type summary
                now = time.time()
                if now - last_types_log >= 10.0:
                    if self.rtcm_types:
                        top = ", ".join(f"{k}:{self.rtcm_types[k]}" for k in sorted(self.rtcm_types)[:10])
                        logger.info(f"RTCM types (running counts): {top} | total msgs={self.correction_count}, bytes={self.bytes_forwarded}")
                    last_types_log = now

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
        logger.info(f"Disconnected from Emlid NTRIP caster. Forwarded {self.correction_count} messages ({self.bytes_forwarded} bytes).")

class UBloxRTKGPS:
    """
    u-blox ZED-F9P interface (UBX). Adds PVT polling so we can log carrSoln/diffSoln.
    """
    UBX_SYNC_CHAR1 = 0xB5
    UBX_SYNC_CHAR2 = 0x62
    UBX_NAV_CLASS = 0x01
    UBX_NAV_PVT = 0x07
    UBX_NAV_HPPOSLLH = 0x14
    UBX_CFG_CLASS = 0x06
    UBX_CFG_RATE = 0x08
    UBX_CFG_PRT = 0x00

    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 38400):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.last_position: Optional[RTKPosition] = None
        self.last_nav_pvt: Optional[Dict] = None

    def connect(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            logger.info(f"Connected to GPS module on {self.port} @ {self.baudrate} baud")
            self._configure_navigation_rate(1000)  # 1Hz
            return True
        except Exception as e:
            logger.error(f"Failed to connect to GPS: {e}")
            return False

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("GPS connection closed")

    def _configure_navigation_rate(self, rate_ms: int):
        payload = struct.pack('<HHH', rate_ms, 1, 1)
        self._send_ubx_message(self.UBX_CFG_CLASS, self.UBX_CFG_RATE, payload)

    def _send_ubx_message(self, msg_class: int, msg_id: int, payload: bytes):
        if not self.serial_conn:
            return
        ca = cb = 0
        header = struct.pack('<BBBBH', self.UBX_SYNC_CHAR1, self.UBX_SYNC_CHAR2, msg_class, msg_id, len(payload))
        # checksum over class,id,len,payload (exclude sync 0xB5 0x62)
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
        return bytes(buf) if len(buf) == n else b''

    def _read_ubx_message(self, timeout: float = 2.0) -> Optional[Tuple[int,int,bytes]]:
        if not self.serial_conn:
            return None
        end = time.time() + timeout
        while time.time() < end:
            b = self.serial_conn.read(1)
            if b != b'\xB5':
                continue
            if self.serial_conn.read(1) != b'\x62':
                continue
            hdr = self._read_exact(4, timeout=0.2)
            if not hdr:
                continue
            msg_class, msg_id, length = struct.unpack('<BBH', hdr)
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

    # ---------- NMEA and UBX polling ----------
    def get_nmea_position(self) -> Optional[RTKPosition]:
        if not self.serial_conn:
            logger.error("GPS not connected")
            return None
        line = self.serial_conn.readline().decode('ascii', errors='replace').strip()
        if not line.startswith('$GNGGA'):
            return None
        try:
            msg = pynmea2.parse(line)
            if int(msg.gps_qual) == 0:
                return None
            return RTKPosition(
                latitude=msg.latitude,
                longitude=msg.longitude,
                altitude=float(msg.altitude),
                accuracy_horizontal=float(msg.horizontal_dil),
                accuracy_vertical=float(msg.altitude),  # NMEA has no vAcc
                fix_type=int(msg.gps_qual),
                satellites_used=int(msg.num_sats),
                timestamp=time.time()
            )
        except Exception as e:
            logger.error(f"NMEA parse error: {e}")
            return None

    def poll_nav_pvt(self) -> Optional[Dict]:
        """Poll UBX-NAV-PVT and return parsed dict w/ key status bits."""
        self._send_ubx_message(self.UBX_NAV_CLASS, self.UBX_NAV_PVT, b'')
        msg = self._read_ubx_message(timeout=0.5)
        if not msg:
            return None
        msg_class, msg_id, payload = msg
        if msg_id != self.UBX_NAV_PVT or len(payload) < 92:
            return None

        # Parse the fields we need
        try:
            # offsets reference u-blox Interface Description
            iTOW, year, month, day, hour, minute, second, valid = struct.unpack('<IHBBBBBB', payload[:12])
            tAcc, nano, fixType, flags, flags2, numSV = struct.unpack('<IBIBBB', payload[12:24])
            lon, lat, height, hMSL = struct.unpack('<iiii', payload[24:40])
            hAcc, vAcc = struct.unpack('<II', payload[40:48])
            # In many firmwares carrSoln is the lowest two bits of flags2 (0 none, 1 float, 2 fixed)
            carrSoln = flags2 & 0x03
            diffSoln = (flags & 0x01)  # differential corrections used
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
            }
            self.last_nav_pvt = out
            return out
        except Exception as e:
            logger.error(f"Error parsing NAV-PVT: {e}")
            return None

    def wait_for_rtk_fix(self, timeout: float = 300.0) -> bool:
        logger.info("Waiting for GPS fix via NMEA $GNGGA...")
        start = time.time()
        while time.time() - start < timeout:
            position = self.get_nmea_position()
            if position and position.fix_type >= 3:
                fix_str = "3D Fix" if position.fix_type < 5 else "RTK Fix"
                logger.info(f"{fix_str} achieved! Lat: {position.latitude:.8f}, Lon: {position.longitude:.8f}")
                return True
            time.sleep(1.0)
        logger.error("GPS fix timeout")
        return False

# ---------- Main ----------
def main():
    logger, logfile, start_stamp = setup_logging()

    print("SparkFun RTK GPS Coordinate Puller with Emlid NTRIP Corrections")
    print("Based on u-blox ZED-F9P protocols + Emlid RTCM corrections")
    print("-" * 60)

    # NTRIP config (allow env overrides)
    ntrip_config = NTRIPConfig(
        host=os.getenv('EMLID_NTRIP_HOST', "caster.emlid.com"),
        port=int(os.getenv('EMLID_NTRIP_PORT', "2101")),
        mountpoint=os.getenv('EMLID_MOUNTPOINT', "MP1979"),
        username=os.getenv('EMLID_USERNAME', "your_username"),
        password=os.getenv('EMLID_PASSWORD', "your_password"),
    )
    print(f"NTRIP Config: {ntrip_config.host}:{ntrip_config.port}/{ntrip_config.mountpoint}")
    logger.info(f"Session start: log={logfile}")
    logger.info(f"NTRIP host={ntrip_config.host} port={ntrip_config.port} mount={ntrip_config.mountpoint} user={ntrip_config.username}")

    gps = UBloxRTKGPS(port=os.getenv("GPS_PORT", "/dev/ttyACM0"), baudrate=int(os.getenv("GPS_BAUD", "38400")))
    ntrip_client = EmlidNTRIPClient(ntrip_config)

    if not gps.connect():
        print("Failed to connect to GPS module")
        sys.exit(1)

    if not ntrip_client.connect():
        print("Warning: Failed to connect to Emlid NTRIP caster")
        print("Proceeding with GPS-only mode (reduced accuracy)")
        logger.warning("NTRIP not connected; proceeding GNSS-only")
    else:
        ntrip_client.start_correction_stream(gps)
        print("✓ Connected to Emlid NTRIP caster - receiving RTCM corrections")
        logger.info("Receiving RTCM corrections")

    try:
        print("Waiting for RTK fix with Emlid corrections...")
        gps.wait_for_rtk_fix(timeout=300)

        print("\nPulling RTK coordinates with Emlid corrections (Ctrl+C to stop):")
        print("Time\t\tLatitude\t\tLongitude\t\tAltitude\tAccuracy\tSats\tFix\t\tRTCM")

        last_types_dump_console = time.time()
        while True:
            # Poll UBX PVT each second so we ALWAYS have status bits
            pvt = gps.poll_nav_pvt()

            if pvt:
                # Derive human-readable states
                fix_map = {0:"No Fix",1:"DR only",2:"2D",3:"3D",4:"GNSS+DR",5:"RTK Float",6:"RTK Fixed"}
                carr_map = {0:"none",1:"float",2:"fixed"}
                # If carrSoln present, prefer it (float/fixed)
                effective_fix = 6 if pvt["carrSoln"] == 2 else (5 if pvt["carrSoln"] == 1 else pvt["fixType"])
                fix_str = fix_map.get(effective_fix, f"Unknown({effective_fix})")
                rtk_status = "✓ RTK" if pvt["carrSoln"] in (1,2) else ("⚡ CORR" if ntrip_client.connected else "✗ NONE")

                # Console line (kept from your version, but now based on UBX PVT)
                print(f"{time.strftime('%H:%M:%S')}\t"
                      f"{pvt['lat']:.8f}\t\t"
                      f"{pvt['lon']:.8f}\t\t"
                      f"{pvt['height']:.2f}m\t\t"
                      f"{pvt['hAcc']:.3f}m\t\t"
                      f"{pvt['numSV']}\t"
                      f"{fix_str}\t\t"
                      f"{rtk_status}")

                # Log a structured line to the file
                logger.info(
                    f"PVT fixType={pvt['fixType']} carrSoln={carr_map.get(pvt['carrSoln'],'?')} "
                    f"diffSoln={pvt['diffSoln']} numSV={pvt['numSV']} "
                    f"lat={pvt['lat']:.8f} lon={pvt['lon']:.8f} h={pvt['height']:.3f}m "
                    f"hAcc={pvt['hAcc']:.3f}m vAcc={pvt['vAcc']:.3f}m "
                    f"rtcm_msgs={ntrip_client.correction_count} rtcm_bytes={ntrip_client.bytes_forwarded}"
                )
            else:
                # Fall back to NMEA if PVT didn’t arrive this tick
                pos = gps.get_nmea_position()
                if pos:
                    fix_types = {0:"No Fix",1:"Dead Reckoning",2:"2D Fix",3:"3D Fix",4:"GNSS+DR",5:"RTK Float",6:"RTK Fixed"}
                    fix_str = fix_types.get(pos.fix_type, f"Unknown({pos.fix_type})")
                    rtk_status = "✓ RTK" if pos.fix_type >= 5 else ("⚡ CORR" if ntrip_client.connected else "✗ NONE")
                    print(f"{time.strftime('%H:%M:%S')}\t"
                          f"{pos.latitude:.8f}\t\t{pos.longitude:.8f}\t\t{pos.altitude:.2f}m\t\t"
                          f"{pos.accuracy_horizontal:.3f}m\t\t{pos.satellites_used}\t{fix_str}\t\t{rtk_status}")
                    logger.info(
                        f"NMEA GGA fix={fix_str} numSV={pos.satellites_used} "
                        f"lat={pos.latitude:.8f} lon={pos.longitude:.8f} h={pos.altitude:.3f}m "
                        f"rtcm_msgs={ntrip_client.correction_count} rtcm_bytes={ntrip_client.bytes_forwarded}"
                    )
                else:
                    rtcm_count = ntrip_client.correction_count if ntrip_client.connected else 0
                    print(f"{time.strftime('%H:%M:%S')}\tNo position data available\t\t\t\t\t\t\t\tRTCM:{rtcm_count}")
                    logger.warning(f"No position data this second | rtcm_msgs={rtcm_count} bytes={ntrip_client.bytes_forwarded}")

            # Also print (to log) a concise RTCM type summary every 10s
            if time.time() - last_types_dump_console >= 10.0 and ntrip_client.rtcm_types:
                top = ", ".join(f"{t}:{ntrip_client.rtcm_types[t]}" for t in sorted(ntrip_client.rtcm_types))
                logger.info(f"RTCM type histogram: {top}")
                last_types_dump_console = time.time()

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopping coordinate collection...")
        logger.info("KeyboardInterrupt: stopping")

    finally:
        ntrip_client.disconnect()
        gps.disconnect()
        print("GPS and NTRIP connections closed")
        logger.info("Session end")

def print_usage():
    print("""
Usage: python 02_pull_RTK_coordinates.py
Environment Variables:
  EMLID_NTRIP_HOST, EMLID_NTRIP_PORT, EMLID_MOUNTPOINT, EMLID_USERNAME, EMLID_PASSWORD
  GPS_PORT (default /dev/ttyACM0), GPS_BAUD (default 38400)
Logs:
  logs/rtk_<YYYY-mm-dd_HH-MM-SS>.log contains per-second PVT status & RTCM stats.
""")

if __name__ == "__main__":
    main()
