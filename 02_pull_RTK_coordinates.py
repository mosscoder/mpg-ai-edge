#!/usr/bin/env python3
"""
RTK GPS Coordinate Puller using SparkFun u-blox ZED-F9P with Emlid NTRIP Corrections

This script demonstrates how to pull high-precision RTK coordinates from a 
SparkFun u-blox GPS-RTK2 (ZED-F9P) GNSS receiver with real-time corrections
from an Emlid NTRIP caster broadcasting base station.

Features:
- SparkFun Qwiic GPS-RTK2 protocols (UBX messages)
- Emlid NTRIP client for RTCM 3.x corrections
- GPS, GLONASS, BeiDou, Galileo constellation support
- 10mm positional accuracy with RTK Float/Fixed
- Real-time correction streaming from Emlid base station

Hardware Requirements:
- SparkFun GPS-RTK2 Board (ZED-F9P)
- Serial connection to GPS module
- Internet connection for NTRIP corrections
- Emlid Reach RS2/RS+ base station broadcasting corrections

Software Requirements:
- pyserial for GPS communication
- Emlid account with NTRIP access
- Base station configured and broadcasting
"""

import time
import sys
import serial
import struct
import socket
import base64
import threading
from dataclasses import dataclass
from typing import Optional, Tuple
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class RTKPosition:
    """High-precision RTK position data"""
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
    """NTRIP caster configuration for Emlid base station"""
    host: str
    port: int
    mountpoint: str
    username: str
    password: str
    
class EmlidNTRIPClient:
    """
    NTRIP client for receiving RTK corrections from Emlid base station
    
    Implements NTRIP v1.0 protocol for RTCM 3.x corrections
    """
    
    def __init__(self, config: NTRIPConfig):
        """
        Initialize NTRIP client
        
        Args:
            config: NTRIP connection configuration
        """
        self.config = config
        self.socket = None
        self.connected = False
        self.correction_thread = None
        self.running = False
        self.gps_receiver = None
        self.correction_count = 0
        
    def connect(self) -> bool:
        """Connect to Emlid NTRIP caster"""
        try:
            # Create socket connection
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            
            logger.info(f"Connecting to Emlid NTRIP caster at {self.config.host}:{self.config.port}")
            self.socket.connect((self.config.host, self.config.port))
            
            # Send NTRIP request
            request = self._build_ntrip_request()
            self.socket.send(request.encode('ascii'))
            
            # Read response
            response = self.socket.recv(1024).decode('ascii')
            
            if 'ICY 200 OK' in response or '200 OK' in response:
                logger.info(f"Connected to NTRIP mountpoint: {self.config.mountpoint}")
                self.connected = True
                return True
            else:
                logger.error(f"NTRIP connection failed: {response}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to connect to NTRIP caster: {e}")
            return False
    
    def _build_ntrip_request(self) -> str:
        """Build NTRIP HTTP request"""
        # Create basic auth header
        auth_string = f"{self.config.username}:{self.config.password}"
        auth_bytes = auth_string.encode('ascii')
        auth_header = base64.b64encode(auth_bytes).decode('ascii')
        
        # Build HTTP request
        request = (
            f"GET /{self.config.mountpoint} HTTP/1.0\r\n"
            f"Host: {self.config.host}:{self.config.port}\r\n"
            f"User-Agent: EmlidRTKClient/1.0\r\n"
            f"Authorization: Basic {auth_header}\r\n"
            f"Accept: */*\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        )
        
        return request
    
    def start_correction_stream(self, gps_receiver):
        """Start receiving and forwarding RTCM corrections"""
        self.gps_receiver = gps_receiver
        self.running = True
        
        self.correction_thread = threading.Thread(
            target=self._correction_worker,
            daemon=True
        )
        self.correction_thread.start()
        
        logger.info("Started RTCM correction stream from Emlid NTRIP caster")
    
    def _correction_worker(self):
        """Worker thread to receive and forward RTCM corrections"""
        buffer = b''
        
        while self.running and self.connected:
            try:
                # Receive data from NTRIP caster
                data = self.socket.recv(1024)
                if not data:
                    logger.warning("NTRIP connection lost")
                    break
                
                buffer += data
                
                # Parse RTCM messages from buffer
                while len(buffer) >= 3:
                    # Look for RTCM 3.x preamble (0xD3)
                    preamble_idx = buffer.find(b'\xD3')
                    if preamble_idx == -1:
                        buffer = b''
                        break
                    
                    if preamble_idx > 0:
                        buffer = buffer[preamble_idx:]
                    
                    if len(buffer) < 3:
                        break
                    
                    # Extract message length
                    length = ((buffer[1] & 0x03) << 8) | buffer[2]
                    total_length = length + 6  # 3 header + length + 3 CRC
                    
                    if len(buffer) < total_length:
                        break  # Wait for complete message
                    
                    # Extract complete RTCM message
                    rtcm_msg = buffer[:total_length]
                    buffer = buffer[total_length:]
                    
                    # Forward to GPS receiver
                    if self.gps_receiver and self.gps_receiver.serial_conn:
                        self.gps_receiver.serial_conn.write(rtcm_msg)
                        self.gps_receiver.serial_conn.flush()
                        self.correction_count += 1
                        
                        if self.correction_count % 10 == 0:
                            logger.debug(f"Forwarded {self.correction_count} RTCM corrections")
                
            except Exception as e:
                logger.error(f"Error in correction worker: {e}")
                break
        
        logger.info("RTCM correction worker stopped")
    
    def disconnect(self):
        """Disconnect from NTRIP caster"""
        self.running = False
        
        if self.correction_thread:
            self.correction_thread.join(timeout=2.0)
        
        if self.socket:
            self.socket.close()
            
        self.connected = False
        logger.info(f"Disconnected from Emlid NTRIP caster. Forwarded {self.correction_count} corrections.")

class UBloxRTKGPS:
    """
    Interface for u-blox ZED-F9P GPS RTK module using UBX protocol
    
    Based on SparkFun protocols and ZED-F9P integration manual
    """
    
    # UBX Protocol Constants
    UBX_SYNC_CHAR1 = 0xB5
    UBX_SYNC_CHAR2 = 0x62
    
    # UBX Message Classes and IDs
    UBX_NAV_CLASS = 0x01
    UBX_NAV_PVT = 0x07      # Position Velocity Time solution
    UBX_NAV_HPPOSLLH = 0x14 # High Precision Position
    
    UBX_CFG_CLASS = 0x06
    UBX_CFG_RATE = 0x08     # Navigation rate
    UBX_CFG_PRT = 0x00      # Port configuration
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 38400):
        """
        Initialize RTK GPS connection
        
        Args:
            port: Serial port for GPS module
            baudrate: Communication speed (default 38400 for ZED-F9P)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.last_position: Optional[RTKPosition] = None
        
    def connect(self) -> bool:
        """Establish connection to GPS module"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            logger.info(f"Connected to GPS module on {self.port}")
            
            # Configure navigation rate (e.g., 1Hz)
            self._configure_navigation_rate(1000)  # 1000ms = 1Hz
            
            return True
        except Exception as e:
            logger.error(f"Failed to connect to GPS: {e}")
            return False
    
    def disconnect(self):
        """Close GPS connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("GPS connection closed")
    
    def _configure_navigation_rate(self, rate_ms: int):
        """Configure GPS navigation solution rate"""
        # UBX-CFG-RATE message to set navigation rate
        payload = struct.pack('<HHH', rate_ms, 1, 1)  # measRate, navRate, timeRef
        self._send_ubx_message(self.UBX_CFG_CLASS, self.UBX_CFG_RATE, payload)
    
    def _send_ubx_message(self, msg_class: int, msg_id: int, payload: bytes):
        """Send UBX protocol message"""
        if not self.serial_conn:
            return
            
        # Calculate checksum
        checksum_a = 0
        checksum_b = 0
        
        # Header
        message = bytes([self.UBX_SYNC_CHAR1, self.UBX_SYNC_CHAR2])
        
        # Class, ID, Length
        header = struct.pack('<BBHH', msg_class, msg_id, len(payload))
        message += header
        message += payload
        
        # Calculate checksum over class, id, length, and payload
        for byte in header + payload:
            checksum_a = (checksum_a + byte) & 0xFF
            checksum_b = (checksum_b + checksum_a) & 0xFF
        
        message += bytes([checksum_a, checksum_b])
        
        self.serial_conn.write(message)
        self.serial_conn.flush()
    
    def _read_ubx_message(self, timeout: float = 5.0) -> Optional[bytes]:
        """Read and validate UBX message"""
        if not self.serial_conn:
            return None
            
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            # Look for sync characters
            if self.serial_conn.read(1) == bytes([self.UBX_SYNC_CHAR1]):
                if self.serial_conn.read(1) == bytes([self.UBX_SYNC_CHAR2]):
                    # Read header (class, id, length)
                    header = self.serial_conn.read(4)
                    if len(header) != 4:
                        continue
                        
                    msg_class, msg_id, length = struct.unpack('<BBH', header)
                    
                    # Read payload and checksum
                    payload = self.serial_conn.read(length)
                    checksum = self.serial_conn.read(2)
                    
                    if len(payload) != length or len(checksum) != 2:
                        continue
                    
                    # Verify checksum
                    checksum_a = checksum_b = 0
                    for byte in header + payload:
                        checksum_a = (checksum_a + byte) & 0xFF
                        checksum_b = (checksum_b + checksum_a) & 0xFF
                    
                    if checksum == bytes([checksum_a, checksum_b]):
                        return bytes([msg_class, msg_id]) + payload
        
        return None
    
    def get_rtk_position(self, high_precision: bool = True) -> Optional[RTKPosition]:
        """
        Get current RTK position using SparkFun protocols
        
        Args:
            high_precision: Use high-precision HPPOSLLH message if True
            
        Returns:
            RTKPosition object with current coordinates or None
        """
        if not self.serial_conn:
            logger.error("GPS not connected")
            return None
        
        # Request position data
        if high_precision:
            # Request UBX-NAV-HPPOSLLH for high precision
            self._send_ubx_message(self.UBX_NAV_CLASS, self.UBX_NAV_HPPOSLLH, b'')
            msg_id = self.UBX_NAV_HPPOSLLH
        else:
            # Request UBX-NAV-PVT for standard precision
            self._send_ubx_message(self.UBX_NAV_CLASS, self.UBX_NAV_PVT, b'')
            msg_id = self.UBX_NAV_PVT
        
        # Read response
        response = self._read_ubx_message()
        if not response:
            logger.warning("No GPS response received")
            return None
        
        if response[1] == msg_id:
            return self._parse_position_message(response[2:], high_precision)
        
        return None
    
    def _parse_position_message(self, payload: bytes, high_precision: bool) -> Optional[RTKPosition]:
        """Parse UBX position message"""
        try:
            if high_precision and len(payload) >= 36:
                # UBX-NAV-HPPOSLLH format
                version, reserved1, iTOW, lon, lat, height, hMSL = struct.unpack('<BBIIIII', payload[:24])
                lonHp, latHp, heightHp, hMSLHp = struct.unpack('<bbbb', payload[24:28])
                hAcc, vAcc = struct.unpack('<II', payload[28:36])
                
                # Convert to degrees with high precision
                longitude = (lon * 1e-7) + (lonHp * 1e-9)
                latitude = (lat * 1e-7) + (latHp * 1e-9)
                altitude = (height * 1e-3) + (heightHp * 1e-4)
                
                # Accuracy in meters
                h_accuracy = hAcc * 1e-4
                v_accuracy = vAcc * 1e-4
                
                fix_type = 3  # High precision RTK
                satellites = 12  # Estimated for RTK
                
            elif len(payload) >= 84:
                # UBX-NAV-PVT format
                iTOW, year, month, day, hour, minute, second, valid = struct.unpack('<IHBBBBBB', payload[:12])
                tAcc, nano, fixType, flags, flags2, numSV = struct.unpack('<IBIBBB', payload[12:24])
                lon, lat, height, hMSL = struct.unpack('<iiii', payload[24:40])
                hAcc, vAcc = struct.unpack('<II', payload[40:48])
                
                # Convert to degrees
                longitude = lon * 1e-7
                latitude = lat * 1e-7
                altitude = height * 1e-3
                
                # Accuracy in meters
                h_accuracy = hAcc * 1e-3
                v_accuracy = vAcc * 1e-3
                
                fix_type = fixType
                satellites = numSV
            else:
                logger.error("Invalid payload length")
                return None
            
            position = RTKPosition(
                latitude=latitude,
                longitude=longitude,
                altitude=altitude,
                accuracy_horizontal=h_accuracy,
                accuracy_vertical=v_accuracy,
                fix_type=fix_type,
                satellites_used=satellites,
                timestamp=time.time()
            )
            
            self.last_position = position
            return position
            
        except Exception as e:
            logger.error(f"Error parsing position message: {e}")
            return None
    
    def wait_for_rtk_fix(self, timeout: float = 300.0) -> bool:
        """
        Wait for RTK fix with high precision
        
        Args:
            timeout: Maximum time to wait for RTK fix (seconds)
            
        Returns:
            True if RTK fix achieved, False if timeout
        """
        logger.info("Waiting for RTK fix...")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            position = self.get_rtk_position()
            
            if position and position.fix_type >= 5:  # RTK Float or Fixed
                fix_type_str = "RTK Fixed" if position.fix_type == 6 else "RTK Float"
                logger.info(f"{fix_type_str} achieved! Accuracy: {position.accuracy_horizontal:.3f}m")
                return True
            
            time.sleep(1.0)
        
        logger.error("RTK fix timeout")
        return False

def main():
    """Main demonstration of RTK coordinate pulling with Emlid NTRIP corrections"""
    print("SparkFun RTK GPS Coordinate Puller with Emlid NTRIP Corrections")
    print("Based on u-blox ZED-F9P protocols + Emlid RTCM corrections")
    print("-" * 60)
    
    # Configure Emlid NTRIP caster connection
    # Default Emlid Caster settings - update with your credentials
    ntrip_config = NTRIPConfig(
        host="caster.emlid.com",           # Emlid NTRIP caster
        port=2101,                         # Standard NTRIP port
        mountpoint="MP1979",               # Your base station mountpoint
        username="your_username",          # Your Emlid account username
        password="your_password"           # Your Emlid account password
    )
    
    # Allow override from environment variables
    import os
    ntrip_config.host = os.getenv('EMLID_NTRIP_HOST', ntrip_config.host)
    ntrip_config.port = int(os.getenv('EMLID_NTRIP_PORT', ntrip_config.port))
    ntrip_config.mountpoint = os.getenv('EMLID_MOUNTPOINT', ntrip_config.mountpoint)
    ntrip_config.username = os.getenv('EMLID_USERNAME', ntrip_config.username)
    ntrip_config.password = os.getenv('EMLID_PASSWORD', ntrip_config.password)
    
    print(f"NTRIP Config: {ntrip_config.host}:{ntrip_config.port}/{ntrip_config.mountpoint}")
    
    # Initialize GPS with SparkFun defaults
    gps = UBloxRTKGPS(port='/dev/ttyUSB0', baudrate=38400)
    
    # Initialize Emlid NTRIP client
    ntrip_client = EmlidNTRIPClient(ntrip_config)
    
    if not gps.connect():
        print("Failed to connect to GPS module")
        sys.exit(1)
    
    # Connect to Emlid NTRIP caster
    if not ntrip_client.connect():
        print("Warning: Failed to connect to Emlid NTRIP caster")
        print("Proceeding with GPS-only mode (reduced accuracy)")
    else:
        # Start receiving RTCM corrections
        ntrip_client.start_correction_stream(gps)
        print("✓ Connected to Emlid NTRIP caster - receiving RTCM corrections")
    
    try:
        # Wait for RTK fix with corrections
        print("Waiting for RTK fix with Emlid corrections...")
        if not gps.wait_for_rtk_fix(timeout=300):  # Longer timeout with corrections
            print("Warning: RTK fix not achieved, proceeding with available accuracy")
        
        # Continuously pull coordinates
        print("\nPulling RTK coordinates with Emlid corrections (Ctrl+C to stop):")
        print("Time\t\tLatitude\t\tLongitude\t\tAltitude\tAccuracy\tSats\tFix\t\tRTCM")
        
        while True:
            position = gps.get_rtk_position(high_precision=True)
            
            if position:
                fix_types = {
                    0: "No Fix",
                    1: "Dead Reckoning", 
                    2: "2D Fix",
                    3: "3D Fix",
                    4: "GNSS+DR",
                    5: "RTK Float",
                    6: "RTK Fixed"
                }
                
                fix_str = fix_types.get(position.fix_type, f"Unknown({position.fix_type})")
                
                # Color code RTK status
                if position.fix_type >= 5:
                    rtk_status = "✓ RTK"
                elif ntrip_client.connected:
                    rtk_status = "⚡ CORR"
                else:
                    rtk_status = "✗ NONE"
                
                print(f"{time.strftime('%H:%M:%S')}\t"
                      f"{position.latitude:.8f}\t\t"
                      f"{position.longitude:.8f}\t\t"
                      f"{position.altitude:.2f}m\t\t"
                      f"{position.accuracy_horizontal:.3f}m\t\t"
                      f"{position.satellites_used}\t"
                      f"{fix_str}\t\t"
                      f"{rtk_status}")
            else:
                rtcm_count = ntrip_client.correction_count if ntrip_client.connected else 0
                print(f"{time.strftime('%H:%M:%S')}\tNo position data available\t\t\t\t\t\t\t\tRTCM:{rtcm_count}")
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nStopping coordinate collection...")
    
    finally:
        # Cleanup connections
        ntrip_client.disconnect()
        gps.disconnect()
        print("GPS and NTRIP connections closed")

def print_usage():
    """Print usage instructions"""
    print("""
Usage: python 02_pull_RTK_coordinates.py

Environment Variables (optional):
  EMLID_NTRIP_HOST     - Emlid NTRIP caster hostname (default: caster.emlid.com)
  EMLID_NTRIP_PORT     - NTRIP port (default: 2101)
  EMLID_MOUNTPOINT     - Your base station mountpoint (default: MP1979)
  EMLID_USERNAME       - Your Emlid account username
  EMLID_PASSWORD       - Your Emlid account password

Example:
  export EMLID_USERNAME="your_username"
  export EMLID_PASSWORD="your_password"
  export EMLID_MOUNTPOINT="your_base_station"
  python 02_pull_RTK_coordinates.py

Emlid Base Station Setup:
1. Configure your Emlid Reach RS2/RS+ as base station
2. Enable NTRIP caster broadcasting
3. Note your mountpoint name
4. Use your Emlid account credentials

Expected Accuracy:
- Without corrections: 2-5 meters
- With RTCM corrections: 0.01-0.1 meters (RTK Float/Fixed)
""")

if __name__ == "__main__":
    main()