# Accessing IMU Data via unitree_webrtc_connect

This document describes how to access IMU (Inertial Measurement Unit) data from the Go2 robot using the `unitree_webrtc_connect` library.

## Overview

The Go2 robot provides IMU data through its WebRTC data channel interface. The `unitree_webrtc_connect` library abstracts the connection and provides a pub/sub mechanism for subscribing to sensor data topics.

**Library:** https://github.com/legion1581/unitree_webrtc_connect
**Installation:** `pip install unitree_webrtc_connect`

## Available Topics

IMU data is available through several topics, each with different update frequencies and data payloads:

| Topic Constant | Topic String | Description |
|---------------|--------------|-------------|
| `LOW_STATE` | `rt/lf/lowstate` | Low-level state including IMU, motor states, and battery |
| `SPORT_MOD_STATE` | `rt/sportmodestate` | Sport mode state with IMU + navigation data |
| `LF_SPORT_MOD_STATE` | `rt/lf/sportmodestate` | Low-frequency sport mode state (recommended for general use) |

**Recommendation:** Use `LF_SPORT_MOD_STATE` for most applications. It provides a good balance of update rate and includes both IMU and locomotion state data.

## IMU Data Structure

The IMU state is nested within the sport mode state message under `data['imu_state']`:

```python
imu_state = message['data']['imu_state']

# Available fields:
quaternion = imu_state['quaternion']       # [w, x, y, z] - Orientation quaternion
gyroscope = imu_state['gyroscope']         # [x, y, z] rad/s - Angular velocity
accelerometer = imu_state['accelerometer'] # [x, y, z] m/s² - Linear acceleration
rpy = imu_state['rpy']                     # [roll, pitch, yaw] radians - Euler angles
temperature = imu_state['temperature']     # °C - Sensor temperature
```

### Field Details

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `quaternion` | list[4] | unitless | Orientation as [w, x, y, z] quaternion |
| `gyroscope` | list[3] | rad/s | Angular velocity [x, y, z] |
| `accelerometer` | list[3] | m/s² | Linear acceleration [x, y, z] |
| `rpy` | list[3] | radians | Roll, pitch, yaw Euler angles |
| `temperature` | float | °C | IMU sensor temperature |

## Code Example

### Basic IMU Subscription

```python
import asyncio
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC

async def main():
    # Connect to the robot
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.1.105")
    await conn.connect()

    def imu_callback(message):
        imu = message['data']['imu_state']
        print(f"Quaternion: {imu['quaternion']}")
        print(f"Gyroscope: {imu['gyroscope']} rad/s")
        print(f"Accelerometer: {imu['accelerometer']} m/s²")
        print(f"RPY: {imu['rpy']} rad")
        print(f"Temperature: {imu['temperature']} °C")
        print("---")

    # Subscribe to sport mode state (includes IMU)
    conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], imu_callback)

    # Keep running to receive callbacks
    await asyncio.sleep(3600)

if __name__ == "__main__":
    asyncio.run(main())
```

### Extracting Yaw for Navigation

```python
import math

def imu_callback(message):
    imu = message['data']['imu_state']

    # Get yaw angle in radians
    yaw_rad = imu['rpy'][2]

    # Convert to degrees if needed
    yaw_deg = math.degrees(yaw_rad)

    print(f"Current heading: {yaw_deg:.1f}°")
```

## Additional Data from Sport Mode State

Beyond IMU, the sport mode state message provides additional useful data:

```python
def state_callback(message):
    data = message['data']

    # IMU data
    imu = data['imu_state']

    # Position and velocity
    position = data['position']           # Robot position estimate [x, y, z]
    velocity = data['velocity']           # Robot velocity [vx, vy, vz]
    yaw_speed = data['yaw_speed']         # Yaw angular rate

    # Body state
    body_height = data['body_height']     # Current body height

    # Foot information
    foot_force = data['foot_force']       # Foot force sensors
    foot_position_body = data['foot_position_body']  # Foot positions in body frame
    foot_speed_body = data['foot_speed_body']        # Foot velocities in body frame

    # Mode information
    gait_type = data['gait_type']         # Current gait type
    mode = data['mode']                   # Current motion mode
```

## Integration with Go2Robot Class

To add IMU data to the existing `Go2Robot` class in `nav_utils.py`:

```python
from unitree_webrtc_connect.constants import RTC_TOPIC

class Go2Robot:
    def __init__(self, ip="192.168.12.1"):
        # ... existing init code ...
        self.latest_imu = None

    async def connect(self):
        # ... existing connect code ...

        # Subscribe to IMU data after connection
        self.conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC['LF_SPORT_MOD_STATE'],
            self._on_sport_state
        )

    def _on_sport_state(self, message):
        """Callback for sport mode state updates."""
        self.latest_imu = message['data']['imu_state']

    def get_yaw(self):
        """Get current yaw angle in radians."""
        if self.latest_imu is None:
            return None
        return self.latest_imu['rpy'][2]

    def get_orientation_quaternion(self):
        """Get current orientation as quaternion [w, x, y, z]."""
        if self.latest_imu is None:
            return None
        return self.latest_imu['quaternion']
```

## Important Notes

1. **Callback is synchronous:** The `subscribe()` method and its callback are NOT async. The callback executes in the data channel's thread.

2. **Thread safety:** If accessing `latest_imu` from multiple threads, consider using a lock or `threading.Lock()`.

3. **Topic constants:** Always use `RTC_TOPIC['LF_SPORT_MOD_STATE']` rather than hardcoding the topic string to ensure compatibility with library updates.

4. **Connection required:** You must call `await conn.connect()` before subscribing to topics.

## References

- [unitree_webrtc_connect GitHub](https://github.com/legion1581/unitree_webrtc_connect)
- [Sport Mode State Example](https://github.com/legion1581/unitree_webrtc_connect/blob/main/examples/go2/data_channel/sportmodestate/sportmodestate.py)
- [Library Constants](https://github.com/legion1581/unitree_webrtc_connect/blob/main/unitree_webrtc_connect/constants.py)
