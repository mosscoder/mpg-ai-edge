# RTK Waypoint Navigation Missions

This guide describes setup and operation of the Go2 robot waypoint navigation system using RTK GPS.

![Mission Waypoints](init_rtk_missions.png)

## Hardware Requirements

- **Unitree Go2 Robot Dog**
- **NVIDIA Jetson** (mounted on robot) with WiFi
- **SparkFun ZED-F9P RTK GPS** connected to Jetson via USB (default: `/dev/ttyACM0`)
- **NTRIP corrections** via Emlid caster (requires account)
- **Laptop** for SSH access

## Network Architecture (LocalSTA)

```
[Laptop] ---SSH---> [Jetson] ---LocalSTA---> [Go2 Robot]
    |                   |                        |
    +--------(local WiFi network)----------------+
                        |
                   [NTRIP caster]
```

All devices connect to the same local WiFi network. The Jetson runs the mission scripts and communicates with the Go2 over the local network.

## Software Dependencies

On the Jetson:
```bash
pip install unitree_webrtc_connect pyserial pynmea2
```

## Go2 WiFi Configuration

Configure the Go2 to join your local WiFi network:

1. Open the **Unitree Go** app on your phone
2. Connect to the robot
3. Go to **Settings > Wi-Fi connection mode**
4. Select your local WiFi network and enter the password
5. The robot will reboot and join the network

## Finding the Jetson's IP Address

The Jetson reports its local IP to a web service every 5 minutes, making it easy to find even if it changes networks:

```bash
# Query the Jetson's last known IP
curl -s "https://mpg.commsat.org/api/lastIP?password=robotanist"
```

This returns the Jetson's current IP address for SSH access.

## Finding the Robot's IP Address

After the Go2 joins your network, find its IP:

```bash
# Option 1: nmap scan (replace with your subnet)
nmap -sn 192.168.1.0/24 | grep -i -B2 "unitree\|go2"

# Option 2: arp scan
sudo arp-scan --localnet | grep -i unitree

# Option 3: Check router's DHCP leases
# Look for hostname containing the robot's serial number
```

## Environment Setup

Set these variables on the Jetson before running missions:

```bash
# Robot connection (required)
export ROBOT_IP="192.168.1.105"  # Go2's IP on your network

# NTRIP credentials (required for RTK)
export EMLID_USERNAME="your_username"
export EMLID_PASSWORD="your_password"

# Optional overrides
export EMLID_MOUNTPOINT="MP1979"
export GPS_PORT="/dev/ttyACM0"
export GPS_BAUD="38400"
```

## Running Missions

1. SSH into the Jetson from your laptop
2. Position robot at starting location (see image above)
3. Run mission:

```bash
cd /path/to/mpg-ai-edge
python autonomous_nav/mission/mission_00.py  # Single waypoint
python autonomous_nav/mission/mission_01.py  # Multi-waypoint
```

---

## Mission 00: Single Waypoint Navigation

**Script:** `autonomous_nav/mission/mission_00.py`

**Behavior:**
1. Connects to RTK GPS and waits for RTK Float fix (type 5+)
2. Connects to Go2 robot via LocalSTA (same WiFi network)
3. Navigates to **Point 0** only
4. Stops when within 0.2m of target

**Use case:** Testing basic navigation, verifying GPS accuracy.

---

## Mission 01: Multi-Waypoint Navigation

**Script:** `autonomous_nav/mission/mission_01.py`

**Behavior:**
1. Connects to RTK GPS and waits for RTK Float fix (type 5+)
2. Connects to Go2 robot via LocalSTA (same WiFi network)
3. Navigates to **Point 0**, then **Point 1** sequentially
4. Stops after reaching all waypoints

**Use case:** Full waypoint circuit, patrol patterns.

---

## Robot Behavior Details

### Navigation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Arrival tolerance | 0.2m | Distance threshold to consider waypoint reached |
| Max velocity | 0.3 m/s | Forward speed (conservative for precision) |
| Rotation rate | 0.3 rad/s | Turning speed when correcting heading |
| Min fix type | 5 | RTK Float or better required |
| Navigation timeout | 300s | Max time to reach a single waypoint |
| GPS pause timeout | 300s | Max wait for GPS fix restore during navigation |
| Calibration timeout | 30s | Max time for IMU calibration walk |

### Movement Control

The robot uses proportional navigation with IMU-based heading:

**Sensor Fusion:**
| Sensor | Purpose |
|--------|---------|
| RTK GPS | Absolute position, distance/bearing to waypoint |
| IMU | Current heading (calibrated to true north) |

**IMU Heading Calibration:**
- On startup, the IMU yaw is relative to power-on orientation (not north)
- During the first ~1.5m of forward motion, the system calibrates IMU yaw against GPS bearing
- After calibration, heading is available instantly at ~50-100 Hz
- Calibration offset is computed once and persists for the mission
- If calibration doesn't complete within 30s (robot blocked, insufficient displacement), it resets and retries automatically

**Navigation Behavior:**
- **Pre-calibration:** Walks forward toward waypoint to calibrate IMU (~5 seconds)
- **Large heading error (>30deg):** Rotates in place to face target
- **Small heading error:** Moves forward while correcting heading
- **Approaching target:** Slows down as distance decreases

### GPS Fix Loss Handling

If RTK fix degrades below minimum threshold:
1. Robot **stops immediately**
2. Waits for fix to be restored (periodic status logged every 15s)
3. If fix is not restored within **300s**, navigation aborts with a timeout error
4. **Resumes navigation** immediately once fix returns — IMU heading survives GPS outages

The IMU provides continuous heading even during GPS outages, enabling faster recovery compared to position-based heading estimation.

### Velocity Commands

| Command | Direction | Positive Value |
|---------|-----------|----------------|
| x | Forward/Back | Forward |
| y | Lateral | Left |
| z | Rotation | Counter-clockwise |

---

## Waypoint Coordinates

From `data/vector/tennis_court_points.geojson`:

| Point | Latitude | Longitude |
|-------|----------|-----------|
| Point 0 | 46.673892439183376 | -114.016136552705788 |
| Point 1 | 46.67381341902108 | -114.016197834855006 |

---

## Troubleshooting

**"Robot API not responding"**
- Close Unitree app on phone (single client limit)
- Verify ROBOT_IP environment variable is set correctly
- Confirm Go2 is on the same WiFi network as the Jetson

**"GPS fix timeout after 300s"**
- Message includes elapsed time (e.g. `GPS fix timeout after 300s`)
- Ensure clear sky view for GPS antenna
- Verify NTRIP credentials are correct
- Check GPS serial port connection

**"Navigation timeout after 300s for waypoint ..."**
- Waypoint was not reached within the timeout (default 300s)
- Check for physical obstacles blocking the robot's path
- Verify GPS accuracy is sufficient — if hAcc is high the robot may oscillate near the target
- Message includes remaining distance to waypoint

**"IMU calibration timeout after 30s"**
- Robot couldn't walk 1.5m within 30s (obstacle, terrain, leash)
- Calibration resets automatically and retries
- Ensure the robot has a clear path forward at startup

**Robot moves erratically or spins**
- IMU calibration completes after ~1.5m of forward movement (~5 seconds)
- Check logs for "IMU calibrated!" message indicating successful calibration
- If robot spins repeatedly after calibration, verify IMU data is being received (check for `IMU cal: yes` in logs)
- Ensure RTK fix is stable (type 5+) before expecting accurate navigation

---

## Logs

Mission logs are saved to `autonomous_nav/mission/logs/`:
```
autonomous_nav/mission/logs/mission_00_2026-01-29_10-30-45.log
autonomous_nav/mission/logs/mission_01_2026-01-29_11-00-00.log
```

Log entries include fix type and horizontal accuracy for debugging:
```
Pos: (46.67389244, -114.01613655) | Dist: 5.23m | Fix: 6 | hAcc: 0.014m | ...
```

Major state transitions are highlighted with bordered banners for easy scanning:
```
============================================================
========= GPS RTK Fixed ACHIEVED | hAcc: 0.014m ============
============================================================
```

Error and warning banners use `!` borders:
```
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!! GPS FIX LOST | type 2 | robot paused !!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
```

During long waits (GPS fix, GPS pause, IMU calibration), periodic progress messages are logged so the output doesn't go silent.
