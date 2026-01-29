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

### Movement Control

The robot uses proportional navigation with position-based heading estimation:

**Heading Estimation:**
- Heading is computed from position changes, not GPS course-over-ground (COG)
- Only updates heading when moving forward (vx > 0) and after moving >1.5m
- The 1.5m threshold ensures accuracy even with RTK Float GPS noise
- If heading is unknown, robot walks forward ~5 seconds to establish it

**Navigation Behavior:**
- **No heading:** Walks forward slowly to establish heading from position delta
- **Large heading error (>30deg):** Rotates in place to face target
- **Small heading error:** Moves forward while correcting heading
- **Approaching target:** Slows down as distance decreases

### GPS Fix Loss Handling

If RTK fix degrades below minimum threshold:
1. Robot **stops immediately**
2. Waits for fix to be restored
3. **Resets heading** to unknown when fix returns
4. **Resumes navigation** by walking forward to re-establish heading

This prevents drift or inaccurate movement during GPS outages and ensures heading is re-calibrated after recovery.

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

**"GPS fix timeout"**
- Ensure clear sky view for GPS antenna
- Verify NTRIP credentials are correct
- Check GPS serial port connection

**Robot moves erratically or spins**
- Heading establishes after ~1.5m of forward movement (~5 seconds)
- If robot spins repeatedly, check that position-based heading is updating (see logs)
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
