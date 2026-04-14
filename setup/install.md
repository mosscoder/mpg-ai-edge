# Installing go2_survey

End-to-end setup for the `go2-survey` CLI and its first mission run.

## Prerequisites

- Python **3.11+** (`tomllib` is a stdlib dependency)
- A SparkFun ZED-F9P/F9R RTK GPS connected over USB (typically `/dev/ttyACM0`)
- A Unitree Go2 reachable on the local network (LocalSTA mode)
- NTRIP credentials for an Emlid caster (defaults in `dev/missions/_template/mission.toml` match the house base station)
- `nmap` on PATH if you want robot IP auto-discovery via `scripts/find_robot_ip.sh`

## Install the package

```bash
git clone <this-repo> mpg-ai-edge
cd mpg-ai-edge
pip install -e .
```

The editable install pulls in `pyserial`, `pynmea2`, `numpy`, `Pillow`, and
`unitree_webrtc_connect` from GitHub. `aiortc`, `av`, and `opencv-python` come
along as transitive deps of `unitree_webrtc_connect` — useful later for video
frame capture.

Verify the CLI is on PATH:

```bash
go2-survey --help
go2-survey list          # shows the missions under dev/missions/
```

## Running a mission

Each mission is a folder containing `mission.toml` + `waypoints.geojson` +
`run.sh`. The `go2-survey` CLI resolves missions by name (searched under
`dev/missions/`) or by path.

**Dry run** — loads config and waypoints, prints the plan, and exits without
touching hardware:

```bash
go2-survey run mission_02 --dry-run
```

**Live run:**

```bash
go2-survey run mission_02          # by name
./dev/missions/mission_02/run.sh   # by co-located script
```

Pass `-v` for debug-level logging. Logs stream to the console and a timestamped
file under `dev/missions/<mission>/logs/`.

## Configuring a mission

Open the mission's `mission.toml` and edit any section:

```toml
[gps]
port = "/dev/ttyACM0"
baud = 38400

[ntrip]
host       = "caster.emlid.com"
port       = 2101
mountpoint = "MP15774"
username   = "u65352"
password   = "338zca"

[robot]
connection_mode = "LocalSTA"
# ip     = "192.168.1.105"
# serial = "..."

[navigation]
arrival_tolerance = 0.5
max_velocity      = 0.5
rotation_rate     = 0.8
min_fix_type      = 4
max_hacc          = 0.10
gps_fix_timeout   = 300
```

Any field can be overridden at runtime by setting an env var — e.g.
`GPS_PORT=/dev/ttyUSB0 go2-survey run mission_02`.

## Creating a new mission

```bash
cp -r dev/missions/_template dev/missions/mission_NAME
```

Edit `mission.toml` (at minimum `name` and `description`) and replace
`waypoints.geojson` with your FeatureCollection of `Point` features. The CLI
will pick it up automatically the next time you run `go2-survey list`.

## Robot IP discovery

If the Go2's IP is not hardcoded in `mission.toml` and `unitree_webrtc_connect`
can't find it, run:

```bash
./scripts/find_robot_ip.sh        # nmap scan for ports 8081/9991 on local subnet
```

Then set `robot.ip` in the mission config to the result.

## Troubleshooting

**GPS port permission denied** — add your user to the `dialout` group
(`sudo usermod -a -G dialout $USER` and log back in), or temporarily
`sudo chmod 666 /dev/ttyACM0`.

**NTRIP returns `HTTP/1.1 400 BAD REQUEST`** — the historical failure mode is
a bad password. Verify the `[ntrip]` block matches the caster's credentials
exactly.

**Robot connection fails** — confirm the Jetson and Go2 are on the same WiFi
SSID, ping the robot's IP, and check WebRTC ports 8081/9991 are open on the
robot.

**No RTK Fixed** — you'll see RTK Float first while the solution converges.
Wait 30–120 s under open sky. The fix is considered achieved at `min_fix_type`
or higher (4 = GNSS+DR, 5 = RTK Float, 6 = RTK Fixed).

## Jetson-specific notes

See `setup/jetson_orin_nano_setup.md`, `setup/jetson_orin_nano_status.md`, and
`setup/orin_nano_ml_setup.md` for hardware/OS setup on the Jetson Orin Nano.
