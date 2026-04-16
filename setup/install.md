# Installing go2_survey

End-to-end setup for the `go2-survey` CLI and its first mission run.

## Prerequisites

- Python **3.10+** (on 3.10 the `tomli` backport is pulled in automatically; on 3.11+ the stdlib `tomllib` is used)
- A SparkFun ZED-F9P/F9R RTK GPS connected over USB (typically `/dev/ttyACM0`)
- A Unitree Go2 reachable on the local network (LocalSTA mode)
- NTRIP credentials for an Emlid caster (defaults in `dev/missions/_template/mission.toml` match the house base station)
- `nmap` on PATH if you want robot IP auto-discovery via `go2-survey discover-ip`

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

Each mission is a folder containing `mission.toml` + `waypoints.geojson`.
The `go2-survey` CLI resolves missions by name (searched under
`dev/missions/`) or by any path to a directory containing those two files.

**Dry run** — loads config and waypoints, prints the plan, and exits without
touching hardware:

```bash
go2-survey run 00_parking_lot --dry-run
```

**Live run:**

```bash
go2-survey run 00_parking_lot                      # by name under dev/missions/
go2-survey run dev/missions/00_parking_lot         # or by path
go2-survey run /absolute/path/to/my_survey_dir     # or by an arbitrary path
go2-survey run 00_parking_lot -v                   # with debug logging
```

Logs stream to the console and to
`<mission>/logs/<name>_<timestamp>.log` (where `<name>` comes from the
mission.toml `name` field), alongside the mission config.

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
`GPS_PORT=/dev/ttyUSB0 go2-survey run 00_parking_lot`.

## Creating a new mission

```bash
cp -r dev/missions/_template dev/missions/mission_NAME
```

Edit `mission.toml` (at minimum `name` and `description`) and replace
`waypoints.geojson` with your FeatureCollection of `Point` features. The CLI
will pick it up automatically the next time you run `go2-survey list`.

## Robot IP discovery

`go2-survey run <mission>` **auto-discovers** the Go2 when `[robot] ip` is
unset in `mission.toml` (and `ROBOT_IP` is unset in the environment). It
shells out to `nmap -n -sT -p 8081,9991 --open -Pn <CIDR>` against the local
subnet and uses the first host with either WebRTC port open. You don't
normally need to do anything manually — leave `robot.ip` commented out in
mission.toml and the runner handles it before connecting to the robot.

If you want to run the scan standalone — e.g. to diagnose a failure, or to
find the IP before editing a mission config:

```bash
go2-survey discover-ip                  # auto-detect CIDR from default route
go2-survey discover-ip --cidr 10.0.0.0/24   # or pass an explicit CIDR
```

Both paths go through the same `src/go2_survey/discovery.py` module.

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

## CLI reference

`go2-survey` exposes three subcommands. All are invoked through the installed
entry point — there are no per-mission shell wrappers.

### `go2-survey run <mission> [flags]`

Run a mission end-to-end: connect GPS, wait for RTK fix, auto-discover the
Go2's IP if `[robot] ip` is unset (Linux-only; via `nmap` scan for WebRTC
ports 8081/9991 in `LocalSTA` mode), connect to the robot, and iterate
waypoints through the `calibrate → turn → walk` state machine.

**Mission resolution.** `<mission>` can be:

- A **bare name** (e.g. `00_parking_lot`), resolved under
  `<repo-root>/dev/missions/<name>`. Repo root is auto-detected by walking
  up from `cwd` looking for `pyproject.toml`, so this form works from any
  directory inside the repo.
- An **absolute or cwd-relative path** to any directory containing
  `mission.toml` and `waypoints.geojson`. The folder can live anywhere on
  the filesystem and be named anything — `mission_` is just a convention,
  not a requirement.

**Flags:**

- `--dry-run` — load config and waypoints, log the plan, exit without
  touching hardware. Use this to sanity-check a config change before a
  live run.
- `-v` / `--verbose` — debug-level logging (default is INFO).
- `--capture-images` — placeholder. Parses cleanly and prints a warning;
  no-op until the `src/go2_survey/vision/frames.py` hook lands.

Logs always go to both the console and
`<mission>/logs/<name>_<timestamp>.log`, where `<name>` comes from the
`name` field of `mission.toml`.

### `go2-survey list`

Walks `<repo-root>/dev/missions/` **recursively** and lists every mission
folder it finds. A mission folder is any directory that contains a
`mission.toml`; once found, the walker does not descend into it (missions
are leaves, not containers). Any directory whose name starts with `_` is
skipped entirely, including its subtree — so `_template/` stays hidden
from `list`, and you can hide a whole experimental subtree by prefixing
the parent with `_`.

Output is the **relative path from `dev/missions/`**, so whatever `list`
prints can be pasted directly back into `go2-survey run`. A flat layout
prints `00_parking_lot`, `01_tennis_court`, etc.; a nested layout prints
`tennis_court/wp_set_a`, `parking_lot/circuit_02`, etc.

Exits 0 even when no missions exist. Missions at arbitrary filesystem
paths outside `dev/missions/` still do not appear in `list` — you invoke
those with their full path.

### `go2-survey discover-ip [--cidr CIDR]`

Scan the local network for a Unitree Go2 by probing TCP ports 8081 and
9991 (the Go2's WebRTC listener). Shells out to
`nmap -n -sT -p 8081,9991 --open -Pn <CIDR>` and parses its output.

- If `--cidr` is omitted, the CIDR is auto-detected from the default route
  via `ip route show default` + `ip -o -4 addr show dev <iface> scope global`.
  This path is Linux-only.
- Prints the first candidate IP to stdout, diagnostics to stderr.
- Exits 1 if no candidate is found.

## Jetson-specific notes

See `setup/jetson_orin_nano_setup.md`, `setup/jetson_orin_nano_status.md`, and
`setup/orin_nano_ml_setup.md` for hardware/OS setup on the Jetson Orin Nano.
