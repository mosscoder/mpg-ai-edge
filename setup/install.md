# Installing go2_survey

End-to-end setup for the `go2-survey` CLI and its first mission run.

## Prerequisites

- Python **3.8+** (on 3.8–3.10 the `tomli` backport is pulled in automatically; on 3.11+ the stdlib `tomllib` is used)
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
go2-survey run mission_00 --dry-run
```

**Live run:**

```bash
go2-survey run mission_00                          # by name under dev/missions/
go2-survey run dev/missions/mission_00             # or by path
go2-survey run /absolute/path/to/my_survey_dir     # or by an arbitrary path
go2-survey run mission_00 -v                       # with debug logging
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
`GPS_PORT=/dev/ttyUSB0 go2-survey run mission_00`.

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
go2-survey discover-ip                  # auto-detect CIDR from default route
go2-survey discover-ip --cidr 10.0.0.0/24   # or pass an explicit CIDR
```

Then set `robot.ip` in the mission config to the first IP printed. The
implementation lives in `src/go2_survey/discovery.py` as a subprocess wrapper
around `nmap -n -sT -p 8081,9991 --open -Pn <CIDR>`.

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

Run a mission end-to-end: connect GPS, wait for RTK fix, connect to the
robot, iterate waypoints through the `calibrate → turn → walk` state
machine.

**Mission resolution.** `<mission>` can be:

- A **bare name** (e.g. `mission_00`), resolved under
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

Enumerate every mission directory under `<repo-root>/dev/missions/` that
contains a `mission.toml` and whose name does not start with `_` (so the
`_template` directory is hidden). Exits 0 even when no missions exist.
Missions at arbitrary filesystem paths outside `dev/missions/` do not
appear in `list` — you invoke those by full path.

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
