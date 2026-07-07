"""Mission configuration loader.

A mission is a directory containing at minimum:
    mission.toml       — gps / ntrip / robot / navigation settings
    waypoints.geojson  — FeatureCollection of Point features (lon, lat)

Precedence (lowest to highest):
    1. Dataclass defaults
    2. mission.toml fields
    3. Environment variables (GPS_PORT, EMLID_USERNAMES, ...)
"""

import os
import sys
from dataclasses import dataclass, field, fields
from pathlib import Path

try:
    import tomllib  # Python 3.11+ stdlib
except ModuleNotFoundError:  # Python <3.11 backport
    import tomli as tomllib


@dataclass
class GPSSettings:
    port: str = "/dev/ttyACM0"
    baud: int = 38400


@dataclass
class NTRIPSettings:
    """NTRIP caster settings with parallel-list endpoint fallback.

    `mountpoints[i]`, `usernames[i]`, `passwords[i]` together describe
    endpoint `i`. Index 0 is the primary; subsequent indices are tried
    in order on failure. All three lists must be the same length.

    `on_unavailable` controls behavior when every endpoint exhausts its
    retry budget (3 attempts each):
      - "warn_continue": loud warning, mission proceeds in GNSS-only
        mode with a tightened quality gate (Float treated as suspect).
      - "abort":         mission aborts before the robot connects.
        Use for production survey runs that require RTK precision.
    """

    host: str = "caster.emlid.com"
    port: int = 2101
    mountpoints: list[str] = field(default_factory=list)
    usernames: list[str] = field(default_factory=list)
    passwords: list[str] = field(default_factory=list)
    on_unavailable: str = "warn_continue"


@dataclass
class RobotSettings:
    connection_mode: str = "LocalSTA"
    ip: str | None = None
    serial: str | None = None


@dataclass
class NavigationSettings:
    arrival_tolerance: float = 0.25
    max_velocity: float = 0.5
    rotation_rate: float = 0.8
    min_fix_type: int = 4
    max_hacc: float = 0.10
    gps_fix_timeout: int = 300
    # Mid-mission tolerance for a degraded fix before aborting the leg.
    # Shorter than gps_fix_timeout because a stationary robot in the
    # field is a worse failure mode than a longer initial wait.
    mid_mission_fix_timeout: int = 60
    # Maximum age of incoming RTCM frames (seconds) before we treat the
    # receiver's RTK fix as "coasting" — the F9P/F9R holds Float for
    # ~30-60s after corrections stop, but those readings are increasingly
    # stale. Below this threshold, RTK readings are trusted.
    max_rtcm_age_s: float = 5.0
    # Soak time after the initial fix and before the robot starts
    # moving. Lets the F9P's carrier-phase ambiguity resolution
    # complete (Float -> Fixed typically takes 30-60s) and captures a
    # clean quality baseline in gps.log. Set to 0 to disable the dwell.
    stabilization_period_s: int = 60
    # Early-exit threshold: if the receiver holds RTK Fixed (type 6)
    # WITH active NTRIP corrections for this many consecutive seconds,
    # the dwell exits early. Cuts wallclock when RTK converges fast.
    # Set to 0 to disable early exit (force full duration).
    stabilization_early_exit_s: float = 3.0
    # Refresh the IMU→true-north offset on each waypoint arrival using
    # GPS+IMU samples already collected during the walk leg. Disable to
    # preserve the legacy single-shot calibration set on the first leg.
    imu_recalibrate_on_arrival: bool = True
    # Thigh ("shoulder") motor temperature thresholds (°C) for the health
    # banners. Caution = start watching the climb; danger = collapse risk,
    # cool down. Anchored on the 2026-06-16 strip_2 cut-out at 83 °C
    # (Unitree's soft ceiling ~80 °C), with margin below it.
    motor_caution_temp_c: int = 70
    motor_danger_temp_c: int = 78


@dataclass
class CoolingSettings:
    """Thermal interlock settings for motor-temperature cooldowns."""

    enabled: bool = True
    trigger_temp_c: int = 70
    resume_temp_c: int = 55
    poll_interval_s: float = 5.0
    log_interval_s: float = 30.0
    telemetry_max_age_s: float = 15.0
    lock_wait_s: float = 1.0
    crouch_wait_s: float = 3.0
    stand_wait_s: float = 2.0


@dataclass
class ProbeSettings:
    """Settings for mode = "probe_gps" diagnostic missions."""

    duration_sec: float = 120.0
    sample_rate_hz: float = 5.0
    wait_for_fix: bool = True


@dataclass
class CaptureSettings:
    """Per-waypoint capture behavior. See `capture.py::STRATEGIES`."""

    strategy: str = "none"              # "none" | "frame_only" | "waypoint_forward" | "line_survey"
    settle_time: float = 2.0            # seconds after stop before sampling
    gps_avg_sec: float = 3.0            # GPS averaging window (0 disables)
    output_subdir: str = "captures"     # relative to the run dir
    frame_max_age: float = 0.5          # max staleness / selection window half-width (s)
    frame_wait_timeout: float = 5.0     # max wait for a fresh frame (s)
    prefer_clean_frame: bool = True     # prefer a non-corrupt frame within the max_age window
    # Corner-turn alignment tolerance: how close to the target bearing the
    # in-place turn must land before a line-survey leg drive begins (also the
    # rotate-in-place tolerance for waypoint_forward approaches). The
    # turn_to_bearing P-controller reaches 2° without hunting.
    turn_tolerance_deg: float = 2.0
    # line_survey strategy: drive straight legs between corner waypoints at
    # navigation.max_velocity, turning in place at each corner, and capture a
    # clean frame every capture_interval_m of along-track travel (geotagged
    # per-image, so capture points are not waypoints). Speed comes from
    # [navigation] max_velocity.
    capture_interval_m: float = 2.0
    # line_survey leg steering law:
    #   "point_seek"  — re-aim at the end corner each tick on the calibrated
    #     IMU heading. Tolerant of a small residual offset but bows off-line
    #     (and never recovers at speed) when the offset has drifted. Default.
    #   "cross_track" — pure-pursuit a near carrot on the leg line, steering on
    #     GPS-derived course (offset-free) instead of the IMU heading. Immune
    #     to IMU-offset drift; tracks the line, not just the corner. lookahead_m
    #     is the carrot distance ahead on the line; course_lookback_m is the
    #     GPS-course baseline (≈1 m gives <1° noise at survey speed).
    leg_steering: str = "point_seek"   # "point_seek" | "cross_track"
    lookahead_m: float = 4.0           # cross_track carrot distance (m)
    course_lookback_m: float = 1.0     # cross_track GPS-course baseline (m)


@dataclass
class MissionSettings:
    name: str = ""
    description: str = ""
    mode: str = "nav"  # "nav" | "cooling_test" | "static_camera" | "static_geotag" | "probe_gps" | "probe_lidar"
    # Root directory for run outputs (logs, captures, sidecars, manifest).
    # Empty (default) keeps runs at <mission_dir>/runs/ — tracked in the
    # repo, the historical layout. Set a path to keep field data out of
    # the repo: runs then land at <output_dir>/<mission_name>/<run_name>/.
    # `~` and $ENV_VARS are expanded; a relative path resolves against the
    # mission dir. Env override: GO2_SURVEY_OUTPUT_DIR.
    output_dir: str = ""
    gps: GPSSettings = field(default_factory=GPSSettings)
    ntrip: NTRIPSettings = field(default_factory=NTRIPSettings)
    robot: RobotSettings = field(default_factory=RobotSettings)
    navigation: NavigationSettings = field(default_factory=NavigationSettings)
    cooling: CoolingSettings = field(default_factory=CoolingSettings)
    probe: ProbeSettings = field(default_factory=ProbeSettings)
    capture: CaptureSettings = field(default_factory=CaptureSettings)


def _apply_section(target: object, section: dict) -> None:
    known = {f.name for f in fields(target)}
    for key, value in section.items():
        if key in known:
            setattr(target, key, value)


def _apply_env_overrides(cfg: MissionSettings) -> None:
    env = os.environ
    if "GO2_SURVEY_OUTPUT_DIR" in env:
        cfg.output_dir = env["GO2_SURVEY_OUTPUT_DIR"]
    if "GPS_PORT" in env:
        cfg.gps.port = env["GPS_PORT"]
    if "GPS_BAUD" in env:
        cfg.gps.baud = int(env["GPS_BAUD"])
    if "EMLID_NTRIP_HOST" in env:
        cfg.ntrip.host = env["EMLID_NTRIP_HOST"]
    if "EMLID_NTRIP_PORT" in env:
        cfg.ntrip.port = int(env["EMLID_NTRIP_PORT"])
    if "EMLID_MOUNTPOINTS" in env:
        cfg.ntrip.mountpoints = _split_csv(env["EMLID_MOUNTPOINTS"])
    if "EMLID_USERNAMES" in env:
        cfg.ntrip.usernames = _split_csv(env["EMLID_USERNAMES"])
    if "EMLID_PASSWORDS" in env:
        cfg.ntrip.passwords = _split_csv(env["EMLID_PASSWORDS"])
    if "CONNECTION_MODE" in env:
        cfg.robot.connection_mode = env["CONNECTION_MODE"]
    if "ROBOT_IP" in env:
        cfg.robot.ip = env["ROBOT_IP"]
    if "ROBOT_SERIAL" in env:
        cfg.robot.serial = env["ROBOT_SERIAL"]


def _split_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


_LEGACY_NTRIP_KEYS = {
    "mountpoint": "mountpoints (list)",
    "username": "usernames (list)",
    "password": "passwords (list)",
}


def _check_ntrip_legacy(ntrip_section: dict, mission_file: Path) -> None:
    """Fail fast if a TOML still uses the pre-fallback scalar keys."""
    found = [k for k in _LEGACY_NTRIP_KEYS if k in ntrip_section]
    if not found:
        return
    rename_hints = "\n".join(
        f"  - rename `{k}` to `{_LEGACY_NTRIP_KEYS[k]}`" for k in found
    )
    raise ValueError(
        f"NTRIP config in {mission_file} uses legacy scalar keys "
        f"({', '.join(found)}); the schema now expects parallel lists.\n"
        f"{rename_hints}"
    )


def _coerce_endpoint_list(value, field_name: str, mission_file: Path) -> list[str]:
    """Accept either a list of strings or a single string (treated as length-1).

    A bare string in TOML (`mountpoints = "MP22385"`) is the natural way
    to write a single-endpoint config, so we silently wrap it. Anything
    else (number, bool, list-of-non-strings, ...) raises with a clear
    message instead of being mishandled downstream.
    """
    if isinstance(value, str):
        return [value]
    if isinstance(value, list) and all(isinstance(x, str) for x in value):
        return value
    raise TypeError(
        f"NTRIP config in {mission_file}: `{field_name}` must be a string "
        f"or a list of strings, got {type(value).__name__} ({value!r})."
    )


_VALID_ON_UNAVAILABLE = ("warn_continue", "abort")


def _validate_ntrip(
    ntrip: NTRIPSettings, mission_file: Path
) -> None:
    """Coerce scalar strings to length-1 lists, then check lengths align."""
    ntrip.mountpoints = _coerce_endpoint_list(
        ntrip.mountpoints, "mountpoints", mission_file
    )
    ntrip.usernames = _coerce_endpoint_list(
        ntrip.usernames, "usernames", mission_file
    )
    ntrip.passwords = _coerce_endpoint_list(
        ntrip.passwords, "passwords", mission_file
    )

    n_mp, n_u, n_p = len(ntrip.mountpoints), len(ntrip.usernames), len(ntrip.passwords)
    if n_mp != n_u or n_mp != n_p:
        msg = (
            f"NTRIP config invalid in {mission_file}:\n"
            f"  mountpoints has {n_mp} entries, usernames has {n_u}, passwords has {n_p}.\n"
            f"  Each list must have the same length (one entry per endpoint)."
        )
        print(msg, file=sys.stderr)
        raise ValueError(msg)

    if ntrip.on_unavailable not in _VALID_ON_UNAVAILABLE:
        msg = (
            f"NTRIP config invalid in {mission_file}:\n"
            f"  on_unavailable = {ntrip.on_unavailable!r} is not one of "
            f"{list(_VALID_ON_UNAVAILABLE)}."
        )
        print(msg, file=sys.stderr)
        raise ValueError(msg)


def load_mission_config(mission_dir: Path) -> MissionSettings:
    toml_path = mission_dir / "mission.toml"
    if not toml_path.exists():
        raise FileNotFoundError(f"mission.toml not found in {mission_dir}")
    data = tomllib.loads(toml_path.read_text())

    cfg = MissionSettings()
    cfg.name = data.get("name", mission_dir.name)
    cfg.description = data.get("description", "")
    cfg.mode = data.get("mode", "nav")
    cfg.output_dir = data.get("output_dir", "")
    _apply_section(cfg.gps, data.get("gps", {}))
    ntrip_section = data.get("ntrip", {})
    _check_ntrip_legacy(ntrip_section, toml_path)
    _apply_section(cfg.ntrip, ntrip_section)
    _apply_section(cfg.robot, data.get("robot", {}))
    _apply_section(cfg.navigation, data.get("navigation", {}))
    _apply_section(cfg.cooling, data.get("cooling", {}))
    _apply_section(cfg.probe, data.get("probe", {}))
    _apply_section(cfg.capture, data.get("capture", {}))
    _apply_env_overrides(cfg)
    _validate_ntrip(cfg.ntrip, toml_path)
    return cfg
