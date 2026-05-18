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
    # Refresh the IMU→true-north offset on each waypoint arrival using
    # GPS+IMU samples already collected during the walk leg. Disable to
    # preserve the legacy single-shot calibration set on the first leg.
    imu_recalibrate_on_arrival: bool = True


@dataclass
class ProbeSettings:
    """Settings for mode = "probe_gps" diagnostic missions."""

    duration_sec: float = 120.0
    sample_rate_hz: float = 5.0
    wait_for_fix: bool = True


@dataclass
class CaptureSettings:
    """Per-waypoint capture behavior. See `capture.py::STRATEGIES`."""

    strategy: str = "none"              # "none" | "frame_only" | "waypoint_forward" | "rotating_quadrat"
    settle_time: float = 2.0            # seconds after stop before sampling
    gps_avg_sec: float = 3.0            # GPS averaging window (0 disables)
    bearings: list = field(default_factory=lambda: [0.0, 90.0, 180.0, 270.0])
    output_subdir: str = "captures"     # relative to mission dir
    frame_max_age: float = 0.5          # max staleness for a captured frame (s)
    frame_wait_timeout: float = 5.0     # max wait for a fresh frame (s)
    # P-controller bearing alignment for rotating_quadrat. The controller
    # decelerates as it approaches the target so the stop command lands
    # before overshoot, enabling sub-2° alignment without hunting.
    # Tuning rule of thumb: turn_kp = self.rotation_rate / angle_at_full_rate.
    # Default 0.04 = full rotation_rate commanded at error ≥ 20°.
    # turn_min_rate_rad_s defeats Go2 motor stiction near zero error
    # (a pure P controller stalls sub-threshold and never converges).
    turn_tolerance_deg: float = 2.0     # was 5.0; the P-controller makes 2° reachable
    turn_kp: float = 0.04               # P-controller gain (rad/s per degree of error)
    turn_min_rate_rad_s: float = 0.15   # dead-band floor (rad/s); below = no motion
    turn_timeout_sec: float = 15.0      # per-bearing rotation timeout


@dataclass
class MissionSettings:
    name: str = ""
    description: str = ""
    mode: str = "nav"  # "nav" | "static_camera" | "static_geotag" | "probe_gps"
    gps: GPSSettings = field(default_factory=GPSSettings)
    ntrip: NTRIPSettings = field(default_factory=NTRIPSettings)
    robot: RobotSettings = field(default_factory=RobotSettings)
    navigation: NavigationSettings = field(default_factory=NavigationSettings)
    probe: ProbeSettings = field(default_factory=ProbeSettings)
    capture: CaptureSettings = field(default_factory=CaptureSettings)


def _apply_section(target: object, section: dict) -> None:
    known = {f.name for f in fields(target)}
    for key, value in section.items():
        if key in known:
            setattr(target, key, value)


def _apply_env_overrides(cfg: MissionSettings) -> None:
    env = os.environ
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
    _apply_section(cfg.gps, data.get("gps", {}))
    ntrip_section = data.get("ntrip", {})
    _check_ntrip_legacy(ntrip_section, toml_path)
    _apply_section(cfg.ntrip, ntrip_section)
    _apply_section(cfg.robot, data.get("robot", {}))
    _apply_section(cfg.navigation, data.get("navigation", {}))
    _apply_section(cfg.probe, data.get("probe", {}))
    _apply_section(cfg.capture, data.get("capture", {}))
    _apply_env_overrides(cfg)
    _validate_ntrip(cfg.ntrip, toml_path)
    return cfg
