"""Mission configuration loader.

A mission is a directory containing at minimum:
    mission.toml       — gps / ntrip / robot / navigation settings
    waypoints.geojson  — FeatureCollection of Point features (lon, lat)

Precedence (lowest to highest):
    1. Dataclass defaults
    2. mission.toml fields
    3. Environment variables (GPS_PORT, EMLID_USERNAME, ...)
"""

import os
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
    host: str = "caster.emlid.com"
    port: int = 2101
    mountpoint: str = "MP15774"
    username: str = ""
    password: str = ""


@dataclass
class RobotSettings:
    connection_mode: str = "LocalSTA"
    ip: str | None = None
    serial: str | None = None


@dataclass
class NavigationSettings:
    arrival_tolerance: float = 0.5
    max_velocity: float = 0.5
    rotation_rate: float = 0.8
    min_fix_type: int = 4
    max_hacc: float = 0.10
    gps_fix_timeout: int = 300
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
    turn_tolerance_deg: float = 5.0     # how close to target bearing before capture
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
    if "EMLID_MOUNTPOINT" in env:
        cfg.ntrip.mountpoint = env["EMLID_MOUNTPOINT"]
    if "EMLID_USERNAME" in env:
        cfg.ntrip.username = env["EMLID_USERNAME"]
    if "EMLID_PASSWORD" in env:
        cfg.ntrip.password = env["EMLID_PASSWORD"]
    if "CONNECTION_MODE" in env:
        cfg.robot.connection_mode = env["CONNECTION_MODE"]
    if "ROBOT_IP" in env:
        cfg.robot.ip = env["ROBOT_IP"]
    if "ROBOT_SERIAL" in env:
        cfg.robot.serial = env["ROBOT_SERIAL"]


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
    _apply_section(cfg.ntrip, data.get("ntrip", {}))
    _apply_section(cfg.robot, data.get("robot", {}))
    _apply_section(cfg.navigation, data.get("navigation", {}))
    _apply_section(cfg.probe, data.get("probe", {}))
    _apply_section(cfg.capture, data.get("capture", {}))
    _apply_env_overrides(cfg)
    return cfg
