"""Mission configuration loader.

A mission is a directory containing at minimum:
    mission.toml       — gps / ntrip / robot / navigation settings
    waypoints.geojson  — FeatureCollection of Point features (lon, lat)

Precedence (lowest to highest):
    1. Dataclass defaults
    2. mission.toml fields
    3. Environment variables (GPS_PORT, EMLID_USERNAME, ...)
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field, fields
from pathlib import Path

try:
    import tomllib  # Python 3.11+ stdlib
except ModuleNotFoundError:  # Python 3.8-3.10 backport
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


@dataclass
class ProbeSettings:
    """Settings for mode = "probe_gps" diagnostic missions."""

    duration_sec: float = 120.0
    sample_rate_hz: float = 5.0
    wait_for_fix: bool = True


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
    _apply_env_overrides(cfg)
    return cfg
