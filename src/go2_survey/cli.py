"""go2-survey CLI.

Usage:
    go2-survey list                   # list available missions
    go2-survey run 00_parking_lot     # run a mission by name under dev/missions/
    go2-survey run path/to/mission    # ... or by path
    go2-survey run 00_parking_lot --dry-run
    go2-survey run 00_parking_lot -v  # debug logging
    go2-survey make-waypoints PATH --side-len-m 30 --leg-space-m 5 --epsg 6514
                                      # generate lawnmower leg-endpoint
                                      # waypoints from a Point or Polygon
"""

import argparse
import asyncio
import logging
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterator

from go2_survey import __version__
from go2_survey.logging_utils import (
    GPSTelemetryFilter,
    GPSTelemetryOnlyFilter,
    SportModeStateFilter,
    SportModeStateOnlyFilter,
    WebRTCFallbackNoiseFilter,
    WebRTCTeardownNoiseFilter,
)
from go2_survey.mission_runner import MissionRunner, run_mission
from go2_survey.waypoint_gen import cmd_make_waypoints, cmd_plot_waypoints


def _git_short_sha() -> str:
    """Return the current git SHA (short form), or 'unknown' on failure."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=_find_repo_root(),
            capture_output=True,
            text=True,
            timeout=2,
        )
        if result.returncode == 0:
            return result.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        pass
    return "unknown"


def _find_repo_root(start: Path | None = None) -> Path:
    """Walk up from `start` (or cwd) looking for a pyproject.toml."""
    current = (start or Path.cwd()).resolve()
    for parent in [current, *current.parents]:
        if (parent / "pyproject.toml").exists():
            return parent
    return current


def _missions_root() -> Path:
    return _find_repo_root() / "dev" / "missions"


def resolve_mission_dir(arg: str) -> Path | None:
    """Resolve a mission argument to a directory.

    Accepts either an existing directory path containing mission.toml,
    or a mission name resolved under <repo-root>/dev/missions/<name>.
    """
    candidate = Path(arg)
    if candidate.is_dir() and (candidate / "mission.toml").exists():
        return candidate.resolve()

    named = _missions_root() / arg
    if named.is_dir() and (named / "mission.toml").exists():
        return named.resolve()

    return None


def setup_logging(mission_dir: Path, verbose: bool = False) -> Path:
    """Configure logging to a per-run directory under <mission_dir>/runs/.

    Layout: ``<mission_dir>/runs/<name>_<TIMESTAMP>/{main.log, imu.log, gps.log}``.
    Captures land under the same run directory (``captures/<wp>/...``)
    so every artifact from one mission run lives in one place.

    - ``main.log`` and the console carry the mission narrative
      (everything EXCEPT the 20 Hz IMU flood and the dense per-second
      GPS telemetry).
    - ``imu.log`` captures the rt/lf/sportmodestate stream by itself.
    - ``gps.log`` captures dense JSON-per-line GPS/RTK telemetry at 1 Hz
      so post-hoc analysis can reconstruct fix transitions, hAcc, NTRIP
      state, and RTCM-age timelines without requiring a verbose re-run.

    All three sidecars are always on regardless of ``-v``.

    Returns the run directory.
    """
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    run_dir = mission_dir / "runs" / f"{mission_dir.name}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    main_log = run_dir / "main.log"
    imu_log = run_dir / "imu.log"
    gps_log = run_dir / "gps.log"

    level = logging.DEBUG if verbose else logging.INFO
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    root = logging.getLogger()
    root.setLevel(level)
    # Clear any handlers a prior call (e.g. test harness) installed.
    for h in list(root.handlers):
        root.removeHandler(h)

    # Console + main.log: drop the 20 Hz IMU flood, the dense GPS
    # telemetry, and the unitree_webrtc_connect legacy-SDP-probe errors
    # so the mission narrative stays readable.
    main_filters: list[logging.Filter] = [
        SportModeStateFilter(),
        GPSTelemetryFilter(),
        WebRTCFallbackNoiseFilter(),
        WebRTCTeardownNoiseFilter(),
    ]
    for handler in (
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(main_log),
    ):
        handler.setFormatter(fmt)
        for f in main_filters:
            handler.addFilter(f)
        root.addHandler(handler)

    # imu.log: capture ONLY the rt/lf/sportmodestate stream. Always on
    # at DEBUG so an INFO root level still records the IMU history.
    imu_handler = logging.FileHandler(imu_log)
    imu_handler.setFormatter(fmt)
    imu_handler.addFilter(SportModeStateOnlyFilter())
    imu_handler.setLevel(logging.DEBUG)
    root.addHandler(imu_handler)

    # gps.log: capture ONLY the dense GPS/RTK telemetry stream emitted
    # at 1 Hz from GPSManager.get_position. Always on at DEBUG so the
    # JSON-per-line feed is preserved for post-hoc analysis regardless
    # of root level.
    gps_handler = logging.FileHandler(gps_log)
    gps_handler.setFormatter(fmt)
    gps_handler.addFilter(GPSTelemetryOnlyFilter())
    gps_handler.setLevel(logging.DEBUG)
    root.addHandler(gps_handler)

    # Quiet noisy transitive deps
    for noisy in ("aioice", "aiortc", "av"):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    # Version banner — first line in every log so post-mortems can pin
    # the run to a specific build.
    logging.getLogger(__name__).info(
        f"go2-survey v{__version__} | git {_git_short_sha()} "
        f"| run dir: {run_dir}"
    )

    return run_dir


def cmd_run(args: argparse.Namespace) -> int:
    mission_dir = resolve_mission_dir(args.mission)
    if mission_dir is None:
        print(f"error: could not find mission '{args.mission}'", file=sys.stderr)
        print(f"       tried: {Path(args.mission).resolve()}", file=sys.stderr)
        print(
            f"       tried: {(_missions_root() / args.mission).resolve()}",
            file=sys.stderr,
        )
        return 1

    run_dir = setup_logging(mission_dir, verbose=args.verbose)
    logger = logging.getLogger(__name__)
    logger.info(f"Logging to: {run_dir}/main.log (+ imu.log, gps.log)")
    logger.info(f"Mission dir: {mission_dir}")

    if args.capture_images:
        logger.warning(
            "--capture-images is a placeholder; hook not yet wired up"
        )

    bearing_method = "cog_fusion" if args.cog_fusion else "running_cog"
    runner = MissionRunner(
        mission_dir=mission_dir,
        run_dir=run_dir,
        dry_run=args.dry_run,
        bearing_method=bearing_method,
    )

    try:
        success = asyncio.run(run_mission(runner))
        return 0 if success else 1
    except KeyboardInterrupt:
        logger.warning("Interrupted")
        return 1


def _walk_missions(root: Path) -> Iterator[Path]:
    """Yield every mission folder under `root`, recursively.

    A directory is a mission folder if it contains `mission.toml`; once
    found, we stop descending into it (missions are leaves, not
    containers for other missions). Any directory whose name starts
    with `_` is skipped entirely, including its subtree — so
    `_template/` stays hidden and a user can hide a whole experimental
    subtree by prefixing the parent directory with `_`.
    """
    for entry in sorted(root.iterdir()):
        if not entry.is_dir():
            continue
        if entry.name.startswith("_"):
            continue
        if (entry / "mission.toml").exists():
            yield entry
        else:
            yield from _walk_missions(entry)


def cmd_list(args: argparse.Namespace) -> int:
    root = _missions_root()
    if not root.exists():
        print(f"no missions directory found at {root}")
        return 0

    missions = sorted(_walk_missions(root))
    if not missions:
        print(f"no missions found in {root}")
        return 0

    for m in missions:
        print(m.relative_to(root))
    return 0


def cmd_discover_ip(args: argparse.Namespace) -> int:
    from go2_survey.discovery import find_robot_ips

    try:
        ips = find_robot_ips(cidr=args.cidr)
    except RuntimeError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    if not ips:
        print(
            "No host found with 8081 or 9991 open. "
            "Ensure the robot is on this network and in LocalSTA mode.",
            file=sys.stderr,
        )
        return 1

    if len(ips) > 1:
        print(
            f"Warning: multiple candidates found: {ips}; using first",
            file=sys.stderr,
        )

    print(ips[0])
    return 0


def cmd_finalize_bearings(args: argparse.Namespace) -> int:
    from go2_survey.bearings import finalize_bearings

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    path = Path(args.run_dir)
    captures = path / "captures" if (path / "captures").is_dir() else path
    if not captures.is_dir():
        print(f"error: no captures directory at {captures}", file=sys.stderr)
        return 1
    n = finalize_bearings(captures, args.window_m)
    print(f"finalized {n} capture bearings in {captures}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="go2-survey",
        description="Autonomous waypoint navigation for the Unitree Go2",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    run_parser = subparsers.add_parser("run", help="run a mission")
    run_parser.add_argument(
        "mission",
        help="mission directory path or name under dev/missions/",
    )
    run_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="load config + waypoints, log plan, don't touch hardware",
    )
    run_parser.add_argument(
        "--capture-images",
        action="store_true",
        help="[placeholder] capture a frame at each waypoint (not wired)",
    )
    run_parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="set root log level to DEBUG (the IMU stream goes to imu.log "
        "regardless)",
    )
    run_parser.add_argument(
        "--cog-fusion",
        action="store_true",
        help="opt in to inverse-variance-weighted GPS course-over-ground "
        "fusion for per-arrival IMU recalibration. Default off uses the "
        "endpoint A→B bearing. Experimental — see changelog.",
    )
    run_parser.set_defaults(func=cmd_run)

    list_parser = subparsers.add_parser("list", help="list available missions")
    list_parser.set_defaults(func=cmd_list)

    discover_parser = subparsers.add_parser(
        "discover-ip",
        help="scan local network for a Go2 via WebRTC ports 8081/9991",
    )
    discover_parser.add_argument(
        "--cidr",
        help="network CIDR to scan (default: auto-detect via `ip route`)",
    )
    discover_parser.set_defaults(func=cmd_discover_ip)

    mkwp_parser = subparsers.add_parser(
        "make-waypoints",
        help="generate lawnmower leg-endpoint waypoints from a Point or "
        "Polygon (GeoJSON or KML). Writes waypoints.geojson next to the input.",
    )
    mkwp_parser.add_argument(
        "input",
        help="path to a GeoJSON or KML file containing a single Point or "
        "Polygon (or multiple features — centroid is used)",
    )
    mkwp_parser.add_argument(
        "--side-len-m",
        type=float,
        default=None,
        help="side length of the square coverage area in meters (REQUIRED "
        "for point input; IGNORED for polygon input)",
    )
    mkwp_parser.add_argument(
        "--leg-space-m",
        type=float,
        required=True,
        help="spacing between parallel legs (swath width) in meters",
    )
    mkwp_parser.add_argument(
        "--epsg",
        type=int,
        required=True,
        help="EPSG code of a projected CRS (units must be METERS) used for "
        "the grid math, e.g. 6514 for NAD83(2011) / Montana. Output is "
        "always EPSG:4326 (lon/lat).",
    )
    mkwp_parser.add_argument(
        "--bearing-deg",
        type=float,
        default=0.0,
        help="rotate the legs clockwise from the default E–W orientation. "
        "e.g. --bearing-deg=30 → legs tilt 30° clockwise.",
    )
    mkwp_parser.add_argument(
        "--no-plot",
        action="store_true",
        help="skip generating mission_layout.png alongside waypoints.geojson "
        "(default: plot is generated automatically)",
    )
    mkwp_parser.set_defaults(func=cmd_make_waypoints)

    plotwp_parser = subparsers.add_parser(
        "plot-waypoints",
        help="render mission_layout.png from a waypoints.geojson — "
        "bounding box, numbered waypoints, serpentine arrows.",
    )
    plotwp_parser.add_argument(
        "input",
        help="path to a waypoints.geojson",
    )
    plotwp_parser.add_argument(
        "--output",
        default=None,
        help="output PNG path (default: mission_layout.png next to input)",
    )
    plotwp_parser.set_defaults(func=cmd_plot_waypoints)

    fb_parser = subparsers.add_parser(
        "finalize-bearings",
        help="(re)compute post-hoc centered RTK-course bearings for a "
        "line-survey run and write them to each capture's sidecar + JPEG EXIF.",
    )
    fb_parser.add_argument(
        "run_dir",
        help="a run directory (.../runs/<run>) or a captures directory",
    )
    fb_parser.add_argument(
        "--window-m",
        type=float,
        default=1.0,
        help="look-back/forward chord half-length in meters (default 1.0)",
    )
    fb_parser.set_defaults(func=cmd_finalize_bearings)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
