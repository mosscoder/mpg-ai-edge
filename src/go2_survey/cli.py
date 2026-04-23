"""go2-survey CLI.

Usage:
    go2-survey list                   # list available missions
    go2-survey run 00_parking_lot     # run a mission by name under dev/missions/
    go2-survey run path/to/mission    # ... or by path
    go2-survey run 00_parking_lot --dry-run
    go2-survey run 00_parking_lot -v  # debug logging
"""

import argparse
import asyncio
import logging
import sys
import time
from pathlib import Path
from typing import Iterator

from go2_survey.logging_utils import (
    SportModeStateFilter,
    WebRTCFallbackNoiseFilter,
)
from go2_survey.mission_runner import MissionRunner, run_mission


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
    """Configure root logging to console + <mission_dir>/logs/<name>_<ts>.log."""
    log_dir = mission_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_file = log_dir / f"{mission_dir.name}_{timestamp}.log"

    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(log_file),
        ],
    )
    # Quiet noisy transitive deps
    for noisy in ("aioice", "aiortc", "av"):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    # Drop the unitree_webrtc_connect 'old method' fallback errors —
    # they fire on every run but aren't real failures. Under default
    # (non-verbose) runs also drop the 20 Hz rt/lf/sportmodestate
    # INFO flood; -v/--verbose keeps it for diagnostic work. Attach
    # to handlers so records from the root logger (where the library
    # emits them) are filtered out regardless.
    filters: list[logging.Filter] = [WebRTCFallbackNoiseFilter()]
    if not verbose:
        filters.append(SportModeStateFilter())
    for handler in logging.getLogger().handlers:
        for f in filters:
            handler.addFilter(f)

    return log_file


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

    log_file = setup_logging(mission_dir, verbose=args.verbose)
    logger = logging.getLogger(__name__)
    logger.info(f"Logging to: {log_file}")
    logger.info(f"Mission dir: {mission_dir}")

    if args.capture_images:
        logger.warning(
            "--capture-images is a placeholder; hook not yet wired up"
        )

    runner = MissionRunner(
        mission_dir=mission_dir,
        dry_run=args.dry_run,
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
        "-v", "--verbose", action="store_true", help="enable debug logging"
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

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
