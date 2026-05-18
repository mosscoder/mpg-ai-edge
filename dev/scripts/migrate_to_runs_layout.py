#!/usr/bin/env python3
"""One-shot migration: <mission>/logs/ + <mission>/captures/ -> <mission>/runs/.

Before v0.10.0 a mission directory looked like::

    <mission>/
      logs/
        <mission>_<TS>/        # main.log + imu.log + gps.log
      captures/                # all runs' captures mixed together
        <wp>_<bearing>_<TS>.jpg
        <wp>_<bearing>_<TS>.json

From v0.10.0 onward captures live alongside logs in a per-run dir::

    <mission>/
      runs/
        <mission>_<TS>/
          main.log
          imu.log
          gps.log
          captures/
            <wp>/
              <bearing>_<TS>.jpg
              <bearing>_<TS>.json

This script migrates a tree of pre-v0.10.0 missions to the new layout:

1. Rename ``logs/`` -> ``runs/`` per mission (no-op if already renamed).
2. For each capture under the legacy ``captures/`` dir, parse the
   embedded timestamp and move the file (with its sidecar JSON) into the
   matching ``runs/<TS>/captures/<wp>/`` directory.
3. Orphans (captures with no matching run window) move to
   ``runs/_orphans/`` so nothing is silently dropped.
4. Once the legacy ``captures/`` dir is empty, remove it.

Usage::

    python dev/scripts/migrate_to_runs_layout.py [--dry-run] [PATH ...]

PATH may be either a missions root (e.g. ``dev/missions``) or one or
more individual mission directories. With no PATH, defaults to
``dev/missions`` under the repo root.

Idempotent: re-running after a partial migration finishes the job and
skips already-migrated artifacts.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

# Matches a capture filename written by capture.py prior to v0.10.0:
#   <safe_wp>_<bearing_tag>_<YYYY-MM-DDTHH-MM-SS>.jpg
# Bearing tag is either b<3 digits> or nobrg.
CAPTURE_RE = re.compile(
    r"^(?P<wp>.+?)_(?P<bearing>b\d{3}|nobrg)_"
    r"(?P<ts>\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2})\.jpg$"
)

# Matches a run-directory name: <mission_name>_<YYYY-MM-DD_HH-MM-SS>.
RUN_DIR_RE = re.compile(
    r"^(?P<name>.+)_(?P<ts>\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})$"
)

# Matches the OLDEST flat log layout: <mission>/logs/<mission>_<TS>.log
# (predates the per-run sidecar directory). These get promoted to
# <TS>/main.log during migration so every run is uniformly a directory.
LEGACY_LOG_RE = re.compile(
    r"^(?P<name>.+)_(?P<ts>\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})\.log$"
)

TS_FMT_CAPTURE = "%Y-%m-%dT%H-%M-%S"
TS_FMT_RUN = "%Y-%m-%d_%H-%M-%S"


def _is_mission_dir(path: Path) -> bool:
    return path.is_dir() and (path / "mission.toml").exists()


def _iter_mission_dirs(roots: list[Path]) -> list[Path]:
    """Expand each path into the set of mission dirs it represents.

    A path is a mission dir if it contains mission.toml; otherwise we
    walk it recursively looking for mission dirs (skipping subtrees
    whose top-level dir starts with ``_``).
    """
    out: list[Path] = []
    for root in roots:
        if not root.exists():
            print(f"warn: {root} does not exist; skipping", file=sys.stderr)
            continue
        if _is_mission_dir(root):
            out.append(root)
            continue
        for child in sorted(root.iterdir()):
            if not child.is_dir() or child.name.startswith("_"):
                continue
            if _is_mission_dir(child):
                out.append(child)
            else:
                # one level of recursion is enough for the current layout
                out.extend(_iter_mission_dirs([child]))
    # de-dupe while preserving order
    seen: set[Path] = set()
    deduped: list[Path] = []
    for m in out:
        if m not in seen:
            seen.add(m)
            deduped.append(m)
    return deduped


def _list_run_dirs(runs_root: Path) -> list[tuple[datetime, Path]]:
    """Return [(start_ts, path)] for each run, sorted ascending.

    Accepts both per-run directories AND loose legacy ``<run>.log``
    files (which haven't been promoted yet — happens during dry-run).
    For a legacy log file, the synthesized path is what the directory
    WILL be after promotion, so capture-routing decisions in dry-run
    match what'll happen in the real run.
    """
    if not runs_root.is_dir():
        return []
    out: list[tuple[datetime, Path]] = []
    for entry in runs_root.iterdir():
        if entry.is_dir():
            m = RUN_DIR_RE.match(entry.name)
            if not m:
                continue
            try:
                ts = datetime.strptime(m.group("ts"), TS_FMT_RUN)
            except ValueError:
                continue
            out.append((ts, entry))
        elif entry.is_file():
            m = LEGACY_LOG_RE.match(entry.name)
            if not m:
                continue
            try:
                ts = datetime.strptime(m.group("ts"), TS_FMT_RUN)
            except ValueError:
                continue
            # Synthesize the post-promotion directory path.
            out.append((ts, runs_root / entry.stem))
    out.sort(key=lambda x: x[0])
    return out


def _find_owning_run(
    capture_ts: datetime, runs: list[tuple[datetime, Path]]
) -> Path | None:
    """Return the latest run dir whose start_ts <= capture_ts.

    If captures fall before every known run start, return None (orphan).
    """
    chosen: Path | None = None
    for start_ts, path in runs:
        if start_ts <= capture_ts:
            chosen = path
        else:
            break
    return chosen


def _rename_logs_to_runs(mission_dir: Path, dry_run: bool) -> Path | None:
    """Rename ``<mission>/logs/`` -> ``<mission>/runs/``.

    Returns the resulting runs dir path. None if neither exists (nothing
    to migrate). If both exist (partial prior migration), bails out so
    the user can sort out the conflict by hand.
    """
    logs = mission_dir / "logs"
    runs = mission_dir / "runs"
    if logs.is_dir() and runs.is_dir():
        print(
            f"  ! both logs/ and runs/ exist in {mission_dir}; "
            f"manual reconciliation needed — skipping",
            file=sys.stderr,
        )
        return None
    if runs.is_dir():
        return runs  # already migrated
    if not logs.is_dir():
        return None  # nothing to rename; still let captures migrate (creates runs/ lazily)
    if dry_run:
        print(f"  [dry-run] would rename {logs} -> {runs}")
        return runs  # pretend, so capture pass can plan
    logs.rename(runs)
    print(f"  renamed {logs.name}/ -> {runs.name}/")
    return runs


def _promote_legacy_logs(runs_root: Path, dry_run: bool) -> None:
    """Promote loose ``<run>.log`` files into ``<run>/main.log`` dirs.

    Older runs (pre per-run sidecar split) wrote a single flat log file
    instead of a directory. Wrap each one in a per-run dir so every run
    has a uniform shape going forward.
    """
    if not runs_root.is_dir():
        return
    # In dry-run mode the parent may still be named logs/; tolerate both.
    scan_root = runs_root
    if dry_run and not runs_root.is_dir():
        return
    for entry in sorted(runs_root.iterdir()) if runs_root.is_dir() else []:
        if not entry.is_file():
            continue
        m = LEGACY_LOG_RE.match(entry.name)
        if not m:
            continue
        # <mission>_<TS>.log -> <mission>_<TS>/main.log
        run_dir = runs_root / entry.stem
        dest = run_dir / "main.log"
        if dest.exists():
            continue
        if dry_run:
            print(f"  [dry-run] would promote {entry.name} -> {entry.stem}/main.log")
        else:
            run_dir.mkdir(parents=True, exist_ok=True)
            shutil.move(str(entry), str(dest))
            print(f"  promoted {entry.name} -> {entry.stem}/main.log")


def _migrate_captures(mission_dir: Path, runs_root: Path, dry_run: bool) -> None:
    captures = mission_dir / "captures"
    if not captures.is_dir():
        return

    # In dry-run mode the rename may not have happened yet, so fall
    # back to scanning the legacy logs/ dir if runs/ is absent.
    runs_for_lookup = runs_root
    if dry_run and not runs_for_lookup.is_dir():
        legacy = mission_dir / "logs"
        if legacy.is_dir():
            runs_for_lookup = legacy

    runs = _list_run_dirs(runs_for_lookup)

    jpgs = sorted(p for p in captures.iterdir() if p.suffix.lower() == ".jpg")
    if not jpgs:
        # Maybe just sidecars left over; clean up below.
        pass

    moved = 0
    orphans = 0
    skipped = 0
    for jpg in jpgs:
        m = CAPTURE_RE.match(jpg.name)
        if not m:
            print(
                f"  ? {jpg.name}: filename doesn't match expected pattern; "
                f"skipping",
                file=sys.stderr,
            )
            skipped += 1
            continue
        wp = m.group("wp")
        bearing = m.group("bearing")
        ts_str = m.group("ts")
        try:
            capture_ts = datetime.strptime(ts_str, TS_FMT_CAPTURE)
        except ValueError:
            print(f"  ? {jpg.name}: bad timestamp '{ts_str}'; skipping", file=sys.stderr)
            skipped += 1
            continue

        owning_run = _find_owning_run(capture_ts, runs)
        if owning_run is None:
            dest_dir = runs_root / "_orphans" / wp
            orphans += 1
        else:
            dest_dir = owning_run / "captures" / wp

        dest_jpg = dest_dir / f"{bearing}_{ts_str}.jpg"
        sidecar = jpg.with_suffix(".json")
        dest_sidecar = dest_dir / f"{bearing}_{ts_str}.json"

        if dest_jpg.exists():
            print(f"  = {jpg.name}: {dest_jpg} already exists; skipping")
            skipped += 1
            continue

        if dry_run:
            tag = "orphan" if owning_run is None else owning_run.name
            print(f"  [dry-run] {jpg.name} -> {tag}/captures/{wp}/{dest_jpg.name}")
        else:
            dest_dir.mkdir(parents=True, exist_ok=True)
            shutil.move(str(jpg), str(dest_jpg))
            if sidecar.exists():
                shutil.move(str(sidecar), str(dest_sidecar))
            tag = "orphan" if owning_run is None else owning_run.name
            print(f"  moved {jpg.name} -> {tag}/captures/{wp}/{dest_jpg.name}")
        moved += 1

    # Stray sidecars whose paired JPG never existed (manual deletion
    # before migration). The dry-run pass leaves all sidecars in place
    # since no real moves happened, so suppress this warning there.
    if not dry_run:
        for stray in sorted(captures.iterdir()):
            if stray.suffix.lower() == ".json":
                print(
                    f"  ? {stray.name}: orphaned sidecar with no matching jpg "
                    f"— leaving in place"
                )

    print(
        f"  summary: moved={moved} orphans={orphans} skipped={skipped}"
    )

    # Try to remove the now-empty captures/ dir.
    if not dry_run:
        try:
            captures.rmdir()
            print(f"  removed empty {captures.name}/")
        except OSError:
            # Not empty (orphaned sidecars, hidden files); leave it.
            pass


def migrate_mission(mission_dir: Path, dry_run: bool) -> None:
    print(f"\n=== {mission_dir.relative_to(mission_dir.parent.parent) if mission_dir.parent.parent.exists() else mission_dir} ===")
    runs_root = _rename_logs_to_runs(mission_dir, dry_run)
    if runs_root is None:
        runs_root = mission_dir / "runs"
    # In dry-run the rename hasn't actually happened, so legacy promotion
    # has to walk the original logs/ dir to plan accurately.
    promote_root = runs_root
    if dry_run and not runs_root.is_dir():
        legacy = mission_dir / "logs"
        if legacy.is_dir():
            promote_root = legacy
    _promote_legacy_logs(promote_root, dry_run)
    _migrate_captures(mission_dir, runs_root, dry_run)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Migrate <mission>/logs + <mission>/captures to "
        "<mission>/runs/<TS>/captures/<wp>/ layout."
    )
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        help="Mission directory or missions root (default: ./dev/missions)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print planned moves without touching disk",
    )
    args = parser.parse_args(argv)

    if not args.paths:
        # Default to the missions root relative to the repo root (one
        # directory above this script's dev/scripts/ location).
        repo_root = Path(__file__).resolve().parent.parent.parent
        args.paths = [repo_root / "dev" / "missions"]

    missions = _iter_mission_dirs(args.paths)
    if not missions:
        print("No mission directories found.", file=sys.stderr)
        return 1

    mode = "DRY RUN" if args.dry_run else "LIVE"
    print(f"[{mode}] Migrating {len(missions)} mission(s) to runs/ layout")

    for mission_dir in missions:
        migrate_mission(mission_dir, args.dry_run)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
