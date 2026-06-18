"""Resume an interrupted line survey from the last cleanly-captured mark.

``go2-survey run DIR --resume`` picks an interrupted survey back up (GPS loss,
Ctrl-C, crash) without re-walking the covered ground. The durable record is the
per-capture sidecar JSON — written on every frame — so the resume anchor
survives any abrupt stop with no separate checkpoint file.

:func:`find_resume_anchor` inspects the newest run directory by timestamp and
returns the highest-``(leg, mark)`` capture whose RTK position passes the
mission's fix gate (so a GPS-degraded tail of bad fixes is discarded). The
mission runner then treats that mark's position as a synthetic starting
waypoint: the dog drives to it (self-seeding the IMU offset and lining up on the
leg bearing) and continues the remaining legs, writing into the *same* run
directory so the survey ends up as one complete dataset.
"""

from __future__ import annotations

import json
import logging
import re
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)

# capture sidecars are named "leg<NN>_m<MMM>_b<BBB>_<ts>.json"
_SIDECAR_RE = re.compile(r"leg(\d+)_m(\d+)_")


@dataclass
class ResumeAnchor:
    """The point an interrupted survey resumes from."""

    run_dir: Path     # the partial run dir to resume INTO (captures append here)
    leg: int          # 1-indexed leg number (legNN) of the last good mark
    mark: int         # mark index of the last good mark within that leg
    latitude: float
    longitude: float
    n_captured: int   # quality-passing captures already in the run (banner total)


def _anchor_in_run(
    run_dir: Path, output_subdir: str, min_fix_type: int, max_hacc: float
) -> ResumeAnchor | None:
    """Highest ``(leg, mark)`` capture in ``run_dir`` passing the fix gate, else
    None. Half-written sidecars (from an abrupt stop) and a GPS-degraded tail of
    sub-quality fixes are skipped, so the anchor is the last *good* mark."""
    captures = run_dir / output_subdir
    if not captures.is_dir():
        return None
    best_key: tuple[int, int] | None = None
    best_pos: tuple[float, float] = (0.0, 0.0)
    n_good = 0
    for sidecar in captures.glob("leg*_m*.json"):
        m = _SIDECAR_RE.match(sidecar.name)
        if not m:
            continue
        try:
            data = json.loads(sidecar.read_text())
        except (OSError, json.JSONDecodeError):
            continue  # half-written sidecar from an abrupt stop — skip
        pos = data.get("position") or {}
        lat, lon = pos.get("latitude"), pos.get("longitude")
        if lat is None or lon is None:
            continue
        if (pos.get("fix_type") or 0) < min_fix_type:
            continue
        hacc = pos.get("accuracy_horizontal")
        if hacc is None or hacc > max_hacc:
            continue
        n_good += 1
        key = (int(m.group(1)), int(m.group(2)))
        if best_key is None or key > best_key:
            best_key, best_pos = key, (lat, lon)
    if best_key is None:
        return None
    return ResumeAnchor(
        run_dir=run_dir, leg=best_key[0], mark=best_key[1],
        latitude=best_pos[0], longitude=best_pos[1], n_captured=n_good,
    )


def _is_complete(run_dir: Path) -> bool:
    """True if this run already logged ``LINE SURVEY COMPLETE`` (nothing to do)."""
    main_log = run_dir / "main.log"
    if not main_log.exists():
        return False
    try:
        return "LINE SURVEY COMPLETE" in main_log.read_text(errors="replace")
    except OSError:
        return False


def find_resume_anchor(
    run_parent: Path, output_subdir: str, min_fix_type: int, max_hacc: float
) -> ResumeAnchor | None:
    """Find the mark to resume from across the runs under ``run_parent``.

    Walks run directories newest-first (the dir name carries the run
    timestamp, which sorts chronologically). Refuses if the newest real run
    already logged ``LINE SURVEY COMPLETE`` (the survey finished). Falls through
    0-capture false-starts to the newest run that has quality-passing marks.
    Returns the anchor, or None if there is nothing to resume.
    """
    if not run_parent.is_dir():
        logger.debug("--resume: no runs directory at %s", run_parent)
        return None
    for run_dir in sorted(
        (d for d in run_parent.iterdir() if d.is_dir()), reverse=True
    ):
        if _is_complete(run_dir):
            logger.debug("--resume: %s already complete", run_dir.name)
            return None
        anchor = _anchor_in_run(run_dir, output_subdir, min_fix_type, max_hacc)
        if anchor is not None:
            return anchor
        logger.debug("--resume: %s has no quality captures; older run", run_dir.name)
    return None
