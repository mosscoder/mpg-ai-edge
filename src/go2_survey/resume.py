"""Resume an interrupted line survey from the last photo of a given run.

``go2-survey run --resume <failed_run_dir>`` continues an interrupted survey
(GPS loss, Ctrl-C, a collapse) without re-walking the covered ground. You point
it at the failed run; it reads that run's **last photo**, makes that photo's
position the start, and drives the remaining legs into the same run dir so the
survey ends up as one complete dataset. The mission (waypoints) is derived from
the run path (``<mission>/runs/<run>``).

The last photo is the durable record of where the dog got to — and a frame only
exists because its position already passed the live quality gate at capture
time, so the last readable capture is, by construction, a good anchor. No
re-gating, no scanning sibling runs: the operator names the run to continue.
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
    """The point an interrupted survey resumes from — the run's last photo."""

    run_dir: Path     # the run dir to resume INTO (captures append here)
    leg: int          # 1-indexed leg number (legNN) of the last photo
    mark: int         # mark index of the last photo within that leg
    latitude: float
    longitude: float
    n_captured: int   # photos already in the run (kept; for the banner tally)


def anchor_from_run(run_dir: Path, output_subdir: str = "captures") -> ResumeAnchor | None:
    """The last photo of ``run_dir`` as a resume anchor, or None if it has no
    readable capture.

    Walks the capture sidecars in survey order ``(leg, mark)`` and takes the
    highest one that carries a position — skipping a half-written final sidecar
    from an abrupt stop. No fix/hAcc re-gate: a capture only got written because
    its position already passed the live quality check, so its existence is the
    signal.
    """
    captures = run_dir / output_subdir
    if not captures.is_dir():
        logger.error("--resume: no captures directory at %s", captures)
        return None

    keyed: list[tuple[tuple[int, int], Path]] = []
    for sidecar in captures.glob("leg*_m*.json"):
        m = _SIDECAR_RE.match(sidecar.name)
        if m:
            keyed.append(((int(m.group(1)), int(m.group(2))), sidecar))
    if not keyed:
        logger.error("--resume: %s has no captures to resume from", run_dir.name)
        return None

    n_captured = len(list(captures.glob("leg*_m*.jpg")))  # frames already on disk
    for (leg, mark), sidecar in sorted(keyed, key=lambda kv: kv[0], reverse=True):
        try:
            pos = json.loads(sidecar.read_text()).get("position") or {}
        except (OSError, json.JSONDecodeError):
            continue  # half-written sidecar from an abrupt stop — try the prior
        if pos.get("latitude") is None or pos.get("longitude") is None:
            continue
        return ResumeAnchor(
            run_dir=run_dir, leg=leg, mark=mark,
            latitude=pos["latitude"], longitude=pos["longitude"],
            n_captured=n_captured,
        )

    logger.error("--resume: %s has no readable photo position", run_dir.name)
    return None
