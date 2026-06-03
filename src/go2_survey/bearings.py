"""Post-hoc image-bearing finalization for the line-survey strategy.

At capture time the EXIF GPSImgDirection holds a provisional bearing (the
calibrated IMU heading / target). Once the survey finishes, the whole RTK
track of each leg is known, so we compute a far better *centered* course per
capture — a look-back + look-forward chord clamped to the leg — and write it
into both the sidecar JSON and the JPEG EXIF.

This is the in-pipeline version of the offline corrector that produced the
aerial movie's `cor_true`. It reuses the per-capture positions already in the
sidecars (no dense GPS track needed) and rewrites only the EXIF
GPSImgDirection via piexif, leaving the JPEG pixels untouched.
"""

import glob
import json
import logging
import os
import re
from fractions import Fraction
from pathlib import Path

import piexif

from go2_survey.geometry import calculate_bearing, haversine_distance

logger = logging.getLogger(__name__)

_NAME_RE = re.compile(r"leg(\d+)_m(\d+)_")


def _deg_to_rational(deg: float) -> tuple[int, int]:
    """EXIF GPSImgDirection rational (num, den) at 0.01-degree precision."""
    return (int(round((deg % 360.0) * 100)), 100)


def _rationalize(exif: dict) -> None:
    """piexif.dump can't serialize a bare float for a rational-typed tag, and
    Pillow writes GPSHPositioningError as a float — so a load->modify->dump
    round-trip raises. Convert any float IFD value to a (num, den) rational.
    """
    for ifd_name in ("0th", "Exif", "GPS", "1st"):
        ifd = exif.get(ifd_name)
        if not ifd:
            continue
        for tag, val in list(ifd.items()):
            if isinstance(val, float):
                fr = Fraction(val).limit_denominator(1_000_000)
                ifd[tag] = (fr.numerator, fr.denominator)


def _collect_rows(captures_dir: Path) -> list[dict]:
    """Read line-survey sidecars into ordered (leg, mark, lat, lon) rows."""
    rows = []
    for f in sorted(glob.glob(str(captures_dir / "*.json"))):
        m = _NAME_RE.match(os.path.basename(f))
        if not m:
            continue
        try:
            d = json.loads(Path(f).read_text())
        except (OSError, json.JSONDecodeError) as e:
            logger.warning(f"finalize_bearings: skip unreadable sidecar {f}: {e}")
            continue
        p = d.get("position") or {}
        if p.get("latitude") is None or p.get("longitude") is None:
            continue
        rows.append({
            "sidecar": Path(f),
            "leg": int(m.group(1)),
            "mark": int(m.group(2)),
            "lat": p["latitude"],
            "lon": p["longitude"],
        })
    rows.sort(key=lambda r: (r["leg"], r["mark"]))
    return rows


def compute_bearings(
    rows: list[dict], window_m: float = 1.0
) -> list[tuple[dict, float, str]]:
    """Per-leg centered (look-back + look-forward) true bearing for each row.

    Pure (no I/O). For each capture: bearing of the chord from the nearest
    same-leg neighbour >= window_m behind to the nearest >= window_m ahead;
    degrade to forward-only (leg start), backward-only (leg end), or the leg's
    overall first->last direction (leg too short for either). Mirrors
    prep2.py's `cor_true`. Returns [(row, bearing_deg, source), ...].
    """
    by_leg: dict[int, list[int]] = {}
    for i, r in enumerate(rows):
        by_leg.setdefault(r["leg"], []).append(i)

    out: list[tuple[dict, float, str]] = []
    for idx in by_leg.values():
        pts = [(rows[i]["lat"], rows[i]["lon"]) for i in idx]
        leg_dir = calculate_bearing(*pts[0], *pts[-1]) if len(pts) > 1 else 0.0
        for k, i in enumerate(idx):
            la0, lo0 = pts[k]
            jb = jf = None
            for q in range(k - 1, -1, -1):
                if haversine_distance(pts[q][0], pts[q][1], la0, lo0) >= window_m:
                    jb = q
                    break
            for q in range(k + 1, len(pts)):
                if haversine_distance(pts[q][0], pts[q][1], la0, lo0) >= window_m:
                    jf = q
                    break
            if jb is not None and jf is not None:
                brg, src = calculate_bearing(*pts[jb], *pts[jf]), "gps_course_posthoc"
            elif jf is not None:
                brg, src = calculate_bearing(la0, lo0, *pts[jf]), "gps_course_posthoc"
            elif jb is not None:
                brg, src = calculate_bearing(*pts[jb], la0, lo0), "gps_course_posthoc"
            else:
                brg, src = leg_dir, "leg_direction"
            out.append((rows[i], brg, src))
    return out


def _apply_bearing(sidecar_path: Path, bearing: float, source: str) -> bool:
    """Write `bearing` to the sidecar JSON and the sibling JPEG's EXIF.

    Sidecar: set heading.course_degrees_true + heading.exif_direction_source,
    bump schema_version to 3 (achieved_degrees_true kept as IMU provenance).
    JPEG: rewrite only GPSImgDirection(+Ref) via piexif — no pixel re-encode.
    Returns True if the sidecar was updated.
    """
    bearing = bearing % 360.0
    ok = False
    try:
        d = json.loads(sidecar_path.read_text())
        h = d.setdefault("heading", {})
        h["course_degrees_true"] = round(bearing, 2)
        h["exif_direction_source"] = source
        d["schema_version"] = 3
        sidecar_path.write_text(json.dumps(d, indent=2, default=str))
        ok = True
    except (OSError, json.JSONDecodeError) as e:
        logger.warning(
            f"finalize_bearings: sidecar update failed {sidecar_path.name}: {e}"
        )

    jpg = sidecar_path.with_suffix(".jpg")
    if jpg.exists():
        try:
            exif = piexif.load(str(jpg))
            _rationalize(exif)
            exif["GPS"][piexif.GPSIFD.GPSImgDirectionRef] = b"T"
            exif["GPS"][piexif.GPSIFD.GPSImgDirection] = _deg_to_rational(bearing)
            piexif.insert(piexif.dump(exif), str(jpg))
        except Exception as e:
            logger.warning(f"finalize_bearings: EXIF update failed {jpg.name}: {e}")
    else:
        logger.warning(f"finalize_bearings: no JPEG beside {sidecar_path.name}")
    return ok


def finalize_bearings(captures_dir, window_m: float = 1.0) -> int:
    """Rewrite each line-survey capture's bearing to a post-hoc centered RTK
    course (look-back + look-forward, leg-clamped), in both the sidecar JSON
    and the JPEG EXIF GPSImgDirection. Returns the number of captures updated.
    """
    captures_dir = Path(captures_dir)
    rows = _collect_rows(captures_dir)
    if not rows:
        logger.info(f"finalize_bearings: no line-survey captures in {captures_dir}")
        return 0
    n = 0
    for row, brg, src in compute_bearings(rows, window_m):
        if _apply_bearing(row["sidecar"], brg, src):
            n += 1
    logger.info(
        f"finalize_bearings: updated {n}/{len(rows)} capture bearings in {captures_dir}"
    )
    return n
