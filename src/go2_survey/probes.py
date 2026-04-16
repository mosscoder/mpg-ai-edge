"""Diagnostic probes for hardware whose behavior we don't yet fully
characterize. Used by mission_runner mode="probe_gps".

The F9R probe runs three phases:

    1. One-time snapshots (firmware via MON-VER, sensor fusion via
       ESF-STATUS, initial status via NAV-STATUS, dual-offset headAcc
       read from NAV-PVT).
    2. Steady-state sampling — NAV-PVT + NAV-HPPOSLLH + NAV-DOP at
       full rate, NAV-STATUS/NAV-SAT/ESF-STATUS every 10th sample.
       Each sample written as one JSON line.
    3. Report — markdown summary answering each outstanding question.

No robot, no navigation. Just the GPS on USB + NTRIP corrections.
"""

from __future__ import annotations

import asyncio
import json
import logging
import statistics
import time
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional

from go2_survey.gps import GPSManager
from go2_survey.logging_utils import log_banner

logger = logging.getLogger(__name__)


async def run_f9r_probe(
    gps: GPSManager,
    out_dir: Path,
    duration_sec: float = 120.0,
    sample_rate_hz: float = 5.0,
) -> Dict:
    """Run the F9R diagnostic probe. Returns the summary dict.

    Writes `probe_<timestamp>.jsonl` (one sample per line) and
    `probe_<timestamp>_report.md` (human-readable summary) into
    `out_dir`.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    ts_tag = time.strftime("%Y-%m-%d_%H-%M-%S")
    jsonl_path = out_dir / f"probe_{ts_tag}.jsonl"
    report_path = out_dir / f"probe_{ts_tag}_report.md"

    log_banner("F9R PROBE — PHASE 1: SNAPSHOTS", char="-", logger=logger)
    snapshots = _phase1_snapshots(gps)

    log_banner(
        f"F9R PROBE — PHASE 2: SAMPLING ({duration_sec:.0f}s @ {sample_rate_hz:.1f}Hz)",
        char="-",
        logger=logger,
    )
    samples = await _phase2_sample(gps, duration_sec, sample_rate_hz, jsonl_path)

    log_banner("F9R PROBE — PHASE 3: REPORT", char="-", logger=logger)
    summary = _phase3_report(snapshots, samples, report_path)

    for line in _format_summary_banner(summary):
        logger.info(line)
    logger.info(f"JSONL data: {jsonl_path}")
    logger.info(f"Report:     {report_path}")

    return summary


def _phase1_snapshots(gps: GPSManager) -> Dict:
    """One-time diagnostic pulls at the start of the probe."""
    snapshots: Dict = {}

    mon = gps.gps.poll_mon_ver()
    snapshots["mon_ver"] = mon
    if mon:
        logger.info(f"MON-VER sw:  {mon.get('sw')!r}")
        logger.info(f"MON-VER hw:  {mon.get('hw')!r}")
        for ext in mon.get("extensions", []):
            logger.info(f"MON-VER ext: {ext!r}")
    else:
        logger.warning("MON-VER poll failed")

    esf = gps.gps.poll_esf_status()
    snapshots["esf_status"] = esf
    if esf:
        logger.info(
            f"ESF fusionMode: {esf.get('fusionModeLabel')} "
            f"({esf.get('numSensors')} sensors)"
        )
        for s in esf.get("sensors", []):
            logger.info(
                f"  sensor type={s['type']} used={s['used']} "
                f"ready={s['ready']} calib={s['calibration']}"
            )
    else:
        logger.warning("ESF-STATUS poll failed")

    status = gps.gps.poll_nav_status()
    snapshots["nav_status"] = status
    if status:
        logger.info(
            f"NAV-STATUS gpsFix={status['gpsFix']} "
            f"diffSoln={status['diffSoln']} ttff_ms={status['ttff_ms']} "
            f"msss_ms={status['msss_ms']}"
        )
    else:
        logger.warning("NAV-STATUS poll failed")

    pvt = gps.gps.poll_nav_pvt()
    snapshots["nav_pvt_initial"] = pvt
    if pvt:
        logger.info(
            f"NAV-PVT headAcc candidates: offset_72={pvt['headAcc_72']} "
            f"offset_88={pvt['headAcc_88']} (raw deg × 1e-5)"
        )
        logger.info(
            f"  offset_72 → {pvt['headAcc_72'] * 1e-5:.3f}°, "
            f"offset_88 → {pvt['headAcc_88'] * 1e-5:.3f}°"
        )
    else:
        logger.warning("NAV-PVT initial poll failed")

    return snapshots


async def _phase2_sample(
    gps: GPSManager,
    duration_sec: float,
    sample_rate_hz: float,
    jsonl_path: Path,
) -> List[Dict]:
    """Steady-state sampling loop. Writes JSONL + returns in-memory list."""
    period = 1.0 / sample_rate_hz
    samples: List[Dict] = []
    t_start = time.monotonic()
    t_last_summary = t_start
    n = 0

    with jsonl_path.open("w") as fh:
        while time.monotonic() - t_start < duration_sec:
            t_sample_start = time.monotonic()
            wall = time.time()

            pvt = gps.gps.poll_nav_pvt()
            hpposllh = gps.gps.poll_nav_hpposllh()
            dop = gps.gps.poll_nav_dop()

            # Expensive polls throttled to every 10th sample.
            sat = None
            status = None
            esf = None
            if n % 10 == 0:
                sat = gps.gps.poll_nav_sat()
                status = gps.gps.poll_nav_status()
                esf = gps.gps.poll_esf_status()

            sample = {
                "n": n,
                "t_wall": wall,
                "t_mono": t_sample_start,
                "nav_pvt": pvt,
                "nav_hpposllh": hpposllh,
                "nav_dop": dop,
                "nav_sat": sat,
                "nav_status": status,
                "esf_status": esf,
            }
            fh.write(json.dumps(sample) + "\n")
            samples.append(sample)
            n += 1

            now = time.monotonic()
            if now - t_last_summary >= 5.0:
                _log_interval_summary(samples, t_last_summary, now)
                t_last_summary = now

            elapsed = time.monotonic() - t_sample_start
            to_sleep = period - elapsed
            if to_sleep > 0:
                await asyncio.sleep(to_sleep)

    logger.info(f"Collected {n} samples over {time.monotonic() - t_start:.1f}s")
    return samples


def _log_interval_summary(samples: List[Dict], t_from: float, t_to: float) -> None:
    """Summarize the most recent window at INFO level."""
    window = [s for s in samples if t_from <= s["t_mono"] <= t_to]
    pvts = [s["nav_pvt"] for s in window if s["nav_pvt"]]
    if not pvts:
        logger.info("[interval] no valid NAV-PVT in last 5s")
        return

    hacc = statistics.mean(p["hAcc"] for p in pvts)
    pdop_samples = [p["pDOP"] for p in pvts if p.get("pDOP")]
    pdop_mean = statistics.mean(pdop_samples) if pdop_samples else float("nan")
    valid_hd = sum(1 for p in pvts if p.get("headVehValid"))
    headveh = [p["headVeh"] * 1e-5 for p in pvts if p.get("headVehValid")]
    heading_str = (
        f"mean={statistics.mean(headveh):.2f}°" if headveh else "unavailable"
    )
    last = pvts[-1]
    geoid = (last["hMSL"] - last["height"]) if last.get("hMSL") is not None else None
    geoid_str = f"geoid_sep={geoid:+.2f}m" if geoid is not None else "geoid_sep=?"

    logger.info(
        f"[interval] {len(pvts)} samples | hAcc_mean={hacc:.3f}m | "
        f"pDOP={pdop_mean:.2f} | numSV={last['numSV']} | {geoid_str} | "
        f"headVehValid={valid_hd}/{len(pvts)} ({heading_str})"
    )


def _phase3_report(
    snapshots: Dict, samples: List[Dict], report_path: Path
) -> Dict:
    """Synthesize answers to each outstanding question."""
    pvts = [s["nav_pvt"] for s in samples if s["nav_pvt"]]
    hpps = [s["nav_hpposllh"] for s in samples if s["nav_hpposllh"]]

    # Heading accuracy candidates
    ha72 = [p["headAcc_72"] * 1e-5 for p in pvts]
    ha88 = [p["headAcc_88"] * 1e-5 for p in pvts]
    ha72_mean = statistics.mean(ha72) if ha72 else float("nan")
    ha88_mean = statistics.mean(ha88) if ha88 else float("nan")
    # Plausible heading accuracy: < 10° typical, datasheet claims 0.2°.
    # Implausible: giant numbers, or values > 360°.
    ha72_plausible = 0.0 <= ha72_mean <= 360.0
    ha88_plausible = 0.0 <= ha88_mean <= 360.0

    # headVeh behavior
    valid_count = sum(1 for p in pvts if p.get("headVehValid"))
    heads = [p["headVeh"] * 1e-5 for p in pvts if p.get("headVehValid")]
    heads_stats = None
    if heads:
        heads_stats = {
            "mean": statistics.mean(heads),
            "stddev": statistics.pstdev(heads) if len(heads) > 1 else 0.0,
            "min": min(heads),
            "max": max(heads),
        }

    # Geoid separation (hMSL - hEllipsoid)
    seps = [p["hMSL"] - p["height"] for p in pvts if p.get("hMSL") is not None]
    geoid_sep = statistics.mean(seps) if seps else None

    # pDOP range
    pdops = [p["pDOP"] for p in pvts if p.get("pDOP")]
    pdop_stats = None
    if pdops:
        pdop_stats = {
            "mean": statistics.mean(pdops),
            "min": min(pdops),
            "max": max(pdops),
        }

    # Precision gain — compare NAV-HPPOSLLH lat resolution vs NAV-PVT's
    # raw integer grid. If HPPOSLLH's latHp bytes are nonzero, it adds
    # real precision; if they're all zero, NAV-PVT is already the floor.
    nonzero_hp = 0
    for h in hpps:
        if h.get("latHp") or h.get("lonHp"):
            nonzero_hp += 1
    hp_fraction = nonzero_hp / len(hpps) if hpps else 0.0

    # Per-constellation sat counts (from the nav_sat samples we captured)
    gnss_used: Counter = Counter()
    for s in samples:
        sat = s.get("nav_sat")
        if sat:
            for rec in sat.get("sats", []):
                if rec.get("used"):
                    gnss_used[rec["gnss"]] += 1
    gnss_per_sample = {k: v / max(1, sum(1 for s in samples if s.get("nav_sat")))
                       for k, v in gnss_used.items()}

    # Differential correction age — coarse. NAV-STATUS gives `msss` (ms
    # since startup). Better correction age comes from NAV-HPPOSLLH or
    # UBX-NAV-PVT's flags but u-blox has long surfaced it via
    # NAV-RELPOSNED / NAV-STATUS.flags only.
    diff_samples = [s["nav_status"] for s in samples if s.get("nav_status")]
    diff_applied = sum(1 for d in diff_samples if d and d.get("diffSoln"))
    diff_fraction = diff_applied / len(diff_samples) if diff_samples else 0.0

    summary = {
        "n_samples": len(samples),
        "firmware": snapshots.get("mon_ver"),
        "fusion": snapshots.get("esf_status"),
        "headAcc_offset_72": {
            "mean_deg": ha72_mean,
            "plausible": ha72_plausible,
            "raw_mean": statistics.mean([p["headAcc_72"] for p in pvts]) if pvts else None,
        },
        "headAcc_offset_88": {
            "mean_deg": ha88_mean,
            "plausible": ha88_plausible,
            "raw_mean": statistics.mean([p["headAcc_88"] for p in pvts]) if pvts else None,
        },
        "headVeh": {
            "valid_count": valid_count,
            "total": len(pvts),
            "stats_deg": heads_stats,
        },
        "geoid_separation_m": geoid_sep,
        "pdop": pdop_stats,
        "nav_hpposllh_nonzero_hp_fraction": hp_fraction,
        "gnss_used_mean_per_sample": gnss_per_sample,
        "diff_correction_fraction": diff_fraction,
    }

    _write_report_markdown(summary, report_path)
    return summary


def _write_report_markdown(summary: Dict, path: Path) -> None:
    lines: List[str] = []
    lines.append(f"# F9R Probe Report")
    lines.append("")
    lines.append(f"Samples: {summary['n_samples']}")
    lines.append("")

    fw = summary["firmware"] or {}
    lines.append("## Firmware")
    lines.append("")
    lines.append(f"- **sw:** `{fw.get('sw', 'unknown')}`")
    lines.append(f"- **hw:** `{fw.get('hw', 'unknown')}`")
    for ext in fw.get("extensions", []) if fw else []:
        lines.append(f"- ext: `{ext}`")
    lines.append("")

    fusion = summary["fusion"] or {}
    lines.append("## Sensor fusion (ESF-STATUS)")
    lines.append("")
    lines.append(f"- **fusionMode:** {fusion.get('fusionModeLabel', 'unknown')}")
    lines.append(f"- **numSensors:** {fusion.get('numSensors', 0)}")
    for s in fusion.get("sensors", []) if fusion else []:
        lines.append(
            f"  - `{s['type']}` used={s['used']} ready={s['ready']} "
            f"calib={s['calibration']}"
        )
    lines.append("")

    h72 = summary["headAcc_offset_72"]
    h88 = summary["headAcc_offset_88"]
    lines.append("## Heading accuracy offset")
    lines.append("")
    lines.append(f"- Offset 72: mean {h72['mean_deg']:.3f}° "
                 f"(raw {h72['raw_mean']}) — "
                 f"{'plausible' if h72['plausible'] else 'IMPLAUSIBLE'}")
    lines.append(f"- Offset 88: mean {h88['mean_deg']:.3f}° "
                 f"(raw {h88['raw_mean']}) — "
                 f"{'plausible' if h88['plausible'] else 'IMPLAUSIBLE'}")
    if h72["plausible"] and not h88["plausible"]:
        lines.append("- **Conclusion: offset 72 is the real headAcc.**")
    elif h88["plausible"] and not h72["plausible"]:
        lines.append("- **Conclusion: offset 88 is the real headAcc.**")
    else:
        lines.append("- Conclusion: inconclusive — both ranges look similar; "
                     "verify against datasheet.")
    lines.append("")

    hv = summary["headVeh"]
    lines.append("## headVeh behavior")
    lines.append("")
    lines.append(f"- Valid samples: {hv['valid_count']}/{hv['total']}")
    if hv["stats_deg"]:
        s = hv["stats_deg"]
        lines.append(f"- Mean: {s['mean']:.3f}°")
        lines.append(f"- Stddev: {s['stddev']:.3f}°")
        lines.append(f"- Range: {s['min']:.3f}° – {s['max']:.3f}°")
    lines.append("")

    lines.append("## Other findings")
    lines.append("")
    if summary["geoid_separation_m"] is not None:
        lines.append(
            f"- Geoid separation (hMSL − hEllipsoid): "
            f"{summary['geoid_separation_m']:+.3f} m"
        )
    if summary["pdop"]:
        p = summary["pdop"]
        lines.append(
            f"- pDOP: mean {p['mean']:.2f}, range {p['min']:.2f} – {p['max']:.2f}"
        )
    lines.append(
        f"- NAV-HPPOSLLH high-precision bytes nonzero in "
        f"{summary['nav_hpposllh_nonzero_hp_fraction']:.0%} of samples "
        f"(near 0 → NAV-PVT is the precision floor; near 1 → HPPOSLLH adds real digits)"
    )
    if summary["gnss_used_mean_per_sample"]:
        lines.append("- Mean satellites used per fix, by constellation:")
        for k, v in sorted(summary["gnss_used_mean_per_sample"].items()):
            lines.append(f"  - {k}: {v:.1f}")
    lines.append(
        f"- Differential correction applied in "
        f"{summary['diff_correction_fraction']:.0%} of NAV-STATUS samples"
    )
    lines.append("")

    path.write_text("\n".join(lines))


def _format_summary_banner(summary: Dict) -> List[str]:
    lines = []
    lines.append("=" * 60)
    lines.append("F9R PROBE SUMMARY")
    lines.append("=" * 60)
    h72 = summary["headAcc_offset_72"]
    h88 = summary["headAcc_offset_88"]
    lines.append(
        f"headAcc @ 72: {h72['mean_deg']:.3f}° "
        f"({'plausible' if h72['plausible'] else 'IMPLAUSIBLE'})"
    )
    lines.append(
        f"headAcc @ 88: {h88['mean_deg']:.3f}° "
        f"({'plausible' if h88['plausible'] else 'IMPLAUSIBLE'})"
    )
    hv = summary["headVeh"]
    lines.append(
        f"headVeh valid: {hv['valid_count']}/{hv['total']} samples"
    )
    if summary["geoid_separation_m"] is not None:
        lines.append(
            f"geoid separation: {summary['geoid_separation_m']:+.2f} m"
        )
    if summary["pdop"]:
        lines.append(
            f"pDOP mean: {summary['pdop']['mean']:.2f}"
        )
    lines.append("=" * 60)
    return lines
