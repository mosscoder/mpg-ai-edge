"""Diagnostic probes for hardware whose behavior we don't yet fully
characterize. Used by mission_runner modes "probe_gps" and "probe_lidar".

The F9R probe runs three phases:

    1. One-time snapshots (firmware via MON-VER, sensor fusion via
       ESF-STATUS, initial status via NAV-STATUS, dual-offset headAcc
       read from NAV-PVT).
    2. Steady-state sampling — NAV-PVT + NAV-HPPOSLLH + NAV-DOP at
       full rate, NAV-STATUS/NAV-SAT/ESF-STATUS every 10th sample.
       Each sample written as one JSON line.
    3. Report — markdown summary answering each outstanding question.

The lidar probe verifies the WebRTC voxel-map topic streams, captures
one decoded frame to .npy for offline rasterizer development, and
emits three quick-look PNGs (BEV / side elevation / forward cone) so
the operator can eyeball whether the data is sensible before any
downstream code consumes it.
"""

import asyncio
import json
import logging
import math
import statistics
import threading
import time
from collections import Counter
from pathlib import Path

from go2_survey.gps import GPSManager
from go2_survey.logging_utils import log_banner

logger = logging.getLogger(__name__)


async def run_f9r_probe(
    gps: GPSManager,
    out_dir: Path,
    duration_sec: float = 120.0,
    sample_rate_hz: float = 5.0,
) -> dict:
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


def _phase1_snapshots(gps: GPSManager) -> dict:
    """One-time diagnostic pulls at the start of the probe."""
    snapshots: dict = {}

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
            f"NAV-PVT headAcc: {pvt['headAcc']} raw (= "
            f"{pvt['headAcc'] * 1e-5:.3f}°)"
        )
        logger.info(
            f"NAV-PVT lastCorrectionAge bin: {pvt['lastCorrectionAge']} "
            f"(0=N/A, 1-11 = age bins up to 120s)"
        )
    else:
        logger.warning("NAV-PVT initial poll failed")

    return snapshots


async def _phase2_sample(
    gps: GPSManager,
    duration_sec: float,
    sample_rate_hz: float,
    jsonl_path: Path,
) -> list[dict]:
    """Steady-state sampling loop. Writes JSONL + returns in-memory list."""
    period = 1.0 / sample_rate_hz
    samples: list[dict] = []
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


def _log_interval_summary(samples: list[dict], t_from: float, t_to: float) -> None:
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
    snapshots: dict, samples: list[dict], report_path: Path
) -> dict:
    """Synthesize answers to each outstanding question."""
    pvts = [s["nav_pvt"] for s in samples if s["nav_pvt"]]
    hpps = [s["nav_hpposllh"] for s in samples if s["nav_hpposllh"]]

    # Heading accuracy (offset 72 per spec, confirmed by RTFM)
    ha = [p["headAcc"] * 1e-5 for p in pvts]
    ha_mean = statistics.mean(ha) if ha else float("nan")
    ha_max = max(ha) if ha else float("nan")

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
        "headAcc": {
            "mean_deg": ha_mean,
            "max_deg": ha_max,
            "raw_mean": statistics.mean([p["headAcc"] for p in pvts]) if pvts else None,
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


def _write_report_markdown(summary: dict, path: Path) -> None:
    lines: list[str] = []
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

    ha = summary["headAcc"]
    lines.append("## Heading accuracy (NAV-PVT offset 72, per F9 HPS 1.30 spec)")
    lines.append("")
    lines.append(f"- Mean: {ha['mean_deg']:.3f}°")
    lines.append(f"- Max:  {ha['max_deg']:.3f}°")
    lines.append(f"- Raw mean (deg × 1e-5): {ha['raw_mean']}")
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


def _format_summary_banner(summary: dict) -> list[str]:
    lines = []
    lines.append("=" * 60)
    lines.append("F9R PROBE SUMMARY")
    lines.append("=" * 60)
    ha = summary["headAcc"]
    lines.append(
        f"headAcc: mean {ha['mean_deg']:.3f}° / max {ha['max_deg']:.3f}°"
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


# ---------------------------------------------------------------------------
# Lidar probe — RTC_TOPIC["ULIDAR_ARRAY"] (rt/utlidar/voxel_map_compressed)
# ---------------------------------------------------------------------------
#
# Goal: confirm the library's documented lidar plumbing actually works
# end-to-end before any feature code is built on top. See
# docs/webrtc/README.md "Lidar data stream" for the source-derived
# expectations being verified here.

# Default probe duration (seconds). 30s gives 100-300 frames at typical
# lidar rates — enough to compute a stable mean rate and catch
# intermittent dropouts without parking the operator for too long.
LIDAR_PROBE_DURATION_S = 30.0

# Settle period before counting frames. The first 1-2s after subscribe
# tends to be empty while the data channel toggles state.
LIDAR_PROBE_SETTLE_S = 2.0


async def run_lidar_probe(
    robot,
    out_dir: Path,
    duration_sec: float = LIDAR_PROBE_DURATION_S,
) -> dict:
    """Subscribe to the Go2's compressed voxel-map topic, log per-frame
    metadata for `duration_sec`, and dump the first valid frame to
    .npy + three quick-look PNGs (BEV, side elevation, forward cone).

    Returns a summary dict with frame count, mean rate, per-frame point
    count distribution, and the metadata seen on the first frame.

    The robot must already be connected. This probe does not move the
    robot — it only reads from the data channel.
    """
    from unitree_webrtc_connect.constants import RTC_TOPIC

    out_dir.mkdir(parents=True, exist_ok=True)

    # Frame cache populated by the (synchronous) subscriber callback.
    # Holds the most recent decoded frame so the sampling loop can
    # snapshot it once per second without racing the producer.
    lock = threading.Lock()
    state: dict = {
        "frames": [],          # list of dicts: ts, point_count, origin, resolution, extents
        "first_payload": None,  # full message['data'] of the first frame (for offline replay)
        "first_points": None,   # np.ndarray(N, 3) of the first frame
        "errors": [],           # decode/shape errors caught from the callback
    }

    def on_lidar(message: dict) -> None:
        try:
            payload = message.get("data", {}) or {}
            decoded = payload.get("data")
            if decoded is None:
                state["errors"].append("payload missing data.data")
                return
            # native decoder returns {"points": np.ndarray(N, 3)};
            # libvoxel returns {"point_count", "positions", ...}. We
            # called set_decoder('native') so we expect the former,
            # but tolerate the latter for diagnostic visibility.
            points = decoded.get("points") if isinstance(decoded, dict) else None
            point_count = (
                int(points.shape[0]) if points is not None and hasattr(points, "shape")
                else int(decoded.get("point_count", 0)) if isinstance(decoded, dict)
                else 0
            )
            extents = None
            if points is not None and hasattr(points, "shape") and points.shape[0] > 0:
                import numpy as _np
                extents = {
                    "x": [float(_np.min(points[:, 0])), float(_np.max(points[:, 0]))],
                    "y": [float(_np.min(points[:, 1])), float(_np.max(points[:, 1]))],
                    "z": [float(_np.min(points[:, 2])), float(_np.max(points[:, 2]))],
                }
            entry = {
                "ts": time.time(),
                "point_count": point_count,
                "origin": list(payload.get("origin", [])) or None,
                "resolution": payload.get("resolution"),
                "src_size": payload.get("src_size"),
                "extents": extents,
            }
            with lock:
                state["frames"].append(entry)
                if state["first_points"] is None and points is not None and point_count > 0:
                    # Strip the points array out of the payload echo so
                    # the diagnostic dump is lightweight metadata only.
                    meta_only = {k: v for k, v in payload.items() if k != "data"}
                    state["first_payload"] = meta_only
                    state["first_points"] = points
        except Exception as e:
            state["errors"].append(f"{type(e).__name__}: {e}")

    # Switch the decoder to the lz4→numpy backend before subscribing.
    # Default is libvoxel which returns Three.js mesh data — unhelpful
    # for our use case. See docs/webrtc/README.md.
    log_banner("LIDAR PROBE — PHASE 1: WIRE UP", char="-", logger=logger)
    robot.conn.datachannel.set_decoder("native")
    logger.info("decoder set to: native (lz4 → numpy (N,3) point cloud)")

    robot.conn.datachannel.pub_sub.subscribe(RTC_TOPIC["ULIDAR_ARRAY"], on_lidar)
    logger.info(
        f"subscribed to {RTC_TOPIC['ULIDAR_ARRAY']!r} "
        f"(constant: ULIDAR_ARRAY)"
    )

    # The library author flags this as required for utlidar topics in
    # an inline comment. Probe verifies whether it's truly required vs
    # optional — if frames arrive before we call it, that's worth
    # knowing.
    pre_call_count_t0 = time.monotonic()
    await asyncio.sleep(LIDAR_PROBE_SETTLE_S)
    with lock:
        pre_call_frames = len(state["frames"])
    logger.info(
        f"after {LIDAR_PROBE_SETTLE_S:.0f}s settle (before disableTrafficSaving): "
        f"{pre_call_frames} frames received"
    )

    try:
        await robot.conn.datachannel.disableTrafficSaving(True)
        logger.info("disableTrafficSaving(True) accepted")
        traffic_call_ok = True
    except Exception as e:
        logger.warning(f"disableTrafficSaving(True) raised: {e!r}")
        traffic_call_ok = False

    log_banner(
        f"LIDAR PROBE — PHASE 2: SAMPLE ({duration_sec:.0f}s)",
        char="-",
        logger=logger,
    )
    sample_t0 = time.monotonic()
    last_log_t = sample_t0
    last_log_count = pre_call_frames
    while time.monotonic() - sample_t0 < duration_sec:
        await asyncio.sleep(1.0)
        now = time.monotonic()
        with lock:
            n = len(state["frames"])
            latest = state["frames"][-1] if state["frames"] else None
            err_count = len(state["errors"])
        dt = max(now - last_log_t, 1e-6)
        rate_hz = (n - last_log_count) / dt
        elapsed = now - sample_t0
        if latest is not None and latest.get("extents") is not None:
            ex = latest["extents"]
            logger.info(
                f"t={elapsed:5.1f}s  frames={n:4d}  rate={rate_hz:4.1f}Hz  "
                f"last pts={latest['point_count']:5d}  "
                f"x=[{ex['x'][0]:+5.1f},{ex['x'][1]:+5.1f}] "
                f"y=[{ex['y'][0]:+5.1f},{ex['y'][1]:+5.1f}] "
                f"z=[{ex['z'][0]:+5.1f},{ex['z'][1]:+5.1f}]  "
                f"errs={err_count}"
            )
        else:
            logger.info(
                f"t={elapsed:5.1f}s  frames={n:4d}  rate={rate_hz:4.1f}Hz  "
                f"no frames yet  errs={err_count}"
            )
        last_log_t = now
        last_log_count = n

    # Drop traffic saving back on (default state) so we don't leave the
    # robot in an unexpected mode after the probe ends.
    try:
        await robot.conn.datachannel.disableTrafficSaving(False)
        logger.info("disableTrafficSaving(False) — restored default")
    except Exception as e:
        logger.warning(f"disableTrafficSaving(False) raised: {e!r}")

    log_banner("LIDAR PROBE — PHASE 3: REPORT", char="-", logger=logger)
    with lock:
        frames = list(state["frames"])
        first_payload = state["first_payload"]
        first_points = state["first_points"]
        errors = list(state["errors"])

    summary = _summarize_lidar_probe(
        frames=frames,
        first_payload=first_payload,
        errors=errors,
        duration_sec=duration_sec,
        pre_call_frames=pre_call_frames,
        traffic_call_ok=traffic_call_ok,
    )

    summary_path = out_dir / "lidar_probe_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, default=str))
    logger.info(f"summary: {summary_path}")

    if first_points is not None and first_points.shape[0] > 0:
        npy_path = out_dir / "lidar_first_frame.npy"
        try:
            import numpy as _np
            _np.save(npy_path, first_points)
            logger.info(
                f"first frame: {npy_path} "
                f"(shape={first_points.shape}, dtype={first_points.dtype})"
            )
        except Exception as e:
            logger.warning(f"could not save .npy: {e!r}")

        _write_lidar_rasters(first_points, out_dir)
    else:
        logger.error(
            "no valid frame captured — cannot dump .npy or rasters. "
            "Probe failed to verify the lidar stream."
        )

    for line in _format_lidar_summary_banner(summary):
        logger.info(line)

    return summary


def _summarize_lidar_probe(
    *,
    frames: list,
    first_payload: dict | None,
    errors: list,
    duration_sec: float,
    pre_call_frames: int,
    traffic_call_ok: bool,
) -> dict:
    """Roll up the per-frame log into a single summary dict."""
    point_counts = [f["point_count"] for f in frames if f.get("point_count")]
    n_frames = len(frames)
    n_with_points = len(point_counts)
    mean_rate = n_frames / duration_sec if duration_sec > 0 else 0.0

    return {
        "duration_sec": duration_sec,
        "frame_count_total": n_frames,
        "frame_count_with_points": n_with_points,
        "mean_rate_hz": round(mean_rate, 2),
        "frames_before_disableTrafficSaving": pre_call_frames,
        "disableTrafficSaving_call_ok": traffic_call_ok,
        "point_count_stats": (
            {
                "min": int(min(point_counts)),
                "mean": int(statistics.mean(point_counts)),
                "max": int(max(point_counts)),
                "stdev": (
                    round(statistics.stdev(point_counts), 1)
                    if len(point_counts) > 1 else 0.0
                ),
            }
            if point_counts else None
        ),
        "first_frame_metadata": first_payload,
        "error_count": len(errors),
        "errors_sample": errors[:5],
    }


def _format_lidar_summary_banner(summary: dict) -> list:
    lines = ["=" * 60, "LIDAR PROBE SUMMARY", "=" * 60]
    lines.append(f"duration:            {summary['duration_sec']:.1f}s")
    lines.append(f"frames total:        {summary['frame_count_total']}")
    lines.append(f"frames w/ points:    {summary['frame_count_with_points']}")
    lines.append(f"mean rate:           {summary['mean_rate_hz']:.1f} Hz")
    lines.append(
        f"frames before disableTrafficSaving(True): "
        f"{summary['frames_before_disableTrafficSaving']}  "
        f"-> {'gate required' if summary['frames_before_disableTrafficSaving'] == 0 else 'gate optional'}"
    )
    pc = summary.get("point_count_stats")
    if pc:
        lines.append(
            f"point counts:        "
            f"min={pc['min']}  mean={pc['mean']}  max={pc['max']}  "
            f"stdev={pc['stdev']}"
        )
    meta = summary.get("first_frame_metadata") or {}
    if meta.get("origin"):
        lines.append(f"first-frame origin:  {meta['origin']}")
    if meta.get("resolution"):
        lines.append(f"first-frame res:     {meta['resolution']} m")
    if summary["error_count"]:
        lines.append(f"errors observed:     {summary['error_count']} (see summary.json)")
    lines.append("=" * 60)
    return lines


# ---------------------------------------------------------------------------
# Quick-look rasterizers — three projections to eyeball before committing
# to a single capture-time companion. See "Stage 2" plan in the README.
# ---------------------------------------------------------------------------

# Output bitmap size for the quick-look PNGs. Small enough to render
# instantly, large enough to read structure by eye.
_LIDAR_RASTER_PX = 480

# Range cap (meters) for the quick-look PNGs. Points beyond this are
# dropped before rasterization — keeps the 480px grid from being
# dominated by far returns.
_LIDAR_RASTER_RANGE_M = 8.0


def _write_lidar_rasters(points, out_dir: Path) -> None:
    """Render three quick-look PNGs from the first captured frame.

    These are diagnostic — the goal is "yes, the room is in there" not
    metric accuracy. The final capture-time companion bitmap will be
    designed once we know which projection reads best on real data.
    """
    try:
        import numpy as np
        from PIL import Image
    except ImportError as e:
        logger.warning(f"raster output skipped (missing dep): {e}")
        return

    pts = np.asarray(points)
    if pts.ndim != 2 or pts.shape[1] != 3:
        logger.warning(f"raster output skipped (bad shape: {pts.shape})")
        return

    # Cap to a sensible local range so far outliers don't dominate
    # auto-scaling. Lidar in world frame can have arbitrary origin
    # offsets — we re-center each projection around its own median for
    # interpretability.
    r = np.linalg.norm(pts[:, :2], axis=1)
    near = pts[r < _LIDAR_RASTER_RANGE_M]
    if near.shape[0] == 0:
        logger.warning("no points within range cap — rasters will be empty")
        return
    logger.info(
        f"raster input: {near.shape[0]} of {pts.shape[0]} points "
        f"within {_LIDAR_RASTER_RANGE_M:.0f}m"
    )

    bev_png = out_dir / "lidar_first_frame_bev.png"
    side_png = out_dir / "lidar_first_frame_side.png"
    front_png = out_dir / "lidar_first_frame_front.png"

    _save_density_raster(
        near, axis_h=0, axis_v=1, label="BEV (top-down: X forward, Y lateral)",
        path=bev_png,
    )
    _save_density_raster(
        near, axis_h=0, axis_v=2, label="SIDE (X forward, Z up)",
        path=side_png,
    )
    _save_density_raster(
        near, axis_h=1, axis_v=2, label="FRONT (Y lateral, Z up)",
        path=front_png,
    )


def _save_density_raster(
    pts, *, axis_h: int, axis_v: int, label: str, path: Path
) -> None:
    """Histogram2d-based density raster of `pts` projected onto two axes."""
    import numpy as np
    from PIL import Image

    h = pts[:, axis_h]
    v = pts[:, axis_v]
    # Symmetric square extent around the median so the grid origin
    # lands in the middle of the visible area regardless of world-frame
    # offset.
    h_med, v_med = float(np.median(h)), float(np.median(v))
    half = _LIDAR_RASTER_RANGE_M
    bins = _LIDAR_RASTER_PX
    H, _, _ = np.histogram2d(
        v - v_med, h - h_med,  # row = vertical axis, col = horizontal
        bins=bins,
        range=[[-half, half], [-half, half]],
    )
    # Log-compress + normalize to 8-bit so even sparse cells show.
    H = np.log1p(H)
    if H.max() > 0:
        H = (255.0 * H / H.max()).astype(np.uint8)
    else:
        H = H.astype(np.uint8)
    # Flip vertically so positive axis goes up in the rendered image.
    img = np.flipud(H)
    Image.fromarray(img).save(path)
    logger.info(f"raster: {path.name}  ({label})")
