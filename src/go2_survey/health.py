"""Combined battery + motor "health" banners for the mission narrative.

Bridges :mod:`battery` (BMS) and :mod:`motors` (12 leg-motor temps) into the
human-readable banners the operator scans waypoint-to-waypoint:

  - HEALTH AT START / END  — boxed, once each, with a warm-start warning.
  - per-leg HEALTH CHECK   — boxed, at every corner (the waypoint boundary):
    battery + all four thighs + per-leg rise + °C/min trend + status.
  - periodic HEALTH line   — one line every ``interval`` s between corners.
  - THERMAL alert          — a loud box the instant a thigh crosses a limit.

The thigh ("shoulder") motors are the thermal failure mode (the 2026-06-16
strip_2 collapse cut out at 83 °C), so they drive the status + alerts.

All banners flow through ``go2_survey.health`` → main.log + console (not
filtered). Detailed per-sample battery still goes to battery.log via
``battery.telemetry_logger``; the raw lowstate frames are routed to motor.log
by the LowStateOnlyFilter (see logging_utils / cli.setup_logging).
"""

from __future__ import annotations

import asyncio
import logging

from go2_survey import battery
from go2_survey.battery import RuntimeEstimator
from go2_survey.logging_utils import log_banner
from go2_survey.motors import LEG_NAMES, MotorTempTrend

banner_logger = logging.getLogger("go2_survey.health")

# Markers — kept as constants so a terminal that mangles them can be served
# ASCII swaps in one place (e.g. RISE="^", DONE="*", GE=">=", BOX="=").
RISE = "▲"
FALL = "▼"
DONE = "✓"
DOT = "·"
SEP = "│"
GE = "≥"
LE = "≤"
CAUTION = "[!]"
DANGER = "[STOP]"
_LEG_BOX = "═"
_FRAME_BOX = "─"
_BLOCK_W = 58


def _emit_block(title: str, body_lines: list[str], char: str, level: str = "info") -> None:
    """Boxed banner: centered title in the top border, body lines, bottom border."""
    log_fn = getattr(banner_logger, level)
    log_fn(f" {title} ".center(_BLOCK_W, char))
    for ln in body_lines:
        log_fn(ln)
    log_fn(char * _BLOCK_W)


def _thighs_line(ms) -> str:
    th = ms.thighs
    hot = ms.hottest_thigh.split("_")[0]
    cells = [f"{leg}{th[leg]}" + (RISE if leg == hot else "") for leg in LEG_NAMES]
    return f"  THIGHS   {' '.join(cells)} °C (hot diag {ms.hot_diagonal})"


def _status_line(mon, ms) -> str:
    lvl = mon.classify(ms.max_thigh)
    if lvl == "danger":
        return f"  STATUS   {DANGER} {ms.hottest_thigh} {GE} {mon.danger_c}°C ceiling — COOL DOWN"
    if lvl == "caution":
        return f"  STATUS   {CAUTION} {ms.hottest_thigh} {GE} {mon.caution_c}°C — watch the climb"
    return f"  STATUS   ok {SEP} thighs {LE} {mon.caution_c}°C"


def _battery_line(mon, bs) -> str:
    eta = mon.eta()
    eta_s = f"~{eta:.0f}min" if eta is not None else "est…"
    return f"  BATTERY  {bs.soc}% {SEP} {bs.current_a:+.1f}A {SEP} {eta_s}"


# ---- pure formatters -------------------------------------------------------

def format_start_banner_lines(mon, bs, ms) -> list[str]:
    lines = []
    if bs is not None:
        lines.append(f"  BATTERY  {bs.soc}% {SEP} {bs.voltage:.1f}V {SEP} batt {bs.batt_temp_c}°C")
    if ms is not None:
        lines.append(_thighs_line(ms))
        lvl = mon.classify(ms.max_thigh)
        if lvl == "danger":
            lines.append(f"  STATUS   {DANGER} HOT START: {ms.hottest_thigh} {ms.max_thigh}°C — COOL DOWN before running")
        elif lvl == "caution":
            lines.append(f"  STATUS   {CAUTION} WARM START: {ms.hottest_thigh} {ms.max_thigh}°C — cooldown advised")
        else:
            lines.append(f"  STATUS   ok {SEP} thighs {LE} {mon.caution_c}°C — clear to run")
    return lines or ["  (telemetry not yet available)"]


def format_leg_banner_lines(mon, bs, ms, leg_idx: int, n_legs: int) -> list[str]:
    lines = []
    if bs is not None:
        delta = ""
        if mon._last_leg is not None:
            d = bs.soc - mon._last_leg["soc"]
            delta = f" {SEP} {d:+d}% since leg{mon._last_leg['idx']:02d}"
        lines.append(_battery_line(mon, bs) + delta)
    else:
        lines.append("  BATTERY  (no telemetry)")
    if ms is not None:
        lines.append(_thighs_line(ms))
        leg_d = ""
        if mon._last_leg is not None:
            dd = ms.max_thigh - mon._last_leg["max_thigh"]
            leg_d = f" {RISE}+{dd}° this leg" if dd > 0 else f" {dd:+d}° this leg"
        rate = mon.rate()
        rate_s = f" ({rate:+.1f}°/min)" if rate is not None else ""
        lines.append(f"  HOTTEST  {ms.hottest_thigh} {ms.max_thigh}°C{leg_d}{rate_s}")
        lines.append(f"  OTHER    8 motors {LE} {ms.max_other}°C")
        lines.append(_status_line(mon, ms))
    else:
        lines.append("  MOTORS   (no telemetry)")
    return lines


def format_end_banner_lines(mon, bs, start_soc) -> list[str]:
    lines = []
    if bs is not None:
        d = f" {SEP} {bs.soc - start_soc:+d}% over run" if start_soc is not None else ""
        lines.append(f"  BATTERY  {bs.soc}% {SEP} {bs.voltage:.1f}V{d}")
    if mon._peak_thigh:
        lines.append(f"  PEAK     thigh {mon._peak_thigh}°C ({mon._peak_thigh_label}) this run")
    return lines or ["  (telemetry unavailable at teardown)"]


def format_periodic_line(mon, bs, ms) -> str:
    if bs is not None:
        eta = mon.eta()
        eta_s = f"~{eta:.0f}min" if eta is not None else "est…"
        bcell = f"batt {bs.soc}% {bs.current_a:+.1f}A {eta_s}"
    else:
        bcell = "batt (n/a)"
    if ms is not None:
        rate = mon.rate()
        if rate is None:
            rcell = ""
        else:
            arrow = RISE if rate > 0 else FALL if rate < 0 else ""
            rcell = f" {arrow}{abs(rate):.1f}°/min"
        mark = {"danger": f"  {DANGER}", "caution": f"  {CAUTION}", "ok": ""}[mon.classify(ms.max_thigh)]
        mcell = f"motors {ms.max_thigh}°C max {ms.hottest_thigh}{rcell}"
    else:
        mcell = "motors (n/a)"
        mark = ""
    return f"HEALTH  {bcell} {DOT} {mcell}{mark}"


def format_alert_message(mon, ms) -> str:
    rate = mon.rate()
    rs = f" ({rate:+.1f}°/min)" if rate is not None else ""
    if mon.classify(ms.max_thigh) == "danger":
        return f"{DANGER} THERMAL DANGER: {ms.hottest_thigh} {ms.max_thigh}°C{rs} — COLLAPSE RISK, COOL DOWN"
    return f"{CAUTION} THERMAL CAUTION: {ms.hottest_thigh} {ms.max_thigh}°C{rs} — approaching ceiling"


# ---- monitor ---------------------------------------------------------------

class HealthMonitor:
    """Owns the rolling trend, runtime estimate, per-leg deltas, alert state,
    and run-peak. Single instance shared by the per-leg banner (navigator) and
    the periodic logger task (mission_runner) so both read the same numbers.
    """

    def __init__(self, caution_c: int = 70, danger_c: int = 78) -> None:
        self.caution_c = caution_c
        self.danger_c = danger_c
        self._trend = MotorTempTrend()
        self._runtime = RuntimeEstimator()
        self._last_leg: dict | None = None
        self._alert_level = "ok"
        self._peak_thigh = 0
        self._peak_thigh_label = "?"

    def sample(self, bs, ms) -> None:
        if ms is not None:
            self._trend.update(ms)
            self._note_peak(ms)
        if bs is not None:
            self._runtime.update(bs)

    def _note_peak(self, ms) -> None:
        if ms is not None and ms.max_thigh > self._peak_thigh:
            self._peak_thigh = ms.max_thigh
            self._peak_thigh_label = ms.hottest_thigh

    def classify(self, max_thigh: int) -> str:
        if max_thigh >= self.danger_c:
            return "danger"
        if max_thigh >= self.caution_c:
            return "caution"
        return "ok"

    def eta(self):
        return self._runtime.eta_min()

    def rate(self):
        return self._trend.rate_per_min()

    def poll_alert(self, ms) -> str | None:
        """Return 'caution'/'danger' if max_thigh just crossed UP a threshold
        since the last poll; None otherwise. Resets on cool-down so re-crossing
        re-alerts."""
        lvl = self.classify(ms.max_thigh)
        order = {"ok": 0, "caution": 1, "danger": 2}
        prev, self._alert_level = self._alert_level, lvl
        return lvl if order[lvl] > order[prev] else None

    # emit wrappers (read robot state, format, log) --------------------------

    def emit_start(self, robot):
        bs = robot.get_battery_state()
        ms = robot.get_motor_state()
        self.sample(bs, ms)
        _emit_block("HEALTH AT START", format_start_banner_lines(self, bs, ms), _FRAME_BOX)
        return bs

    def emit_leg(self, robot, leg_idx: int, n_legs: int) -> None:
        bs = robot.get_battery_state()
        ms = robot.get_motor_state()
        self._note_peak(ms)
        if bs is None and ms is None:
            return
        lines = format_leg_banner_lines(self, bs, ms, leg_idx, n_legs)
        _emit_block(f"LEG {leg_idx}/{n_legs} {DONE} {DOT} HEALTH CHECK", lines, _LEG_BOX)
        if bs is not None and ms is not None:
            self._last_leg = {"idx": leg_idx, "soc": bs.soc, "max_thigh": ms.max_thigh}

    def emit_end(self, robot, start_soc) -> None:
        bs = robot.get_battery_state()
        _emit_block("HEALTH AT END", format_end_banner_lines(self, bs, start_soc), _FRAME_BOX)


async def run_health_logger(robot, health: HealthMonitor, interval: float = 10.0) -> None:
    """Every ``interval`` s: sample, write the detailed battery.log line, fire a
    THERMAL alert on a fresh threshold crossing, and emit the periodic HEALTH
    line. Runs as a concurrent task for the mission's duration; cancel to stop.
    """
    while True:
        await asyncio.sleep(interval)
        bs = robot.get_battery_state()
        ms = robot.get_motor_state()
        health.sample(bs, ms)
        if bs is not None:
            battery.telemetry_logger.info(battery.format_telemetry(bs))
        if ms is not None:
            alert = health.poll_alert(ms)
            if alert is not None:
                log_banner(
                    format_alert_message(health, ms),
                    level="error" if alert == "danger" else "warning",
                    logger=banner_logger,
                )
        if bs is not None or ms is not None:
            banner_logger.info(format_periodic_line(health, bs, ms))
