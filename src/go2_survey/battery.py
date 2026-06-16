"""Go2 battery (BMS) telemetry: state model, formatting, and periodic logging.

The Go2 publishes battery state on the ``rt/lf/lowstate`` data-channel topic
(see ``Go2Robot._on_low_state``). This module turns one ``message['data']``
into a :class:`BatteryState`, formats the two log lines (a detailed one for
the dedicated ``battery.log`` and a concise banner for ``main.log``), and runs
the per-interval logger task that emits both during a mission.

Two loggers, mirroring the GPS telemetry split (see logging_utils):
  - ``go2_survey.battery.telemetry`` → routed to ``battery.log`` only (detailed).
  - ``go2_survey.battery``           → flows to ``main.log`` + console (banner).
"""

from __future__ import annotations

import asyncio
import logging
from collections import deque
from dataclasses import dataclass

telemetry_logger = logging.getLogger("go2_survey.battery.telemetry")
banner_logger = logging.getLogger("go2_survey.battery")


@dataclass
class BatteryState:
    """One battery snapshot parsed from a lowstate ``bms_state`` + ``power_v``."""

    soc: int            # state of charge, 0-100 %
    voltage: float      # pack voltage, V (lowstate power_v)
    current_a: float    # pack current, A (negative = discharging)
    batt_temp_c: int    # battery NTC, °C (max of bq_ntc)
    mcu_temp_c: int     # MCU NTC, °C (max of mcu_ntc)
    cycles: int         # charge-cycle count
    timestamp: float    # wall clock of the sample


def _max_temp(v) -> int:
    """bq_ntc / mcu_ntc arrive as a list ([t1, t2]) or occasionally a scalar."""
    if isinstance(v, (list, tuple)):
        return int(max(v)) if v else 0
    try:
        return int(v)
    except (TypeError, ValueError):
        return 0


def parse_bms(data: dict, timestamp: float) -> BatteryState | None:
    """Build a BatteryState from a lowstate ``message['data']`` dict, or None.

    ``data['bms_state']`` carries soc / current / cycle / bq_ntc / mcu_ntc;
    pack voltage is ``data['power_v']`` (alongside bms_state, not inside it).
    """
    bms = data.get("bms_state")
    if not isinstance(bms, dict) or bms.get("soc") is None:
        return None
    return BatteryState(
        soc=int(bms["soc"]),
        voltage=float(data.get("power_v") or 0.0),
        current_a=float(bms.get("current") or 0) / 1000.0,
        batt_temp_c=_max_temp(bms.get("bq_ntc", 0)),
        mcu_temp_c=_max_temp(bms.get("mcu_ntc", 0)),
        cycles=int(bms.get("cycle") or 0),
        timestamp=timestamp,
    )


def format_telemetry(bs: BatteryState) -> str:
    """Detailed line for battery.log."""
    direction = "discharging" if bs.current_a < 0 else "charging"
    return (
        f"SOC {bs.soc:>3}% | {bs.voltage:.2f}V | {bs.current_a:+.1f}A "
        f"({direction}) | batt {bs.batt_temp_c}°C | mcu {bs.mcu_temp_c}°C "
        f"| cycles {bs.cycles}"
    )


def format_banner(bs: BatteryState, eta_min: float | None) -> str:
    """Concise battery line for main.log (interleaved with the nav banners)."""
    if eta_min is not None:
        tail = f"~{eta_min:.0f} min left @ current draw"
    else:
        tail = "estimating runtime…"
    return f"BATTERY {bs.soc}% | {bs.current_a:+.1f}A | {tail}"


def format_start_banner(bs: BatteryState) -> str:
    return f"BATTERY AT START: {bs.soc}% | {bs.voltage:.1f}V | {bs.batt_temp_c}°C"


def format_end_banner(bs: BatteryState, start_soc: int | None) -> str:
    delta = f" ({bs.soc - start_soc:+d}% over run)" if start_soc is not None else ""
    return f"BATTERY AT END: {bs.soc}% | {bs.voltage:.1f}V | {bs.batt_temp_c}°C{delta}"


class RuntimeEstimator:
    """Rolling SOC-drain → minutes-remaining estimate.

    Holds recent (t, soc) samples over a ``WINDOW_S`` window and divides the
    current SOC by the observed drain rate. Returns None until there's a
    measurable drop over a meaningful span, so the first ~minute reads
    "estimating…" rather than a wild number off two near-equal samples.
    """

    WINDOW_S = 180.0
    MIN_SPAN_S = 60.0
    MIN_DROP_PCT = 1

    def __init__(self) -> None:
        self._hist: deque[tuple[float, int]] = deque()

    def update(self, bs: BatteryState) -> None:
        self._hist.append((bs.timestamp, bs.soc))
        while len(self._hist) > 2 and bs.timestamp - self._hist[0][0] > self.WINDOW_S:
            self._hist.popleft()

    def eta_min(self) -> float | None:
        if len(self._hist) < 2:
            return None
        (t0, s0), (t1, s1) = self._hist[0], self._hist[-1]
        span, drop = t1 - t0, s0 - s1
        if span < self.MIN_SPAN_S or drop < self.MIN_DROP_PCT:
            return None
        rate_per_min = drop / (span / 60.0)
        if rate_per_min <= 0:
            return None
        return s1 / rate_per_min


async def run_battery_logger(robot, interval: float = 10.0) -> None:
    """Every ``interval`` s, emit the detailed battery.log line + the concise
    main.log banner. Runs as a concurrent task for the mission's duration;
    cancel it to stop. Ticks that have no battery sample yet are skipped.
    """
    est = RuntimeEstimator()
    while True:
        # Sleep first: the caller already emitted the start banner + an
        # initial battery.log sample at t=0, so the first periodic line
        # belongs at t=interval, not immediately beside the start banner.
        await asyncio.sleep(interval)
        bs = robot.get_battery_state()
        if bs is not None:
            est.update(bs)
            telemetry_logger.info(format_telemetry(bs))
            banner_logger.info(format_banner(bs, est.eta_min()))
