"""Go2 battery (BMS) telemetry: state model + the detailed battery.log line.

The Go2 publishes battery state on the ``rt/lf/lowstate`` data-channel topic
(see ``Go2Robot._on_low_state``). This module turns one ``message['data']``
into a :class:`BatteryState` and formats the detailed per-interval line routed
to ``battery.log`` via ``go2_survey.battery.telemetry``. The operator-facing
banners (start / per-leg / periodic / thermal alert) live in :mod:`health`,
which combines this BMS state with the motor temperatures from :mod:`motors`;
:class:`RuntimeEstimator` here backs the runtime-remaining figure they show.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass

telemetry_logger = logging.getLogger("go2_survey.battery.telemetry")


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
