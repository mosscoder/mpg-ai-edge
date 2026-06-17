"""Go2 leg-motor telemetry: per-joint temperature model, parsing, trend.

The Go2 publishes 12 motor states (4 legs × hip/thigh/calf) on the
``rt/lf/lowstate`` data-channel topic (see ``Go2Robot._on_low_state``),
alongside the battery BMS. The thigh ("shoulder") motors — indices
1/4/7/10 — do the standing + propulsion work and are the ones that
overheat under sustained survey load: the 2026-06-16 strip_2 collapse
hit 83 °C on the FL+RR thighs while the FR+RL diagonal stayed ~12 °C
cooler. This module surfaces those thighs explicitly so the health
banners can warn before a thermal cut-out.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

# 12 motors, ordered FR/FL/RR/RL × hip/thigh/calf (Unitree index convention).
LEG_NAMES = ("FR", "FL", "RR", "RL")
JOINT_NAMES = ("hip", "thigh", "calf")
# thigh ("shoulder") joints — index -> leg.
THIGH_INDEX = {1: "FR", 4: "FL", 7: "RR", 10: "RL"}


def motor_label(i: int) -> str:
    """e.g. 7 -> 'RR_thigh'."""
    return f"{LEG_NAMES[i // 3]}_{JOINT_NAMES[i % 3]}"


@dataclass
class MotorState:
    """One 12-motor snapshot parsed from a lowstate ``motor_state`` list."""

    temps: list[int]     # 12 temperatures, °C, indexed 0-11
    lost: list[int]      # 12 comm-error ("lost") counters
    timestamp: float

    @property
    def thighs(self) -> dict[str, int]:
        """{'FR': t, 'FL': t, 'RR': t, 'RL': t} thigh temperatures."""
        return {leg: self.temps[i] for i, leg in THIGH_INDEX.items()}

    @property
    def max_thigh(self) -> int:
        return max(self.thighs.values())

    @property
    def hottest_thigh(self) -> str:
        """Label of the hottest thigh, e.g. 'RR_thigh'."""
        idx = max(THIGH_INDEX, key=lambda i: self.temps[i])
        return motor_label(idx)

    @property
    def hot_diagonal(self) -> str:
        """Which diagonal pair is hotter — 'FL+RR' or 'FR+RL'."""
        th = self.thighs
        return "FL+RR" if (th["FL"] + th["RR"]) >= (th["FR"] + th["RL"]) else "FR+RL"

    @property
    def max_other(self) -> int:
        """Hottest of the 8 non-thigh motors (hips + calves)."""
        return max(t for i, t in enumerate(self.temps) if i not in THIGH_INDEX)

    @property
    def max_any(self) -> int:
        return max(self.temps)

    @property
    def comm_lost(self) -> int:
        return sum(self.lost)


def parse_motors(data: dict, timestamp: float) -> MotorState | None:
    """Build a MotorState from a lowstate ``message['data']`` dict, or None.

    ``data['motor_state']`` is a list of 12 dicts, each carrying
    ``temperature`` and ``lost`` (comm-error count). Returns None if the
    list is missing or short (a partial frame).
    """
    ms = data.get("motor_state")
    if not isinstance(ms, list) or len(ms) < 12:
        return None
    try:
        temps = [int(ms[i].get("temperature") or 0) for i in range(12)]
        lost = [int(ms[i].get("lost") or 0) for i in range(12)]
    except (AttributeError, TypeError, ValueError):
        return None
    return MotorState(temps=temps, lost=lost, timestamp=timestamp)


class MotorTempTrend:
    """Rolling max-thigh temperature → °C/min slope.

    Mirrors ``battery.RuntimeEstimator``: holds recent (t, max_thigh)
    samples over ``WINDOW_S`` and fits a simple endpoint slope. Returns
    None until a meaningful span exists, so the first reading isn't noise
    off two near-equal samples.
    """

    WINDOW_S = 120.0
    MIN_SPAN_S = 30.0

    def __init__(self) -> None:
        self._hist: deque[tuple[float, int]] = deque()

    def update(self, ms: MotorState) -> None:
        self._hist.append((ms.timestamp, ms.max_thigh))
        while len(self._hist) > 2 and ms.timestamp - self._hist[0][0] > self.WINDOW_S:
            self._hist.popleft()

    def rate_per_min(self) -> float | None:
        if len(self._hist) < 2:
            return None
        (t0, v0), (t1, v1) = self._hist[0], self._hist[-1]
        span = t1 - t0
        if span < self.MIN_SPAN_S:
            return None
        return (v1 - v0) / (span / 60.0)
