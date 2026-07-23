"""
control/trim_store.py

TrimStore — the persistent roll/pitch level offsets, and nothing else.

This replaced control/auto_trim.py in the 2026-07-23 I-term rework. The
old module was a gated position-error integrator (settle gates, hold
timers, target-hold) that corrected in bursts and caused a family of rig
failures (burst limit cycle, path-following deadlock, home-cal stalls).
Live leveling now happens continuously in the PIDCore integral term; trim
is a pure STORE that the integral gets folded into for persistence:

  - manual slider writes land here (set_offsets)
  - Save Trim / home-cal completion fold the current integral in
    (fold — the offsets absorb what the integral learned, the caller
    zeros the integral via PIDCore.take_integrator, net output change 0)
  - reset restores the settings defaults

Single-writer rule: these offsets are mutated only here (set_offsets,
fold, reset). The integral state is PIDCore's alone. BallController's
fold_integrator_into_trim() is the only bridge.

Home calibration here is only the flag + start timestamp; the
convergence watching lives in BallController (it alone sees the
integral and the ball).
"""
import time
from collections.abc import Callable
from typing import Any

from settings import (
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
    TRIM_LIMIT_DEG,
)


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min(value, max_val), min_val)


class TrimStore:
    def __init__(
        self,
        roll_offset: float = MANUAL_ROLL_TRIM_DEG,
        pitch_offset: float = MANUAL_PITCH_TRIM_DEG,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        self._clock = clock
        self._roll_offset = float(roll_offset)
        self._pitch_offset = float(pitch_offset)
        self.home_calibration_active = False
        self._home_calibration_start_ts = 0.0

    # ---------------------------
    # Offsets
    # ---------------------------

    @property
    def roll_offset(self) -> float:
        return self._roll_offset

    @property
    def pitch_offset(self) -> float:
        return self._pitch_offset

    def set_offsets(self, roll_offset_deg: float, pitch_offset_deg: float) -> None:
        self._roll_offset = _clamp(
            float(roll_offset_deg), -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG
        )
        self._pitch_offset = _clamp(
            float(pitch_offset_deg), -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG
        )

    def fold(self, roll_delta_deg: float, pitch_delta_deg: float) -> tuple[float, float]:
        """Absorb an integral contribution into the stored trim.

        Returns the new (roll, pitch) offsets — the values a caller
        should persist.
        """
        self._roll_offset = _clamp(
            self._roll_offset + float(roll_delta_deg),
            -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG,
        )
        self._pitch_offset = _clamp(
            self._pitch_offset + float(pitch_delta_deg),
            -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG,
        )
        return self._roll_offset, self._pitch_offset

    def reset(self) -> None:
        """Restore the settings defaults (manual/reset-button path)."""
        self._roll_offset = float(MANUAL_ROLL_TRIM_DEG)
        self._pitch_offset = float(MANUAL_PITCH_TRIM_DEG)

    # ---------------------------
    # Home calibration flag
    # ---------------------------

    def start_home_calibration(self) -> None:
        self.home_calibration_active = True
        self._home_calibration_start_ts = self._clock()

    def cancel_home_calibration(self) -> None:
        self.home_calibration_active = False
        self._home_calibration_start_ts = 0.0

    @property
    def home_calibration_elapsed_s(self) -> float:
        if not self.home_calibration_active:
            return 0.0
        if self._home_calibration_start_ts <= 0.0:
            return 0.0
        return max(0.0, self._clock() - self._home_calibration_start_ts)

    # ---------------------------
    # Telemetry (terms-dict keys)
    # ---------------------------

    def telemetry(self, integral_enabled: bool) -> dict[str, Any]:
        return {
            "auto_trim_enabled": integral_enabled,
            "home_calibration_active": self.home_calibration_active,
            "home_calibration_elapsed_s": self.home_calibration_elapsed_s,
        }
