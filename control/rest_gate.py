"""
control/rest_gate.py

RestGate — near-target rest mode state machine (perf pass).

When the ball has sat inside a small radius at low speed for a sustained
hold, the controller stops chasing PD noise and rests the platform at
level + trim. The gate is deliberately asymmetric:

  Entry (active/holding -> resting) is CONSERVATIVE: raw radius must be
  inside REST_ENTER_RADIUS_MM and RestGate's OWN low-passed speed must be
  under REST_ENTER_SPEED_MM_S continuously for REST_ENTER_HOLD_S. Any
  violation resets the hold timer.

  Exit (resting -> active) is INSTANT: checked first, on the RAW per-call
  radius and instantaneous speed (no LPF, no timer), so a real disturbance
  restores full PD authority on the same control cycle.

Pure Python, no Qt, injected clock. One update() per control cycle.
"""
from collections.abc import Callable

from settings import (
    REST_ENTER_HOLD_S,
    REST_ENTER_RADIUS_MM,
    REST_ENTER_SPEED_MM_S,
    REST_EXIT_RADIUS_MM,
    REST_EXIT_SPEED_MM_S,
    REST_SPEED_LPF_ALPHA,
)

STATE_ACTIVE = "active"
STATE_HOLDING = "holding"
STATE_RESTING = "resting"


class RestGate:
    def __init__(self, clock: Callable[[], float], enabled: bool = True) -> None:
        self._clock = clock
        self.enabled = bool(enabled)

        self.enter_radius_mm = float(REST_ENTER_RADIUS_MM)
        self.exit_radius_mm = float(REST_EXIT_RADIUS_MM)
        self.enter_speed_mm_s = float(REST_ENTER_SPEED_MM_S)
        self.exit_speed_mm_s = float(REST_EXIT_SPEED_MM_S)
        self.enter_hold_s = float(REST_ENTER_HOLD_S)
        self.speed_lpf_alpha = float(REST_SPEED_LPF_ALPHA)

        self._state: str = STATE_ACTIVE
        self._hold_start: float | None = None
        self._hold_elapsed_s = 0.0
        # RestGate's OWN speed EMA (independent of AutoTrim's settle LPFs).
        self._speed_lpf: float | None = None

    # ---------------------------
    # Read-only state (telemetry)
    # ---------------------------

    @property
    def state(self) -> str:
        return self._state

    @property
    def resting(self) -> bool:
        return self._state == STATE_RESTING

    @property
    def speed_lpf_mm_s(self) -> float:
        return 0.0 if self._speed_lpf is None else float(self._speed_lpf)

    @property
    def hold_elapsed_s(self) -> float:
        return self._hold_elapsed_s

    # ---------------------------
    # State machine
    # ---------------------------

    def reset(self) -> None:
        """Back to active: hold timer and speed LPF cleared.

        Called on target changes, tracking loss / motion reset, and mode
        changes — any event that invalidates the "ball has been quietly
        parked here" evidence.
        """
        self._state = STATE_ACTIVE
        self._hold_start = None
        self._hold_elapsed_s = 0.0
        self._speed_lpf = None

    def update(
        self,
        radius_mm: float,
        speed_inst_mm_s: float,
        now: float | None = None,
    ) -> str:
        """One control-cycle step; returns the new state string."""
        t = self._clock() if now is None else float(now)
        radius = float(radius_mm)
        speed_inst = float(speed_inst_mm_s)

        if not self.enabled:
            self.reset()
            return self._state

        # Speed EMA (alpha = weight on the previous value; first sample seeds).
        if self._speed_lpf is None:
            self._speed_lpf = speed_inst
        else:
            a = self.speed_lpf_alpha
            self._speed_lpf = a * self._speed_lpf + (1.0 - a) * speed_inst

        if self._state == STATE_RESTING:
            # Exit check FIRST, on RAW values — takes effect this same call.
            if radius > self.exit_radius_mm or speed_inst > self.exit_speed_mm_s:
                self._state = STATE_ACTIVE
                self._hold_start = None
                self._hold_elapsed_s = 0.0
            return self._state

        # Entry: raw radius small AND low-passed speed slow, sustained.
        entry_ok = (
            radius <= self.enter_radius_mm
            and self._speed_lpf <= self.enter_speed_mm_s
        )
        if not entry_ok:
            self._state = STATE_ACTIVE
            self._hold_start = None
            self._hold_elapsed_s = 0.0
            return self._state

        if self._hold_start is None:
            self._hold_start = t
        self._hold_elapsed_s = max(0.0, t - self._hold_start)
        # 1e-9 epsilon: N accumulated float dt steps summing to the hold
        # duration (e.g. 5 x 0.1 s) land a few ULP under it — do not make
        # the caller pay a whole extra cycle for float rounding.
        if self._hold_elapsed_s >= self.enter_hold_s - 1e-9:
            self._state = STATE_RESTING
            self._hold_start = None
        else:
            self._state = STATE_HOLDING
        return self._state
