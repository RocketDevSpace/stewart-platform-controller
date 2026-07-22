"""
control/setpoint.py

SetpointArbiter — single owner of the ball target setpoint.

Holds the GUI-facing *manual* target plus an optional *override* target
(used by autotune to drive step-test legs). ``active`` is what the PD loop
should chase: the override when one is set, the manual target otherwise.

Every mutation stamps ``last_change_time`` via the injected clock, so
downstream hold gates (auto-trim's target-hold window) see target motion
regardless of which source moved it.
"""
from collections.abc import Callable


class SetpointArbiter:
    def __init__(self, x0: float, y0: float, clock: Callable[[], float]) -> None:
        self._clock = clock
        self._manual: tuple[float, float] = (float(x0), float(y0))
        self._override: tuple[float, float] | None = None
        self._last_change_time: float | None = None

    def set_manual(self, x_mm: float, y_mm: float) -> None:
        self._manual = (float(x_mm), float(y_mm))
        self._last_change_time = self._clock()

    def set_override(self, x_mm: float, y_mm: float) -> None:
        self._override = (float(x_mm), float(y_mm))
        self._last_change_time = self._clock()

    def clear_override(self) -> None:
        self._override = None
        self._last_change_time = self._clock()

    @property
    def manual(self) -> tuple[float, float]:
        return self._manual

    @property
    def active(self) -> tuple[float, float]:
        return self._override if self._override is not None else self._manual

    @property
    def last_change_time(self) -> float | None:
        return self._last_change_time

    @property
    def override_active(self) -> bool:
        return self._override is not None
