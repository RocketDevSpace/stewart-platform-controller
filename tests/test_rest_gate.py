"""
Unit tests for control/rest_gate.py — the near-target rest mode gate.

Rules under test:
- Entry requires radius AND (LPF) speed inside the enter thresholds
  continuously for REST_ENTER_HOLD_S ("holding" while the timer runs).
- Any violation mid-hold resets the timer (a single speed blip is enough).
- Hysteresis: radius between enter (6) and exit (10) neither enters nor
  exits — the current state is preserved.
- Exit is same-call and uses RAW values: radius > 10 or speed > 25 flips
  resting -> active on the very update that observes it.
- reset() returns to active from any state and clears timer + LPF.
"""
import pytest

from control.rest_gate import RestGate
from settings import (
    REST_ENTER_HOLD_S,
    REST_ENTER_RADIUS_MM,
    REST_EXIT_RADIUS_MM,
    REST_EXIT_SPEED_MM_S,
)


class _FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.now = float(start)

    def advance(self, dt_s: float) -> None:
        self.now += float(dt_s)

    def __call__(self) -> float:
        return self.now


def _make_gate(clock: _FakeClock, enabled: bool = True) -> RestGate:
    return RestGate(clock=clock, enabled=enabled)


def _drive_to_rest(gate: RestGate, clock: _FakeClock, dt: float = 0.1) -> str:
    """Feed quiet center frames until the gate rests (bounded)."""
    state = gate.update(0.0, 0.0)
    for _ in range(20):
        if state == "resting":
            return state
        clock.advance(dt)
        state = gate.update(0.0, 0.0)
    return state


class TestEntry:
    def test_enter_only_after_sustained_hold(self) -> None:
        clock = _FakeClock(10.0)
        gate = _make_gate(clock)

        # First qualifying frame starts the hold timer: "holding", not resting.
        assert gate.update(0.0, 0.0) == "holding"

        # Sub-hold steps: stays "holding" strictly before the 0.5 s mark.
        steps_before = int(REST_ENTER_HOLD_S / 0.1) - 1  # 0.1..0.4 s
        for _ in range(steps_before):
            clock.advance(0.1)
            assert gate.update(0.0, 0.0) == "holding"

        # Crossing the hold threshold flips to "resting".
        clock.advance(0.1)
        assert gate.update(0.0, 0.0) == "resting"
        assert gate.resting is True

    def test_speed_blip_mid_hold_resets_timer(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock)

        gate.update(0.0, 0.0)
        for _ in range(3):  # 0.3 s of clean hold
            clock.advance(0.1)
            assert gate.update(0.0, 0.0) == "holding"

        # One-frame speed blip: LPF jumps (0.4 * 200 = 80 mm/s) -> violation.
        clock.advance(0.1)
        assert gate.update(0.0, 200.0) == "active"

        # The LPF must decay back under the enter threshold before the hold
        # can restart; without the blip we would have rested at t=0.5 s.
        states = []
        for _ in range(4):
            clock.advance(0.1)
            states.append(gate.update(0.0, 0.0))
        # 80 -> 48 -> 28.8 -> 17.28 -> 10.37: three actives, then holding.
        assert states == ["active", "active", "active", "holding"]

        # Full hold period required again from the restart.
        for _ in range(4):
            clock.advance(0.1)
            assert gate.update(0.0, 0.0) == "holding"
        clock.advance(0.1)
        assert gate.update(0.0, 0.0) == "resting"

    def test_radius_violation_mid_hold_resets_timer(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock)
        gate.update(0.0, 0.0)
        for _ in range(3):
            clock.advance(0.1)
            gate.update(0.0, 0.0)
        clock.advance(0.1)
        assert gate.update(REST_ENTER_RADIUS_MM + 0.5, 0.0) == "active"
        assert gate.hold_elapsed_s == 0.0


class TestHysteresisBand:
    def test_band_radius_does_not_enter_from_active(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock)
        # 8 mm sits between enter (6) and exit (10): never enters.
        for _ in range(20):
            assert gate.update(8.0, 0.0) == "active"
            clock.advance(0.1)

    def test_band_radius_does_not_exit_from_resting(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock)
        assert _drive_to_rest(gate, clock) == "resting"
        # Ball drifts to 8 mm: still inside the exit radius -> keeps resting.
        for _ in range(20):
            clock.advance(0.1)
            assert gate.update(8.0, 0.0) == "resting"


class TestSameCallExit:
    def test_radius_past_exit_threshold_exits_same_call(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock)
        assert _drive_to_rest(gate, clock) == "resting"
        clock.advance(0.1)
        assert gate.update(REST_EXIT_RADIUS_MM + 0.1, 0.0) == "active"

    def test_speed_past_exit_threshold_exits_same_call(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock)
        assert _drive_to_rest(gate, clock) == "resting"
        # Raw instantaneous speed is checked (the LPF would still be low:
        # 0.4 * 25.1 = 10.04 mm/s) — exit must not wait for the filter.
        clock.advance(0.1)
        assert gate.update(0.0, REST_EXIT_SPEED_MM_S + 0.1) == "active"
        assert gate.speed_lpf_mm_s < REST_EXIT_SPEED_MM_S


class TestReset:
    def test_reset_from_holding(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock)
        assert gate.update(0.0, 10.0) == "holding"
        gate.reset()
        assert gate.state == "active"
        assert gate.hold_elapsed_s == 0.0
        assert gate.speed_lpf_mm_s == 0.0

    def test_reset_from_resting_requires_full_rehold(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock)
        assert _drive_to_rest(gate, clock) == "resting"
        gate.reset()
        assert gate.state == "active"
        # Full hold period is required again after a reset.
        clock.advance(0.1)
        assert gate.update(0.0, 0.0) == "holding"
        for _ in range(4):
            clock.advance(0.1)
            assert gate.update(0.0, 0.0) == "holding"
        clock.advance(0.1)
        assert gate.update(0.0, 0.0) == "resting"


class TestDisabled:
    def test_disabled_gate_never_leaves_active(self) -> None:
        clock = _FakeClock()
        gate = _make_gate(clock, enabled=False)
        for _ in range(30):
            assert gate.update(0.0, 0.0) == "active"
            clock.advance(0.1)
        assert gate.resting is False


class TestExplicitNow:
    def test_now_parameter_overrides_clock(self) -> None:
        clock = _FakeClock(1000.0)  # clock deliberately far from `now` values
        gate = _make_gate(clock)
        assert gate.update(0.0, 0.0, now=0.0) == "holding"
        assert gate.update(0.0, 0.0, now=REST_ENTER_HOLD_S - 0.1) == "holding"
        assert gate.update(0.0, 0.0, now=REST_ENTER_HOLD_S) == "resting"
        assert gate.hold_elapsed_s == pytest.approx(REST_ENTER_HOLD_S)
