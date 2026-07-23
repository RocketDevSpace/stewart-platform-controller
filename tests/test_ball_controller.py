"""
Unit tests for the decomposed ball controller (M10):
BallController facade (compute_with_terms) + PDAutotuner.

Rules under test:
- None input returns (0.0, 0.0)
- Disabled controller returns (0.0, 0.0) regardless of input
- Ball at positive x produces negative pitch (platform corrects toward center)
- Ball at positive y produces positive roll (platform corrects toward center)
- Output is clamped to [-max_tilt_deg, max_tilt_deg]
- Autotune (SysID rework 2026-07-23): enabling starts the scripted probe
  through the override channel (target pinned at the manual center during
  S0, no settle gates); ball-lost / out-of-bounds abort the probe; disable
  clears the override and returns to idle
- End-to-end: a full probe session against a KNOWN simulated plant must
  recover its parameters and produce an applicable suggestion (the property
  the old step-test estimator could not deliver)
- Autotune session log goes to the injected path, opened fresh per session
- Rest mode: resting output is exactly level + trim; auto-trim keeps
  integrating during rest and the resting output follows it; a fast
  disturbance regains near-full PD authority on the same call
- D-term pins: no derivative kick on target steps; d-cap clamps
"""
import random
from pathlib import Path

import numpy as np
import pytest

from control.autotune import PDAutotuner
from control.ball_controller import BallController
from control.plant_model import PlantParams, plant_step
from control.setpoint import SetpointArbiter
from core.platform_state import BallState
from cv.measurement_filter import AlphaBetaFilter2D
from settings import PD_D_TERM_LIMIT_DEG

DT = 1.0 / 30.0


class _FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.now = float(start)

    def advance(self, dt_s: float) -> None:
        self.now += float(dt_s)

    def __call__(self) -> float:
        return self.now


def _state(x: float = 0.0, y: float = 0.0, vx: float = 0.0, vy: float = 0.0) -> BallState:
    return BallState(x_mm=x, y_mm=y, vx_mm_s=vx, vy_mm_s=vy)


def _make_tuner(log_path: str, clock: _FakeClock | None = None) -> PDAutotuner:
    clk = clock if clock is not None else _FakeClock()
    arbiter = SetpointArbiter(0.0, 0.0, clk)
    return PDAutotuner(arbiter, clk, log_path=log_path)


class TestNoneInput:
    def test_none_returns_zero(self) -> None:
        ctrl = BallController()
        roll, pitch, _ = ctrl.compute_with_terms(None)
        assert roll == 0.0
        assert pitch == 0.0


class TestDisabled:
    def test_disabled_returns_zero(self) -> None:
        ctrl = BallController()
        ctrl.disable()
        roll, pitch, _ = ctrl.compute_with_terms(_state(x=50.0, y=50.0))
        assert roll == 0.0
        assert pitch == 0.0

    def test_re_enabled_produces_nonzero(self) -> None:
        ctrl = BallController()
        ctrl.disable()
        ctrl.enable()
        roll, pitch, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert pitch != 0.0


class TestOutputDirection:
    def test_ball_positive_x_gives_negative_pitch(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert pitch < 0.0

    def test_ball_negative_x_gives_positive_pitch(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=-10.0))
        assert pitch > 0.0

    def test_ball_positive_y_gives_positive_roll(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        roll, _, _ = ctrl.compute_with_terms(_state(y=10.0))
        assert roll > 0.0

    def test_ball_negative_y_gives_negative_roll(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        roll, _, _ = ctrl.compute_with_terms(_state(y=-10.0))
        assert roll < 0.0

    def test_zero_position_zero_velocity_gives_zero(self) -> None:
        ctrl = BallController(kp=1.0, kd=1.0)
        roll, pitch, _ = ctrl.compute_with_terms(_state())
        assert roll == pytest.approx(0.0)
        assert pitch == pytest.approx(0.0)


class TestClamping:
    def test_large_x_clamps_pitch_to_max(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=1000.0))
        assert pitch == pytest.approx(-5.0)

    def test_large_negative_x_clamps_pitch_to_min(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=-1000.0))
        assert pitch == pytest.approx(5.0)

    def test_large_y_clamps_roll_to_max(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        roll, _, _ = ctrl.compute_with_terms(_state(y=1000.0))
        assert roll == pytest.approx(5.0)

    def test_within_limit_not_clamped(self) -> None:
        ctrl = BallController(kp=0.01, kd=0.0, max_tilt_deg=5.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert abs(pitch) < 5.0


class TestGains:
    def test_set_gains_updates_output(self) -> None:
        ctrl = BallController(kp=0.0, kd=0.0, max_tilt_deg=90.0)
        _, pitch_before, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert pitch_before == pytest.approx(0.0)

        ctrl.set_gains(kp=1.0, kd=0.0)
        _, pitch_after, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert pitch_after != pytest.approx(0.0)


class TestAutotuneFreezesIntegral:
    """I-term rework: autotune no longer touches the auto_trim_enabled
    flag (the old stash/restore is gone). The integral freeze during a
    session is DERIVED from autotuner.enabled inside compute — the leg
    evaluator assumes a pure PD step response."""

    def test_autotune_does_not_flip_the_flag(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.chdir(tmp_path)
        ctrl = BallController(auto_trim_enabled=True)
        ctrl.set_pd_autotune(True)
        assert ctrl.auto_trim_enabled is True
        ctrl.set_pd_autotune(False)
        assert ctrl.auto_trim_enabled is True

    def test_integral_frozen_during_session_thaws_after(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.chdir(tmp_path)
        clock = _FakeClock()
        ctrl = BallController(auto_trim_enabled=True, clock=clock)
        ctrl.set_pd_autotune(True)
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_state(x=10.0))
        assert terms["i_frozen"] is True
        ctrl.set_pd_autotune(False)
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_state(x=10.0))
        assert terms["i_frozen"] is False

    def test_flag_off_stays_off_through_a_session(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.chdir(tmp_path)
        ctrl = BallController(auto_trim_enabled=False)
        ctrl.set_pd_autotune(True)
        ctrl.set_pd_autotune(False)
        assert ctrl.auto_trim_enabled is False


class TestAutotuneProbeLifecycle:
    """SysID rework: enabling autotune starts the scripted probe
    IMMEDIATELY (no settle gates) through the arbiter override channel.
    These tests pin the probe's observable lifecycle through the
    controller facade: start event, S0 center hold, the S0→S1 target
    step, clean disable, and both abort paths."""

    def _probe_controller(self, clock: _FakeClock) -> BallController:
        return BallController(
            kp=0.045, kd=0.022, clock=clock, auto_trim_enabled=False
        )

    def test_enable_starts_probe_pinned_at_center(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)  # session log goes to tmp cwd
        clock = _FakeClock()
        ctrl = self._probe_controller(clock)
        ctrl.set_pd_autotune(True)
        clock.advance(DT)
        _, _, terms = ctrl.compute_with_terms(_state())
        event = terms.get("pd_autotune_event")
        assert event is not None
        assert event["type"] == "probe_started"
        # S0 quiet hold: the override pins the target at the manual center.
        assert (terms["target_x_mm"], terms["target_y_mm"]) == (0.0, 0.0)

    def test_s0_to_s1_steps_target_10mm_x(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        clock = _FakeClock()
        ctrl = self._probe_controller(clock)
        ctrl.set_pd_autotune(True)
        clock.advance(DT)
        _, _, terms = ctrl.compute_with_terms(_state())
        assert (terms["target_x_mm"], terms["target_y_mm"]) == (0.0, 0.0)
        # Cross the 3.0 s S0→S1 boundary in sub-second hops (a valid-frame
        # gap over 1 s would trip the ball-lost abort).
        for _ in range(8):
            clock.advance(0.45)
            _, _, terms = ctrl.compute_with_terms(_state())
        # S1 precheck: the active target moved to center + (10, 0).
        assert (terms["target_x_mm"], terms["target_y_mm"]) == (10.0, 0.0)

    def test_disable_mid_probe_restores_manual_target(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        clock = _FakeClock()
        ctrl = self._probe_controller(clock)
        ctrl.set_target(5.0, 7.0)  # manual target == session center
        ctrl.set_pd_autotune(True)
        clock.advance(DT)
        _, _, terms = ctrl.compute_with_terms(_state(x=5.0, y=7.0))
        assert (terms["target_x_mm"], terms["target_y_mm"]) == (5.0, 7.0)
        # Advance into S1 so the override is visibly NOT the manual target.
        for _ in range(8):
            clock.advance(0.45)
            _, _, terms = ctrl.compute_with_terms(_state(x=5.0, y=7.0))
        assert (terms["target_x_mm"], terms["target_y_mm"]) == (15.0, 7.0)

        ctrl.set_pd_autotune(False)
        assert ctrl._autotuner._state == "idle"
        # Override cleared: the manual target is active again, and further
        # computes run without events or crashes.
        clock.advance(DT)
        _, _, terms = ctrl.compute_with_terms(_state(x=5.0, y=7.0))
        assert (terms["target_x_mm"], terms["target_y_mm"]) == (5.0, 7.0)
        assert "pd_autotune_event" not in terms

    def test_ball_lost_aborts_probe(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        clock = _FakeClock()
        ctrl = self._probe_controller(clock)
        ctrl.set_pd_autotune(True)
        for _ in range(5):
            clock.advance(DT)
            ctrl.compute_with_terms(_state())
        # update() only runs on valid frames: a 2 s silence (> the 1 s
        # PD_AUTOTUNE_BALL_LOST_S gap) means sustained tracking loss.
        clock.advance(2.0)
        _, _, terms = ctrl.compute_with_terms(_state())
        event = terms.get("pd_autotune_event")
        assert event is not None
        assert event["type"] == "probe_aborted"
        assert event["reason"] == "ball lost"
        assert ctrl._autotuner._state == "idle"

    def test_out_of_bounds_aborts_probe(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        clock = _FakeClock()
        ctrl = self._probe_controller(clock)
        ctrl.set_pd_autotune(True)
        # Ball parked 100 mm out (> the 70 mm abort radius): the probe
        # recenters immediately, then aborts once it persists > 2 s.
        events: list[dict] = []
        for _ in range(75):
            clock.advance(DT)
            _, _, terms = ctrl.compute_with_terms(_state(x=100.0))
            if "pd_autotune_event" in terms:
                events.append(terms["pd_autotune_event"])
        assert events, "expected probe_started and probe_aborted events"
        assert events[-1]["type"] == "probe_aborted"
        assert events[-1]["reason"] == "out of bounds"
        assert ctrl._autotuner._state == "idle"


class TestAutotuneEndToEnd:
    """The SysID killer test through the controller session: enable →
    scripted probe against a KNOWN simulated plant → inline fit+design →
    suggestion → apply. The fit must recover the true plant (this is the
    property the old step-test estimator could not deliver) and Apply
    must calibrate gains + prediction horizon + integral deadband."""

    TRUTH = PlantParams(
        g_eff=171.0,
        latency_s=0.0667,
        stiction_deg=0.30,
        warp_c_deg_per_mm=0.0055,
        bias_roll_deg=0.2,
        bias_pitch_deg=-0.1,
    )

    def test_full_session_suggests_recovers_plant_and_applies(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)  # session log goes to tmp cwd
        clock = _FakeClock()
        ctrl = BallController(
            kp=0.045, kd=0.022, ki=0.030, clock=clock,
            auto_trim_enabled=False,
        )
        # Synchronous fit+design on the frame after the probe completes
        # (the app runs it on a background thread; tests must be
        # deterministic).
        ctrl._autotuner.run_compute_inline = True
        ctrl.set_pd_autotune(True, auto_apply=False)

        truth = self.TRUTH
        filt = AlphaBetaFilter2D()
        rng = random.Random(0)
        x = y = vx = vy = 0.0
        cmd_t: list[float] = []
        cmd_r: list[float] = []
        cmd_p: list[float] = []
        events: list[dict] = []
        terms: dict = {}
        # Probe duration + margin: the frame where elapsed reaches
        # total_s runs the inline fit+design; the next emits the
        # suggestion event.
        frames = int(ctrl._autotuner._script.total_s * 30.0) + 10
        for _ in range(frames):
            clock.advance(DT)
            raw_x = x + rng.gauss(0, 0.2)
            raw_y = y + rng.gauss(0, 0.2)
            fx, fy, fvx, fvy = filt.update(raw_x, raw_y, clock.now)
            roll, pitch, terms = ctrl.compute_with_terms(BallState(
                x_mm=fx, y_mm=fy, vx_mm_s=fvx, vy_mm_s=fvy,
                raw_x_mm=raw_x, raw_y_mm=raw_y,
            ))
            if "pd_autotune_event" in terms:
                events.append(terms["pd_autotune_event"])
            # Truth plant with CONTINUOUS latency on the command stream
            # (same pattern as tests/test_plant_id._run_probe_session).
            cmd_t.append(clock.now)
            cmd_r.append(roll)
            cmd_p.append(pitch)
            t_delayed = clock.now - truth.latency_s
            r_act = float(np.interp(t_delayed, cmd_t, cmd_r, left=cmd_r[0]))
            p_act = float(np.interp(t_delayed, cmd_t, cmd_p, left=cmd_p[0]))
            x, y, vx, vy = plant_step(x, y, vx, vy, r_act, p_act, DT, truth)

        types = [e["type"] for e in events]
        assert "probe_started" in types
        assert "suggestion" in types
        suggestion = next(e for e in events if e["type"] == "suggestion")
        assert terms["pd_autotune_has_suggestion"] is True

        # The fit recovered the KNOWN plant (closed-loop through-the-
        # session tolerances: slightly wider than the direct plant_id
        # test's).
        fit = ctrl._autotuner.plant_fit
        assert fit is not None
        assert fit.params.g_eff == pytest.approx(truth.g_eff, rel=0.15)
        assert fit.params.latency_s == pytest.approx(
            truth.latency_s, abs=0.025
        )
        # The design never returns gains it measured as worse than today.
        assert terms["pd_autotune_predicted_cost"] <= 1.0 + 1e-9

        # Apply: gains land on the facade and the NEXT compute carries
        # the transient applied event with the plant calibration payload.
        applied, _, _ = ctrl.apply_pd_autotune_recommendation()
        assert applied is True
        assert ctrl.kp == pytest.approx(suggestion["kp_suggested"])
        assert ctrl.kd == pytest.approx(suggestion["kd_suggested"])
        assert ctrl.ki == pytest.approx(suggestion["ki_suggested"])

        clock.advance(DT)
        _, _, terms = ctrl.compute_with_terms(_state())
        event = terms.get("pd_autotune_event")
        assert event is not None
        assert event["type"] == "applied"
        # Confident fit: Apply also calibrated the prediction horizon
        # (pipeline latency + measured filter lag, clamped to 0.15 s)
        # and the integral deadband.
        assert not fit.low_confidence
        assert event["predict_s"] == pytest.approx(
            max(0.0, min(0.15, fit.predict_s))
        )
        assert 1.0 <= event["deadband_mm"] <= 6.0


class TestRestModeIntegration:
    """Near-target rest mode wired through BallController (RestGate)."""

    def _rest_controller(
        self,
        clock: _FakeClock,
        kp: float = 0.045,
        kd: float = 0.022,
        roll_offset: float = 0.0,
        pitch_offset: float = 0.0,
        auto_trim_enabled: bool = False,
        rest_mode_enabled: bool = True,
    ) -> BallController:
        return BallController(
            kp=kp, kd=kd,
            roll_offset=roll_offset, pitch_offset=pitch_offset,
            auto_trim_enabled=auto_trim_enabled,
            rest_mode_enabled=rest_mode_enabled,
            clock=clock,
        )

    def test_rest_terms_keys_present(self) -> None:
        clock = _FakeClock()
        ctrl = self._rest_controller(clock)
        _, _, terms = ctrl.compute_with_terms(_state(x=5.0))
        for key in (
            "rest_state", "rest_mode_active", "rest_radius_mm",
            "rest_speed_lpf_mm_s", "rest_hold_elapsed_s",
        ):
            assert key in terms, f"missing terms key {key!r}"
        assert terms["rest_state"] in ("active", "holding", "resting")
        assert terms["rest_radius_mm"] == pytest.approx(5.0)

    def test_resting_output_is_exactly_trim_offsets(self) -> None:
        clock = _FakeClock(100.0)
        ctrl = self._rest_controller(
            clock, roll_offset=0.5, pitch_offset=-0.5)
        state = _state(x=3.0)  # 3 mm off target: inside the enter radius

        # Active PD: output carries the P contribution on top of the trim.
        _, pitch, terms = ctrl.compute_with_terms(state)
        assert terms["rest_state"] == "holding"
        assert pitch == pytest.approx(0.045 * (-3.0) - 0.5)

        # Dwell past the hold; slew (300 deg/s * 0.1 s = 30 deg/step) has
        # long since settled the tiny hop to level + trim.
        for _ in range(10):
            clock.advance(0.1)
            roll, pitch, terms = ctrl.compute_with_terms(state)
        assert terms["rest_state"] == "resting"
        assert terms["rest_mode_active"] is True
        assert roll == pytest.approx(0.5)
        assert pitch == pytest.approx(-0.5)
        assert terms["roll_cmd"] == pytest.approx(0.5)
        assert terms["pitch_cmd"] == pytest.approx(-0.5)

    def test_rest_waits_for_integral_settle_then_serves_trim(self) -> None:
        # I-term rework composition: resting parks the output at
        # trim + I with P and D dropped, so rest must WAIT until the
        # integral is flat (|dI/dt| under REST_I_RATE_MAX_DEG_S) — a
        # still-converging I is not an equilibrium and limit-cycles
        # (sim-caught). Once flat, rest engages and its command is
        # trim + I exactly.
        clock = _FakeClock(50.0)
        ctrl = self._rest_controller(clock, kd=0.0, auto_trim_enabled=True)
        ctrl.set_target(0.0, 0.0)

        # Ball parked 5 mm off: the integral walks at ~0.14 deg/s —
        # far above the settle threshold. Rest must NOT engage no
        # matter how long the ball sits quiet.
        state = _state(x=5.0)
        terms: dict = {}
        for _ in range(30):
            clock.advance(0.1)
            _, _, terms = ctrl.compute_with_terms(state)
        assert terms["i_term"][0] < 0.0
        assert terms["i_rate_deg_s"] > 0.02
        assert terms["rest_mode_active"] is False

        # Ball now essentially AT target (0.4 mm — the leak-equilibrium
        # zone): the integral rate falls under the threshold, rest
        # engages after its own hold, and the resting output is
        # trim + integral (pitch contribution = i_x).
        state = _state(x=0.4)
        for _ in range(30):
            clock.advance(0.1)
            _, pitch, terms = ctrl.compute_with_terms(state)
        assert terms["i_rate_deg_s"] <= 0.02
        assert terms["rest_mode_active"] is True
        # Trim store untouched; the leveling is all integral.
        assert terms["pitch_offset"] == 0.0
        assert pitch == pytest.approx(
            terms["pitch_offset"] + terms["i_term"][0], abs=0.02
        )

    def test_fast_disturbance_gets_full_pd_response_same_call(self) -> None:
        # END-TO-END: a resting controller hit by a fast disturbance must
        # answer on that same call with >= 85% of the command an identical
        # never-resting controller produces for the same input history.
        def drive(rest_mode_enabled: bool) -> tuple[float, float, dict]:
            clock = _FakeClock(10.0)
            ctrl = self._rest_controller(
                clock, rest_mode_enabled=rest_mode_enabled)
            terms: dict = {}
            for _ in range(10):
                _, _, terms = ctrl.compute_with_terms(_state())
                clock.advance(0.1)
            if rest_mode_enabled:
                assert terms["rest_mode_active"] is True  # was resting
            # One disturbance frame: radius 15 mm, speed 200 mm/s.
            roll, pitch, terms = ctrl.compute_with_terms(
                _state(x=15.0, vx=200.0))
            return roll, pitch, terms

        roll_r, pitch_r, terms_r = drive(rest_mode_enabled=True)
        roll_n, pitch_n, terms_n = drive(rest_mode_enabled=False)

        # Rest exits on the SAME call: full PD is computed this cycle.
        assert terms_r["rest_state"] == "active"
        assert terms_r["rest_mode_active"] is False
        assert pitch_n < 0.0  # sanity: disturbance demands a real command
        assert pitch_r * pitch_n > 0.0  # same direction
        assert abs(pitch_r) >= 0.85 * abs(pitch_n)

    def test_set_target_resets_rest(self) -> None:
        clock = _FakeClock()
        ctrl = self._rest_controller(clock)
        for _ in range(10):
            _, _, terms = ctrl.compute_with_terms(_state())
            clock.advance(0.1)
        assert terms["rest_mode_active"] is True

        ctrl.set_target(0.0, 0.0)  # same coordinates — still a reset event
        _, _, terms = ctrl.compute_with_terms(_state())
        assert terms["rest_state"] != "resting"

    def test_reset_motion_state_resets_rest(self) -> None:
        clock = _FakeClock()
        ctrl = self._rest_controller(clock)
        for _ in range(10):
            _, _, terms = ctrl.compute_with_terms(_state())
            clock.advance(0.1)
        assert terms["rest_mode_active"] is True

        ctrl.reset_motion_state()  # tracking loss / reacquire path
        _, _, terms = ctrl.compute_with_terms(_state())
        assert terms["rest_state"] != "resting"

    def test_rest_mode_disabled_never_rests(self) -> None:
        clock = _FakeClock()
        ctrl = self._rest_controller(clock, rest_mode_enabled=False)
        for _ in range(30):
            _, _, terms = ctrl.compute_with_terms(_state())
            clock.advance(0.1)
        assert terms["rest_state"] == "active"
        assert terms["rest_mode_active"] is False


class TestHomeCalConvergence:
    """I-term rework: home calibration auto-completes — the controller
    watches the integral, and once it is flat with the ball slow and
    near center, folds it into trim, cancels, and flags a transient
    home_cal_event the GUI persists."""

    def test_converges_folds_and_completes(self) -> None:
        clock = _FakeClock(100.0)   # nonzero: elapsed-timer guard
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
        ctrl.start_home_calibration()
        # Ball essentially at center and still: the integral is flat
        # from the start; the 2 s window fills and convergence fires.
        event: dict = {}
        for _ in range(120):
            clock.advance(1.0 / 30.0)
            _, _, terms = ctrl.compute_with_terms(
                BallState(x_mm=0.2, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
            if "home_cal_event" in terms:
                event = terms["home_cal_event"]
                break
        assert event.get("type") == "completed"
        assert ctrl.home_calibration_active is False
        # The event carries the freshly folded offsets (what the GUI
        # must persist) and they match the store.
        assert event["roll"] == ctrl.roll_offset
        assert event["pitch"] == ctrl.pitch_offset
        # Fold drained the integral (next frame re-integrates a tick).
        clock.advance(1.0 / 30.0)
        _, _, terms = ctrl.compute_with_terms(
            BallState(x_mm=0.2, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
        )
        assert abs(terms["i_term"][0]) < 0.01
        assert abs(terms["i_term"][1]) < 0.01

    def test_far_ball_saturated_integral_does_not_fake_converge(self) -> None:
        # A ball stuck 30 mm out drives the integral to the wide
        # home-cal limit, where it goes FLAT — the radius gate must
        # refuse to fold that windup; the run ends in timeout with the
        # integral discarded and trim untouched.
        clock = _FakeClock(100.0)   # nonzero: elapsed-timer guard
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
        ctrl.start_home_calibration()
        event: dict = {}
        for _ in range(int(31.0 * 30)):
            clock.advance(1.0 / 30.0)
            _, _, terms = ctrl.compute_with_terms(
                BallState(x_mm=30.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
            if "home_cal_event" in terms:
                event = terms["home_cal_event"]
                break
        assert event.get("type") == "timeout"
        assert ctrl.home_calibration_active is False
        assert (ctrl.roll_offset, ctrl.pitch_offset) == (0.0, 0.0)
        clock.advance(1.0 / 30.0)
        _, _, terms = ctrl.compute_with_terms(
            BallState(x_mm=30.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
        )
        # Windup discarded (next frame re-integrates a tick at most).
        assert abs(terms["i_term"][0]) < 0.05
        assert abs(terms["i_term"][1]) < 0.05

    def test_request_trim_fold_flags_saved_event(self) -> None:
        clock = _FakeClock()
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock,
                              auto_trim_enabled=True)
        ctrl.compute_with_terms(
            BallState(x_mm=15.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
        )
        for _ in range(60):
            clock.advance(1.0 / 30.0)
            ctrl.compute_with_terms(
                BallState(x_mm=15.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
        roll, pitch = ctrl.request_trim_fold()
        assert pitch < 0.0                     # absorbed the -x integral
        assert (ctrl.roll_offset, ctrl.pitch_offset) == (roll, pitch)
        clock.advance(1.0 / 30.0)
        _, _, terms = ctrl.compute_with_terms(
            BallState(x_mm=15.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
        )
        event = terms.get("home_cal_event", {})
        assert event.get("type") == "saved"
        assert event["roll"] == roll
        assert event["pitch"] == pitch


class TestRestHomeCalExclusion:
    def test_never_rests_during_home_calibration(self) -> None:
        """Home calibration needs ACTIVE PD centering the ball; rest mode
        parking at level+trim made calibration far less reliable
        (observed on the rig 2026-07-22)."""
        clock = _FakeClock()
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
        ctrl.start_home_calibration()
        # Ball near center and still — prime rest-entry conditions.
        # (4 mm sits above the integration deadband, keeping the
        # integral visibly moving so the convergence watcher cannot
        # auto-complete the calibration mid-test.)
        for _ in range(60):
            clock.advance(1.0 / 30.0)
            _, _, terms = ctrl.compute_with_terms(
                BallState(x_mm=4.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
        assert terms["rest_state"] == "active"
        assert terms["rest_mode_active"] is False
        # After calibration ends, rest can engage again. Ball essentially
        # AT target (0.3 mm): the integral rate is flat (a synthetic ball
        # parked at 1 mm never converges — it can't move — which would
        # hold the I-settle entry gate open forever; a real ball closes
        # the loop).
        ctrl.cancel_home_calibration()
        for _ in range(60):
            clock.advance(1.0 / 30.0)
            _, _, terms = ctrl.compute_with_terms(
                BallState(x_mm=0.3, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
        assert terms["rest_mode_active"] is True


class TestDTermPins:
    """Pinned d-term semantics (control code unchanged by the rest work)."""

    def test_target_step_does_not_kick_d_term(self) -> None:
        # D acts on measured ball velocity, not on the error derivative:
        # stepping the target must not change the d-term at all.
        clock = _FakeClock()
        ctrl = BallController(
            kp=0.05, kd=0.02, rest_mode_enabled=False, clock=clock)
        _, _, before = ctrl.compute_with_terms(_state(x=10.0, vx=40.0))

        clock.advance(0.02)
        ctrl.set_target(30.0, 0.0)  # 30 mm target step
        _, _, after = ctrl.compute_with_terms(_state(x=10.0, vx=40.0))

        assert after["d_term"] == pytest.approx(before["d_term"])
        assert after["d_term"] == pytest.approx((0.02 * -40.0, 0.0))
        # ...while the P term did see the step (the error jumped).
        assert after["p_term"] != pytest.approx(before["p_term"])

    def test_d_term_cap_clamps_extreme_velocity(self) -> None:
        ctrl = BallController(
            kp=0.0, kd=1.0, rest_mode_enabled=False, clock=_FakeClock())
        _, _, terms = ctrl.compute_with_terms(
            _state(vx=10000.0, vy=-10000.0))
        assert terms["d_term"] == pytest.approx(
            (-PD_D_TERM_LIMIT_DEG, PD_D_TERM_LIMIT_DEG))


class TestAutotuneSessionLog:
    def test_session_log_written_to_injected_path(self, tmp_path: Path) -> None:
        log_file = tmp_path / "autotune_session_test.log"
        tuner = _make_tuner(log_path=str(log_file))
        tuner.set_enabled(True, None, 0.045, 0.022)
        assert log_file.exists()
        assert "SESSION START (SysID)" in log_file.read_text(encoding="utf-8")

        tuner.set_enabled(False, None, 0.045, 0.022)
        text = log_file.read_text(encoding="utf-8")
        assert "SESSION END" in text

    def test_re_enable_swaps_handler_and_truncates(self, tmp_path: Path) -> None:
        log_file = tmp_path / "autotune_session_test.log"
        tuner = _make_tuner(log_path=str(log_file))
        tuner.set_enabled(True, None, 0.045, 0.022)
        tuner.set_enabled(False, None, 0.045, 0.022)
        assert "SESSION END" in log_file.read_text(encoding="utf-8")

        # New session opens the file fresh (mode="w"): old lines are gone.
        tuner.set_enabled(True, None, 0.050, 0.020)
        text = log_file.read_text(encoding="utf-8")
        assert "SESSION START" in text
        assert "SESSION END" not in text
