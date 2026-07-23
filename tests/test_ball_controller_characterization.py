"""
Characterization tests for control/ball_controller.py — M10 decomposition guard.

These tests pin the OBSERVABLE behavior of BallController through its public
API only (constructor, public methods, and the compute_with_terms terms dict).
They were written against the pre-decomposition monolith; the terms-dict key
set and value semantics are frozen because the GUI reads these keys.

CONTRACT AMENDMENT 2026-07-23 (I-term rework): the gated AutoTrim integrator
was deleted — integral action now lives in PIDCore, trim is a pure store
(control/trim_store.py). 23 auto_trim_* gate/state keys were REMOVED from the
contract (deliberately — see the list at TERMS_KEYS), 5 i_* keys added, and
`auto_trim_enabled` was repurposed as the integral-enable flag (default now
ON via settings.PD_I_ENABLED). This is the only sanctioned contract break;
outside a dated amendment like this one, removals/renames remain forbidden.
"""
from pathlib import Path

import pytest

from control.ball_controller import BallController
from core.platform_state import BallState
from settings import PD_I_LIMIT_DEG


class FakeClock:
    """Injectable monotonic clock for deterministic dt stepping."""

    def __init__(self, start: float = 0.0) -> None:
        self.now = float(start)

    def advance(self, dt_s: float) -> None:
        self.now += float(dt_s)

    def __call__(self) -> float:
        return self.now


def _state(x: float = 0.0, y: float = 0.0, vx: float = 0.0, vy: float = 0.0) -> BallState:
    return BallState(x_mm=x, y_mm=y, vx_mm_s=vx, vy_mm_s=vy)


# Frozen key set of the full terms dict (autotune event key excluded: it is
# only present on autotune state transitions and is pinned separately below).
TERMS_KEYS = frozenset({
    "position_vec_mm",
    "velocity_vec_mm_s",
    "pd_vec",
    "p_term",
    "d_term",
    "roll_raw",
    "pitch_raw",
    "roll_clamped",
    "pitch_clamped",
    "roll_cmd",
    "pitch_cmd",
    "roll_offset",
    "pitch_offset",
    "auto_trim_enabled",
    "home_calibration_active",
    "home_calibration_elapsed_s",
    # REMOVED 2026-07-23 (I-term rework) — the gated AutoTrim integrator
    # and every key describing its gates/state machine were deleted:
    #   trim_settled_s, trim_updates, auto_trim_state,
    #   auto_trim_target_hold_remaining_s, auto_trim_speed_mm_s,
    #   auto_trim_radius_mm, auto_trim_speed_lpf_mm_s,
    #   auto_trim_radius_lpf_mm, auto_trim_speed_thresh_mm_s,
    #   auto_trim_radius_thresh_mm, auto_trim_hold_s,
    #   auto_trim_roll_step_deg, auto_trim_pitch_step_deg,
    #   auto_trim_limit_deg_active, auto_trim_limit_deg_normal,
    #   auto_trim_limit_deg_home_cal, auto_trim_roll_sat,
    #   auto_trim_pitch_sat, auto_trim_gate_speed_ok,
    #   auto_trim_gate_radius_ok, auto_trim_gate_radius_bypassed,
    #   auto_trim_gate_settled_ok, auto_trim_gate_reason
    # (GUI read none of them outside the home-cal diag line, rewritten in
    # the same rework. auto_trim_enabled now means: integral enabled.)
    # ADDED 2026-07-23: the PIDCore integral's telemetry.
    "i_term",
    "i_sat_x",
    "i_sat_y",
    "i_atten",
    "i_frozen",
    "i_rate_deg_s",
    "ff_vec",
    "v_des_mm_s",
    "home_cal_converge_s",
    # Near-target rest mode (perf pass, 2026-07-22): keys ADDED to the
    # frozen contract — additions are allowed (the GUI ignores unknown
    # keys); removals/renames of anything above remain forbidden.
    "rest_state",
    "rest_mode_active",
    "rest_radius_mm",
    "rest_speed_lpf_mm_s",
    "rest_hold_elapsed_s",
    # Path-following keys added 2026-07-22 (feat/path) — additions to this
    # set are allowed; removals/renames remain forbidden.
    "path_active",
    "path_state",
    "path_name",
    "path_progress",
    "path_lap",
    "path_s_mm",
    "path_error_mm",
    "path_speed_mm_s",
    "kp",
    "kd",
    "pd_autotune_enabled",
    "pd_autotune_auto_apply",
    "pd_autotune_trial_count",
    "pd_autotune_message",
    "pd_autotune_has_suggestion",
    "pd_autotune_suggested_kp",
    "pd_autotune_suggested_kd",
    # ADDED 2026-07-23 (SysID autotune rework): additive telemetry from
    # the probe→fit→design pipeline — the 7 keys above stay frozen.
    "pd_autotune_suggested_ki",
    "pd_autotune_phase",
    "pd_autotune_progress",
    "pd_autotune_plant_g_eff",
    "pd_autotune_plant_latency_s",
    "pd_autotune_plant_stiction_deg",
    "pd_autotune_predict_s",
    "pd_autotune_predicted_cost",
    "target_x_mm",
    "target_y_mm",
})

# Frozen key set of the early-return terms dict (disabled or None input).
EARLY_RETURN_KEYS = frozenset({
    "position_vec_mm",
    "velocity_vec_mm_s",
    "pd_vec",
})


class TestTermsDictContract:
    def test_full_terms_key_set_is_frozen(self) -> None:
        clock = FakeClock(100.0)
        ctrl = BallController(clock=clock)
        _, _, terms = ctrl.compute_with_terms(_state(x=5.0, y=-3.0, vx=1.0, vy=2.0))
        assert set(terms.keys()) == TERMS_KEYS

    def test_none_input_terms_key_set(self) -> None:
        ctrl = BallController(clock=FakeClock())
        roll, pitch, terms = ctrl.compute_with_terms(None)
        assert (roll, pitch) == (0.0, 0.0)
        assert set(terms.keys()) == EARLY_RETURN_KEYS
        assert terms["position_vec_mm"] == (0.0, 0.0)
        assert terms["velocity_vec_mm_s"] == (0.0, 0.0)
        assert terms["pd_vec"] == (0.0, 0.0)

    def test_disabled_terms_key_set(self) -> None:
        ctrl = BallController(clock=FakeClock())
        ctrl.disable()
        roll, pitch, terms = ctrl.compute_with_terms(_state(x=50.0, y=50.0))
        assert (roll, pitch) == (0.0, 0.0)
        assert set(terms.keys()) == EARLY_RETURN_KEYS

    def test_representative_values(self) -> None:
        clock = FakeClock(10.0)
        ctrl = BallController(kp=0.1, kd=0.0, roll_offset=0.5, pitch_offset=-0.5,
                              clock=clock)
        roll, pitch, terms = ctrl.compute_with_terms(_state(x=10.0, y=-20.0))
        # error vector = target - ball
        assert terms["position_vec_mm"] == pytest.approx((-10.0, 20.0))
        assert terms["velocity_vec_mm_s"] == pytest.approx((0.0, 0.0))
        assert terms["p_term"] == pytest.approx((-1.0, 2.0))
        assert terms["d_term"] == pytest.approx((0.0, 0.0))
        assert terms["pd_vec"] == pytest.approx((-1.0, 2.0))
        # pitch = pd_x + pitch_offset ; roll = -pd_y + roll_offset
        assert terms["pitch_raw"] == pytest.approx(-1.5)
        assert terms["roll_raw"] == pytest.approx(-1.5)
        assert terms["roll_cmd"] == pytest.approx(roll)
        assert terms["pitch_cmd"] == pytest.approx(pitch)
        assert terms["roll_offset"] == pytest.approx(0.5)
        assert terms["pitch_offset"] == pytest.approx(-0.5)
        assert terms["kp"] == pytest.approx(0.1)
        assert terms["kd"] == pytest.approx(0.0)
        assert terms["target_x_mm"] == 0.0
        assert terms["target_y_mm"] == 0.0
        # Default flipped ON in the I-term rework (settings.PD_I_ENABLED)
        assert terms["auto_trim_enabled"] is True
        # First call seeds the integral timebase only — exact-PD values
        # above hold because the first frame contributes no integral.
        assert terms["i_term"] == (0.0, 0.0)
        assert terms["i_frozen"] is False
        assert terms["home_calibration_active"] is False
        assert terms["pd_autotune_enabled"] is False


class TestSlewLimit:
    def test_first_sample_passes_through(self) -> None:
        clock = FakeClock(5.0)
        ctrl = BallController(kp=1.0, kd=0.0, clock=clock)
        _, pitch, terms = ctrl.compute_with_terms(_state(x=-100.0))
        # raw = kp * 100 = 100 -> clamped to max_tilt 10 -> first sample passes
        assert terms["pitch_clamped"] == pytest.approx(10.0)
        assert terms["pitch_cmd"] == pytest.approx(10.0)
        assert pitch == pytest.approx(10.0)

    def test_step_limited_at_300_deg_s(self) -> None:
        clock = FakeClock(5.0)
        ctrl = BallController(kp=1.0, kd=0.0, clock=clock)
        ctrl.compute_with_terms(_state())  # cmd = 0, initializes slew state
        clock.advance(0.010)
        _, pitch, terms = ctrl.compute_with_terms(_state(x=-100.0))
        # clamped command jumps 0 -> 10 but slew allows 300 deg/s * 10 ms = 3 deg
        assert terms["pitch_clamped"] == pytest.approx(10.0)
        assert terms["pitch_cmd"] == pytest.approx(3.0)
        assert pitch == pytest.approx(3.0)
        # second step advances another 3 deg
        clock.advance(0.010)
        _, pitch, terms = ctrl.compute_with_terms(_state(x=-100.0))
        assert terms["pitch_cmd"] == pytest.approx(6.0)


class TestDTermCap:
    def test_d_term_capped_at_settings_limit(self) -> None:
        # Pins the MECHANISM (symmetric clamp at PD_D_TERM_LIMIT_DEG), not
        # the tunable value — the cap was deliberately raised 2.5 -> 6.0 in
        # the 2026-07-22 live tuning session (it saturated on every fast
        # event and starved the flick response).
        from settings import PD_D_TERM_LIMIT_DEG as CAP
        ctrl = BallController(kp=0.0, kd=1.0, clock=FakeClock())
        _, _, terms = ctrl.compute_with_terms(_state(vx=100.0, vy=-100.0))
        # d = kd * (-v) = (-100, +100) -> capped to +/-CAP
        assert terms["d_term"] == pytest.approx((-CAP, CAP))
        assert terms["pd_vec"] == pytest.approx((-CAP, CAP))

    def test_small_d_term_not_capped(self) -> None:
        ctrl = BallController(kp=0.0, kd=0.01, clock=FakeClock())
        _, _, terms = ctrl.compute_with_terms(_state(vx=100.0))
        assert terms["d_term"] == pytest.approx((-1.0, 0.0))


class TestIntegralTerm:
    def test_standing_error_integrates_continuously_trim_untouched(self) -> None:
        clock = FakeClock(50.0)
        ctrl = BallController(kp=0.0, kd=0.0, auto_trim_enabled=True, clock=clock)
        state = _state(x=30.0)  # ball parked 30 mm off target, zero velocity
        ctrl.set_target(0.0, 0.0)

        _, _, terms = ctrl.compute_with_terms(state)  # timebase seed call
        assert terms["i_term"] == (0.0, 0.0)

        # NO gates, NO hold windows: the very next frame integrates.
        clock.advance(0.1)
        _, _, terms = ctrl.compute_with_terms(state)
        # error = target - ball = -30 -> i_x negative; err 30 mm is inside
        # the taper band (weight (60-30)/35 ~ 0.857).
        assert terms["i_term"][0] < 0.0
        assert terms["i_atten"] == pytest.approx((60.0 - 30.0) / 35.0)

        # Long dwell: the integral clamps at PD_I_LIMIT_DEG; the persistent
        # TRIM OFFSETS never move (trim is a store, not an integrator).
        for _ in range(3000):
            clock.advance(0.1)
            _, _, terms = ctrl.compute_with_terms(state)
        assert terms["i_term"][0] == pytest.approx(-PD_I_LIMIT_DEG)
        assert terms["i_sat_x"] == 1.0
        assert terms["i_sat_y"] == 0.0
        assert terms["pitch_offset"] == 0.0
        assert terms["roll_offset"] == 0.0
        # The integral rides in the raw pitch command exactly like trim did.
        assert terms["pitch_raw"] == pytest.approx(-PD_I_LIMIT_DEG)

    def test_disable_freezes_in_place(self) -> None:
        clock = FakeClock()
        ctrl = BallController(kp=0.0, kd=0.0, auto_trim_enabled=True, clock=clock)
        state = _state(x=20.0)
        ctrl.compute_with_terms(state)
        for _ in range(30):
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(state)
        i_before = terms["i_term"]
        assert i_before[0] < 0.0

        ctrl.set_auto_trim_enabled(False)
        for _ in range(300):
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(state)
        assert terms["i_frozen"] is True
        assert terms["i_term"] == i_before   # bit-identical hold, leak paused

        ctrl.set_auto_trim_enabled(True)
        clock.advance(1 / 30)
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(state)
        assert terms["i_frozen"] is False
        assert terms["i_term"][0] < i_before[0]   # integrating again

    def test_fold_moves_integral_into_trim_zero_net_output(self) -> None:
        clock = FakeClock()
        ctrl = BallController(kp=0.0, kd=0.0, auto_trim_enabled=True, clock=clock)
        state = _state(x=20.0)
        ctrl.compute_with_terms(state)
        for _ in range(60):
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(state)
        raw_before = terms["pitch_raw"]
        i_pitch = terms["i_term"][0]
        assert i_pitch < 0.0

        roll_off, pitch_off = ctrl.fold_integrator_into_trim()
        assert pitch_off == pytest.approx(i_pitch)
        assert ctrl.pitch_offset == pytest.approx(i_pitch)
        # Zero net output change: trim gained exactly what I gave up.
        # (freeze so re-integration doesn't move the comparison frame)
        ctrl.set_auto_trim_enabled(False)
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(state)
        assert terms["i_term"] == (0.0, 0.0)
        assert terms["pitch_raw"] == pytest.approx(raw_before)


class TestAutotuneCoordination:
    def test_enable_freezes_integral_and_disable_restores_target(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path,
    ) -> None:
        monkeypatch.chdir(tmp_path)  # autotune session log goes to tmp cwd
        clock = FakeClock(10.0)
        ctrl = BallController(auto_trim_enabled=True, clock=clock)
        ctrl.set_target(5.0, 7.0)
        _, _, terms = ctrl.compute_with_terms(_state(x=5.0, y=7.0))
        assert terms["target_x_mm"] == 5.0
        assert terms["target_y_mm"] == 7.0
        assert ctrl.auto_trim_enabled is True

        # I-term rework: autotune no longer flips the enable flag (no
        # stash/restore to get wrong) — the integral freeze is DERIVED
        # from autotuner.enabled inside compute (the plant fit assumes a
        # pure PID response with a constant bias; a frozen I is exactly
        # that).
        ctrl.set_pd_autotune(True)
        assert ctrl.pd_autotune_enabled is True
        assert ctrl.auto_trim_enabled is True

        # SysID rework: the probe starts immediately — during the S0
        # quiet hold the override pins the target at the session center
        # (== the manual target).
        for _ in range(8):
            clock.advance(0.1)
            _, _, terms = ctrl.compute_with_terms(_state(x=5.0, y=7.0))
        assert terms["pd_autotune_enabled"] is True
        assert terms["i_frozen"] is True
        assert (terms["target_x_mm"], terms["target_y_mm"]) == (5.0, 7.0)

        # Disable: the integral thaws AND the manual target is restored.
        ctrl.set_pd_autotune(False)
        assert ctrl.pd_autotune_enabled is False
        assert ctrl.auto_trim_enabled is True
        clock.advance(0.1)
        _, _, terms = ctrl.compute_with_terms(_state(x=5.0, y=7.0))
        assert terms["i_frozen"] is False
        assert terms["target_x_mm"] == 5.0
        assert terms["target_y_mm"] == 7.0

    def test_set_gains_clears_autotune_suggestion(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path,
    ) -> None:
        monkeypatch.chdir(tmp_path)
        clock = FakeClock(0.0)
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
        ctrl.set_target(0.0, 0.0)
        ctrl.set_pd_autotune(True, auto_apply=False)

        # Simulate a finished design (running the real ~78 s probe +
        # fit here would make a characterization test expensive — the
        # pipeline itself is covered in test_ball_controller).
        ctrl._autotuner.has_suggestion = True
        ctrl._autotuner.suggested_kp = 0.060
        ctrl._autotuner.suggested_kd = 0.025
        ctrl._autotuner.suggested_ki = 0.020
        clock.advance(0.1)
        _, _, terms = ctrl.compute_with_terms(_state())
        assert terms["pd_autotune_has_suggestion"] is True
        assert terms["kp"] == pytest.approx(0.045)  # not applied

        # A manual gain change invalidates the pending suggestion.
        ctrl.set_gains(0.05, 0.02)
        clock.advance(0.1)
        _, _, terms = ctrl.compute_with_terms(_state())
        assert terms["pd_autotune_has_suggestion"] is False
        assert terms["kp"] == pytest.approx(0.05)
        assert terms["kd"] == pytest.approx(0.02)
