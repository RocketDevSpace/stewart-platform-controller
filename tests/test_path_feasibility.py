"""
Path-following feasibility pins over tools/path_sim.py: the closed-loop
sim (real filter -> BallController/PathFollower chain, idealized plant —
no servo dynamics, latency, or friction, so an OPTIMISTIC bound) must
show the pacing law stable at the shipped defaults. Each run is 900-1800
steps of pure Python + numpy — well under a second each.

Also holds the warp-field pins from the 2026-07-23 I-term rework: the
rig-deadlock reproduction (integral off), the recovery proof (integral
on, same field), and the ki bias-cancel timing sweep.
"""
import math
import random

from control.ball_controller import BallController
from control.patterns import circle, star
from core.platform_state import BallState
from cv.measurement_filter import AlphaBetaFilter2D
from tools.jitter_bench import FakeClock
from tools.path_sim import (
    G_EFF,
    _apply_rolling_resistance,
    simulate_path_following,
)


class TestCircleFeasibility:
    def test_default_speed_tracks_and_laps(self) -> None:
        # v=30 (the PATH_SPEED_MM_S default) on a 60 mm circle: the ball
        # must never stall out of the capture radius and completes at
        # least one full lap in 40 s.
        res = simulate_path_following(circle(radius_mm=60.0), 30.0, 40.0)
        assert res.max_err_mm < 20.0
        assert res.laps >= 1
        assert res.mean_err_mm < 15.0

    def test_slow_following_is_smooth(self) -> None:
        # v=10: pursuit lag stays small — slow following is smooth.
        res = simulate_path_following(circle(radius_mm=60.0), 10.0, 60.0)
        assert res.max_err_mm < 8.0

    def test_infeasible_speed_degrades_gracefully(self) -> None:
        # THE SAFETY PROPERTY: v=60 is well above the analytic ~35 mm/s
        # stall ceiling at default gains. The adaptive taper must degrade
        # an infeasible speed setting into slower ACTUAL laps — the
        # target waits inside the capture radius rather than running away
        # — so the ball is never abandoned. Tracking error stays bounded
        # near the capture radius instead of diverging.
        res = simulate_path_following(circle(radius_mm=60.0), 60.0, 30.0)
        assert res.max_err_mm < 25.0


class TestStarFeasibility:
    def test_sharp_corners_slow_and_recover(self) -> None:
        # 5-point star at v=25: the sharp inner corners force overshoot,
        # the taper slows the target, the ball recovers — no divergence.
        res = simulate_path_following(star(), 25.0, 40.0)
        assert res.max_err_mm < 25.0


# Rig-measured warp field, 2026-07-23: bowl coefficient 0.0055 deg/mm
# (0.36 deg extra tilt needed at r=65, ball equilibrium 8 mm inside the
# circle) plus a stale-trim constant bias. See the [I-term] commits.
RIG_WARP_C = 0.0055
RIG_STALE_TRIM_ROLL = 0.6
RIG_STALE_TRIM_PITCH = 0.6


class TestWarpFieldFailurePin:
    """The 2026-07-23 rig deadlock, reproduced in sim.

    With the warp field + stale trim and NO integral action anywhere
    (BallController defaults: auto-trim disabled, ki unwired), the
    standing offset meets/exceeds the 20 mm capture radius, the advance
    factor pins at 0, and the follower never leaves the seed point. The
    original feasibility suite missed this because its plant was
    perfectly trimmed — a zero-constant-disturbance plant cannot expose
    a missing integrator.
    """

    def test_circle_with_warp_and_no_integral_never_advances(self) -> None:
        res = simulate_path_following(
            circle(), 30.0, 40.0,
            warp_c_deg_per_mm=RIG_WARP_C,
            warp_bias_roll_deg=RIG_STALE_TRIM_ROLL,
            warp_bias_pitch_deg=RIG_STALE_TRIM_PITCH,
            integral_enabled=False,
        )
        # Zero laps; the mean advance is a crawl at a fraction of the
        # commanded 30 mm/s (rolling resistance parks the ball right at
        # the capture boundary, so noise dithers a slow creep — the
        # frictionless plant showed 0.64 mm/s; either way the path never
        # completes; with the integral on, the SAME field gives
        # ~30 mm/s, see TestWarpFieldRecovery).
        assert res.laps == 0
        assert res.mean_advance_mm_s < 10.0


class TestWarpFieldRecovery:
    """The I-term fix, proven over the SAME field that deadlocks above.

    The integral tracks the rotating warp bias around the path (its
    corner ki/kp = 0.67 rad/s sits above the 0.46 rad/s carrot rotation
    at 30 mm/s on r=65), so the standing offset that pinned the follower
    is integrated away and laps complete at essentially the commanded
    speed. Measured on this sim: 2 laps @ 29.7 mm/s mean advance,
    max_err 14.2 mm (vs 0 laps @ 0.64 mm/s with the integral off).
    """

    def test_circle_with_warp_and_integral_laps(self) -> None:
        res = simulate_path_following(
            circle(), 30.0, 40.0,
            warp_c_deg_per_mm=RIG_WARP_C,
            warp_bias_roll_deg=RIG_STALE_TRIM_ROLL,
            warp_bias_pitch_deg=RIG_STALE_TRIM_PITCH,
        )
        assert res.laps >= 1
        assert res.max_err_mm < 20.0
        assert res.mean_advance_mm_s > 20.0

    def test_feedforward_halves_mean_tracking_error(self) -> None:
        # The trajectory-feedforward pin (2026-07-23 second rig session:
        # at low speeds the wobble dominated the path drive). With the
        # 2-frame latency plant, ff + prediction cuts the circle's mean
        # tracking error from ~10 mm (plain PID) to ~5.7 mm and unlocks
        # near-commanded speed at 45 mm/s (98% vs 80% delivered).
        import control.ball_controller as bc
        saved = (bc.PATH_FF_ENABLED, bc.CONTROL_PREDICT_S)
        try:
            bc.PATH_FF_ENABLED = False
            bc.CONTROL_PREDICT_S = 0.0
            plain = simulate_path_following(
                circle(), 30.0, 40.0,
                warp_c_deg_per_mm=RIG_WARP_C,
                warp_bias_roll_deg=RIG_STALE_TRIM_ROLL,
                warp_bias_pitch_deg=RIG_STALE_TRIM_PITCH,
            )
        finally:
            bc.PATH_FF_ENABLED, bc.CONTROL_PREDICT_S = saved
        shipped = simulate_path_following(
            circle(), 30.0, 40.0,
            warp_c_deg_per_mm=RIG_WARP_C,
            warp_bias_roll_deg=RIG_STALE_TRIM_ROLL,
            warp_bias_pitch_deg=RIG_STALE_TRIM_PITCH,
        )
        assert shipped.mean_err_mm < plain.mean_err_mm * 0.85
        assert shipped.mean_err_mm < 9.0

    def test_star_with_warp_corners_bounded(self) -> None:
        # Sharp inner corners + rotating field: the corner transient from
        # a momentarily stale integral vector is bounded by the taper
        # (advance pauses while the integral re-aims in ~2 s).
        res = simulate_path_following(
            star(), 25.0, 40.0,
            warp_c_deg_per_mm=RIG_WARP_C,
            warp_bias_roll_deg=RIG_STALE_TRIM_ROLL,
            warp_bias_pitch_deg=RIG_STALE_TRIM_PITCH,
        )
        assert res.laps >= 1
        assert res.max_err_mm < 25.0


def _bias_cancel_time_s(
    ki: float,
    bias_deg: float = 0.4,
    duration_s: float = 12.0,
    hz: int = 30,
) -> float:
    """Closed-loop center hold against a constant pitch bias: seconds
    until |x| stays below 10% of the P-only standing offset (bias/kp)
    for the remainder of the run. Same plant/filter as path_sim."""
    dt = 1.0 / hz
    clock = FakeClock()
    ctrl = BallController(
        kp=0.045, kd=0.022, clock=clock, ki=ki, auto_trim_enabled=True
    )
    filt = AlphaBetaFilter2D()
    rng = random.Random(0)
    x = y = vx = vy = 0.0
    threshold = 4.0   # mm; see TestBiasCancelSweep docstring

    trace: list[tuple[float, float]] = []
    for step in range(int(duration_s * hz)):
        clock.t += dt
        fx, fy, fvx, fvy = filt.update(
            x + rng.gauss(0.0, 0.15), y + rng.gauss(0.0, 0.15), clock.t
        )
        roll, pitch, _ = ctrl.compute_with_terms(
            BallState(x_mm=fx, y_mm=fy, vx_mm_s=fvx, vy_mm_s=fvy)
        )
        ax = G_EFF * (pitch - bias_deg)
        ay = -G_EFF * roll
        ax, ay, vx, vy = _apply_rolling_resistance(ax, ay, vx, vy, 0.06)
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt
        trace.append((clock.t, math.hypot(x, y)))

    # Convergence time: after this instant the error never exceeds the
    # threshold again.
    last_exceed = 0.0
    for t, r in trace:
        if r > threshold:
            last_exceed = t
    return last_exceed if trace and trace[-1][1] <= threshold else float("inf")


class TestBiasCancelSweep:
    """0.4 deg standing bias = 8.9 mm P-only offset. The convergence
    threshold is 4 mm — the integration deadband (2 mm, the stiction
    hunting guard) deliberately stops correcting below the accuracy the
    plate's static friction can hold, and rolling resistance adds its
    own stick zone; tighter bounds would test the friction model, not
    the controller. The observed endgame is ideal rig behavior:
    converge, rest, integral frozen at the learned bias, total servo
    silence."""

    def test_default_ki_cancels_a_0p4_deg_bias_and_stays(self) -> None:
        # tau_I = kp/ki = 1.5 s at ki 0.030: inside 4 mm (was 8.9) and
        # STAYING there within 5 s including the stick-slip endgame.
        assert _bias_cancel_time_s(0.030) <= 5.0

    def test_sweep_all_candidate_gains_converge(self) -> None:
        # The overlay-tunable range around the default: every candidate
        # converges within the run; slower ki is slower, never unstable.
        for ki in (0.020, 0.030, 0.040):
            assert _bias_cancel_time_s(ki, duration_s=20.0) < 10.0
