"""
System-ID killer tests: run the full probe closed-loop against a plant
with KNOWN parameters, then require fit_plant to recover them —
including with the ball's unmodeled 0.6-0.9 Hz self-rock injected.
This is the property the old estimator could not deliver (it
random-walked against the very same known plant).
"""
import math
import random
import threading
import time

import numpy as np
import pytest

from control.ball_controller import BallController
from control.plant_id import ProbeRecording, ProbeScript, fit_plant
from control.plant_model import PlantParams, ServoLag, plant_step
from core.platform_state import BallState
from cv.measurement_filter import AlphaBetaFilter2D
from tools.jitter_bench import FakeClock

HZ = 30.0
DT = 1.0 / HZ

TRUTH = PlantParams(
    g_eff=171.0,
    latency_s=0.0667,
    stiction_deg=0.30,
    warp_c_deg_per_mm=0.0055,
    bias_roll_deg=0.2,
    bias_pitch_deg=-0.1,
    # Real servos take tens of ms to reach a commanded angle. Omitting
    # this from the truth plant is exactly the blindness that biased
    # the first rig fit 39% low on g.
    servo_tau_s=0.06,
)


def _run_probe_session(
    truth: PlantParams,
    rock_amp_mm: float = 0.0,
    rock_freq_hz: float = 0.8,
    seed: int = 0,
    fraction: float = 1.0,
) -> ProbeRecording:
    """Closed-loop probe: script targets -> real controller -> truth
    plant with CONTINUOUS latency -> recording, exactly as a session
    would produce. Integral frozen (auto_trim off), like the session."""
    script = ProbeScript()
    rec = ProbeRecording()
    clock = FakeClock()
    ctrl = BallController(
        kp=0.045, kd=0.022, clock=clock, auto_trim_enabled=False
    )
    filt = AlphaBetaFilter2D()
    rng = random.Random(seed)

    x = y = vx = vy = 0.0
    cmd_t: list[float] = []
    cmd_r: list[float] = []
    cmd_p: list[float] = []
    rock_a = rock_amp_mm * (2.0 * math.pi * rock_freq_hz) ** 2
    phase = rng.random() * 2 * math.pi
    lag_r = ServoLag(truth.servo_tau_s)
    lag_p = ServoLag(truth.servo_tau_s)

    steps = int(script.total_s * fraction * HZ)
    for i in range(steps):
        clock.t += DT
        elapsed = i * DT
        tx, ty = script.target_at(elapsed)
        ctrl.set_target(tx, ty)

        raw_x = x + rng.gauss(0, 0.2)
        raw_y = y + rng.gauss(0, 0.2)
        fx, fy, fvx, fvy = filt.update(raw_x, raw_y, clock.t)
        rec.begin_row(clock.t, raw_x, raw_y, fx, fy, fvx, fvy, tx, ty)
        roll, pitch, _ = ctrl.compute_with_terms(
            BallState(x_mm=fx, y_mm=fy, vx_mm_s=fvx, vy_mm_s=fvy)
        )
        rec.complete_row(roll, pitch)

        # Truth plant with continuous latency on the command stream.
        cmd_t.append(clock.t)
        cmd_r.append(roll)
        cmd_p.append(pitch)
        t_delayed = clock.t - truth.latency_s
        r_del = float(np.interp(t_delayed, cmd_t, cmd_r, left=cmd_r[0]))
        p_del = float(np.interp(t_delayed, cmd_t, cmd_p, left=cmd_p[0]))
        r_act = lag_r.step(r_del, DT)
        p_act = lag_p.step(p_del, DT)
        x, y, vx, vy = plant_step(x, y, vx, vy, r_act, p_act, DT, truth)
        if rock_amp_mm > 0.0:
            w = 2.0 * math.pi * rock_freq_hz
            vx += rock_a * math.sin(w * elapsed + phase) * DT
            vy += rock_a * math.cos(w * elapsed + phase * 1.7) * DT
    return rec


class TestProbeScript:
    def test_duration_and_progress(self) -> None:
        s = ProbeScript()
        assert 70.0 < s.total_s < 90.0
        assert s.progress(0.0) == 0.0
        assert s.progress(s.total_s) == 1.0

    def test_targets_never_exceed_30mm(self) -> None:
        s = ProbeScript()
        for t in np.arange(0.0, s.total_s, 0.05):
            tx, ty = s.target_at(float(t))
            assert math.hypot(tx, ty) <= 30.0 * math.sqrt(2) + 1e-9

    def test_s0_window(self) -> None:
        s = ProbeScript()
        assert s.window_of("S0") == (0.0, 3.0)
        tx, ty = s.target_at(1.0)
        assert (tx, ty) == (0.0, 0.0)

    def test_relay_toggles_alternate(self) -> None:
        s = ProbeScript()
        t0, t1 = s.window_of("S2")
        mids = np.arange(t0 + 1.0, t1, 2.2)
        signs = [np.sign(s.target_at(float(m))[0]) for m in mids]
        assert all(a != b for a, b in zip(signs, signs[1:]))


class TestProbeRecording:
    def test_row_pairing_and_capacity(self) -> None:
        rec = ProbeRecording(capacity=2)
        rec.begin_row(0.0, 1, 2, 3, 4, 5, 6, 7, 8)
        rec.complete_row(0.5, -0.5)
        assert len(rec) == 1
        rec.complete_row(9, 9)          # no open row: ignored
        assert len(rec) == 1
        rec.begin_row(0.033, 1, 2, 3, 4, 5, 6, 7, 8)
        rec.complete_row(0.1, 0.1)
        rec.begin_row(0.066, 1, 2, 3, 4, 5, 6, 7, 8)  # over capacity
        rec.complete_row(0.1, 0.1)
        assert len(rec) == 2
        a = rec.arrays()
        assert a.shape == (2, 11)
        assert a[0, 7] == 0.5


class TestFitRecovery:
    def test_recovers_known_plant(self) -> None:
        rec = _run_probe_session(TRUTH)
        t0 = time.perf_counter()
        fit = fit_plant(rec.arrays(), prior_g_eff=171.0)
        elapsed = time.perf_counter() - t0
        assert fit is not None
        p = fit.params
        assert p.g_eff == pytest.approx(TRUTH.g_eff, rel=0.12)
        # Delay vs first-order lag are partially degenerate; the SUM is
        # what prediction needs and must be tight. tau itself must land
        # in the right decade.
        total_delay = p.latency_s + p.servo_tau_s
        assert total_delay == pytest.approx(
            TRUTH.latency_s + TRUTH.servo_tau_s, abs=0.030
        )
        assert 0.0 <= p.servo_tau_s <= 0.15
        assert p.stiction_deg == pytest.approx(TRUTH.stiction_deg, abs=0.12)
        assert not fit.low_confidence
        # Filter group delay measured separately (raw vs filtered
        # cross-correlation) — sane band for the alpha-beta gains.
        assert 0.0 <= fit.filter_lag_s <= 0.15
        assert fit.predict_s == pytest.approx(
            p.latency_s + p.servo_tau_s + fit.filter_lag_s
        )
        assert elapsed < 30.0

    def test_prior_deviation_flags_low_confidence(self) -> None:
        # THE 2026-07-23 RIG FAILURE PIN: the session fitted g=104
        # against a working prior of 171 with conf=ok, and the design
        # "compensated" into violent oscillation. A fit deviating >35%
        # from the prior must come back low_confidence so the gain
        # change gets capped at +-30%.
        weak_truth = PlantParams(
            g_eff=85.0, latency_s=TRUTH.latency_s,
            stiction_deg=TRUTH.stiction_deg,
            warp_c_deg_per_mm=TRUTH.warp_c_deg_per_mm,
            bias_roll_deg=TRUTH.bias_roll_deg,
            bias_pitch_deg=TRUTH.bias_pitch_deg,
            servo_tau_s=TRUTH.servo_tau_s,
        )
        rec = _run_probe_session(weak_truth)
        fit = fit_plant(rec.arrays(), prior_g_eff=171.0)
        assert fit is not None
        assert fit.low_confidence
        assert "prior" in fit.notes

    def test_recovers_with_self_rock(self) -> None:
        # The 0.8 Hz +-4 mm self-rock must not break g/L recovery: its
        # band (S0-measured, sub-bin interpolated) is notched from both
        # sides of the regression (in closed loop the controller
        # responds to the rock, so it correlates with the command
        # regressor — plain OLS would be biased).
        rec = _run_probe_session(TRUTH, rock_amp_mm=4.0, rock_freq_hz=0.8)
        fit = fit_plant(rec.arrays(), prior_g_eff=171.0)
        assert fit is not None
        p = fit.params
        # 15% tolerance under HEAVY rock (vs 12% clean): the notch
        # necessarily removes a sliver of legitimate command content
        # near the rock band. A moderate fit error is now bounded
        # downstream by the prior-deviation guard and the per-session
        # gain caps — the defense-in-depth this failure mode bought.
        assert p.g_eff == pytest.approx(TRUTH.g_eff, rel=0.15)
        total_delay = p.latency_s + p.servo_tau_s
        assert total_delay == pytest.approx(
            TRUTH.latency_s + TRUTH.servo_tau_s, abs=0.040
        )
        # Stiction under strong dither: the rock never lets the ball
        # stick, so the plant's EFFECTIVE friction is genuinely small
        # (dither linearization). The fit must report that effective
        # value — bounded well below the no-dither cone — not try to
        # reconstruct a cone that is not the operative physics.
        assert 0.0 <= p.stiction_deg <= 0.35

    def test_rock_stats_reported(self) -> None:
        rec = _run_probe_session(TRUTH, rock_amp_mm=4.0, rock_freq_hz=0.8)
        fit = fit_plant(rec.arrays())
        assert fit is not None
        assert 0.5 < fit.rock_freq_hz < 1.1

    def test_short_recording_refused(self) -> None:
        rec = _run_probe_session(TRUTH, fraction=0.4)
        assert fit_plant(rec.arrays()) is None

    def test_cancel_returns_none(self) -> None:
        rec = _run_probe_session(TRUTH)
        ev = threading.Event()
        ev.set()
        assert fit_plant(rec.arrays(), cancel=ev) is None
