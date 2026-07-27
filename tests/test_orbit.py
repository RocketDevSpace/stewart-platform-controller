"""
Unit tests for control/orbit.py (HarmonicOrbit): reference/feedforward
geometry against the PID axis mapping, phase advance, spin-up ramps,
entrain seeding, the ILC learning law (convergence on the rig-warp
plant — the empirical arbiter for the generalized gamma sign), learn
gating, recover semantics, resonance clamp, and determinism.
"""
import math

import numpy as np
import pytest

from control.orbit import (
    STATE_CONE,
    STATE_ENTRAIN,
    STATE_IDLE,
    STATE_RECOVER,
    STATE_TRACK,
    HarmonicOrbit,
    OrbitCommand,
)
from control.pid_core import PIDCore
from control.plant_model import PlantParams, plant_step

G_EFF = 171.0
KP = 0.045
KD = 0.022
KP_EFF = KP * 0.5
DT = 1.0 / 30.0


class FakeClock:
    def __init__(self, start: float = 100.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt_s: float) -> None:
        self.t += dt_s


def _orbit(clock: FakeClock, radius: float = 50.0, speed: float = 40.0) -> HarmonicOrbit:
    o = HarmonicOrbit(clock, radius_mm=radius, speed_mm_s=speed)
    # These suites pin the CLOSED-LOOP machinery (reference + ILC),
    # which stays available behind the cone_only flag; the shipping
    # cone default is pinned in TestConeMode.
    o.cone_only = False
    return o


def _force_track(o: HarmonicOrbit, clock: FakeClock, phi: float = 0.0) -> None:
    """White-box: place the orbit fully spun up at a known phase."""
    o.start()
    o.update(o.radius_mm * math.cos(phi), o.radius_mm * math.sin(phi),
             0.0, G_EFF, KP_EFF)                     # seed
    o._omega = o.speed_mm_s / o.radius_mm
    o._omega_target_eff = o._omega
    o._r = o.radius_mm
    o._phi = phi
    o._state = STATE_TRACK


def _ball_at_ref(o: HarmonicOrbit) -> tuple[float, float]:
    return o._r * math.cos(o._phi), o._r * math.sin(o._phi)


class TestReferenceAndFeedforward:
    def test_ff_and_vdes_signs_at_phi_zero(self) -> None:
        # At (R, 0) moving CCW: v_des = (0, +omega*R); the centripetal
        # acceleration points inward (-x), so ff_x < 0 and through the
        # PID axis mapping pitch_raw < 0 (ax = +g*pitch).
        clock = FakeClock()
        o = _orbit(clock)
        _force_track(o, clock, phi=0.0)
        omega = o.speed_mm_s / o.radius_mm
        clock.advance(1e-4)                          # phi moves ~8e-6 rad
        bx, by = _ball_at_ref(o)
        cmd = o.update(bx, by, 0.0, G_EFF, KP_EFF)
        assert cmd.v_des_mm_s[0] == pytest.approx(0.0, abs=0.2)
        assert cmd.v_des_mm_s[1] == pytest.approx(omega * 50.0, rel=0.01)
        expected_ff_x = -(omega ** 2) * 50.0 / G_EFF
        assert cmd.ff_deg[0] == pytest.approx(expected_ff_x, abs=1e-3)
        assert cmd.ff_deg[1] == pytest.approx(0.0, abs=1e-3)

        pd = PIDCore(kp=KP, kd=KD, max_tilt_deg=10.0,
                     max_tilt_rate_deg_s=1e6, d_term_limit_deg=6.0,
                     clock=clock, ki=0.0)
        res = pd.compute(0.0, 0.0, *cmd.v_des_mm_s, 0.0, 0.0,
                         v_des=cmd.v_des_mm_s, ff=cmd.ff_deg,
                         freeze_integrator=True)
        assert res.pitch_raw < 0.0                   # inward accel at (R, 0)

    def test_phase_advance_rotates_ff(self) -> None:
        # ff with lead_s equals the zero-lead ff rotated by omega*lead
        # (analytic part; the table is empty).
        results: list[OrbitCommand] = []
        for lead in (0.0, 0.1):
            clock = FakeClock()
            o = _orbit(clock)
            _force_track(o, clock, phi=0.7)
            clock.advance(1e-6)
            bx, by = _ball_at_ref(o)
            results.append(o.update(bx, by, lead, G_EFF, KP_EFF))
        omega = 40.0 / 50.0
        th = omega * 0.1
        f0 = results[0].ff_deg
        rot = (f0[0] * math.cos(th) - f0[1] * math.sin(th),
               f0[0] * math.sin(th) + f0[1] * math.cos(th))
        assert results[1].ff_deg[0] == pytest.approx(rot[0], abs=1e-9)
        assert results[1].ff_deg[1] == pytest.approx(rot[1], abs=1e-9)


class TestSpinUp:
    def _run_perfect(self, o: HarmonicOrbit, clock: FakeClock, frames: int) -> list[dict]:
        """Ball rides the reference perfectly; returns telemetry trail."""
        bx, by = 50.0, 0.0
        trail = []
        for _ in range(frames):
            clock.advance(DT)
            cmd = o.update(bx, by, 0.0, G_EFF, KP_EFF)
            bx, by = cmd.target_x_mm, cmd.target_y_mm
            trail.append(o.telemetry())
        return trail

    def test_omega_ramps_linearly_to_target(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        o.start()
        o.update(50.0, 0.0, 0.0, G_EFF, KP_EFF)      # seed at (50, 0)
        trail = self._run_perfect(o, clock, int(5.0 / DT))
        omega_t = 40.0 / 50.0
        # Halfway through the spin-up: about half the target rate.
        mid = trail[int(2.0 / DT)]
        assert mid["orbit_omega"] == pytest.approx(0.5 * omega_t, rel=0.1)
        # Arrived (within a frame) by SPINUP_S, then TRACK.
        end = trail[int(4.2 / DT)]
        assert end["orbit_omega"] == pytest.approx(omega_t, abs=1e-6)
        assert end["orbit_state"] == STATE_TRACK

    def test_tangential_ff_during_ramp(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        o.start()
        o.update(50.0, 0.0, 0.0, G_EFF, KP_EFF)
        clock.advance(DT)
        cmd = o.update(50.0, 0.0, 0.0, G_EFF, KP_EFF)
        omega_t = 40.0 / 50.0
        alpha = omega_t / o.spinup_s
        # Early in the ramp omega ~ 0: the ff is almost purely the
        # tangential ramp term alpha*r/g at phi ~ 0 -> +y direction.
        assert cmd.ff_deg[1] == pytest.approx(alpha * 50.0 / G_EFF, rel=0.05)


class TestEntrainSeed:
    def test_seed_starts_at_ball(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        o.start()
        assert o.state == STATE_ENTRAIN
        cmd = o.update(20.0, 20.0, 0.0, G_EFF, KP_EFF)
        assert cmd.target_x_mm == pytest.approx(20.0, abs=1e-9)
        assert cmd.target_y_mm == pytest.approx(20.0, abs=1e-9)

    def test_seed_radius_floor(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        o.start()
        cmd = o.update(2.0, 1.0, 0.0, G_EFF, KP_EFF)
        r = math.hypot(cmd.target_x_mm, cmd.target_y_mm)
        assert r == pytest.approx(15.0, abs=1e-6)    # floor, ball's angle
        ang = math.atan2(cmd.target_y_mm, cmd.target_x_mm)
        assert ang == pytest.approx(math.atan2(1.0, 2.0), abs=1e-6)

    def test_radius_ramps_to_target(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        o.start()
        o.update(20.0, 0.0, 0.0, G_EFF, KP_EFF)      # seed at r=20
        bx, by = 20.0, 0.0
        for _ in range(int(5.0 / DT)):
            clock.advance(DT)
            cmd = o.update(bx, by, 0.0, G_EFF, KP_EFF)
            bx, by = cmd.target_x_mm, cmd.target_y_mm
        assert o.telemetry()["orbit_r_mm"] == pytest.approx(50.0, abs=1e-6)


def _closed_loop(
    o: HarmonicOrbit,
    clock: FakeClock,
    duration_s: float,
    warp_c: float = 0.0055,
    bias: float = 0.6,
    # The integral runs (as on the real controller): it owns the DC
    # bias (0.6 deg = a 27 mm standing offset at these gains — beyond
    # the ILC learn gate); the ILC owns the rotating warp component.
    ki: float = 0.030,
    latency_frames: int = 2,
    kick: tuple[float, float] | None = None,
    kick_at_s: float | None = None,
) -> dict[int, list[float]]:
    """HarmonicOrbit + scaled PIDCore + warped plant, no noise. Returns
    per-lap |radius - R| samples for convergence analysis."""
    pd = PIDCore(kp=KP, kd=KD, max_tilt_deg=10.0, max_tilt_rate_deg_s=1e6,
                 d_term_limit_deg=6.0, clock=clock, ki=ki)
    plant = PlantParams(g_eff=G_EFF, latency_s=latency_frames * DT,
                        stiction_deg=0.06, warp_c_deg_per_mm=warp_c,
                        bias_roll_deg=bias, bias_pitch_deg=bias)
    x, y, vx, vy = 50.0, 0.0, 0.0, 0.0
    queue = [(0.0, 0.0)] * latency_frames
    o.start()
    per_lap: dict[int, list[float]] = {}
    t = 0.0
    for _ in range(int(duration_s / DT)):
        clock.advance(DT)
        t += DT
        if kick is not None and kick_at_s is not None and abs(t - kick_at_s) < DT / 2:
            x += kick[0]
            y += kick[1]
        cmd = o.update(x, y, 0.08, G_EFF, KP_EFF)
        px = x + vx * 0.08
        py = y + vy * 0.08
        # Division of labor (mirrors the controller wiring): the
        # integral owns the DC bias through entrain + one settling lap,
        # then freezes and the table owns everything periodic — at
        # orbit frequency the running integral's gain exceeds the
        # scaled P-term with 90 deg lag, so the two adapters fight
        # (sim-caught: convergence plateaued at ~6 mm).
        freeze = (ki == 0.0) or o.wants_integral_frozen
        res = pd.compute(cmd.target_x_mm - px, cmd.target_y_mm - py,
                         vx, vy, 0.0, 0.0,
                         v_des=cmd.v_des_mm_s, ff=cmd.ff_deg,
                         gain_scale=0.5, freeze_integrator=freeze)
        queue.append((res.roll_cmd, res.pitch_cmd))
        roll_act, pitch_act = queue.pop(0)
        x, y, vx, vy = plant_step(x, y, vx, vy, roll_act, pitch_act, DT, plant)
        lap = int(o.telemetry()["orbit_lap"])
        if o.state == STATE_TRACK:
            per_lap.setdefault(lap, []).append(abs(math.hypot(x, y) - 50.0))
    return per_lap


class TestILCConvergence:
    """THE sign arbiter: on the rig-warp plant the per-lap radial error
    must shrink as the table learns. A wrong-signed gamma diverges and
    fails this immediately."""

    def test_radial_error_converges_over_laps(self) -> None:
        # Measured (2026-07-27, this exact configuration): lap RMS
        # 15.3 -> 12.3 -> 10.8 -> 8.6 -> 6.5 -> 5.0 -> 3.9 -> 3.1 mm,
        # monotone, harmonics n>=2 clean, bins under the clamp. Bounds
        # pinned with margin.
        clock = FakeClock()
        o = _orbit(clock)
        per_lap = _closed_loop(o, clock, duration_s=70.0)
        laps = sorted(k for k, v in per_lap.items() if len(v) > 150)
        assert len(laps) >= 6, f"not enough full laps: {laps}"
        rms = {k: float(np.sqrt(np.mean(np.square(per_lap[k])))) for k in laps}
        early = rms[laps[1]]
        late = rms[laps[-1]]
        assert late < 0.45 * early, f"no convergence: lap RMS {rms}"
        assert late < 4.0, f"converged level too high: {late:.2f} mm"
        # Table respected its clamp throughout.
        norms = np.hypot(o._cx, o._cy)
        assert float(norms.max()) <= o.ilc_clamp_deg + 1e-9

    def test_determinism(self) -> None:
        runs = []
        for _ in range(2):
            clock = FakeClock()
            o = _orbit(clock)
            _closed_loop(o, clock, duration_s=30.0)
            runs.append((o._cx.copy(), o._cy.copy(), o.state))
        assert np.array_equal(runs[0][0], runs[1][0])
        assert np.array_equal(runs[0][1], runs[1][1])
        assert runs[0][2] == runs[1][2]


class TestLearnGating:
    def test_no_learning_outside_track(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        o.start()
        o.update(50.0, 0.0, 0.0, G_EFF, KP_EFF)      # seed -> ENTRAIN
        clock.advance(DT)
        o.update(48.0, 1.0, 0.0, G_EFF, KP_EFF)
        assert o.state == STATE_ENTRAIN
        assert not np.any(o._cx) and not np.any(o._cy)
        assert o.telemetry()["orbit_learning"] is False

    def test_no_learning_above_gate(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        _force_track(o, clock)
        for _ in range(30):                          # crosses several bins
            clock.advance(DT)
            bx, by = _ball_at_ref(o)
            o.update(bx + 35.0, by, 0.0, G_EFF, KP_EFF)   # 35 > 30 mm gate
        assert not np.any(o._cx) and not np.any(o._cy)
        assert o.telemetry()["orbit_learning"] is False

    def test_learning_writes_toward_reference_on_bin_transit(self) -> None:
        # Ball riding 5 mm INSIDE the circle: ref - ball = +5 mm radial,
        # so with gamma > 0 the flushed corrections gain an outward
        # component — the rig-observed bowl case. Writes happen only on
        # bin transits (one kernel-spread write per bin per lap).
        clock = FakeClock()
        o = _orbit(clock)
        _force_track(o, clock, phi=0.0)
        for _ in range(30):                          # crosses ~3 bins
            clock.advance(DT)
            rx, ry = _ball_at_ref(o)
            r = math.hypot(rx, ry)
            o.update(rx * (r - 5.0) / r, ry * (r - 5.0) / r,
                     0.0, G_EFF, KP_EFF)
        assert o.telemetry()["orbit_learning"] is True
        assert o._cx[0] > 0.0                        # outward at phi ~ 0
        # Radial (outward) projection of the written table is positive
        # where writes landed.
        assert float(np.max(np.hypot(o._cx, o._cy))) > 0.0


class TestRecover:
    def test_tripwire_reseeds_and_freezes_table(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        _force_track(o, clock)
        o._cx[3] = 0.2                               # sentinel knowledge
        table_before = (o._cx.copy(), o._cy.copy())
        for _ in range(o.recover_frames):
            clock.advance(DT)
            bx, by = _ball_at_ref(o)
            # 60 mm: past the (rig-softened) 45 mm tripwire.
            o.update(bx + 60.0, by, 0.0, G_EFF, KP_EFF)
        assert o.state == STATE_RECOVER
        assert o.telemetry()["orbit_recover_count"] == 1
        assert np.array_equal(o._cx, table_before[0])
        assert np.array_equal(o._cy, table_before[1])
        # Perfect tracking from the re-seed: returns to TRACK.
        bx, by = _ball_at_ref(o)
        for _ in range(int(6.0 / DT)):
            clock.advance(DT)
            cmd = o.update(bx, by, 0.0, G_EFF, KP_EFF)
            bx, by = cmd.target_x_mm, cmd.target_y_mm
        assert o.state == STATE_TRACK

    def test_kick_mid_orbit_recovers_in_closed_loop(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        # Even a 60 mm kick is absorbed by feedback FASTER than the
        # 20-frame tripwire (back under 45 mm in ~15 frames) — the
        # desired outcome: no mode churn, still tracking, re-converged.
        # The tripwire mechanism itself is pinned by the direct unit
        # test above (sustained displacement).
        per_lap = _closed_loop(o, clock, duration_s=60.0,
                               kick=(60.0, 0.0), kick_at_s=30.0)
        assert o.state == STATE_TRACK
        last_lap = max(k for k, v in per_lap.items() if len(v) > 50)
        late_rms = float(np.sqrt(np.mean(np.square(per_lap[last_lap]))))
        assert late_rms < 3.0


class TestConeMode:
    """The shipping default (ORBIT_CONE_ONLY): a pure open-loop cone —
    tilt amplitude A rotating at omega = sqrt(g*A/R), feedback OFF,
    ball input unused for control."""

    def _cone(self, clock: FakeClock, radius: float = 50.0) -> HarmonicOrbit:
        o = HarmonicOrbit(clock, radius_mm=radius, speed_mm_s=40.0)
        assert o.cone_only is True                   # the shipping default
        return o

    def test_omega_and_amplitude_from_physics(self) -> None:
        clock = FakeClock()
        o = self._cone(clock)
        o.start()
        bx, by = 30.0, 0.0
        cmd = o.update(bx, by, 0.0, G_EFF, 0.0)      # seed
        for _ in range(int(6.0 / DT)):
            clock.advance(DT)
            # Ball rides the expected point: center EMA ~ 0 -> dc ~ 0,
            # so the ff magnitude is the pure cone amplitude.
            bx, by = cmd.target_x_mm, cmd.target_y_mm
            cmd = o.update(bx, by, 0.0, G_EFF, 0.0)
        tel = o.telemetry()
        assert tel["orbit_state"] == STATE_CONE
        # omega includes the warp-spring term (the plate's bowl acts as
        # a central spring; sim-caught when the naive formula landed
        # the ball at 88 mm instead of 50).
        expected_omega = math.sqrt(
            G_EFF * (o.cone_warp_c + o.cone_tilt_deg / 50.0)
        )
        assert tel["orbit_omega"] == pytest.approx(expected_omega, rel=1e-6)
        # The center-EMA of a rotating point keeps ~5 mm of ripple,
        # feeding a benign ~0.03 deg dc — hence the loose tolerance.
        assert math.hypot(*cmd.ff_deg) == pytest.approx(
            o.cone_tilt_deg, abs=0.1
        )
        # Expected ring radius = g*A/omega^2 = the dialed radius.
        assert tel["orbit_r_mm"] == pytest.approx(50.0, rel=1e-6)

    def test_open_loop_ball_input_does_not_change_cone(self) -> None:
        # With the (slow, separate) center corrector disabled, two runs
        # with completely different ball feeds produce IDENTICAL tilt
        # sequences — the cone itself never chases the ball.
        seqs = []
        for feed in ((30.0, 0.0), (-45.0, 60.0)):
            clock = FakeClock()
            o = self._cone(clock)
            o.cone_center_gain = 0.0                 # isolate the pure cone
            o.start()
            o.update(30.0, 0.0, 0.0, G_EFF, 0.0)     # same seed ball
            seq = []
            for _ in range(60):
                clock.advance(DT)
                cmd = o.update(feed[0], feed[1], 0.0, G_EFF, 0.0)
                seq.append(cmd.ff_deg)
            seqs.append(seq)
        assert seqs[0] == seqs[1]

    def test_center_corrector_opposes_a_parked_offset(self) -> None:
        # Ball parked 40 mm off in +x for many laps: the center EMA
        # converges there and the DC tilt grows NEGATIVE in x (pushing
        # the orbit center back), never exceeding its clamp. This is
        # the ONLY feedback in cone mode — it acts on the per-lap
        # average, so it cannot jitter against the rotation.
        clock = FakeClock()
        o = self._cone(clock)
        o.start()
        o.update(40.0, 0.0, 0.0, G_EFF, 0.0)
        for _ in range(int(60.0 / DT)):
            clock.advance(DT)
            o.update(40.0, 0.0, 0.0, G_EFF, 0.0)
        assert o._center_x == pytest.approx(40.0, rel=0.1)
        assert o._dc_x < -0.1                        # steering back
        assert math.hypot(o._dc_x, o._dc_y) <= o.cone_dc_clamp_deg + 1e-9

    def test_amplitude_ramps_over_spinup(self) -> None:
        clock = FakeClock()
        o = self._cone(clock)
        o.start()
        o.update(30.0, 0.0, 0.0, G_EFF, 0.0)
        cmd = None
        for _ in range(int(o.spinup_s / 2 / DT)):    # frame-wise: dt clamps
            clock.advance(DT)
            cmd = o.update(30.0, 0.0, 0.0, G_EFF, 0.0)
        assert cmd is not None
        assert math.hypot(*cmd.ff_deg) == pytest.approx(
            0.5 * o.cone_tilt_deg, rel=0.05
        )
        assert o.state == STATE_ENTRAIN

    def test_target_is_anti_phase_to_tilt(self) -> None:
        clock = FakeClock()
        o = self._cone(clock)
        o.cone_center_gain = 0.0                     # pure cone geometry
        o.start()
        o.update(30.0, 0.0, 0.0, G_EFF, 0.0)
        cmd = None
        for _ in range(int(6.0 / DT)):
            clock.advance(DT)
            cmd = o.update(30.0, 0.0, 0.0, G_EFF, 0.0)
        assert cmd is not None
        ff_mag = math.hypot(*cmd.ff_deg)
        t_mag = math.hypot(cmd.target_x_mm, cmd.target_y_mm)
        dot = (cmd.ff_deg[0] * cmd.target_x_mm
               + cmd.ff_deg[1] * cmd.target_y_mm) / (ff_mag * t_mag)
        assert dot == pytest.approx(-1.0, abs=1e-9)  # ball rides opposite

    def test_set_cone_tilt_clamps_and_applies_live(self) -> None:
        clock = FakeClock()
        o = self._cone(clock)
        o.set_cone_tilt(10.0)
        assert o.cone_tilt_deg == pytest.approx(4.0)  # max clamp
        o.set_cone_tilt(0.01)
        assert o.cone_tilt_deg == pytest.approx(0.25)  # min clamp
        o.set_cone_tilt(2.5)
        assert o.cone_tilt_deg == pytest.approx(2.5)

    def test_feedback_off_and_integral_frozen(self) -> None:
        clock = FakeClock()
        o = self._cone(clock)
        assert o.feedback_scale == 0.0
        o.start()
        o.update(30.0, 0.0, 0.0, G_EFF, 0.0)
        assert o.wants_integral_frozen is True       # whole session
        assert o.telemetry()["orbit_learning"] is False

    def test_ball_settles_on_the_predicted_ring_in_sim(self) -> None:
        # Closed physics check: drive the cone ff through the plant
        # (warp + drag + latency); the ball must settle onto the ring
        # R = g*A/(omega^2 - g*warp_c). Measured on this plant: exact
        # convergence to r=50.0 at ~86 mm/s; the open-loop transient
        # (overshoot to ~87 mm) decays through rolling resistance
        # alone over ~90 s (the real rig's higher friction settles
        # faster), so the pin measures the settled window.
        from control.plant_model import PlantParams
        clock = FakeClock()
        o = self._cone(clock)
        plant = PlantParams(g_eff=G_EFF, latency_s=2 * DT, stiction_deg=0.06,
                            warp_c_deg_per_mm=0.0055, bias_roll_deg=0.0,
                            bias_pitch_deg=0.0)
        o.start()
        x, y, vx, vy = 10.0, 0.0, 0.0, 0.0
        o.update(x, y, 0.0, G_EFF, 0.0)              # seed
        queue = [(0.0, 0.0)] * 2
        radii = []
        for i in range(int(150.0 / DT)):
            clock.advance(DT)
            cmd = o.update(x, y, 0.0, G_EFF, 0.0)
            # Tilt = ff only (feedback off): pitch = ff_x, roll = -ff_y.
            queue.append((-cmd.ff_deg[1], cmd.ff_deg[0]))
            roll_act, pitch_act = queue.pop(0)
            x, y, vx, vy = plant_step(x, y, vx, vy, roll_act, pitch_act, DT, plant)
            if i > int(120.0 / DT):
                radii.append(math.hypot(x, y))
        mean_r = float(np.mean(radii))
        assert mean_r == pytest.approx(50.0, abs=5.0)   # measured 49.7
        assert float(np.std(radii)) < 3.0               # circulating cleanly


class TestConfig:
    def test_radius_change_resets_table_speed_change_keeps_it(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        o._cx[0] = 0.3
        o.set_speed(50.0)
        assert o._cx[0] == 0.3
        o.set_radius(60.0)
        assert not np.any(o._cx)

    def test_resonance_clamps_omega(self) -> None:
        # r=30 at slider max 80 mm/s asks omega=2.67 — beyond the
        # scaled-gain resonance (1.96 rad/s); the clamp holds 0.85x it.
        clock = FakeClock()
        o = _orbit(clock, radius=30.0, speed=80.0)
        o.start()
        o.update(30.0, 0.0, 0.0, G_EFF, KP_EFF)
        bx, by = 30.0, 0.0
        for _ in range(int(8.0 / DT)):
            clock.advance(DT)
            cmd = o.update(bx, by, 0.0, G_EFF, KP_EFF)
            bx, by = cmd.target_x_mm, cmd.target_y_mm
        limit = 0.85 * math.sqrt(G_EFF * KP_EFF)
        assert o.telemetry()["orbit_omega"] <= limit + 1e-6
        assert o.state == STATE_TRACK                # clamp doesn't wedge entrain

    def test_stop_keeps_table_and_goes_idle(self) -> None:
        clock = FakeClock()
        o = _orbit(clock)
        _force_track(o, clock)
        o._cx[2] = 0.1
        o.stop()
        assert o.state == STATE_IDLE
        assert not o.active
        assert o._cx[2] == 0.1
