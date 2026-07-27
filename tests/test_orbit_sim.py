"""
Closed-loop harmonic-orbit pins over tools/path_sim.py's orbit driver
(fractional latency + servo lag + the rig-warp field — a richer plant
than the frozen simulate_path_following one).

Measured on 2026-07-27 at the pinned reference gains (r=50, v=40,
90 s, seed 0):
- orbit:  laps 11, mean radius err 1.92 mm, last-2-laps ripple 0.86 mm,
  tangential speed std 0.38 mm/s, lap-ripple curve 7.26 -> 0.77 mm
  (the ILC learning curve).
- carrot follower, SAME plant/driver/metrics: mean radius err 7.04 mm
  (the bowl pulls it inside the circle and the pacing law cannot see
  radius error), speed std 0.55 mm/s.
- kicks at t=45 s: 20 mm absorbed without losing the lap cadence;
  40 mm re-converges to < 1.5 mm last-lap ripple.

Bounds below pin those measurements with margin. All runs use the
hermetic reference gains (overlay independence — the 2026-07-23 rule).
"""
import pytest

from control.plant_model import PlantParams
from tools.path_sim import (
    OrbitSimResult,
    simulate_carrot_circle as _simulate_carrot_circle,
    simulate_orbit as _simulate_orbit,
)


def simulate_orbit(*args, **kwargs):  # type: ignore[no-untyped-def]
    """Hermetic wrapper: pin the historical reference gains (the sim
    defaults read the USER OVERLAY — the 2026-07-23 rule)."""
    kwargs.setdefault("kp", 0.045)
    kwargs.setdefault("kd", 0.022)
    kwargs.setdefault("ki", 0.030)
    return _simulate_orbit(*args, **kwargs)


def simulate_carrot_circle(*args, **kwargs):  # type: ignore[no-untyped-def]
    kwargs.setdefault("kp", 0.045)
    kwargs.setdefault("kd", 0.022)
    kwargs.setdefault("ki", 0.030)
    return _simulate_carrot_circle(*args, **kwargs)


@pytest.fixture(scope="module")
def orbit_run() -> OrbitSimResult:
    result: OrbitSimResult = simulate_orbit(50.0, 40.0, 90.0)
    return result


@pytest.fixture(scope="module")
def carrot_run() -> OrbitSimResult:
    result: OrbitSimResult = simulate_carrot_circle(50.0, 40.0, 90.0)
    return result


class TestConvergedAccuracy:
    def test_radius_accuracy_and_ripple(self, orbit_run: OrbitSimResult) -> None:
        assert orbit_run.laps >= 9
        assert orbit_run.mean_radius_err_mm < 3.0     # measured 1.92
        assert orbit_run.radial_ripple_mm < 1.5       # measured 0.86

    def test_learning_curve_shrinks_ripple(self, orbit_run: OrbitSimResult) -> None:
        # The ILC proof at the system level: last-lap ripple under 30%
        # of the first full lap's (measured 0.77 / 7.26 = 0.11).
        curve = orbit_run.lap_ripple_mm
        assert len(curve) >= 6
        assert curve[-1] < 0.3 * curve[0], f"no learning: {curve}"


class TestSmootherThanCarrot:
    """The headline A/B — same plant, same driver, same metrics."""

    def test_radius_accuracy_beats_carrot(
        self, orbit_run: OrbitSimResult, carrot_run: OrbitSimResult
    ) -> None:
        # The carrot orbits the WRONG radius (bowl warp pulls it ~7 mm
        # inside and the pacing law cannot see radius error); the orbit
        # learns the warp away. Measured 1.92 vs 7.04 mm.
        assert orbit_run.mean_radius_err_mm < 0.5 * carrot_run.mean_radius_err_mm

    def test_tangential_speed_smoother_than_carrot(
        self, orbit_run: OrbitSimResult, carrot_run: OrbitSimResult
    ) -> None:
        # Measured 0.38 vs 0.55 mm/s std. NOTE: this plant has no
        # ArUco glitches or ball self-rock — the rig's carrot jank
        # sources — so the real-world gap should be larger; the pin
        # claims only the strict ordering with margin.
        assert (
            orbit_run.tangential_speed_std
            < 0.85 * carrot_run.tangential_speed_std
        )


class TestKickRobustness:
    def test_20mm_kick_absorbed(self) -> None:
        res = simulate_orbit(
            50.0, 40.0, 90.0, kick_at_s=45.0, kick_mm=20.0
        )
        assert res.laps >= 9                          # cadence kept
        assert res.radial_ripple_mm < 1.5             # measured 0.90

    def test_40mm_kick_reconverges(self) -> None:
        res = simulate_orbit(
            50.0, 40.0, 90.0, kick_at_s=45.0, kick_mm=40.0
        )
        assert res.laps >= 8
        assert res.lap_ripple_mm[-1] < 1.5            # measured 0.84


class TestRigStictionScenario:
    """The first rig session's failure, reproduced and fixed
    (2026-07-27): at rig-level stiction (~0.45 deg equivalent vs the
    0.06 the plant default assumes) the pure clock reference outran the
    ball, the trailing error crossed the old 30 mm tripwire, and the
    orbit churned recover->entrain forever (8 laps in 120 s, ripple
    stuck at 7-8 mm — matching the logged rig data). With the phase
    governor + capture gate + softened tripwires the same scenario
    CONVERGES: measured 12 laps, lap ripple 11.3 -> 3.6 mm, no
    recovers."""

    def test_heavy_stiction_converges_without_churn(self) -> None:
        plant = PlantParams(
            g_eff=171.0, latency_s=2.0 / 30.0, stiction_deg=0.45,
            warp_c_deg_per_mm=0.0055, bias_roll_deg=0.6,
            bias_pitch_deg=0.6, servo_tau_s=0.06,
        )
        # Rig-REPRESENTATIVE gains, passed as explicit literals (still
        # hermetic — nothing read from the overlay): at the weak
        # historical reference gains this plant is near-uncontrollable
        # (P authority ~2x stiction at the tripwire) and churns
        # regardless of orbit logic — a gain-adequacy fact, not an
        # orbit property. The rig runs ~these gains.
        res = _simulate_orbit(
            65.0, 40.0, 120.0, kp=0.072, kd=0.030, ki=0.050, plant=plant
        )
        assert res.laps >= 10                     # was 8 while churning
        curve = res.lap_ripple_mm
        assert curve[3] < 0.6 * curve[0]          # measured 3.6 / 11.3
        assert res.radial_ripple_mm < 5.0         # measured 3.6 (was 7.1)


class TestDeterminism:
    def test_same_seed_same_result(self) -> None:
        a = simulate_orbit(50.0, 40.0, 30.0)
        b = simulate_orbit(50.0, 40.0, 30.0)
        assert a.laps == b.laps
        assert a.mean_radius_err_mm == b.mean_radius_err_mm
        assert a.radial_ripple_mm == b.radial_ripple_mm
        assert a.tangential_speed_std == b.tangential_speed_std
        assert a.err_trace == b.err_trace
