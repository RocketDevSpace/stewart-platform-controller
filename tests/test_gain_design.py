"""
Gain-design killer tests: on a KNOWN fitted plant, the search must
never return gains it measured as worse than the current ones, must
substantially improve a deliberately bad starting point, and must be
deterministic. (The old estimator failed all three: it random-walked.)

The design calls are the expensive part (~20 s each: hundreds of short
closed-loop sims through the real control chain); module-scoped
fixtures run each configuration once.
"""
import threading

import pytest

from control.gain_design import GainDesign, design_gains
from control.plant_id import PlantFit
from control.plant_model import PlantParams

PLANT = PlantParams(
    g_eff=171.0,
    latency_s=0.0667,
    stiction_deg=0.30,
    warp_c_deg_per_mm=0.0055,
    bias_roll_deg=0.2,
    bias_pitch_deg=-0.1,
)

FIT = PlantFit(
    params=PLANT,
    residual_rms_mm=0.8,
    noise_rms_mm=0.3,
    low_confidence=False,
    rock_amp_mm=0.0,
    rock_freq_hz=0.0,
    windows_used=70,
    filter_lag_s=0.05,
)

GOOD = (0.045, 0.022, 0.030)
BAD = (0.020, 0.005, 0.0)


@pytest.fixture(scope="module")
def design_from_good() -> GainDesign:
    d = design_gains(FIT, *GOOD)
    assert d is not None
    return d


@pytest.fixture(scope="module")
def design_from_bad() -> GainDesign:
    d = design_gains(FIT, *BAD)
    assert d is not None
    return d


class TestNeverWorse:
    def test_predicted_cost_never_above_baseline(
        self, design_from_good: GainDesign
    ) -> None:
        # The current gains are always in the candidate pool, so the
        # winner can never be measured-worse than today (J <= 1).
        assert design_from_good.predicted_cost <= 1.0 + 1e-9

    def test_gains_inside_bounds(self, design_from_good: GainDesign) -> None:
        d = design_from_good
        assert 0.010 <= d.kp <= 0.150
        assert 0.005 <= d.kd <= 0.080
        assert 0.0 <= d.ki <= 0.080


class TestBadStartImproves:
    def test_at_least_30pct_improvement(
        self, design_from_bad: GainDesign
    ) -> None:
        # J is normalized to the BAD start's own baseline: <= 0.7 means
        # the designed gains beat it by >= 30% on the same seeds.
        assert design_from_bad.predicted_cost <= 0.70

    def test_moves_toward_sane_gains(self, design_from_bad: GainDesign) -> None:
        # From (0.02, 0.005, 0): stiffness and damping must both rise.
        assert design_from_bad.kp > 0.020
        assert design_from_bad.kd > 0.005


class TestDeterminismAndControl:
    def test_deterministic(self, design_from_bad: GainDesign) -> None:
        d2 = design_gains(FIT, *BAD)
        assert d2 is not None
        assert (d2.kp, d2.kd, d2.ki) == (
            design_from_bad.kp, design_from_bad.kd, design_from_bad.ki
        )

    def test_cancel_returns_none(self) -> None:
        ev = threading.Event()
        ev.set()
        assert design_gains(FIT, *GOOD, cancel=ev) is None

    def test_low_confidence_caps_change(self) -> None:
        fit_low = PlantFit(
            params=PLANT,
            residual_rms_mm=5.0,
            noise_rms_mm=0.3,
            low_confidence=True,
            rock_amp_mm=0.0,
            rock_freq_hz=0.0,
            windows_used=70,
        )
        d = design_gains(fit_low, *GOOD, time_budget_s=25.0)
        assert d is not None
        assert GOOD[0] * 0.7 - 1e-9 <= d.kp <= GOOD[0] * 1.3 + 1e-9
        assert GOOD[1] * 0.7 - 1e-9 <= d.kd <= GOOD[1] * 1.3 + 1e-9
        assert "capped" in d.notes
