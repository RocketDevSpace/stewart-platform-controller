"""
Unit tests for the PIDCore integral term (I-term rework).

Pins the integral's contracts in isolation: integration rate, the
exponential leak, the error-magnitude taper, per-axis directional
anti-windup at the tilt clamp (including the negated roll mapping),
limit clamp + sat flags, freeze semantics (value held, leak paused),
the take_integrator fold primitive, and reset boundaries
(reset_motion_state keeps I; reset_integrator clears it).
"""
import pytest

from control.pid_core import PIDCore


class FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt_s: float) -> None:
        self.t += dt_s


def _make(
    clock: FakeClock,
    ki: float = 0.030,
    i_limit_deg: float = 1.5,
    i_leak_tau_s: float = 25.0,
    max_tilt_deg: float = 10.0,
) -> PIDCore:
    return PIDCore(
        kp=0.045,
        kd=0.022,
        max_tilt_deg=max_tilt_deg,
        max_tilt_rate_deg_s=100000.0,   # slew inert for these tests
        d_term_limit_deg=6.0,
        clock=clock,
        ki=ki,
        i_limit_deg=i_limit_deg,
        i_leak_tau_s=i_leak_tau_s,
        i_err_full_mm=25.0,
        i_err_zero_mm=60.0,
    )


def _run(
    pd: PIDCore,
    clock: FakeClock,
    ex: float,
    ey: float,
    cycles: int,
    hz: float = 30.0,
    **kw: object,
) -> None:
    for _ in range(cycles):
        clock.advance(1.0 / hz)
        pd.compute(ex, ey, 0.0, 0.0, 0.0, 0.0, **kw)  # type: ignore[arg-type]


class TestIntegrationRate:
    def test_first_call_contributes_nothing(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        res = pd.compute(10.0, -10.0, 0.0, 0.0, 0.0, 0.0)
        assert res.i_term == (0.0, 0.0)
        assert res.pitch_raw == pytest.approx(0.045 * 10.0)

    def test_constant_error_integrates_at_ki(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        _run(pd, clock, 10.0, 0.0, 31)          # seed + 1 s
        # Ideal ki*e*t = 0.3 deg; leak over 1 s costs ~2%.
        assert pd.i_pitch_contrib == pytest.approx(0.30, rel=0.05)
        assert pd.i_roll_contrib == 0.0

    def test_roll_axis_sign_mapping(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        _run(pd, clock, 0.0, 10.0, 31)
        # ey > 0 grows i_y; roll contribution is the NEGATED i_y.
        assert pd.i_roll_contrib == pytest.approx(-0.30, rel=0.05)
        res = pd.compute(0.0, 10.0, 0.0, 0.0, 0.0, 0.0)
        assert res.roll_raw < -(0.045 * 10.0)   # P and I both push down


class TestLeak:
    def test_zero_error_decays_exponentially(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        _run(pd, clock, 10.0, 0.0, 31)
        i0 = pd.i_pitch_contrib
        _run(pd, clock, 0.0, 0.0, 750)          # 25 s = one tau
        assert pd.i_pitch_contrib == pytest.approx(i0 * 0.3679, rel=0.05)

    def test_freeze_pauses_leak_and_holds_value(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        _run(pd, clock, 10.0, 0.0, 31)
        i0 = pd.i_pitch_contrib
        _run(pd, clock, 10.0, 0.0, 300, freeze_integrator=True)
        assert pd.i_pitch_contrib == i0         # bit-identical hold
        res = pd.compute(
            10.0, 0.0, 0.0, 0.0, 0.0, 0.0, freeze_integrator=True
        )
        assert res.i_frozen is True


class TestTaper:
    @pytest.mark.parametrize(
        "err,expected",
        [(10.0, 1.0), (25.0, 1.0), (42.5, 0.5), (60.0, 0.0), (200.0, 0.0)],
    )
    def test_weight_at(self, err: float, expected: float) -> None:
        clock = FakeClock()
        pd = _make(clock)
        clock.advance(1 / 30)
        res = pd.compute(err, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert res.i_atten == pytest.approx(expected)

    @pytest.mark.parametrize(
        "err,expected",
        [(0.5, 0.0), (2.0, 0.0), (3.0, 0.5), (4.0, 1.0), (10.0, 1.0)],
    )
    def test_low_side_deadband(self, err: float, expected: float) -> None:
        # Stiction hunting guard (rig-observed 2026-07-23): no
        # integration below the deadband, ramping to full by 2x.
        clock = FakeClock()
        pd = _make(clock)
        pd.i_err_deadband_mm = 2.0
        clock.advance(1 / 30)
        res = pd.compute(err, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert res.i_atten == pytest.approx(expected)

    def test_deadband_error_never_integrates(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        pd.i_err_deadband_mm = 2.0
        _run(pd, clock, 1.0, -1.0, 150)         # |e|=1.4 mm, 5 s inside
        assert pd.i_pitch_contrib == 0.0
        assert pd.i_roll_contrib == 0.0

    def test_flick_error_does_not_integrate(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        _run(pd, clock, 80.0, 80.0, 90)         # 3 s way outside band
        assert pd.i_pitch_contrib == 0.0
        assert pd.i_roll_contrib == 0.0


class TestAntiWindup:
    def test_pitch_saturation_blocks_deepening(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        # pitch_offset 11 deg saturates pitch high; ex > 0 would deepen.
        for _ in range(31):
            clock.advance(1 / 30)
            pd.compute(10.0, 0.0, 0.0, 0.0, 0.0, 11.0)
        assert pd.i_pitch_contrib == 0.0
        # Un-integrating out of saturation stays allowed (ex < 0).
        for _ in range(31):
            clock.advance(1 / 30)
            pd.compute(-10.0, 0.0, 0.0, 0.0, 0.0, 11.0)
        assert pd.i_pitch_contrib < 0.0

    def test_roll_saturation_negated_mapping(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        # roll_offset -11 saturates roll LOW. ey > 0 pushes roll_raw
        # further down -> blocked; ey < 0 recovers -> allowed.
        for _ in range(31):
            clock.advance(1 / 30)
            pd.compute(0.0, 10.0, 0.0, 0.0, -11.0, 0.0)
        assert pd.i_roll_contrib == 0.0
        for _ in range(31):
            clock.advance(1 / 30)
            pd.compute(0.0, -10.0, 0.0, 0.0, -11.0, 0.0)
        assert pd.i_roll_contrib > 0.0


class TestLimitAndFlags:
    def test_clamps_at_limit_with_sat_flags(self) -> None:
        clock = FakeClock()
        pd = _make(clock, i_leak_tau_s=1e9)     # leak inert
        _run(pd, clock, 20.0, 0.0, 30 * 60)     # over a minute at err 20
        assert pd.i_pitch_contrib == pytest.approx(1.5)
        res = pd.compute(20.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert res.i_sat == (True, False)

    def test_limit_override_widens(self) -> None:
        clock = FakeClock()
        pd = _make(clock, i_leak_tau_s=1e9)
        _run(pd, clock, 20.0, 0.0, 30 * 60, i_limit_override=6.0)
        assert pd.i_pitch_contrib > 1.5

    def test_ki_zero_is_inert(self) -> None:
        clock = FakeClock()
        pd = _make(clock, ki=0.0)
        _run(pd, clock, 20.0, 5.0, 90)
        assert pd.i_pitch_contrib == 0.0
        assert pd.i_roll_contrib == 0.0


class TestFeedforward:
    def test_d_term_damps_velocity_error_not_motion(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        # Ball moving exactly at the desired velocity: zero D braking —
        # commanded trajectory motion is never fought.
        res = pd.compute(0.0, 0.0, 30.0, -10.0, 0.0, 0.0,
                         v_des=(30.0, -10.0))
        assert res.d_term == pytest.approx((0.0, 0.0))
        # Zero v_des reproduces classic velocity damping exactly.
        res = pd.compute(0.0, 0.0, 30.0, 0.0, 0.0, 0.0)
        assert res.d_term[0] == pytest.approx(0.022 * -30.0)

    def test_ff_maps_into_raws_like_pd(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        res = pd.compute(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ff=(0.5, 0.3))
        assert res.pitch_raw == pytest.approx(0.5)    # +ff_x
        assert res.roll_raw == pytest.approx(-0.3)    # -(ff_y)
        assert res.ff_vec == (0.5, 0.3)


class TestFoldAndResets:
    def test_take_integrator_maps_and_zeros(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        _run(pd, clock, 10.0, 10.0, 31)
        roll_c, pitch_c = pd.i_roll_contrib, pd.i_pitch_contrib
        assert roll_c < 0.0 < pitch_c
        taken = pd.take_integrator()
        assert taken == (roll_c, pitch_c)
        assert pd.i_roll_contrib == 0.0
        assert pd.i_pitch_contrib == 0.0

    def test_reset_motion_state_keeps_integral(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        _run(pd, clock, 10.0, 0.0, 31)
        i0 = pd.i_pitch_contrib
        pd.reset_motion_state()
        assert pd.i_pitch_contrib == i0

    def test_reset_integrator_clears(self) -> None:
        clock = FakeClock()
        pd = _make(clock)
        _run(pd, clock, 10.0, 0.0, 31)
        pd.reset_integrator()
        assert pd.i_pitch_contrib == 0.0
        # Timebase reseeds: next call contributes nothing again.
        clock.advance(1 / 30)
        res = pd.compute(10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert res.i_term == (0.0, 0.0)
