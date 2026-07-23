"""
control/pid_core.py

PIDCore — pure PID math for the ball balancer: proportional + derivative
terms, the derivative-term cap (FG-11), a true integral term (the loop's
ONLY integral action — it cancels standing tilt bias the pure P+D cannot),
the max-tilt clamp, and the commanded-tilt slew ("tilt rate") limiter
(FG-11). No target, trim, or autotune knowledge; the caller supplies
error vectors and offsets.

Integral protections (all continuous — no gates, no state machine):
- error-weighted taper: full integration at |e| <= i_err_full_mm, zero at
  >= i_err_zero_mm (flick protection; replaces the old settle gates)
- per-axis conditional anti-windup: no integration in the direction that
  would deepen an active tilt-clamp saturation
- exponential leak (i_leak_tau_s): a mis-learned correction bleeds out on
  its own; steady-state accuracy cost is ~kp/(kp + ki*tau) — sub-mm here
- clamp to ±i_limit_deg (caller may widen per-call, e.g. home cal)
- freeze_integrator holds the value exactly (leak paused too) — used
  while autotune owns the loop or the I feature is toggled off
"""
from collections.abc import Callable
from dataclasses import dataclass


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min(value, max_val), min_val)


@dataclass
class PIDResult:
    """Intermediate and final values of one PID step (terms-dict inputs)."""

    p_term: tuple[float, float]
    d_term: tuple[float, float]
    pd_vec: tuple[float, float]
    roll_raw: float
    pitch_raw: float
    roll_clamped: float
    pitch_clamped: float
    roll_cmd: float
    pitch_cmd: float
    i_term: tuple[float, float] = (0.0, 0.0)
    i_sat: tuple[bool, bool] = (False, False)
    i_atten: float = 1.0
    i_frozen: bool = False
    ff_vec: tuple[float, float] = (0.0, 0.0)


class PIDCore:
    def __init__(
        self,
        kp: float,
        kd: float,
        max_tilt_deg: float,
        max_tilt_rate_deg_s: float,
        d_term_limit_deg: float | None,
        clock: Callable[[], float],
        ki: float = 0.0,
        i_limit_deg: float = 1.5,
        i_leak_tau_s: float = 25.0,
        i_err_full_mm: float = 25.0,
        i_err_zero_mm: float = 60.0,
        i_err_deadband_mm: float = 0.0,
    ) -> None:
        self._clock = clock
        self.kp = float(kp)
        self.kd = float(kd)
        self.max_tilt_deg = float(max_tilt_deg)
        self.max_tilt_rate_deg_s = float(max_tilt_rate_deg_s)
        self.d_term_limit_deg = (
            None if d_term_limit_deg is None else float(d_term_limit_deg)
        )
        self.ki = float(ki)
        self.i_limit_deg = float(i_limit_deg)
        self.i_leak_tau_s = float(i_leak_tau_s)
        self.i_err_full_mm = float(i_err_full_mm)
        self.i_err_zero_mm = float(i_err_zero_mm)
        self.i_err_deadband_mm = float(i_err_deadband_mm)

        # Slew limit state (FG-11)
        self._prev_roll_cmd = 0.0
        self._prev_pitch_cmd = 0.0
        self._prev_cmd_time: float | None = None

        # Integral state — ERROR space (x, y), like p/d, mapped to
        # pitch/roll at the raw-command step. Own timebase, separate
        # from the slew clock: reset_motion_state() must not disturb
        # the learned correction or its dt.
        self._i_x = 0.0
        self._i_y = 0.0
        self._i_prev_time: float | None = None
        # EMA (~0.3 s) of the NET integral rate |dI/dt| (integration +
        # leak combined — zero at the leak equilibrium even though both
        # terms are nonzero). RestGate uses it: rest may only engage
        # once the integral is flat, because resting parks the output at
        # trim + I with P and D dropped — resting on a still-moving I is
        # not an equilibrium and limit-cycles (sim-caught).
        self._i_rate_ema = 0.0

    def reset_motion_state(self) -> None:
        """Clear slew state (tracking loss/reacquire). The integral is
        deliberately KEPT — it is learned plate knowledge, not motion
        state; staleness is the leak's job."""
        self._prev_roll_cmd = 0.0
        self._prev_pitch_cmd = 0.0
        self._prev_cmd_time = None

    def reset_integrator(self) -> None:
        self._i_x = 0.0
        self._i_y = 0.0
        self._i_prev_time = None
        self._i_rate_ema = 0.0

    @property
    def i_rate_deg_s(self) -> float:
        """Smoothed |dI/dt| (net of leak). ~0 when converged or frozen."""
        return self._i_rate_ema

    @property
    def i_pitch_contrib(self) -> float:
        """The integral's contribution to pitch_raw (axis-mapped)."""
        return self._i_x

    @property
    def i_roll_contrib(self) -> float:
        """The integral's contribution to roll_raw (axis-mapped)."""
        return -self._i_y

    def take_integrator(self) -> tuple[float, float]:
        """Atomically drain the integral as (roll, pitch) contributions.

        The fold primitive: TrimStore.fold(*pd.take_integrator()) moves
        the learned correction into persistent trim with zero net change
        to the commanded output (trim rises by exactly what I gave up).
        """
        out = (self.i_roll_contrib, self.i_pitch_contrib)
        self._i_x = 0.0
        self._i_y = 0.0
        return out

    def compute(
        self,
        ex: float,
        ey: float,
        vx: float,
        vy: float,
        roll_offset: float,
        pitch_offset: float,
        slew_target_override: tuple[float, float] | None = None,
        freeze_integrator: bool = False,
        i_limit_override: float | None = None,
        v_des: tuple[float, float] = (0.0, 0.0),
        ff: tuple[float, float] = (0.0, 0.0),
    ) -> PIDResult:
        """One PID step from error vector (ex, ey) and ball velocity.

        slew_target_override, when given, replaces the clamped PID command
        as the SLEW LIMITER's target (roll, pitch) for this step (rest
        mode parks the platform at level + trim + I through this path).
        The full PID pipeline still runs — every intermediate term is
        computed and reported — and the shared prev-command slew state
        advances toward the override instead, so the transition is
        rate-limited and a later normal step resumes continuously from
        the true last command.

        freeze_integrator holds the integral exactly (no step, no leak).
        i_limit_override widens/narrows the integral clamp for this call
        (home calibration uses the wide limit).

        v_des (mm/s, error space): the DESIRED ball velocity — the
        derivative term damps velocity ERROR (v_des - v), not raw
        velocity, so commanded trajectory motion is never braked (a
        zero v_des reproduces classic velocity damping exactly).
        ff (deg, error space): feedforward tilt added beside P/I/D
        (path centripetal pre-tilt), mapped to pitch/roll like the
        other x/y terms.
        """
        p_x = self.kp * ex
        p_y = self.kp * ey
        d_x = self.kd * (v_des[0] - vx)
        d_y = self.kd * (v_des[1] - vy)

        # Derivative term cap (FG-11)
        if self.d_term_limit_deg is not None:
            d_x = _clamp(d_x, -self.d_term_limit_deg, self.d_term_limit_deg)
            d_y = _clamp(d_y, -self.d_term_limit_deg, self.d_term_limit_deg)

        pd_x = p_x + d_x
        pd_y = p_y + d_y

        i_atten = self._step_integrator(
            ex, ey, pd_x + ff[0], pd_y + ff[1], roll_offset, pitch_offset,
            freeze_integrator, i_limit_override,
        )

        pitch_raw = pd_x + self._i_x + ff[0] + pitch_offset
        roll_raw = -(pd_y + self._i_y + ff[1]) + roll_offset
        roll_clamped = _clamp(roll_raw, -self.max_tilt_deg, self.max_tilt_deg)
        pitch_clamped = _clamp(pitch_raw, -self.max_tilt_deg, self.max_tilt_deg)

        # Slew limit (FG-11) — target is the clamped PD command, or the
        # caller's override (tilt-clamped too: nothing bypasses max_tilt).
        if slew_target_override is None:
            slew_roll, slew_pitch = roll_clamped, pitch_clamped
        else:
            slew_roll = _clamp(
                slew_target_override[0], -self.max_tilt_deg, self.max_tilt_deg
            )
            slew_pitch = _clamp(
                slew_target_override[1], -self.max_tilt_deg, self.max_tilt_deg
            )
        roll_cmd, pitch_cmd = self._apply_slew_limit(slew_roll, slew_pitch)

        i_limit = (
            self.i_limit_deg if i_limit_override is None
            else float(i_limit_override)
        )
        return PIDResult(
            p_term=(p_x, p_y),
            d_term=(d_x, d_y),
            pd_vec=(pd_x, pd_y),
            roll_raw=roll_raw,
            pitch_raw=pitch_raw,
            roll_clamped=roll_clamped,
            pitch_clamped=pitch_clamped,
            roll_cmd=roll_cmd,
            pitch_cmd=pitch_cmd,
            i_term=(self._i_x, self._i_y),
            i_sat=(
                abs(self._i_x) >= i_limit - 1e-9,
                abs(self._i_y) >= i_limit - 1e-9,
            ),
            i_atten=i_atten,
            i_frozen=freeze_integrator,
            ff_vec=ff,
        )

    def _step_integrator(
        self,
        ex: float,
        ey: float,
        pd_x: float,
        pd_y: float,
        roll_offset: float,
        pitch_offset: float,
        freeze: bool,
        i_limit_override: float | None,
    ) -> float:
        """Advance the integral one step; returns the taper weight used.

        The step happens BEFORE the raw command is formed, so this
        frame's integral feeds this frame's output (same-frame ordering,
        matching the pre-rework trim behavior). Anti-windup is per-axis
        and directional: with the provisional raw command (current I)
        tilt-clamped, integration is blocked only in the direction that
        would deepen the saturation — un-integrating out of it stays
        allowed.
        """
        err_mag = (ex * ex + ey * ey) ** 0.5
        span = max(1e-6, self.i_err_zero_mm - self.i_err_full_mm)
        weight = _clamp((self.i_err_zero_mm - err_mag) / span, 0.0, 1.0)
        # Low-side deadband (stiction hunting guard): no integration
        # below i_err_deadband_mm, ramping to full by 2x — the integral
        # must not demand accuracy the plate's static friction cannot
        # hold, or it winds up, snaps the ball loose, and hunts forever.
        dead = self.i_err_deadband_mm
        if dead > 0.0:
            weight *= _clamp((err_mag - dead) / max(1e-6, dead), 0.0, 1.0)
        if self.ki <= 0.0 or freeze:
            # No motion this step: the rate estimate decays toward zero
            # (fixed ~0.3 s factor at the nominal 30 Hz cadence).
            self._i_rate_ema *= 0.9
            return weight

        now = self._clock()
        if self._i_prev_time is None:
            # First call seeds the timebase only — the first frame
            # contributes no integral (exact-PD single-call behavior
            # is preserved for callers/tests that expect it).
            self._i_prev_time = now
            return weight
        dt = _clamp(now - self._i_prev_time, 1e-4, 0.1)
        self._i_prev_time = now
        i_x_before = self._i_x
        i_y_before = self._i_y

        # Provisional raw commands with the CURRENT integral decide the
        # anti-windup directions for this step.
        pitch_prov = pd_x + self._i_x + pitch_offset
        roll_prov = -(pd_y + self._i_y) + roll_offset

        # x axis drives pitch directly: growth blocked in the sign that
        # deepens a pitch saturation.
        allow_x = not (
            (pitch_prov >= self.max_tilt_deg and ex > 0.0)
            or (pitch_prov <= -self.max_tilt_deg and ex < 0.0)
        )
        # y axis drives roll NEGATED (roll_raw = -(pd_y + i_y) + offset):
        # i_y growth (ey > 0) pushes roll_raw DOWN, so a low-saturated
        # roll blocks positive ey integration and a high-saturated roll
        # blocks negative ey integration.
        allow_y = not (
            (roll_prov <= -self.max_tilt_deg and ey > 0.0)
            or (roll_prov >= self.max_tilt_deg and ey < 0.0)
        )

        step = self.ki * weight * dt
        if allow_x:
            self._i_x += step * ex
        if allow_y:
            self._i_y += step * ey

        if self.i_leak_tau_s > 0.0:
            decay = 1.0 - dt / self.i_leak_tau_s
            self._i_x *= decay
            self._i_y *= decay

        i_limit = (
            self.i_limit_deg if i_limit_override is None
            else float(i_limit_override)
        )
        self._i_x = _clamp(self._i_x, -i_limit, i_limit)
        self._i_y = _clamp(self._i_y, -i_limit, i_limit)

        # Net rate EMA (post-leak, post-clamp): ~0 at the leak
        # equilibrium and while parked at the limit.
        di_x = self._i_x - i_x_before
        di_y = self._i_y - i_y_before
        rate = ((di_x * di_x + di_y * di_y) ** 0.5) / dt
        alpha = _clamp(dt / 0.3, 0.0, 1.0)
        self._i_rate_ema += alpha * (rate - self._i_rate_ema)
        return weight

    def _apply_slew_limit(self, roll: float, pitch: float) -> tuple[float, float]:
        now = self._clock()
        if self._prev_cmd_time is None:
            self._prev_cmd_time = now
            self._prev_roll_cmd = roll
            self._prev_pitch_cmd = pitch
            return roll, pitch
        dt = max(1e-4, now - self._prev_cmd_time)
        self._prev_cmd_time = now
        max_step = self.max_tilt_rate_deg_s * dt
        roll_out = self._prev_roll_cmd + _clamp(
            roll - self._prev_roll_cmd, -max_step, max_step
        )
        pitch_out = self._prev_pitch_cmd + _clamp(
            pitch - self._prev_pitch_cmd, -max_step, max_step
        )
        self._prev_roll_cmd = roll_out
        self._prev_pitch_cmd = pitch_out
        return roll_out, pitch_out
