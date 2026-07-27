"""
control/orbit.py

HarmonicOrbit — feedforward-driven smooth circular motion (2026-07-27).

A SEPARATE mode from the carrot-chasing PathFollower: the reference is
CLOCK-driven (constant angular rate, no error-paced advance — the
pacing law's error->speed coupling is the jank source in path-follow
circles), the platform plays a smooth rotating tilt pattern tuned to
the plant, and feedback is demoted to a trim role.

Physics: for x_r = R(cos phi, sin phi), phi_dot = omega = v/R, exact
tracking needs the centripetal tilt omega^2*R/g_eff as a constant-
magnitude ROTATING vector (a coning motion), phase-ADVANCED by
omega*lead_s to cancel the actuation delay. At rig numbers (r=50,
v=40) that is only ~0.19 deg — the plate-specific disturbances DWARF
it (bowl warp alone needs ~0.28 deg at r=50), and while orbiting they
are PERIODIC at omega. So the heart of the mode is a LEARNED per-phase
correction table (iterative learning control): ORBIT_ILC_BINS tilt
vectors indexed by reference phase, refined every visit.

ILC update law (the sign subtlety): with feedback active at these
gains, the feedback stiffness g*kp_eff dominates ball inertia omega^2
at the fundamental, so the position error is ALIGNED with the
disturbance (rig confirmation: the center-attracting bowl parks the
ball ~8 mm INSIDE the circle — the needed correction is outward,
toward the reference). The per-visit update is

    delta_c_k = mu * gamma * (ref - ball),
    gamma = kp_eff - omega^2 / g_eff        [deg/mm]

which is the in-phase inverse of the closed-loop correction->error
map; it reduces to the inertia-dominated law mu*(omega^2/g)*(ball-ref)
as kp_eff -> 0 and self-flips sign at the scaled-gain resonance
omega = sqrt(g*kp_eff). The effective orbit speed is soft-clamped so
omega stays below 0.85x that resonance.

Learning stability (sim-caught, twice): (a) each write fires once per
BIN TRANSIT using the mean error across the transit — per-frame
point-writes hit each bin ~10x per lap and turned the intended 0.75
per-lap contraction into near-instant (and, for wrong-signed
harmonics, explosive) updates; (b) writes are spread over neighboring
bins with a Gaussian kernel (ORBIT_ILC_WRITE_SIGMA_BINS), band-
limiting the table below the resonance — harmonics n >= 3 at these
numbers have a SIGN-FLIPPED closed-loop response, and unfiltered
point-writes pumped n=3-4 to the clamp (divergence after 4 laps).
Remaining aids: per-LAP table leak, per-bin norm clamp, a 1-2-1
circular smoothing pass once per lap, and linear interpolation
between bins on OUTPUT (a 15-deg-bin staircase would thump the
servos).

Phase bookkeeping (delay attribution): OUTPUT reads the table at
phi + omega*lead_s; UPDATE writes at phi. The bin read lead_s ago is
exactly the bin whose tilt is acting on the ball now, so writing the
currently-observed error at the CURRENT phase charges the right bin.
The learned error therefore uses the FILTERED ball position, not the
predicted one.

States: idle -> entrain (seed at the ball's current angle/radius, ramp
omega and r over ORBIT_SPINUP_S — the ball is picked up where it is,
never dragged) -> track (learning on) -> recover (error tripwire:
freeze the table, re-entrain from the current ball state) -> track.

Pure Python + numpy, injected clock, no Qt (PathFollower precedent).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from settings import (
    ORBIT_CONE_CENTER_GAIN,
    ORBIT_CONE_DC_CLAMP_DEG,
    ORBIT_CONE_OMEGA_MAX_RAD_S,
    ORBIT_CONE_OMEGA_RAD_S,
    ORBIT_CONE_ONLY,
    ORBIT_CONE_TILT_DEG,
    ORBIT_CONE_TILT_MAX_DEG,
    ORBIT_CONE_TILT_MIN_DEG,
    ORBIT_CONE_WARP_C,
    ORBIT_ENTRAIN_MIN_RADIUS_MM,
    ORBIT_FB_GAIN_SCALE,
    ORBIT_FF_TILT_MAX_DEG,
    ORBIT_ILC_BINS,
    ORBIT_ILC_CLAMP_DEG,
    ORBIT_ILC_LEAK,
    ORBIT_ILC_MU,
    ORBIT_ILC_WRITE_SIGMA_BINS,
    ORBIT_LEARN_GATE_MM,
    ORBIT_PHASE_GOV_ENTRAIN_PER_S,
    ORBIT_PHASE_GOV_TRACK_CAP,
    ORBIT_PHASE_GOV_TRACK_PER_S,
    ORBIT_RADIUS_MAX_MM,
    ORBIT_RADIUS_MIN_MM,
    ORBIT_RADIUS_MM,
    ORBIT_RECOVER_FRAMES,
    ORBIT_RECOVER_MM,
    ORBIT_SPINUP_S,
    ORBIT_TRACK_ENTRY_ERR_MM,
    PATH_SPEED_MAX_MM_S,
    PATH_SPEED_MIN_MM_S,
    PATH_SPEED_MM_S,
)

STATE_IDLE = "idle"
STATE_ENTRAIN = "entrain"
STATE_TRACK = "track"
STATE_RECOVER = "recover"
STATE_CONE = "cone"

_TWO_PI = 2.0 * math.pi
# Fraction of the scaled-gain resonance the orbit omega may reach: past
# the resonance the ILC gain gamma crosses zero and learning stalls.
_RESONANCE_MARGIN = 0.85


@dataclass(frozen=True)
class OrbitCommand:
    """One frame's orbit output for the controller."""
    target_x_mm: float
    target_y_mm: float
    v_des_mm_s: tuple[float, float]
    ff_deg: tuple[float, float]      # analytic + learned, PIDCore ff space
    err_mm: float                    # |ball - reference|


class HarmonicOrbit:
    """Clock-driven circular reference + feedforward + per-phase ILC.

    See the module docstring for the physics and the state machine."""

    def __init__(
        self,
        clock: Callable[[], float],
        radius_mm: float = ORBIT_RADIUS_MM,
        speed_mm_s: float = PATH_SPEED_MM_S,
    ) -> None:
        self._clock = clock
        self._r_target = self._clamp_radius(radius_mm)
        self._speed_mm_s = self._clamp_speed(speed_mm_s)

        self.fb_gain_scale = float(ORBIT_FB_GAIN_SCALE)
        self.spinup_s = float(ORBIT_SPINUP_S)
        self.ff_tilt_max_deg = float(ORBIT_FF_TILT_MAX_DEG)
        self.ilc_bins = int(ORBIT_ILC_BINS)
        self.ilc_mu = float(ORBIT_ILC_MU)
        self.ilc_leak = float(ORBIT_ILC_LEAK)
        self.ilc_clamp_deg = float(ORBIT_ILC_CLAMP_DEG)
        self.learn_gate_mm = float(ORBIT_LEARN_GATE_MM)
        self.recover_mm = float(ORBIT_RECOVER_MM)
        self.recover_frames = int(ORBIT_RECOVER_FRAMES)
        self.gov_track_per_s = float(ORBIT_PHASE_GOV_TRACK_PER_S)
        self.gov_track_cap = float(ORBIT_PHASE_GOV_TRACK_CAP)
        self.gov_entrain_per_s = float(ORBIT_PHASE_GOV_ENTRAIN_PER_S)
        self.track_entry_err_mm = float(ORBIT_TRACK_ENTRY_ERR_MM)
        # Cone mode: open-loop rotating tilt, feedback off (see the
        # settings block). The closed-loop machinery below stays
        # available with cone_only=False.
        self.cone_only = bool(ORBIT_CONE_ONLY)
        self.cone_tilt_deg = float(ORBIT_CONE_TILT_DEG)
        self.cone_warp_c = float(ORBIT_CONE_WARP_C)
        self.cone_center_gain = float(ORBIT_CONE_CENTER_GAIN)
        self.cone_dc_clamp_deg = float(ORBIT_CONE_DC_CLAMP_DEG)
        self.cone_omega_rad_s = float(ORBIT_CONE_OMEGA_RAD_S)  # 0 = auto
        self._cone_amp = 0.0
        self._center_x = 0.0     # slow EMA of the ball = orbit center
        self._center_y = 0.0
        self._dc_x = 0.0         # DC tilt correction steering the center
        self._dc_y = 0.0

        self._state = STATE_IDLE
        self._seeded = False
        self._phi = 0.0
        self._omega = 0.0
        self._r = 0.0
        self._r_rate_hold = 0.0        # fixed r ramp rate, set at seed
        self._omega_target_eff = 0.0   # last resonance-clamped target
        self._last_t: float | None = None
        self._lap = 0
        self._recover_count = 0
        self._err_frames = 0
        self._learning = False
        self._last_err_mm = 0.0
        self._last_ff_deg = 0.0
        self._last_ilc_deg = 0.0
        self._cx = np.zeros(self.ilc_bins)
        self._cy = np.zeros(self.ilc_bins)

        # Bin-transit accumulator: mean error while traversing one bin,
        # flushed as a single kernel-spread write on bin exit.
        self._bin_cur: int | None = None
        self._bin_err_x = 0.0
        self._bin_err_y = 0.0
        self._bin_err_n = 0
        # Circular Gaussian write kernel, normalized to sum 1.
        sigma = max(0.5, float(ORBIT_ILC_WRITE_SIGMA_BINS))
        idx = np.arange(self.ilc_bins)
        dist = np.minimum(idx, self.ilc_bins - idx).astype(float)
        kernel = np.exp(-0.5 * (dist / sigma) ** 2)
        self._write_kernel = kernel / kernel.sum()

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp_radius(radius_mm: float) -> float:
        return max(ORBIT_RADIUS_MIN_MM, min(ORBIT_RADIUS_MAX_MM, float(radius_mm)))

    @staticmethod
    def _clamp_speed(mm_s: float) -> float:
        return max(PATH_SPEED_MIN_MM_S, min(PATH_SPEED_MAX_MM_S, float(mm_s)))

    def set_radius(self, radius_mm: float) -> None:
        """Change the reference radius. RESETS the correction table —
        the learned warp correction is radius-specific."""
        self._r_target = self._clamp_radius(radius_mm)
        self._cx[:] = 0.0
        self._cy[:] = 0.0
        if self._seeded:
            # Mid-orbit change: ramp to the new radius at the entrain rate.
            self._r_rate_hold = abs(self._r_target - self._r) / max(
                0.5, self.spinup_s
            )

    def set_speed(self, mm_s: float) -> None:
        """Change the tangential speed. The table is KEPT: the dominant
        warp part is omega-independent and the leak ages out the rest."""
        self._speed_mm_s = self._clamp_speed(mm_s)

    def set_cone_tilt(self, deg: float) -> None:
        """Live cone-amplitude change (GUI spinbox); the running cone
        slews to the new amplitude at the spin-up rate."""
        self.cone_tilt_deg = max(
            ORBIT_CONE_TILT_MIN_DEG, min(ORBIT_CONE_TILT_MAX_DEG, float(deg))
        )

    def set_cone_omega(self, rad_s: float) -> None:
        """Live angular-frequency override (GUI spinbox); 0 = auto
        (derived from tilt + radius via the warp-spring physics)."""
        self.cone_omega_rad_s = max(
            0.0, min(ORBIT_CONE_OMEGA_MAX_RAD_S, float(rad_s))
        )

    @property
    def radius_mm(self) -> float:
        return self._r_target

    @property
    def speed_mm_s(self) -> float:
        return self._speed_mm_s

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Arm the orbit; the seed happens on the FIRST update() with a
        ball, so the platform does not move on start (PathFollower's
        armed-seed pattern)."""
        if self._state != STATE_IDLE:
            return True
        self._state = STATE_ENTRAIN
        self._seeded = False
        self._lap = 0
        self._recover_count = 0
        self._err_frames = 0
        return True

    def stop(self) -> None:
        """Back to idle. The correction table is kept — a restart on the
        same radius resumes with everything already learned."""
        self._state = STATE_IDLE
        self._seeded = False
        self._last_t = None

    @property
    def active(self) -> bool:
        return self._state != STATE_IDLE

    @property
    def state(self) -> str:
        return self._state

    @property
    def feedback_scale(self) -> float:
        """P/D gain scale the controller applies while this mode is
        active: 0 in cone mode (pure open-loop cone — no ball chasing),
        else the configured trim scale."""
        return 0.0 if self.cone_only else self.fb_gain_scale

    @property
    def wants_integral_frozen(self) -> bool:
        """The controller freezes the PID integral when this is True —
        the whole TRACK state. Division of labor (both sides sim-
        caught): a RUNNING integral at orbit frequency has more gain
        than the scaled P-term with 90 deg lag, so it chases the
        rotating error itself and fights the learning table
        (convergence plateaued); and freezing it mid-chase later
        snapshots a rotating component as a bogus DC (a 3x error jump
        at the handoff). So the integral runs only while the reference
        is not yet rotating fast (entrain/recover — it grabs the DC
        bias there), freezes for all of TRACK, and the table's n0
        component finishes whatever DC remains. Cone mode freezes it
        for the whole session — an integrator watching an orbiting
        ball it isn't allowed to correct would only wind up."""
        if self.cone_only:
            return self._seeded and self._state != STATE_IDLE
        return self._state == STATE_TRACK

    # ------------------------------------------------------------------
    # Per-cycle
    # ------------------------------------------------------------------

    def update(
        self,
        ball_x: float,
        ball_y: float,
        lead_s: float,
        g_eff: float,
        kp_eff: float,
    ) -> OrbitCommand:
        """One control cycle; see the module docstring for the flow.

        lead_s: total actuation delay (the controller's _predict_s);
        g_eff: mm/s^2 per deg; kp_eff: the SCALED feedback kp actually
        flying (kp * ORBIT_FB_GAIN_SCALE) — both feed the ILC gain."""
        g_eff = max(1.0, float(g_eff))
        if self._state == STATE_IDLE:
            return OrbitCommand(ball_x, ball_y, (0.0, 0.0), (0.0, 0.0), 0.0)

        if not self._seeded:
            self._seed(ball_x, ball_y)
            if self.cone_only:
                # The expected ball point (anti-phase to the tilt)
                # should START at the ball, so the tilt begins by
                # pushing from where the ball already is.
                self._phi = self._wrap_pi(self._phi + math.pi)
                if self._phi < 0.0:
                    self._phi += _TWO_PI
                self._cone_amp = 0.0
                self._center_x = ball_x
                self._center_y = ball_y
                self._dc_x = 0.0
                self._dc_y = 0.0
            return OrbitCommand(
                self._r * math.cos(self._phi),
                self._r * math.sin(self._phi),
                (0.0, 0.0), (0.0, 0.0), 0.0,
            )

        if self.cone_only:
            return self._update_cone(ball_x, ball_y, g_eff)

        now = self._clock()
        prev_t = self._last_t if self._last_t is not None else now
        dt = min(0.1, max(1e-4, now - prev_t))
        self._last_t = now

        # Soft resonance clamp on the commanded angular rate.
        omega_res = math.sqrt(g_eff * max(1e-6, kp_eff))
        omega_target = min(
            self._speed_mm_s / max(1.0, self._r_target),
            _RESONANCE_MARGIN * omega_res,
        )
        self._omega_target_eff = omega_target

        # Slew omega and r toward their targets (the entrain/spin-up
        # ramps); the APPLIED rates feed the feedforward below.
        omega_rate = omega_target / max(0.5, self.spinup_s)
        alpha = self._slew_toward(omega_target, "_omega", omega_rate, dt)
        r_rate = self._r_rate_hold if self._r_rate_hold > 0.0 else 1.0
        r_dot = self._slew_toward(self._r_target, "_r", r_rate, dt)

        # Phase governor (rig-caught): under real stiction the pure
        # clock reference outruns the ball and the trailing error grows
        # into the recover tripwire — churn. A weak PLL slews the
        # reference phase toward the ball's actual angle: sustained lag
        # is absorbed; the track-state bandwidth (~0.02 Hz) is far
        # below jank frequencies so measurement jitter cannot couple
        # into the reference (the carrot-pacing failure mode). During
        # entrain/recover the lock is STRONG — the reference stays
        # glued to the ball until capture.
        gov = 0.0
        r_ball = math.hypot(ball_x, ball_y)
        if r_ball > 10.0:
            dphi = self._wrap_pi(math.atan2(ball_y, ball_x) - self._phi)
            if self._state == STATE_TRACK:
                gov = self.gov_track_per_s * dphi
                cap = self.gov_track_cap * max(self._omega, 1e-3)
                gov = max(-cap, min(cap, gov))
            else:
                gov = self.gov_entrain_per_s * dphi
                gov = max(-1.5, min(1.5, gov))
        phi_rate = self._omega + gov

        # Advance phase; a wrap is one lap and triggers the smoothing
        # pass over the correction table.
        self._phi += phi_rate * dt
        if self._phi >= _TWO_PI:
            self._phi -= _TWO_PI
            self._lap += 1
            self._smooth_table()
        elif self._phi < 0.0:
            self._phi += _TWO_PI

        ref_x, ref_y, v_des = self._reference(r_dot, phi_rate)
        ex = ball_x - ref_x
        ey = ball_y - ref_y
        err = math.hypot(ex, ey)
        self._last_err_mm = err

        reseeded = self._step_state_machine(err, ball_x, ball_y)
        if reseeded:
            # The seed moved the reference onto the ball: recompute this
            # frame's output from the fresh state (omega=0, no ramps) so
            # the command is continuous with the new entrainment.
            alpha = 0.0
            ref_x, ref_y, v_des = self._reference(0.0, 0.0)
            ex = ball_x - ref_x
            ey = ball_y - ref_y
            err = math.hypot(ex, ey)
            self._last_err_mm = err

        # Feedforward at the ADVANCED phase: the tilt commanded now acts
        # lead_s later, when the reference is at phi + omega*lead_s.
        phi_a = self._phi + self._omega * max(0.0, float(lead_s))
        cos_a = math.cos(phi_a)
        sin_a = math.sin(phi_a)
        a_cent = self._omega * self._omega * self._r
        a_tan = alpha * self._r
        ff_x = (-a_cent * cos_a - a_tan * sin_a) / g_eff
        ff_y = (-a_cent * sin_a + a_tan * cos_a) / g_eff
        ilc_x, ilc_y = self._ilc_read(phi_a)
        self._last_ilc_deg = math.hypot(ilc_x, ilc_y)
        ff_x += ilc_x
        ff_y += ilc_y
        ff_mag = math.hypot(ff_x, ff_y)
        if ff_mag > self.ff_tilt_max_deg:
            scale = self.ff_tilt_max_deg / ff_mag
            ff_x *= scale
            ff_y *= scale
            ff_mag = self.ff_tilt_max_deg
        self._last_ff_deg = ff_mag

        self._learning = (
            self._state == STATE_TRACK and err <= self.learn_gate_mm
        )
        self._ilc_step(-ex, -ey, g_eff, kp_eff)

        return OrbitCommand(ref_x, ref_y, v_des, (ff_x, ff_y), err)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def telemetry(self) -> dict[str, Any]:
        """Terms-dict fragment; identical key set in every state."""
        return {
            "orbit_active": self.active,
            "orbit_state": self._state,
            "orbit_phase": self._phi,
            "orbit_omega": self._omega,
            "orbit_r_mm": self._r,
            "orbit_err_mm": self._last_err_mm,
            "orbit_ff_deg": self._last_ff_deg,
            "orbit_ilc_deg": self._last_ilc_deg,
            "orbit_lap": self._lap,
            "orbit_recover_count": self._recover_count,
            "orbit_learning": self._learning,
        }

    # ------------------------------------------------------------------
    # Cone mode (open loop)
    # ------------------------------------------------------------------

    def update_blind(self, g_eff: float) -> OrbitCommand:
        """Cone step with NO ball: self-seeds if needed, advances the
        cone, HOLDS the center corrector (no data to estimate a center
        from), and reports the expected ball point. Lets the rig run
        and inspect the cone with the ball off the platform, and keeps
        the motion continuous between camera frames."""
        g_eff = max(1.0, float(g_eff))
        if self._state == STATE_IDLE or not self.cone_only:
            return OrbitCommand(0.0, 0.0, (0.0, 0.0), (0.0, 0.0), 0.0)
        if not self._seeded:
            self._phi = 0.0
            self._cone_amp = 0.0
            self._center_x = 0.0
            self._center_y = 0.0
            self._dc_x = 0.0
            self._dc_y = 0.0
            self._seeded = True
            self._last_t = self._clock()
        return self._cone_step(None, None, g_eff)

    def _update_cone(
        self, ball_x: float, ball_y: float, g_eff: float
    ) -> OrbitCommand:
        return self._cone_step(ball_x, ball_y, g_eff)

    def _cone_step(
        self, ball_x: float | None, ball_y: float | None, g_eff: float
    ) -> OrbitCommand:
        """Pure open-loop cone: tilt vector of amplitude cone_tilt_deg
        rotating at omega = sqrt(g_eff*(warp_c + A/R)) — the plate's
        bowl warp acts as a central spring (omega_n^2 = g*warp_c), so
        the driven orbit radius is g*A/|omega^2 - omega_n^2|, ridden
        anti-phase above the warp resonance and in-phase below it. A
        manual cone_omega_rad_s (> 0) overrides the derived rate. The
        BALL input (None when running blind) never steers the cone —
        feedback is off; it only feeds the slow center corrector and
        the err telemetry."""
        now = self._clock()
        prev_t = self._last_t if self._last_t is not None else now
        dt = min(0.1, max(1e-4, now - prev_t))
        self._last_t = now

        if self.cone_omega_rad_s > 0.0:
            omega = self.cone_omega_rad_s
        else:
            omega = math.sqrt(
                g_eff * (
                    self.cone_warp_c
                    + self.cone_tilt_deg / max(20.0, self._r_target)
                )
            )
        # Amplitude ramp over the spin-up window (the plate eases into
        # the cone; the expected ring grows with it).
        rate = self.cone_tilt_deg / max(0.5, self.spinup_s)
        self._slew_toward(self.cone_tilt_deg, "_cone_amp", rate, dt)
        if abs(self._cone_amp - self.cone_tilt_deg) <= 1e-9:
            self._state = STATE_CONE
        else:
            self._state = STATE_ENTRAIN
        self._omega = omega

        self._phi += omega * dt
        if self._phi >= _TWO_PI:
            self._phi -= _TWO_PI
            self._lap += 1

        # Center corrector (the only feedback here): the orbit center
        # is the slow EMA of the ball position — the rotating component
        # averages out over a lap, so this cannot react to (or jitter
        # against) the orbital motion itself. INTEGRAL-ONLY and slow by
        # design: the center is itself a weakly damped oscillator at
        # the warp frequency (~1 rad/s), and any faster/proportional
        # feedback through the estimator's lag turns into NEGATIVE
        # damping and pumps it (sim-caught: a P term at the physically
        # "correct" gain flung the center off exponentially). Engaged
        # only once the cone is fully spun up (rig-caught: estimating
        # during the spiral-out transient poisons the center estimate
        # and the correction chases it — the logged orbit ran clean
        # 47-49 mm circles around a 13-26 mm wandering center).
        # Running blind (no ball) everything HOLDS.
        if (
            ball_x is not None and ball_y is not None
            and self._state == STATE_CONE
        ):
            lap_period = _TWO_PI / max(omega, 1e-3)
            ema_alpha = dt / max(1.5 * lap_period, 1.0)
            self._center_x += ema_alpha * (ball_x - self._center_x)
            self._center_y += ema_alpha * (ball_y - self._center_y)
            self._dc_x -= self.cone_center_gain * self._center_x * dt
            self._dc_y -= self.cone_center_gain * self._center_y * dt
            dc_mag = math.hypot(self._dc_x, self._dc_y)
            if dc_mag > self.cone_dc_clamp_deg:
                scale = self.cone_dc_clamp_deg / dc_mag
                self._dc_x *= scale
                self._dc_y *= scale

        ff = (
            self._cone_amp * math.cos(self._phi) + self._dc_x,
            self._cone_amp * math.sin(self._phi) + self._dc_y,
        )
        # Predicted ring: anti-phase above the warp resonance, in-phase
        # below; near the resonance the linear prediction diverges —
        # display-capped at the platform edge.
        denom = omega * omega - g_eff * self.cone_warp_c
        if abs(denom) < 1e-3:
            r_exp = 120.0
            anti_phase = True
        else:
            r_exp = min(120.0, g_eff * self._cone_amp / abs(denom))
            anti_phase = denom > 0.0
        self._r = r_exp
        sign = -1.0 if anti_phase else 1.0
        tx = sign * r_exp * math.cos(self._phi)
        ty = sign * r_exp * math.sin(self._phi)
        if ball_x is not None and ball_y is not None:
            err = abs(math.hypot(ball_x, ball_y) - r_exp)
        else:
            err = 0.0
        self._last_err_mm = err
        self._last_ff_deg = self._cone_amp
        self._last_ilc_deg = math.hypot(self._dc_x, self._dc_y)
        self._learning = False
        return OrbitCommand(tx, ty, (0.0, 0.0), ff, err)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        """Wrap to (-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _reference(
        self, r_dot: float, phi_rate: float
    ) -> tuple[float, float, tuple[float, float]]:
        """Current reference point + desired velocity (phi_rate is the
        ACTUAL phase rate incl. the governor)."""
        cos_p = math.cos(self._phi)
        sin_p = math.sin(self._phi)
        ref_x = self._r * cos_p
        ref_y = self._r * sin_p
        v_des = (
            -phi_rate * self._r * sin_p + r_dot * cos_p,
            phi_rate * self._r * cos_p + r_dot * sin_p,
        )
        return ref_x, ref_y, v_des

    def _seed(self, ball_x: float, ball_y: float) -> None:
        """Pick the ball up where it is: phase from its current angle,
        radius from its current radius (floored), omega from zero."""
        r_ball = math.hypot(ball_x, ball_y)
        self._phi = math.atan2(ball_y, ball_x) if r_ball > 1.0 else 0.0
        if self._phi < 0.0:
            self._phi += _TWO_PI
        self._r = max(
            ORBIT_ENTRAIN_MIN_RADIUS_MM,
            min(ORBIT_RADIUS_MAX_MM, r_ball),
        )
        # Fixed r ramp rate for the whole entrain (recomputing from the
        # shrinking distance would asymptote instead of arriving).
        self._r_rate_hold = abs(self._r_target - self._r) / max(0.5, self.spinup_s)
        self._omega = 0.0
        self._seeded = True
        self._last_t = self._clock()
        self._err_frames = 0

    def _slew_toward(
        self, target: float, attr: str, rate: float, dt: float
    ) -> float:
        """Move self.<attr> toward target at |rate|; returns the APPLIED
        signed rate this frame (0 when already there)."""
        current = float(getattr(self, attr))
        delta = target - current
        step = rate * dt
        if abs(delta) <= step:
            setattr(self, attr, target)
            return delta / dt if dt > 0 else 0.0
        applied = math.copysign(step, delta)
        setattr(self, attr, current + applied)
        return applied / dt

    def _step_state_machine(
        self, err: float, ball_x: float, ball_y: float
    ) -> bool:
        """Advance entrain/track/recover; returns True when a RECOVER
        trip re-seeded the reference this frame."""
        if self._state in (STATE_ENTRAIN, STATE_RECOVER):
            at_speed = abs(self._omega - self._omega_target_eff) < 1e-3
            at_radius = abs(self._r - self._r_target) < 0.5
            # Capture gate: TRACK (which freezes the integral and
            # starts learning) is entered only with the ball actually
            # near the reference — never from a bad state.
            captured = err <= self.track_entry_err_mm
            if at_speed and at_radius and captured:
                self._state = STATE_TRACK
                self._err_frames = 0
            return False
        # TRACK: error tripwire -> re-entrain (never drag the ball).
        if err > self.recover_mm:
            self._err_frames += 1
            if self._err_frames >= self.recover_frames:
                self._state = STATE_RECOVER
                self._recover_count += 1
                self._seed(ball_x, ball_y)
                return True
        else:
            self._err_frames = 0
        return False

    # --- ILC table ---

    def _bin_pos(self, phi: float) -> float:
        return (phi % _TWO_PI) / _TWO_PI * self.ilc_bins

    def _ilc_read(self, phi: float) -> tuple[float, float]:
        """Linear interpolation between the two adjacent bins."""
        pos = self._bin_pos(phi)
        k0 = int(pos) % self.ilc_bins
        k1 = (k0 + 1) % self.ilc_bins
        frac = pos - int(pos)
        return (
            (1.0 - frac) * self._cx[k0] + frac * self._cx[k1],
            (1.0 - frac) * self._cy[k0] + frac * self._cy[k1],
        )

    def _ilc_gain(self, g_eff: float, kp_eff: float) -> float:
        """deg/mm error->correction gain: the in-phase inverse of the
        closed-loop correction->error map (see module docstring)."""
        return float(kp_eff) - self._omega * self._omega / g_eff

    def _ilc_step(
        self, ref_minus_ball_x: float, ref_minus_ball_y: float,
        g_eff: float, kp_eff: float,
    ) -> None:
        """Accumulate the error over the current bin transit; flush ONE
        kernel-spread write when the phase enters a new bin (per-lap
        contraction as designed — see the module docstring)."""
        k_now = int(self._bin_pos(self._phi)) % self.ilc_bins
        if self._learning:
            if k_now != self._bin_cur:
                self._flush_bin(g_eff, kp_eff)
                self._bin_cur = k_now
            self._bin_err_x += ref_minus_ball_x
            self._bin_err_y += ref_minus_ball_y
            self._bin_err_n += 1
        else:
            # Not learning: drop any partial accumulation (stale data
            # from before a gate trip must not be written later).
            self._bin_cur = k_now
            self._bin_err_x = 0.0
            self._bin_err_y = 0.0
            self._bin_err_n = 0

    def _flush_bin(self, g_eff: float, kp_eff: float) -> None:
        if self._bin_cur is None or self._bin_err_n == 0:
            self._bin_err_x = 0.0
            self._bin_err_y = 0.0
            self._bin_err_n = 0
            return
        mean_x = self._bin_err_x / self._bin_err_n
        mean_y = self._bin_err_y / self._bin_err_n
        gamma = self._ilc_gain(g_eff, kp_eff)
        kernel = np.roll(self._write_kernel, self._bin_cur)
        self._cx += self.ilc_mu * gamma * mean_x * kernel
        self._cy += self.ilc_mu * gamma * mean_y * kernel
        self._clamp_table()
        self._bin_err_x = 0.0
        self._bin_err_y = 0.0
        self._bin_err_n = 0

    def _clamp_table(self) -> None:
        mag = np.hypot(self._cx, self._cy)
        over = mag > self.ilc_clamp_deg
        if np.any(over):
            scale = np.ones_like(mag)
            scale[over] = self.ilc_clamp_deg / mag[over]
            self._cx *= scale
            self._cy *= scale

    def _smooth_table(self) -> None:
        """Once per lap: 1-2-1 circular smoothing (damps high-harmonic
        content whose closed-loop response sign flips above the
        resonance) plus the table leak (mis-learned corrections age
        out; a restart on the same radius keeps what is still valid)."""
        for table in (self._cx, self._cy):
            table[:] = (
                0.25 * np.roll(table, 1)
                + 0.5 * table
                + 0.25 * np.roll(table, -1)
            ) * (1.0 - self.ilc_leak)
