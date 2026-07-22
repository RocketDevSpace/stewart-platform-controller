"""
control/ball_controller.py

BallController — facade over the decomposed ball-balancing controller
(M10). Constructs and coordinates the four components:

  SetpointArbiter (control/setpoint.py)   target ownership + change time
  PDCore          (control/pd_core.py)    PD math, d-cap, tilt clamp, slew
  AutoTrim        (control/auto_trim.py)  trim offsets + slow integrator
  PDAutotuner     (control/autotune.py)   step-test gain tuning sessions

The public method surface and the compute_with_terms terms-dict contract
are unchanged from the pre-decomposition monolith (the GUI reads those
keys — they are frozen, see tests/test_ball_controller_characterization).
"""
import math
import time
from collections.abc import Callable

from control.auto_trim import AutoTrim
from control.autotune import PDAutotuner
from control.path_follower import PathFollower
from control.patterns import Path
from control.pd_core import PDCore
from control.rest_gate import STATE_RESTING, RestGate
from control.setpoint import SetpointArbiter
from core.platform_state import BallState
from settings import (
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
    PD_D_TERM_LIMIT_DEG,
    PD_MAX_TILT_RATE_DEG_S,
    REST_MODE_ENABLED,
)


class BallController:
    """
    PD controller for ball balancing on Stewart platform.

    Inputs:
        Ball position + velocity (mm, mm/s)

    Outputs:
        Desired platform tilt angles (degrees)
            roll_deg  -> rotation about X axis
            pitch_deg -> rotation about Y axis
    """

    def __init__(
        self,
        kp: float = PD_DEFAULT_KP,
        kd: float = PD_DEFAULT_KD,
        max_tilt_deg: float = 10.0,
        max_tilt_rate_deg_s: float = PD_MAX_TILT_RATE_DEG_S,
        d_term_limit_deg: float | None = PD_D_TERM_LIMIT_DEG,
        roll_offset: float = 0.0,
        pitch_offset: float = 0.0,
        auto_trim_enabled: bool = False,
        rest_mode_enabled: bool | None = None,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        self.enabled = True

        # PD math + d-cap + tilt clamp + slew limiter (FG-11)
        self._pd = PDCore(
            kp=kp,
            kd=kd,
            max_tilt_deg=max_tilt_deg,
            max_tilt_rate_deg_s=max_tilt_rate_deg_s,
            d_term_limit_deg=d_term_limit_deg,
            clock=clock,
        )

        # Target — owned by the arbiter (manual vs autotune override) (FG-9)
        self._arbiter = SetpointArbiter(
            float(BALL_TARGET_DEFAULT_X_MM), float(BALL_TARGET_DEFAULT_Y_MM), clock
        )

        # Trim offsets + integrator + home-cal state (FG-9)
        self._auto_trim = AutoTrim(roll_offset, pitch_offset, clock)
        self.auto_trim_enabled = bool(auto_trim_enabled)

        # Near-target rest mode (perf pass) — parks the platform at
        # level + trim once the ball has sat quietly at the target.
        self._rest_gate = RestGate(
            clock=clock,
            enabled=(
                bool(REST_MODE_ENABLED)
                if rest_mode_enabled is None
                else bool(rest_mode_enabled)
            ),
        )

        # PD autotune state machine (FG-10)
        self._autotuner = PDAutotuner(self._arbiter, clock)
        self._autotuner.clear_suggestion(kp, kd)
        # Auto-trim flag stashed while an autotune session runs; the
        # stash/restore coordination stays here (single owner of the flag).
        self._pd_autotune_prev_auto_trim_enabled = self.auto_trim_enabled

        # Path following (feat/path): drives the arbiter override channel
        # (mutually exclusive with autotune, which owns the same channel).
        self._path_follower = PathFollower(clock)

    # ---------------------------
    # Delegated read-only state (telemetry code reads these)
    # ---------------------------

    @property
    def target_x_mm(self) -> float:
        return self._arbiter.active[0]

    @property
    def target_y_mm(self) -> float:
        return self._arbiter.active[1]

    @property
    def roll_offset(self) -> float:
        return self._auto_trim.roll_offset

    @property
    def pitch_offset(self) -> float:
        return self._auto_trim.pitch_offset

    @property
    def home_calibration_active(self) -> bool:
        return self._auto_trim.home_calibration_active

    @property
    def pd_autotune_enabled(self) -> bool:
        return self._autotuner.enabled

    @property
    def pd_autotune_auto_apply(self) -> bool:
        return self._autotuner.auto_apply

    @property
    def path_following_active(self) -> bool:
        return self._path_follower.active

    # ---------------------------
    # Gains / limits
    # ---------------------------

    @property
    def kp(self) -> float:
        return self._pd.kp

    @kp.setter
    def kp(self, value: float) -> None:
        self._pd.kp = float(value)

    @property
    def kd(self) -> float:
        return self._pd.kd

    @kd.setter
    def kd(self, value: float) -> None:
        self._pd.kd = float(value)

    @property
    def max_tilt_deg(self) -> float:
        return self._pd.max_tilt_deg

    @max_tilt_deg.setter
    def max_tilt_deg(self, value: float) -> None:
        self._pd.max_tilt_deg = float(value)

    @property
    def max_tilt_rate_deg_s(self) -> float:
        return self._pd.max_tilt_rate_deg_s

    @property
    def d_term_limit_deg(self) -> float | None:
        return self._pd.d_term_limit_deg

    def set_gains(self, kp: float, kd: float) -> None:
        self._pd.kp = float(kp)
        self._pd.kd = float(kd)
        self._autotuner.clear_suggestion(kp, kd)

    def set_max_tilt(self, max_tilt_deg: float) -> None:
        self._pd.max_tilt_deg = float(max_tilt_deg)

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    # ---------------------------
    # Trim / target / home-cal
    # ---------------------------

    def set_trim(self, roll_offset_deg: float, pitch_offset_deg: float) -> None:
        self._auto_trim.set_offsets(roll_offset_deg, pitch_offset_deg)

    def reset_trim(self) -> None:
        self._auto_trim.reset()

    def set_auto_trim_enabled(self, enabled: bool) -> None:
        self.auto_trim_enabled = bool(enabled)
        if not self.auto_trim_enabled:
            self._auto_trim.cancel_home_calibration()

    def start_home_calibration(self) -> None:
        # Exclusion: calibration needs a stationary center target.
        self.stop_path()
        self.auto_trim_enabled = True
        # Deliberate unification (M10): home-cal routes its center target
        # through the manual setpoint. The GUI already emits set_target(0,0)
        # alongside, so the manual target ends up at center either way.
        self._arbiter.set_manual(
            float(BALL_TARGET_DEFAULT_X_MM), float(BALL_TARGET_DEFAULT_Y_MM)
        )
        self._auto_trim.start_home_calibration()

    def cancel_home_calibration(self) -> None:
        self._auto_trim.cancel_home_calibration()

    def set_target(self, x_mm: float, y_mm: float) -> None:
        self._arbiter.set_manual(float(x_mm), float(y_mm))
        self._auto_trim.notify_target_changed()
        # Target moved: any "parked at the target" evidence is stale.
        self._rest_gate.reset()
        if self._autotuner.enabled:
            self._autotuner.set_center(float(x_mm), float(y_mm))

    # ---------------------------
    # Path following coordination (feat/path)
    # ---------------------------

    def set_path(self, path: Path) -> None:
        """Load/replace the path. Stops any active following first (with the
        motion-free target transfer) so a mid-run swap cannot jump the
        target."""
        if self._path_follower.active:
            self.stop_path()
        self._path_follower.set_path(path)

    def set_path_speed(self, mm_s: float) -> None:
        self._path_follower.set_speed(float(mm_s))

    def start_path(self) -> bool:
        """Begin following the loaded path.

        Mutual exclusion: the follower drives the arbiter's override
        channel — the same channel autotune owns — so autotune and
        home calibration are force-disabled first. No override is written
        here; the first compute_with_terms with a valid ball seeds the
        target at the nearest path point (the platform does not move on
        start until the ball is seen).
        """
        self._auto_trim.cancel_home_calibration()
        self.set_pd_autotune(False)
        self._rest_gate.reset()
        return self._path_follower.start()

    def stop_path(self) -> None:
        """Stop following, freezing the target IN PLACE (zero motion):
        the manual target is set to the current active target before the
        override clears, so `active` is bit-identical across the
        transition. (Safe against clobbering autotune's override because
        the exclusion contract guarantees autotune is off while a path
        runs.)"""
        self._path_follower.stop()
        if self._arbiter.override_active:
            fx, fy = self._arbiter.active
            self._arbiter.set_manual(fx, fy)
            self._arbiter.clear_override()
        self._auto_trim.notify_target_changed()
        self._rest_gate.reset()

    # ---------------------------
    # Autotune coordination
    # ---------------------------

    def set_pd_autotune(self, enabled: bool, auto_apply: bool | None = None) -> None:
        if bool(enabled):
            self._auto_trim.cancel_home_calibration()
            # Exclusion: autotune owns the override channel next.
            self.stop_path()
        changed = self._autotuner.set_enabled(enabled, auto_apply, self.kp, self.kd)
        if changed:
            # Mode change: rest evidence gathered in the old mode is stale.
            self._rest_gate.reset()
            # Autotune sessions stash the auto-trim flag on enable and
            # restore it on disable (the arbiter override restore of the
            # manual target lives inside PDAutotuner.set_enabled).
            if self._autotuner.enabled:
                self._pd_autotune_prev_auto_trim_enabled = self.auto_trim_enabled
                self.auto_trim_enabled = False
            else:
                self.auto_trim_enabled = self._pd_autotune_prev_auto_trim_enabled

    def apply_pd_autotune_recommendation(self) -> tuple[bool, float, float]:
        applied, kp, kd = self._autotuner.take_recommendation(self.kp, self.kd)
        if applied:
            self._pd.kp = float(kp)
            self._pd.kd = float(kd)
        return applied, float(self.kp), float(self.kd)

    # ---------------------------
    # Control step
    # ---------------------------

    def reset_motion_state(self) -> None:
        self._pd.reset_motion_state()
        # Tracking loss / reacquire: the ball may be anywhere now.
        self._rest_gate.reset()

    def compute_with_terms(
        self, ball_state: BallState | None
    ) -> tuple[float, float, dict]:
        """
        Compute desired platform tilt and expose full diagnostics.

        Returns:
            (roll_deg, pitch_deg, terms_dict)
        """
        if not self.enabled or ball_state is None:
            return 0.0, 0.0, {
                "position_vec_mm": (0.0, 0.0),
                "velocity_vec_mm_s": (0.0, 0.0),
                "pd_vec": (0.0, 0.0),
            }

        x = ball_state.x_mm
        y = ball_state.y_mm
        vx = ball_state.vx_mm_s
        vy = ball_state.vy_mm_s

        # PD autotune step (FG-10) — may move the arbiter override target
        # and (in auto-apply mode) update the gains.
        self.kp, self.kd, tune_info = self._autotuner.update(
            x, y, vx, vy, self.kp, self.kd
        )

        # Path follower (feat/path): advances the override target along the
        # loaded path at the adaptive rate, seeded at the nearest path
        # point on the first ball sighting. set_override stamps
        # last_change_time every cycle, which deliberately holds auto-trim
        # in target_hold for the whole run — pursuit lag is NOT a level
        # error and must not be integrated into trim.
        if self._path_follower.active:
            ptx, pty = self._path_follower.update(x, y)
            self._arbiter.set_override(ptx, pty)

        target_x, target_y = self._arbiter.active
        pos_vec_x = target_x - x
        pos_vec_y = target_y - y

        # Auto-trim update (FG-9) — this frame's trim step feeds this
        # frame's output, matching the pre-decomposition ordering.
        self._auto_trim.update(
            pos_vec_x, pos_vec_y, vx, vy,
            enabled=self.auto_trim_enabled and self.enabled,
            target_change_time=self._arbiter.last_change_time,
        )

        # Near-target rest gate (perf pass). Evaluated from the same error
        # vector / velocity the PD sees. While resting, the final command is
        # level + trim (the trim offsets ARE the current best estimate of
        # level, and auto-trim keeps integrating — its 35 mm / 25 mm/s gates
        # are wider than rest's — so trim updates still move the platform).
        # The PD pipeline ALWAYS runs (telemetry + slew-state continuity);
        # resting only redirects the slew limiter's target, so both the
        # entry hop and the exit back to full PD are rate-limited and
        # continuous. Exit is same-cycle: an "active" verdict here computes
        # a full PD command on this very call.
        rest_radius_mm = math.hypot(pos_vec_x, pos_vec_y)
        rest_speed_mm_s = math.hypot(vx, vy)
        path_following = (
            self._path_follower.active and not self._path_follower.done
        )
        if self.home_calibration_active or path_following:
            # Home calibration needs ACTIVE PD driving the ball to center
            # while auto-trim absorbs the bias — resting at level+trim with
            # the ball parked off-center would leave centering to the slow
            # trim walk alone (observed on the rig: calibration far less
            # reliable with rest engaged).
            # Path following likewise: a slow segment (taper, low speed)
            # can satisfy the rest entry gates while the target crawls,
            # producing a rest/wake limbo — suppress. When an OPEN path is
            # done (ball parked at the endpoint) suppression lifts and
            # rest may engage exactly like a manual target hold.
            self._rest_gate.reset()
            rest_state = "active"
            resting = False
        else:
            rest_state = self._rest_gate.update(rest_radius_mm, rest_speed_mm_s)
            resting = rest_state == STATE_RESTING

        res = self._pd.compute(
            pos_vec_x, pos_vec_y, vx, vy, self.roll_offset, self.pitch_offset,
            slew_target_override=(
                (self.roll_offset, self.pitch_offset) if resting else None
            ),
        )

        terms: dict = {
            "position_vec_mm": (pos_vec_x, pos_vec_y),
            "velocity_vec_mm_s": (vx, vy),
            "pd_vec": res.pd_vec,
            "p_term": res.p_term,
            "d_term": res.d_term,
            "roll_raw": res.roll_raw,
            "pitch_raw": res.pitch_raw,
            "roll_clamped": res.roll_clamped,
            "pitch_clamped": res.pitch_clamped,
            "roll_cmd": res.roll_cmd,
            "pitch_cmd": res.pitch_cmd,
            "roll_offset": self.roll_offset,
            "pitch_offset": self.pitch_offset,
            **self._auto_trim.telemetry(self.auto_trim_enabled),
            "rest_state": rest_state,
            "rest_mode_active": resting,
            "rest_radius_mm": rest_radius_mm,
            "rest_speed_lpf_mm_s": self._rest_gate.speed_lpf_mm_s,
            "rest_hold_elapsed_s": self._rest_gate.hold_elapsed_s,
            "kp": self.kp,
            "kd": self.kd,
            **self._autotuner.telemetry(),
            **self._path_follower.telemetry(),
            "target_x_mm": self.target_x_mm,
            "target_y_mm": self.target_y_mm,
        }
        if tune_info:
            terms["pd_autotune_event"] = tune_info
        return res.roll_cmd, res.pitch_cmd, terms
