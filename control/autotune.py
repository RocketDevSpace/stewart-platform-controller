"""
control/autotune.py

PDAutotuner — the PD autotune state machine (FG-10) plus the per-leg step
response evaluator. Drives its leg/center targets through the injected
SetpointArbiter override channel; gain ownership stays with the caller
(gains are passed into update() and possibly-updated values returned).

The per-session log path is injectable so tests can point it at tmp_path;
production uses settings.AUTOTUNE_LOG_PATH, opened fresh per session.
"""
import logging
import math
from collections.abc import Callable
from typing import Any

from control.setpoint import SetpointArbiter
from settings import (
    AUTOTUNE_LOG_PATH,
    PD_AUTOTUNE_AUTO_APPLY,
    PD_AUTOTUNE_ENABLED,
    PD_AUTOTUNE_G_EFF,
    PD_AUTOTUNE_MAX_GAIN_DELTA_FRAC,
    PD_AUTOTUNE_MAX_KD,
    PD_AUTOTUNE_MAX_KP,
    PD_AUTOTUNE_MIN_CROSS_S,
    PD_AUTOTUNE_MIN_KD,
    PD_AUTOTUNE_MIN_KP,
    PD_AUTOTUNE_MIN_OVERSHOOT_RATIO,
    PD_AUTOTUNE_MIN_TRIAL_S,
    PD_AUTOTUNE_SETTLE_HOLD_S,
    PD_AUTOTUNE_SETTLE_RADIUS_MM,
    PD_AUTOTUNE_SETTLE_SPEED_MM_S,
    PD_AUTOTUNE_STEP_MM,
    PD_AUTOTUNE_TARGET_ZETA,
    PD_AUTOTUNE_TIMEOUT_S,
    PD_AUTOTUNE_WAIT_SETTLE_HOLD_S,
    PD_AUTOTUNE_WAIT_SETTLE_RADIUS_MM,
    PD_AUTOTUNE_WAIT_SETTLE_SPEED_MM_S,
)


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min(value, max_val), min_val)


class _PDLegEvaluator:
    """Evaluates a single leg of a two-target step test for PD autotune."""

    def __init__(self) -> None:
        self.active = False
        self.start_ts = 0.0
        self.last_ts = 0.0
        self.settle_timer = 0.0
        self.u_x = 0.0
        self.u_y = 0.0
        self.s0 = 0.0
        self.min_s = 0.0
        self.peak_abs_s = 0.0
        self.iae = 0.0
        self.sign_changes = 0
        self.last_s: float | None = None
        self.first_crossing_elapsed_s: float | None = None

    def start(self, ts: float, x: float, y: float, tx: float, ty: float) -> bool:
        ex = tx - x
        ey = ty - y
        e0 = math.hypot(ex, ey)
        if e0 <= 1e-6:
            return False
        self.active = True
        self.start_ts = ts
        self.last_ts = ts
        self.settle_timer = 0.0
        self.u_x = ex / e0
        self.u_y = ey / e0
        self.s0 = e0
        self.min_s = e0
        self.peak_abs_s = e0
        self.iae = 0.0
        self.sign_changes = 0
        self.last_s = e0
        self.first_crossing_elapsed_s = None
        return True

    def observe(
        self,
        ts: float,
        x: float,
        y: float,
        vx: float,
        vy: float,
        tx: float,
        ty: float,
        settle_radius_mm: float,
        settle_speed_mm_s: float,
        settle_hold_s: float,
        min_trial_s: float,
        timeout_s: float,
    ) -> dict | None:
        if not self.active:
            return None
        ex = tx - x
        ey = ty - y
        err = math.hypot(ex, ey)
        speed = math.hypot(vx, vy)
        dt = max(1e-4, min(0.1, ts - self.last_ts))
        self.last_ts = ts

        s = ex * self.u_x + ey * self.u_y
        self.iae += abs(s) * dt
        self.peak_abs_s = max(self.peak_abs_s, abs(s))
        self.min_s = min(self.min_s, s)
        if self.last_s is not None:
            band = max(2.0, settle_radius_mm)
            if abs(s) > band and abs(self.last_s) > band and (s * self.last_s) < 0.0:
                self.sign_changes += 1
            if self.first_crossing_elapsed_s is None and self.last_s > 0.0 and s <= 0.0:
                self.first_crossing_elapsed_s = ts - self.start_ts
        self.last_s = s

        if err <= settle_radius_mm and speed <= settle_speed_mm_s:
            self.settle_timer += dt
        else:
            self.settle_timer = 0.0

        elapsed = ts - self.start_ts
        settled = self.settle_timer >= settle_hold_s
        timed_out = elapsed >= timeout_s
        if elapsed < min_trial_s:
            return None
        if not settled and not timed_out:
            return None

        overshoot_mm = max(0.0, -self.min_s)
        metrics: dict = {
            "initial_offset_mm": self.s0,
            "settle_time_s": elapsed,
            "overshoot_mm": overshoot_mm,
            "overshoot_ratio": overshoot_mm / max(self.s0, 1e-6),
            "oscillation_crossings": float(self.sign_changes),
            "iae_mm_s": self.iae,
            "peak_abs_proj_mm": self.peak_abs_s,
            "timed_out": 1.0 if timed_out and not settled else 0.0,
            "target_x_mm": float(tx),
            "target_y_mm": float(ty),
            "first_crossing_elapsed_s": self.first_crossing_elapsed_s,
        }
        self.active = False
        return metrics


class PDAutotuner:
    def __init__(
        self,
        arbiter: SetpointArbiter,
        clock: Callable[[], float],
        log_path: str = AUTOTUNE_LOG_PATH,
    ) -> None:
        self._arbiter = arbiter
        self._clock = clock
        self._log_path = str(log_path)

        self.enabled = bool(PD_AUTOTUNE_ENABLED)
        self.auto_apply = bool(PD_AUTOTUNE_AUTO_APPLY)
        self.min_kp = float(PD_AUTOTUNE_MIN_KP)
        self.max_kp = float(PD_AUTOTUNE_MAX_KP)
        self.min_kd = float(PD_AUTOTUNE_MIN_KD)
        self.max_kd = float(PD_AUTOTUNE_MAX_KD)
        self.step_mm = float(PD_AUTOTUNE_STEP_MM)
        self.g_eff = float(PD_AUTOTUNE_G_EFF)
        self.target_zeta = float(PD_AUTOTUNE_TARGET_ZETA)
        self.min_overshoot_ratio = float(PD_AUTOTUNE_MIN_OVERSHOOT_RATIO)

        self.trial_count = 0
        self.last_message = ""
        self.has_suggestion = False
        self.suggested_kp = 0.0
        self.suggested_kd = 0.0

        self._leg_eval = _PDLegEvaluator()
        self._leg_index = 0
        self._state = "idle"  # "wait_settle" | "measuring"
        self._wait_settle_timer = 0.0
        self._wait_settle_ts = 0.0
        self._leg_initialized = False
        self._center_x_mm, self._center_y_mm = arbiter.active

        # Per-session autotune log: stable logger name; the file handler is
        # swapped in fresh at each session start (path injectable for tests).
        self._log = logging.getLogger("stewart.autotune")
        self._log.propagate = False

    # ---------------------------
    # Session control
    # ---------------------------

    def clear_suggestion(self, kp: float, kd: float) -> None:
        self.has_suggestion = False
        self.suggested_kp = float(kp)
        self.suggested_kd = float(kd)

    def take_recommendation(self, kp: float, kd: float) -> tuple[bool, float, float]:
        """Consume the pending suggestion; returns (applied, kp, kd)."""
        if not self.has_suggestion:
            return False, float(kp), float(kd)
        self.has_suggestion = False
        return True, float(self.suggested_kp), float(self.suggested_kd)

    def set_center(self, x_mm: float, y_mm: float) -> None:
        self._center_x_mm = float(x_mm)
        self._center_y_mm = float(y_mm)
        self._leg_initialized = False
        self._leg_eval.active = False

    def _setup_session_log(self) -> None:
        log = self._log
        for h in log.handlers[:]:
            log.removeHandler(h)
            h.close()
        try:
            fh = logging.FileHandler(self._log_path, mode="w", encoding="utf-8")
            fh.setFormatter(logging.Formatter(
                "%(asctime)s.%(msecs)03d  %(message)s", datefmt="%H:%M:%S"
            ))
            log.addHandler(fh)
            log.setLevel(logging.DEBUG)
        except OSError:
            pass

    def set_enabled(
        self, enabled: bool, auto_apply: bool | None, kp: float, kd: float
    ) -> bool:
        """Enable/disable the autotune session. Returns True if the enabled
        state changed. Gain stash/restore of auto-trim stays with the caller.
        """
        prev_enabled = self.enabled
        self.enabled = bool(enabled)
        if auto_apply is not None:
            self.auto_apply = bool(auto_apply)
        changed = prev_enabled != self.enabled
        if changed:
            self._leg_initialized = False
            self._state = "idle"
            self._leg_eval.active = False
            if self.enabled:
                self._setup_session_log()
                self._log.info(
                    "SESSION START  kp=%.4f kd=%.4f  g_eff=%.1f zeta_target=%.2f"
                    " step_mm=%.1f  max_kp=%.3f max_kd=%.3f"
                    "  min_cross_s=%.2f max_gain_delta_frac=%.2f",
                    kp, kd, self.g_eff,
                    self.target_zeta, self.step_mm,
                    self.max_kp, self.max_kd,
                    PD_AUTOTUNE_MIN_CROSS_S, PD_AUTOTUNE_MAX_GAIN_DELTA_FRAC,
                )
                manual_x, manual_y = self._arbiter.manual
                self._center_x_mm = float(manual_x)
                self._center_y_mm = float(manual_y)
                # Pin the active target at the center (== manual) and stamp
                # the change time, matching the pre-decomposition behavior.
                self._arbiter.set_override(manual_x, manual_y)
            else:
                self._log.info(
                    "SESSION END  trials=%d  final_kp=%.4f  final_kd=%.4f",
                    self.trial_count, kp, kd,
                )
                self._arbiter.clear_override()
        if (not self.enabled) or self.auto_apply:
            self.has_suggestion = False
        return changed

    # ---------------------------
    # Gain inversion (FG-10)
    # ---------------------------

    def _autotune_leg_target(self, leg_index: int) -> tuple[float, float]:
        d = self.step_mm
        cx = self._center_x_mm
        cy = self._center_y_mm
        offsets = [(d, 0.0), (-d, 0.0), (0.0, d), (0.0, -d)]
        ox, oy = offsets[int(leg_index) % 4]
        return cx + ox, cy + oy

    def _compute_pd_from_metrics(
        self, metrics: dict, kp: float, kd: float
    ) -> tuple[float, float, str]:
        overshoot_ratio = float(metrics.get("overshoot_ratio", 0.0))
        settle_s = float(metrics.get("settle_time_s", 1.0))
        t_cross = metrics.get("first_crossing_elapsed_s")
        timed_out = bool(metrics.get("timed_out", 0.0) > 0.5)
        g = self.g_eff
        zeta_target = self.target_zeta

        if timed_out:
            rationale = "timeout: hold gains"
            return (
                _clamp(kp, self.min_kp, self.max_kp),
                _clamp(kd, self.min_kd, self.max_kd),
                rationale,
            )

        if overshoot_ratio >= self.min_overshoot_ratio:
            ln_os = math.log(max(overshoot_ratio, 1e-9))
            zeta_obs = -ln_os / math.sqrt(math.pi ** 2 + ln_os ** 2)
            zeta_obs = _clamp(zeta_obs, 0.05, 0.99)
        else:
            zeta_obs = zeta_target
            rationale = "overdamped: using target zeta"

        if t_cross is not None and t_cross >= PD_AUTOTUNE_MIN_CROSS_S:
            wd = math.pi / t_cross
            zeta_for_wn = _clamp(zeta_obs, 0.05, 0.999)
            wn = wd / math.sqrt(1.0 - zeta_for_wn ** 2)
            rationale = f"underdamped zeta={zeta_obs:.2f} wn={wn:.2f} (crossing)"
        else:
            wn = 5.8 / max(settle_s, 0.05)
            rationale = f"overdamped wn={wn:.2f} (settle fallback)"

        kp_new = wn ** 2 / g
        kd_new = 2.0 * zeta_target * wn / g

        # Cap gain change to MAX_GAIN_DELTA_FRAC of current value per trial.
        # Absolute floor of 0.001 prevents the window from collapsing to [0,0]
        # when kp or kd is zero (e.g. user zeroed kd before enabling).
        max_frac = float(PD_AUTOTUNE_MAX_GAIN_DELTA_FRAC)
        kp_step = max(kp * max_frac, 0.001)
        kd_step = max(kd * max_frac, 0.001)
        kp_new = _clamp(kp_new, kp - kp_step, kp + kp_step)
        kd_new = _clamp(kd_new, kd - kd_step, kd + kd_step)

        kp_new = _clamp(kp_new, self.min_kp, self.max_kp)
        kd_new = _clamp(kd_new, self.min_kd, self.max_kd)
        return kp_new, kd_new, rationale

    # ---------------------------
    # Per-frame update (FG-10)
    # ---------------------------

    def update(
        self, x: float, y: float, vx: float, vy: float, kp: float, kd: float
    ) -> tuple[float, float, dict | None]:
        """Advance the autotune state machine one frame.

        Returns (kp, kd, event): the possibly auto-applied gains and an
        event dict on state transitions (None otherwise).
        """
        if not self.enabled:
            return kp, kd, None
        now = self._clock()

        if not self._leg_initialized:
            self._leg_index = 0
            self._arbiter.set_override(self._center_x_mm, self._center_y_mm)
            self._state = "wait_settle"
            self._wait_settle_timer = 0.0
            self._wait_settle_ts = now
            self._leg_initialized = True
            self._log.info(
                "WAIT_SETTLE  leg=%d  center=(%.1f,%.1f)",
                self._leg_index, self._center_x_mm, self._center_y_mm,
            )
            msg = "autotune: waiting for ball to settle at center"
            self.last_message = msg
            return kp, kd, {"type": "wait_settle_started", "message": msg}

        if self._state == "wait_settle":
            err = math.hypot(self._center_x_mm - x, self._center_y_mm - y)
            speed = math.hypot(vx, vy)
            dt = max(1e-4, min(0.1, now - self._wait_settle_ts))
            self._wait_settle_ts = now
            within_r = err <= PD_AUTOTUNE_WAIT_SETTLE_RADIUS_MM
            within_v = speed <= PD_AUTOTUNE_WAIT_SETTLE_SPEED_MM_S
            if within_r and within_v:
                self._wait_settle_timer += dt
            else:
                self._wait_settle_timer = 0.0
            if self._wait_settle_timer < PD_AUTOTUNE_WAIT_SETTLE_HOLD_S:
                return kp, kd, None
            tx, ty = self._autotune_leg_target(self._leg_index)
            self._arbiter.set_override(tx, ty)
            self._leg_eval.start(now, x, y, tx, ty)
            self._state = "measuring"
            self._log.info(
                "LEG %d START  target=(%.1f,%.1f)  ball=(%.1f,%.1f)  s0=%.1f",
                self._leg_index, tx, ty, x, y, self._leg_eval.s0,
            )
            msg = f"autotune: leg {self._leg_index} started target=({tx:.1f},{ty:.1f})"
            self.last_message = msg
            return kp, kd, {"type": "trial_started", "message": msg}

        active_x, active_y = self._arbiter.active
        metrics = self._leg_eval.observe(
            now, x, y, vx, vy,
            active_x, active_y,
            settle_radius_mm=PD_AUTOTUNE_SETTLE_RADIUS_MM,
            settle_speed_mm_s=PD_AUTOTUNE_SETTLE_SPEED_MM_S,
            settle_hold_s=PD_AUTOTUNE_SETTLE_HOLD_S,
            min_trial_s=PD_AUTOTUNE_MIN_TRIAL_S,
            timeout_s=PD_AUTOTUNE_TIMEOUT_S,
        )
        if metrics is None:
            return kp, kd, None

        self.trial_count += 1
        kp_before = float(kp)
        kd_before = float(kd)
        kp_new, kd_new, rationale = self._compute_pd_from_metrics(metrics, kp, kd)
        changed = (abs(kp_new - kp) > 1e-9) or (abs(kd_new - kd) > 1e-9)

        kp_out = float(kp)
        kd_out = float(kd)
        if changed and self.auto_apply:
            kp_out = float(kp_new)
            kd_out = float(kd_new)
            self.has_suggestion = False
            self.suggested_kp = float(kp_out)
            self.suggested_kd = float(kd_out)
            action = f"applied kp={kp_out:.4f}, kd={kd_out:.4f}"
        elif changed:
            self.has_suggestion = True
            self.suggested_kp = float(kp_new)
            self.suggested_kd = float(kd_new)
            action = f"suggest kp={kp_new:.4f}, kd={kd_new:.4f}"
        else:
            self.has_suggestion = False
            self.suggested_kp = float(kp)
            self.suggested_kd = float(kd)
            action = "hold gains"

        self._leg_index = (self._leg_index + 1) % 4
        self._arbiter.set_override(self._center_x_mm, self._center_y_mm)
        self._state = "wait_settle"
        self._wait_settle_timer = 0.0
        self._wait_settle_ts = now

        completed_leg = (self._leg_index - 1) % 4
        tc = metrics["first_crossing_elapsed_s"]
        tc_str = f"{tc:.3f}s" if tc is not None else "None"
        self._log.info(
            "LEG %d DONE  settle=%.3fs  OS=%.4f  t_cross=%s  crossings=%.0f  iae=%.1f",
            completed_leg,
            metrics["settle_time_s"], metrics["overshoot_ratio"],
            tc_str, metrics["oscillation_crossings"], metrics["iae_mm_s"],
        )
        self._log.info(
            "  -> kp %.4f->%.4f  kd %.4f->%.4f  [%s]  %s",
            kp_before, kp_new, kd_before, kd_new, action, rationale,
        )

        msg = (
            f"trial#{self.trial_count} "
            f"settle={metrics['settle_time_s']:.2f}s "
            f"overshoot={metrics['overshoot_ratio']:.2f} "
            f"cross={metrics['oscillation_crossings']:.0f} "
            f"iae={metrics['iae_mm_s']:.1f} -> {action} ({rationale})"
        )
        self.last_message = msg
        return kp_out, kd_out, {
            "type": "trial_done",
            "message": msg,
            "metrics": metrics,
            "kp_suggested": kp_new,
            "kd_suggested": kd_new,
            "applied": 1.0 if (changed and self.auto_apply) else 0.0,
        }

    # ---------------------------
    # Telemetry (terms-dict keys — FROZEN, the GUI reads these)
    # ---------------------------

    def telemetry(self) -> dict[str, Any]:
        return {
            "pd_autotune_enabled": self.enabled,
            "pd_autotune_auto_apply": self.auto_apply,
            "pd_autotune_trial_count": self.trial_count,
            "pd_autotune_message": self.last_message,
            "pd_autotune_has_suggestion": self.has_suggestion,
            "pd_autotune_suggested_kp": self.suggested_kp,
            "pd_autotune_suggested_kd": self.suggested_kd,
        }
