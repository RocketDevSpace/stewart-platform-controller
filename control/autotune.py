"""
control/autotune.py

PDAutotuner — the SysID autotune pipeline (2026-07-23 rework):

    idle → probing → computing (fit + design, background thread)
         → suggestion_ready → idle

The old step-test estimator (2-scalar-feature inversion of an ideal
2nd-order model behind settle gates) is gone — it never completed a leg
on the rig and random-walked against a known simulated plant. The
replacement: run the ~78 s scripted probe (control/plant_id.ProbeScript,
no settle gates) through the arbiter override channel while recording
raw detections + sent commands; fit the plant (convex regression,
control/plant_id.fit_plant); search kp/ki/kd on the fitted plant
(control/gain_design.design_gains); suggest — with the identified plant
numbers riding along so Apply can also calibrate the prediction horizon
and integral deadband.

Threading: the fit + design compute runs on a daemon threading.Thread
owned by this object (precedent: MainWindow's serial-connect thread).
The worker-thread update() is the ONLY mutator of public state; the
compute thread communicates through a lock-guarded (phase, progress)
mirror and a single-slot generation-tagged result. Cancellation is an
Event checked inside fit/design chunk boundaries. No join() on the
control thread. Tests can set `run_compute_inline = True` to execute
the compute synchronously.

Gain ownership stays with the caller exactly as before: gains are
passed into update() and possibly-updated values returned.
"""
import logging
import math
import threading
from collections.abc import Callable
from typing import Any

import numpy as np

from control.gain_design import GainDesign, design_gains
from control.plant_id import PlantFit, ProbeRecording, ProbeScript, fit_plant
from control.setpoint import SetpointArbiter
from settings import (
    AUTOTUNE_LOG_PATH,
    PD_AUTOTUNE_ABORT_RADIUS_MM,
    PD_AUTOTUNE_AUTO_APPLY,
    PD_AUTOTUNE_BALL_LOST_S,
    PD_AUTOTUNE_ENABLED,
)


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

        # Public suggestion surface (kept from the old estimator)
        self.trial_count = 0            # completed probe segments
        self.last_message = ""
        self.has_suggestion = False
        self.suggested_kp = 0.0
        self.suggested_kd = 0.0
        self.suggested_ki = 0.0

        # Pipeline state
        self._state = "idle"            # probing|computing|suggestion_ready
        self._script = ProbeScript()
        self._recording: ProbeRecording | None = None
        self._probe_start: float | None = None
        self._last_valid_ts: float | None = None
        self._out_since: float | None = None
        self._last_segment = -1
        self._center_x_mm, self._center_y_mm = arbiter.active
        self._gains_at_start = (0.0, 0.0, 0.0)

        # Results
        self.plant_fit: PlantFit | None = None
        self._design: GainDesign | None = None

        # Compute-thread handoff (see module docstring)
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._generation = 0
        self._thread: threading.Thread | None = None
        self._thread_phase = ("idle", 0.0)
        self._thread_result: dict | None = None
        self.run_compute_inline = False     # tests: synchronous compute

        self._log = logging.getLogger("stewart.autotune")
        self._log.propagate = False

    # ---------------------------
    # Session control
    # ---------------------------

    def clear_suggestion(
        self, kp: float, kd: float, ki: float = 0.0
    ) -> None:
        self.has_suggestion = False
        self.suggested_kp = float(kp)
        self.suggested_kd = float(kd)
        self.suggested_ki = float(ki)

    def take_recommendation(
        self, kp: float, kd: float, ki: float
    ) -> tuple[bool, float, float, float]:
        """Consume the pending suggestion; returns (applied, kp, kd, ki)."""
        if not self.has_suggestion:
            return False, float(kp), float(kd), float(ki)
        self.has_suggestion = False
        return (
            True,
            float(self.suggested_kp),
            float(self.suggested_kd),
            float(self.suggested_ki),
        )

    def set_center(self, x_mm: float, y_mm: float) -> None:
        """Session center follows target moves only OUTSIDE a probe —
        the script owns the targets while probing."""
        if self._state not in ("probing", "computing"):
            self._center_x_mm = float(x_mm)
            self._center_y_mm = float(y_mm)

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
        self,
        enabled: bool,
        auto_apply: bool | None,
        kp: float,
        kd: float,
        ki: float = 0.0,
    ) -> bool:
        """Enable/disable the session. Returns True if enabled changed."""
        prev = self.enabled
        self.enabled = bool(enabled)
        if auto_apply is not None:
            self.auto_apply = bool(auto_apply)
        changed = prev != self.enabled
        if changed:
            if self.enabled:
                self._setup_session_log()
                self._log.info(
                    "SESSION START (SysID)  kp=%.4f kd=%.4f ki=%.4f  "
                    "probe=%.1fs", kp, kd, ki, self._script.total_s,
                )
                manual_x, manual_y = self._arbiter.manual
                self._center_x_mm = float(manual_x)
                self._center_y_mm = float(manual_y)
                self._arbiter.set_override(manual_x, manual_y)
                self._state = "probing"
                self._recording = ProbeRecording()
                self._probe_start = None
                self._last_valid_ts = None
                self._out_since = None
                self._last_segment = -1
                self.trial_count = 0
                self._gains_at_start = (float(kp), float(kd), float(ki))
                self.plant_fit = None
                self._design = None
                self._generation += 1
                self._cancel.clear()
                with self._lock:
                    self._thread_phase = ("probing", 0.0)
                    self._thread_result = None
            else:
                self._log.info("SESSION END  state=%s", self._state)
                self._cancel.set()
                self._state = "idle"
                self._recording = None
                self._arbiter.clear_override()
        if (not self.enabled) or self.auto_apply:
            self.has_suggestion = False
        return changed

    # ---------------------------
    # Per-frame update (worker thread — sole public-state mutator)
    # ---------------------------

    def update(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        kp: float,
        kd: float,
        ki: float,
        raw_x: float | None = None,
        raw_y: float | None = None,
    ) -> tuple[float, float, float, dict | None]:
        """Advance the pipeline one valid frame.

        Returns (kp, kd, ki, event): possibly auto-applied gains and an
        event dict on transitions (None otherwise).
        """
        if not self.enabled:
            return kp, kd, ki, None
        now = self._clock()

        if self._state == "probing":
            return self._update_probing(
                now, x, y, vx, vy,
                x if raw_x is None else raw_x,
                y if raw_y is None else raw_y,
                kp, kd, ki,
            )
        if self._state == "computing":
            return self._update_computing(kp, kd, ki)
        return kp, kd, ki, None

    def record_command(self, roll_cmd: float, pitch_cmd: float) -> None:
        """Complete the probe row begun by update() (called after the
        PID compute in the same frame — exact command/frame pairing)."""
        if self._state == "probing" and self._recording is not None:
            self._recording.complete_row(roll_cmd, pitch_cmd)

    def _abort(self, reason: str) -> dict:
        self._log.info("PROBE ABORT: %s", reason)
        self._state = "idle"
        self._recording = None
        self._arbiter.set_override(self._center_x_mm, self._center_y_mm)
        msg = f"autotune probe aborted: {reason}"
        self.last_message = msg
        return {"type": "probe_aborted", "message": msg, "reason": reason}

    def _update_probing(
        self,
        now: float,
        x: float,
        y: float,
        vx: float,
        vy: float,
        raw_x: float,
        raw_y: float,
        kp: float,
        kd: float,
        ki: float,
    ) -> tuple[float, float, float, dict | None]:
        event: dict | None = None
        if self._probe_start is None:
            self._probe_start = now
            self._last_valid_ts = now
            msg = (
                f"autotune: probe started ({self._script.total_s:.0f}s "
                "scripted sequence)"
            )
            self.last_message = msg
            event = {"type": "probe_started", "message": msg}

        # Ball-lost abort: update() only runs on valid frames, so a gap
        # means sustained tracking loss.
        assert self._last_valid_ts is not None
        if now - self._last_valid_ts > PD_AUTOTUNE_BALL_LOST_S:
            return kp, kd, ki, self._abort("ball lost")
        self._last_valid_ts = now

        # Out-of-bounds: recenter immediately; abort if it persists.
        r = math.hypot(x - self._center_x_mm, y - self._center_y_mm)
        if r > PD_AUTOTUNE_ABORT_RADIUS_MM:
            if self._out_since is None:
                self._out_since = now
                self._arbiter.set_override(
                    self._center_x_mm, self._center_y_mm
                )
            elif now - self._out_since > 2.0:
                return kp, kd, ki, self._abort("out of bounds")
            return kp, kd, ki, event
        self._out_since = None

        elapsed = now - self._probe_start
        seg = self._script.segment_index(elapsed)
        if seg != self._last_segment:
            tx, ty = self._script.target_at(elapsed)
            self._arbiter.set_override(
                self._center_x_mm + tx, self._center_y_mm + ty
            )
            self._last_segment = seg
            self.trial_count = seg
        with self._lock:
            self._thread_phase = ("probing", self._script.progress(elapsed))

        assert self._recording is not None
        tgt_x, tgt_y = self._arbiter.active
        self._recording.begin_row(
            now, raw_x, raw_y, x, y, vx, vy, tgt_x, tgt_y
        )

        if elapsed >= self._script.total_s:
            self._state = "computing"
            self._arbiter.set_override(self._center_x_mm, self._center_y_mm)
            rows = self._recording.arrays()
            self._recording = None
            self._start_compute(rows, kp, kd, ki)
            msg = "autotune: probe complete — fitting plant + designing gains"
            self.last_message = msg
            event = {"type": "probe_done", "message": msg}
        return kp, kd, ki, event

    def _start_compute(
        self, rows: np.ndarray, kp: float, kd: float, ki: float
    ) -> None:
        gen = self._generation
        with self._lock:
            self._thread_phase = ("fitting", 0.0)
            self._thread_result = None
        if self.run_compute_inline:
            self._compute(rows, kp, kd, ki, gen)
            return
        self._thread = threading.Thread(
            target=self._compute,
            args=(rows, kp, kd, ki, gen),
            daemon=True,
            name="autotune-compute",
        )
        self._thread.start()

    def _compute(
        self, rows: np.ndarray, kp: float, kd: float, ki: float, gen: int
    ) -> None:
        """Fit + design (background thread; inline in tests)."""

        def fit_progress(p: float) -> None:
            with self._lock:
                self._thread_phase = ("fitting", p)

        def design_progress(p: float) -> None:
            with self._lock:
                self._thread_phase = ("designing", p)

        fit = fit_plant(
            rows, self._script, progress_cb=fit_progress, cancel=self._cancel
        )
        if fit is None:
            with self._lock:
                self._thread_result = {"generation": gen, "error": "fit failed"}
            return
        with self._lock:
            self._thread_phase = ("designing", 0.0)
        design = design_gains(
            fit, kp, kd, ki,
            progress_cb=design_progress, cancel=self._cancel,
        )
        with self._lock:
            if design is None:
                self._thread_result = {
                    "generation": gen, "error": "design cancelled",
                }
            else:
                self._thread_result = {
                    "generation": gen, "fit": fit, "design": design,
                }

    def _update_computing(
        self, kp: float, kd: float, ki: float
    ) -> tuple[float, float, float, dict | None]:
        with self._lock:
            result = self._thread_result
            if result is not None:
                self._thread_result = None
        if result is None:
            return kp, kd, ki, None
        if result.get("generation") != self._generation:
            return kp, kd, ki, None          # stale session — drop
        if "error" in result:
            self._state = "idle"
            msg = f"autotune: {result['error']}"
            self.last_message = msg
            self._log.info("COMPUTE FAILED: %s", result["error"])
            return kp, kd, ki, {"type": "compute_failed", "message": msg}

        fit: PlantFit = result["fit"]
        design: GainDesign = result["design"]
        self.plant_fit = fit
        self._design = design
        self._state = "suggestion_ready"
        self.suggested_kp = design.kp
        self.suggested_kd = design.kd
        self.suggested_ki = design.ki
        self._log.info(
            "SUGGESTION  kp=%.4f kd=%.4f ki=%.4f  J=%.3f  plant: g=%.1f "
            "L=%.3fs stiction=%.2f  conf=%s %s",
            design.kp, design.kd, design.ki, design.predicted_cost,
            fit.params.g_eff, fit.params.latency_s, fit.params.stiction_deg,
            "LOW" if fit.low_confidence else "ok", design.notes,
        )
        msg = (
            f"suggest kp={design.kp:.4f} kd={design.kd:.4f} "
            f"ki={design.ki:.4f} (predicted J={design.predicted_cost:.2f}; "
            f"plant g={fit.params.g_eff:.0f} L={fit.params.latency_s * 1000:.0f}ms "
            f"stiction={fit.params.stiction_deg:.2f}deg"
            f"{'; LOW CONFIDENCE' if fit.low_confidence else ''})"
        )
        self.last_message = msg
        event: dict = {
            "type": "suggestion",
            "message": msg,
            "kp_suggested": design.kp,
            "kd_suggested": design.kd,
            "ki_suggested": design.ki,
            "predicted_cost": design.predicted_cost,
            "applied": 0.0,
        }
        if self.auto_apply:
            self.has_suggestion = False
            event["applied"] = 1.0
            event["message"] = "applied " + msg
            self.last_message = event["message"]
            return design.kp, design.kd, design.ki, event
        self.has_suggestion = True
        return kp, kd, ki, event

    # ---------------------------
    # Telemetry (7 keys FROZEN from the old contract + additive)
    # ---------------------------

    def telemetry(self) -> dict[str, Any]:
        with self._lock:
            phase, progress = self._thread_phase
        if not self.enabled:
            phase, progress = "idle", 0.0
        elif self._state == "suggestion_ready":
            phase, progress = "suggestion_ready", 1.0
        fit = self.plant_fit
        return {
            "pd_autotune_enabled": self.enabled,
            "pd_autotune_auto_apply": self.auto_apply,
            "pd_autotune_trial_count": self.trial_count,
            "pd_autotune_message": self.last_message,
            "pd_autotune_has_suggestion": self.has_suggestion,
            "pd_autotune_suggested_kp": self.suggested_kp,
            "pd_autotune_suggested_kd": self.suggested_kd,
            # Additive (SysID rework)
            "pd_autotune_suggested_ki": self.suggested_ki,
            "pd_autotune_phase": phase,
            "pd_autotune_progress": float(progress),
            "pd_autotune_plant_g_eff": 0.0 if fit is None else fit.params.g_eff,
            "pd_autotune_plant_latency_s": (
                0.0 if fit is None else fit.params.latency_s
            ),
            "pd_autotune_plant_stiction_deg": (
                0.0 if fit is None else fit.params.stiction_deg
            ),
            "pd_autotune_predict_s": 0.0 if fit is None else fit.predict_s,
            "pd_autotune_predicted_cost": (
                0.0 if self._design is None else self._design.predicted_cost
            ),
        }
