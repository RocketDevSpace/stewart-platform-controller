# Codex Audit — offset-tuning-and-camera-exposure

**Branch audited:** `codex/offset-tuning-and-camera-exposure`
**Audit date:** 2026-05-10
**Auditor:** PM Claude
**Purpose:** Phase 1 of the Codex integration milestone (M7). Maps every meaningful file and feature from the Codex branch to its target in the post-M6 refactored codebase, enabling implementation PRs with full coverage and no dropped features.

---

## Module Mapping

| Codex File | Refactored Target | Notes |
|---|---|---|
| `config.py` (camera / tracker / loop-rate sections) | `settings.py` | ~35 new constants; no geometry changes |
| `config.py` (auto-trim section) | `settings.py` | ~12 new constants |
| `config.py` (PD autotune section) | `settings.py` | ~10 new constants |
| `cv/ball_tracker.py` | `cv/ball_tracker.py` | Major rewrite; background thread, adaptive exposure, ArUco improvements, position filter |
| `cv/ball_controller.py` | `control/ball_controller.py` | Significant additions: auto-trim, PD autotune, slew limit, compute_with_terms() |
| `cv/vision_control_worker.py` | **NEW** `cv/vision_control_worker.py` | Entirely new module; QObject/QThread worker with ControlSnapshot |
| `gui/gui_layout.py` | `gui/control_panel.py` + **NEW** `gui/vision_monitor.py` | New monitor window; new controls for target, trim, autotune |
| `gui/gui_main.py` | `gui/main_window.py` | Major additions: worker wiring, timing plot, bidirectional state sync, neutral-pose fallback |
| `routines/routines.py` | `routines/` | Identical to refactored version — no porting needed |
| `docs/feedback-loop-optimizations.md` | `docs/` (reference) | 5 latency recommendations; port as improvements, not direct code copy |

---

## Commit Log

Commits listed oldest → newest. Merge commits omitted.

| SHA | Date | Message | Functional Change |
|---|---|---|---|
| `d418bdd3` | 2026-02-22 | Initial clean commit | Baseline Codex monolith |
| `8b3afef0` | 2026-02-24 | add cv script | Initial `BallTracker`; HSV blob detection, ArUco homography |
| `35d4b5241` | 2026-02-24 | updated cv script to work with new geometry and added velocity calculation | Velocity via finite diff; platform geometry calibration |
| `bc291a3e` | 2026-02-24 | change cv to a class to return values | `BallTracker` becomes instantiable class |
| `b3e986cc` | 2026-02-24 | added ball balancing feature | `BallController` added; PD loop closes on roll/pitch |
| `f4ad30f3` | 2026-02-25 | pitch and roll offsets, tuning pd values | Manual pitch/roll trim constants; kp/kd baseline values |
| `60836d71` | 2026-02-25 | move visualizer rendering out of main loop | Visualizer update decoupled from vision loop rate |
| `85cc7ac7` | 2026-02-27 | Full Codex Refactor | Package layout, `config.py`, all classes restructured |
| `c345fd64` | 2026-02-27 | fix camera vision, add pd graphics, add timing telemetry graphs | Matplotlib timing plot; PD telemetry overlay on camera frame |
| `0bb449bd` | 2026-02-27 | gui refactor, embed camera, dark mode | Dark mode QSS stylesheet; camera view embedded in main window |
| `302cee6a` | 2026-02-27 | split camera views into new window | `VisionMonitorWindow` extracted to separate `QWidget` |
| `c61f8e48` | 2026-02-27 | move visualizer to main window, speed up ball tracker frequency | Visualizer in main window; vision loop frequency raised |
| `6f3f1a14` | 2026-02-27 | add some neutral bias and fix cap period inflating | Cap period measurement fix; neutral-position bias in controller |
| `65f30948` | 2026-02-27 | reducing lag, running at 30 hz | QTimer-based loop rate tuning; latency reduction focus |
| `94ca4a2d` | 2026-02-27 | add PD autotune functionality, fix HSV errors with exposure changes | `_PDLegEvaluator`; two-leg step test; kp/kd recommendation; HSV handling across exposure changes |
| `f8d01f89` | 2026-02-27 | attempt to add offset tuning to PD autotune | Partial auto-offset tuning experiment (superseded by auto-trim commits) |
| `13890ae9` | 2026-02-27 | add feedback loop optimization recommendations | `docs/feedback-loop-optimizations.md` added |
| `82f22746` | 2026-03-07 | current attempt to add autohome, not fully functional | Auto-trim / auto-home initial implementation |
| `fad0a5c3` | 2026-03-07 | autohome mostly working but centering on slightly wrong point | Auto-trim calibration logic refinement; settle/hold gating |
| `f249eaa2` | 2026-03-07 | raise max degree trim so autotrim won't overwrite ideal values | `AUTO_TRIM_HOME_CAL_MAX_DEG` raised; prevents autotrim from overwriting manual calibration |

---

## Feature Inventory

### FG-1: Background Capture Thread
**Codex file:** `cv/ball_tracker.py`
**Target:** `cv/ball_tracker.py`
**What it does:** A daemon thread (`_capture_loop`) continuously reads frames from the camera into a shared buffer (`_latest_frame`, `_latest_frame_ts`). The main `update()` method reads from the buffer instead of calling `cap.read()` directly, decoupling capture latency from the vision loop. Stale frames (where `frame_ts <= _last_processed_frame_ts`) return early with `reason="stale_frame_repeat"` and do not advance controller state.

**Why it matters:** The most impactful single latency fix. `cap.read()` blocks for up to one frame period (~33ms at 30fps). The background thread eliminates this from the hot path.

**Key new state:** `_capture_thread`, `_latest_frame`, `_latest_frame_ts`, `_capture_running` flag, `_last_processed_frame_ts`.

---

### FG-2: Camera Backend Selection
**Codex file:** `cv/ball_tracker.py`
**Target:** `cv/ball_tracker.py`
**What it does:** `_open_best_camera()` tries DSHOW, MSMF, and ANY backends in sequence. For each successful open it measures the frame period and mean brightness over a short warmup, then picks the fastest. Falls back gracefully if a backend fails to open.

**Why it matters:** DSHOW is generally fastest on Windows but can fail on some hardware. The probe-and-pick approach makes startup robust.

**Config:** `CAMERA_FORCE_BACKEND` (override to skip probing), `CAMERA_TARGET_FPS`, `CAMERA_BUFFER_SIZE`.

---

### FG-3: Camera Configuration
**Codex file:** `cv/ball_tracker.py`
**Target:** `cv/ball_tracker.py`
**What it does:** `_configure_camera()` sets FOURCC=MJPG, FPS, buffer size, exposure, gain, and `autofocus=0`. Bootstraps by first enabling auto-exposure, waiting for warmup, then switching to manual if `CAMERA_AUTO_EXPOSURE=False`. Reads back actual period and brightness after configuration.

**Config:** `CAMERA_AUTO_EXPOSURE`, `CAMERA_EXPOSURE`, `CAMERA_BUFFER_SIZE`, `CAMERA_FORCE_BACKEND`.

---

### FG-4: Runtime Adaptive Exposure
**Codex file:** `cv/ball_tracker.py`
**Target:** `cv/ball_tracker.py`
**What it does:** `_evaluate_runtime_camera_policy()` runs every `CAMERA_RUNTIME_CHECK_S` seconds after a warmup period. If the measured frame period exceeds `CAMERA_RUNTIME_MAX_PERIOD_MS` or brightness falls below `CAMERA_RUNTIME_MIN_GRAY`, it probes a set of manual exposure candidates and adopts the fastest that meets the brightness floor.

**Why it matters:** Camera auto-exposure can destabilize the capture period. This keeps the loop tight at runtime.

**Config:** `CAMERA_RUNTIME_ADAPTIVE`, `CAMERA_RUNTIME_WARMUP_S`, `CAMERA_RUNTIME_CHECK_S`, `CAMERA_RUNTIME_MAX_PERIOD_MS`, `CAMERA_RUNTIME_MIN_GRAY`, `CAMERA_RUNTIME_TARGET_GRAY`.

---

### FG-5: Software Brightness Gain
**Codex file:** `cv/ball_tracker.py`
**Target:** `cv/ball_tracker.py`
**What it does:** `_update_software_gain()` and `_apply_software_brightness()` scale the frame with a gain coefficient when the camera is persistently dark. Gain is bounded by `CAMERA_RUNTIME_SOFT_GAIN_MAX`.

**Why it matters:** When the camera's minimum manual exposure still produces a dark image, software gain recovers enough brightness for HSV detection without changing the exposure (which would slow the capture period).

---

### FG-6: ArUco Improvements
**Codex file:** `cv/ball_tracker.py`
**Target:** `cv/ball_tracker.py`
**What it does:**
- Detection at 0.5× scale (`TRACKER_ARUCO_DETECT_SCALE`) — much faster than full resolution.
- Histogram equalization retry when detection fails at reduced scale.
- Center low-pass filter (`_filter_marker_center`, alpha=`TRACKER_ARUCO_CENTER_FILTER_ALPHA=0.70`) smooths jitter in marker position between frames.
- Homography cache (`_last_H`) — recalculates only when markers move significantly.
- Hold frames (`TRACKER_MAX_ARUCO_HOLD_FRAMES=3`) — uses cached homography when ArUco temporarily disappears (ball occlusion).
- Re-detect every N frames (`TRACKER_ARUCO_REDETECT_EVERY_N=5`) rather than every frame.

**Config:** `TRACKER_ARUCO_DETECT_SCALE`, `TRACKER_ARUCO_CENTER_FILTER_ALPHA`, `TRACKER_MAX_ARUCO_HOLD_FRAMES`, `TRACKER_ARUCO_REDETECT_EVERY_N`.

---

### FG-7: Adaptive HSV Detection
**Codex file:** `cv/ball_tracker.py`
**Target:** `cv/ball_tracker.py`
**What it does:**
- Adaptive V-min relaxation: if `gray_warp_mean < 45`, relaxes V-min threshold proportionally to compensate for dark warped image.
- Two-pass detection: if first HSV pass finds no candidates, retries with further relaxed V-min.
- Ball quality checks: circularity, fill ratio, and speed outlier rejection added beyond existing radius/area filters.

**Config:** `TRACKER_MIN_RADIUS_PX`, `TRACKER_MIN_CONTOUR_AREA`, HSV defaults in settings.

---

### FG-8: Position Filter
**Codex file:** `cv/ball_tracker.py`
**Target:** `cv/ball_tracker.py`
**What it does:** Optional low-pass filter on ball position with adaptive alpha: slow alpha when ball moves slowly (reject noise), fast alpha when ball moves fast (track accurately). Bounded by `pos_filter_max_lag_mm` to prevent excessive lag at high speed.

**Config:** `TRACKER_POS_FILTER_ENABLED`, plus alpha/lag constants.

---

### FG-9: BallController — Auto-Trim
**Codex file:** `cv/ball_controller.py`
**Target:** `control/ball_controller.py`
**What it does:** Slow integral correction (`_update_auto_trim`) that adjusts `pitch_offset`/`roll_offset` when the ball is consistently near the target. Gates on: ball speed below threshold (via LPF), ball radius below threshold (via LPF), and optional target-hold timer. Step-limited per update. Separate tighter limit for home calibration (`AUTO_TRIM_HOME_CAL_MAX_DEG`). State machine with reason strings for diagnostics.

**Why it matters:** Corrects for table tilt, camera angle, or neutral-position bias without manual tuning.

**Config:** `AUTO_TRIM_ENABLED`, `AUTO_TRIM_KI_DEG_PER_MM_S`, `AUTO_TRIM_MAX_DEG`, `AUTO_TRIM_HOME_CAL_MAX_DEG`, `MANUAL_ROLL_TRIM_DEG`, `MANUAL_PITCH_TRIM_DEG`, plus settle/hold params.

---

### FG-10: BallController — PD Autotune
**Codex file:** `cv/ball_controller.py`
**Target:** `control/ball_controller.py`
**What it does:** `_PDLegEvaluator` runs a two-leg step test (diagonal targets at ±N mm). Evaluates settle time, overshoot ratio, oscillation crossings, and IAE. `_update_pd_autotune` and `_recommend_pd_update` produce kp/kd adjustments. Supports both manual-apply and auto-apply modes. Emits recommendation events through `control_terms` dict.

**Config:** `PD_AUTOTUNE_ENABLED`, step size, min/max kp/kd, settle timeout.

---

### FG-11: BallController — Slew Limit + Term Cap
**Codex file:** `cv/ball_controller.py`
**Target:** `control/ball_controller.py`
**What it does:** `_apply_slew_limit()` caps the rate of change of the roll/pitch command between frames. `d_term_limit_deg` caps the derivative term independently to prevent kick from velocity spikes.

**Why it matters:** Prevents jerky platform motion during ball reacquisition or sudden tracking updates.

---

### FG-12: BallController — compute_with_terms()
**Codex file:** `cv/ball_controller.py`
**Target:** `control/ball_controller.py`
**What it does:** Extended version of `compute()` that returns `(roll, pitch, terms_dict)`. The `terms_dict` is a rich diagnostic package including: `position_vec_mm`, `p_term`, `d_term`, `pd_vec`, `roll_raw`, `pitch_raw`, `roll_clamped`, `pitch_clamped`, `roll_cmd`, `pitch_cmd`, `kp`, `kd`, `target_x_mm`, `target_y_mm`, `roll_offset`, `pitch_offset`, all auto-trim state fields, all PD autotune state fields.

**Why it matters:** Enables GUI state sync, diagnostics logging, and the timing plot without coupling the controller to the GUI.

---

### FG-13: VisionControlWorker (New Module)
**Codex file:** `cv/vision_control_worker.py`
**Target:** **NEW** `cv/vision_control_worker.py`
**What it does:**
- `VisionControlWorker(QtCore.QObject)` owns `BallTracker` and `BallController`.
- Runs in a `QThread` via `start()` / `stop()` slots.
- `ControlSnapshot` dataclass: full telemetry package (timestamp, `ball_state`, `pose`, `servo_angles`, `ik_success`, `timings_ms`, `ik_result`, `control_terms`, `tracking_valid`, `reason`, `miss_count`, `camera_bgr`, `warped_bgr`, `mask_gray`, `worker_emit_perf_ts`).
- Stale frame: skip snapshot emission without resetting controller state.
- Miss count: on reacquisition after ≥1 miss, calls `reset_motion_state()` to prevent velocity spikes.
- Backpressure: `_snapshot_inflight` flag; GUI must call `mark_snapshot_consumed()` before next snapshot is emitted.
- `command_sender` callable: decoupled from serial implementation.
- Qt slots: `set_gains`, `set_pd_autotune_enabled`, `set_pd_autotune_auto_apply`, `apply_pd_autotune_recommendation`, `set_hsv`, `set_target`, `set_trim`, `set_auto_trim_enabled`, `reset_trim`, `set_home_calibration`, `mark_snapshot_consumed`, `stop`.
- Signals: `snapshot_ready(ControlSnapshot)`, `camera_ready(dict)`, `error(str)`, `stopped()`.

**Why it matters:** This is the architectural core of the vision integration. Moves the entire vision control loop off the GUI thread, decouples serial from snapshot timing, and enables the backpressure mechanism that prevents GUI flooding.

---

### FG-14: GUI — VisionMonitorWindow
**Codex file:** `gui/gui_layout.py`
**Target:** **NEW** `gui/vision_monitor.py`
**What it does:** Separate `QWidget` window with three `QLabel` views: Camera View (raw BGR), Warped Platform View (homography-corrected), HSV/Mask View (binary mask). Populated via `_update_camera_views()` in main window using `_cv_to_pixmap()`.

---

### FG-15: GUI — New Controls
**Codex file:** `gui/gui_layout.py`
**Target:** `gui/control_panel.py`
**What it does:** New UI elements:
- Target X / Target Y sliders (±60mm range)
- Roll Trim / Pitch Trim sliders (±8.0° range, 0.01° resolution)
- Auto-Trim enable/disable button
- Calibrate Home / Cancel Calibration button
- Reset Trim to Config button
- PD AutoTune enable button
- PD AutoTune apply-now button
- PD AutoTune auto-apply toggle button
- HSV sliders in GUI (replaces OpenCV trackbars)
- "Open Vision Monitor" button

---

### FG-16: GUI — Dark Mode Theme
**Codex file:** `gui/gui_layout.py`
**Target:** `gui/main_window.py` or `gui/control_panel.py`
**What it does:** Full dark mode QSS stylesheet applied to the top-level widget. Background `#0f1726`, accent `#38bdf8`, grid lines `#2a3b59`.

---

### FG-17: GUI — Telemetry Timing Plot
**Codex file:** `gui/gui_main.py`
**Target:** `gui/main_window.py`
**What it does:** Rolling 30-second matplotlib plot of per-step vision loop timings (`ball_update`, `pd_compute`, `ik_solve`, `serial_enqueue`, `visualizer_gui`, `frame_to_worker_ms`, `worker_to_gui_ms`, `frame_to_cmd`, `total`). Y-axis zoom via scroll wheel or pinch gesture. Summary label below plot with per-key averages. Tracker telemetry keys (`trk_*`) tracked separately in history for log output. Plot redraws every N frames (`_timing_plot_update_every=5`).

---

### FG-18: GUI — Bidirectional State Sync
**Codex file:** `gui/gui_main.py`
**Target:** `gui/main_window.py`
**What it does:** Sync methods that read state back from `ControlSnapshot.control_terms` and update GUI widgets without triggering signal emission loops:
- `_sync_gain_sliders_from_terms()` — kp/kd sliders
- `_sync_autotune_from_terms()` — autotune button states
- `_sync_target_from_terms()` — target X/Y labels
- `_sync_trim_from_terms()` — trim sliders and labels
- `_sync_auto_trim_from_terms()` — auto-trim button state
- `_sync_home_calibration_from_terms()` — calibrate button state

All use `blockSignals(True/False)` around slider updates to prevent feedback loops.

---

### FG-19: GUI — Neutral Pose Safety Fallback
**Codex file:** `gui/gui_main.py`
**Target:** `gui/main_window.py`
**What it does:** In `_on_control_snapshot`, when `miss_count >= 20` and at least 0.5s has elapsed since last neutral send, solves IK for `(0, 0, z_setpoint, 0, 0, 0)` and enqueues it with `policy="latest"`. Keeps the platform level during sustained ball loss instead of holding the last commanded angle.

---

### FG-20: GUI — Valid Streak Gating
**Codex file:** `gui/gui_main.py`
**Target:** `gui/main_window.py`
**What it does:** After reacquisition, `_valid_streak` must reach `TRACKER_REACQUIRE_VALID_FRAMES` before commands are sent. Prevents a single spurious detection from sending an aggressive command.

**Config:** `TRACKER_REACQUIRE_VALID_FRAMES` (new constant in settings.py).

---

### FG-21: Home Calibration Diagnostic Logging
**Codex file:** `gui/gui_main.py`
**Target:** `gui/main_window.py`
**What it does:** When `_home_calibration_active`, logs detailed auto-trim state every 0.5s: state machine state, gate reasons, speed/radius LPF values vs thresholds, settle/hold timers, trim steps, saturation flags, current trim values.

---

### FG-22: Performance Optimizations (from docs)
**Codex file:** `docs/feedback-loop-optimizations.md`
**Target:** `cv/ball_tracker.py`, `cv/vision_control_worker.py`
**What it does:** Five recommended improvements (in priority order):
1. Decouple serial command send from snapshot emission — command goes to serial immediately; GUI snapshot is throttled separately.
2. Replace `QTimer` polling with `threading.Event` — worker wakes on new frame instead of polling at fixed interval.
3. Double-buffer in `BallTracker` — eliminates per-frame image copy.
4. Cache warp brightness calculation every N frames.
5. Increase ArUco redetect interval from 5 to 8–12.

---

## New Constants Required in settings.py

### Camera
```python
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FORCE_BACKEND = "DSHOW"   # or "" to probe
CAMERA_AUTO_EXPOSURE = True
CAMERA_EXPOSURE = -4.0
CAMERA_BUFFER_SIZE = 1
CAMERA_TARGET_FPS = 30
CAMERA_RUNTIME_ADAPTIVE = True
CAMERA_RUNTIME_WARMUP_S = 1.0
CAMERA_RUNTIME_CHECK_S = 1.0
CAMERA_RUNTIME_MAX_PERIOD_MS = 42.0
CAMERA_RUNTIME_MIN_GRAY = 28.0
CAMERA_RUNTIME_TARGET_GRAY = 55.0
CAMERA_RUNTIME_SOFT_GAIN_MAX = 2.4
```

### Tracker
```python
TRACKER_WARP_SIZE_PX = 480
TRACKER_ARUCO_DETECT_SCALE = 0.5
TRACKER_ARUCO_REDETECT_EVERY_N = 5
TRACKER_MIN_RADIUS_PX = 4.0
TRACKER_MIN_CONTOUR_AREA = 150.0
TRACKER_ARUCO_CENTER_FILTER_ALPHA = 0.70
TRACKER_MAX_ARUCO_HOLD_FRAMES = 3
TRACKER_POS_FILTER_ENABLED = False
TRACKER_REACQUIRE_VALID_FRAMES = 3  # new; used in gui/main_window.py
# HSV defaults (already in settings.py — verify values match)
# H: 10–28, S: 83–255, V: 125–255
```

### PD controller
```python
PD_DEFAULT_KP = 0.045
PD_DEFAULT_KD = 0.022
BALL_TARGET_DEFAULT_X_MM = 0.0
BALL_TARGET_DEFAULT_Y_MM = 0.0
```

### Auto-trim
```python
MANUAL_ROLL_TRIM_DEG = -0.8
MANUAL_PITCH_TRIM_DEG = 4.6
AUTO_TRIM_ENABLED = True
AUTO_TRIM_KI_DEG_PER_MM_S = 0.008
AUTO_TRIM_MAX_DEG = 6.0
AUTO_TRIM_HOME_CAL_MAX_DEG = 8.0
# plus settle/hold timing params — see codex config.py
```

### PD Autotune
```python
PD_AUTOTUNE_ENABLED = False
# step size, min/max kp/kd, settle/timeout params — see codex config.py
```

### Loop rates
```python
VISION_LOOP_HZ = 120
VISUALIZER_HZ = 25
GUI_SNAPSHOT_HZ = 30
TIMING_PLOT_POINTS = 300
SERIAL_QUEUE_MAX = 256
GUI_LOG_MAX_LINES = 500
LOG_EVERY_N = 30   # log every N vision frames
```

---

## Proposed Implementation Order

Each item below maps to a logical PR boundary. Dependencies flow top-to-bottom.

| Step | Target File(s) | Feature Groups | Prerequisite |
|---|---|---|---|
| 1 | `settings.py` | All new constants | None |
| 2 | `cv/ball_tracker.py` | FG-1 (background thread) | Step 1 |
| 3 | `cv/ball_tracker.py` | FG-2, FG-3, FG-4, FG-5 (camera open + config + adaptive exposure + software gain) | Step 2 |
| 4 | `cv/ball_tracker.py` | FG-6, FG-7, FG-8 (ArUco improvements + adaptive HSV + position filter) | Step 3 |
| 5 | `control/ball_controller.py` | FG-9 (auto-trim) | Step 1 |
| 6 | `control/ball_controller.py` | FG-10, FG-11, FG-12 (PD autotune + slew + compute_with_terms) | Step 5 |
| 7 | **NEW** `cv/vision_control_worker.py` | FG-13 (full worker + ControlSnapshot) | Steps 4, 6 |
| 8 | `gui/control_panel.py` | FG-15, FG-16 (new controls + dark mode) | None (parallel with steps 2–6) |
| 9 | **NEW** `gui/vision_monitor.py` | FG-14 (monitor window) | Step 8 |
| 10 | `gui/main_window.py` | FG-17–FG-21 (worker wiring, timing plot, state sync, fallback) | Steps 7, 9 |
| 11 | `cv/ball_tracker.py`, `cv/vision_control_worker.py` | FG-22 (performance optimizations) | Step 10 |

Steps 1–6 can be done as one PR ("vision backend + controller enrichment") or split at step boundaries. Step 7 is a natural PR break. Steps 8–10 are "GUI integration" PR. Step 11 is an optional optimization pass.

---

## Open Questions for PM Review

1. **PR granularity:** Steps 2–4 (`ball_tracker.py` rewrite) are large. Should they ship as one PR or in three passes (thread, camera, detection)?
2. **Dark mode scope:** Does dark mode apply to the full app or only the vision-related panels?
3. **Backward compatibility:** `BallTracker.update()` already returns `BallState`. The Codex version returns a `dict`. Port should adopt the refactored signature (returning `BallState`) while adding all the new internal logic. Confirm this is the intent.
4. **config.py geometry constants:** The Codex `config.py` also contains platform geometry constants. These must NOT be ported — they belong in the existing `config.py` and are hardware-specific. Confirm dev Claude should treat all geometry constants as read-only.
5. **`comms/serial_sender.py`:** The Codex `gui_main.py` imports `SerialSender` from `stewart_control.comms.serial_sender`. Post-M6 refactor uses `hardware/` equivalents. Dev Claude must use `comms/serial_sender.py` was deleted in M6; the equivalent is `hardware/serial_manager.py` + `hardware/servo_driver.py`. Confirm mapping.
