# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [Unreleased]

---

## [Milestone-8] - 2026-06-11

Housekeeping milestone opening the post-refactor hardening phase (M8–M12).

### Added
- `requirements.txt` / `requirements-dev.txt` — runtime and dev/CI dependencies,
  previously only implicit in `ci.yml` and CLAUDE.md.

### Changed
- `CLAUDE.md` — "Refactor state" (stale at "M6 active") replaced with "Project
  state": refactor complete (M1–M7), hardening roadmap M8–M12; architecture tree
  gains `cv/vision_control_worker.py` and `gui/vision_monitor.py`.
- `PROJECT_STATE.md` — current phase, decisions log (velocity filter placement,
  servo 4 geometry correction, M9 slew-guard applies to manual sends), phase
  roadmap with hardening milestones, refreshed open questions.
- `config.py` — coordinate comment blocks above `SERVO_SHAFTS_XY` and
  `PLATFORM_POINTS_LOCAL_XY` described an older measurement iteration; replaced
  with the actual generation methodology (measured 0/5 mirror pair rotated
  ±120° about Z) and 2026-06-11 validation results. Comments only.

### Removed
- 11 tracked `__pycache__/*.pyc` files committed before `.gitignore` existed,
  including binaries of long-deleted modules (`gui_layout`, `gui_main`,
  `cv/ball_controller`).

### Fixed (pre-M8 standalone PRs, same day)
- PR #14 — tracker velocity low-pass filter (`settings.BALL_VEL_FILTER_ALPHA`),
  replacing the hardcoded `vel_alpha` smoother; filter state resets on tracking
  loss so stale velocity no longer kicks the d-term on reacquire.
- PR #15 — `config.py` servo 4 shaft Y coordinate: −99.920 → −99.290 (digit
  transposition; validated by radius + 120° rotation-symmetry check).

---

## [Milestone-7] - 2026-05-12

### Added
- `cv/vision_control_worker.py` — `VisionControlWorker(QObject)` runs the full
  vision loop (`BallTracker` → `BallController` → `ServoDriver`) in a `QThread`;
  `ControlSnapshot` dataclass carries per-frame diagnostics (ball state, timing,
  control terms, raw camera/warped/mask frames) back to the GUI thread via Qt signal.
  Backpressure mechanism drops frames when the GUI cannot keep up.
- `gui/vision_monitor.py` — floating debug window with three camera views: warped
  (perspective-corrected) with vector overlays, raw camera, HSV mask. Overlays on
  warped view: ball circle, displacement-to-target arrow (white), velocity arrow
  (orange), PD restoration arrow (magenta), legend, position/velocity/PD text.
- ~60 new runtime constants in `settings.py` sourced from Codex audit: camera
  backend/resolution/FPS, adaptive exposure, software gain, ArUco detection params,
  HSV range defaults, position filter, auto-trim, PD autotune, loop rates.

### Changed
- `cv/ball_tracker.py` — background capture thread decouples frame acquisition
  from the control loop; camera backend probing selects the fastest available
  backend; `configure()` sets resolution/FPS/exposure from `settings`; adaptive
  exposure adjusts based on frame brightness; software brightness/contrast gain
  applied when needed; ArUco detection uses scale, caching, and hold-last-good
  logic; adaptive HSV range; position EMA filter; stores
  `_last_camera_bgr`, `_last_warped_bgr`, `_last_mask_gray` for snapshot delivery.
- `control/ball_controller.py` — auto-trim: running integral correction converges
  ball to target under persistent table tilt; `compute_with_terms()` returns full
  diagnostic dict (`position_vec_mm`, `velocity_vec_mm_s`, `pd_vec`, timing);
  PD autotune: two-leg step test yields Kp/Kd recommendations; slew limiter caps
  per-frame command change; d-term cap prevents derivative spike.
- `gui/control_panel.py` — three-column layout: 6-axis sliders + HSV | ball target
  + trim + PD | routines + vision + commands; compact sliders (max height 20 px);
  Ball Target X/Y sliders; Roll/Pitch Trim sliders; Auto-Trim, Calibrate Home,
  Reset Trim, PD AutoTune buttons; "Save Trim as Default" button writes current
  trim back to `settings.py`; dark mode QSS exported as `DARK_QSS`; vision monitor
  button grouped with enable/cancel.
- `gui/main_window.py` — `VisionControlWorker` owns the vision thread; snapshot
  handler routes frames to `VisionMonitorWindow` and timing plot; rolling 30-second
  timing chart; neutral-pose safety fallback on sustained ball loss; valid-streak
  reacquisition gating; `_on_save_trim_as_default()` rewrites trim constants in
  `settings.py` in place; 3D visualizer canvas uses `setMinimumSize` (not fixed).
- `visualization/visualizer3d.py` — `_apply_dark_style()` reapplied after every
  `ax.cla()` call: pane facecolors, edgecolors, tick/label/title colors all dark.
- `.github/workflows/ci.yml` — restructured so mypy runs before PyQt5 is
  installed (avoids strict 5.15.x stub errors on unqualified `Qt.Xxx` enum style);
  system Qt libs (`libgl1`, `libegl1`, xcb family) installed before pytest;
  `PyQt5` and `opencv-python-headless` added for test step; `QT_QPA_PLATFORM:
  offscreen` set for headless CI.

### Fixed
- `gui/main_window.py:_trim_preview_log` — added `None` guard before
  `doc.blockCount()` to satisfy mypy `QTextDocument | None` return type.
- `tests/test_vision_control_worker.py` — `union-attr` mypy error on
  `snap.ball_state.x_mm` (added `is not None` guard); `no-untyped-def` on
  `_get_app()` (added `-> object:` return annotation).
- CI: `libgl1-mesa-glx` package renamed to `libgl1` + `libegl1` in Ubuntu 22.04.

---

## [Milestone-6] - 2026-05-10

### Added
- `control/ball_controller.py` — `BallController` moved from `cv/` to `control/`;
  now imports `BallState` from `core.platform_state` (local duplicate removed);
  `compute()` accepts `BallState` only (dict fallback removed); timing print gated
  on `settings.DEBUG_PRINTS`.
- `tests/test_ball_controller.py` — 13 unit tests covering None input, disabled
  controller, roll/pitch output direction (all four quadrants), clamping at
  `max_tilt_deg`, and `set_gains()` behaviour.

### Changed
- `cv/ball_tracker.py` — `update()` returns `BallState` (from `core.platform_state`)
  instead of a raw dict; three timing `print()` calls gated on `settings.DEBUG_PRINTS`.
- `setup.cfg` — exclude lists trimmed: `comms`, `gui/gui_layout_legacy.py`,
  `gui/gui_main.py` removed (files deleted); `cv/` directory exclude narrowed to
  `cv/ball_tracker.py` (ball_controller moved to `control/`); `[mypy-cv.*]`
  override narrowed to `[mypy-cv.ball_tracker]`.

### Removed
- `cv/ball_controller.py` — moved to `control/ball_controller.py`.
- `comms/` — entire folder deleted (`__init__.py`, `arduino_protocol.py`,
  `serial_sender.py` all superseded by `hardware/` since M2; nothing imported them).
- `gui/gui_layout_legacy.py` — legacy monolith retired in M5, deleted in M6.
- `gui/gui_main.py` — orphaned shim, never wired into architecture.

---

## [Milestone-5] - 2026-04-23

### Added
- `gui/main_window.py` — top-level window; wires `SerialManager`,
  `ServoDriver`, `IKEngine`, `RoutineRunner`, `StewartVisualizer`,
  `ControlPanel`, `SerialMonitor`, `BallTracker`, `BallController`. Owns
  the routine and vision QTimers and the background visualizer thread.
- `gui/control_panel.py` — slider/button widget; signal-only public
  interface (`slider_changed`, `routine_selected`, `routine_cancelled`,
  `send_clicked`, `vision_toggled`, `kp_changed`, `kd_changed`,
  `raw_command_sent`) plus setter methods for view state.
- `gui/serial_monitor.py` — read-only display widget for the Arduino
  serial stream.

### Changed
- `main.py` — now imports `MainWindow`; `IKWrapper` removed (`MainWindow`
  instantiates `IKEngine` directly).
- `gui/gui_layout.py` renamed to `gui/gui_layout_legacy.py` (retired;
  scheduled for deletion in M6).
- `hardware/serial_manager.py` is now used directly by `MainWindow`;
  `_LegacySerialAdapter` bridge removed.
- `setup.cfg` — `gui/` removed from CI exclude lists; only
  `gui/gui_layout_legacy.py` and the pre-existing
  `gui/gui_main.py` shim remain excluded. Added `[mypy-cv.*]`,
  `[mypy-PyQt5.*]`, and `[mypy-matplotlib.*]` overrides.

### Removed
- `_LegacySerialAdapter` bridge class.
- `IKWrapper` class from `main.py`.

---

## [Milestone-4] - 2026-04-22

### Added
- `control/__init__.py`, `control/routine_runner.py` — Qt-free routine
  playback state machine; owns step list, current index, running flag, and
  preview/send mode; accepts tick() calls from an external timer
- `tests/test_routine_runner.py` — 14 unit tests covering load, preview
  wrap-around, send-mode IK + serial dispatch, exhaustion, and cancel

### Changed
- `gui/gui_layout.py` — routine playback delegates to `RoutineRunner`;
  removed `self.preview_mode`, `self.current_routine_steps`,
  `self.current_routine_index`, and dead `run_selected_routine()`; the
  send-routine path is now `load() + start_send()` (pre-compute loop,
  confirm dialog, and profiling harness removed per milestone spec)
- `setup.cfg` — added `[mypy-routines.*]` per-module override

---

## [Milestone-3] - 2026-04-22

### Added
- `core/ik_engine.py` — single IK call site; `IKEngine.solve()` wraps
  `kinematics/ik_solver.py`; module-level `solve_ik()` convenience function
- `tests/test_ik_engine.py` — unit tests for `IKEngine.solve()` and `solve_ik()`
  covering neutral pose, key dict shape, servo angle range, determinism, and
  exception-to-failure mapping

### Changed
- `visualization/visualizer3d.py` — refactored to accept a pre-solved
  `ik_result` dict; uses `IKEngine` as fallback; no longer imports
  `ik_solver` directly
- `setup.cfg` — removed `visualization/` from flake8 exclude list and mypy
  exclude regex; it is now covered by CI automatically
- `conftest.py` — added parent directory to `sys.path` so that legacy
  `from stewart_control.config import ...` paths resolve under pytest

---

## [Milestone-2] - 2026-04-22

### Added
- `hardware/__init__.py`, `hardware/serial_manager.py` — Serial connection
  lifecycle and background read loop; clean refactor of `comms/serial_sender.py`
  with no hardcoded config (port/baud injected by caller)
- `hardware/servo_driver.py` — Single call site for `"S,..."` command formatting
  and dispatch; replaces 4 inline builds in `gui/gui_layout.py` (switchover in M5)
- `tests/test_servo_driver.py` — Unit tests for `format_command` and `send_angles`
  via mock `SerialManager`; hardware test marked `[HARDWARE]` and skipped in CI

### Changed
- `.github/workflows/ci.yml` — Switched flake8 and mypy from explicit include
  list to exclude list in `setup.cfg`; added `types-pyserial` to CI dependencies
- `setup.cfg` — Added exclude lists for legacy modules so new clean modules are
  covered automatically without CI config changes

---

## [Milestone-1] - 2026-04-21

### Added
- `core/platform_state.py` — `Pose`, `ServoAngles`, `BallState` dataclasses;
  typed data contracts for all modules
- `settings.py` — Runtime config extracted from hardcoded values in existing
  codebase; single source of truth for port, baud, intervals, safety limits
- `core/safety.py` — Servo angle clipping logic extracted from
  `gui/gui_layout.py`; exact behavioural match verified by unit tests
- `tests/test_safety.py` — 17 unit tests for `clip_servo_angles`
- `CHANGELOG.md` — Project changelog
- `AI_WORKFLOW.md` — Development process, role definitions, phase gates,
  templates, and standing rules
- `SPEC.md` — Feature specs and acceptance criteria; source of truth for
  what gets built
- `.github/workflows/ci.yml` — CI pipeline (pytest, flake8, mypy) on every
  push and PR
- `.gitignore`, `setup.cfg`, `conftest.py` — Project tooling config

### Fixed
- Import paths in `core/safety.py` and `tests/` corrected to
  repo-root-relative (`from settings import ...` not `from stewart_control...`)
