# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [Unreleased]

Two bodies of work on branch `overhaul/safety-ik-vision-gui`, merging as one
PR: the 2026-07-20 firmware capture and the 2026-07-22 six-step overhaul
(absorbs milestones M9–M12; driven by the five-agent full-repo review — see
`docs/code-review-2026-07-22.md` for the finding-by-finding record).
The 2026-07-22 performance pass (branch `perf/latency-jitter`, stacked on
the overhaul) is recorded in its own subsection at the end of [Unreleased].

### Added
- `firmware/` — captured Uno firmware (2026-07-20): byte-for-byte flash dump
  (`flash_dump_2026-07-20.hex`, ground truth), reconstructed
  `stewart_platform_uno.ino` sketch (original source was never committed),
  `wiring_check.py` bench utility, and `firmware/README.md` with the pin map
  (servo i → D2+i), servo clocking vs `config.py` geometry, and the verified
  serial protocol semantics.
- `docs/code-review-2026-07-22.md` — full-repo review record; every finding
  maps to the overhaul step that addressed it.
- `core/safety.py::select_speed_delay` — large-move ramp policy (step 2):
  jumps over `SERVO_SLEW_INSTANT_MAX_DEG` are sent with a firmware
  `speedDelay` so the ramp happens in hardware; applies to manual sends too.
- `core/platform_state.py::IKResult` — frozen dataclass replacing the untyped
  IK result dict (step 3): per-servo `servo_status`, complete failure shape,
  neutral placeholders for failed servos.
- `cv/camera_source.py` — camera lifecycle split out of the tracker (step 4):
  backend probe, capture thread, runtime exposure policy, software gain;
  two-phase init (no I/O in `__init__`).
- `control/setpoint.py`, `control/auto_trim.py`, `control/autotune.py`,
  `control/pd_core.py` — controller decomposition (step 5): `ball_controller`
  becomes a ~300-line facade; injected clock throughout makes the state
  machines testable.
- `settings_store.py` + untracked `user_settings.json` — per-machine settings
  overlay (step 5): 12-key whitelist, atomic writes, corrupt input degrades
  to defaults; `settings.py` applies the overlay at import.
- `control/pose_commander.py` + `gui/timing_plot.py` — GUI slimming (step 6):
  zero IK call sites left in `gui/`; timing plot extracted with persistent
  Line2D artists.
- Test suite 90 → 206: fake-serial `SerialManager` suite, golden-value +
  sweep-continuity IK suites, synthetic-frame ArUco/tracker pipeline tests,
  `CameraSource` fake-capture tests, controller characterization suite with
  fake clock, settings store/overlay tests.

### Changed
- Tooling (step 1): local `mypy .` works (missing `cv/__init__.py` had broken
  it); CI installs from requirements files with PyQt5 present for mypy (local
  == CI); major-version ceilings on dependencies; `setup.cfg` exclude lists
  now empty — every module linted and type-checked.
- `hardware/serial_manager.py` rewritten (step 2): write lock, read loop
  survives callback exceptions, link death reported via disconnect callback,
  double-connect guard, `send_latest()` latest-wins writer thread for
  streaming setpoints; connect() waits for the firmware `[READY]` banner
  (3 s cap, old-firmware fallback) instead of a fixed 2 s sleep (step 6).
- `kinematics/ik_solver.py` rewritten (step 3): branch seeding by servo-angle
  validity, symmetric elbow handling with a single continuity-driven choice;
  out-of-range angles are per-servo failures, never silent clamps.
- Send-mode routines ease back to neutral on completion and cancel (step 3,
  approved behavior change); IK failures during playback are counted and
  surfaced in the GUI; routine labels state their real envelopes.
- `cv/ball_tracker.py` rewritten as pure detection (step 4):
  `process(frame, ts, gain) -> BallState | None`; no camera, no threads;
  worker migrated off all private reach-ins onto the public API; the 28
  zero-filled `trk_*` placeholder telemetry keys removed.
- Vision-mode reacquire gating moved into the worker command path, counting
  real processed frames (step 3); neutral-pose fallback moved into the worker
  miss branch on real frame counts (step 6).
- GUI (step 6): non-blocking serial connect on a background thread;
  mode mutual exclusion (vision locks routines/SEND/raw box, step 2); raw
  commands routed through validated `ServoDriver.send_raw` (step 2);
  `visualizer3d` and the timing plot use persistent artists + `draw_idle`
  instead of full rebuilds on the GUI thread; log widgets capped.
- `main.py` (step 2): custom `sys.excepthook` logs the traceback, attempts a
  ramped-neutral emergency shutdown, and disconnects — instead of PyQt5's
  `qFatal()` abort with servos live.

### Fixed
- IK branch-flip defect (M5's screw-routine servo snap at yaw=-35°):
  root-caused and redesigned; screw max step is now 2.7° where it commanded
  an instantaneous 180° snap (step 3).
- Stale ArUco homography could be served forever after marker loss; now
  bounded by `TRACKER_MAX_ARUCO_HOLD_FRAMES` with cache invalidation
  (step 4). Also: capture release race (segfault vector), torn-frame copy,
  position-filter reset on loss, `TRACKER_WARP_SIZE_PX` finally honored,
  exposure probes no longer starve the pipeline.
- Dead `SAFETY_LIMITS` global min/max keys now enforced: every servo is
  clamped 0–180 with length/finiteness validation on the single send path
  (step 2); the three duplicated inline clamp copies removed.
- `closeEvent` deadlock (guaranteed 3 s hang + "QThread destroyed while
  running" on close with vision active) (step 2).
- Save-trim regex rewrite of `settings.py` — which had corrupted trim values
  with unrounded repr floats — replaced by the overlay (step 5).
- Telemetry honesty: `frame_to_cmd` and `serial_enqueue` plot lines had never
  displayed a real measurement; timing-plot history was sized off the wrong
  rate by 4x (steps 4/6).
- Slider sync no longer fights the user mid-drag; autotune double-apply race
  guarded; vision UI fully resets on disable (step 6).

### Removed
- Dead config (`SERIAL_QUEUE_MAX`, `VISION_LOOP_INTERVAL_MS`, and 3 more),
  the `stewart_control` conftest alias, and dead tracker/controller state
  (step 1/4/5).
- `BallController.compute()` — divergent from the `compute_with_terms()` path
  production actually ran; its 16 headline tests migrated to the real path,
  tautological tests deleted (step 5).
- Dead `ServoAngles` contract and `solve_ik()` wrapper (step 3).

### Performance pass (branch `perf/latency-jitter`, 2026-07-22)

Six measured steps stacked on the overhaul; every change gated by
`tools/jitter_bench.py` A/B numbers. Headlines: rest-state servo commands
6125 integer flips/min → 0 (one send per 30 s at rest); serial RTT
57 → 4.4 ms p50; camera stays 30 fps (hardware-capped).

#### Added
- `tools/jitter_bench.py` — headless A/B bench driving the real
  MeasurementFilter → BallController → IK → ServoDriver chain on synthetic
  profiles (quiescent/step3hz/ramp/impulse) or a recorded CSV; reports
  integer flip rate, send rate, command std, d-term saturation (step 1).
- `tools/latency_bench.py` (on-rig command→ack RTT percentiles) and
  `tools/camera_probe.py` (fps × exposure sweep with recommendation); both
  exit gracefully without hardware. `VISION_POSITION_LOG_PATH` session
  recorder dumps `t,x,y` per valid frame for bench replay (step 1).
- Serial RTT telemetry: write-stamp/ack matching in `SerialManager`,
  `rtt_stats()` EMA surfaced through the worker timing dict into the
  timing plot (step 1).
- `cv/measurement_filter.py` — tracker filtering extracted into a pure
  module; new adaptive alpha-beta filter (constant-velocity model, gains
  scheduled by innovation 1→4 mm and predicted speed 60→150 mm/s) with
  modes `alpha_beta` (default) / `legacy` (regression reference, pinned
  byte-identical) / `raw` (bench only) (steps 1+3).
- `control/rest_gate.py` — near-target rest mode: ball within 6 mm with
  LPF speed ≤ 12 mm/s sustained 0.5 s → hold LEVEL + trim; exit hysteretic
  and same-cycle (radius > 10 mm OR raw speed > 25 mm/s → full PD, no
  filter, no timer). Auto-trim stays live during rest and keeps walking
  the ball toward center (step 4).
- `firmware/stewart_platform_uno_v2/` + `flash_dump_v2_2026-07-22.hex`
  (new ground truth; v1 dump retained as rollback): tenth-degree `T`
  protocol via `writeMicroseconds` with terse `k`/`e` acks, legacy `S`
  protocol bit-compatible with shared position state, 250000 baud,
  `[READY v2]` banner, no-String non-blocking parser (step 5).

#### Changed
- `hardware/servo_driver.py` — Schmitt-trigger quantizer
  (`SERVO_QUANT_HYST_DEG`) + identical-command dedup: rounding-boundary
  noise commits nothing, large jumps land in one call; `send_raw` bypasses
  the Schmitt but keeps committed state truthful (step 2). Hybrid dispatch:
  v2 + small streaming move → `T` on the 0.1° grid (hysteresis 0.15°);
  large jumps and v1/forced-legacy → `S` with the firmware ramp (step 5).
- `hardware/serial_manager.py` — parses the firmware version from the boot
  banner; auto-falls back to 115200 when no banner appears at the
  configured rate, so `SERIAL_BAUD = 250000` is safe on either firmware
  (step 5).
- `cv/ball_tracker.py` — ArUco detection every frame (the old freeze-10/
  re-solve cadence injected a ~3 Hz position stairstep); detector-level +
  full-res sub-pixel corner refinement; deadband + scheduled-alpha
  marker-center filter (H bit-identical frame to frame at rest); sub-pixel
  ball centroid (~0.2 mm noise floor, was 0.5 mm steps) (step 3).
- `cv/camera_source.py` + `cv/vision_control_worker.py` — event-driven
  tick: a frame callback fires processing the moment a frame arrives
  (saves 0–8 ms, mean ~4 ms, vs the 8 ms poll grid; the 120 Hz QTimer
  stays as fallback heartbeat); single-pass copy+chirality-flip into
  reusable buffers; software gain into a preallocated buffer (step 6).
- Measured decisions recorded: rig webcam hard-capped ~31 fps at 640×480
  MJPG under DSHOW and MSMF — staying at 30 fps; IK vectorization skipped
  by its measure gate (0.29 ms/call < 1.0 ms threshold) (step 6).

#### Fixed
- Serial read latency: the reader thread's `ser.read(128)` waited for
  128 bytes or the 100 ms timeout, delaying every received line by up to
  100 ms. Now blocks for 1 byte and drains `in_waiting` — v1 RTT fell
  57 → 9.7 ms p50 from this fix alone (step 5).
- Latent mutate-after-emit race on debug frames: per-tick references
  leaked into snapshots; debug views are now copied only at snapshot
  emission (≤ 30 Hz) (step 6).
- D-term derivative kick on target steps pinned by tests: D acts on
  measured velocity, not the error derivative; cap still clamps at
  `PD_D_TERM_LIMIT_DEG` (step 4).

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
