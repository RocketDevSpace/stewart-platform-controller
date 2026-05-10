# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

---

## [Unreleased]

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
