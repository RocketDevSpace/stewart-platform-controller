# Changelog

## Milestone 1 — Foundation

### Step 1 — core/platform_state.py
- Added `Pose`, `ServoAngles`, `BallState` dataclasses
- Establishes typed data contracts for use across all modules

### Step 2 — settings.py
- Runtime config extracted from hardcoded values in existing codebase
- Single source of truth for port, baud, intervals, safety limits

### Step 3 — core/safety.py + tests/test_safety.py
- Servo clipping logic extracted and unit tested (17 tests, all passing)
- Exact behavioral match to `safety_clip_servos()` in `gui/gui_layout.py`

### Step 4 — CHANGELOG.md
- Project changelog created

## Milestone 2 — Hardware Layer

### Step 1 — hardware/serial_manager.py
- Clean refactor of comms/serial_sender.py
- No hardcoded config — port/baud injected by caller (sourced from settings.py)

### Step 2 — hardware/servo_driver.py
- Single command formatting and dispatch call site
- Replaces 4 inline "S,..." string builds in gui/gui_layout.py (switchover in M5)

### Step 3 — tests/test_servo_driver.py
- Unit tests for format_command and send_angles (15 tests via mock SerialManager)
- Hardware test marked [HARDWARE] and skipped in CI

### Step 4 — CI scope + CHANGELOG
- hardware/ added to flake8 and mypy in .github/workflows/ci.yml

---

### PM additions (post-merge)
- AI_WORKFLOW.md and SPEC.md pushed to main
- CI workflow added (.github/workflows/ci.yml)
- Import paths corrected in core/safety.py and tests/
