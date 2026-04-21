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

### PM additions (post-merge)
- AI_WORKFLOW.md and SPEC.md pushed to main
- CI workflow added (.github/workflows/ci.yml)
- Import paths corrected in core/safety.py and tests/
