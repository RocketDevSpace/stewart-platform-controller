# Changelog

## Milestone 1 — Foundation

### Step 1 — core/platform_state.py
- Added Pose, ServoAngles, BallState dataclasses
- Establishes typed data contracts for use across all modules

### Step 2 — settings.py
- Runtime config extracted from hardcoded values in existing codebase
- Single source of truth for port, baud, intervals, safety limits

### Step 3 — core/safety.py
- Servo clipping logic extracted and unit tested
- Exact behavioral match to safety_clip_servos in gui/gui_layout.py
