# Stewart Platform Controller — Project Spec

> Source of truth for what gets built. Updated by the PM when requirements change.
> Every feature that gets implemented must have a section here first.

---

## Project Purpose

A desktop application to control a 6-DOF Stewart platform via Arduino serial,
with 3D visualization, scripted motion routines, and closed-loop ball balancing
using computer vision.

---

## Current Functionality (as of refactor baseline)

- 6-axis manual control via GUI sliders (X, Y, Z, Roll, Pitch, Yaw)
- IK solver computing 6 servo angles from a 6-DOF pose
- 3D matplotlib visualizer showing platform geometry
- 5 scripted motion routines (cube path, circle, cone orbit, parabola dance, screw)
- Routine preview mode (visualizer only, no serial)
- Routine send mode (serial output, step-by-step at 20ms intervals)
- Serial communication to Arduino at 115200 baud
- Safety clipping on servo angles before send
- Ball tracking via ArUco corner markers + HSV color detection
- PD controller for ball balancing (roll/pitch output)
- Live Kp/Kd gain adjustment via GUI sliders
- Vision mode: closed-loop ball balance at ~50Hz

---

## Refactor Milestones

### Milestone 1 — Foundation
**Status:** Complete (2026-04-21)

**What it does:** Establishes data contracts, runtime config, and safety layer.
No behavior changes. Pure restructuring.

**Steps:**
1. `core/platform_state.py` — Pose, ServoAngles, BallState dataclasses ✅ merged
2. `settings.py` — runtime config constants
3. `core/safety.py` — servo clipping logic + unit tests
4. `CHANGELOG.md` — project changelog skeleton

**Out of scope:** No changes to existing logic files. No behavior changes.

**Acceptance criteria:**
- All new files importable with no errors
- Existing code unchanged and still runnable
- `clip_servo_angles` in `core/safety.py` is exact behavioral match to
  `safety_clip_servos` in `gui/gui_layout.py` (verified by unit test)
- All runtime literals in existing code match values in `settings.py`

**Test gate:** Unit tests for `core/safety.py`. Import smoke test. No hardware required.

---

### Milestone 2 — Hardware Layer
**Status:** Complete (2026-04-22)

**What it does:** Consolidates all serial command building and dispatch.

**Scope:**
- Create `hardware/serial_manager.py` (refactor of `comms/serial_sender.py`)
- Create `hardware/servo_driver.py` — single command formatting call site
- Remove inline `"S," + ...` command strings from `gui/gui_layout.py` (4 locations)
- Port/baud sourced from `settings.py`

**Out of scope:** No GUI restructuring. No IK changes.

**Acceptance criteria:**
- Single command-building code path
- No hardcoded port strings in logic files
- Existing send behavior unchanged (manual test with Arduino)

**Test gate:** Unit test for command formatting. Manual smoke test on hardware `[HARDWARE]`.

---

### Milestone 3 — IK Consolidation
**Status:** Complete (2026-04-22)

**What it does:** Creates a single IK call site. Removes IK from the visualizer.

**Scope:**
- Create `core/ik_engine.py`
- Refactor `visualization/visualizer3d.py` to accept pre-solved geometry
- All IK calls routed through `ik_engine.py`

**Acceptance criteria:**
- `ik_solver.solve_pose()` called only from `ik_engine.py`
- Visualizer does not import `ik_solver`
- No double IK solve on any code path

**Test gate:** Unit test confirming IK output unchanged. Manual visual check.

---

### Milestone 4 — Routine Runner Extraction
**Status:** Complete (2026-04-22)

**What it does:** Moves routine playback state machine out of the GUI.

**Scope:**
- Create `control/routine_runner.py`
- Extract all routine state and step logic from `gui/gui_layout.py`
- GUI calls `runner.start()`, `runner.cancel()`, `runner.tick()` only

**Acceptance criteria:**
- `gui_layout.py` contains no routine index/step logic
- All 5 routines work in preview and send modes
- Cancel works correctly

**Test gate:** Manual test of all 5 routines in both modes.

---

### Milestone 5 — GUI Split
**Status:** Complete (2026-05-10)

**What it does:** Breaks `gui_layout.py` into focused widgets.

**Scope:**
- Create `gui/main_window.py`, `gui/control_panel.py`, `gui/serial_monitor.py`
- `gui/gui_layout.py` retired

**Acceptance criteria:**
- All existing GUI functionality present
- No logic, IK calls, or serial command building in any gui/ file

**Test gate:** Full manual GUI smoke test.

**Notes:** Screw routine produces unexpected servo limit behaviour (3 servos
snap min→max ~2s in). Root cause is a pre-existing IK branch-switching
workspace issue at yaw=-35°, not an M5 regression. Root-caused and fixed in
the 2026-07-22 overhaul (branch-selection redesign in
`kinematics/ik_solver.py`; see the Overhaul section below and
`docs/code-review-2026-07-22.md`, finding C1).

---

### Milestone 6 — Vision Loop Cleanup
**Status:** Complete (2026-05-10, PR #7)

**What it does:** Wires CV through clean interfaces. Cleans up debug output.

**Scope:**
- `BallTracker` returns `BallState` dataclass (not raw dict)
- `BallController` accepts `BallState` dataclass
- Debug timing prints gated by `settings.DEBUG_PRINTS`
- `ball_controller.py` moved from `cv/` to `control/`
- `comms/` retired; `gui_layout_legacy.py` and `gui_main.py` deleted

**Acceptance criteria met:**
- `BallState` dataclass used end-to-end (BallTracker → BallController) ✅
- No timing prints when `DEBUG_PRINTS = False` ✅
- Vision loop wires cleanly; ball balance behavior fix deferred to M7 ✅
- No regressions in manual control, routines, visualizer, serial monitor ✅

---

### Milestone 7 — Codex Audit + Integration
**Status:** Complete (2026-05-12, PR #10)

**What it does:** Ports all features and vision fixes from the Codex
`offset-tuning-and-camera-exposure` branch into the refactored codebase. Restores
ball balancing to full working state and adds all Codex-developed capabilities.

**Scope (22 feature groups — see `docs/codex_audit.md` for full detail):**
- `settings.py`: ~60 new runtime constants (camera, tracker, auto-trim, PD autotune, loop rates)
- `cv/ball_tracker.py`: background capture thread; camera backend probing + adaptive exposure; ArUco scale/cache/hold/filter improvements; adaptive HSV detection; position filter
- `control/ball_controller.py`: auto-trim (integral correction for table tilt); PD autotune (two-leg step test, kp/kd recommendation); slew limit + d-term cap; `compute_with_terms()` returning rich diagnostic dict
- **NEW** `cv/vision_control_worker.py`: `VisionControlWorker(QObject)` + `ControlSnapshot` dataclass; runs in `QThread`; backpressure mechanism; decoupled command_sender
- `gui/control_panel.py`: Target X/Y sliders; Roll/Pitch Trim sliders; Auto-Trim, Calibrate Home, Reset Trim, PD AutoTune buttons; HSV sliders in GUI; dark mode theme; Save Trim as Default button
- **NEW** `gui/vision_monitor.py`: separate monitor window with Camera, Warped, and Mask views with vector overlays
- `gui/main_window.py`: VisionControlWorker wiring; rolling 30s timing plot; bidirectional state sync; neutral-pose safety fallback on sustained ball loss; valid-streak reacquisition gating

**Out of scope:**
- Platform geometry constants in `config.py` — hardware-specific, do not change
- `routines/` — Codex version is identical to refactored version; no porting needed

**Acceptance criteria met:**
- Ball balancing functional end-to-end (camera → tracker → controller → serial → platform) ✅
- Auto-trim converges ball to target position under normal conditions ✅
- No Qt calls from background threads ✅
- All new constants sourced from `settings.py` — no inline literals in logic files ✅
- CI passes (pytest, flake8, mypy) ✅
- Manual smoke test: manual control, routines, vision mode all working ✅

**Test gate:** Unit tests for new `BallController` methods passed. Manual hardware smoke test passed (no missed ball captures, all modes working).

---

### 2026-07-22 Overhaul — Safety, IK, Vision, GUI (absorbs M9–M12)
**Status:** Implemented (branch `overhaul/safety-ik-vision-gui`); merge gated
on hardware smoke tests

**What it does:** Six-step hardening pass driven by the 2026-07-22 five-agent
full-repo review (`docs/code-review-2026-07-22.md` — the finding-by-finding
record and acceptance evidence). Absorbs milestones M9 (IK correctness), M10
(controller decomposition), M11 (vision split), and M12 (settings overlay +
GUI slimming), plus safety work the review surfaced that was never on the
roadmap.

**Serial protocol additions (host side — firmware unchanged):**
- The host now uses the firmware's `speedDelay` field: per-servo jumps larger
  than `SERVO_SLEW_INSTANT_MAX_DEG` are sent with
  `SERVO_LARGE_MOVE_SPEED_DELAY_MS` so the ramp happens in hardware
  (`core/safety.select_speed_delay`, applied on every send path including
  manual). Small streaming steps stay `speedDelay=0`.
- On connect, `SerialManager` poll-reads for the firmware's `[READY]` boot
  banner (3 s cap, with a fallback for firmware that never prints it)
  instead of an open-loop 2 s sleep.

**Safety rails:**
- Serial hardening: write lock (GUI + vision worker both send), read loop
  survives callback exceptions, read/write errors mark the link dead and
  fire a disconnect callback, double-connect guard, latest-wins depth-1
  writer thread for streaming setpoints.
- Global 0–180 clamp on every servo plus length/finiteness validation in
  `core/safety.py` (the min/max limit keys were previously dead).
- Raw-command box routed through `ServoDriver.send_raw` (parse, clip, ramp,
  guaranteed newline) instead of GUI→serial directly.
- Mode mutual exclusion: vision mode locks routines, SEND, and the raw box.
- Crash paths: `closeEvent` thread-shutdown handshake (no more 3 s hang);
  `sys.excepthook` logs, attempts ramped-neutral emergency shutdown, and
  disconnects instead of PyQt5's `qFatal()` abort with servos live.
- Vision: bounded stale-homography hold with cache invalidation; reacquire
  gating moved into the worker command path; neutral-pose fallback policy
  moved into the worker miss branch.

**IK behavior change:**
- Branch selection redesigned (seed by servo-angle validity, symmetric elbow
  handling, continuity from a single choice). Out-of-range angles are now
  **per-servo failures** (`IKResult.servo_status`) instead of silent clamps;
  failed servos carry neutral placeholders. Frozen `IKResult` dataclass
  replaces the untyped result dict for all consumers.
- Send-mode routines ease back to neutral on completion and cancel (approved
  behavior change — screw used to park at z=-12/yaw=-35). IK failures during
  playback are counted and reported in the GUI.

**Acceptance criteria:**
- 206 unit tests pass; flake8/mypy clean with empty setup.cfg exclude lists ✅
- Sweep-continuity regression across all routines (screw max step 2.7°,
  formerly a commanded 180° snap) ✅
- Manual hardware smoke test — pending (gates the PR)

**Known limitation:** Cone Tracing exits the workspace on 20 of 120 steps
(pre-existing, now reported honestly and pinned by test); envelope
adjustment is an open physical-design decision.

---

### 2026-07-22 Performance Pass — Latency, Jitter, Smoothness
**Status:** Implemented (branch `perf/latency-jitter`, stacked on the
overhaul); firmware v2 flashed and validated on the rig; merge gated on the
live tuning session

**What it does:** Six measured steps eliminating servo dither, sensor
jitter, and pipeline latency, with a hard requirement that smoothing never
degrades genuine fast response (all smoothing is speed/innovation-scheduled).

**Firmware v2 serial protocol (see `firmware/README.md` for full detail):**
- `T,d0,d1,d2,d3,d4,d5\n` — tenth-degree units 0..1800, instant
  `writeMicroseconds`, terse ack `k\n` (`e\n` on malformed input).
- Legacy `S,a0..a5,speedDelay\n` retained bit-compatible (whole degrees,
  firmware ramp, verbose ack). Position state shared between protocols.
- 250000 baud; boot banner `[READY v2]`. The host auto-detects the version
  from the banner and falls back to 115200 when none appears (v1 boards).
- Host hybrid dispatch: v2 + instant move → `T` (Schmitt hysteresis on the
  0.1° grid); large jumps and v1/forced-legacy → `S` + firmware ramp.

**Rest-mode contract (`control/rest_gate.py`):**
- Enter (via a 0.5 s sustained hold): radius ≤ 6 mm AND LPF speed ≤ 12 mm/s.
- Exit (checked first, same-cycle, raw values, no filter/timer):
  radius > 10 mm OR speed > 25 mm/s.
- Resting output = level + live trim offsets, slew-limited on transition;
  auto-trim keeps integrating during rest (its gates are strictly wider),
  so trim updates still walk the ball toward center.
- Exit latency is exactly one control cycle; verified equal-command vs a
  never-resting controller on a disturbance frame.

**Tracker measurement filtering (`cv/measurement_filter.py`):**
- Modes: `alpha_beta` (default; constant-velocity filter, gains scheduled
  from quiet 0.30/0.05 to fast 0.90/0.50 by max(innovation 1→4 mm, speed
  60→150 mm/s)), `legacy` (pre-pass EMA behavior, regression-pinned),
  `raw` (bench comparison).
- ArUco: every-frame detection, sub-pixel corner refinement (detector-level
  + full-res `cornerSubPix`), marker-center deadband 0.3 px / fast-track
  1.5 px per frame. Sub-pixel ball centroid.

**Acceptance criteria:**
- 294 unit tests pass; flake8/mypy clean ✅
- Bench (through the real filter→PD→IK→servo chain): quiescent servo
  activity 6125 → 0 integer flips/min with rest engaged (one send per
  30 s); impulse response ≥ 85% of unsmoothed command within one cycle
  (measured 100%) ✅
- Serial RTT p50: 57 ms → 9.7 ms (host read fix, v1) → 4.4 ms (v2) ✅
- Camera: 60 fps experiment closed — hardware capped at ~31 fps; 30 fps
  retained ✅
- Live ball-balancing tuning session — pending (gates the PR)

---

## Future Features (not scheduled)

### Multi-Camera Ball Tracking
- Second camera for orthogonal view, 3D position reconstruction
- BallState.z_mm populated
- BallTracker remains instantiable; multi-tracker via composition

### Ball Catching
- Trajectory prediction from 3D ball state
- Platform pre-positioning based on predicted landing
- Requires sub-20ms control loop — evaluate hardware limits first

### Ball Bouncing
- Timed platform impulse for vertical oscillation
- Requires z_mm tracking (second camera)
- Centering during bounce = PD in x/y + z impulse timing

### Multiple Ball Targets
- Multiple blob tracking, target assignment
- BallTracker and BallController stay instantiable for multi-instance use

---

## Constraints and Decisions

- Arduino serial at 115200 baud, command format: `S,a0,a1,a2,a3,a4,a5,0\n`
- 6 servos, 0-indexed, angles in integer degrees 0-180
- Safety limits: servos 0,2,4 max 170°; servos 1,3,5 min 10°
- Control loop target: 50Hz (20ms timer)
- IK solve time: <1ms typical, acceptable per frame
- Platform geometry in config.py is physical measurement — do not change without
  re-measuring hardware
- `__pycache__` and `.pyc` files should be in `.gitignore`
