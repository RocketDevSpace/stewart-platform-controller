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

### 2026-07-22 Path Following — Ball Traces Virtual Patterns
**Status:** Implemented (branch `feat/path-following`, stacked on the
performance pass); merge gated on the rig hardware session

**What it does:** The ball follows virtual patterns — circle, square,
heart, star, or arbitrary point lists — rendered as overlays in the
vision monitor. The patterns are conceptual: no physical lines on the
platform, no camera line detection.

**Pacing contract (`control/path_follower.py`):**
- Adaptive with a max speed: each cycle the moving target advances by
  `speed · factor · dt` where `factor` tapers linearly from 1 at
  ball-to-target error ≤ `PATH_FULL_SPEED_RADIUS_MM` (10 mm) to 0 at
  ≥ `PATH_CAPTURE_RADIUS_MM` (20 mm). The taper law is continuous —
  "stalled" is telemetry-only and auto-resumes. Consequence (pinned by
  the `tools/path_sim.py` closed-loop feasibility tests): any commanded
  speed degrades to slower laps, never to losing the ball.
- `start()` only arms; the first valid ball sighting seeds the arc
  position at the nearest path point and never re-seeds (no lobe
  jumping on self-approaching shapes; nothing moves until the ball is
  seen).
- Closed paths wrap and count laps; open paths finish `done` and hold
  the endpoint. Speed is live-adjustable 10–80 mm/s (default 30 —
  ≈ 2/3 of the analytic ~35 mm/s stall ceiling at default gains).

**Controller integration contract (`control/ball_controller.py`):**
- The follower drives the SetpointArbiter **override channel**
  (autotune's channel) from inside `compute_with_terms` — never
  streaming `set_target`, whose side effects would reset the rest gate
  and freeze auto-trim every cycle.
- Mutual exclusion both directions: starting a path cancels home
  calibration and autotune; enabling either stops the path. The GUI
  mirrors the same exclusions (belt and braces).
- `stop_path()` is motion-free: the active target freezes in place
  bit-identically.
- Standing offsets along the path (the plate's position-dependent tilt
  field) are cancelled by the PDCore integral — see the 2026-07-23
  I-term rework section below for the contract. Rest mode is
  force-suppressed while following and re-engages when an open path
  completes.
- Telemetry: 8 additive `path_*` keys in the control terms
  (active/state/name/progress/lap/s_mm/error_mm/speed_mm_s).

**Patterns (`control/patterns.py`):** frozen `Path` dataclass, uniform
2 mm arc-length resampling, 85 mm radial clamp (inside the 84.85 mm
marker-corner radius). Registry labels: Circle (r=65), Square (diamond,
d=140 — rotated 45° so corners avoid marker dwell), Heart (~65 mm),
Star (5pt, r=70 — inner corners are the stress test), plus
`from_points` for arbitrary shapes.

**Acceptance criteria:**
- Unit/sim suite green (351 passed); flake8/mypy clean ✅
- Sim safety property: max tracking error saturates ~16 mm < 20 mm
  capture radius at any commanded speed (10/30/60 mm/s swept) ✅
- Rig session (gates the PR): circle at 30 mm/s — full laps, no path
  loss, overlay tracks; square corners; star inner corners visibly
  slow-and-recover; Stop on an open path = no jump; target drag
  cancels; autotune/path mutual exclusion via GUI — pending

---

### 2026-07-23 I-Term Rework — True PID, Trim Demoted to a Store
**Status:** Implemented (branch `rework/integral-trim`, stacked on path
following); merge gated on the rig session

**Why:** the first path-following rig session traced four field
failures (home-cal stalls, rest rocking, the path deadlock, a measured
1.2 Hz burst limit cycle) to the gated AutoTrim integrator — the wrong
signal (gated position error) in the wrong structure (bang-bang bursts)
in the wrong data structure (one global scalar for a position-dependent
field: rig-measured ~0.36° more compensation needed at r=65 than
center).

**Integral contract (`control/pid_core.py`):**
- Continuous, no gates; integrates position error in x/y error space,
  mapped to pitch/roll beside P and D. ki = 0.030 deg/(mm·s)
  (τ_I = kp/ki = 1.5 s; a 0.4° bias is inside 2 mm and staying there
  in ≤ 5 s — friction bounds standing accuracy to ~1.3 mm at kp 0.045).
- Protections: error-taper (full ≤ 25 mm, zero ≥ 60 mm — flicks never
  integrate), per-axis directional anti-windup at the tilt clamp
  (un-integrating out of saturation always allowed), 25 s leak
  (steady-state cost ~0.5 mm; mis-learned corrections self-heal),
  clamp ±1.5° (±6° during home-cal), freeze-in-place (no zeroing) for
  autotune sessions / feature-off / rest.
- `reset_motion_state()` (tracking loss) KEEPS the integral — learned
  plate knowledge; staleness is the leak's job. `reset_trim` clears it.
- Known behaviors: unreachable-target windup parks at the clamp and
  self-heals on the leak; a standing trim+I near the 10° tilt clamp
  asymmetrically reduces flick headroom in that direction (physics).

**Rest interlocks (both sim-caught):**
- Rest ENTRY requires the integral flat (`i_rate_deg_s` EMA under
  `REST_I_RATE_MAX_DEG_S` = 0.02): entering on a converging integral
  is not an equilibrium (P and D are dropped while resting) and
  limit-cycles at ~0.2 Hz. Entry-only — exit stays radius/speed-based.
- The integral FREEZES while resting: an integrator acting on an
  undamped parked ball is structurally unstable; freezing also pauses
  the leak so a long rest cannot drain the learned correction.
- Endgame: converge → rest → integral frozen at the learned bias →
  total servo silence.

**Trim contract (`control/trim_store.py`):** a pure store — manual
slider writes, reset to settings defaults, and `fold()` (absorb the
integral into persistent trim, clamped ±8°, zero net output change).
Single-writer preserved: TrimStore owns the offsets, PDCore owns the
integral, `BallController.fold_integrator_into_trim()` is the only
bridge.

**Home calibration:** auto-completes. Converged = integral moved
< 0.05°/axis over a trailing 2 s window AND ball < 20 mm/s AND ball
< 15 mm from center (the radius gate refuses to fold an integral that
saturated flat against an unreachable ball) → fold → auto-save (GUI
persists the folded values carried in the transient `home_cal_event`)
→ auto-complete. Timeout 30 s → cancel, discard the windup, save
nothing. Save Trim routes through the same fold path — one save code
path.

**Autotune:** the integral freeze is derived from `autotuner.enabled`
(no flag stash/restore); a frozen integral is a constant bias, which
the leg evaluator's second-order-step inversion tolerates exactly like
trim.

**Acceptance criteria:**
- Suite green (383 passed) incl.: deadlock reproduction over the
  measured warp field with the integral off (0 laps), recovery with it
  on (laps at ~99% of commanded speed, max_err 13 mm), ki sweep, rest
  stability, home-cal convergence/fake-flatness/timeout ✅
- jitter_bench A/B: quiescent unchanged (2 sends, 4 flips/min), +5 mm
  standing bias converges in a ~13-send burst then silence, impulse
  response untouched (taper) ✅
- Rig session (gates the PR): center hold with injected 0.4° trim
  error recenters in 2–4 s with no 0.77 Hz growth; the 206 s
  stationary run repeats with ~0 movement bursts (was 200); rest goes
  send-silent after convergence; 300 mm/s flick recovery at baseline
  with |i_term| excursion < 0.2°; r=65 target equilibrium within
  ~2 mm (was 8 short); circle laps + star corners over the real plate;
  home-cal from a deliberately wrong ±2° trim converges, auto-saves,
  survives restart; autotune session sane; 2 s camera blackout
  mid-path reacquires with no trim transient — pending

---

### 2026-07-23 SysID AutoTune — Measure the Plant, Design the Gains
**Status:** Implemented (branch `rework/autotune-id`, stacked on the
I-term rework); merge gated on the rig session

**Why:** the step-test estimator completed zero legs on the rig (its
settle gates never opened against the wobble floor) and random-walked
against a known simulated plant. Its 2-feature model inversion cannot
survive latency + stiction + warp + the ball's self-rock, and it never
tuned ki.

**Pipeline contract (same GUI button flow):**
1. PROBE (~78 s, `control/plant_id.ProbeScript`, NO settle gates):
   scripted targets through the arbiter override — quiet hold (noise +
   self-rock measurement), safety pre-check, relay toggles ±25 mm x/y,
   diagonals, stiction micro-steps, baseline steps. Per-frame recording
   of raw detections + actually-sent commands. Aborts: ball lost > 1 s,
   out of bounds (> 70 mm, recenter then 2 s grace), user toggle.
2. FIT (background thread, < 5 s): convex per-latency acceleration
   regression recovers (g_eff, pipeline latency, effective stiction,
   warp, biases); filter group delay measured separately
   (raw↔filtered cross-correlation); self-rock band notched from both
   regression sides; confidence gate (low_confidence caps everything
   downstream).
3. DESIGN (background thread, ~15-30 s): kp/ki/kd search on the fitted
   plant, CRN-paired sims of the real chain, J normalized to the
   current gains (J = 1.0 = today); the current gains are always in
   the candidate pool — a suggestion can NEVER be measured-worse than
   today.
4. APPLY: lands kp/kd/ki + calibrates the prediction horizon (fitted
   latency + filter lag) and the integral deadband
   (0.5·stiction/kp_new, clamped [1, 6] mm); the GUI persists gains +
   calibrations + g_eff to the settings overlay in one save
   (transient `pd_autotune_event {"type": "applied"}`, the
   home_cal_event pattern). Auto-Apply = apply when design completes.

**Acceptance criteria:**
- Killer tests green (418 passed): known-plant recovery through a full
  closed-loop probe (g ±10-15%, latency ±20-25 ms, with and without an
  injected 0.8 Hz rock), never-worse + ≥30%-from-bad-start design,
  deterministic, end-to-end session through compute_with_terms ✅
- Rig session (gates the PR): AutoTune with vision on → probe ~80 s
  with visible phase/progress → suggestion with plant numbers (sanity:
  latency 0.06-0.12 s, g_eff 120-220) → Apply updates all three
  sliders + user_settings.json gains + calibrations; timing strip
  stays flat during compute; cancel mid-probe returns to normal
  balancing; re-run after the ball swap re-identifies — pending
- Deferred: post-apply validation re-probe (before/after measured
  cost) — follow-up.

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
