# Stewart Platform Controller

PyQt5 desktop app controlling a 6-DOF Stewart platform via Arduino serial. Includes IK solving, 3D visualization, scripted motion routines, and closed-loop ball balancing via OpenCV. Solo project. Hudson is the only contributor.

**Stack:** Python 3.11+, PyQt5, matplotlib, numpy, pyserial, opencv-contrib-python.

## How to work in this codebase

**Run:** `python main.py` from repo root. Arduino must be connected on the port set in `settings.py` (default `COM4`).
**Test:** `pytest` from repo root. CI runs pytest, flake8, mypy on every push.
**Lint/typecheck:** `flake8` and `mypy` per `setup.cfg`. Legacy modules excluded; exclusions shrink as refactor advances.
**Imports:** repo-root-relative only. `from settings import ...`, `from core.safety import ...`. Never use `stewart_control.*` prefixes — that's an artifact of the original ChatGPT Codex codebase and is being removed.

## Current architecture

```
main.py                # entry point — repo-root-relative imports. ✅
config.py              # physical geometry constants only. NOT runtime config.
settings.py            # runtime config: port, baud, intervals, safety limits.
conftest.py            # pytest path shim.
setup.cfg              # flake8 + mypy config. Exclude list shrinks each milestone.

core/
  platform_state.py    # dataclasses: Pose, ServoAngles, BallState. No logic. ✅
  ik_engine.py         # single IK call site. ✅
  safety.py            # servo clipping + validation. ✅

hardware/
  serial_manager.py    # connection lifecycle, read loop, callbacks. ✅
  servo_driver.py      # command formatting and dispatch. ✅

control/
  routine_runner.py    # routine playback state machine. No Qt deps. ✅
  ball_controller.py   # PD controller. ✅

cv/
  ball_tracker.py      # vision pipeline: capture thread + ArUco + detection. Returns BallState. ✅
  vision_control_worker.py  # owns BallTracker + BallController; vision/PD/IK loop in a QThread. ✅

routines/              # pure pose-list generators.
visualization/
  visualizer3d.py      # drawing only. Accepts pre-solved geometry. ✅
gui/
  main_window.py       # top-level window. Wires all modules. ✅
  control_panel.py     # sliders, buttons, signals. ✅
  serial_monitor.py    # serial output display widget. ✅
  vision_monitor.py    # floating camera/warped/mask debug views. ✅
tests/                 # test_safety, test_servo_driver, test_ik_engine, test_routine_runner, test_ball_controller, test_vision_control_worker.
```

**Module ownership boundaries:**
- `config.py` owns physical geometry. NOT runtime settings.
- `settings.py` owns runtime config. NOT geometry.
- `core/ik_engine.py` is the ONLY caller of `kinematics/ik_solver.solve_pose()`.
- `hardware/servo_driver.py` is the ONLY place `"S,..."` command strings are built.
- `gui/*` files contain view and wiring only. NEVER control logic, IK calls, or serial command building.
- `control/routine_runner.py` has NO Qt imports. It accepts `tick()` calls from a GUI-owned QTimer.
- `visualization/visualizer3d.py` does NOT call IK directly. Accepts pre-solved geometry; uses `IKEngine` only as a fallback.

**Shared data contracts** (defined in `core/platform_state.py`):
- `Pose`: x, y, z, roll, pitch, yaw — all float, mm and degrees.
- `ServoAngles`: list[float], 6 elements, degrees, index 0–5.
- `BallState`: x_mm, y_mm, vx_mm_s, vy_mm_s required; z_mm, vz_mm_s Optional (None if 2D-only).
- IK result dict: `{success, platform_points, arm_points, servo_angles_deg, platform_center, platform_R, debug}`.

## Project state

**The original 7-milestone refactor is complete — M1–M7 all merged** (M7 merged 2026-05-12; autotune guard follow-ups via PR #13; tracker velocity low-pass via PR #14 and servo 4 geometry fix via PR #15, both merged 2026-06-11). Read `CHANGELOG.md` for what each milestone shipped. Always check the open PR list (`gh pr list`) before treating any status written here as current — implementation may be in review.

**Current phase: post-refactor hardening (M8–M12), scoped 2026-06-11.** Full detail and decisions in PROJECT_STATE.md.

- **M8 — Housekeeping:** untrack `__pycache__` artifacts, requirements files, sync docs to reality, fix stale `config.py` comments.
- **M9 — IK correctness + safety rail:** fix the IK branch-flip bug in `kinematics/ik_solver.py` — the "closest to neutral" servo-angle pick silently overrides the continuity-based elbow choice (root cause of the screw-routine servo snap at yaw=-35° logged at M5). Add a slew-rate guard in `core/safety.py` applied in `ServoDriver.send_angles`; manual sends ramp through it too (decided 2026-06-11). Consolidate the three duplicated inline 0–180 clamps. Move reacquire gating into the worker command path. Routine-sweep continuity regression tests.
- **M10 — Controller decomposition:** split autotune + auto-trim out of the 800-line `control/ball_controller.py`; collapse `compute()` into `compute_with_terms()` (tests currently cover the unused `compute()` path; production uses `compute_with_terms()`).
- **M11 — Vision split:** extract camera lifecycle from `BallTracker`; public tracker API (the worker currently reaches into tracker privates); synthetic-frame tests; remove `cv/ball_tracker.py` from setup.cfg excludes; remove dead velocity-smoother state left by PR #14.
- **M12 — GUI slimming:** replace the `settings.py` regex rewrite (save-trim-as-default) with a user-settings overlay; extract timing plot + vision bridge from `gui/main_window.py`; non-blocking serial connect (currently freezes the GUI ~2 s at startup).

## What good looks like in this domain

Hudson is still building reps in software architecture, so this section exists as scaffolding — recognize when something matches or violates these patterns.

- **Layer separation is non-negotiable.** GUI talks to control. Control talks to hardware. Hardware talks to serial. A GUI file building a serial command string is a layer violation, even if it works. Reject it.
- **Single call sites for shared concerns.** IK lives in one place. Serial command formatting lives in one place. Safety clipping lives in one place. If you find yourself writing the same logic in two places, stop and consolidate.
- **Data contracts before code that uses them.** If a step passes data between modules, the dataclass schema in `core/platform_state.py` must exist and be approved before code that uses it.
- **Pure logic stays pure.** `control/routine_runner.py`, `control/ball_controller.py`, and `core/*` files have no Qt or hardware dependencies. This is what makes them testable.
- **Dead code is worse than ugly code.** When refactoring, fully remove what's superseded — don't leave the old version behind "just in case."
- **No hardcoded ports, intervals, or limits.** Everything goes in `settings.py`. A literal port string in a logic file is a bug.

## Hard constraints

These are non-negotiable. Violating any is grounds to reject and revert.

1. Do not modify `config.py` geometry without explicit approval — those are physical measurements.
2. Do not add runtime config to `config.py`. Use `settings.py`.
3. Do not call `ik_solver.solve_pose()` directly. Use `core/ik_engine.py`.
4. Do not build `"S,..."` command strings inline. Use `hardware/servo_driver.py`.
5. Do not put control logic, IK calls, or serial command building in any `gui/` file.
6. Do not use `stewart_control.*` import prefixes. Repo-root-relative only.
7. Hardware tests requiring a physical Arduino must be marked `[HARDWARE]` and skipped in CI via `pytest.mark.skip(reason="requires hardware")`.
8. Qt timer callbacks must stay fast (<5ms). Offload blocking work to threads.
9. `visualization/visualizer3d.py` must NOT call IK directly. IKEngine is fallback only.
10. `control/routine_runner.py` must have NO Qt imports.

## Git rules

1. Before touching any file, run `git status`. If the tree is dirty or you're on a non-main branch, stop and report.
2. Create ONE fresh branch per milestone: `git checkout -b milestone/[N]-[short-name]`. Never reuse a branch.
3. Commit after each numbered step: `[MN] Step X: description`. Be specific. Never use "update" / "fix" / "changes".
4. Do not commit broken or untested code.
5. Open ONE PR to main when the full milestone is complete. Title: `[MN] Milestone name`. Body lists each step, files changed, test results, deviations.
6. **Reply to review comments individually.** Each review thread gets a reply: `Acknowledged. Fixing in next commit.` then after pushing: `Fixed in <commit-hash> — <one-line description>`. A silent push with no thread reply is a process violation.

## Dev rules

1. **Plan before implementing.** If you don't have an approved plan, produce one and wait for approval. Don't code to figure it out.
2. **One step at a time.** Complete and verify each step before starting the next.
3. **Tests before committing.** Each step passes its test criterion before commit.
4. **Stay in scope.** Don't modify files, refactor code, or add features outside the plan — even if you see something that should be fixed. Log it for later.
5. **No silent assumptions.** Ambiguous = stop and ask.
6. **No duplication.** Check whether a utility already exists before creating a new one.
7. **Flag early.** If the plan won't work as written, stop and report. Don't work around silently.
8. **Verify before asserting authority.** Before stating that a function/file/feature doesn't exist, search thoroughly across multiple plausible locations and state where you searched. Don't pattern-match from training and present as authoritative.
9. **Trust the user's current description.** When a procedure or design has been updated, treat the current description as authoritative. Don't argue from earlier versions of the plan.

## Things Claude has gotten wrong on this project

*Running log. Append when corrections happen. Format: trigger / failure / corrected pattern. Scan at the start of each session.*

- **Entry 1 (2026-05-08):** *Trigger: a fresh Claude session reads PROJECT_STATE.md's 'M5: not started, next up' and treats it as authoritative without checking open PRs. Failure: produced a full M5 plan from scratch when PR #5 had been open for two weeks containing real M5 work. Corrected pattern: at session start, after reading PROJECT_STATE / CLAUDE.md, run `gh pr list` (or the GitHub MCP equivalent) and read open PR titles before treating any 'next up' / 'pending' status as the ground truth.*

## Verification expectations

- After any code edit, run `pytest` (unit tests) and `flake8` / `mypy` (linting/types).
- Hardware-dependent code requires manual smoke test with Arduino connected before merging the milestone.
- "Done" means: tests pass, CI passes, manual hardware test passed if applicable, PR opened with all required content.

## Out of scope

- Mac / Linux support. Windows-only project.
- Cloud / web / remote-control features. Local desktop app only.
- Any change to `config.py` geometry without re-measuring hardware first.

## File reference

| File | Purpose |
|---|---|
| `CLAUDE.md` | This file. Architecture, rules, constraints. Read at session start. |
| `PROJECT_CONTEXT.md` | Project's deeper mission, collaboration patterns, future direction. |
| `PROJECT_STATE.md` | Current phase, decisions log, open questions, phase roadmap. Living. |
| `SPEC.md` | Feature specs and milestone acceptance criteria. |
| `CHANGELOG.md` | Per-milestone shipped changes. Keep-a-changelog format. |
