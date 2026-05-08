# Stewart Platform Controller

PyQt5 desktop app controlling a 6-DOF Stewart platform via Arduino serial. Includes IK solving, 3D visualization, scripted motion routines, and closed-loop ball balancing via OpenCV. Solo project. Hudson is the only contributor.

**Stack:** Python 3.11+, PyQt5, matplotlib, numpy, pyserial, opencv-contrib-python.

## How to work in this codebase

**Run:** `python main.py` from repo root. Arduino must be connected on the port set in `settings.py` (default `COM4`).
**Test:** `pytest` from repo root. CI runs pytest, flake8, mypy on every push.
**Lint/typecheck:** `flake8` and `mypy` per `setup.cfg`. Legacy modules excluded; exclusions shrink as refactor advances.
**Imports:** repo-root-relative only. `from settings import ...`, `from core.safety import ...`. Never use `stewart_control.*` prefixes — that's an artifact of the original ChatGPT Codex codebase and is being removed.

## Current architecture (in flight — see "Refactor state" below)

```
main.py                # entry point — currently has stewart_control.* imports (M5 fix)
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
  ball_controller.py   # PD controller. ⚠ STILL IN cv/ — move pending in M6.

cv/
  ball_tracker.py      # vision pipeline. Returns BallState (pending M6 dataclass refactor).
  ball_controller.py   # ⚠ should be in control/ — pending M6.

routines/              # pure pose-list generators.
visualization/
  visualizer3d.py      # drawing only. Accepts pre-solved geometry. ✅
gui/
  gui_layout.py        # legacy 25KB monolith. RETIRING in M5. Do not add to it.
  gui_main.py          # ⚠ orphaned, not in any architecture doc — likely dead, kill in M5.
tests/                 # test_safety, test_servo_driver, test_ik_engine, test_routine_runner.
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

## Refactor state

A 6-milestone refactor is in flight, extracting logic from `gui_layout.py` into clean modules. **M1–M4 complete. M5 and M6 pending.** Read `SPEC.md` for milestone scope and acceptance criteria. Read `CHANGELOG.md` for what each completed milestone shipped.

**Known scope gaps** to fold into the remaining milestones:
- M5 should also fix `main.py`'s `stewart_control.*` imports and remove orphaned `gui/gui_main.py`.
- M6 should also move `ball_controller.py` from `cv/` to `control/` and retire any remaining usage of the `comms/` folder.

These weren't in the original SPEC and should be added when M5/M6 plans are finalized.

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

- *No entries yet — Stewart project just transitioning to the v5 / bootstrap-v2 doc system. Will accumulate as we work.*

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
