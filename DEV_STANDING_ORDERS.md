# Dev Standing Orders

> Read this entire document before touching any file.
> These rules apply to every milestone, every session, without exception.
> The milestone brief you were given contains only the objective and steps.
> Everything else — architecture, git rules, constraints — lives here.

---

## Architecture

```
Project: stewart_control
Purpose: GUI-driven Stewart platform controller with IK solving, serial comms to Arduino,
         demo routines, and closed-loop ball balancing via computer vision.
Stack: Python 3.11+, PyQt5, matplotlib, numpy, pyserial, opencv-contrib-python

Directory structure (current state on main):
├── main.py                    # Entry point only. Legacy — do not modify.
├── config.py                  # Geometry constants only. Do not add runtime config here.
├── settings.py                # Runtime config: port, baud, intervals, safety limits.
├── conftest.py                # Pytest path shim. Do not modify.
├── setup.cfg                  # flake8 + mypy config. Exclude list shrinks each milestone.
│
├── core/
│   ├── platform_state.py      # Dataclasses: Pose, ServoAngles, BallState. No logic. ✅
│   ├── ik_engine.py           # Single IK call site. Nothing else calls ik_solver directly. ✅
│   └── safety.py              # All servo clipping and validation. One place. ✅
│
├── hardware/
│   ├── serial_manager.py      # Connection lifecycle, read loop, callbacks. ✅
│   └── servo_driver.py        # Formats commands, calls serial_manager. ✅
│
├── control/
│   ├── routine_runner.py      # Routine playback state machine. No Qt dependencies. ✅
│   └── ball_controller.py     # PD controller. Pure math. No Qt dependencies.
│
├── cv/
│   └── ball_tracker.py        # Returns BallState. Camera-index configurable.
│
├── routines/
│   └── routines.py            # Pure pose-list generators. No dependencies.
│
├── visualization/
│   └── visualizer3d.py        # Draws pre-solved geometry. Does NOT call IK. ✅
│
├── gui/
│   ├── gui_layout.py          # Legacy monolith — RETIRED in M5. Do not add to it.
│   ├── main_window.py         # Top-level QWidget. Wires all modules. (M5 target)
│   ├── control_panel.py       # Sliders, buttons, routine selector. (M5 target)
│   └── serial_monitor.py      # Serial output display widget. (M5 target)
│
└── tests/
    ├── test_safety.py         # ✅
    ├── test_servo_driver.py   # ✅
    ├── test_ik_engine.py      # ✅
    └── test_routine_runner.py # ✅
```

**Module responsibilities — what each owns / does NOT own:**
- `config.py` — physical geometry only. Does NOT own runtime settings.
- `settings.py` — port, baud, intervals, limits. Does NOT own geometry.
- `core/platform_state.py` — data shapes only. No logic, no project imports.
- `core/ik_engine.py` — IK call site only. Does NOT own visualization or serial.
- `core/safety.py` — clipping and validation only. No Qt, no serial.
- `hardware/serial_manager.py` — connection and read loop. Does NOT format commands.
- `hardware/servo_driver.py` — command formatting and dispatch. Does NOT own connection state.
- `control/routine_runner.py` — playback state machine. No Qt imports. Accepts tick() from GUI.
- `control/ball_controller.py` — PD math only.
- `cv/ball_tracker.py` — vision pipeline. Returns BallState.
- `visualization/visualizer3d.py` — drawing only. Accepts pre-solved geometry. Does NOT call IK.
- `gui/main_window.py` — top-level window, wires all modules together. View and wiring only.
- `gui/control_panel.py` — slider and button widgets. No logic.
- `gui/serial_monitor.py` — serial display widget. No logic.

**Shared data contracts:**
- `Pose`: x, y, z, roll, pitch, yaw — all float, mm and degrees
- `ServoAngles`: list[float], 6 elements, degrees, index 0-5
- `BallState`: x_mm, y_mm, vx_mm_s, vy_mm_s (required); z_mm, vz_mm_s (Optional, None if 2D)
- IK result dict: {success, platform_points, arm_points, servo_angles_deg, platform_center, platform_R, debug}
- Routine pose dict: {x, y, z, roll, pitch, yaw} — all float, passed to on_pose_update callback

**Naming conventions:**
- files: snake_case — classes: PascalCase — functions/methods: snake_case
- constants: UPPER_SNAKE_CASE (config.py and settings.py only)
- dataclass fields: snake_case

---

## Hard Constraints

These are non-negotiable. Violating any of these is grounds for the PM to reject and revert.

1. Do not modify `config.py` geometry values without PM approval.
2. Do not add runtime config to `config.py` — use `settings.py`.
3. Do not call `ik_solver.solve_pose()` directly — always go through `core/ik_engine.py`.
4. Do not build `"S,..."` serial command strings inline — always use `hardware/servo_driver.py`.
5. Do not put control logic or IK calls in any `gui/` file.
6. Do not add hardcoded ports, intervals, or limits in logic files — use `settings.py`
7. Import paths are repo-root-relative: `from settings import ...`, `from core.safety import ...`
   Never use `stewart_control.*` prefixes.
8. Hardware tests that require a physical Arduino must be marked `[HARDWARE]` and skipped in CI
   with `pytest.mark.skip(reason="requires hardware")`.
9. Qt timer callbacks must stay fast (<5ms). Offload blocking work to threads.
10. `visualization/visualizer3d.py` must NOT call IK directly. It uses IKEngine as a fallback only.
11. `control/routine_runner.py` must have NO Qt imports. It accepts a tick() call from the GUI timer.
12. `gui/main_window.py` owns the QTimer and calls `routine_runner.tick()`. It does NOT contain
    routine state logic, IK calls, or serial command building.

---

## Git Rules

1. Before touching any file, run `git status` and confirm the tree is clean and you are on `main`.
   If the tree is dirty or you are on a non-main branch, stop and report. Do not proceed.
2. Create ONE fresh branch for the entire milestone:
   `git checkout -b milestone/[N]-[short-name]`
   Never reuse an existing branch. Never continue a previous milestone's branch.
3. Commit after each numbered step in the milestone plan:
   `[MN] Step X: description of what was done`
   Messages must be specific. Never use "update", "fix", "changes", or similar.
4. Do not commit broken or untested code.
5. When all steps are complete and all tests pass, open ONE pull request to main:
   - Title: `[MN] Milestone name`
   - Body: list each step completed, files created/modified, test results, any deviations
6. One PR per milestone. Never open multiple PRs for a single milestone.
7. **Review comment replies are mandatory.** When the PM or any reviewer posts a comment on the
   PR, you MUST reply to that specific comment thread before pushing the fix. The reply format is:
   ```
   Acknowledged. Fixing in next commit.
   ```
   Then after pushing, reply again to the same thread:
   ```
   Fixed in <full-commit-hash> — <one line description of exactly what changed>
   ```
   Both replies are required. A silent push with no thread reply is a process violation.
   Do not post a single general "all fixed" comment — reply to each thread individually.

---

## Dev Rules

1. **Follow the plan exactly.** You have an approved milestone plan. Implement it as written.
2. **One step at a time.** Complete and verify each step before starting the next.
3. **Tests before committing.** Each step must pass its test criterion before you commit.
4. **Stay in scope.** Do not modify files, refactor code, or add features outside the plan —
   even if you see something that should be fixed. Log it and move on.
5. **No silent assumptions.** If anything is ambiguous, stop and ask before proceeding.
6. **No duplication.** Before creating a new utility or module, check if one already exists.
7. **Flag early.** If the plan won't work as written, stop and report it immediately.
   Do not work around it silently.
8. **Data contracts first.** If a step passes data between modules, verify the dataclass
   schema in `core/platform_state.py` exists and is approved before writing code that uses it.
9. **Review replies are mandatory.** See Git Rule 7. This is not optional.

---

## CI

Every push and PR triggers `.github/workflows/ci.yml` which runs:
- **pytest** — all tests in `tests/`
- **flake8** — whole repo, legacy modules excluded via `setup.cfg` exclude list
- **mypy** — whole repo, legacy modules excluded via `setup.cfg` exclude regex
- **types-pyserial** is installed as a dev dep — no `# type: ignore[import]` needed for serial

When a legacy module is refactored in a milestone, remove it from the exclude lists in `setup.cfg`.
Do not open a PR if CI is failing.

---

## Completed Milestones

- **M1 — Foundation**: `core/platform_state.py`, `settings.py`, `core/safety.py`, `tests/test_safety.py`
- **M2 — Hardware Layer**: `hardware/serial_manager.py`, `hardware/servo_driver.py`, `tests/test_servo_driver.py`
  - CI flipped to exclude-list pattern per reviewer feedback
  - CHANGELOG reformatted to keepachangelog standard
- **M3 — IK Consolidation**: `core/ik_engine.py`, `tests/test_ik_engine.py`
  - `visualization/visualizer3d.py` refactored to accept pre-solved geometry, uses IKEngine as fallback
  - `visualization/` removed from CI exclude lists
- **M4 — Routine Runner Extraction**: `control/routine_runner.py`, `tests/test_routine_runner.py`
  - `gui/gui_layout.py` routine state machine replaced with RoutineRunner calls
  - `_LegacySerialAdapter` bridges SerialSender to ServoDriver for M4; will be removed in M5
  - `self.preview_mode`, `self.current_routine_steps`, `self.current_routine_index` removed from GUI
