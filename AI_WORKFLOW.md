# AI Development Workflow

> This document defines the process, rules, and role boundaries for AI-assisted development on this project. It applies to all AI tools used: Claude (chat/PM), Claude Code (implementation), Copilot, Cursor, or any equivalent. Read this before starting any AI-assisted work session.

---

## Philosophy

AI coding tools are fast, capable, and context-blind. Left unguided, they optimize locally — writing excellent code for the goal they were given while ignoring architectural consistency, test coverage, and scope. The human's job is not to write code. It is to own the decision surface: architecture, scope, sequencing, and review.

The AI writes. The human governs.

---

## Role Definitions

### PM (Project Manager) — Claude Chat / Project
Owns the spec, architecture, and process. Is consulted before implementation begins. Does not write production code. Is the source of truth for decisions.

**Responsibilities:**
- Maintain and update this document and `SPEC.md`
- Approve or reject implementation plans before work begins
- Break features into bounded, testable milestones
- Flag scope creep, architectural drift, and contradictions
- Conduct or direct post-milestone reviews
- Update the Architecture section after every completed milestone

**How to invoke:** Open the Claude Project for this repo. Ask for a plan. Do not ask it to write code.

### Dev (Developer) — Claude Code
Executes approved implementation plans only. Operates within the boundaries defined by the PM and this document. Does not make architectural decisions unilaterally.

**Responsibilities:**
- Implement exactly what the approved plan specifies
- Follow conventions in this document and `SPEC.md`
- Write tests before marking work complete
- Flag blockers or ambiguities to the PM before proceeding, not after
- One PR per milestone, not one PR per step

**How to invoke:** Paste the milestone brief from the PM into Claude Code. Include the Architecture section as context. Do not start a new milestone until the previous one is merged.

### Human (Engineering Lead) — You
Owns the project. Approves plans, reviews output, makes final calls.

**Responsibilities:**
- Review every plan before approving implementation
- Merge PRs after PM review comment gives the green light
- Update `SPEC.md` when requirements change
- Be the context bridge when PM and Dev are separate instances

---

## Workflow: Phase Gates

Every non-trivial feature or change follows this sequence. Do not skip phases.

```
DEFINE → PLAN → APPROVE → IMPLEMENT → TEST → REVIEW → MERGE
```

### Phase 1: Define
Write a clear statement of what needs to be built. Include:
- What the feature does
- What it does NOT do (explicit scope boundary)
- Acceptance criteria

File this in `SPEC.md`. If you can't write this, you're not ready to build.

### Phase 2: Plan
Send the spec to the PM. Ask for an implementation plan. Do not ask it to write code.

### Phase 3: Approve
Explicitly tell the PM "approved" or request changes. Once approved, copy the final plan. This becomes the Dev's task list.

### Phase 4: Implement
Paste the milestone brief (see Dev Prompt Template below) into Claude Code. The Dev implements all steps, commits after each, and opens ONE PR when the full milestone is complete and tested.

### Phase 5: Test
Tests are written by the Dev before the PR is opened. Test gates are tiered:
- **Unit tests** — required for all pure logic (IK math, PD controller, safety clipping). Must pass before PR is opened.
- **Integration smoke tests** — required for hardware-adjacent code. Mark `[HARDWARE]` if physical Arduino is needed.
- **Manual verification** — acceptable only for pure UI changes. Must be noted explicitly.

### Phase 6: Review
Ping the PM in the Claude Project. The PM reads the PR via GitHub connector and posts a review comment. If approved, merge manually. If changes requested, hand back to Claude Code.

### Phase 7: Merge
- Merge the PR manually on GitHub
- PM updates the Architecture section of this document
- Add entry to `CHANGELOG.md`

---

## Architecture

> **Keep this section updated.** This is the primary context fed to the Dev at the start of every session.

```
Project: stewart_control
Purpose: GUI-driven Stewart platform controller with IK solving, serial comms to Arduino,
         demo routines, and closed-loop ball balancing via computer vision.
Stack: Python 3.11+, PyQt5, matplotlib, numpy, pyserial, opencv-contrib-python

Directory structure (target — in progress):
stewart_control/
├── main.py                    # Entry point only
├── config.py                  # Geometry constants only. Do not add runtime config here.
├── settings.py                # Runtime config: port, baud, intervals, safety limits.
│
├── core/
│   ├── platform_state.py      # Dataclasses: Pose, ServoAngles, BallState. No logic.
│   ├── ik_engine.py           # Single IK call site. Nothing else calls ik_solver directly.
│   └── safety.py              # All servo clipping and validation. One place.
│
├── hardware/
│   ├── serial_manager.py      # Connection lifecycle, read loop, callbacks.
│   └── servo_driver.py        # Formats commands, calls serial_manager.
│
├── control/
│   ├── routine_runner.py      # Routine playback state machine. No Qt dependencies.
│   └── ball_controller.py     # PD controller. Pure math. No Qt dependencies.
│
├── cv/
│   └── ball_tracker.py        # Returns BallState. Camera-index configurable.
│
├── routines/
│   └── routines.py            # Pure pose-list generators.
│
├── visualization/
│   └── visualizer3d.py        # Draws pre-solved geometry. Does NOT call IK.
│
├── gui/
│   ├── main_window.py         # Top-level QWidget. Wires all modules.
│   ├── control_panel.py       # Sliders, buttons, routine selector.
│   └── serial_monitor.py      # Serial output display widget.
│
└── tests/
    └── test_safety.py         # Unit tests

Completed so far:
- core/platform_state.py (M1 Step 1)

Module responsibilities:
- config.py: physical geometry. Does NOT own runtime settings.
- settings.py: port, baud, intervals, limits. Does NOT own geometry.
- core/platform_state.py: data shapes only. No logic, no project imports.
- core/ik_engine.py: IK call site. Does NOT own visualization or serial.
- core/safety.py: clipping and validation. No Qt, no serial.
- hardware/serial_manager.py: connection and read loop. Does NOT format commands.
- hardware/servo_driver.py: command formatting and dispatch. Does NOT own connection state.
- control/routine_runner.py: playback state. Accepts tick() from GUI timer. No Qt imports.
- control/ball_controller.py: PD math only.
- cv/ball_tracker.py: vision pipeline. Returns BallState.
- visualization/visualizer3d.py: drawing only. Accepts pre-solved geometry.
- gui/*: view and wiring only. No control logic, no IK calls, no serial command building.

Naming conventions:
- files: snake_case
- classes: PascalCase
- functions/methods: snake_case
- constants: UPPER_SNAKE_CASE (config.py and settings.py only)
- dataclass fields: snake_case

Shared data contract:
- Pose: x, y, z, roll, pitch, yaw — all float, mm and degrees
- ServoAngles: list[float], 6 elements, degrees, index 0-5
- BallState: x_mm, y_mm, vx_mm_s, vy_mm_s (required floats);
             z_mm, vz_mm_s (Optional[float], None if 2D-only)
- IK result dict: {success, platform_points, arm_points, servo_angles_deg,
                   platform_center, platform_R, debug}

Known constraints:
- Do not modify config.py geometry without PM approval and CHANGELOG entry.
- Do not add runtime config to config.py — use settings.py.
- Do not call ik_solver.solve_pose() directly — use core/ik_engine.py.
- Do not build serial command strings inline — use hardware/servo_driver.py.
- Do not put control logic or IK calls in any gui/ file.
- Hardware tests requiring Arduino must be marked [HARDWARE].
- Qt timer callbacks must stay fast (<5ms). Offload blocking work to threads.

Decisions already made:
- PyQt5: already in use, not changing.
- Matplotlib embedded in Qt: acceptable for current update rate.
- Single Arduino over serial: serial is the hardware interface.
- IK solved on CPU per frame: fast enough (<1ms), no GPU needed.
- BallState includes z fields from day one: future multi-camera support.
- BallTracker is instantiable, not a singleton: future multi-tracker support.
```

---

## Dev Prompt Template

This is the full template to paste into Claude Code at the start of every milestone session.
Replace the bracketed sections with the actual milestone content.

```
You are implementing an approved plan for the stewart_control project.
Read everything below before touching any file.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ARCHITECTURE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[paste the Architecture section above]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GIT RULES
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Before touching any file, run `git status`. If the tree is dirty, stop and report.
2. Create ONE feature branch for this entire milestone:
   git checkout -b milestone/[N]-[short-name]
3. Commit after each numbered step using the format:
   [MN] Step X: description of what was done
4. Commit messages must be specific. Never use "update", "fix", "changes".
5. Do not commit broken or untested code.
6. When all steps are complete and tests pass, open ONE pull request to main.
   Title: [MN] [Milestone name]
   Body: list each step, what was created/changed, confirm test results,
         note any deviations from the plan.
7. Do not open multiple PRs. One PR per milestone.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
APPROVED PLAN — Milestone [N]: [Name]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[paste approved milestone steps here]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
RULES
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Plan before implementing. You have an approved plan — follow it exactly.
2. One step at a time. Complete and confirm each step before starting the next.
3. Test before moving on. Each step must pass its test criterion before committing.
4. Stay in scope. Do not modify files outside the plan, even if you see something to fix.
5. No silent assumptions. If something is ambiguous, stop and ask.
6. Don't duplicate. Check whether a utility already exists before creating a new one.
7. Flag early. If the plan won't work as written, stop and report it.
8. No inline config. Ports, intervals, limits go in settings.py, never in logic files.
9. Data contracts first. If a step passes data between modules, confirm the dataclass
   schema in core/platform_state.py is defined before writing code that uses it.
10. Respect layer boundaries. GUI files do not call IK or build serial commands.
    IK goes through ik_engine.py. Serial commands go through servo_driver.py.
11. Git discipline. One branch per milestone. Commit per step. One PR at the end.
```

---

## Rules for the Dev

1. **Plan before implementing.** Follow the approved plan exactly.
2. **One step at a time.** Complete and confirm before starting the next.
3. **Test before moving on.** Each step passes its test before committing.
4. **Stay in scope.** No changes outside the plan.
5. **No silent assumptions.** Ambiguous = stop and ask.
6. **Don't duplicate.** Check before creating.
7. **Flag early.** Plan won't work = stop and report.
8. **No inline config.** Everything goes in settings.py.
9. **Data contracts first.** Dataclass defined before code that uses it.
10. **Respect layer boundaries.** GUI never calls IK or builds serial commands.
11. **Git discipline.** One branch, commit per step, one PR per milestone.

---

## Anti-Patterns to Watch For

| Anti-pattern | What it looks like | What to do |
|---|---|---|
| **Runaway implementation** | Dev writes 500+ lines before review | Enforce step-by-step confirmation |
| **Scope creep** | Dev refactors unrelated code "while it's in there" | Reject and revert. Log as future task. |
| **Silent assumption** | Dev makes architectural decision without flagging | Ask why. Revert if it contradicts the plan. |
| **Duplicate code** | New utility does what an existing one does | Check before creating anything new. |
| **Plan skipping** | Dev starts implementing before plan is reviewed | Stop. Require a plan first. |
| **Premature abstraction** | Generic framework built for a one-off problem | Reject. Solve the specific problem first. |
| **Test deferral** | "I'll add tests later" | No. Tests before the commit. |
| **Context drift** | Dev contradicts earlier decisions | Re-paste architecture. Fresh session if needed. |
| **Inline config** | Port or limit appears as a literal in a logic file | Reject. Move to settings.py. |
| **Layer violation** | GUI file calls IK or builds a serial command string | Reject. Route through correct module. |
| **PR per step** | Dev opens a PR after every single step | Reject. One PR per milestone. |

---

## Session Hygiene

- **Start every Dev session** by pasting the Architecture section.
- **Keep sessions focused.** One milestone per session.
- **Update this document** after every merged milestone.
- **After every milestone**, PM updates Architecture before writing the next brief.
- **Never let the Dev review its own work.** PM reviews via GitHub connector.

---

## File Reference

| File | Purpose |
|---|---|
| `AI_WORKFLOW.md` | This document. Process, roles, rules, templates. |
| `SPEC.md` | Feature specs and acceptance criteria. |
| `CHANGELOG.md` | Log of completed milestones and decisions. |
