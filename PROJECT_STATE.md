# PROJECT_STATE.md

*Living document — update at the end of substantive sessions. Sync between venues if applicable.*

**Last updated:** May 12, 2026 — M7 implementation complete (PR #10, awaiting review/merge).

---

## What we're building (one-paragraph version)

A PyQt5 desktop application that drives a hand-built 6-DOF Stewart platform via Arduino serial. The app exposes manual 6-axis pose control, runs scripted motion routines, and closes a vision loop on a tracked ball to balance it on the platform surface. The current phase is a multi-milestone refactor of an originally Codex-written codebase, restructuring it into clean layered modules with proper data contracts, single call sites for shared concerns, and a real test suite — without changing functional behavior.

## Current phase

**Refactor complete (M1–M7 implemented). M7 PR #10 awaiting review and merge.**

M7 (Codex audit + integration) is implemented on branch `milestone/7-codex-integration` (PR #10). Shipped in two sub-PRs: PR #9 (Steps 1–6: settings constants, BallTracker background thread + camera improvements + detection improvements, BallController auto-trim + autotune + compute_with_terms) and PR #10 (Steps 7–10: VisionControlWorker, ControlPanel vision controls + dark mode, VisionMonitorWindow, MainWindow rewire + timing plot + safety fallback). Hardware smoke test passed: manual control, routines, and vision mode all working. CI restructured (mypy before PyQt5 install) and passing.

Post-merge, the full refactor is done. The codebase is in the clean layered architecture the refactor was targeting. Future work is feature development (second camera, ball catching, ball bouncing).

## Architectural commitments

These are committed and shape the project. Revisable only with explicit discussion.

- **PyQt5 stays.** Already in use, not changing.
- **Matplotlib embedded in Qt** for visualization. Acceptable for current update rate; no migration to OpenGL or similar.
- **Single Arduino over serial at 115200 baud.** Command format: `S,a0,a1,a2,a3,a4,a5,0\n`. 6 servos, 0-indexed, integer degrees 0–180.
- **IK solved on CPU per frame.** <1ms typical, fast enough; no GPU.
- **`BallState` includes z fields from day one.** Future multi-camera support is anticipated, so the dataclass is shaped for it now even though only x/y are populated currently.
- **`BallTracker` is instantiable, not a singleton.** Future multi-tracker / multi-ball support.
- **Layered module architecture.** GUI → Control → Hardware → Serial. Layer violations are not fixed in place; they are reverted.
- **Single call sites for shared concerns.** IK lives in `core/ik_engine.py`. Serial command formatting in `hardware/servo_driver.py`. Safety clipping in `core/safety.py`. Data shapes in `core/platform_state.py`.
- **Repo-root-relative imports.** No `stewart_control.*` prefixes. The original Codex codebase used these; they're being removed as code touches them.
- **CI on every push and PR** via `.github/workflows/ci.yml` — pytest, flake8, mypy. Legacy modules are excluded; exclusion list shrinks each milestone.

## Decisions log

| # | Decision | Rationale | Status |
|---|----------|-----------|--------|
| 1 | Use `core/ik_engine.py` as the single IK call site | Prevent IK logic duplication across visualizer, GUI, and routines. | Committed (M3) |
| 2 | Use `hardware/servo_driver.py` as the single serial command formatter | Prevent inline `"S,..."` strings scattered across code. | Committed (M2) |
| 3 | Use `core/safety.py` as the single servo clipping site | Prevent drift between separate clipping implementations. | Committed (M1) |
| 4 | Use `core/platform_state.py` for all dataclasses | Single source of truth for `Pose`, `ServoAngles`, `BallState`. | Committed (M1) |
| 5 | `control/routine_runner.py` is Qt-free | Testable in isolation; GUI owns the QTimer. | Committed (M4) |
| 6 | `_LegacySerialAdapter` bridges SerialSender → ServoDriver during refactor | Allows M4 to land without breaking M2's serial work; removed in M5. | Committed (M4), to-remove (M5) |
| 7 | CI uses exclude-list pattern (not include-list) for flake8 / mypy | Per reviewer feedback in M2; new clean modules are covered automatically without CI config changes. | Committed (M2) |
| 8 | Adopt the v5 / bootstrap-v2 doc system | Replaces AI_WORKFLOW.md and DEV_STANDING_ORDERS.md with CLAUDE.md + PROJECT_CONTEXT + PROJECT_STATE. Existing SPEC.md and CHANGELOG.md retained. | Committed (May 7, 2026) |
| 9 | M5 and M6 scope expanded to include cleanup items | `main.py` import fix + `gui/gui_main.py` removal added to M5; `ball_controller.py` move + `comms/` retirement added to M6. | Committed (May 7, 2026) |
| 10 | M5 implementation took the 'preserve legacy file as `gui_layout_legacy.py` for one cycle' approach instead of immediate deletion | Allows side-by-side comparison during the M5 review and first manual hardware test. Deletion happens in M6. | Committed (PR #5, 2026-04-23) |
| 11 | M6 Step 1 also updates `gui/main_window.py` dict accesses on `ball_state` | PM plan omitted this; `_vision_control_step` accesses `ball_state["x_mm"]` and `ball_state["y_mm"]` which break at runtime once tracker returns `BallState`. Fix is in-scope for Step 1 since it's the same transition. | Committed (M6, May 10, 2026) |
| 12 | M6 Step 5 narrows `cv/` flake8/mypy exclude to `cv/ball_tracker.py` rather than removing it | PM plan said "remove cv/ball_controller.py from excludes" but setup.cfg excludes the whole `cv/` directory. After ball_controller moves to `control/`, `cv/ball_tracker.py` remains untyped and must stay excluded. Directory-level exclude is replaced with a file-level one. | Committed (M6, May 10, 2026) |

For shipped technical changes per milestone, see `CHANGELOG.md`. For milestone scope and acceptance criteria, see `SPEC.md`.

## Constraints and known limitations

- **Servo safety limits:** servos 0, 2, 4 max 170°; servos 1, 3, 5 min 10°. Hardware-defined; do not change without re-measuring rig.
- **Control loop:** 50 Hz target (20 ms QTimer). Vision loop runs at the same cadence.
- **Qt timer callbacks must stay fast** (<5 ms). Blocking work goes to threads.
- **Hardware tests require a physical Arduino on COM4** (or whatever `settings.SERIAL_PORT` is set to). These tests are marked `[HARDWARE]` and skipped in CI.
- **Windows-only.** Project depends on COM-port serial conventions and PyQt5; no Mac/Linux support planned.
- **Vision pipeline currently 2D.** Single camera + ArUco corner markers + HSV blob detection. z-axis ball state is plumbed but not populated.
- **Tracked `__pycache__/` directories exist in some folders** as legacy artifacts from before `.gitignore` was set up. They're inert; cleanup is a low-priority housekeeping task.

## Things Claude has gotten wrong on this project

*Running log. Format: trigger / failure / corrected pattern. Scan at the start of each session.*

(Note: a fresh Claude reading the prior `AI_WORKFLOW.md` and `DEV_STANDING_ORDERS.md` would have inferred the "current state" architecture matches the target architecture. It doesn't — `main.py` uses forbidden imports, `ball_controller.py` is in the wrong directory, `gui/gui_main.py` is orphaned. **First entry for the log if any of these surface again:** *Trigger: a fresh session reads the architecture doc and assumes target = current. Failure: would propose changes against the wrong baseline. Corrected pattern: read CLAUDE.md's "Refactor state" section explicitly; do not assume target architecture matches current code.*)

**Entry 1 (2026-05-08):** *Trigger: a fresh Claude session reads PROJECT_STATE.md's 'M5: not started, next up' and treats it as authoritative without checking open PRs. Failure: produced a full M5 plan from scratch when PR #5 had been open for two weeks containing real M5 work. Corrected pattern: at session start, after reading PROJECT_STATE / CLAUDE.md, run `gh pr list` (or the GitHub MCP equivalent) and read open PR titles before treating any 'next up' / 'pending' status as the ground truth.*

## Open questions

**Open, post-M7:**
- Second-camera setup. Needs a real plan after M7 lands. Hardware needs (camera, mounting, calibration approach), software needs (multi-tracker composition, 3D reconstruction math), GUI changes (second video pane).
- Ball catching feasibility gated on sub-20ms loop benchmark. Benchmark on hardware before scoping.

**Closed:**
- Whether to use the v5 / bootstrap-v2 doc system → adopted May 7, 2026 (Decision #8 above).
- Whether to expand M5/M6 scope to include cleanup → yes, expanded May 7, 2026 (Decision #9 above).
- Hardware smoke test gate for PR #5 → passed May 10, 2026.
- `BallTracker` returning `BallState` dataclass → resolved in M6 (PR #7, Step 1).
- Where `ball_controller.py` moves → resolved in M6 (PR #7, Step 3). Moved to `control/ball_controller.py`.
- M7 Phase 1 (audit) → complete May 10, 2026. Audit doc at `docs/codex_audit.md`.
- M7 Phase 2 (implementation) → complete May 12, 2026 (PR #10). All 22 feature groups ported.
- Dark mode scope → full app via `DARK_QSS` applied at `MainWindow` level.
- PR granularity for `cv/ball_tracker.py` → single PR (Steps 2–4 batched in PR #9).
- `BallTracker.update()` signature → kept `BallState` return; Codex dict logic ported internally.

## Phase roadmap

Rough plan, will evolve. Each phase produces reviewable artifacts; each builds on previous.

1. **M1 — Foundation** ✅ (April 21, 2026)
2. **M2 — Hardware Layer** ✅ (April 22, 2026)
3. **M3 — IK Consolidation** ✅ (April 22, 2026)
4. **M4 — Routine Runner Extraction** ✅ (April 22, 2026)
5. **M5 — GUI Split + cleanup items** ✅ (May 10, 2026)
6. **M6 — Vision Loop Cleanup + ball_controller move + comms/ retirement** ✅ (May 10, 2026)
7. **M7 — Codex audit + integration** ✅ (May 12, 2026, PR #10 awaiting merge)
8. **Phase 2 (post-refactor): Second-camera setup** — needs scoping after M6 lands. Adds 3D ball tracking as the foundation for ball catching and bouncing.
9. **Phase 3: Ball catching** — trajectory prediction from 3D state, platform pre-positioning. Requires sub-20ms loop benchmark first.
10. **Phase 4: Ball bouncing** — timed platform impulse for vertical oscillation. Requires Phase 2.
11. **Phase 5: Multiple ball targets** — multi-blob tracking, target assignment.

The post-refactor phases (8–11) are not committed; they're the trajectory the architecture is being built to support. Real plans get written when M6 lands and a Phase 2 SPEC update is needed.

## Project layout on disk

Repo lives at `https://github.com/RocketDevSpace/stewart-platform-controller`. Code lives directly in repo root (no `stewart_control/` package directory despite legacy import paths suggesting one).

For current architecture, see CLAUDE.md.

## How to use this doc

Update at the end of substantive sessions. When a new milestone is approved, add an entry to the Phase roadmap. When a decision is made, log it in the Decisions table. When Claude makes a mistake, append to the "Things Claude got wrong" log with structured format.

This doc lives in the repo root alongside CLAUDE.md, PROJECT_CONTEXT.md, SPEC.md, CHANGELOG.md. It is also uploaded to the Claude.ai project knowledge for the planning venue.

When the project state changes meaningfully — milestone completion, scope expansion, new constraint discovered — the chat venue updates this doc, the code venue commits the update on the active branch. Per the chat-as-planner / code-as-executor write-pattern rule from HUDSON_CONTEXT.md, the code venue does the actual GitHub commits except for short edits.
