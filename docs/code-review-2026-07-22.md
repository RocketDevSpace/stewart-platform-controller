# Full-repo critical review — 2026-07-22

Five parallel line-by-line reviews (kinematics/core, hardware/control,
vision, GUI, tests/tooling/docs) of the entire codebase, synthesized into
one prioritized critique. This document is the citable record: every
finding below maps to the overhaul commit that addressed it (branch
`overhaul/safety-ik-vision-gui`, PR "[Overhaul] Safety rails, IK
correctness, controller/vision/GUI decomposition, tooling").

Status legend: **FIXED-n** = fixed in overhaul step n; **NOTED** =
documented, deliberately not changed; **DEFERRED** = left for future work.

## Systemic patterns (the review's core verdict)

1. **Safety theater** — mechanisms that looked like safety and bound
   nothing: reacquire gate running after commands reached hardware
   (FIXED-3), slew limiter defaulting to 10°/frame — larger than the whole
   tilt envelope (FIXED-5: settings-owned; FIXED-2: real hardware-ramp
   slew policy), dead `SAFETY_LIMITS` min/max keys with a one-sided Python
   clamp (FIXED-2), raw-command box bypassing every rail and dropping the
   newline terminator (FIXED-2), safety tests certifying the gaps as
   correct (FIXED-2/3).
2. **Shadow infrastructure** — things that existed in name only:
   `SERIAL_QUEUE_MAX` with no queue (FIXED-1: deleted; FIXED-2: a real
   latest-wins writer), local mypy broken outright while CI's green
   depended on a synthetic no-PyQt5 environment (FIXED-1), the
   `serial_enqueue` and `frame_to_cmd` plot lines that had never displayed
   a real number (FIXED-4/6), 28 fabricated zero-filled telemetry keys
   (FIXED-4), 16 headline controller tests certifying a `compute()` method
   production never called (FIXED-5), tautological tests asserting their
   own local variables (FIXED-4/5).
3. **Two sources of truth** — wrong routine labels (FIXED-3), 1-indexed
   comments over 0-indexed arrays institutionalized as API naming
   (FIXED-1), a conftest alias justified by imports that don't exist
   (FIXED-1), firmware README claiming a Python 0-180 rail that wasn't
   there (FIXED-2 made the claim true).

## Critical defects

| # | Finding | Status |
|---|---|---|
| C1 | IK branch selection structurally wrong: seed by ball-joint height picks a ~260° branch at yaw=-35 (silently clamped to 180); `theta_alt` always branch-2 so the in-range branch is unrecoverable; returned angles and arm points could describe different elbows; ±180 wrap + silent clamp turned continuous motion into a commanded 180° servo snap (reproduced at yaw≈+15.5°). | FIXED-3 (redesign; sweep-continuity tests; screw max step now 2.7°) |
| C2 | Reacquire gating decorative: ran in the GUI snapshot handler after the servo command was already sent, counting ≤30 Hz snapshots not frames. | FIXED-3 (moved into the worker command path) |
| C3 | ArUco stale-homography served forever after marker loss: hold budget only decremented on every-10th redetect frames; `_last_H` never invalidated. | FIXED-4 (bounded hold + cache invalidation, tested) |
| C4 | Serial read loop died permanently and silently on one callback/read exception while `is_connected()` stayed True. | FIXED-2 |
| C5 | No write lock on serial sends — GUI thread and vision worker both write; interleaved bytes could splice commands. | FIXED-2 |
| C6 | `connect()` double-call leaked the port and raced two reader threads over one byte stream. | FIXED-2 |
| C7 | No flow control / stale-command story on a ~31 ms RTT link fed at up to 30 Hz. | FIXED-2 (latest-wins depth-1 writer thread; vision loop streams through it) |
| C8 | `closeEvent` deadlocked 3 s on every close with vision active, then destroyed a live QThread (GUI thread blocked in `wait()` while the quit chain needed its event loop). | FIXED-2 (local QEventLoop handshake) |
| C9 | 12-line `main.py`: any Qt slot exception → PyQt5 `qFatal()` abort with servos live, serial never disconnected. | FIXED-2 (excepthook + `emergency_shutdown()`: ramped neutral + disconnect) |
| C10 | Raw-command box: GUI→serial in one hop, bypassing ServoDriver, safety clipping, and (future) slew guard — and `.strip()` dropped the `\n`, so commands sat unexecuted in the Arduino line buffer. | FIXED-2 (`ServoDriver.send_raw`) |
| C11 | No mode mutual exclusion: routine playback, manual SEND, raw box all live during vision mode; finishing routines re-enabled sliders vision had locked. | FIXED-2 |
| C12 | `BallTracker.release()` could call `cap.release()` concurrently with a blocked `cap.read()` (segfault vector). | FIXED-4 |
| C13 | mypy dead locally (missing `cv/__init__.py`); CI green only because mypy ran before PyQt5 was installed. | FIXED-1 |
| C14 | Floor-only dependency pins already resolved OpenCV 5.0/numpy 2.4 untested (OpenCV 5 then live-verified compatible). | FIXED-1 (ceilings; comment updated) |

## Major design debt

| Finding | Status |
|---|---|
| `ball_controller.py` god object: 3-writer setpoint + shadow variable, covert `_target_change_time` channel, autotune toggling trim's flag, cwd-relative truncating log file, id()-keyed logger, 6 raw clock calls blocking testability, production limits as signature defaults | FIXED-5 (SetpointArbiter / AutoTrim / PDAutotuner / PDCore split; injected clock; settings-owned limits) |
| Divergent `compute()` (tests-only) vs `compute_with_terms()` (production) | FIXED-5 (deleted; tests migrated) |
| Worker→tracker: 10 private reach-ins incl. acquiring the tracker's internal lock; duplicated stale-frame check | FIXED-4 (public CameraSource/BallTracker API) |
| Camera lifecycle tangled into tracker: multi-second blocking ctor, exposure probes starving the pipeline ~1 s | FIXED-4 (CameraSource, two-phase open, probes publish frames) |
| Double-buffer torn-frame risk (reader and writer both outside the lock) | FIXED-4 (copy under lock both sides) |
| Position filter blended reacquired ball toward stale pre-loss position | FIXED-4 |
| `TRACKER_WARP_SIZE_PX` (tuned 480) silently ignored for ctor default 800 — ~2.8× pixel work | FIXED-4 |
| Untyped IK result dict: zeros as failure sentinels (0° is a valid extreme command), 2-key exception shape violating the 7-key contract, dead `ServoAngles` contract | FIXED-3 (frozen `IKResult` dataclass, complete failure shape, neutral placeholders) |
| Save-trim regex rewrite of settings.py: unrounded repr floats (observed live), silent zero-substitution success, sci-notation corruption path, non-atomic write | FIXED-5 (settings_store overlay, atomic writes, untracked user_settings.json) |
| Blocking ~2 s serial connect freezing the GUI before first paint | FIXED-6 (background connect + [READY] poll) |
| matplotlib full-rebuild rendering: `cla()`+replot at 25 Hz (3D) and per-update legend rebuild (timing plot) on the GUI thread vs the repo's own <5 ms rule | FIXED-6 (persistent artists, `set_data`, `draw_idle`) |
| GUI IK call sites (hard constraint 5); neutral-fallback control policy living in the GUI snapshot handler on subsampled snapshots | FIXED-6 (PoseCommander; policy moved into worker miss branch) |
| `z_provider` lambda: worker-thread read of a GUI attribute bypassing the signals-only contract | FIXED-6 (`set_z` slot) |
| Routines ended parked at their most extreme pose (screw: z=-12/yaw=-35) | FIXED-3 (ease-to-neutral on completion and cancel — approved behavior change) |
| Routine runner restart-after-exhaustion IndexError; silent IK-failure skipping | FIXED-3 |
| Unbounded log widgets; slider sync fighting the user mid-drag; autotune double-apply race; stale vision UI after disable | FIXED-6 |

## Test-quality findings

- Headline controller tests certified dead code; `TestZetaFromOvershoot`
  and the worker "backpressure" tests were literal tautologies — FIXED-4/5
  (replaced with real-path tests + characterization suite with injected
  clock).
- Zero coverage for serial_manager, ik_solver, ball_tracker — FIXED-2/3/4
  (fake-serial suite; golden values + rod-length invariant +
  sweep-continuity; synthetic-frame ArUco pipeline tests; CameraSource
  fake-capture tests).
- Only 1 of 5 routine generators ever executed under test — FIXED-3.
- Suite: 90 tests before the overhaul → 206 after, with the riskiest
  modules covered for the first time. setup.cfg exclude lists now empty:
  every module linted and type-checked.

## Noted / deferred

- **Cone Tracing routine exits the workspace on 20 of 120 steps**
  (`ratio ≈ -1.005`) — pre-existing under the old solver too, just
  invisible. Now honestly reported and pinned by test. Adjusting the
  routine's tilt/radius is Hudson's call (physical envelope decision).
  DEFERRED.
- IKEngine stays stateless (continuity threading remains explicit at call
  sites) — a stateful engine would cross-contaminate independent command
  streams (vision loop vs routines). NOTED, deliberate.
- CI remains ubuntu-only for a Windows app — unit/lint/type coverage only;
  hardware paths stay manual. NOTED in ci.yml.
- `firmware/wiring_check.py` builds its command string inline — documented
  exemption from hard constraint 4 (standalone bench utility). NOTED in
  setup.cfg.
