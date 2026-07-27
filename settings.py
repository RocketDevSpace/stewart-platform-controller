# =============================================================================
# User-settings overlay (M12)
# =============================================================================
# Per-machine overrides live in user_settings.json (untracked; see
# settings_store.OVERRIDABLE_KEYS for the whitelist). Loaded once at import;
# overridable keys below read from _OV with the committed value as default.
import settings_store as _settings_store

_OV = _settings_store.load_user_overrides()

# =============================================================================
# Serial
# =============================================================================
SERIAL_PORT = str(_OV.get("SERIAL_PORT", "COM4"))
# Firmware v2 runs 250000 baud (0% UART timer error at 16 MHz). connect()
# automatically retries at the legacy 115200 when no boot banner appears
# (v1 firmware), so this default is safe on either firmware.
SERIAL_BAUD = 250000

# =============================================================================
# Safety limits
# =============================================================================
# Max platform tilt PER AXIS in vision mode. IK verified solvable at the
# 10/10 combined corner (14 deg total); 12/12 has no real solution.
MAX_TILT_DEG = 10.0

SAFETY_LIMITS = {
    "max_angle": 180,       # global ceiling, all servos (enforced in core/safety.py)
    "min_angle": 0,         # global floor, all servos (enforced in core/safety.py)
    "odd_servo_max": 170,   # extra ceiling for INDICES 0, 2, 4 (mirrored mount)
    "even_servo_min": 10,   # extra floor for INDICES 1, 3, 5 (mirrored mount)
}

# Largest per-servo jump (deg) sent as an instant write (firmware speedDelay 0).
# Bigger jumps are sent with SERVO_LARGE_MOVE_SPEED_DELAY_MS so the FIRMWARE
# ramps the move (1 deg per speedDelay ms, all servos concurrently; firmware
# clamps speedDelay to 0-20 and acks only after the ramp — see firmware/README.md).
SERVO_SLEW_INSTANT_MAX_DEG = 12.0
SERVO_LARGE_MOVE_SPEED_DELAY_MS = 5     # -> 200 deg/s hardware ramp on big moves

# Anti-dither command quantization (perf pass). The protocol carries whole
# degrees; without hysteresis, PD noise near a rounding boundary flaps the
# servos +/-1 deg continuously (measured: ~6100 integer flips/min at rest).
# A servo's committed integer only changes when the commanded float crosses
# the boundary by this margin (Schmitt trigger); identical quantized command
# tuples are not re-sent at all (the firmware holds its last command; it has
# no watchdog).
SERVO_QUANT_HYST_DEG = 0.4
SERVO_DEDUP_ENABLED = True
# With firmware v2's tenth-degree T protocol the command grid is 0.1 deg, so
# the Schmitt margin shrinks accordingly (still > half a grid step).
SERVO_QUANT_HYST_FINE_DEG = 0.25
# "auto": use the tenth-degree T protocol when the connected firmware is v2
# (small streaming moves; large moves still go via legacy S + firmware ramp).
# "legacy": force whole-degree S commands regardless of firmware.
SERVO_PROTOCOL = "auto"

# =============================================================================
# Camera
# =============================================================================
CAMERA_INDEX = int(_OV.get("CAMERA_INDEX", 1))   # 0 = integrated, 1 = USB camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_BUFFER_SIZE = 1
CAMERA_TARGET_FPS = 30
CAMERA_FORCE_BACKEND = "DSHOW"          # "DSHOW", "MSMF", "ANY", or "" to probe
CAMERA_AUTO_EXPOSURE = True
CAMERA_EXPOSURE = -4.0
CAMERA_RUNTIME_ADAPTIVE = True
CAMERA_RUNTIME_WARMUP_S = 1.0
CAMERA_RUNTIME_CHECK_S = 1.0
CAMERA_RUNTIME_MAX_PERIOD_MS = 42.0
CAMERA_RUNTIME_MIN_GRAY = 28.0
CAMERA_RUNTIME_TARGET_GRAY = 55.0
CAMERA_RUNTIME_SOFT_GAIN_MAX = 2.4

# =============================================================================
# Tracker
# =============================================================================
TRACKER_WARP_SIZE_PX = 480
TRACKER_ARUCO_DETECT_SCALE = 0.5
# Every-frame marker detection (perf pass): the old freeze/re-solve cadence
# (N=10) was injecting a ~3 Hz position stairstep as H snapped to each fresh
# solve; solving from filtered centers every frame keeps the warp smooth.
TRACKER_ARUCO_REDETECT_EVERY_N = 1
TRACKER_WARP_GRAY_CACHE_N: int = 5   # recompute warp gray mean every N frames
TRACKER_MIN_RADIUS_PX = 4.0
TRACKER_MIN_CONTOUR_AREA = 150.0
# Marker-center filter (perf pass): deadband + scheduled alpha. Motion below
# DEADBAND_PX freezes the filtered center (H fully static at rest); motion
# past FAST_PX uses the fast alpha so real platform tilt tracks in 1-2
# frames; in between the slow alpha smooths drift.
TRACKER_ARUCO_CENTER_DEADBAND_PX = 0.3
TRACKER_ARUCO_CENTER_FAST_PX = 1.5
TRACKER_ARUCO_CENTER_ALPHA_SLOW = 0.70
TRACKER_ARUCO_CENTER_ALPHA_FAST = 0.2
# 6 hold attempts at every-frame detection = the same wall-clock stale-H
# budget the old 3-attempts policy allowed at N=10 loss cadence.
TRACKER_MAX_ARUCO_HOLD_FRAMES = 6
TRACKER_ARUCO_SUBPIX_REFINE = True    # detector-level SUBPIX corner refinement
TRACKER_ARUCO_FULLRES_SUBPIX = True   # re-refine used corners on full-res gray
TRACKER_BALL_SUBPIXEL = True          # float (sub-pixel) ball centroid
TRACKER_POS_FILTER_ENABLED = False
TRACKER_POS_FILTER_ALPHA_SLOW = 0.88
TRACKER_POS_FILTER_ALPHA_FAST = 0.25
TRACKER_POS_FILTER_SPEED_MM_S = 180.0
TRACKER_POS_FILTER_MAX_LAG_MM = 1.5
TRACKER_REACQUIRE_VALID_FRAMES = 3
TRACKER_MAX_SPEED_MM_S = 0.0            # 0 disables speed outlier rejection
TRACKER_MIN_CIRCULARITY = 0.0           # 0 disables circularity check
TRACKER_MIN_FILL_RATIO = 0.0            # 0 disables fill ratio check

# HSV defaults (orange ball)
TRACKER_HSV_H_MIN = int(_OV.get("TRACKER_HSV_H_MIN", 10))
TRACKER_HSV_H_MAX = int(_OV.get("TRACKER_HSV_H_MAX", 28))
TRACKER_HSV_S_MIN = int(_OV.get("TRACKER_HSV_S_MIN", 83))
TRACKER_HSV_S_MAX = int(_OV.get("TRACKER_HSV_S_MAX", 255))
TRACKER_HSV_V_MIN = int(_OV.get("TRACKER_HSV_V_MIN", 125))
TRACKER_HSV_V_MAX = int(_OV.get("TRACKER_HSV_V_MAX", 255))

# low-pass weight on raw velocity (0=frozen, 1=raw); ~6 Hz cutoff at 30 fps; tunable
BALL_VEL_FILTER_ALPHA: float = 0.55

# Measurement-filter mode (cv/measurement_filter.py):
#   "alpha_beta" — adaptive alpha-beta tracker: gains scheduled between MIN
#                  (quiet, near-static) and MAX (fast acquisition) by
#                  innovation magnitude and predicted speed.
#   "legacy"     — original position low-pass + velocity EMA (regression ref).
#   "raw"        — passthrough position + raw finite-difference velocity
#                  (bench comparison only).
TRACKER_FILTER_MODE = "alpha_beta"
TRACKER_AB_ALPHA_MIN = 0.40
TRACKER_AB_ALPHA_MAX = 0.90
# BETA_MIN raised 0.05 -> 0.25 (live tuning 2026-07-22): at 0.05 the
# quiet-mode velocity estimate lagged ~660 ms - near 180 deg of phase
# at the observed 0.77 Hz small-amplitude oscillation - turning the
# D-term from damping into excitation (self-sustaining rock that also
# blocked auto-trim's settle gate). 0.25 keeps ~<40 deg lag there.
TRACKER_AB_BETA_MIN = 0.25
TRACKER_AB_BETA_MAX = 0.60
TRACKER_AB_INNOV_OPEN_MM = 1.5       # innovation below this: gains stay MIN
TRACKER_AB_INNOV_FULL_MM = 4.0       # innovation above this: gains at MAX
TRACKER_AB_SPEED_OPEN_MM_S = 60.0    # predicted speed below this: no opening
TRACKER_AB_SPEED_FULL_MM_S = 150.0   # predicted speed above this: gains at MAX
# Single-frame glitch veto (2026-07-23 path sessions): when the ball
# transits an ArUco marker its edge partially occludes the marker and
# the homography glitches — >4 mm single-frame position jumps at 3x
# the base rate near the marker diagonals, which the PID then chases.
# An innovation beyond VETO_MM coasts on the prediction for at most
# VETO_MAX_FRAMES (then accepts, so sustained real motion is never
# suppressed — a genuine 300 mm/s flick is delayed by one frame at
# most, and the perf-pass impulse profile at 5 mm/frame never trips
# the 6 mm threshold at all).
TRACKER_AB_VETO_MM = 6.0
TRACKER_AB_VETO_MAX_FRAMES = 1

# --- Boundary-quad platform tracking (2026-07-24) ---
# Primary homography source: fit the platform's four gray boundary edges
# (the ball can NEVER reach them — max center excursion ~85 mm vs the
# ±120 mm boundary) and intersect them for the warp corners, instead of
# the ArUco marker centers at ±60 mm that the ball occludes during path
# transits (rig-measured: >4 mm single-frame H glitches on 19% of frames
# near the marker diagonals, 3x baseline). ArUco is demoted to
# acquisition seed, identity/scale reference, periodic cross-check, and
# fallback. See cv/quad_tracker.py.
TRACKER_QUAD_ENABLED = True
TRACKER_QUAD_SAMPLES_PER_SIDE = 32     # edge samples per boundary side
TRACKER_QUAD_BAND_TRACK_PX = 5         # half-width of the search band (locked)
TRACKER_QUAD_BAND_ACQ_PX = 12          # half-width while acquiring (wider: seed error)
TRACKER_QUAD_MIN_GRAD = 6.0            # min |edge gradient| (contrast floor —
#                                        gray-on-gray background fails CLOSED to ArUco)
TRACKER_QUAD_MIN_INLIERS = 12          # min surviving samples per side after trim
TRACKER_QUAD_TRIM_RESID_PX = 0.75      # residual trim floor (max with 2.5-sigma MAD)
TRACKER_QUAD_BALL_EXCLUDE_PX = 30.0    # drop samples near the ball (its silhouette
#                                        can overlap the boundary from the oblique
#                                        camera because the ball has height); 0 = off
TRACKER_QUAD_ACQ_FRAMES = 10           # consecutive good fits required to lock
TRACKER_QUAD_MAX_MISS_FRAMES = 6       # locked-fit failures before dropping to UNLOCKED
TRACKER_QUAD_MAX_CORNER_STEP_PX = 15.0  # gate: max per-frame corner motion while locked
TRACKER_QUAD_SIDE_RATIO_TOL = 0.06     # gate: side-length drift vs previous frame
TRACKER_QUAD_MAX_ANGLE_DELTA_DEG = 5.0  # gate: corner-angle drift vs previous frame
TRACKER_QUAD_CROSSCHECK_EVERY_N = 15   # run ArUco every N frames while locked
# The audit is BASELINE-RELATIVE (rig finding 2026-07-24): ArUco must
# extrapolate the +/-60 mm marker square 2x outward to predict the
# +/-120 mm boundary corners, and real-lens radial distortion makes that
# prediction structurally wrong by several px — the quad measures where
# the boundary actually IS. On the rig the raw disagreement exceeded the
# old absolute tolerance on EVERY audit, so 3 strikes (45 frames = 1.5 s)
# force-reacquired the quad in a perfectly periodic unlock/relock cycle.
# The residual at lock time is now stored as the baseline; audits alarm
# on CHANGE from it (genuine drift), with a slow blend on passes to
# follow tilt. A raw disagreement over SLIP_PX is a hard strike
# regardless of baseline (identity slip / false structure).
TRACKER_QUAD_CROSSCHECK_TOL_PX = 4.0   # mean |change from baseline| to pass
TRACKER_QUAD_CROSSCHECK_SLIP_PX = 25.0  # raw disagreement = hard strike
TRACKER_QUAD_CROSSCHECK_FAILS_TO_REACQ = 3   # consecutive fails -> force reacquire
# Skip the cross-check while the ball is within this distance of ANY
# marker center: ArUco is exactly then untrustworthy (the occlusion this
# feature exists to defeat), and a slow marker transit otherwise reads
# as PERSISTENT disagreement — three straight failed audits would force
# the quad to reacquire from the glitched ArUco H (caught in sim: the
# synthetic transit sweep reproduced the rig's >4 mm jumps through
# exactly this path). Marker half-diagonal ~21 mm + ball radius ~15 mm
# + margin. 0 disables the guard.
TRACKER_QUAD_CROSSCHECK_BALL_NEAR_MARKER_MM = 45.0
#   (transient disagreement prefers the QUAD — transient ArUco error IS the rig
#   failure mode; persistent disagreement prefers ARUCO — only it carries
#   absolute identity and scale, so a persistently divergent quad relocks)
# Corner deadband LP (mirrors the ArUco marker-center filter): H is fully
# static at rest yet tracks real tilt in 1-2 frames.
TRACKER_QUAD_CORNER_DEADBAND_PX = 0.3
TRACKER_QUAD_CORNER_FAST_PX = 1.5
TRACKER_QUAD_CORNER_ALPHA_SLOW = 0.70
TRACKER_QUAD_CORNER_ALPHA_FAST = 0.2
TRACKER_QUAD_CORNER_SNAP_PX = 40.0

# =============================================================================
# PD controller
# =============================================================================
PD_DEFAULT_KP = float(_OV.get("PD_DEFAULT_KP", 0.045))
PD_DEFAULT_KD = float(_OV.get("PD_DEFAULT_KD", 0.022))
PD_MAX_TILT_RATE_DEG_S = 300.0          # slew ("tilt rate") limit on commanded tilt
# Derivative-term contribution cap. Was 2.5 as a velocity-NOISE guard;
# with the alpha-beta filter the velocity is clean, and live data showed
# the cap saturating on every fast event (a 300 mm/s flick wants
# kd*300 ~ 6.6 deg of braking) - the weak early flick response.
PD_D_TERM_LIMIT_DEG = 6.0
BALL_TARGET_DEFAULT_X_MM = 0.0
BALL_TARGET_DEFAULT_Y_MM = 0.0

# --- Integral term (2026-07-23 I-term rework) ---
# The loop's ONLY integral action: cancels the plate's position-dependent
# tilt bias (rig-measured: ~0.36 deg more compensation needed at r=65 mm
# than at center) that pure P+D holds as a standing offset (~22 mm per
# degree at kp 0.045). Continuous — no settle gates; protections are the
# error taper, per-axis anti-windup at the tilt clamp, the leak, and the
# clamp (see control/pid_core.py).
# ki: tau_I = kp/ki = 1.5 s (0.4 deg bias ~90% cancelled in ~3-4 s).
# I-corner ki/kp = 0.67 rad/s must exceed the 0.46 rad/s carrot rotation
# of a 30 mm/s r=65 circle (tracks the rotating field) while adding only
# ~8 deg phase at the 0.77 Hz problem mode.
PD_I_ENABLED = True
PD_DEFAULT_KI = float(_OV.get("PD_DEFAULT_KI", 0.030))  # deg/(mm*s)
PD_I_LIMIT_DEG = 1.5                 # ~4x the measured field
PD_I_LIMIT_HOME_CAL_DEG = 6.0        # home-cal absorbs a whole bad trim
PD_I_LEAK_TAU_S = 25.0               # steady-state cost ~0.5 mm
PD_I_ERR_FULL_MM = 25.0              # full integration at/below
PD_I_ERR_ZERO_MM = 60.0              # zero integration at/above (linear)
# Low-side integration deadband (rig-tuned 2026-07-23): the plate's
# REAL stiction is ~0.3-0.5 deg — a ball parked inside ~2-7 mm cannot
# be moved by P alone, so an integral that keeps demanding zero error
# winds up until it snaps the ball loose, overshoots, and hunts
# forever at 5-15 mm amplitude (measured: the ball never rested in a
# 3-minute static hold, ~17 mm/s perpetual wander). Below DEADBAND the
# integral stops (ramping to full by 2x DEADBAND): the ball parks
# within the stiction scale, the integral goes flat, and rest engages.
PD_I_ERR_DEADBAND_MM = float(_OV.get("PD_I_ERR_DEADBAND_MM", 2.0))
# Rest may only engage once the integral is flat (|dI/dt| EMA under
# this). Resting parks the output at trim + I with P and D dropped —
# resting on a still-converging integral is not an equilibrium and
# limit-cycles at ~0.2 Hz (sim-caught during the rework; the same
# structural cycle the old gated trim produced as the stale-trim
# "rocking"). At convergence the net rate is ~0 (integration balances
# leak), far under this threshold.
REST_I_RATE_MAX_DEG_S = 0.02

# --- Trajectory feedforward + latency compensation (2026-07-23, second
# rig session) ---
# Measured: at low path speeds the ball's wobble (mean ~28 mm/s total
# motion) dominated the ~11 mm/s path drive — following was invisible.
# Two causes fixed here:
# 1. The D-term damped ALL velocity, including the DESIRED path motion
#    (kd*30 mm/s = 0.66 deg of braking against the carrot — the whole
#    pursuit lag). D now damps velocity ERROR vs the path's desired
#    velocity, and the follower feeds centripetal tilt forward, so path
#    motion is commanded, not dragged out of lag error.
# 2. Pipeline latency (~2-3 frames camera->servo) costs ~20 deg of
#    phase at the 0.78 Hz problem mode. Control errors are computed
#    against the ball position extrapolated by CONTROL_PREDICT_S.
PATH_FF_ENABLED = True
PATH_FF_LOOKAHEAD_S = 0.12           # evaluate ff ahead by the pipeline lag
PATH_FF_TILT_MAX_DEG = 1.5           # cap (polyline corners spike curvature)
CONTROL_PREDICT_S = float(_OV.get("CONTROL_PREDICT_S", 0.08))  # ball-state forward extrapolation

# =============================================================================
# Near-target rest mode (control/rest_gate.py)
# =============================================================================
# When the ball has sat within REST_ENTER_RADIUS_MM with low-passed speed
# under REST_ENTER_SPEED_MM_S continuously for REST_ENTER_HOLD_S, the
# controller rests: it commands level + trim offsets instead of chasing PD
# noise (the sends dedup away and the servos go quiet). Exit is hysteretic
# and INSTANT — raw radius or raw instantaneous speed past the wider exit
# thresholds restores full PD on the same control cycle.
REST_MODE_ENABLED = True
REST_ENTER_RADIUS_MM = 8.0              # enter: raw radius at/under this
REST_EXIT_RADIUS_MM = 12.0              # exit: raw radius above this (same cycle)
REST_ENTER_SPEED_MM_S = 15.0            # enter: LPF speed at/under this
REST_EXIT_SPEED_MM_S = 30.0             # exit: raw speed above this (same cycle)
REST_ENTER_HOLD_S = 0.5                 # entry conditions must hold this long
REST_SPEED_LPF_ALPHA = 0.6              # RestGate's own speed EMA (weight on prev)

# =============================================================================
# Trim store (control/trim_store.py)
# =============================================================================
# Persistent level offsets. The gated auto-trim integrator (and its 11
# AUTO_TRIM_* gate settings) was deleted in the 2026-07-23 I-term rework
# - live leveling now happens in the PIDCore integral above; trim is a
# pure store the integral is FOLDED into on Save Trim / home-cal
# completion.
MANUAL_ROLL_TRIM_DEG = float(_OV.get("MANUAL_ROLL_TRIM_DEG", 0.0))
MANUAL_PITCH_TRIM_DEG = float(_OV.get("MANUAL_PITCH_TRIM_DEG", 0.0))
TRIM_LIMIT_DEG = 8.0                 # offset/fold clamp per axis

# Home calibration (I-term rework): hold the ball at center, watch the
# integral converge, then auto-fold it into trim, auto-save, and
# auto-complete. Converged = the integral moved less than EPS per axis
# over the trailing WINDOW while the ball is slow. Timeout cancels
# WITHOUT folding (an unconverged integral is a transient, not a trim).
HOME_CAL_CONVERGE_WINDOW_S = 2.0
HOME_CAL_CONVERGE_EPS_DEG = 0.05
HOME_CAL_CONVERGE_MAX_SPEED_MM_S = 20.0
# Converged also requires the ball NEAR CENTER: a ball stuck far away
# saturates the integral at the wide home-cal limit, where it goes flat
# — without the radius gate that fake flatness would fold pure windup
# into the persistent trim.
HOME_CAL_CONVERGE_MAX_RADIUS_MM = 15.0
HOME_CAL_TIMEOUT_S = 30.0

# =============================================================================
# AutoTune (2026-07-23 SysID rework)
# =============================================================================
# The step-test estimator and its 13 gate/inversion settings were
# deleted -- tuning is now a probe -> fit -> design pipeline
# (control/plant_id.py + control/gain_design.py; probe script and
# search bounds are module constants there, not user tunables). The
# old estimator was convicted on evidence: zero legs ever completed on
# the rig (its settle gates never opened), and it random-walked when
# run against a known simulated plant.
PD_AUTOTUNE_ENABLED = False
PD_AUTOTUNE_AUTO_APPLY = False
# Effective plant gain (mm/s^2 per deg). Updated by Apply after a fit;
# also feeds the path feedforward tilt divisor.
PD_AUTOTUNE_G_EFF: float = float(_OV.get("PD_AUTOTUNE_G_EFF", 171.0))
PD_AUTOTUNE_ABORT_RADIUS_MM = 70.0     # probe hard-abort radius
PD_AUTOTUNE_BALL_LOST_S = 1.0          # valid-frame gap that aborts a probe
AUTOTUNE_LOG_PATH: str = "autotune_session.log"

# =============================================================================
# Path following (control/patterns.py + control/path_follower.py)
# =============================================================================
# The follower advances a target point along a pattern path, tapering the
# advance rate with the ball's tracking error: full speed while the ball is
# within PATH_FULL_SPEED_RADIUS_MM of the target, linearly down to frozen at
# PATH_CAPTURE_RADIUS_MM. If the ball cannot keep up, the target waits.
# Default target speed: ~2/3 of the analytic 35 mm/s stall ceiling at
# default PD gains.
PATH_SPEED_MM_S = 30.0
PATH_SPEED_MIN_MM_S = 10.0
# MAX is above the default-gain ceiling on purpose — headroom for retuned
# gains; the adaptive taper keeps any setting safe.
PATH_SPEED_MAX_MM_S = 80.0
PATH_CAPTURE_RADIUS_MM = 20.0     # advance frozen at/above this tracking error
# Taper start; deadband so the healthy ~12 mm pursuit lag is not misread
# as a stall.
PATH_FULL_SPEED_RADIUS_MM = 10.0
# Radial clamp; just inside the 84.85 mm ArUco marker-corner radius.
PATH_MAX_RADIUS_MM = 85.0
PATH_POINT_SPACING_MM = 2.0       # uniform resample spacing

# =============================================================================
# Harmonic orbit (control/orbit.py)
# =============================================================================
# A SEPARATE mode from path following: a clock-driven circular reference
# with analytic feedforward tilt (rotating centripetal vector, phase-
# advanced by the actuation delay) plus a LEARNED per-phase correction
# table (iterative learning control — the plate-specific warp/drag
# harmonics dwarf the analytic term: bowl warp alone needs ~0.28 deg at
# r=50 vs 0.19 deg centripetal at v=40). Feedback is demoted to a trim
# role (p/d scaled; integral untouched for DC trim). Speed reuses the
# Path Speed slider (PATH_SPEED_* bounds).
ORBIT_RADIUS_MM = 50.0            # default reference radius (marker-safe)
ORBIT_RADIUS_MIN_MM = 30.0        # GUI spinbox bounds
ORBIT_RADIUS_MAX_MM = 70.0
ORBIT_SPINUP_S = 4.0              # omega 0 -> target ramp (also the r ramp window)
ORBIT_ENTRAIN_MIN_RADIUS_MM = 15.0  # entrain radius floor (atan2 stability)
# p/d scale while orbiting (integral untouched). Second rig session:
# raised 0.5 -> 1.0 — at half gains the P authority (0.036 deg/mm) was
# under the rig's stiction breakaway (0.3-0.5 deg = a +/-8-14 mm dead
# band), producing a 0.25-0.33 Hz stick-slip radial limit cycle with
# 5-10 mm amplitude at exactly the scaled-gain resonance. Full gains
# halve the sim ripple on the rig-like plant (3.2 -> 1.4 mm) with no
# instability; "feedback demoted to trim" is achieved by the ff+table
# carrying the drive, not by weakening the corrector.
ORBIT_FB_GAIN_SCALE = 1.0
ORBIT_FF_TILT_MAX_DEG = 1.5       # ff vector-norm cap (analytic + learned)
ORBIT_ILC_BINS = 24               # per-phase correction bins (15 deg/bin)
ORBIT_ILC_MU = 0.5                # learning rate (fraction of residual per bin-visit)
ORBIT_ILC_LEAK = 0.02             # per-LAP table leak (mis-learned corrections age out)
ORBIT_ILC_CLAMP_DEG = 1.2         # per-bin correction vector-norm clamp
#   Raised 0.8 -> 1.2 (second rig session): the rig's DC-plus-rotating
#   correction exceeds 0.8 — 11-13 of 24 bins sat PINNED at the old
#   clamp (starved), which is exactly the logged outward radius offset
#   (ball riding 10+ mm outside the ring). At 1.2 the sim table peaks
#   at ~1.18 with zero saturated bins and the radius error collapses
#   3.1 -> 0.6 mm.
# Gaussian write kernel width: each bin-transit update is spread over
# neighboring bins, BAND-LIMITING what the table can learn. Sim-caught:
# harmonics at n*omega above the scaled-gain resonance sqrt(g*kp_eff)
# have a sign-flipped closed-loop response, so point-writes PUMP them
# (n=3-4 grew to the clamp and the orbit diverged after 4 laps). At
# sigma=2.5 of 24 bins the fundamental learns at 0.81x speed while n=3
# is attenuated 0.15x — under the per-lap smoothing + leak damping
# even at mu=0.5 (fundamental contraction ~0.68/lap).
ORBIT_ILC_WRITE_SIGMA_BINS = 2.5
ORBIT_LEARN_GATE_MM = 30.0        # no learning above this tracking error
ORBIT_RECOVER_MM = 45.0           # error tripwire -> RECOVER (freeze + re-entrain)
ORBIT_RECOVER_FRAMES = 20         # consecutive frames above the tripwire
# Phase governor (first rig session, 2026-07-27): under the rig's real
# stiction (~0.3-0.5 deg equivalent — the sim assumed 0.06) the pure
# clock reference OUTRUNS the ball; the trailing error crossed the old
# 30 mm tripwire and the orbit churned recover->entrain forever (data:
# omega collapsing and re-ramping all session, radius ripple 5-10 mm,
# the table never got uninterrupted laps to learn; reproduced in sim at
# stiction 0.45). A WEAK phase-locked loop slews the reference phase
# toward the ball's actual angle: sustained lag is absorbed, but the
# bandwidth (~0.02 Hz in track) is far below jank frequencies, so —
# unlike carrot pacing — measurement jitter cannot couple into the
# reference. Entrain/recover use a STRONG lock (the reference stays
# glued to the ball until capture), and TRACK entry additionally
# requires the error under ORBIT_TRACK_ENTRY_ERR_MM so the integral
# never freezes on a bad state.
ORBIT_PHASE_GOV_TRACK_PER_S = 0.15    # rad/s of phase slew per rad of lag
ORBIT_PHASE_GOV_TRACK_CAP = 0.2       # cap as a fraction of omega
ORBIT_PHASE_GOV_ENTRAIN_PER_S = 1.5   # strong lock while entraining
ORBIT_TRACK_ENTRY_ERR_MM = 25.0       # capture gate for entrain -> track
# --- Cone mode (third rig session, 2026-07-27) ---
# Hudson's clarified intent: the platform's DOMINANT motion is the
# CONE itself — a pure open-loop rotating tilt (feedback OFF, integral
# frozen, no ball chasing) — and the ball falls into orbit because the
# physics says so. The plate's bowl warp acts as a central SPRING
# (omega_n = sqrt(g_eff*warp_c) ~ 0.97 rad/s at the rig-measured
# 0.0055 deg/mm — sim-caught: the naive gA/omega^2 formula missed it
# and the ball landed at 88 mm instead of 50), so the driven orbit is
# R = g_eff*A / (omega^2 - omega_n^2), ridden 180 deg out of phase
# with the tilt (driving above the warp resonance). Inverted: A is
# chosen ABOVE the rig's stiction breakaway (~0.3-0.5 deg) so the
# ball actually rolls, and the rate follows from the dialed radius:
#   omega = sqrt(g_eff * (warp_c + A/R))
# At A=0.6, r=50: omega=1.73 rad/s (0.28 Hz, ~3.6 s/rev, ~86 mm/s).
# ORBIT_CONE_ONLY=True makes the Harmonic Orbit button drive this
# mode; the closed-loop reference/ILC machinery stays available
# behind the flag.
ORBIT_CONE_ONLY = True
ORBIT_CONE_TILT_DEG = 0.6
ORBIT_CONE_WARP_C = 0.0055        # rig-measured bowl coefficient (deg/mm)

# =============================================================================
# Loop rates
# =============================================================================
CONTROL_LOOP_INTERVAL_MS = 20
ROUTINE_RETURN_HOME_S = 1.0             # ease-back-to-neutral duration after routines
VISION_LOOP_HZ = 120
VISUALIZER_HZ = 25
GUI_SNAPSHOT_HZ = 30

# =============================================================================
# Vision session recording
# =============================================================================
# When non-empty, the vision worker appends "t,x,y" lines (perf_counter
# seconds, ball x/y in mm) for every valid frame — input for
# tools/jitter_bench.py --csv replay. Empty string = disabled.
VISION_POSITION_LOG_PATH: str = ""

# =============================================================================
# Vision neutral-pose fallback (safety action on sustained ball loss)
# =============================================================================
VISION_MISS_NEUTRAL_AFTER_FRAMES = 20   # consecutive misses before neutral send
VISION_NEUTRAL_RESEND_S = 0.5           # min seconds between neutral resends

# =============================================================================
# GUI / logging
# =============================================================================
GUI_LOG_MAX_LINES = 500
LOG_EVERY_N = 30                        # log every N vision frames
DEBUG_PRINTS = True
