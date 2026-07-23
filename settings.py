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
PD_I_ERR_DEADBAND_MM = 2.0
# Rest may only engage once the integral is flat (|dI/dt| EMA under
# this). Resting parks the output at trim + I with P and D dropped —
# resting on a still-converging integral is not an equilibrium and
# limit-cycles at ~0.2 Hz (sim-caught during the rework; the same
# structural cycle the old gated trim produced as the stale-trim
# "rocking"). At convergence the net rate is ~0 (integration balances
# leak), far under this threshold.
REST_I_RATE_MAX_DEG_S = 0.02

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
# PD autotune
# =============================================================================
PD_AUTOTUNE_ENABLED = False
PD_AUTOTUNE_AUTO_APPLY = False
PD_AUTOTUNE_SETTLE_RADIUS_MM = 6.0
PD_AUTOTUNE_SETTLE_SPEED_MM_S = 18.0
PD_AUTOTUNE_SETTLE_HOLD_S = 0.35
PD_AUTOTUNE_TIMEOUT_S = 6.0
PD_AUTOTUNE_MIN_TRIAL_S = 0.8
PD_AUTOTUNE_MIN_KP = 0.005
PD_AUTOTUNE_MAX_KP = 0.250
PD_AUTOTUNE_MIN_KD = 0.000
PD_AUTOTUNE_MAX_KD = 0.100
PD_AUTOTUNE_STEP_MM: float = 40.0          # step distance from center per leg
PD_AUTOTUNE_G_EFF: float = 171.0           # effective platform gravity (mm/s²/°)
PD_AUTOTUNE_TARGET_ZETA: float = 0.70      # desired closed-loop damping ratio
PD_AUTOTUNE_WAIT_SETTLE_RADIUS_MM: float = 20.0  # pre-leg settle radius
PD_AUTOTUNE_WAIT_SETTLE_SPEED_MM_S: float = 20.0  # pre-leg settle speed
PD_AUTOTUNE_WAIT_SETTLE_HOLD_S: float = 0.5      # pre-leg settle hold duration
PD_AUTOTUNE_MIN_OVERSHOOT_RATIO: float = 0.02    # below this → treat as overdamped
PD_AUTOTUNE_MIN_CROSS_S: float = 0.40           # ignore first_crossing faster than this
PD_AUTOTUNE_MAX_GAIN_DELTA_FRAC: float = 0.50   # max fractional change per trial
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
VISION_POSITION_LOG_PATH: str = "vision_positions.csv"  # TEMP: rig gate, do not commit

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
