# =============================================================================
# Serial
# =============================================================================
SERIAL_PORT = "COM4"
SERIAL_BAUD = 115200

# =============================================================================
# Safety limits
# =============================================================================
MAX_TILT_DEG = 8.0                      # maximum platform tilt in vision mode

SAFETY_LIMITS = {
    "max_angle": 180,       # global ceiling, all servos (enforced in core/safety.py)
    "min_angle": 0,         # global floor, all servos (enforced in core/safety.py)
    "odd_servo_max": 170,   # extra ceiling for INDICES 0, 2, 4 (mirrored mount)
    "even_servo_min": 10,   # extra floor for INDICES 1, 3, 5 (mirrored mount)
}

# =============================================================================
# Camera
# =============================================================================
CAMERA_INDEX = 1                        # 0 = integrated, 1 = USB camera
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
TRACKER_ARUCO_REDETECT_EVERY_N = 10
TRACKER_WARP_GRAY_CACHE_N: int = 5   # recompute warp gray mean every N frames
TRACKER_MIN_RADIUS_PX = 4.0
TRACKER_MIN_CONTOUR_AREA = 150.0
TRACKER_ARUCO_CENTER_FILTER_ALPHA = 0.70
TRACKER_MAX_ARUCO_HOLD_FRAMES = 3
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
TRACKER_HSV_H_MIN = 10
TRACKER_HSV_H_MAX = 28
TRACKER_HSV_S_MIN = 83
TRACKER_HSV_S_MAX = 255
TRACKER_HSV_V_MIN = 125
TRACKER_HSV_V_MAX = 255

# low-pass weight on raw velocity (0=frozen, 1=raw); ~6 Hz cutoff at 30 fps; tunable
BALL_VEL_FILTER_ALPHA: float = 0.55

# =============================================================================
# PD controller
# =============================================================================
PD_DEFAULT_KP = 0.045
PD_DEFAULT_KD = 0.022
BALL_TARGET_DEFAULT_X_MM = 0.0
BALL_TARGET_DEFAULT_Y_MM = 0.0

# =============================================================================
# Manual trim / Auto-trim
# =============================================================================
MANUAL_ROLL_TRIM_DEG = -0.8
MANUAL_PITCH_TRIM_DEG = 4.6

AUTO_TRIM_ENABLED = False
AUTO_TRIM_KI_DEG_PER_MM_S = 0.008
AUTO_TRIM_MAX_DEG = 6.0
AUTO_TRIM_HOME_CAL_MAX_DEG = 8.0
AUTO_TRIM_SETTLE_SPEED_MM_S = 25.0
AUTO_TRIM_SETTLE_RADIUS_MM = 35.0
AUTO_TRIM_SETTLE_HOLD_S = 0.60
AUTO_TRIM_STEP_LIMIT_DEG = 0.08
AUTO_TRIM_TARGET_HOLD_S = 0.60
AUTO_TRIM_ERROR_LPF_ALPHA = 0.85
AUTO_TRIM_SETTLE_SPEED_LPF_ALPHA = 0.75
AUTO_TRIM_SETTLE_RADIUS_LPF_ALPHA = 0.75

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
# Loop rates
# =============================================================================
CONTROL_LOOP_INTERVAL_MS = 20
VISION_LOOP_HZ = 120
VISUALIZER_HZ = 25
GUI_SNAPSHOT_HZ = 30

# =============================================================================
# GUI / logging
# =============================================================================
GUI_LOG_MAX_LINES = 500
TIMING_PLOT_POINTS = 300
LOG_EVERY_N = 30                        # log every N vision frames
DEBUG_PRINTS = True
