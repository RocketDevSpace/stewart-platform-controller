SERIAL_PORT = "COM4"
SERIAL_BAUD = 115200
CONTROL_LOOP_INTERVAL_MS = 20
VISION_LOOP_INTERVAL_MS = 20
VISUALIZER_UPDATE_SKIP = 5
SAFETY_LIMITS = {
    "max_angle": 180,
    "min_angle": 0,
    "odd_servo_max": 170,   # servos 0, 2, 4
    "even_servo_min": 10,   # servos 1, 3, 5
}
DEBUG_PRINTS = True
