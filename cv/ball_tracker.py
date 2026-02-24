import cv2
import numpy as np
import time
import math

# =========================
# USER SETTINGS
# =========================

PLATFORM_SIZE_MM = 240.0
WARP_SIZE_PX = 800
CAMERA_INDEX = 0

ARUCO_SIZE_MM = 37.5

# Velocity smoothing factor (0 = none, 1 = infinite smoothing)
VEL_ALPHA = 0.7   # 0.7 = strong smoothing

# Velocity arrow scaling (mm/s → pixels)
VEL_ARROW_SCALE = 0.15

# =========================
# ARUCO SETUP
# =========================

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

CORNER_IDS = [0, 1, 2, 3]

# Marker centers in STEWART PLATFORM COORDINATES (mm)
aruco_world_mm = {
    0: np.array([ 60.0,  60.0]),   # front right
    1: np.array([-60.0,  60.0]),   # back right
    2: np.array([-60.0, -60.0]),   # back left
    3: np.array([ 60.0, -60.0])    # front left
}

# =========================
# HSV CONTROLS
# =========================

def nothing(x):
    pass

cv2.namedWindow("HSV Controls", cv2.WINDOW_NORMAL)
cv2.resizeWindow("HSV Controls", 500, 300)

cv2.createTrackbar("H Min", "HSV Controls", 10, 179, nothing)
cv2.createTrackbar("H Max", "HSV Controls", 28, 179, nothing)
cv2.createTrackbar("S Min", "HSV Controls", 83, 255, nothing)
cv2.createTrackbar("S Max", "HSV Controls", 255, 255, nothing)
cv2.createTrackbar("V Min", "HSV Controls", 125, 255, nothing)
cv2.createTrackbar("V Max", "HSV Controls", 255, 255, nothing)

# =========================
# HELPERS
# =========================

def get_marker_center(corners):
    pts = corners[0]
    return np.mean(pts, axis=0)

def find_ball_center(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < 150:
        return None

    (x, y), radius = cv2.minEnclosingCircle(largest)

    if radius < 4:
        return None

    return (int(x), int(y)), int(radius)

# Convert mm → warp pixel
def mm_to_warp_px(x_mm, y_mm):
    px_per_mm = WARP_SIZE_PX / PLATFORM_SIZE_MM
    cx = WARP_SIZE_PX // 2
    cy = WARP_SIZE_PX // 2

    px = cx + x_mm * px_per_mm
    py = cy - y_mm * px_per_mm  # minus because image Y is downward

    return [px, py]

# =========================
# CAMERA
# =========================

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

prev_ball_mm = None
prev_time = None

vx_smooth = 0
vy_smooth = 0

print("Press Q to quit.")

# =========================
# MAIN LOOP
# =========================

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Restore horizontal flip (REQUIRED for your camera)
    frame = cv2.flip(frame, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    display = frame.copy()
    warped = None
    ball_mask = None
    
    print("Detected IDs:", ids)

    if ids is not None:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(display, corners, ids)

        marker_centers_px = {}

        for i, marker_id in enumerate(ids):
            if marker_id in CORNER_IDS:
                marker_centers_px[marker_id] = get_marker_center(corners[i])
                

        if len(marker_centers_px) == 4:

            src_pts = []
            dst_pts = []

            for mid in CORNER_IDS:
                src_pts.append(marker_centers_px[mid])

                x_mm, y_mm = aruco_world_mm[mid]
                dst_pts.append(mm_to_warp_px(x_mm, y_mm))

            src_pts = np.array(src_pts, dtype=np.float32)
            dst_pts = np.array(dst_pts, dtype=np.float32)

            H, _ = cv2.findHomography(src_pts, dst_pts)

            if H is not None:

                warped = cv2.warpPerspective(frame, H, (WARP_SIZE_PX, WARP_SIZE_PX))

                hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

                hmin = cv2.getTrackbarPos("H Min", "HSV Controls")
                hmax = cv2.getTrackbarPos("H Max", "HSV Controls")
                smin = cv2.getTrackbarPos("S Min", "HSV Controls")
                smax = cv2.getTrackbarPos("S Max", "HSV Controls")
                vmin = cv2.getTrackbarPos("V Min", "HSV Controls")
                vmax = cv2.getTrackbarPos("V Max", "HSV Controls")

                lower = np.array([hmin, smin, vmin])
                upper = np.array([hmax, smax, vmax])

                ball_mask = cv2.inRange(hsv, lower, upper)

                ball = find_ball_center(ball_mask)

                cx = WARP_SIZE_PX // 2
                cy = WARP_SIZE_PX // 2

                cv2.line(warped, (cx, 0), (cx, WARP_SIZE_PX), (255,255,255), 1)
                cv2.line(warped, (0, cy), (WARP_SIZE_PX, cy), (255,255,255), 1)

                if ball is not None:

                    (bx, by), radius = ball
                    cv2.circle(warped, (bx, by), radius, (0,255,0), 2)
                    cv2.circle(warped, (bx, by), 4, (0,0,255), -1)

                    mm_per_px = PLATFORM_SIZE_MM / WARP_SIZE_PX

                    x_mm = (bx - cx) * mm_per_px
                    y_mm = (cy - by) * mm_per_px

                    current_time = time.time()

                    if prev_ball_mm is not None:
                        dt = current_time - prev_time

                        vx_raw = (x_mm - prev_ball_mm[0]) / dt
                        vy_raw = (y_mm - prev_ball_mm[1]) / dt

                        # smoothing
                        vx_smooth = VEL_ALPHA * vx_smooth + (1 - VEL_ALPHA) * vx_raw
                        vy_smooth = VEL_ALPHA * vy_smooth + (1 - VEL_ALPHA) * vy_raw

                        speed = math.sqrt(vx_smooth**2 + vy_smooth**2)
                        angle = math.degrees(math.atan2(vy_smooth, vx_smooth))

                        # draw velocity arrow
                        arrow_px_x = int(bx + vx_smooth * VEL_ARROW_SCALE)
                        arrow_px_y = int(by - vy_smooth * VEL_ARROW_SCALE)

                        cv2.arrowedLine(
                            warped,
                            (bx, by),
                            (arrow_px_x, arrow_px_y),
                            (255,0,0),
                            3
                        )

                        cv2.putText(warped,
                            f"X={x_mm:.1f}mm Y={y_mm:.1f}mm",
                            (20,40),
                            cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)

                        cv2.putText(warped,
                            f"Vx={vx_raw:.1f} Vy={vy_raw:.1f} (raw)",
                            (20,70),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(200,200,200),2)

                        cv2.putText(warped,
                            f"Vx={vx_smooth:.1f} Vy={vy_smooth:.1f} (smooth)",
                            (20,100),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)

                        cv2.putText(warped,
                            f"Speed={speed:.1f} mm/s  Angle={angle:.1f} deg",
                            (20,130),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)

                    prev_ball_mm = (x_mm, y_mm)
                    prev_time = current_time

    cv2.imshow("Camera View", display)

    if warped is not None:
        cv2.imshow("Warped Platform View", warped)

    if ball_mask is not None:
        cv2.imshow("Ball Mask", ball_mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()