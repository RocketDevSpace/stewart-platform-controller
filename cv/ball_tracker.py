import cv2
import numpy as np
import time


# =========================
# USER SETTINGS
# =========================

PLATFORM_SIZE_MM = 160
WARP_SIZE_PX = 700   # slightly higher res = better ball tracking

CAMERA_INDEX = 0

# =========================
# ARUCO SETUP
# =========================

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

CORNER_IDS = [0, 1, 2, 3]  # clockwise order

dst_pts = np.array([
    [0, 0],
    [WARP_SIZE_PX - 1, 0],
    [WARP_SIZE_PX - 1, WARP_SIZE_PX - 1],
    [0, WARP_SIZE_PX - 1]
], dtype=np.float32)

# =========================
# HSV SLIDER WINDOW
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

def compute_missing_corner(points_dict):
    """
    points_dict contains {id: point} for some of the IDs 0,1,2,3.
    If exactly one is missing, compute it using square/parallelogram rule.
    """

    missing = [i for i in CORNER_IDS if i not in points_dict]

    if len(missing) != 1:
        return points_dict  # nothing to do

    m = missing[0]

    # parallelogram logic for a square with corners:
    # 0 -> 1 -> 2 -> 3 clockwise
    # 0 opposite 2, 1 opposite 3

    if m == 0:
        # P0 = P1 + P3 - P2
        points_dict[0] = points_dict[1] + points_dict[3] - points_dict[2]
    elif m == 1:
        # P1 = P0 + P2 - P3
        points_dict[1] = points_dict[0] + points_dict[2] - points_dict[3]
    elif m == 2:
        # P2 = P1 + P3 - P0
        points_dict[2] = points_dict[1] + points_dict[3] - points_dict[0]
    elif m == 3:
        # P3 = P0 + P2 - P1
        points_dict[3] = points_dict[0] + points_dict[2] - points_dict[1]

    return points_dict

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

# =========================
# CAMERA SETUP
# =========================

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
last_time = time.time()
frame_count = 0
fps = 0


print("Press Q to quit.")

# =========================
# MAIN LOOP
# =========================

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Fix mirrored camera feed
    frame = cv2.flip(frame, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)

    display = frame.copy()

    warped = None
    ball_mask = None

    if ids is not None:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(display, corners, ids)

        marker_centers = {}

        for i, marker_id in enumerate(ids):
            if marker_id in CORNER_IDS:
                marker_centers[marker_id] = get_marker_center(corners[i])

        # If we have at least 3 markers, we can attempt warp
        if len(marker_centers) >= 3:

            # If exactly 3 markers, compute missing corner
            if len(marker_centers) == 3:
                marker_centers = compute_missing_corner(marker_centers)

            # Now if we have 4 (real or computed), warp
            if all(mid in marker_centers for mid in CORNER_IDS):

                src_pts = np.array([
                    marker_centers[0],
                    marker_centers[1],
                    marker_centers[2],
                    marker_centers[3]
                ], dtype=np.float32)

                H, _ = cv2.findHomography(src_pts, dst_pts)

                if H is not None:
                    warped = cv2.warpPerspective(frame, H, (WARP_SIZE_PX, WARP_SIZE_PX))

                    # Get HSV slider values
                    hmin = cv2.getTrackbarPos("H Min", "HSV Controls")
                    hmax = cv2.getTrackbarPos("H Max", "HSV Controls")
                    smin = cv2.getTrackbarPos("S Min", "HSV Controls")
                    smax = cv2.getTrackbarPos("S Max", "HSV Controls")
                    vmin = cv2.getTrackbarPos("V Min", "HSV Controls")
                    vmax = cv2.getTrackbarPos("V Max", "HSV Controls")

                    hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

                    lower = np.array([hmin, smin, vmin])
                    upper = np.array([hmax, smax, vmax])

                    ball_mask = cv2.inRange(hsv, lower, upper)

                    ball = find_ball_center(ball_mask)

                    # Draw center crosshair
                    cx = WARP_SIZE_PX // 2
                    cy = WARP_SIZE_PX // 2
                    cv2.line(warped, (cx, 0), (cx, WARP_SIZE_PX), (255, 255, 255), 1)
                    cv2.line(warped, (0, cy), (WARP_SIZE_PX, cy), (255, 255, 255), 1)

                    if ball is not None:
                        (bx, by), radius = ball

                        cv2.circle(warped, (bx, by), radius, (0, 255, 0), 2)
                        cv2.circle(warped, (bx, by), 4, (0, 0, 255), -1)

                        mm_per_px = PLATFORM_SIZE_MM / WARP_SIZE_PX

                        ball_x_mm = (bx - cx) * mm_per_px
                        ball_y_mm = (by - cy) * mm_per_px

                        cv2.putText(
                            warped,
                            f"Ball X={ball_x_mm:.1f} mm  Y={ball_y_mm:.1f} mm",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (255, 255, 255),
                            2
                        )

                        print(f"Ball position: X={ball_x_mm:.1f} mm, Y={ball_y_mm:.1f} mm")

                    # Show marker count info
                    cv2.putText(
                        warped,
                        f"Markers visible: {len([m for m in ids if m in CORNER_IDS])}/4",
                        (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2
                    )
                    
    frame_count += 1    
    now = time.time()
    
    if now - last_time >= 1.0:
        fps = frame_count / (now - last_time)
        frame_count = 0
        last_time = now
        print(f"CV FPS: {fps:.1f}")


    # Display windows
    cv2.putText(display, f"FPS: {fps:.1f}", (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

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
