import cv2
import numpy as np
import time
import math


class BallTracker:

    # =========================
    # INIT
    # =========================

    def __init__(self,
                 camera_index=0,
                 platform_size_mm=240.0,
                 warp_size_px=800,
                 aruco_size_mm=37.5,
                 vel_alpha=0.7,
                 show_debug=True):

        self.PLATFORM_SIZE_MM = platform_size_mm
        self.WARP_SIZE_PX = warp_size_px
        self.ARUCO_SIZE_MM = aruco_size_mm
        self.VEL_ALPHA = vel_alpha
        self.VEL_ARROW_SCALE = 0.15
        self.show_debug = show_debug

        # --- ArUco setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.CORNER_IDS = [0, 1, 2, 3]

        self.aruco_world_mm = {
            0: np.array([60.0, 60.0]),
            1: np.array([-60.0, 60.0]),
            2: np.array([-60.0, -60.0]),
            3: np.array([60.0, -60.0])
        }

        # --- Camera ---
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            raise RuntimeError("Camera failed to open.")

        # --- Velocity state ---
        self.prev_ball_mm = None
        self.prev_time = None
        self.vx_smooth = 0
        self.vy_smooth = 0

        # --- HSV window (optional) ---
        if self.show_debug:
            self._init_hsv_controls()

    # =========================
    # HSV TRACKBARS
    # =========================

    def _init_hsv_controls(self):
        def nothing(x): pass

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

    def get_marker_center(self, corners):
        pts = corners[0]
        return np.mean(pts, axis=0)

    def find_ball_center(self, mask):
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

    def mm_to_warp_px(self, x_mm, y_mm):
        px_per_mm = self.WARP_SIZE_PX / self.PLATFORM_SIZE_MM
        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2

        px = cx + x_mm * px_per_mm
        py = cy - y_mm * px_per_mm
        return [px, py]

    # =========================
    # MAIN UPDATE (NON-BLOCKING)
    # =========================

    def update(self):
        t0 = time.perf_counter()

        ret, frame = self.cap.read()
        t1 = time.perf_counter()
        if not ret:
            return None

        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        warped = None

        if ids is None:
            self._debug_show(frame, None, None)
            return None

        ids = ids.flatten()

        marker_centers_px = {}

        for i, marker_id in enumerate(ids):
            if marker_id in self.CORNER_IDS:
                marker_centers_px[marker_id] = self.get_marker_center(corners[i])
                
        t2 = time.perf_counter()

        if len(marker_centers_px) < 3:
            self._debug_show(frame, None, None)
            return None
        
        if len(marker_centers_px) == 3:
        
            missing_id = list(set(self.CORNER_IDS) - set(marker_centers_px.keys()))[0]
        
            # Find neighbors in square order
            order = self.CORNER_IDS
            idx = order.index(missing_id)
        
            prev_id = order[(idx - 1) % 4]
            next_id = order[(idx + 1) % 4]
        
            if prev_id in marker_centers_px and next_id in marker_centers_px:
        
                # Opposite corner is the one not prev/next/missing
                opposite_id = order[(idx + 2) % 4]
        
                if opposite_id in marker_centers_px:
        
                    A = marker_centers_px[prev_id]
                    B = marker_centers_px[opposite_id]
                    C = marker_centers_px[next_id]
        
                    # Parallelogram estimate
                    estimated = A + C - B
        
                    marker_centers_px[missing_id] = estimated
        
                else:
                    # Cannot reconstruct
                    self._debug_show(frame, None, None)
                    return None
            else:
                self._debug_show(frame, None, None)
                return None

        src_pts = []
        dst_pts = []

        for mid in self.CORNER_IDS:
            src_pts.append(marker_centers_px[mid])
            x_mm, y_mm = self.aruco_world_mm[mid]
            dst_pts.append(self.mm_to_warp_px(x_mm, y_mm))

        src_pts = np.array(src_pts, dtype=np.float32)
        dst_pts = np.array(dst_pts, dtype=np.float32)

        H, _ = cv2.findHomography(src_pts, dst_pts)

        if H is None:
            self._debug_show(frame, None, None)
            return None

        warped = cv2.warpPerspective(frame, H, (self.WARP_SIZE_PX, self.WARP_SIZE_PX))

        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        if self.show_debug:
            hmin = cv2.getTrackbarPos("H Min", "HSV Controls")
            hmax = cv2.getTrackbarPos("H Max", "HSV Controls")
            smin = cv2.getTrackbarPos("S Min", "HSV Controls")
            smax = cv2.getTrackbarPos("S Max", "HSV Controls")
            vmin = cv2.getTrackbarPos("V Min", "HSV Controls")
            vmax = cv2.getTrackbarPos("V Max", "HSV Controls")
        else:
            # default values if no debug
            hmin, hmax = 10, 28
            smin, smax = 83, 255
            vmin, vmax = 125, 255

        lower = np.array([hmin, smin, vmin])
        upper = np.array([hmax, smax, vmax])

        mask = cv2.inRange(hsv, lower, upper)
        ball = self.find_ball_center(mask)

        if ball is None:
            self._debug_show(frame, warped, mask)
            return None

        (bx, by), radius = ball

        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2

        mm_per_px = self.PLATFORM_SIZE_MM / self.WARP_SIZE_PX
        x_mm = (bx - cx) * mm_per_px
        y_mm = (cy - by) * mm_per_px

        current_time = time.time()
        
        t3 = time.perf_counter()

        if self.prev_ball_mm is None:
            vx = 0
            vy = 0
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                vx = 0
                vy = 0
            else:
                vx_raw = (x_mm - self.prev_ball_mm[0]) / dt
                vy_raw = (y_mm - self.prev_ball_mm[1]) / dt

                self.vx_smooth = self.VEL_ALPHA * self.vx_smooth + (1 - self.VEL_ALPHA) * vx_raw
                self.vy_smooth = self.VEL_ALPHA * self.vy_smooth + (1 - self.VEL_ALPHA) * vy_raw

                vx = self.vx_smooth
                vy = self.vy_smooth

        self.prev_ball_mm = (x_mm, y_mm)
        self.prev_time = current_time

        self._debug_show(frame, warped, mask, bx, by, vx, vy)
        
        print(f"Capture: {(t1-t0)*1000:.1f} ms")
        print(f"Aruco: {(t2-t1)*1000:.1f} ms")
        print(f"Ball: {(t3-t2)*1000:.1f} ms")

        return {
            "x_mm": x_mm,
            "y_mm": y_mm,
            "vx_mm_s": vx,
            "vy_mm_s": vy
        }

    # =========================
    # DEBUG DISPLAY
    # =========================

    def _debug_show(self, frame, warped, mask, bx=None, by=None, vx=0, vy=0):
        if not self.show_debug:
            return

        cv2.imshow("Camera View", frame)

        if warped is not None:
            if bx is not None:
                cv2.circle(warped, (bx, by), 5, (0, 0, 255), -1)

                arrow_px_x = int(bx + vx * self.VEL_ARROW_SCALE)
                arrow_px_y = int(by - vy * self.VEL_ARROW_SCALE)

                cv2.arrowedLine(warped,
                                (bx, by),
                                (arrow_px_x, arrow_px_y),
                                (255, 0, 0),
                                2)

            cv2.imshow("Warped Platform View", warped)

        if mask is not None:
            cv2.imshow("Ball Mask", mask)

        cv2.waitKey(1)

    # =========================
    # CLEANUP
    # =========================

    def release(self):
        if self.cap:
            self.cap.release()
        if self.show_debug:
            cv2.destroyAllWindows()