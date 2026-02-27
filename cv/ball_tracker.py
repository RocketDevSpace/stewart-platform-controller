import cv2
import numpy as np
import time

from stewart_control.config import (
    CAMERA_BUFFER_SIZE,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    DEBUG_LEVEL,
    LOG_EVERY_N,
    TRACKER_ARUCO_DETECT_SCALE,
    TRACKER_ARUCO_REDETECT_EVERY_N,
    TRACKER_WARP_SIZE_PX,
)


class BallTracker:
    def __init__(
        self,
        camera_index=0,
        platform_size_mm=240.0,
        warp_size_px=TRACKER_WARP_SIZE_PX,
        vel_alpha=0.7,
        pd_kp=0.005,
        pd_kd=0.010,
        aruco_detect_scale=TRACKER_ARUCO_DETECT_SCALE,
        aruco_redetect_every_n=TRACKER_ARUCO_REDETECT_EVERY_N,
        debug_level=DEBUG_LEVEL,
        log_every_n=LOG_EVERY_N,
    ):
        self.PLATFORM_SIZE_MM = platform_size_mm
        self.WARP_SIZE_PX = warp_size_px
        self.VEL_ALPHA = vel_alpha

        self.VEL_ARROW_SCALE = 0.15
        self.POS_ARROW_SCALE = 0.20
        self.PD_ARROW_SCALE = 40.0

        self.pd_kp = float(pd_kp)
        self.pd_kd = float(pd_kd)
        self.aruco_detect_scale = float(aruco_detect_scale)
        self.aruco_redetect_every_n = max(1, int(aruco_redetect_every_n))
        self.debug_level = debug_level
        self.log_every_n = max(1, int(log_every_n))
        self._log_counter = 0
        self._frame_counter = 0
        self._last_H = None

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.CORNER_IDS = [0, 1, 2, 3]
        self.aruco_world_mm = {
            0: np.array([60.0, 60.0]),
            1: np.array([-60.0, 60.0]),
            2: np.array([-60.0, -60.0]),
            3: np.array([60.0, -60.0]),
        }

        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        if not self.cap.isOpened():
            raise RuntimeError("Camera failed to open.")

        self.prev_ball_mm = None
        self.prev_time = None
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0
        self.hsv_lower = np.array([10, 83, 125], dtype=np.uint8)
        self.hsv_upper = np.array([28, 255, 255], dtype=np.uint8)

    def set_pd_gains(self, kp, kd):
        self.pd_kp = float(kp)
        self.pd_kd = float(kd)

    def set_hsv_thresholds(self, hmin, hmax, smin, smax, vmin, vmax):
        self.hsv_lower = np.array([int(hmin), int(smin), int(vmin)], dtype=np.uint8)
        self.hsv_upper = np.array([int(hmax), int(smax), int(vmax)], dtype=np.uint8)

    def mm_to_warp_px(self, x_mm, y_mm):
        px_per_mm = self.WARP_SIZE_PX / self.PLATFORM_SIZE_MM
        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2
        px = cx + x_mm * px_per_mm
        py = cy - y_mm * px_per_mm
        return [px, py]

    @staticmethod
    def get_marker_center(corners):
        pts = corners[0]
        return np.mean(pts, axis=0)

    @staticmethod
    def find_ball_center(mask):
        kernel = np.ones((3, 3), np.uint8)
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
        return (int(x), int(y)), int(radius), mask

    def _draw_vectors(self, warped, bx, by, vx, vy, x_mm, y_mm):
        center_px = (self.WARP_SIZE_PX // 2, self.WARP_SIZE_PX // 2)
        cv2.drawMarker(
            warped, center_px, (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2
        )
        cv2.circle(warped, (bx, by), 5, (0, 0, 255), -1)

        vel_end = (int(bx + vx * self.VEL_ARROW_SCALE), int(by - vy * self.VEL_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), vel_end, (255, 0, 0), 2)

        pos_vec_x = -x_mm
        pos_vec_y = -y_mm
        pos_end = (int(bx + pos_vec_x * self.POS_ARROW_SCALE), int(by - pos_vec_y * self.POS_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), pos_end, (0, 255, 255), 2)

        pd_x = self.pd_kp * pos_vec_x + self.pd_kd * (-vx)
        pd_y = self.pd_kp * pos_vec_y + self.pd_kd * (-vy)
        pd_end = (int(bx + pd_x * self.PD_ARROW_SCALE), int(by - pd_y * self.PD_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), pd_end, (0, 255, 0), 2)

        cv2.putText(warped, "Center", (center_px[0] + 8, center_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(warped, "Vel", (vel_end[0] + 4, vel_end[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(warped, "Pos->Center", (pos_end[0] + 4, pos_end[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(warped, "PD", (pd_end[0] + 4, pd_end[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

    def update(self, return_debug_frames=False):
        t0 = time.perf_counter()
        ret, frame = self.cap.read()
        t1 = time.perf_counter()
        if not ret:
            return None

        self._frame_counter += 1
        frame = cv2.flip(frame, 1)
        H = None
        warped = None
        mask = None

        use_cached_h = (self._last_H is not None) and (self._frame_counter % self.aruco_redetect_every_n != 0)
        if use_cached_h:
            H = self._last_H
            t2 = t1
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if self.aruco_detect_scale < 0.99:
                gray_detect = cv2.resize(
                    gray,
                    None,
                    fx=self.aruco_detect_scale,
                    fy=self.aruco_detect_scale,
                    interpolation=cv2.INTER_AREA,
                )
                scale_inv = 1.0 / self.aruco_detect_scale
            else:
                gray_detect = gray
                scale_inv = 1.0

            corners, ids, _ = self.detector.detectMarkers(gray_detect)
            if ids is None:
                return None
            ids = ids.flatten()

            marker_centers_px = {}
            for i, marker_id in enumerate(ids):
                if marker_id in self.CORNER_IDS:
                    marker_centers_px[marker_id] = self.get_marker_center(corners[i]) * scale_inv
            t2 = time.perf_counter()

            if len(marker_centers_px) < 3:
                return None
            if len(marker_centers_px) == 3:
                missing_id = list(set(self.CORNER_IDS) - set(marker_centers_px.keys()))[0]
                order = self.CORNER_IDS
                idx = order.index(missing_id)
                prev_id = order[(idx - 1) % 4]
                next_id = order[(idx + 1) % 4]
                opposite_id = order[(idx + 2) % 4]
                if prev_id not in marker_centers_px or next_id not in marker_centers_px or opposite_id not in marker_centers_px:
                    return None
                A = marker_centers_px[prev_id]
                B = marker_centers_px[opposite_id]
                C = marker_centers_px[next_id]
                marker_centers_px[missing_id] = A + C - B

            src_pts = []
            dst_pts = []
            for mid in self.CORNER_IDS:
                src_pts.append(marker_centers_px[mid])
                x_mm, y_mm = self.aruco_world_mm[mid]
                dst_pts.append(self.mm_to_warp_px(x_mm, y_mm))
            src_pts = np.array(src_pts, dtype=np.float32)
            dst_pts = np.array(dst_pts, dtype=np.float32)

            if len(src_pts) == 4:
                H = cv2.getPerspectiveTransform(src_pts, dst_pts)
            else:
                H, _ = cv2.findHomography(src_pts, dst_pts)
            if H is None:
                return None
            self._last_H = H

        warped = cv2.warpPerspective(frame, H, (self.WARP_SIZE_PX, self.WARP_SIZE_PX), flags=cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        ball = self.find_ball_center(mask)
        if ball is None:
            return None
        (bx, by), _, mask = ball

        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2
        mm_per_px = self.PLATFORM_SIZE_MM / self.WARP_SIZE_PX
        x_mm = (bx - cx) * mm_per_px
        y_mm = (cy - by) * mm_per_px
        current_time = time.time()
        t3 = time.perf_counter()

        if self.prev_ball_mm is None:
            vx = 0.0
            vy = 0.0
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                vx = 0.0
                vy = 0.0
            else:
                vx_raw = (x_mm - self.prev_ball_mm[0]) / dt
                vy_raw = (y_mm - self.prev_ball_mm[1]) / dt
                self.vx_smooth = self.VEL_ALPHA * self.vx_smooth + (1 - self.VEL_ALPHA) * vx_raw
                self.vy_smooth = self.VEL_ALPHA * self.vy_smooth + (1 - self.VEL_ALPHA) * vy_raw
                vx = self.vx_smooth
                vy = self.vy_smooth

        self.prev_ball_mm = (x_mm, y_mm)
        self.prev_time = current_time
        if return_debug_frames:
            self._draw_vectors(warped, bx, by, vx, vy, x_mm, y_mm)

        self._log_counter += 1
        if self.debug_level >= 2 and (self._log_counter % self.log_every_n == 0):
            print(f"Capture: {(t1 - t0) * 1000:.1f} ms")
            print(f"Aruco: {(t2 - t1) * 1000:.1f} ms")
            print(f"Ball: {(t3 - t2) * 1000:.1f} ms")

        result = {
            "x_mm": x_mm,
            "y_mm": y_mm,
            "vx_mm_s": vx,
            "vy_mm_s": vy,
        }
        if return_debug_frames:
            result["camera_bgr"] = frame
            result["warped_bgr"] = warped
            result["mask_gray"] = mask
        return result

    def release(self):
        if self.cap:
            self.cap.release()
