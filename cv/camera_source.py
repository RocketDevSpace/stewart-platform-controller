"""
cv/camera_source.py

Camera lifecycle, backend probing, the background capture thread, runtime
exposure policy, and software-gain estimation — extracted from BallTracker
(M11) so detection logic is camera-free and testable.

Contract:
- __init__ does NO I/O (cheap, non-blocking by design).
- open() performs the blocking backend probe, configures the camera, and
  starts the capture thread. Call it off the GUI thread (the vision worker
  calls it from its own QThread).
- read_latest() returns a COPY of the newest frame, copied UNDER the
  capture lock: the writer only mutates the write slot under the same
  lock discipline, so a reader can never observe a torn frame. (The old
  copy-outside-the-lock scheme could tear when the consumer stalled for
  longer than one capture period.)
- Exposure probes publish their settle frames through the same path as
  the capture loop, so consumers keep receiving frames during a probe
  (the old code starved the pipeline ~1 s per probe).
- close() only releases the capture handle after the capture thread has
  actually exited — releasing concurrently with a blocked read() is
  undefined behavior in OpenCV backends (segfault vector).
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from settings import (
    CAMERA_AUTO_EXPOSURE,
    CAMERA_BUFFER_SIZE,
    CAMERA_EXPOSURE,
    CAMERA_FORCE_BACKEND,
    CAMERA_HEIGHT,
    CAMERA_RUNTIME_ADAPTIVE,
    CAMERA_RUNTIME_CHECK_S,
    CAMERA_RUNTIME_MAX_PERIOD_MS,
    CAMERA_RUNTIME_MIN_GRAY,
    CAMERA_RUNTIME_SOFT_GAIN_MAX,
    CAMERA_RUNTIME_TARGET_GRAY,
    CAMERA_RUNTIME_WARMUP_S,
    CAMERA_TARGET_FPS,
    CAMERA_WIDTH,
    DEBUG_PRINTS,
)

logger = logging.getLogger(__name__)


@dataclass
class CaptureStats:
    """Snapshot of capture health for telemetry / camera_ready payloads."""

    backend: str
    period_ms: float
    gray_mean: float
    fail_streak: int
    slow_streak: int
    policy_mode: str          # "bootstrap_auto" | "manual"
    software_gain: float


class CameraSource:
    """Owns the capture device and its background thread."""

    def __init__(
        self,
        camera_index: int,
        cap_factory: Callable[..., Any] = cv2.VideoCapture,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        self._camera_index = int(camera_index)
        self._cap_factory = cap_factory
        self._clock = clock

        self.cap: Any = None
        self.backend_name = "unopened"
        self.measured_period_ms = 0.0

        self._capture_running = False
        self._capture_thread: threading.Thread | None = None
        self._capture_lock = threading.Lock()
        self._frame_bufs = [
            np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8),
            np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8),
        ]
        self._buf_write_slot = 0
        self._buf_ready = False
        self._latest_frame_ts = 0.0
        self._capture_period_ms = 0.0
        self._capture_gray_mean = 0.0
        self._prev_capture_ts = 0.0
        self._slow_capture_streak = 0
        self._capture_fail_streak = 0

        # Runtime exposure policy state
        self._software_gain = 1.0
        self._policy_mode = "bootstrap_auto"
        self._manual_exposure: float | None = None
        self._recovery_disabled = False
        self._last_policy_ts = 0.0
        self._last_mode_change_ts = 0.0
        self._capture_start_ts = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> CaptureStats:
        """Blocking: probe backends, pick the fastest, start the capture
        thread. Raises RuntimeError if no camera opens."""
        if self._capture_running:
            return self.stats()

        self.cap = self._open_best_camera(self._camera_index)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("Camera failed to open.")

        self._capture_start_ts = self._clock()
        self._capture_running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="camera-capture"
        )
        self._capture_thread.start()
        return self.stats()

    def close(self) -> None:
        self._capture_running = False
        thread = self._capture_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
            if thread.is_alive():
                # A blocked cap.read() never returned. Releasing the handle
                # under it is UB/segfault territory — leak it instead (the
                # thread is a daemon; the OS reclaims at exit).
                logger.warning(
                    "capture thread did not exit; leaving capture handle open"
                )
                return
        self._capture_thread = None
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                logger.exception("cap.release() failed")
            self.cap = None

    # ------------------------------------------------------------------
    # Consumer API
    # ------------------------------------------------------------------

    def latest_frame_ts(self) -> float:
        with self._capture_lock:
            return self._latest_frame_ts

    def has_new_frame(self, since_ts: float) -> bool:
        with self._capture_lock:
            return self._buf_ready and self._latest_frame_ts > since_ts

    def read_latest(self) -> tuple[np.ndarray, float] | None:
        """Copy of the newest frame + its capture timestamp, or None.

        The copy happens UNDER the lock: the writer mutates pixel data only
        for the current write slot and swaps slots under the same lock, so
        a torn read is impossible by construction.
        """
        with self._capture_lock:
            if not self._buf_ready:
                return None
            read_slot = 1 - self._buf_write_slot
            return self._frame_bufs[read_slot].copy(), self._latest_frame_ts

    def stats(self) -> CaptureStats:
        with self._capture_lock:
            return CaptureStats(
                backend=self.backend_name,
                period_ms=float(
                    self._capture_period_ms or self.measured_period_ms
                ),
                gray_mean=float(self._capture_gray_mean),
                fail_streak=int(self._capture_fail_streak),
                slow_streak=int(self._slow_capture_streak),
                policy_mode=self._policy_mode,
                software_gain=float(self._software_gain),
            )

    # ------------------------------------------------------------------
    # Camera selection / configuration
    # ------------------------------------------------------------------

    @staticmethod
    def _backend_label(backend: int) -> str:
        if backend == cv2.CAP_DSHOW:
            return "DSHOW"
        if backend == cv2.CAP_MSMF:
            return "MSMF"
        if backend == cv2.CAP_ANY:
            return "ANY"
        return str(backend)

    def _apply_manual_exposure(
        self, exposure_value: float, settle_reads: int = 0
    ) -> None:
        cap = self.cap
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        except Exception:
            pass
        try:
            cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_value))
            cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_value))
        except Exception:
            pass
        for _ in range(max(0, int(settle_reads))):
            try:
                ret, frame = cap.read()
                # Publish settle frames so consumers aren't starved.
                if ret and self._capture_running:
                    self._publish(frame, self._clock())
            except Exception:
                break

    def _configure_camera(self, cap: Any, force_auto: bool = False) -> None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FPS, CAMERA_TARGET_FPS)
        use_auto = bool(CAMERA_AUTO_EXPOSURE) or bool(force_auto)
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if use_auto else 0.25)
        except Exception:
            pass
        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        except Exception:
            pass

    def _measure_camera_stats(
        self, cap: Any, samples: int = 8, warmup: int = 0, publish: bool = False
    ) -> tuple[float, float]:
        t_prev = None
        periods = []
        gray_levels = []
        for _ in range(max(0, int(warmup))):
            try:
                cap.read()
            except Exception:
                pass
        for _ in range(samples):
            ret, frame = cap.read()
            if not ret:
                continue
            t_now = self._clock()
            if publish and self._capture_running:
                self._publish(frame, t_now)
            if t_prev is not None:
                periods.append((t_now - t_prev) * 1000.0)
            t_prev = t_now
            try:
                gray_levels.append(
                    float(np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
                )
            except Exception:
                pass
        if not periods:
            return 1e9, 0.0
        gray_mean = (
            float(sum(gray_levels) / len(gray_levels)) if gray_levels else 0.0
        )
        period_ms = float(np.median(np.array(periods, dtype=float)))
        return period_ms, gray_mean

    def _open_best_camera(self, camera_index: int) -> Any:
        forced = CAMERA_FORCE_BACKEND.upper()
        if forced == "DSHOW":
            backends = [cv2.CAP_DSHOW]
        elif forced == "MSMF":
            backends = [cv2.CAP_MSMF]
        elif forced == "ANY":
            backends = [cv2.CAP_ANY]
        else:
            backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]

        best_cap = None
        best_period = 1e9
        best_backend: int | None = None

        for backend in backends:
            cap = self._cap_factory(camera_index, backend)
            if not cap.isOpened():
                cap.release()
                continue
            self._configure_camera(cap, force_auto=True)
            period, gray_mean = self._measure_camera_stats(
                cap, samples=8, warmup=2
            )
            if DEBUG_PRINTS:
                logger.info(
                    "backend=%s period~%.1fms gray=%.1f",
                    self._backend_label(backend), period, gray_mean,
                )
            if period < best_period:
                if best_cap is not None:
                    best_cap.release()
                best_cap = cap
                best_period = period
                best_backend = backend
            else:
                cap.release()

        if best_cap is not None and best_backend is not None:
            self.backend_name = self._backend_label(best_backend)
            self.measured_period_ms = float(best_period)
            self._last_mode_change_ts = self._clock()
            logger.info(
                "selected backend=%s period~%.1fms",
                self.backend_name, best_period,
            )
        return best_cap

    # ------------------------------------------------------------------
    # Runtime exposure policy + software gain
    # ------------------------------------------------------------------

    def _probe_manual_exposure_candidates(
        self, candidates: list[float]
    ) -> tuple[float, float, float] | None:
        probe_results = []
        for exp in candidates:
            self._apply_manual_exposure(float(exp), settle_reads=2)
            p_ms, g_mean = self._measure_camera_stats(
                self.cap, samples=4, warmup=1, publish=True
            )
            probe_results.append((float(p_ms), float(g_mean), float(exp)))

        if not probe_results:
            return None

        max_period = float(CAMERA_RUNTIME_MAX_PERIOD_MS)
        min_gray = float(CAMERA_RUNTIME_MIN_GRAY)
        acceptable = [
            r for r in probe_results
            if r[0] <= (max_period + 2.0) and r[1] >= min_gray
        ]
        if acceptable:
            fastest = min(r[0] for r in acceptable)
            near_fast = [r for r in acceptable if r[0] <= (fastest + 6.0)]
            best = max(near_fast, key=lambda r: r[1])
        else:
            bright_enough = [
                r for r in probe_results if r[1] >= max(15.0, min_gray - 10.0)
            ]
            if bright_enough:
                best = min(bright_enough, key=lambda r: r[0])
            else:
                best = min(probe_results, key=lambda r: r[0])

        self._apply_manual_exposure(best[2], settle_reads=2)
        self._manual_exposure = best[2]
        self._policy_mode = "manual"
        self._last_mode_change_ts = self._clock()
        return best

    def _evaluate_runtime_policy(self, now: float) -> None:
        if not CAMERA_RUNTIME_ADAPTIVE or self._recovery_disabled:
            return
        if (now - self._capture_start_ts) < float(CAMERA_RUNTIME_WARMUP_S):
            return
        if (now - self._last_policy_ts) < float(CAMERA_RUNTIME_CHECK_S):
            return
        self._last_policy_ts = now

        period_ms = (
            float(self._capture_period_ms)
            if self._capture_period_ms > 0
            else float(self.measured_period_ms or 1e9)
        )
        gray_mean = (
            float(self._capture_gray_mean) if self._capture_gray_mean > 0 else 0.0
        )
        max_period = float(CAMERA_RUNTIME_MAX_PERIOD_MS)
        min_gray = float(CAMERA_RUNTIME_MIN_GRAY)

        if (
            period_ms > max_period + 2.0
            and (now - self._last_mode_change_ts) > 2.5
        ):
            candidates = list({float(CAMERA_EXPOSURE), -4.0, -5.0, -6.0})
            if self._manual_exposure is not None:
                candidates = [self._manual_exposure] + [
                    c for c in candidates if c != self._manual_exposure
                ]
            best = self._probe_manual_exposure_candidates(candidates)
            if best is not None and DEBUG_PRINTS:
                logger.info(
                    "runtime: manual exp=%.2f period~%.1fms", best[2], best[0]
                )
        elif (
            gray_mean < min_gray
            and (now - self._last_mode_change_ts) > 2.5
            and self._policy_mode != "manual"
        ):
            candidates = list({float(CAMERA_EXPOSURE), -4.0, -3.0, -5.0})
            best = self._probe_manual_exposure_candidates(candidates)
            if best is not None and DEBUG_PRINTS:
                logger.info(
                    "runtime brightened: exp=%.2f gray=%.1f", best[2], best[1]
                )

        self._update_software_gain(gray_mean)

    def _update_software_gain(self, gray_level: float) -> None:
        target_gray = float(CAMERA_RUNTIME_TARGET_GRAY)
        max_gain = float(CAMERA_RUNTIME_SOFT_GAIN_MAX)
        if gray_level <= 0.5:
            desired_gain = max_gain
        elif gray_level < target_gray:
            desired_gain = min(max_gain, max(1.0, target_gray / gray_level))
        else:
            desired_gain = 1.0
        self._software_gain = 0.85 * self._software_gain + 0.15 * desired_gain

    # ------------------------------------------------------------------
    # Capture loop
    # ------------------------------------------------------------------

    def _publish(self, frame: np.ndarray, now: float) -> None:
        """Store a frame into the double buffer (single publish path used
        by both the capture loop and exposure-probe settle reads)."""
        # NOTE: gray_sample is the GREEN-CHANNEL mean — a cheap luma
        # approximation, consumed against the CAMERA_RUNTIME_*_GRAY
        # thresholds. Good enough for exposure policy; not true luma.
        gray_sample = float(np.mean(frame[:, :, 1]))
        if frame.shape != self._frame_bufs[0].shape:
            with self._capture_lock:
                self._frame_bufs = [np.zeros_like(frame), np.zeros_like(frame)]
                self._buf_ready = False
        with self._capture_lock:
            np.copyto(self._frame_bufs[self._buf_write_slot], frame)
            if self._prev_capture_ts > 0:
                dt_ms = (now - self._prev_capture_ts) * 1000.0
                if self._capture_period_ms > 0:
                    self._capture_period_ms = (
                        0.9 * self._capture_period_ms + 0.1 * dt_ms
                    )
                else:
                    self._capture_period_ms = dt_ms
                if dt_ms > (float(CAMERA_RUNTIME_MAX_PERIOD_MS) * 2.0):
                    self._slow_capture_streak += 1
                else:
                    self._slow_capture_streak = max(
                        0, self._slow_capture_streak - 1
                    )
            self._capture_gray_mean = (
                0.92 * self._capture_gray_mean + 0.08 * gray_sample
                if self._capture_gray_mean > 0
                else gray_sample
            )
            self._prev_capture_ts = now
            self._buf_write_slot ^= 1
            self._latest_frame_ts = now
            self._buf_ready = True

    def _capture_loop(self) -> None:
        while self._capture_running:
            try:
                ret, frame = self.cap.read()
            except Exception:
                logger.exception("cap.read() raised; stopping capture")
                break
            if not ret:
                self._capture_fail_streak += 1
                time.sleep(0.001)
                continue
            self._capture_fail_streak = 0
            self._publish(frame, self._clock())
            self._evaluate_runtime_policy(self._clock())
