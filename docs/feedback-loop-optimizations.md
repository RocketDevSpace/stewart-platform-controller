# Feedback Loop Optimizations

Five prioritized changes to reduce end-to-end latency from camera frame capture to servo command arrival. Each targets a specific stage in the pipeline:

```
Camera capture thread  -->  BallTracker.update()  -->  VisionControlWorker._tick()
  (PD compute + IK solve + serial enqueue)  -->  GUI snapshot emit  -->  serial write thread
```

Reference branch: `origin/codex/offset-tuning-and-camera-exposure` (commit `f8d01f8`).

---

## 1. Decouple servo command dispatch from GUI snapshot emission

**Summary:** Send the servo command immediately in the worker tick and stop letting a slow GUI frame delay the next control cycle.

**Problem:**
`VisionControlWorker._tick()` (in `cv/vision_control_worker.py`) does everything in a single method: tracker update, PD compute, IK solve, serial enqueue, *and* snapshot emission. The snapshot is sent to the GUI thread via a cross-thread Qt signal (`snapshot_ready.emit(snapshot)`). A backpressure mechanism prevents emitting a new snapshot while the previous one is still being processed by the GUI:

```python
# vision_control_worker.py, ~line 256/393
if should_emit_snapshot and (not self._snapshot_inflight):
    self._snapshot_inflight = True
    self.snapshot_ready.emit(snapshot)
```

The `_snapshot_inflight` flag is only cleared when the GUI calls `mark_snapshot_consumed()` back via another cross-thread signal. If the GUI is busy painting camera views, updating timing plots, or running the 3D visualizer (see `gui_main.py:650-750`), the flag stays set and the *next* `_tick()` call skips emission. While the serial command itself is sent before the snapshot check, the `_tick()` method still assembles the full `ControlSnapshot` dataclass (including optional BGR frame copies for camera views) on every cycle, which is wasted work when the GUI can't consume it.

**Fix:**
Split `_tick()` into two phases:

1. **Control phase (latency-critical):** Fetch tracker state, compute PD, solve IK, enqueue serial command. This should complete as fast as possible with no allocations for GUI data.
2. **Telemetry phase (best-effort):** Build the `ControlSnapshot`, copy debug frames if `return_debug_frames` is true, emit the signal. Gate this phase on both `should_emit_snapshot` and `not self._snapshot_inflight`, and skip the frame copies / dict construction entirely when the gate is closed.

This means the serial command goes out on every tick regardless of GUI state. Currently the serial command *does* go out before the snapshot check, but the overhead of building the snapshot dict, copying timings, copying quality metrics, etc. still adds ~0.2-0.5 ms of unnecessary work on ticks where the snapshot won't be consumed.

**Files to modify:**
- `cv/vision_control_worker.py`: `_tick()` method (lines 182-398)

---

## 2. Replace QTimer polling with an event-driven control loop

**Summary:** Wake the control loop the instant a new frame arrives instead of polling on a fixed-interval QTimer.

**Problem:**
The vision worker runs on a `QThread` and is driven by a `QTimer` set to fire every `1000 / VISION_LOOP_HZ` ms (currently `1000/120 = ~8.3 ms`). This creates two sources of latency:

- **Timer jitter:** Qt's `PreciseTimer` on most platforms (especially macOS/Darwin) still has 1-2 ms of jitter. The timer may fire 1 ms before the capture thread has a new frame (wasting a cycle on the stale-frame check) or 1 ms after (adding unnecessary delay).
- **Stale-frame spinning:** When the timer fires but no new frame is available, `BallTracker.update()` detects `frame_ts <= self._last_processed_frame_ts` and returns `None` with reason `"stale_frame_repeat"`. The worker then returns early. At 120 Hz timer vs. 30 FPS camera, roughly 3 out of every 4 ticks are wasted stale-frame checks.

**Fix:**
Replace the QTimer + QThread architecture with a dedicated `threading.Thread` that blocks on a `threading.Event`:

```python
# In the capture loop (BallTracker._capture_loop), after storing the frame:
self._new_frame_event.set()

# In the control loop (new dedicated thread):
while self._running:
    self._new_frame_event.wait(timeout=0.05)  # 50ms safety timeout
    self._new_frame_event.clear()
    # ... fetch frame, PD, IK, serial enqueue ...
```

This eliminates all stale-frame wakeups and ensures the control computation starts within microseconds of the frame landing, rather than waiting up to 8.3 ms for the next timer tick. The safety timeout handles the case where the camera stops producing frames.

If the Qt signal mechanism is still needed for GUI snapshots, the control thread can emit snapshots via `QMetaObject.invokeMethod(..., Qt.QueuedConnection)` or a simple `queue.Queue` polled by a low-frequency GUI timer.

**Files to modify:**
- `cv/vision_control_worker.py`: Replace `QTimer`-based `_tick()` with a `threading.Thread` loop
- `cv/ball_tracker.py`: Add a `threading.Event` (`_new_frame_event`) set in `_capture_loop()` after storing a new frame (~line 548-553)

---

## 3. Eliminate the per-frame image copy in BallTracker.update()

**Summary:** Use a double-buffer swap instead of `numpy.copy()` to hand frames from the capture thread to the processing thread.

**Problem:**
In `cv/ball_tracker.py`, the `update()` method acquires the capture lock and copies the entire frame:

```python
# ball_tracker.py, ~line 768-791
with self._capture_lock:
    frame_ts = self._latest_frame_ts
    latest_frame = self._latest_frame
# ...
frame = latest_frame.copy()  # <-- full 640x480x3 = 921,600 byte memcpy
```

The copy is necessary because the capture thread (`_capture_loop`) continuously overwrites `self._latest_frame` with new data from `cap.read()`. Without the copy, the processing thread would be reading from a buffer that the capture thread is simultaneously writing to. At 640x480 BGR, this copy takes ~0.3-0.5 ms per frame (measured via `perf_counter` deltas in the profiling data as `capture_fetch`).

**Fix:**
Use a double-buffer (ping-pong) scheme:

```python
# __init__:
self._buffers = [None, None]
self._write_idx = 0  # capture thread writes here
self._read_idx = -1  # processing thread reads here (-1 = no frame ready)
self._buffer_lock = threading.Lock()

# _capture_loop:
ret, frame = self.cap.read()
if ret:
    with self._buffer_lock:
        self._buffers[self._write_idx] = frame  # cap.read() already allocates a new array
        self._read_idx = self._write_idx
        self._write_idx = 1 - self._write_idx
        self._latest_frame_ts = time.perf_counter()

# update():
with self._buffer_lock:
    if self._read_idx < 0:
        return None
    frame = self._buffers[self._read_idx]  # no copy -- capture thread is writing to the other buffer
    frame_ts = self._latest_frame_ts
    self._read_idx = -1  # mark as consumed
```

Note: OpenCV's `cap.read()` already allocates a new numpy array on each call (it does not reuse the previous buffer), so the capture thread's next `cap.read()` will not overwrite the frame the processing thread is reading. The double-buffer scheme simply makes this guarantee explicit and removes the need for `.copy()`.

**Files to modify:**
- `cv/ball_tracker.py`: `__init__`, `_capture_loop()` (~line 519-553), `update()` (~line 766-791)

---

## 4. Cache the warped-frame brightness calculation

**Summary:** Compute the warp grayscale mean every Nth frame instead of every frame, since room brightness changes slowly.

**Problem:**
In `cv/ball_tracker.py`, after warping the perspective-corrected image, the code computes mean brightness to decide whether to relax the HSV V-minimum threshold for ball detection in dim conditions:

```python
# ball_tracker.py, ~line 993-1006
warped = cv2.warpPerspective(frame, H, (self.WARP_SIZE_PX, self.WARP_SIZE_PX), ...)
gray_warp_mean = float(np.mean(cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)))  # actually BGR2GRAY
# ...
hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
if gray_warp_mean < 45.0:
    relax = min(70, int((45.0 - gray_warp_mean) * 2.0))
    vmin_eff = max(20, vmin_base - relax)
```

This involves:
1. `cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)` -- a per-pixel weighted sum on a 480x480x3 image (691,200 bytes in, 230,400 bytes out)
2. `np.mean(...)` -- a reduction over 230,400 elements

Together these take ~0.3-0.5 ms. Room lighting and camera exposure change on the scale of seconds, not frames. Computing this every frame is wasteful.

**Fix:**
Cache the result and recompute every N frames (e.g., every 10):

```python
# __init__:
self._cached_gray_warp_mean = 80.0  # assume reasonable brightness at start
self._gray_warp_recompute_every = 10

# update(), after warpPerspective:
if self._frame_counter % self._gray_warp_recompute_every == 0:
    self._cached_gray_warp_mean = float(np.mean(cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)))
gray_warp_mean = self._cached_gray_warp_mean
```

This saves ~0.3-0.5 ms on 9 out of every 10 frames. The `_frame_counter` is already incremented on every `update()` call (line 802), so no new counter is needed.

**Files to modify:**
- `cv/ball_tracker.py`: `__init__` (add cache fields), `update()` (~line 995)

---

## 5. Increase ArUco detection skip interval and ensure getPerspectiveTransform is used

**Summary:** Reuse the cached homography matrix for more frames between ArUco re-detections, and always use the fast closed-form perspective transform.

**Problem:**
ArUco marker detection (`self.detector.detectMarkers(gray_detect)`) is the single most expensive per-frame operation, typically 2-5 ms depending on image size and number of markers. The code already caches the homography matrix `_last_H` and reuses it for `TRACKER_ARUCO_REDETECT_EVERY_N` frames (currently set to 5 in `config.py`). On cache-hit frames, the code skips grayscale conversion, optional downscaling, ArUco detection, and homography computation entirely.

However:
- The platform moves slowly relative to the camera frame rate. At 30 FPS with max platform tilt rate of 300 deg/s, the perspective change between frames is negligible. The homography is valid for many more frames than 5.
- When ArUco detection fails on the full frame, a retry pass with histogram equalization adds another `detectMarkers` call (~2-5 ms, tracked as `aruco_retry`).
- `findHomography` (RANSAC-based, handles outliers) is used as a fallback even when exactly 4 points are available. `getPerspectiveTransform` is the correct choice for exactly 4 point correspondences -- it's a direct matrix solve with no iteration.

**Fix:**

1. Increase `TRACKER_ARUCO_REDETECT_EVERY_N` from 5 to 8-12 in `config.py`. At 30 FPS, this means re-detecting markers every ~270-400 ms, which is still fast enough to track any realistic platform movement. The existing `TRACKER_MAX_ARUCO_HOLD_FRAMES` (currently 3) provides a safety bound when detection fails.

2. Verify the code path at ~line 975 always uses `getPerspectiveTransform` when 4 points are available (it currently does via `if len(src_pts) == 4`), and only falls back to `findHomography` for degenerate cases. This is already correct on the branch but worth a unit test to prevent regression.

3. Consider removing the histogram-equalization retry (`cv2.equalizeHist` + second `detectMarkers`) or making it opt-in via config. When the cached homography is available, a failed detection on one frame is harmless -- the cache covers it. The retry is only valuable when the cache is exhausted *and* the scene is low-contrast, which is a rare combination.

**Files to modify:**
- `config.py`: Change `TRACKER_ARUCO_REDETECT_EVERY_N` from 5 to 8-12
- `cv/ball_tracker.py`: Optionally gate the `equalizeHist` retry behind a config flag (~line 906-909)
