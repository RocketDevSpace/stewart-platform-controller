"""
control/routine_runner.py

Routine playback state machine. Has NO Qt imports — the GUI owns the
QTimer and calls tick() on each interval. RoutineRunner owns all state:
the loaded step list, the current index, the running flag, and the
preview/send mode flag.

Preview mode: tick() just forwards the current pose to the GUI via the
on_pose_update callback, advances the index, and wraps at the end.

Send mode: tick() solves IK via the injected IKEngine, sends the
resulting servo angles via the injected ServoDriver, forwards the pose
to the GUI, and returns False once all steps have been consumed.

Send-mode routines never park the platform at an extreme pose: start_send()
appends a cosine ease back to neutral after the routine's last waypoint,
and cancel() replaces the remaining steps with the same ease (wind-down)
instead of stopping dead — tick() keeps returning True until the platform
is home. IK failures during playback are counted, surfaced via
ik_failure_count, and logged on first occurrence (the platform silently
holding its previous pose while the preview marches on was invisible to
the operator).
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable

import numpy as np

from core.ik_engine import IKEngine
from core.platform_state import Pose
from hardware.servo_driver import ServoDriver
from routines.routines import ROUTINES
from settings import CONTROL_LOOP_INTERVAL_MS, ROUTINE_RETURN_HOME_S

logger = logging.getLogger(__name__)

_NEUTRAL_POSE: dict[str, float] = {
    "x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
}


def _ease_to_neutral(from_pose: dict[str, float]) -> list[dict[str, float]]:
    """Cosine-eased pose interpolation from from_pose to neutral."""
    steps = max(2, int(ROUTINE_RETURN_HOME_S * 1000.0 / CONTROL_LOOP_INTERVAL_MS))
    out: list[dict[str, float]] = []
    for n in range(1, steps + 1):
        t = n / steps
        blend = (1.0 - math.cos(math.pi * t)) / 2.0
        out.append({
            k: float(from_pose.get(k, 0.0)) * (1.0 - blend)
            + _NEUTRAL_POSE[k] * blend
            for k in _NEUTRAL_POSE
        })
    return out


class RoutineRunner:
    def __init__(
        self,
        ik_engine: IKEngine,
        serial_driver: ServoDriver,
        on_pose_update: Callable[[dict], None],
    ) -> None:
        self._ik = ik_engine
        self._servo = serial_driver
        self._on_pose = on_pose_update
        self._steps: list[dict] = []
        self._index: int = 0
        self._running: bool = False
        self._send_mode: bool = False
        self._prev_arm_points: np.ndarray | None = None
        self._ik_failures: int = 0

    def load(self, routine_name: str) -> bool:
        if routine_name not in ROUTINES:
            return False
        self._steps = list(ROUTINES[routine_name]())
        self._index = 0
        self._running = False
        self._send_mode = False
        self._prev_arm_points = None
        self._ik_failures = 0
        return True

    def start_preview(self) -> None:
        if self._index >= len(self._steps):
            self._index = 0  # restart-after-exhaustion guard
        self._running = True
        self._send_mode = False

    def start_send(self) -> None:
        if self._index >= len(self._steps):
            self._index = 0  # restart-after-exhaustion guard
        # Never park at the routine's last waypoint (screw ends at
        # z=-12/yaw=-35): ease back to neutral after the final step.
        if self._steps and self._steps[-1] != _NEUTRAL_POSE:
            self._steps = self._steps + _ease_to_neutral(self._steps[-1])
        self._running = True
        self._send_mode = True
        self._ik_failures = 0

    def cancel(self, wind_down: bool = True) -> bool:
        """Cancel playback.

        In send mode mid-routine (with wind_down=True, the default) this
        does NOT stop dead: the remaining steps are replaced with an ease
        back to neutral and the runner keeps running until it lands
        (returns True = winding down; the caller must keep ticking).
        In preview mode, with wind_down=False, or when nothing is running,
        playback stops immediately (returns False).
        """
        if (
            wind_down
            and self._send_mode
            and self._running
            and self._index < len(self._steps)
        ):
            current = self._steps[max(0, self._index - 1)]
            self._steps = _ease_to_neutral(current)
            self._index = 0
            return True

        self._running = False
        self._send_mode = False
        self._steps = []
        self._index = 0
        self._prev_arm_points = None
        return False

    def tick(self) -> bool:
        if not self._running or not self._steps:
            return False

        pose_dict = self._steps[self._index]

        if self._send_mode:
            pose = Pose(
                x=float(pose_dict["x"]),
                y=float(pose_dict["y"]),
                z=float(pose_dict["z"]),
                roll=float(pose_dict["roll"]),
                pitch=float(pose_dict["pitch"]),
                yaw=float(pose_dict["yaw"]),
            )
            ik_result = self._ik.solve(pose, self._prev_arm_points)
            if ik_result.success:
                self._prev_arm_points = np.asarray(ik_result.arm_points)
                self._servo.send_angles(list(ik_result.servo_angles_deg))
            else:
                self._ik_failures += 1
                if self._ik_failures == 1:
                    logger.warning(
                        "IK failed at routine step %d (pose=%s): %s — "
                        "platform holds previous pose for skipped steps",
                        self._index, pose_dict, ik_result.servo_status,
                    )

            self._on_pose(pose_dict)
            self._index += 1
            if self._index >= len(self._steps):
                self._running = False
                if self._ik_failures:
                    logger.warning(
                        "routine finished with %d IK-failed steps skipped",
                        self._ik_failures,
                    )
                return False
            return True

        # preview mode: wrap at end, never stops on its own
        self._on_pose(pose_dict)
        self._index += 1
        if self._index >= len(self._steps):
            self._index = 0
        return True

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_send_mode(self) -> bool:
        return self._send_mode

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def total_steps(self) -> int:
        return len(self._steps)

    @property
    def ik_failure_count(self) -> int:
        """IK-failed (skipped) steps in the current/most recent send run."""
        return self._ik_failures
