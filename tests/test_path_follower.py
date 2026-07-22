"""
Unit tests for control/path_follower.py — the path-following target advancer.

Rules under test:
- Seeding: the FIRST update() after start() locks onto the path point
  nearest the ball (never index 0, never re-seeded afterward).
- Advance law: full rate (speed*dt) at err < 10 mm; linear taper (factor
  0.5 at err = 15); frozen with state "stalled" at err >= 20; the law is
  continuous and auto-resumes when the ball catches up.
- Closed paths wrap s and increment the lap counter; open paths clamp at
  the end, report "done", and hold the endpoint until stop().
- pause() freezes s under updates; resume() continues; set_speed clamps.
- telemetry() returns the identical key set in every state; update() in
  idle (or with no path) is a no-op returning the ball position.
"""
import math

import pytest

from control.path_follower import PathFollower
from control.patterns import circle, from_points
from settings import (
    PATH_CAPTURE_RADIUS_MM,
    PATH_SPEED_MAX_MM_S,
    PATH_SPEED_MIN_MM_S,
    PATH_SPEED_MM_S,
)

_TELEMETRY_KEYS = {
    "path_active",
    "path_state",
    "path_name",
    "path_progress",
    "path_lap",
    "path_s_mm",
    "path_error_mm",
    "path_speed_mm_s",
}

_CIRCLE_LEN = 2.0 * math.pi * 65.0  # nominal; polyline total is a hair under


class _FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.now = float(start)

    def advance(self, dt_s: float) -> None:
        self.now += float(dt_s)

    def __call__(self) -> float:
        return self.now


def _seeded_follower(clock: _FakeClock) -> tuple[PathFollower, float, float]:
    """Follower on the default circle, seeded at (65, 0) (the path start)."""
    follower = PathFollower(clock=clock)
    follower.set_path(circle())
    assert follower.start() is True
    tx, ty = follower.update(65.0, 0.0)
    return follower, tx, ty


class TestSeeding:
    def test_seeds_at_nearest_point_not_index_zero(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        path = circle()
        follower.set_path(path)
        assert follower.start() is True

        n = len(path.points)
        seed_pt = path.points[n // 3]  # ~1/3 of the way around the loop
        tx, ty = follower.update(float(seed_pt[0]) + 1.0, float(seed_pt[1]) + 1.0)
        assert math.hypot(tx - seed_pt[0], ty - seed_pt[1]) < 2.5
        tel = follower.telemetry()
        assert tel["path_state"] == "following"
        assert tel["path_s_mm"] == pytest.approx(_CIRCLE_LEN / 3.0, abs=5.0)

    def test_never_reseeds_after_the_first_update(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        path = circle()
        follower.set_path(path)
        follower.start()
        n = len(path.points)
        seed_pt = path.points[n // 3]
        follower.update(float(seed_pt[0]), float(seed_pt[1]))

        # Ball teleports to the path start (a point ON the path, far from the
        # target): the follower must stall in place, NOT jump s back to 0.
        clock.advance(0.1)
        follower.update(65.0, 0.0)
        tel = follower.telemetry()
        assert tel["path_state"] == "stalled"
        assert tel["path_s_mm"] == pytest.approx(_CIRCLE_LEN / 3.0, abs=5.0)


class TestAdvanceLaw:
    def test_full_rate_when_error_inside_full_speed_radius(self) -> None:
        clock = _FakeClock()
        follower, tx, ty = _seeded_follower(clock)
        s0 = follower.telemetry()["path_s_mm"]
        clock.advance(0.05)
        follower.update(tx, ty)  # ball dead on target: err = 0
        tel = follower.telemetry()
        assert tel["path_s_mm"] - s0 == pytest.approx(PATH_SPEED_MM_S * 0.05)
        assert tel["path_state"] == "following"
        assert tel["path_speed_mm_s"] == pytest.approx(PATH_SPEED_MM_S)

    def test_taper_gives_half_speed_at_err_15(self) -> None:
        clock = _FakeClock()
        follower, tx, ty = _seeded_follower(clock)
        s0 = follower.telemetry()["path_s_mm"]
        clock.advance(0.1)
        # Ball 15 mm radially outside the target: factor = (20-15)/10 = 0.5.
        scale = (65.0 + 15.0) / 65.0
        follower.update(tx * scale, ty * scale)
        tel = follower.telemetry()
        assert tel["path_error_mm"] == pytest.approx(15.0, abs=0.05)
        assert tel["path_s_mm"] - s0 == pytest.approx(PATH_SPEED_MM_S * 0.5 * 0.1, abs=0.03)
        assert tel["path_speed_mm_s"] == pytest.approx(PATH_SPEED_MM_S * 0.5, abs=0.1)

    def test_frozen_and_stalled_at_capture_radius_then_auto_resumes(self) -> None:
        clock = _FakeClock()
        follower, tx, ty = _seeded_follower(clock)
        s0 = follower.telemetry()["path_s_mm"]

        clock.advance(0.1)
        follower.update(tx + PATH_CAPTURE_RADIUS_MM + 5.0, ty)  # err = 25
        tel = follower.telemetry()
        assert tel["path_state"] == "stalled"
        assert tel["path_s_mm"] == s0  # advance frozen
        assert tel["path_speed_mm_s"] == 0.0

        # Ball catches up: auto-resume, no explicit call needed.
        clock.advance(0.1)
        follower.update(tx, ty)
        tel = follower.telemetry()
        assert tel["path_state"] == "following"
        assert tel["path_s_mm"] - s0 == pytest.approx(PATH_SPEED_MM_S * 0.1)


class TestClosedWrap:
    def test_wrap_increments_lap_and_wraps_s(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        follower.set_path(circle())
        follower.set_speed(80.0)
        follower.start()
        target = follower.update(65.0, 0.0)  # seeds at s = 0
        for _ in range(60):  # one lap needs ~51 updates at 8 mm/update
            clock.advance(0.1)
            target = follower.update(target[0], target[1])
            if follower.telemetry()["path_lap"] == 1:
                break
        tel = follower.telemetry()
        assert tel["path_lap"] == 1
        assert tel["path_s_mm"] < 10.0  # s wrapped back near the start
        assert tel["path_state"] == "following"


class TestOpenPathDone:
    def _drive_to_done(
        self, follower: PathFollower, clock: _FakeClock
    ) -> tuple[float, float]:
        target = follower.update(-40.0, 0.0)  # seeds at the (-40, 0) end
        for _ in range(60):  # 80 mm at 3 mm/update -> ~27 updates
            if follower.telemetry()["path_state"] == "done":
                break
            clock.advance(0.1)
            target = follower.update(target[0], target[1])
        return target

    def test_reaches_done_holds_endpoint_and_reports_progress_1(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        follower.set_path(
            from_points([(-40.0, 0.0), (40.0, 0.0)], closed=False, name="line")
        )
        follower.start()
        target = self._drive_to_done(follower, clock)

        tel = follower.telemetry()
        assert tel["path_state"] == "done"
        assert follower.done is True
        assert tel["path_progress"] == pytest.approx(1.0)
        assert target == pytest.approx((40.0, 0.0), abs=1e-9)

        # Endpoint holds under further updates, wherever the ball goes.
        clock.advance(0.5)
        held = follower.update(-10.0, 5.0)
        assert held == pytest.approx((40.0, 0.0), abs=1e-9)
        tel = follower.telemetry()
        assert tel["path_state"] == "done"
        assert tel["path_speed_mm_s"] == 0.0

        follower.stop()
        assert follower.active is False
        assert follower.done is False
        assert follower.telemetry()["path_state"] == "idle"


class TestPauseResume:
    def test_pause_freezes_s_and_resume_continues(self) -> None:
        clock = _FakeClock()
        follower, tx, ty = _seeded_follower(clock)
        clock.advance(0.1)
        follower.update(tx, ty)
        follower.pause()
        assert follower.telemetry()["path_state"] == "paused"

        s_paused = follower.telemetry()["path_s_mm"]
        held = None
        for _ in range(5):
            clock.advance(0.1)
            held = follower.update(0.0, 0.0)  # ball wandering: still frozen
        tel = follower.telemetry()
        assert tel["path_s_mm"] == s_paused
        assert tel["path_state"] == "paused"
        assert tel["path_speed_mm_s"] == 0.0
        assert tel["path_error_mm"] > 0.0  # err still tracked while paused
        assert held is not None

        follower.resume()
        assert follower.telemetry()["path_state"] == "following"
        clock.advance(0.1)
        follower.update(held[0], held[1])  # ball back on the held target
        assert follower.telemetry()["path_s_mm"] - s_paused == pytest.approx(
            PATH_SPEED_MM_S * 0.1
        )

    def test_pause_is_ignored_outside_following_or_stalled(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        follower.pause()
        assert follower.telemetry()["path_state"] == "idle"
        follower.set_path(circle())
        follower.start()
        follower.pause()  # armed: not pausable
        assert follower.telemetry()["path_state"] == "armed"


class TestSpeed:
    def test_set_speed_clamps_to_limits(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        follower.set_speed(PATH_SPEED_MIN_MM_S - 5.0)
        assert follower.speed_mm_s == PATH_SPEED_MIN_MM_S
        follower.set_speed(PATH_SPEED_MAX_MM_S + 100.0)
        assert follower.speed_mm_s == PATH_SPEED_MAX_MM_S

    def test_constructor_speed_is_clamped_too(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock, speed_mm_s=500.0)
        assert follower.speed_mm_s == PATH_SPEED_MAX_MM_S


class TestTelemetry:
    def test_key_set_identical_in_every_state(self) -> None:
        clock = _FakeClock()
        seen: dict[str, set[str]] = {}

        follower = PathFollower(clock=clock)
        seen["idle"] = set(follower.telemetry())

        follower.set_path(circle())
        follower.start()
        seen["armed"] = set(follower.telemetry())

        tx, ty = follower.update(65.0, 0.0)
        seen["following"] = set(follower.telemetry())

        clock.advance(0.1)
        follower.update(tx + 40.0, ty + 40.0)
        assert follower.telemetry()["path_state"] == "stalled"
        seen["stalled"] = set(follower.telemetry())

        follower.pause()
        seen["paused"] = set(follower.telemetry())

        done_follower = PathFollower(clock=clock)
        done_follower.set_path(
            from_points([(0.0, 0.0), (10.0, 0.0)], closed=False, name="stub")
        )
        done_follower.start()
        target = done_follower.update(0.0, 0.0)
        for _ in range(10):
            clock.advance(0.1)
            target = done_follower.update(target[0], target[1])
        assert done_follower.telemetry()["path_state"] == "done"
        seen["done"] = set(done_follower.telemetry())

        for state, keys in seen.items():
            assert keys == _TELEMETRY_KEYS, f"telemetry keys differ in {state!r}"

    def test_error_is_zero_before_first_update(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        follower.set_path(circle())
        assert follower.telemetry()["path_error_mm"] == 0.0


class TestIdleNoOp:
    def test_update_without_path_returns_ball_position(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        assert follower.update(3.0, 4.0) == (3.0, 4.0)
        assert follower.active is False
        assert follower.start() is False  # no path loaded

    def test_update_with_path_but_not_started_is_a_noop(self) -> None:
        clock = _FakeClock()
        follower = PathFollower(clock=clock)
        follower.set_path(circle())
        assert follower.update(3.0, 4.0) == (3.0, 4.0)
        tel = follower.telemetry()
        assert tel["path_state"] == "idle"
        assert tel["path_active"] is False
        assert tel["path_s_mm"] == 0.0
