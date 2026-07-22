"""
Path-following feasibility pins over tools/path_sim.py: the closed-loop
sim (real filter -> BallController/PathFollower chain, idealized plant —
no servo dynamics, latency, or friction, so an OPTIMISTIC bound) must
show the pacing law stable at the shipped defaults. Each run is 900-1800
steps of pure Python + numpy — well under a second each.
"""

from control.patterns import circle, star
from tools.path_sim import simulate_path_following


class TestCircleFeasibility:
    def test_default_speed_tracks_and_laps(self) -> None:
        # v=30 (the PATH_SPEED_MM_S default) on a 60 mm circle: the ball
        # must never stall out of the capture radius and completes at
        # least one full lap in 40 s.
        res = simulate_path_following(circle(radius_mm=60.0), 30.0, 40.0)
        assert res.max_err_mm < 20.0
        assert res.laps >= 1
        assert res.mean_err_mm < 15.0

    def test_slow_following_is_smooth(self) -> None:
        # v=10: pursuit lag stays small — slow following is smooth.
        res = simulate_path_following(circle(radius_mm=60.0), 10.0, 60.0)
        assert res.max_err_mm < 8.0

    def test_infeasible_speed_degrades_gracefully(self) -> None:
        # THE SAFETY PROPERTY: v=60 is well above the analytic ~35 mm/s
        # stall ceiling at default gains. The adaptive taper must degrade
        # an infeasible speed setting into slower ACTUAL laps — the
        # target waits inside the capture radius rather than running away
        # — so the ball is never abandoned. Tracking error stays bounded
        # near the capture radius instead of diverging.
        res = simulate_path_following(circle(radius_mm=60.0), 60.0, 30.0)
        assert res.max_err_mm < 25.0


class TestStarFeasibility:
    def test_sharp_corners_slow_and_recover(self) -> None:
        # 5-point star at v=25: the sharp inner corners force overshoot,
        # the taper slows the target, the ball recovers — no divergence.
        res = simulate_path_following(star(), 25.0, 40.0)
        assert res.max_err_mm < 25.0
