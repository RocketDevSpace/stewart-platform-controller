"""
Smoke test for tools/jitter_bench.py: the quiescent profile runs through
the real MeasurementFilter -> BallController -> IK -> ServoDriver chain
headlessly and yields finite metrics. Kept short (60 frames) so the whole
test stays well under 2 s.
"""

import math

from tools.jitter_bench import build_profile, run_bench, run_profile


class TestJitterBenchSmoke:
    def test_quiescent_60_frames_returns_finite_metrics(self) -> None:
        report = run_profile("quiescent", frames=60)
        assert report["frames"] == 60
        assert isinstance(report["total_sends"], int)
        numeric = [
            v for v in report.values()
            if isinstance(v, (int, float)) and not isinstance(v, bool)
        ]
        assert numeric, "report has no numeric metrics"
        assert all(math.isfinite(float(v)) for v in numeric)
        for seq_key in ("flips_per_min_per_servo", "cmd_std_deg_per_servo"):
            seq = report[seq_key]
            assert isinstance(seq, list) and len(seq) == 6
            assert all(math.isfinite(float(v)) for v in seq)
        assert report["rest_duty"] is None  # rest mode not implemented yet

    def test_profiles_are_deterministic(self) -> None:
        a = build_profile("quiescent", frames=60, fps=30.0, seed=42)
        b = build_profile("quiescent", frames=60, fps=30.0, seed=42)
        assert a == b

    def test_commands_are_produced(self) -> None:
        report = run_bench(
            build_profile("quiescent", frames=60, fps=30.0, seed=42),
            "quiescent",
        )
        total_sends = report["total_sends"]
        assert isinstance(total_sends, int)
        assert total_sends > 0
