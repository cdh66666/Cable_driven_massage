"""Offline checks for exact target gating and unsmoothed residual correction."""
from __future__ import annotations

import unittest
from unittest.mock import patch

import y42_pc_direct_web as web


class TrajectoryControlTests(unittest.TestCase):
    def test_xy_gate_requires_motor_flags_and_cartesian_error(self):
        positions = (1.0, 2.0, 3.0, 4.0)
        rpms = (0.0, 0.0, 0.0, 0.0)
        flags = (web.TRAJECTORY_TARGET_REACHED_FLAG,) * 4
        targets = list(positions)
        self.assertTrue(
            web.trajectory_feedback_target_reached_xy(
                positions, rpms, flags, targets, (0.5, 0.5), (0.0, 0.0)
            )
        )
        self.assertFalse(
            web.trajectory_feedback_target_reached_xy(
                positions, rpms, flags, targets, (2.0, 0.0), (0.0, 0.0)
            )
        )

    def test_residual_correction_is_bounded_and_does_not_change_xy(self):
        def fake_map(x, y, calibration):
            return [float(x), float(y), float(x) + float(y), float(x) - float(y)]

        with patch.object(web, "grid_motor_positions_at", side_effect=fake_map):
            result = web.exact_xy_residual_motor_correction(
                (10.0, 10.0), (0.0, 0.0), {}, max_step_deg=0.25, max_total_deg=0.5
            )
        self.assertTrue(result["ok"])
        self.assertEqual(result["correction_deg"], [1 / 6, 1 / 6, 1 / 6, 0.0])
        self.assertLessEqual(sum(abs(v) for v in result["correction_deg"]), 0.5)
        self.assertEqual(result["residual_mm"], (200.0) ** 0.5)

    def test_all_test_patterns_are_one_continuous_motion_plan(self):
        spec = web.all_test_pattern_spec(110.0, 148.0)
        self.assertEqual(spec["sequence_shapes"], list(web.ALL_TEST_PATTERN_KINDS))
        self.assertEqual(len(spec["draw_segments"]), len(web.ALL_TEST_PATTERN_KINDS))
        self.assertGreater(len(spec["motion_segments"]), len(spec["draw_segments"]))
        self.assertGreater(spec["travel_distance_mm"], 0.0)
        self.assertGreater(spec["duration_ms"], 0)
        self.assertEqual(web.validate_trajectory_bounds("custom", spec), (110.0, 148.0))
        self.assertEqual(web.sample_path("custom", 0.0, spec), spec["draw_segments"][0][0])
        self.assertEqual(web.sample_path("custom", 1.0, spec), spec["draw_segments"][-1][-1])


if __name__ == "__main__":
    unittest.main(verbosity=2)
