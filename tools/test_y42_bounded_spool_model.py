"""Pure unit tests: imports no controller, opens no serial/network/file data."""

import math
import unittest

from y42_bounded_spool_model import (
    affine_length_mm,
    affine_position_deg,
    fit_bounded_affine,
    slope_limits_from_radius,
)


class BoundedAffineTests(unittest.TestCase):
    def fit(self, xs, ys, direction=1, **kwargs):
        return fit_bounded_affine(xs, ys, direction=direction,
                                  abs_slope_bounds_deg_per_mm=(2.0, 6.0), **kwargs)

    def test_radius_bounds_are_reciprocal_and_explicit(self):
        low, high = slope_limits_from_radius(12.0, 20.0)
        self.assertAlmostEqual(low, 180.0 / math.pi / 20.0)
        self.assertAlmostEqual(high, 180.0 / math.pi / 12.0)
        for limits in [(0, 20), (20, 12), (12, math.inf), (math.nan, 20)]:
            with self.assertRaises(ValueError):
                slope_limits_from_radius(*limits)

    def test_signed_exact_fit_and_bidirectional_round_trip(self):
        for direction in (-1, 1):
            model = self.fit([10, 50, 100, 180],
                             [17 + direction * 4 * x for x in [10, 50, 100, 180]], direction)
            self.assertTrue(model["accepted"])
            self.assertAlmostEqual(model["slope_deg_per_mm"], direction * 4)
            self.assertEqual(len(model["runtime_curve"]), 4)
            self.assertTrue(model["runtime_knots_are_synthetic"])
            for length in [-5, 10, 30, 80, 180, 200]:
                self.assertAlmostEqual(affine_length_mm(model, affine_position_deg(model, length)), length)

    def test_out_of_bound_or_wrong_direction_is_not_clipped(self):
        for slope in [0, 1, 7, -4]:
            model = self.fit([0, 10, 20], [slope * x for x in [0, 10, 20]])
            self.assertFalse(model["accepted"])
            self.assertEqual(model["slope_deg_per_mm"], slope)
            self.assertIsNone(model["runtime_curve"])
            with self.assertRaises(ValueError):
                affine_position_deg(model, 12)
            with self.assertRaises(ValueError):
                affine_length_mm(model, 12)

    def test_near_flat_measured_segment_not_used_for_runtime(self):
        # Immutable values copied from real run 20260905_103233... M4.
        xs = [19.84269692481342, 126.74775193765765, 137.54973144739324, 185.98706573643773]
        ys = [-370.45, 153.15, 153.30, 410.0]
        model = self.fit(xs, ys)
        self.assertTrue(model["accepted"])
        self.assertEqual(model["raw_distance_samples_mm"], xs)
        self.assertEqual(model["raw_position_samples_deg"], ys)
        self.assertAlmostEqual(model["slope_deg_per_mm"], 4.671155683308748)
        self.assertAlmostEqual(model["raw_local_segments"][1]["slope_deg_per_mm"], 0.0138863435, places=8)
        self.assertFalse(model["raw_local_segments"][1]["within_supplied_bounds"])
        for a, b in zip(model["runtime_curve"], model["runtime_curve"][1:]):
            self.assertAlmostEqual((b[1] - a[1]) / (b[0] - a[0]), model["slope_deg_per_mm"])
        self.assertGreater(model["max_abs_residual_mm"], 1.0)
        self.assertFalse(model["physical_pose_accuracy_verified"])
        rejected = self.fit(xs, ys, max_abs_residual_mm=0.5)
        self.assertFalse(rejected["accepted"])
        self.assertIn("affine_residual_exceeds_supplied_limit", rejected["rejection_reasons"])

    def test_duplicate_distances_do_not_hide_conflicting_readings(self):
        model = self.fit([0, 10, 10, 20], [0, 20, 60, 80], max_abs_residual_mm=2)
        self.assertFalse(model["accepted"])
        self.assertEqual(model["sample_count"], 4)
        self.assertEqual(model["distinct_length_count"], 3)
        self.assertEqual(model["raw_position_samples_deg"], [0, 20, 60, 80])
        self.assertEqual(model["max_abs_residual_mm"], 5.0)

    def test_bad_inputs_raise_instead_of_generating_nan_model(self):
        bad = [([0, 1], [0, 4]), ([0, 1, 2], [0, 4]),
               ([0, 1, math.nan], [0, 4, 8]), ([0, 1, 2], [0, 4, math.inf]),
               ([1, 1, 1], [1, 2, 3]), ([-1, 1, 2], [0, 4, 8])]
        for xs, ys in bad:
            with self.assertRaises(ValueError):
                self.fit(xs, ys)
        for kwargs in [{"max_abs_residual_mm": math.nan}, {"min_r2": -1},
                       {"min_distance_span_mm": 100}, {"runtime_knot_count": 1}]:
            with self.assertRaises(ValueError):
                self.fit([0, 10, 20], [0, 40, 80], **kwargs)

    def test_input_lists_are_not_modified(self):
        xs, ys = [20, 0, 10], [80, 0, 40]
        self.fit(xs, ys)
        self.assertEqual(xs, [20, 0, 10])
        self.assertEqual(ys, [80, 0, 40])


if __name__ == "__main__":
    unittest.main()
