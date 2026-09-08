"""Offline tests for the fixed-plane motor-target inverse diagnostic."""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from y42_xy_inverse import solve_fixed_plane_xy


def _synthetic_calibration(web):
    """Create a disabled CAD model with an explicitly complete synthetic grid."""
    calibration = {
        "valid": False,
        "workspace_width": 110.0,
        "workspace_height": 148.0,
        "slope": [-3.8, 4.1, -4.0, 3.9],
        "intercept": [100.0, -220.0, 310.0, -430.0],
        "slope_trim_percent": [0.0, 0.0, 0.0, 0.0],
        "samples": [],
    }
    grid = web.blank_grid_compensation(3, 3)
    compensation_at_corner = {
        (0, 0): (0.0, 0.0, 0.0, 0.0),
        (0, 2): (4.0, -2.0, 1.0, 3.0),
        (2, 0): (-3.0, 2.0, -4.0, 1.0),
        (2, 2): (2.0, 4.0, -1.0, -3.0),
    }
    for node in grid["samples"]:
        key = (node["row"], node["col"])
        if key in compensation_at_corner:
            node["position_compensation_deg"] = list(compensation_at_corner[key])
        else:
            # The centre and edge nodes are bilinear samples of the four
            # corners, so the map remains smooth and has no nearest fallback.
            u = node["col"] / 2.0
            v = node["row"] / 2.0
            corners = compensation_at_corner
            node["position_compensation_deg"] = [
                (corners[(0, 0)][index] * (1.0 - u) + corners[(0, 2)][index] * u) * (1.0 - v)
                + (corners[(2, 0)][index] * (1.0 - u) + corners[(2, 2)][index] * u) * v
                for index in range(4)
            ]
        node.update(
            {
                "valid": True,
                "invalid_reason": "",
                "absolute_positions_deg": [0.0, 0.0, 0.0, 0.0],
                "phase_current_ma": [120.0, 120.0, 120.0, 120.0],
                "solved_xy_mm": [node["x_mm"], node["y_mm"]],
                "solved_z_mm": 0.0,
                "plane_reference_z_mm": 0.0,
                "height_error_mm": 0.0,
                "z_fit_spread_mm": 0.0,
                "xy_error_mm": 0.0,
                "fit_error_mm": 0.0,
                "tension_feedforward_ma": [0.0, 0.0, 0.0, 0.0],
            }
        )
    grid.update(
        {
            "valid": True,
            "enabled": True,
            "sample_count": 9,
            "valid_sample_count": 9,
            "reference_phase_current_ma": [120.0, 120.0, 120.0, 120.0],
        }
    )
    calibration["grid_compensation"] = grid
    return calibration


class FixedPlaneInverseTests(unittest.TestCase):
    def test_exact_generic_map_reports_convergence_and_residual(self):
        target = [2.0 * 31.0 + 17.0, 31.0 - 2.0 * 17.0, 31.0 + 17.0]
        result = solve_fixed_plane_xy(
            target,
            lambda x, y: [2.0 * x + y, x - 2.0 * y, x + y],
            initial_xy=(0.0, 0.0),
            bounds=((0.0, 110.0), (0.0, 148.0)),
        )
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["converged"])
        self.assertLessEqual(result["residual_max"], 1e-4)
        self.assertAlmostEqual(result["x"], 31.0, places=7)
        self.assertAlmostEqual(result["y"], 17.0, places=7)

    def test_nonfinite_and_singular_maps_are_not_valid(self):
        nonfinite = solve_fixed_plane_xy([1.0, 2.0], lambda _x, _y: [math.nan, 2.0])
        self.assertFalse(nonfinite["ok"])
        self.assertFalse(nonfinite["finite"])
        singular = solve_fixed_plane_xy(
            [3.0, 3.0],
            lambda x, y: [x + y, x + y],
            initial_xy=(0.0, 0.0),
            max_iterations=4,
        )
        self.assertFalse(singular["ok"])
        self.assertEqual(singular["termination"], "singular_jacobian")

    def test_grid_compensation_forward_is_inverted_without_mutating_valid_flag(self):
        # Importing the controller through the existing isolated fixture keeps
        # this test away from the real calibration JSON and serial subsystem.
        from test_y42_calibration_flow import isolated_web_module

        with tempfile.TemporaryDirectory(prefix="y42-xy-inverse-") as directory:
            web = isolated_web_module(directory)
            calibration = _synthetic_calibration(web)
            point = (37.25, 96.75)
            targets = web.grid_motor_positions_at(point[0], point[1], calibration)
            result = web.motor_targets_to_xy(targets, calibration, initial_xy=(55.0, 74.0))
            self.assertTrue(result["ok"], result)
            self.assertTrue(result["numerical_valid"], result)
            self.assertFalse(result["valid"], result)
            self.assertTrue(result["grid_compensation_applied"], result)
            self.assertEqual(result["forward_model"], "grid_motor_positions_at")
            self.assertAlmostEqual(result["x_mm"], point[0], delta=1e-3)
            self.assertAlmostEqual(result["y_mm"], point[1], delta=1e-3)
            # The controller's default diagnostic gate is deliberately wider
            # than the exact-map floating-point residual.
            self.assertLessEqual(result["residual_max_deg"], 0.20)
            self.assertIs(calibration["valid"], False)

    def test_angle_quantisation_is_reported_with_bounded_residual(self):
        from test_y42_calibration_flow import isolated_web_module

        with tempfile.TemporaryDirectory(prefix="y42-xy-quantised-") as directory:
            web = isolated_web_module(directory)
            calibration = _synthetic_calibration(web)
            point = (78.4, 42.6)
            exact = web.grid_motor_positions_at(point[0], point[1], calibration)
            quantised = [round(value, 1) for value in exact]
            result = web.motor_targets_to_xy(
                quantised,
                calibration,
                initial_xy=(55.0, 74.0),
                finite_difference_step_mm=0.10,
                residual_tolerance_deg=0.20,
            )
            self.assertTrue(result["finite"], result)
            self.assertTrue(result["converged"], result)
            self.assertTrue(result["numerical_valid"], result)
            self.assertFalse(result["valid"], result)
            self.assertLessEqual(result["residual_max_deg"], 0.20)
            self.assertLess(math.hypot(result["x_mm"] - point[0], result["y_mm"] - point[1]), 0.10)
            self.assertIs(calibration["valid"], False)


if __name__ == "__main__":
    unittest.main()
