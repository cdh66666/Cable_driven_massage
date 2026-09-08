"""Pure finite-difference checks; no controller imports or hardware I/O."""
import math
import unittest

from y42_spool_geometry import (
    cylindrical_payout,
    fixed_z_yaw_xy_roundtrip,
    platform_payouts_fixed_z,
    planar_platform_payouts,
    solve_fixed_z_yaw_xy,
    unwrap_near,
)


class SpoolGeometryTests(unittest.TestCase):
    @staticmethod
    def _fixed_geometry():
        axes = [(-44, 148), (154, 148), (154, 0), (-44, 0)]
        attachments = [
            (-15.267, 11.4115),
            (15.267, 11.4115),
            (15.267, -11.4115),
            (-15.267, -11.4115),
        ]
        branches = [1, -1, 1, -1]
        references = [
            math.atan2(74 + offset[1] - axis[1], 55 + offset[0] - axis[0])
            for axis, offset in zip(axes, attachments)
        ]
        return axes, attachments, branches, references

    def test_tangent_radius_orthogonality_and_unit_gradient(self):
        for branch in (-1, 1):
            for endpoint in ((80, 60), (-80, 60), (-80, -60), (80, -60), (12.1, 0.0)):
                item = cylindrical_payout((0, 0), endpoint, 12, branch, reference_phi_rad=0)
                tx, ty = item["tangent_xy_mm"]
                px, py = endpoint
                gx, gy = item["gradient_xy"]
                self.assertAlmostEqual(math.hypot(tx, ty), 12, places=10)
                self.assertAlmostEqual(tx * (px - tx) + ty * (py - ty), 0, places=9)
                self.assertAlmostEqual(math.hypot(gx, gy), 1, places=10)
                self.assertAlmostEqual(gx, (px - tx) / item["free_span_mm"], places=10)
                self.assertAlmostEqual(gy, (py - ty) / item["free_span_mm"], places=10)

    def test_cartesian_gradient_matches_full_payout_finite_difference(self):
        epsilon = 1e-4
        for branch in (-1, 1):
            for x, y in ((80, 60), (-80, 60), (-80, -60), (80, -60)):
                kwargs = dict(radius_mm=12, branch=branch, reference_phi_rad=math.atan2(y, x))
                item = cylindrical_payout((0, 0), (x, y), **kwargs)
                for index in range(2):
                    before, after = [x, y], [x, y]
                    before[index] -= epsilon
                    after[index] += epsilon
                    numerical = (cylindrical_payout((0, 0), after, **kwargs)["length_mm"] - cylindrical_payout((0, 0), before, **kwargs)["length_mm"]) / (2 * epsilon)
                    self.assertAlmostEqual(numerical, item["gradient_xy"][index], places=8)

    def test_azimuth_derivative_includes_changing_wrapped_arc(self):
        epsilon = 1e-5
        for branch in (-1, 1):
            lengths = [cylindrical_payout((0, 0), (100 * math.cos(phi), 100 * math.sin(phi)), 12, branch, reference_phi_rad=0)["length_mm"] for phi in (-epsilon, epsilon)]
            self.assertAlmostEqual((lengths[1] - lengths[0]) / (2 * epsilon), -branch * 12, places=8)

    def test_reference_removes_atan2_negative_axis_jump(self):
        for branch in (-1, 1):
            before = cylindrical_payout((0, 0), (-100, 1e-5), 12, branch, reference_phi_rad=math.pi)
            after = cylindrical_payout((0, 0), (-100, -1e-5), 12, branch, reference_phi_rad=math.pi)
            self.assertLess(abs(after["length_mm"] - before["length_mm"]), 1e-4)
            self.assertAlmostEqual(unwrap_near(-math.pi + 0.01, math.pi), math.pi + 0.01)

    def test_reference_alpha_changes_only_additive_datum(self):
        first = cylindrical_payout((0, 0), (80, 60), 12, 1, reference_phi_rad=0)
        second = cylindrical_payout((0, 0), (80, 60), 12, 1, reference_phi_rad=0, reference_alpha_rad=0.8)
        self.assertAlmostEqual(second["length_mm"] - first["length_mm"], 9.6)
        self.assertEqual(first["gradient_xy"], second["gradient_xy"])

    def test_platform_jacobian_matches_three_pose_directions(self):
        axes = [(-44, 148), (154, 148), (154, 0), (-44, 0)]
        attachments = [(-15.267, 11.4115), (15.267, 11.4115), (15.267, -11.4115), (-15.267, -11.4115)]
        args = (axes, attachments, [12] * 4, [1, -1, 1, -1], [-math.pi / 4, -3 * math.pi / 4, 3 * math.pi / 4, math.pi / 4])
        pose = [22.0, 29.6, 0.4]  # Third coordinate is degrees at the API boundary.
        epsilon = 1e-4
        expected = planar_platform_payouts(*pose, *args)
        for index in range(3):
            before, after = pose[:], pose[:]
            delta = math.degrees(epsilon) if index == 2 else epsilon
            before[index] -= delta
            after[index] += delta
            low = planar_platform_payouts(*before, *args)["lengths_mm"]
            high = planar_platform_payouts(*after, *args)["lengths_mm"]
            for motor in range(4):
                    self.assertAlmostEqual((high[motor] - low[motor]) / (2 * epsilon), expected["jacobian_xy_yaw_rad"][motor][index], places=6)

    def test_fixed_exit_jacobian_matches_four_pose_directions(self):
        axes, attachments, branches, references = self._fixed_geometry()
        pose = [43.0, 81.0, 28.0, 0.13]  # x/y/z mm, yaw radians.
        expected = platform_payouts_fixed_z(
            *pose,
            axes,
            attachments,
            [12.0] * 4,
            branches,
            references,
            exit_z_mm=10.5,
        )
        epsilon = 1e-5
        for column in range(4):
            before, after = pose[:], pose[:]
            before[column] -= epsilon
            after[column] += epsilon
            low = platform_payouts_fixed_z(
                *before,
                axes,
                attachments,
                [12.0] * 4,
                branches,
                references,
                exit_z_mm=10.5,
            )["lengths_mm"]
            high = platform_payouts_fixed_z(
                *after,
                axes,
                attachments,
                [12.0] * 4,
                branches,
                references,
                exit_z_mm=10.5,
            )["lengths_mm"]
            for motor in range(4):
                numerical = (high[motor] - low[motor]) / (2 * epsilon)
                self.assertAlmostEqual(
                    numerical,
                    expected["jacobian_xyz_yaw_rad"][motor][column],
                    places=6,
                )

    def test_fixed_xy_inverse_roundtrip_without_quantization(self):
        axes, attachments, branches, references = self._fixed_geometry()
        forward = platform_payouts_fixed_z(
            43.0,
            81.0,
            28.0,
            0.13,
            axes,
            attachments,
            [12.0] * 4,
            branches,
            references,
            exit_z_mm=10.5,
        )
        solved = solve_fixed_z_yaw_xy(
            forward["lengths_mm"],
            28.0,
            0.13,
            axes,
            attachments,
            [12.0] * 4,
            branches,
            references,
            exit_z_mm=10.5,
            initial_xy=(55.0, 74.0),
        )
        self.assertTrue(solved["converged"])
        self.assertAlmostEqual(solved["x_mm"], 43.0, places=8)
        self.assertAlmostEqual(solved["y_mm"], 81.0, places=8)
        self.assertLessEqual(solved["residual_max_mm"], 1e-8)

    def test_fixed_xy_roundtrip_with_0p1_degree_quantization(self):
        axes, attachments, branches, references = self._fixed_geometry()
        result = fixed_z_yaw_xy_roundtrip(
            43.0,
            81.0,
            28.0,
            0.13,
            axes,
            attachments,
            [12.0] * 4,
            branches,
            references,
            exit_z_mm=10.5,
            angle_resolution_deg=0.1,
        )
        self.assertTrue(result["converged"])
        self.assertFalse(result["hardware_validated"])
        self.assertLessEqual(result["max_angle_error_deg"], 0.050000001)
        self.assertLessEqual(result["max_length_error_mm"], 12.0 * math.radians(0.05) + 1e-10)
        self.assertLessEqual(result["solver"]["residual_max_mm"], result["solver_tolerance_mm"])

    def test_fixed_geometry_requires_finite_and_converged_inputs(self):
        axes, attachments, branches, references = self._fixed_geometry()
        with self.assertRaises(ValueError):
            platform_payouts_fixed_z(
                43.0,
                81.0,
                float("nan"),
                0.13,
                axes,
                attachments,
                [12.0] * 4,
                branches,
                references,
            )
        with self.assertRaises(ValueError):
            solve_fixed_z_yaw_xy(
                [float("nan")] * 4,
                28.0,
                0.13,
                axes,
                attachments,
                [12.0] * 4,
                branches,
                references,
            )
        with self.assertRaises(ValueError):
            solve_fixed_z_yaw_xy(
                [0.0, 0.0, 0.0, 0.0],
                28.0,
                0.13,
                axes,
                attachments,
                [12.0] * 4,
                branches,
                references,
                max_iterations=2,
            )

    def test_invalid_radius_branch_or_endpoint_is_rejected(self):
        for radius, branch, endpoint in ((0, 1, (80, 60)), (12, 0, (80, 60)), (12, True, (80, 60)), (12, 1, (12, 0)), (12, 1, (0, 0)), (float("nan"), 1, (80, 60))):
            with self.assertRaises(ValueError):
                cylindrical_payout((0, 0), endpoint, radius, branch, reference_phi_rad=0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
