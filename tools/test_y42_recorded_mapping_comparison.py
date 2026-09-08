import copy
import unittest

import numpy as np

from compare_y42_recorded_mappings import make_geometry, model_lengths, reconstruct_joint_snapshots, solve_mapping


def calibration():
    return {"motor_center_coordinates_mm": [[-44, 148], [154, 148], [154, 0], [-44, 0]], "cable_exit_offset_x_mm": 12.5, "ball_attachment_span_x_mm": 30.534, "ball_attachment_span_y_mm": 22.823, "workspace_width": 110, "workspace_height": 148, "slope": [-4, 4, -5, 5], "intercept": [100, -100, 150, -150], "distance_samples_mm": [[0, 250]] * 4, "position_samples_deg": [[100, -900], [-100, 900], [150, -1100], [-150, 1100]]}


def raw(axis, q, t):
    packet = bytearray(37)
    packet[:4] = bytes((axis, 0x43, 0x25, 0x0C))
    packet[22] = int(q < 0)
    packet[23:27] = round(abs(q) * 10).to_bytes(4, "big")
    packet[-1] = 0x6B
    return {"address": axis, "position_deg": q, "rpm": 0, "phase_current_ma": 0, "sampled_monotonic_s": t, "raw_packet_hex": packet.hex()}


class RecordedMappingComparisonTests(unittest.TestCase):
    def test_linear_curve_equals_affine_for_positive_and_negative_axes(self):
        c = calibration()
        q = [20, 35, -100, 225]
        np.testing.assert_allclose(model_lengths(c, q, "legacy_piecewise"), model_lengths(c, q, "global_affine"))

    def test_joint_batch_preserves_raw_angles_without_smoothing(self):
        events = [{"sequence": 1, "event": "trajectory_feedback_stream_start"}]
        for i, q in enumerate([1.1, -2.2, 3.3, -4.4], 1):
            events.append({"sequence": i + 1, "event": "timed_feedback_frames", "samples": [raw(i, q, i * 0.01)]})
        events.append({"sequence": 6, "event": "trajectory_feedback_stream_stop"})
        batches, counts = reconstruct_joint_snapshots(events)
        self.assertEqual(batches[0]["positions_deg"], [1.1, -2.2, 3.3, -4.4])
        self.assertAlmostEqual(batches[0]["host_batch_span_ms"], 30)
        self.assertEqual(counts["raw_frames"], 4)

    def test_same_geometry_matches_legacy_corner_distance(self):
        predicted, _ = make_geometry(calibration())([0, 148, 0])
        self.assertAlmostEqual(predicted[0], 19.842696924813417)

    def test_solver_has_no_target_input_and_does_not_mutate_raw_or_calibration(self):
        c = calibration()
        original = copy.deepcopy(c)
        lengths, _ = make_geometry(c)([35, 65, 3])
        q = lengths * c["slope"] + c["intercept"]
        snapshots = [{"index": 0, "positions_deg": q.tolist()}]
        preserved = copy.deepcopy(snapshots)
        a = solve_mapping(c, snapshots, "legacy_piecewise")
        b = solve_mapping(c, snapshots, "global_affine")
        np.testing.assert_allclose(a["xy"], b["xy"], atol=1e-6)
        self.assertEqual(c, original)
        self.assertEqual(snapshots, preserved)
        self.assertLess(a["points"][0]["closure_rms_mm"], 1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
