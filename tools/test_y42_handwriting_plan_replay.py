"""No export or hardware calls: planning-report consistency only."""
import hashlib
import math
import unittest

import y42_handwriting_plan_replay as replay


class PlanReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source_bytes = replay.SOURCE.read_bytes()
        cls.report = replay.build_report(cls.source_bytes)

    def test_exact_source_and_fixed_geometry_identity(self):
        self.assertEqual(self.report["source_sha256"], hashlib.sha256(self.source_bytes).hexdigest())
        self.assertEqual(self.report["geometry_sha256"], "6c0b45bd429ebcccc49b5676bcc95b6cf4b48bdf9a8a908d9050644a3c7a1565")
        self.assertEqual(self.report["input_point_count"], 138)
        self.assertFalse(self.report["hardware_executed"])
        self.assertFalse(self.report["calibration_or_motor_limit_validated"])

    def test_actual_normalized_conversion_and_refinement(self):
        for normalized, absolute in zip(self.report["input_normalized_segments"], self.report["production_converted_input_mm"]):
            for point, (x, y) in zip(normalized, absolute):
                self.assertEqual(x, 110.0 * point["x"])
                self.assertEqual(y, 148.0 * (1.0 - point["y"]))
        self.assertEqual(self.report["filtered_point_count"], 138)
        self.assertGreater(self.report["refined_point_count"], 138)

    def test_three_speeds_phase_sum_distances_and_frames(self):
        runs = self.report["runs"]
        self.assertEqual([run["requested_draw_speed_mm_s"] for run in runs], [12.5, 25.0, 50.0])
        for run in runs:
            self.assertEqual(run["travel_speed_mm_s"], run["requested_draw_speed_mm_s"])
            self.assertEqual([p["kind"] for p in run["phases"]], ["draw", "travel", "draw", "travel", "draw", "travel", "draw"])
            self.assertAlmostEqual(sum(p["duration_s"] for p in run["phases"]), run["total_plan_time_s"])
            self.assertAlmostEqual(sum(p["distance_mm"] for p in run["phases"]), run["draw_distance_mm"] + run["travel_distance_mm"])
            self.assertEqual(run["expected_100hz_command_frames"], math.ceil(run["duration_ms"] / 10.0))
            self.assertEqual(run["draw_distance_mm"], runs[0]["draw_distance_mm"])
            self.assertEqual(run["travel_distance_mm"], runs[0]["travel_distance_mm"])
            for phase in run["phases"]:
                self.assertEqual(phase["start_speed_mm_s"], 0.0)
                self.assertEqual(phase["end_speed_mm_s"], 0.0)
        self.assertGreater(runs[0]["total_plan_time_s"], runs[1]["total_plan_time_s"])
        self.assertGreaterEqual(runs[1]["total_plan_time_s"], runs[2]["total_plan_time_s"])

    def test_every_phase_boundary_continuous_and_transfers_not_ink(self):
        for run in self.report["runs"]:
            for before, after in zip(run["phases"], run["phases"][1:]):
                self.assertEqual(before["end_s"], after["start_s"])
                self.assertEqual(before["end_xy_mm"], after["start_xy_mm"])
            self.assertEqual(sum(p["kind"] == "draw" for p in run["phases"]), 4)


if __name__ == "__main__":
    unittest.main()
