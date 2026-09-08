import unittest
import tempfile
from pathlib import Path

from analyze_y42_observation_run import analyze_events, read_events


def snapshot(t, positions):
    return {"captured_start": t, "captured_end": t + 0.01, "motors": [{"address": i + 1, "position": q, "rpm": 0, "phase_current": 180, "flags": 1} for i, q in enumerate(positions)]}


def window(index, positions):
    return {"event": "torque_observation_static_window", "vector_index": index, "currents_ma": [180] * 4, "window": [snapshot(index * 2 + dt, positions) for dt in (0, 0.25, 0.5)]}


class ObservationAnalysisTests(unittest.TestCase):
    def test_no_windows_does_not_claim_success_or_fit(self):
        result = analyze_events([])
        self.assertFalse(result["calibration_fit_performed"])
        self.assertFalse(result["absolute_ball_xy_accuracy_verified"])
        self.assertEqual(result["encoder_configuration_cluster_count"], 0)

    def test_duplicate_encoder_configurations_do_not_count_as_extra_poses(self):
        result = analyze_events([window(0, [0, 1, 2, 3]), window(1, [0.1, 1, 2, 3]), window(2, [5, -3, 2, 3])])
        self.assertEqual(result["encoder_configuration_cluster_count"], 2)
        self.assertEqual(result["conditional_3dof_bundle_global_rank_upper_bound"], 2)
        self.assertEqual(result["encoder_repeat_visits"], [[0, 1]])

    def test_timestamps_are_host_intervals_and_stability_excludes_batch_time(self):
        result = analyze_events([window(0, [0, 1, 2, 3])])
        self.assertIn("NOT device", result["clock_semantics"])
        self.assertAlmostEqual(result["static_windows"][0]["host_stability_coverage_s"], 0.49)
        self.assertFalse(result["static_windows"][0]["tautness_verified"])

    def test_missing_axis_is_rejected_without_fabricating_noise(self):
        event = window(0, [0, 1, 2, 3])
        event["window"][1]["motors"].pop()
        result = analyze_events([event])
        self.assertEqual(len(result["invalid_records"]), 1)
        self.assertEqual(result["encoder_configuration_cluster_count"], 0)
        self.assertNotIn("static_noise_mad_sigma_deg", result)

    def test_excursion_and_sampled_travel_are_reported_as_encoder_metrics(self):
        events = [{"event": "torque_observation_feedback", **snapshot(i, q)} for i, q in enumerate(([0] * 4, [5, -3, 2, 1], [1, -1, 0, 0]))]
        result = analyze_events(events)
        self.assertEqual(result["observed_excursion_from_first_deg"], [5, 3, 2, 1])
        self.assertEqual(result["sampled_cumulative_angle_lower_bound_deg"], [9, 5, 4, 2])

    def test_raw_rejected_guard_sample_is_not_lost_or_counted_as_accepted(self):
        raw = {"event": "torque_observation_raw_batch", **snapshot(0, [100, 0, 0, 0])}
        raw["motors"][0]["rpm"] = 26
        result = analyze_events([raw, {"event": "torque_observation_safety_stop", "reason": "speed"}])
        self.assertEqual(result["joint_feedback_records"], 0)
        self.assertEqual(result["raw_batches_not_accepted_as_feedback_count"], 1)
        self.assertEqual(result["all_parseable_raw_batches_including_rejected"]["all_feedback_maximum_abs_rpm"], 26)
        self.assertFalse(result["completion"]["sampling_complete_as_recorded"])

    def test_completion_requires_matching_all_vectors_and_end_record(self):
        begin = {"event": "torque_observation_begin", "vectors": [[180] * 4] * 2, "config": {}}
        result = analyze_events([begin, window(0, [0] * 4), {"event": "torque_observation_end", "status": "incomplete"}])
        self.assertEqual(result["completion"]["missing_requested_static_windows"], 1)
        self.assertFalse(result["completion"]["sampling_complete_as_recorded"])
        result = analyze_events([begin, window(0, [0] * 4), window(1, [1] * 4), {"event": "torque_observation_end", "status": "observed"}])
        self.assertTrue(result["completion"]["sampling_complete_as_recorded"])
        self.assertFalse(result["absolute_ball_xy_accuracy_verified"])

    def test_truncated_archive_keeps_prior_valid_records_but_exposes_issue(self):
        with tempfile.TemporaryDirectory() as temporary:
            event_file = Path(temporary) / "events.jsonl"
            event_file.write_text('{"event":"first"}\n{"event":', encoding="utf-8")
            events, issues = read_events(event_file)
        self.assertEqual(events, [{"event": "first"}])
        self.assertEqual(len(issues), 1)
        self.assertTrue(issues[0]["last_line"])
        self.assertEqual(issues[0]["line"], 2)

    def test_malformed_raw_batch_is_reported_not_silently_dropped(self):
        raw = {"event": "torque_observation_raw_batch", **snapshot(0, [1] * 4)}
        raw["motors"][0]["phase_current"] = None
        result = analyze_events([raw])
        self.assertEqual(result["raw_batch_records"], 1)
        self.assertEqual(result["structurally_parseable_raw_batches"], 0)
        self.assertEqual(len(result["invalid_records"]), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
