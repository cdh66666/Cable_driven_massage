"""Small in-memory event tests; no archive or controller writes."""
import copy
import unittest

from review_y42_position_probe import summarize_events


def aa(targets):
    body = b""
    for axis, target in enumerate(targets, 1):
        body += bytes((axis, 0xCB, int(target < 0))) + b"\x00\x0a" + round(abs(target) * 10).to_bytes(4, "big") + b"\x01\x00\x00\x96\x6b"
    return b"\x00\xaa\x00\x3d" + body + b"\x6b"


def write(packet, stamp, phase="probe_positions", count=None):
    return {"event": "position_probe_serial_write", "phase": phase, "attempted_tx_hex": packet.hex(),
            "bytes_written": len(packet) if count is None else count, "expected_bytes": len(packet),
            "started_monotonic_ns": round(stamp * 1e9), "completed_monotonic_ns": round((stamp + 0.001) * 1e9)}


def feedback(stamps):
    return {"rows": [{"id": axis + 1, "position": 0.0, "sampled_at": stamp} for axis, stamp in enumerate(stamps)]}


class ProbeReviewTests(unittest.TestCase):
    def test_no_motion_cannot_be_zero_error_success(self):
        result = {"success": True, "final_hold_verified": True, "final_disabled_confirmed": True,
                  "configuration_restored": True, "axis_results": [{"id": axis, "observed_signed_response_deg": 0.5,
                      "peak_error_deg": 0.0, "return_error_deg": 0.0} for axis in range(1, 5)]}
        summary = summarize_events([{"event": "run_end", "result": result}])
        self.assertEqual(summary["execution"], "not_executed")
        self.assertFalse(summary["encoder_probe_pass_evidenced"])
        self.assertTrue(all(axis["peak_error_deg"] is None for axis in summary["axes"]))
        self.assertIsNone(summary["raw_host_write_start_timing"]["hz"])

    def test_handshake_cb_and_enable_are_not_movement(self):
        events = [write(bytes((1, 0xF3, 0xAB, 1, 0, 0x6B)), 1.0, "enable"),
                  write(aa([0.0] * 4), 1.1, "initial_hold"), {"event": "run_end", "result": {}}]
        summary = summarize_events(events)
        self.assertEqual(summary["execution"], "handshake_only")
        self.assertEqual(summary["enable_tx_complete_write_axes"], [1])
        self.assertEqual(summary["cb_tx_complete_write_axes"], [1, 2, 3, 4])
        self.assertFalse(summary["encoder_probe_pass_evidenced"])

    def test_dedicated_writes_dedup_duplicate_direct_traffic_and_keep_both_times(self):
        events = [write(aa([0.0] * 4), 10.0), write(aa([0.5, 0, 0, 0]), 10.01), write(aa([0.0] * 4), 10.02)]
        events += [{"event": "serial_transaction", "tx_hex": event["attempted_tx_hex"],
                    "started_monotonic_ns": event["started_monotonic_ns"], "completed_monotonic_ns": event["completed_monotonic_ns"]} for event in list(events)]
        events.append({"event": "run_end", "result": {"commands": [{"scheduled_send_time": 9.0,
                            "completed_send_time": 9.03}, {"scheduled_send_time": 9.2, "completed_send_time": 9.3}]}})
        summary = summarize_events(events)
        self.assertEqual(summary["raw_host_write_start_timing"]["count"], 3)
        self.assertAlmostEqual(summary["raw_host_write_start_timing"]["hz"], 100.0)
        self.assertAlmostEqual(summary["raw_host_write_start_timing"]["interval_p95_ms"], 10.0)
        self.assertAlmostEqual(summary["module_callback_start_timing"]["hz"], 5.0)
        self.assertEqual(len(summary["raw_probe_write_timestamps"]), 3)
        self.assertEqual(summary["module_callback_timestamps"][0]["completed_send_time"], 9.03)

    def test_cached_snapshots_and_partially_new_groups_are_separate(self):
        rows = [feedback([10.0] * 4)] * 10
        rows += [feedback([10.05, 10, 10, 10])] * 3
        rows += [feedback([10.1] * 4)] * 4
        summary = summarize_events([{"event": "run_end", "result": {"feedback": rows}}])
        self.assertEqual(summary["feedback_logged_groups"], 17)
        self.assertEqual(summary["unique_four_timestamp_groups"]["count"], 3)
        self.assertEqual(summary["fully_new_four_axis_groups"]["count"], 2)
        self.assertEqual(summary["cached_or_duplicate_feedback_groups"], 14)
        self.assertAlmostEqual(summary["fully_new_four_axis_groups"]["hz"], 10.0)

    def test_microprobe_command_phase_keeps_raw_write_clock(self):
        events = [write(aa([0.0] * 4), 10.0, "microprobe_command_stream"),
                  write(aa([0.5, 0, 0, 0]), 10.01, "microprobe_command_stream"),
                  {"event": "run_end", "result": {"success": False}}]
        summary = summarize_events(events)
        self.assertEqual(summary["execution"], "motion_commands_sent")
        self.assertEqual(summary["raw_host_write_start_timing"]["count"], 2)
        self.assertAlmostEqual(summary["raw_host_write_start_timing"]["hz"], 100.0)
        self.assertFalse(summary["encoder_probe_pass_evidenced"])

    def test_four_axis_pass_requires_response_and_confirmed_cleanup(self):
        events = [write(aa([0.0] * 4), 10.0)]
        for axis in range(4):
            targets = [0.0] * 4
            targets[axis] = 0.5
            events.extend([write(aa(targets), 10.01 + axis * 0.02), write(aa([0.0] * 4), 10.02 + axis * 0.02)])
        result = {"success": True, "final_hold_verified": True, "final_disabled_confirmed": True,
                  "configuration_restored": True, "original_configurations_hex": {"1": "..."},
                  "axis_results": [{"id": axis, "observed_signed_response_deg": 0.5,
                      "peak_error_deg": 0.0, "return_error_deg": 0.0} for axis in range(1, 5)]}
        events.append({"event": "run_end", "result": result})
        summary = summarize_events(events)
        self.assertTrue(summary["encoder_probe_pass_evidenced"])
        self.assertFalse(summary["xyz_precision_measured"])
        for changed in ({"configuration_restored": False}, {"final_disabled_confirmed": "unknown"},
                        {"axis_results": result["axis_results"][:3]}):
            other = copy.deepcopy(events)
            other[-1]["result"].update(changed)
            self.assertFalse(summarize_events(other)["encoder_probe_pass_evidenced"])

    def test_partial_write_error_does_not_count_as_successful_tx(self):
        event = write(aa([0.5] * 4), 10.0, count=10)
        summary = summarize_events([event, {"event": "run_end", "result": {}}])
        self.assertEqual(summary["cb_tx_attempted_axes"], [1, 2, 3, 4])
        self.assertEqual(summary["cb_tx_complete_write_axes"], [])
        self.assertFalse(summary["encoder_probe_pass_evidenced"])
        self.assertEqual(summary["raw_host_write_start_timing"]["count"], 0)

    def test_recorded_raw_limit_fault_is_reported_even_without_adapter_reason(self):
        sample = feedback([10.0] * 4)
        sample["rows"][0].update(rpm=3.2, phase_current=351.0, temperature=45.0)
        summary = summarize_events([{"event": "run_end", "result": {"feedback": [sample]}}])
        self.assertEqual(summary["axes"][0]["observed_maxima"],
                         {"abs_rpm": 3.2, "abs_phase_current_ma": 351.0, "temperature_c": 45.0})
        self.assertTrue(any("M1 recorded feedback exceeds" in alert for alert in summary["alerts"]))
        self.assertFalse(summary["encoder_probe_pass_evidenced"])


if __name__ == "__main__":
    unittest.main()
