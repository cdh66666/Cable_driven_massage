"""Entirely fake IO/clock; never imports the controller or serial libraries."""
from dataclasses import replace
import math
import unittest

from y42_position_probe import ProbeConfig, ProbeSafetyError, run_position_probe


class FakeIO:
    def __init__(self):
        self.now = 10.0
        self.q0 = [120.0, -90.0, 50.0, -40.0]
        self.positions = list(self.q0)
        self.commands = []
        self.emergencies = []
        self.events = []
        self.current = 20
        self.transform = lambda rows: rows
        self.respond = True
        self.send_delay = 0.0
        self.frozen_timestamp = None

    def clock(self):
        return self.now

    def sleep(self, seconds):
        assert seconds >= 0.0
        self.now += seconds

    def send(self, targets, current, rpm):
        self.commands.append((self.now, list(targets), current, rpm))
        if self.respond:
            self.positions = list(targets)
        self.current = current
        self.now += self.send_delay

    def snapshot(self):
        stamp = self.now if self.frozen_timestamp is None else self.frozen_timestamp
        return self.transform([{"id": axis + 1, "position": value, "rpm": 0.0,
            "phase_current": self.current, "temperature": 28.0, "flags": 3,
            "sampled_at": stamp} for axis, value in enumerate(self.positions)])

    def emergency(self, reason):
        self.emergencies.append((self.now, reason))
        return {"disabled_ack": True}

    def run(self, config=None):
        return run_position_probe(self.q0, snapshot=self.snapshot, send_positions=self.send,
            clock=self.clock, sleep=self.sleep, emergency_stop=self.emergency,
            emit=self.events.append, config=config)


class PositionProbeTests(unittest.TestCase):
    def test_default_four_axes_respond_and_final_20ma_static_window(self):
        io = FakeIO()
        result = io.run()
        self.assertTrue(result["success"])
        self.assertTrue(result["final_hold_verified"])
        self.assertFalse(result["xyz_precision_measured"])
        self.assertFalse(result["tension_verified"])
        self.assertEqual([item["id"] for item in result["axis_results"]], [1, 2, 3, 4])
        for item in result["axis_results"]:
            self.assertAlmostEqual(item["observed_signed_response_deg"], 0.5)
            self.assertAlmostEqual(item["peak_error_deg"], 0.0)
            self.assertAlmostEqual(item["return_error_deg"], 0.0)
        self.assertLess(result["elapsed_seconds"], 5.5)
        self.assertEqual(io.commands[-1][1:], (io.q0, 20, 1.0))
        self.assertEqual(len(result["static_windows"]), 9)
        final_window = result["static_windows"][-1]
        self.assertEqual(final_window["stage"], "final_20ma_hold")
        self.assertGreaterEqual(len(final_window["samples"]), 3)
        self.assertGreaterEqual(final_window["samples"][-1]["host_time"] - final_window["samples"][0]["host_time"], 0.15 - 1e-8)
        self.assertEqual(io.emergencies, [])

    def test_only_one_axis_moves_at_once_and_slew_is_bounded(self):
        io = FakeIO()
        result = io.run(ProbeConfig(axis_signs=(1, -1, 1, -1)))
        for _, targets, current, rpm in io.commands:
            changed = [abs(value - zero) > 1e-8 for value, zero in zip(targets, io.q0)]
            self.assertLessEqual(sum(changed), 1)
            self.assertTrue(20 <= current <= 150)
            self.assertEqual(rpm, 1.0)
            self.assertTrue(all(abs(value - zero) <= 0.5 + 1e-9 for value, zero in zip(targets, io.q0)))
        for first, second in zip(io.commands, io.commands[1:]):
            dt = second[0] - first[0]
            self.assertGreaterEqual(dt, 0.009999)
            self.assertLessEqual(abs(second[2] - first[2]), 7)
            self.assertLessEqual(max(abs(b - a) for a, b in zip(first[1], second[1])) / dt / 6.0, 1.0 + 1e-7)
        self.assertEqual([row["sign"] for row in result["axis_results"]], [1, -1, 1, -1])

    def test_amplitude_two_degrees_stretches_profile_under_one_rpm(self):
        io = FakeIO()
        result = io.run(ProbeConfig(amplitude_deg=2.0))
        self.assertTrue(result["success"])
        self.assertLessEqual(result["elapsed_seconds"], 8.0)
        for first, second in zip(io.commands, io.commands[1:]):
            dt = second[0] - first[0]
            self.assertGreater(dt, 0.0)
            self.assertLessEqual(max(abs(b - a) for a, b in zip(first[1], second[1])) / dt / 6.0, 1.0 + 1e-7)

    def test_150ma_entry_is_allowed_but_still_finishes_at_20(self):
        io = FakeIO()
        result = io.run(ProbeConfig(entry_current_ma=150))
        self.assertTrue(result["success"])
        self.assertEqual(io.commands[0][2], 150)
        self.assertEqual(io.commands[-1][2], 20)

    def test_no_response_is_not_passed_and_no_amplitude_retry(self):
        io = FakeIO()
        io.respond = False
        with self.assertRaisesRegex(ProbeSafetyError, "未证实跟随") as caught:
            io.run()
        self.assertFalse(caught.exception.result["success"])
        self.assertEqual(len(io.emergencies), 1)
        self.assertTrue(all(targets[1:] == io.q0[1:] for _, targets, _, _ in io.commands))
        self.assertLessEqual(max(abs(targets[0] - io.q0[0]) for _, targets, _, _ in io.commands), 0.5 + 1e-9)

    def test_response_of_point_one_degree_is_explicitly_unproven(self):
        io = FakeIO()
        io.respond = False
        def small_response(rows):
            if io.commands and io.commands[-1][1][0] > io.q0[0] + 0.01:
                rows[0]["position"] += 0.1
            return rows
        io.transform = small_response
        with self.assertRaisesRegex(ProbeSafetyError, "0.1度"):
            io.run()
        self.assertEqual(len(io.emergencies), 1)

    def test_raw_guards_faults_stale_span_and_nan(self):
        variants = [
            ("rpm", 3.1), ("phase_current", 351), ("temperature", 45),
            ("position", 124.1), ("position", float("nan")), ("flags", 2),
            ("flags", 0x0B), ("flags", 0x43), ("flags", 0xE2),
            ("sampled_at", 9.79), ("sampled_at", 10.03),
        ]
        for key, value in variants:
            with self.subTest(key=key, value=value):
                io = FakeIO()
                io.transform = lambda rows: [dict(rows[0], **{key: value})] + rows[1:]
                with self.assertRaises(ProbeSafetyError):
                    io.run()
                self.assertEqual(len(io.emergencies), 1)
                self.assertEqual(len(io.commands), 0)
        io = FakeIO()
        io.transform = lambda rows: [dict(rows[0], sampled_at=io.now - 0.11)] + rows[1:]
        with self.assertRaisesRegex(ProbeSafetyError, "跨度"):
            io.run()

    def test_duplicate_or_missing_feedback_rejected(self):
        for transform in (lambda rows: rows[:3], lambda rows: [rows[0]] * 4):
            io = FakeIO()
            io.transform = transform
            with self.assertRaises(ProbeSafetyError):
                io.run()
            self.assertEqual(len(io.emergencies), 1)

    def test_error_over_one_degree_for_point_three_seconds_trips(self):
        io = FakeIO()
        def lag(rows):
            if io.now >= 10.05:
                rows[0]["position"] -= 1.1
            return rows
        io.transform = lag
        with self.assertRaisesRegex(ProbeSafetyError, "持续0.3秒"):
            io.run()
        self.assertLess(io.now, 10.5)

    def test_final_current_drop_requires_static_feedback(self):
        io = FakeIO()
        def restless_finish(rows):
            if io.now > 13.0 and io.current <= 20:
                for row in rows:
                    row["rpm"] = 0.4
            return rows
        io.transform = restless_finish
        with self.assertRaisesRegex(ProbeSafetyError, "连续静止") as caught:
            io.run()
        self.assertFalse(caught.exception.result["final_hold_verified"])
        self.assertEqual(len(io.emergencies), 1)

    def test_frozen_feedback_cannot_create_a_static_window(self):
        io = FakeIO()
        io.frozen_timestamp = 10.0
        with self.assertRaisesRegex(ProbeSafetyError, "陈旧"):
            io.run()
        self.assertEqual(len(io.emergencies), 1)

    def test_slow_io_skips_ticks_instead_of_catchup_and_deadline_stops(self):
        io = FakeIO()
        io.send_delay = 0.015
        result = io.run()
        self.assertGreater(result["skipped_ticks"], 0)
        gaps = [second[0] - first[0] for first, second in zip(io.commands, io.commands[1:])]
        self.assertGreaterEqual(min(gaps), 0.015 - 1e-8)
        io = FakeIO()
        with self.assertRaisesRegex(ProbeSafetyError, "总时限") as caught:
            io.run(ProbeConfig(total_seconds=0.5))
        self.assertLessEqual(io.now, 10.5 + 1e-8)
        self.assertEqual(len(io.emergencies), 1)
        self.assertFalse(caught.exception.result["success"])

    def test_settle_boundaries_keep_absolute_tick_with_slow_io(self):
        """A settle phase must not add its IO time on top of the next tick."""
        io = FakeIO()
        io.send_delay = 0.028
        config = ProbeConfig(tick_seconds=0.04)

        result = io.run(config)

        self.assertTrue(result["success"])
        settle_stages = {
            "M1_peak", "M1_returned", "M2_peak", "M2_returned",
            "M3_peak", "M3_returned", "M4_peak", "M4_returned",
            "final_20ma_hold",
        }
        boundaries = []
        for previous, current in zip(result["commands"], result["commands"][1:]):
            if current["stage"] in settle_stages and current["stage"] != previous["stage"]:
                boundaries.append((current["stage"],
                                   current["scheduled_send_time"] - previous["scheduled_send_time"]))

        self.assertEqual([stage for stage, _ in boundaries], [
            "M1_peak", "M1_returned", "M2_peak", "M2_returned",
            "M3_peak", "M3_returned", "M4_peak", "M4_returned",
            "final_20ma_hold",
        ])
        for _, interval in boundaries:
            self.assertAlmostEqual(interval, config.tick_seconds, places=7)
        self.assertEqual(result["skipped_ticks"], 0)

    def test_send_failure_and_unknown_emergency_do_not_report_success(self):
        io = FakeIO()
        def send_error(*args):
            raise OSError("write failed")
        def emergency_error(reason):
            io.emergencies.append(reason)
            raise OSError("disable unknown")
        io.send = send_error
        io.emergency = emergency_error
        with self.assertRaisesRegex(ProbeSafetyError, "write failed") as caught:
            io.run()
        self.assertEqual(caught.exception.result["emergency_error"], "disable unknown")
        self.assertFalse(caught.exception.result["success"])

    def test_config_cannot_weaken_hard_limits(self):
        invalid = [replace(ProbeConfig(), amplitude_deg=2.01), replace(ProbeConfig(), max_rpm=1.1),
            replace(ProbeConfig(), settle_seconds=0.1), replace(ProbeConfig(), final_current_ma=150),
            replace(ProbeConfig(), probe_current_ma=200), replace(ProbeConfig(), total_seconds=9),
            replace(ProbeConfig(), feedback_max_age_seconds=0.5)]
        for config in invalid:
            with self.subTest(config=config):
                io = FakeIO()
                with self.assertRaises(ProbeSafetyError):
                    io.run(config)
                self.assertEqual(len(io.commands), 0)
                self.assertEqual(len(io.emergencies), 1)


if __name__ == "__main__":
    unittest.main()
