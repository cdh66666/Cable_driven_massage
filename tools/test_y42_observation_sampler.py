"""Only synthetic IO/clock; no controller imports, serial, network or files."""
from dataclasses import replace
import unittest

from y42_observation_sampler import ObservationConfig, plan_observation_vectors, run_observation_sampler


class FakeClock:
    def __init__(self):
        self.t = 100.0
        self.on_wait = None

    def now(self):
        return self.t

    def wait_until(self, deadline):
        self.t = max(self.t, deadline)
        if self.on_wait:
            self.on_wait()


class FakeIO:
    def __init__(self, clock):
        self.clock = clock
        self.commands = []
        self.events = []
        self.emergencies = []
        self.count = 0
        self.read_delay = 0.01
        self.mutate = None
        self.on_send = None
        self.on_emit = None
        self.read_deadlines = []

    def send_torque(self, currents_ma, *, max_rpm, deadline):
        self.commands.append((self.clock.now(), tuple(currents_ma), max_rpm, deadline))
        if self.on_send:
            return self.on_send()
        return True

    def read_snapshot(self, *, deadline):
        self.read_deadlines.append(deadline)
        begin = self.clock.now()
        self.clock.t += self.read_delay
        self.count += 1
        motors = [dict(address=i + 1, position=0.0, rpm=0.0, phase_current=150.0, temperature=29.0, flags=3) for i in range(4)]
        batch = dict(captured_start=begin, captured_end=self.clock.now(), motors=motors)
        if self.mutate:
            self.mutate(batch, self.count)
        return batch

    def emit(self, event, **fields):
        self.events.append((event, fields))
        if self.on_emit:
            self.on_emit(event, fields)

    def emergency_stop(self, reason):
        self.emergencies.append(reason)
        return {"requested": True, "disabled_acknowledged": False}


class ObservationSamplerTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.io = FakeIO(self.clock)
        self.stopped = False
        self.vectors = [(225, 150, 150, 150), (150, 225, 150, 150)]

    def run_plan(self, **kwargs):
        return run_observation_sampler(self.vectors, self.io, self.clock, lambda: self.stopped,
                                       initial_torque_currents_ma=(150, 150, 150, 150), **kwargs)

    def final_hold_started(self):
        return any(event == "torque_observation_final_hold_begin" for event, _ in self.io.events)

    def test_normal_soft_end_observation_only_and_not_independent_pose_claim(self):
        result = self.run_plan()
        self.assertEqual(result["status"], "observed")
        self.assertEqual(len(result["samples"]), 2)
        self.assertFalse(result["calibration_published"])
        self.assertFalse(result["absolute_pose_verified"])
        self.assertIsNone(result["independent_pose_count"])
        self.assertEqual(self.io.commands[-1][1], (150,) * 4)
        self.assertFalse(self.io.emergencies)
        self.assertLessEqual(result["elapsed_s"], 15.0)
        self.assertTrue(result["final_hold_verified"])
        window = result["final_hold_window"]
        self.assertGreaterEqual(len(window), 3)
        self.assertGreaterEqual(window[-1]["captured_start"] - window[0]["captured_end"], 0.4)
        self.assertEqual(len(result["final_hold_raw_batches"]), len(window))
        self.assertTrue(all(row["phase"] == "final_hold_verify" for row in window))
        self.assertTrue(all(row["command_currents_ma"] == [150] * 4 for row in window))
        self.assertTrue(any(event == "torque_observation_final_hold_window" and fields["verified"]
                            for event, fields in self.io.events))
        for previous, following in zip(self.io.commands, self.io.commands[1:]):
            self.assertGreaterEqual(following[0] - previous[0] + 1e-9, 0.05)
            self.assertLessEqual(max(abs(a - b) for a, b in zip(previous[1], following[1])), 21)
        self.assertTrue(all(command[2] <= 8 for command in self.io.commands))

    def test_bad_modes_or_out_of_range_vectors_cannot_send(self):
        with self.assertRaises(ValueError):
            run_observation_sampler(self.vectors, self.io, self.clock, lambda: False, initial_torque_currents_ma=None)
        for vector in [(500, 150, 150, 150), (0, 150, 150, 150), (150, 150), (151.2, 150, 150, 150)]:
            with self.assertRaises(ValueError):
                plan_observation_vectors([vector])
        self.assertFalse(self.io.commands)

    def test_stop_during_wait_sends_no_following_motion(self):
        self.clock.on_wait = lambda: setattr(self, "stopped", True)
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertFalse(self.io.commands)
        self.assertEqual(len(self.io.emergencies), 1)

    def test_each_guard_emergencies_and_preserves_bad_raw_batch(self):
        cases = [
            lambda batch, n: batch["motors"].pop(),
            lambda batch, n: batch["motors"][0].update(rpm=26),
            lambda batch, n: batch["motors"][0].update(phase_current=401),
            lambda batch, n: batch["motors"][0].update(temperature=45),
            lambda batch, n: batch["motors"][0].update(flags=0),
            lambda batch, n: batch["motors"][0].update(flags=11),
            lambda batch, n: batch["motors"][0].update(position=float("nan")),
            lambda batch, n: batch.update(captured_start=batch["captured_end"] - 2),
        ]
        for mutation in cases:
            with self.subTest(mutation=mutation):
                self.setUp()
                self.io.mutate = mutation
                result = self.run_plan()
                self.assertEqual(result["status"], "safety_stopped")
                self.assertEqual(len(result["raw_batches"]), 1)
                self.assertFalse(self.io.commands)
                self.assertEqual(len(self.io.emergencies), 1)

    def test_excursion_and_cumulative_travel_are_separate_limits(self):
        self.io.mutate = lambda batch, n: batch["motors"][0].update(position=91 if n > 1 else 0)
        result = self.run_plan()
        self.assertIn("90°", result["reason"])
        self.setUp()
        self.io.mutate = lambda batch, n: batch["motors"][0].update(position=(50 if n % 2 == 0 else -50) if n > 1 else 0)
        result = self.run_plan()
        self.assertIn("180°", result["reason"])

    def test_no_static_window_returns_gently_without_escalating(self):
        self.io.mutate = lambda batch, n: batch["motors"][0].update(rpm=0 if self.final_hold_started() else 2)
        result = self.run_plan()
        self.assertEqual(result["status"], "incomplete")
        self.assertEqual(result["reason"], "static_window_not_obtained")
        self.assertFalse(result["samples"])
        self.assertEqual(self.io.commands[-1][1], (150,) * 4)
        self.assertFalse(self.io.emergencies)
        self.assertTrue(result["final_hold_verified"])
        self.assertTrue(all(max(command[1]) <= 225 for command in self.io.commands))

    def test_final_hold_never_settles_requests_stop_without_more_force(self):
        self.io.mutate = lambda batch, n: batch["motors"][0].update(rpm=2)
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn("连续静止", result["reason"])
        self.assertFalse(result["final_hold_verified"])
        self.assertGreaterEqual(len(result["final_hold_raw_batches"]), 3)
        self.assertEqual(result["final_hold_window"], [])
        self.assertEqual(len(self.io.emergencies), 1)
        self.assertEqual(self.io.commands[-1][1], (150,) * 4)
        self.assertTrue(all(max(command[1]) <= 225 for command in self.io.commands))
        begin = next(fields for event, fields in self.io.events if event == "torque_observation_final_hold_begin")
        self.assertLessEqual(self.clock.now(), begin["deadline"] + 1e-9)

    def test_final_hold_zero_rpm_with_encoder_drift_is_not_static(self):
        def drift(batch, n):
            if self.final_hold_started():
                batch["motors"][0]["position"] = len([row for row in self.io.events if row[0] == "torque_observation_final_hold_window"]) * 0.9
        self.io.mutate = drift
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertFalse(result["final_hold_verified"])
        self.assertLessEqual(len(result["final_hold_window"]), 1)
        self.assertGreaterEqual(len(result["final_hold_raw_batches"]), 3)
        self.assertEqual(len(self.io.emergencies), 1)

    def test_two_final_batches_even_spanning_window_cannot_verify_hold(self):
        config = replace(ObservationConfig(), feedback_interval_s=0.4, maximum_dwell_s=0.7)
        result = self.run_plan(config=config)
        self.assertEqual(len(result["final_hold_window"]), 2)
        window = result["final_hold_window"]
        self.assertGreaterEqual(window[-1]["captured_start"] - window[0]["captured_end"], 0.4)
        self.assertFalse(result["final_hold_verified"])
        self.assertEqual(result["status"], "safety_stopped")

    def test_final_hold_read_receives_local_deadline_and_late_data_rejected(self):
        def slow_final_read(event, fields):
            if event == "torque_observation_final_hold_begin":
                self.io.read_delay = 0.6
        self.io.on_emit = slow_final_read
        result = self.run_plan(config=replace(ObservationConfig(), maximum_dwell_s=0.5))
        hold = next(fields for event, fields in self.io.events if event == "torque_observation_final_hold_begin")
        self.assertEqual(self.io.read_deadlines[-1], hold["deadline"])
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn("deadline", result["reason"])
        self.assertFalse(result["final_hold_verified"])
        self.assertEqual(len(result["final_hold_raw_batches"]), 1)
        self.assertEqual(len(self.io.emergencies), 1)

    def test_stop_during_final_window_does_not_continue_torque_commands(self):
        final_command_count = []
        def stop_in_final_wait():
            if self.final_hold_started():
                final_command_count.append(len(self.io.commands))
                self.stopped = True
        self.clock.on_wait = stop_in_final_wait
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn("用户停止", result["reason"])
        self.assertFalse(result["final_hold_verified"])
        self.assertEqual(len(result["final_hold_raw_batches"]), 1)
        self.assertEqual(len(self.io.commands), final_command_count[0])
        self.assertEqual(len(self.io.emergencies), 1)

    def test_repeated_final_batch_rejected_and_raw_retained(self):
        first_final = []
        def repeated_final(batch, n):
            if self.final_hold_started():
                if not first_final:
                    first_final.append((batch["captured_start"], batch["captured_end"]))
                else:
                    batch["captured_start"], batch["captured_end"] = first_final[0]
        self.io.mutate = repeated_final
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn("重复", result["reason"])
        self.assertFalse(result["final_hold_verified"])
        self.assertEqual(len(result["final_hold_raw_batches"]), 2)
        self.assertEqual(len(result["final_hold_window"]), 1)

    def test_stop_at_final_window_publication_cannot_publish_normal_hold(self):
        def stop_before_publish(event, fields):
            if event == "torque_observation_final_hold_window" and fields["verified"]:
                self.stopped = True
        self.io.on_emit = stop_before_publish
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertFalse(result["final_hold_verified"])
        self.assertEqual(len(self.io.emergencies), 1)
        self.assertNotIn("final_command_currents_ma", result)

    def test_deadline_covers_read_and_send_callbacks(self):
        self.io.read_delay = 0.7
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn("deadline", result["reason"])
        self.setUp()
        def late_send():
            self.clock.t += 0.7
        self.io.on_send = late_send
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn("deadline", result["reason"])

    def test_total_budget_reserves_normal_end_instead_of_running_more_vectors(self):
        self.vectors = self.vectors * 3
        result = self.run_plan(config=replace(ObservationConfig(), total_budget_s=6.0))
        self.assertEqual(result["status"], "incomplete")
        self.assertLess(len(result["samples"]), 6)
        self.assertLessEqual(result["elapsed_s"], 6.0)
        self.assertEqual(self.io.commands[-1][1], (150,) * 4)
        self.assertFalse(self.io.emergencies)
        self.assertTrue(result["final_hold_verified"])

    def test_six_vectors_and_serial_batch_latency_do_not_claim_twelve_poses(self):
        self.vectors = self.vectors * 3
        self.io.read_delay = 0.2
        result = self.run_plan()
        self.assertLessEqual(result["elapsed_s"], 15.0)
        self.assertFalse(result["emergency_requested"])
        self.assertIsNone(result["independent_pose_count"])
        self.assertEqual(self.io.commands[-1][1], (150,) * 4)
        self.assertTrue(result["final_hold_verified"])

    def test_repeated_zero_duration_feedback_is_not_fresh(self):
        self.io.mutate = lambda batch, n: batch.update(captured_start=100.0, captured_end=100.0)
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn("重复", result["reason"])
        self.assertFalse(result["samples"])

    def test_failed_send_never_continues_or_claims_normal_hold(self):
        self.io.on_send = lambda: False
        result = self.run_plan()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertEqual(len(self.io.commands), 1)
        self.assertEqual(len(self.io.emergencies), 1)
        self.assertNotIn("final_command_currents_ma", result)


if __name__ == "__main__":
    unittest.main()
