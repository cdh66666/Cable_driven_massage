"""Production AST reader/guard tests: fake byte source only; never serial/API IO."""
import ast
import copy
import math
from pathlib import Path
import queue
import threading
import types
import unittest

SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


def record(address=2, **patch):
    values = dict(address=address, sampled_at=10.0, position=100.0, rpm=0.0,
        phase_current=150, temperature=30, flags=1, raw_packet_hex="02430000")
    values.update(patch)
    return types.SimpleNamespace(**values)


class IsolatedRuntime:
    def __init__(self, batch=()):
        self.now = 10.022
        self.raw, self.events, self.order = [], [], []
        self.batch = list(batch)
        self.env = {"math": math, "time": types.SimpleNamespace(perf_counter=lambda: self.now),
            "threading": threading, "queue": queue, "TimedFeedbackRecord": object,
            "TIMED_FEEDBACK_FRAME_BYTES": 31, "WINDOWS_THREAD_PRIORITY_BELOW_NORMAL": 0,
            "set_current_thread_priority": lambda value: None,
            "_consume_timed_position_buffer": self.parse,
            "preserve_raw_timed_feedback": self.preserve}
        names = {"AsyncTimedFeedbackReader", "ObservationRawRecordGuard"}
        nodes = [node for node in TREE.body if isinstance(node, ast.ClassDef) and node.name in names]
        assert {node.name for node in nodes} == names
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), self.env)

    def parse(self, buffer, **kwargs):
        buffer.clear()
        return self.batch[:], None

    def preserve(self, batch):
        self.order.append("raw_saved")
        self.raw.extend(copy.deepcopy(batch))

    def guard(self, stop=None, emit=None):
        return self.env["ObservationRawRecordGuard"]([100.0] * 4,
            stop if stop is not None else threading.Event(),
            emit=emit if emit is not None else lambda event, **fields: self.events.append((event, copy.deepcopy(fields))))

    def run_reader(self, on_record=None):
        runtime = self
        class FakeConnection:
            timeout = 0.001
            def __init__(self):
                self.used = False
                self.writes = []
            def read(self, count):
                if not self.used:
                    self.used = True
                    return b"fake-parser-input"
                reader.stop_event.set()
                return b""
            def write(self, data):
                self.writes.append(data)
                raise AssertionError("reader must not write serial")
        connection = FakeConnection()
        reader = self.env["AsyncTimedFeedbackReader"](connection, on_record=on_record)
        reader.start()
        reader.thread.join(timeout=1.0)
        assert not reader.thread.is_alive(), "fake reader did not terminate"
        return reader, connection


class PerRecordGuardTests(unittest.TestCase):
    def test_twenty_ms_speed_pulse_is_latched_despite_latest_normal_record(self):
        pulse = record(rpm=-101.7, sampled_at=10.000)
        normal = record(rpm=0.0, sampled_at=10.020)
        runtime = IsolatedRuntime([pulse, normal])
        stop = threading.Event(); guard = runtime.guard(stop)
        def observe(item):
            runtime.order.append("guard_called")
            guard(item)
        reader, connection = runtime.run_reader(observe)
        self.assertTrue(stop.is_set())
        self.assertEqual(runtime.order[0], "raw_saved")
        self.assertEqual([item.rpm for item in runtime.raw], [-101.7, 0.0])
        self.assertEqual(reader.drain()[0].rpm, -101.7, "rejected record stays available")
        self.assertEqual(guard.failure["record"]["rpm"], -101.7)
        with self.assertRaisesRegex(RuntimeError, "101.7.*25RPM"):
            reader.check()
        with self.assertRaisesRegex(RuntimeError, "101.7.*25RPM"):
            guard(normal)
        self.assertEqual(len(runtime.events), 1)
        self.assertEqual(connection.writes, [])
        # This is an injected clock example, NOT a real-device latency bound.
        self.assertAlmostEqual(guard.failure["detected_at"] - pulse.sampled_at, 0.022)

    def test_default_reader_without_callback_keeps_previous_unfiltered_behavior(self):
        runtime = IsolatedRuntime([record(rpm=-101.7), record(rpm=0.0, sampled_at=10.02)])
        reader, connection = runtime.run_reader()
        reader.check()
        self.assertEqual([item.rpm for item in reader.drain()], [-101.7, 0.0])
        self.assertEqual(len(runtime.raw), 2)
        self.assertEqual(connection.writes, [])

    def test_first_failure_record_and_reason_are_never_replaced(self):
        runtime = IsolatedRuntime(); stop = threading.Event(); guard = runtime.guard(stop)
        with self.assertRaises(RuntimeError): guard(record(rpm=25.1))
        first = copy.deepcopy(guard.failure)
        stop.clear(); runtime.now += 0.020
        with self.assertRaises(RuntimeError): guard(record(temperature=90))
        self.assertTrue(stop.is_set())
        self.assertEqual(guard.failure, first)
        self.assertEqual(len(runtime.events), 1)

    def test_speed_current_temperature_position_and_flags_are_checked_per_record(self):
        cases = [({"rpm": 25.1}, "25RPM"), ({"rpm": -25.1}, "25RPM"),
            ({"phase_current": 400.1}, "400mA"), ({"phase_current": -400.1}, "400mA"),
            ({"temperature": 45}, "45"), ({"position": 190.1}, "90"),
            ({"position": 9.9}, "90"), ({"flags": 0}, "失能"),
            ({"flags": 9}, "驱动保护"), ({"flags": 0xE2}, "状态字无效"),
            ({"flags": 1.5}, "状态字无效")]
        for fields, message in cases:
            with self.subTest(fields=fields):
                runtime = IsolatedRuntime(); stop = threading.Event(); guard = runtime.guard(stop)
                with self.assertRaisesRegex(RuntimeError, message): guard(record(**fields))
                self.assertTrue(stop.is_set())

    def test_missing_and_nonfinite_fields_fail_closed(self):
        for field in ("position", "rpm", "phase_current", "temperature", "flags", "sampled_at"):
            for value in (None, float("nan"), float("inf")):
                with self.subTest(field=field, value=value):
                    guard = IsolatedRuntime().guard()
                    with self.assertRaisesRegex(RuntimeError, field): guard(record(**{field: value}))
                    self.assertTrue(guard.stop_event.is_set())

    def test_unknown_motor_ids_do_not_escape_without_stop(self):
        for address in (None, 0, 5, True, 1.0):
            guard = IsolatedRuntime().guard()
            with self.assertRaisesRegex(RuntimeError, "编号"):
                guard(record(address=address))
            self.assertTrue(guard.stop_event.is_set())

    def test_exact_inclusive_boundaries_remain_valid(self):
        guard = IsolatedRuntime().guard()
        guard(record(rpm=25, phase_current=400, temperature=44.99, position=190))
        self.assertIsNone(guard.failure)
        self.assertFalse(guard.stop_event.is_set())

    def test_intentional_disable_relaxes_only_enabled_expectation(self):
        guard = IsolatedRuntime().guard(); guard.require_enabled = False
        guard(record(flags=0))
        with self.assertRaisesRegex(RuntimeError, "25RPM"):
            guard(record(flags=0, rpm=26))
        self.assertTrue(guard.stop_event.is_set())

    def test_secondary_log_failure_cannot_cancel_stop_latch(self):
        def failing_log(*args, **kwargs): raise OSError("fake log failure")
        guard = IsolatedRuntime().guard(emit=failing_log)
        with self.assertRaisesRegex(RuntimeError, "25RPM"): guard(record(rpm=26))
        self.assertTrue(guard.stop_event.is_set())
        self.assertIsNotNone(guard.failure)

    def test_callback_sees_raw_record_only_after_preservation(self):
        runtime = IsolatedRuntime([record()])
        callback_orders = []
        def callback(item):
            callback_orders.append(runtime.order[:])
        reader, connection = runtime.run_reader(callback)
        reader.check()
        self.assertEqual(callback_orders, [["raw_saved"]])
        self.assertEqual(connection.writes, [])


if __name__ == "__main__":
    unittest.main()
