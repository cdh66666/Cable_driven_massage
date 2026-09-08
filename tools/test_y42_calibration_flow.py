#!/usr/bin/env python3
"""Isolated calibration regressions: fake serial, fake clock, temporary data.

No serial port, network request, real calibration file or manual-test archive is
opened.  These tests measure software deadlines, not physical calibration time.
"""
from __future__ import annotations

import copy
import math
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import y42_pc_direct as direct


class FakeClock:
    def __init__(self):
        self.now = 100.0
        self.on_wait = None

    def monotonic(self):
        return self.now

    perf_counter = monotonic
    time = monotonic

    def perf_counter_ns(self):
        return int(self.now * 1_000_000_000)

    time_ns = perf_counter_ns

    def sleep(self, seconds):
        self.now += max(0.0, float(seconds))

    def wait(self, deadline):
        self.now = max(self.now, deadline)
        if self.on_wait:
            self.on_wait()


class FakeConnection:
    baudrate = 921600
    timeout = 0.01
    write_timeout = 0.1

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def write(self, *_):
        raise AssertionError("Unmocked serial write")


def isolated_web_module(directory):
    name = "y42_calibration_flow_isolated"
    module = types.ModuleType(name)
    module.__file__ = str(Path(directory) / "y42_pc_direct_web.py")
    sys.modules[name] = module
    source = (TOOLS / "y42_pc_direct_web.py").read_text(encoding="utf-8-sig")
    with patch.object(direct, "open_serial", side_effect=AssertionError("Serial forbidden during import")), \
         patch.object(direct, "set_traffic_callback"):
        exec(compile(source, module.__file__, "exec"), module.__dict__)
    return module


class CalibrationFlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="y42-calibration-offline-")
        cls.web = isolated_web_module(cls.temporary.name)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.w = self.web
        self.real_compute = self.w.compute_calibration
        self.real_structure_ready = self.w.persisted_calibration_structure_ready
        self.real_transact = direct.transact
        self.clock = FakeClock()
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.original_state = copy.deepcopy(self.w.STATE)
        self.addCleanup(lambda: self.w.STATE.update(self.original_state))
        self.w.STOP_EVENT.clear()
        self.addCleanup(self.w.STOP_EVENT.clear)
        self.sent = []
        self.logs = []
        self.disabled = []
        self.enable_calls = []
        self.corner = 0
        self.read_count = 0
        self.read_delay = 0.1
        self.read_hook = None
        self.currents = [150] * 4
        self.flags = 3
        self.position = [0.0] * 4
        self.connection = FakeConnection()
        self.calibration = self.w.blank_calibration()
        self.calibration.update({
            "valid": True,
            "internal_model_verified": True,
            "session_calibration_mode": self.w.SESSION_CALIBRATION_MODE,
            "session_id": self.w.PROCESS_SESSION_ID,
            "session_local_verified": True,
            "session_tension_held": True,
            "session_hold_current_ma": 150,
            "tension_checkpoint_positions_deg": [0.0] * 4,
            "tension_checkpoint_currents_ma": [150.0] * 4,
            "tension_checkpoint_controller": self.w.TENSION_CHECKPOINT_CONTROLLER_FOUR_POSITION_FORCE,
            "slope": [-4.7, 4.7, -4.7, 4.7],
            "r2": [0.999] * 4,
        })
        self.w.STATE.update(calibration=copy.deepcopy(self.calibration), auto="idle", corner=0)
        for name, value in {
            "time": self.clock,
            "current_port": lambda: "MOCK-NO-HARDWARE",
            "current_baud": lambda: 921600,
            "wait_tracking_deadline": self.clock.wait,
            "set_high_resolution_timer": lambda *_: False,
            "append_process_log": lambda event, **fields: self.logs.append((event, fields)),
            "read_all_reliable": self.read,
            "read_firmware_status": lambda _, address, **kw: {"id": address, "firmware": "X"},
            "enable_address": self.enable,
            "send_fast": self.send,
            "disable_all_with_reconnect": self.disable,
            "request_emergency_stop": self.request_stop,
            "persisted_calibration_structure_ready": lambda _: (True, ""),
            "mark_internal_model_ready": lambda calibration: dict(calibration, internal_model_verified=True),
            "position_to_xy": lambda *_: (55.0, 74.0),
            "compute_calibration": lambda *_: (copy.deepcopy(self.calibration), ""),
        }.items():
            self.stack.enter_context(patch.object(self.w, name, value))
        self.stack.enter_context(patch.object(direct, "open_serial", return_value=self.connection))
        self.stack.enter_context(patch.object(direct, "transact", side_effect=AssertionError("Unmocked serial transaction")))

    def read(self, connection, *args, **kwargs):
        self.read_count += 1
        deadline = kwargs.get("deadline")
        if deadline is not None and self.clock.now + self.read_delay >= deadline:
            self.clock.wait(deadline)
            raise TimeoutError("模拟串口读取已达到 5 秒总预算")
        self.clock.sleep(self.read_delay)
        if self.read_hook:
            self.read_hook()
        result = []
        for index, address in enumerate(self.w.MOTOR_IDS):
            item = direct.Telemetry(address)
            item.position = self.position[index]
            item.phase_current = self.currents[index]
            item.rpm = 0.0
            item.temperature = 25.0
            item.flags = self.flags
            result.append(item)
        return result

    def enable(self, _, address):
        self.enable_calls.append(address)
        return True, b"", b"ok"

    def send(self, _, packet):
        self.sent.append((self.clock.now, packet))

    def disable(self, *args, **kwargs):
        self.disabled.append(self.clock.now)
        return {"ok": True, "error": ""}

    def request_stop(self, *args, **kwargs):
        self.w.STOP_EVENT.set()
        return {"ok": True, "message": "模拟急停已请求"}

    def simulate_corners(self):
        original = self.w.switch_calibration_corner_force_vector

        def switch(connection, previous, corner, strong, hold):
            self.corner = corner + 1
            self.position = [float(self.corner * 600)] * 4
            self.currents = self.w.calibration_corner_current_vector(corner, strong, hold)
            return original(connection, previous, corner, strong, hold)

        self.stack.enter_context(patch.object(self.w, "switch_calibration_corner_force_vector", switch))

        # Final release lowers the actual simulated motor load to its target.
        original_ramp = self.w.smooth_torque_vector_ramp

        def ramp(connection, start, target, **kwargs):
            self.currents = list(target)
            return original_ramp(connection, start, target, **kwargs)

        self.stack.enter_context(patch.object(self.w, "smooth_torque_vector_ramp", ramp))

    def test_full_worker_soft_first_corner_then_three_single_switches(self):
        self.simulate_corners()
        self.w.calibration_worker(110, 148, 40, 900, 150)
        self.assertEqual(self.w.STATE["calibration_stage"], "complete", self.w.STATE.get("message"))
        ramps = [fields for event, fields in self.logs if event == "calibration_torque_ramp"]
        switches = [fields for event, fields in self.logs if event == "calibration_corner_force_switch"]
        self.assertEqual(len(ramps), 3)
        self.assertEqual([row["corner"] for row in switches], [2, 3, 4])
        entry = ramps[1]
        self.assertEqual((entry["start_max_rpm"], entry["max_rpm"]), (8.0, 80.0))
        self.assertLessEqual(entry["duration_s"], 1.3)
        self.assertLessEqual(ramps[2]["duration_s"], 1.3)
        events = [event for event, _ in self.logs]
        self.assertLess(events.index("calibration_hold_checked"), events.index("calibration_complete"))
        self.assertLess(events.index("tension_checkpoint_saved"), events.index("calibration_complete"))
        self.assertTrue(all(row["host_ramp"] is False for row in switches))
        self.assertFalse(self.disabled)
        self.assertTrue(all(packet[:2] == b"\x00\xaa" for _, packet in self.sent))

    def test_full_worker_corner_failure_stops_without_sampling_remaining_corners(self):
        self.simulate_corners()

        def break_read():
            if self.corner == 2:
                raise RuntimeError("模拟 M2 通讯中断")

        self.read_hook = break_read
        self.w.calibration_worker(110, 148, 40, 900, 150)
        self.assertEqual(self.w.STATE["calibration_stage"], "failed")
        self.assertIn("M2", self.w.STATE["calibration_last_error"])
        self.assertTrue(self.disabled)
        self.assertEqual(len([1 for event, _ in self.logs if event == "calibration_corner"]), 1)
        self.assertFalse(any(event == "tension_checkpoint_saved" for event, _ in self.logs))

    def test_full_worker_stop_does_not_continue_to_next_corner(self):
        self.simulate_corners()

        def stop_read():
            if self.corner == 2:
                self.w.STOP_EVENT.set()

        self.read_hook = stop_read
        self.w.calibration_worker(110, 148, 40, 900, 150)
        self.assertTrue(self.disabled)
        switches = [fields["corner"] for event, fields in self.logs if event == "calibration_corner_force_switch"]
        self.assertEqual(switches, [2])
        self.assertFalse(any(event == "tension_checkpoint_saved" for event, _ in self.logs))

    def test_full_worker_pending_stop_never_enables_motors(self):
        self.w.STOP_EVENT.set()
        self.w.calibration_worker(110, 148, 40, 900, 150)
        self.assertFalse(self.enable_calls, "Worker enabled motors despite a pending stop")
        self.assertFalse(self.sent)

    def test_full_worker_invalid_final_feedback_cannot_mark_success(self):
        self.simulate_corners()
        release = self.w.smooth_calibration_release

        def lose_feedback_after_release(*args):
            feedback = release(*args)
            for item in feedback:
                item.position = None
            return feedback

        with patch.object(self.w, "smooth_calibration_release", lose_feedback_after_release):
            self.w.calibration_worker(110, 148, 40, 900, 150)
        self.assertEqual(self.w.STATE["calibration_stage"], "failed")
        self.assertTrue(self.disabled)
        self.assertFalse(any(event == "tension_checkpoint_saved" for event, _ in self.logs))

    def test_first_corner_rpm_and_current_are_smooth_and_bounded(self):
        self.w.smooth_torque_vector_ramp(
            self.connection, [150] * 4, [900, 150, 150, 150],
            max_rpm=80, start_max_rpm=8,
            max_slew_ma_per_s=self.w.CALIBRATION_ENTRY_EXIT_SLEW_MA_PER_S,
            driver_slope_ma_per_s=self.w.CALIBRATION_HARD_SWITCH_DRIVER_SLOPE_MA_PER_S,
            label="离线首角包络", verify_safety=False,
        )
        rpms = [direct.read_u16(packet, 12) / 10 for _, packet in self.sent]
        currents = [direct.read_u16(packet, 9) for _, packet in self.sent]
        self.assertEqual((rpms[0], rpms[-1]), (8, 80))
        self.assertEqual((currents[0], currents[-1]), (150, 900))
        self.assertTrue(all(0 <= b - a <= 2.5 for a, b in zip(rpms, rpms[1:])))
        self.assertTrue(all(0 <= b - a <= 25 for a, b in zip(currents, currents[1:])))
        self.assertTrue(all(8 <= rpm <= 80 for rpm in rpms))
        self.assertTrue(all(150 <= current <= 900 for current in currents))

    def test_unloaded_final_hold_does_not_publish_or_persist_candidate(self):
        self.simulate_corners()
        candidate = dict(self.calibration, candidate_marker="must-not-publish")

        def drop_final_hold():
            if self.w.STATE.get("calibration_stage") == "final_soft_hold":
                self.assertNotIn("candidate_marker", self.w.STATE["calibration"])
                self.currents[2] = 0

        self.read_hook = drop_final_hold
        with patch.object(self.w, "compute_calibration", return_value=(candidate, "")), \
             patch.object(self.w, "persist_calibration") as persist:
            self.w.calibration_worker(110, 148, 40, 900, 150)
        self.assertEqual(self.w.STATE["calibration_stage"], "failed")
        self.assertIn("保持", self.w.STATE["calibration_last_error"])
        self.assertNotIn("candidate_marker", self.w.STATE["calibration"])
        persist.assert_not_called()
        self.assertFalse(any(event == "calibration_complete" for event, _ in self.logs))

    def test_persist_failure_does_not_publish_candidate(self):
        self.simulate_corners()
        candidate = dict(self.calibration, candidate_marker="must-not-publish")
        with patch.object(self.w, "compute_calibration", return_value=(candidate, "")), \
             patch.object(self.w, "persist_calibration", side_effect=OSError("模拟磁盘写入失败")):
            self.w.calibration_worker(110, 148, 40, 900, 150)
        self.assertEqual(self.w.STATE["calibration_stage"], "failed")
        self.assertIn("磁盘", self.w.STATE["calibration_last_error"])
        self.assertNotIn("candidate_marker", self.w.STATE["calibration"])
        self.assertFalse(any(event == "calibration_complete" for event, _ in self.logs))

    def test_stop_during_ramp_wait_prevents_next_motor_frame(self):
        before_stop = []

        def stop_during_wait():
            before_stop.append(len(self.sent))
            self.w.STOP_EVENT.set()

        self.clock.on_wait = stop_during_wait
        with self.assertRaisesRegex(RuntimeError, "停止"):
            self.w.smooth_torque_vector_ramp(self.connection, [20] * 4, [150] * 4,
                                           max_rpm=8, label="离线停止测试", verify_safety=False)
        self.assertEqual(len(self.sent), before_stop[0], "Stop arrived while waiting, but another force frame was sent")

    def test_stop_during_last_ramp_wait_prevents_endpoint_reassertion(self):
        plan, _ = self.w.calibration_torque_ramp_plan([20] * 4, [150] * 4)

        def stop_at_end():
            if len(self.sent) == len(plan):
                self.w.STOP_EVENT.set()

        self.clock.on_wait = stop_at_end
        with self.assertRaisesRegex(RuntimeError, "停止"):
            self.w.smooth_torque_vector_ramp(self.connection, [20] * 4, [150] * 4,
                                           max_rpm=8, label="离线尾帧停止测试", verify_safety=False)
        self.assertEqual(len(self.sent), len(plan))

    def test_enable_saved_failure_never_runs_full_calibration(self):
        with patch.object(self.w, "try_enable_saved_calibration_datum", return_value=(False, "模拟校验失败")), \
             patch.object(self.w, "calibration_worker", side_effect=AssertionError("Unexpected full calibration")):
            self.w.enable_saved_calibration_worker()
        self.assertEqual(self.w.STATE["auto"], "error")
        self.assertFalse(self.w.STATE["calibration_auto_fallback"])
        self.assertIn("模拟校验失败", self.w.STATE["message"])
        self.assertFalse(self.disabled, "An ordinary read-only failure must not drop holding torque")

    def test_saved_enable_stop_requests_emergency_disable(self):
        self.read_hook = self.w.STOP_EVENT.set
        self.w.enable_saved_calibration_worker()
        self.assertTrue(self.disabled)
        self.assertFalse(self.sent)
        self.assertEqual(self.w.STATE["calibration_stage"], "stopped")
        self.assertIn("失能已确认", self.w.STATE["message"])
        self.assertFalse(self.w.STATE["calibration"]["session_tension_held"])

    def test_saved_enable_stop_disable_failure_is_visible(self):
        self.read_hook = self.w.STOP_EVENT.set
        with patch.object(self.w, "disable_all_with_reconnect", return_value={"ok": False, "error": "模拟掉线"}):
            self.w.enable_saved_calibration_worker()
        self.assertEqual(self.w.STATE["auto"], "error")
        self.assertIn("无法确认", self.w.STATE["message"])
        self.assertIn("模拟掉线", self.w.STATE["message"])

    def test_failed_saved_check_closes_motion_gate_but_keeps_saved_fit_and_hold(self):
        frame = {"workspace_width": 110.0, "workspace_height": 148.0}
        anchors = tuple(zip(*self.w.workspace_anchor_coordinates(frame)))
        corners = tuple(zip(*self.w.workspace_corner_coordinates(frame)))
        slopes = [-4.7, 4.7, -4.7, 4.7]
        samples = [[100.0 + slopes[i] * math.hypot(x - ax, y - ay)
                    for i, (ax, ay) in enumerate(anchors)] for x, y in corners]
        calibration, reason = self.real_compute(samples, 110, 148, 40)
        self.assertIsNotNone(calibration, reason)
        calibration.update(tension_checkpoint_positions_deg=[0.0] * 4,
                           tension_checkpoint_currents_ma=[150.0] * 4)
        self.assertTrue(self.w.internal_model_readiness(calibration)[0])
        self.w.STATE["calibration"] = calibration
        self.flags = 2
        self.w.enable_saved_calibration_worker()
        result = self.w.STATE["calibration"]
        self.assertFalse(self.w.internal_model_readiness(result)[0])
        self.assertTrue(self.real_structure_ready(result)[0], "Fit must stay saved after failed live check")
        self.assertIn("失能", result["session_invalid_reason"])
        self.assertFalse(self.sent)
        self.assertFalse(self.disabled)

    def test_saved_enable_keeps_live_controller_and_does_not_send_torque(self):
        enabled, reason = self.w.try_enable_saved_calibration_datum()
        self.assertTrue(enabled, reason)
        self.assertFalse(self.enable_calls, "Existing live controller must not be re-enabled")
        self.assertFalse(self.sent, "Read-only enable check must not overwrite four-position hold")
        self.assertFalse(self.disabled)

    def test_saved_enable_rejects_disabled_motors_without_enabling_them(self):
        self.flags = 2
        enabled, reason = self.w.try_enable_saved_calibration_datum()
        self.assertFalse(enabled, reason)
        self.assertFalse(self.enable_calls)
        self.assertFalse(self.sent)

    def test_saved_enable_has_one_end_to_end_five_second_budget(self):
        self.read_delay = 2.0
        started = self.clock.now
        enabled, reason = self.w.try_enable_saved_calibration_datum()
        self.assertFalse(enabled, "Budget exhaustion must never become successful enable")
        self.assertLessEqual(self.clock.now - started, 5.0, f"Total fake elapsed exceeds budget: {reason}")
        self.assertFalse(self.sent)

    def test_saved_enable_deadline_includes_serial_lock_wait(self):
        class BusyLock:
            def acquire(_, *, timeout):
                self.clock.sleep(timeout)
                return False

        with patch.object(self.w, "SERIAL_LOCK", BusyLock()):
            enabled, reason = self.w.try_enable_saved_calibration_datum()
        self.assertFalse(enabled)
        self.assertIn("占用", reason)
        self.assertLessEqual(self.w.STATE["calibration_elapsed_seconds"], 5.0)
        self.assertEqual(self.read_count, 0)
        self.assertFalse(self.sent)

    def test_saved_enable_rejects_known_lost_session_without_query_or_motion(self):
        self.w.STATE["calibration"]["session_tension_held"] = False
        self.w.STATE["calibration"]["session_invalid_reason"] = "用户明确已断电"
        enabled, reason = self.w.try_enable_saved_calibration_datum()
        self.assertFalse(enabled)
        self.assertIn("断电", reason)
        self.assertEqual(self.read_count, 0)
        self.assertFalse(self.enable_calls)
        self.assertTrue(self.w.STATE["calibration"]["valid"], "Persisted fit must remain available")

    def test_quick_enable_api_without_saved_model_never_starts_full(self):
        with patch.object(self.w, "persisted_calibration_structure_ready", return_value=(False, "没有保存的标定")), \
             patch.object(self.w, "require_control_unlocked"), \
             patch.object(self.w, "require_confirmed"), \
             patch.object(self.w, "start_thread") as start:
            with self.assertRaisesRegex(ValueError, "未启动完整校准"):
                self.w.start_auto({"action": "calibrate", "activate_only": True})
        start.assert_not_called()
        self.assertFalse(self.sent)

    def test_snapshot_separates_saved_fit_from_lost_session(self):
        frame = {"workspace_width": 110.0, "workspace_height": 148.0}
        anchors = tuple(zip(*self.w.workspace_anchor_coordinates(frame)))
        corners = tuple(zip(*self.w.workspace_corner_coordinates(frame)))
        slopes = [-4.7, 4.7, -4.7, 4.7]
        samples = [[100.0 + slopes[i] * math.hypot(x - ax, y - ay)
                    for i, (ax, ay) in enumerate(anchors)] for x, y in corners]
        result, reason = self.real_compute(samples, 110, 148, 40)
        self.assertIsNotNone(result, reason)
        result.update(session_tension_held=False, session_local_verified=False)
        self.w.STATE["calibration"] = result
        with patch.object(self.w, "persisted_calibration_structure_ready", self.real_structure_ready):
            snapshot = self.w.web_state_snapshot()
        self.assertTrue(snapshot["saved_model_available"], snapshot["saved_model_reason"])
        self.assertFalse(snapshot["calibration"]["session_tension_held"])

    def test_busy_request_lock_rejects_run_but_not_stop(self):
        with self.w.AUTO_REQUEST_LOCK:
            with self.assertRaisesRegex(RuntimeError, "处理中"):
                self.w.start_auto({"action": "calibrate"})
            result = self.w.start_auto({"action": "abort"})
        self.assertTrue(result["ok"])
        self.assertTrue(self.w.STOP_EVENT.is_set())
        self.assertFalse(self.sent)

    def test_serial_read_deadline_restores_timeouts_and_preserves_partial_traffic(self):
        clock = self.clock

        class QueryPort:
            timeout = 20.0
            write_timeout = 20.0
            baudrate = 921600
            in_waiting = 0
            first = True

            def reset_input_buffer(self):
                pass

            def write(self, packet):
                return len(packet)

            def flush(self):
                raise AssertionError("Deadline query must not use unbounded flush")

            def read(self, length):
                if self.first:
                    self.first = False
                    clock.sleep(0.01)
                    return b"\x01\x36"
                clock.sleep(self.timeout)
                return b""

        connection = QueryPort()
        records = []
        with patch.object(direct, "time", clock), patch.object(direct, "_emit_traffic", records.append):
            with self.assertRaises(TimeoutError):
                self.real_transact(connection, b"\x01\x36\x6b", first_byte_timeout=30,
                                   inter_byte_quiet=30, deadline=clock.now + 0.2)
        self.assertLessEqual(clock.now, 100.2)
        self.assertEqual((connection.timeout, connection.write_timeout), (20.0, 20.0))
        self.assertEqual(records[-1]["tx_hex"], "01366b")
        self.assertEqual(records[-1]["rx_hex"], "0136")
        self.assertIn("TimeoutError", records[-1]["error"])

    def test_hot_adoption_rejects_disabled_flag_even_with_current(self):
        self.flags = 2
        ok, reason = self.w.adopt_persisted_tensioned_calibration(self.connection)
        self.assertFalse(ok, reason)
        self.assertFalse(any(event == "calibration_hot_resumed" for event, _ in self.logs))

    def test_hot_adoption_requires_encoder_checkpoint(self):
        self.w.STATE["calibration"].pop("tension_checkpoint_positions_deg")
        ok, reason = self.w.adopt_persisted_tensioned_calibration(self.connection)
        self.assertFalse(ok, reason)

    def test_hot_adoption_rejects_nonfinite_encoder_checkpoint(self):
        self.w.STATE["calibration"]["tension_checkpoint_positions_deg"] = [float("nan")] * 4
        ok, reason = self.w.adopt_persisted_tensioned_calibration(self.connection)
        self.assertFalse(ok, reason)

    def test_current_evidence_rejects_nonfinite_readings(self):
        for value in (float("nan"), float("inf")):
            with self.subTest(current=value):
                ok, reason = self.w.persisted_checkpoint_tension_evidence(self.calibration, [value] * 4)
                self.assertFalse(ok, reason)


if __name__ == "__main__":
    unittest.main(verbosity=2)
