#!/usr/bin/env python3
"""Offline emergency-stop dispatch regressions; never opens serial or network.

The page module is executed with temporary persistence paths.  Emergency
threads are captured, then run synchronously against a mocked disable/ACK
boundary.  This verifies software admission/dispatch, not physical stopping.
"""
from __future__ import annotations

import copy
from contextlib import ExitStack
import socket
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

from test_y42_calibration_flow import FakeClock, direct, isolated_web_module


class CapturedThread:
    """A thread whose work runs only when the test explicitly calls run()."""

    def __init__(self, *, target, args=(), kwargs=None, name=None, daemon=None):
        self.target = target
        self.args = args
        self.kwargs = kwargs or {}
        self.name = name
        self.daemon = daemon
        self.started = False
        self.finished = False

    def start(self):
        self.started = True

    def is_alive(self):
        return self.started and not self.finished

    def run(self):
        if not self.started:
            raise AssertionError("Captured thread was never started")
        try:
            return self.target(*self.args, **self.kwargs)
        finally:
            self.finished = True

    def join(self, timeout=None):
        # Captured work cannot accidentally become an OS thread or block QA.
        return None


class EmergencyDispatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="y42-emergency-offline-")
        with patch.object(direct.serial, "Serial", side_effect=AssertionError("Serial forbidden")), \
             patch.object(socket, "socket", side_effect=AssertionError("Network forbidden")):
            cls.web = isolated_web_module(cls.temporary.name)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.w = self.web
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        initial_state = copy.deepcopy(self.w.STATE)

        def restore_state():
            self.w.STATE.clear()
            self.w.STATE.update(initial_state)

        self.addCleanup(restore_state)
        for event_name in ("STOP_EVENT", "SOFT_STOP_EVENT", "EMERGENCY_STOP_PENDING", "BOOT_PREPARE_DONE"):
            event = getattr(self.w, event_name)
            original = event.is_set()
            event.clear()
            self.addCleanup(event.set if original else event.clear)
        self.w.BOOT_PREPARE_DONE.set()
        self.w.STATE.update(
            port="MOCK-NO-HARDWARE",
            baud=921600,
            auto="idle",
            control_unlocked=True,
            emergency_stop_acknowledged=None,
            message="",
        )
        self.stack.enter_context(patch.object(self.w, "AUTO_THREAD", None))
        if hasattr(self.w, "EMERGENCY_STOP_THREAD"):
            self.stack.enter_context(patch.object(self.w, "EMERGENCY_STOP_THREAD", None))
        self.clock = FakeClock()
        self.threads = []
        self.logs = []
        self.disable = Mock(return_value={"ok": True, "error": "", "results": []})

        def create_thread(**kwargs):
            thread = CapturedThread(**kwargs)
            self.threads.append(thread)
            return thread

        thread_api = types.SimpleNamespace(**vars(self.w.threading))
        thread_api.Thread = create_thread
        self.stack.enter_context(patch.object(self.w, "threading", thread_api))
        self.stack.enter_context(patch.object(self.w, "time", self.clock))
        self.stack.enter_context(patch.object(self.w, "current_port", return_value="MOCK-NO-HARDWARE"))
        self.stack.enter_context(patch.object(self.w, "current_baud", return_value=921600))
        self.stack.enter_context(patch.object(self.w, "disable_all_with_reconnect", self.disable))
        self.stack.enter_context(patch.object(self.w, "append_process_log", side_effect=lambda event, **fields: self.logs.append((event, fields))))
        for owner, attribute in (
            (direct, "open_serial"),
            (direct, "transact"),
            (direct.serial, "Serial"),
            (direct.list_ports, "comports"),
            (socket, "socket"),
            (socket, "create_connection"),
        ):
            self.stack.enter_context(patch.object(owner, attribute, side_effect=AssertionError("Unmocked hardware/network access")))
        self.stack.enter_context(patch.object(direct, "choose_port", return_value="MOCK-NO-HARDWARE"))

    def request(self):
        response = self.w.request_emergency_stop()
        self.assertIs(response.get("ok"), True)
        self.assertEqual(response.get("action"), "stop")
        self.assertIs(response.get("pending"), True)
        return response

    def run_dispatch(self):
        self.assertEqual(len(self.threads), 1, "One emergency request must publish one independent dispatcher")
        self.threads[0].run()

    def test_late_stop_while_auto_worker_only_archives_still_dispatches_disable(self):
        # The old worker has already left its motor-control finally block, but
        # remains alive while finishing a valuable user-test archive.
        archive_worker = Mock()
        archive_worker.is_alive.return_value = True
        archive_worker.join.side_effect = lambda timeout=None: self.clock.sleep(timeout or 0)
        self.w.AUTO_THREAD = archive_worker
        self.request()
        self.assertTrue(self.w.EMERGENCY_STOP_PENDING.is_set())
        self.assertFalse(self.disable.called, "HTTP acknowledgement must not synchronously wait for hardware")
        self.run_dispatch()
        self.disable.assert_called_once()
        self.assertIs(self.w.STATE.get("emergency_stop_acknowledged"), True)
        self.assertFalse(self.w.EMERGENCY_STOP_PENDING.is_set())

    def test_old_worker_clearing_stop_flag_cannot_cancel_independent_dispatch(self):
        self.request()
        self.w.STOP_EVENT.clear()  # Simulates a racing old worker's finally.
        self.run_dispatch()
        self.disable.assert_called_once()
        self.assertIs(self.w.STATE.get("emergency_stop_acknowledged"), True)

    def test_duplicate_pending_requests_publish_only_one_dispatcher(self):
        self.request()
        self.request()
        self.request()
        self.run_dispatch()
        self.disable.assert_called_once()

    def test_negative_ack_is_visible_and_does_not_unlock_controls(self):
        self.disable.return_value = {"ok": False, "error": "模拟 M3 无失能回包", "results": []}
        self.request()
        self.run_dispatch()
        self.assertIs(self.w.STATE.get("emergency_stop_acknowledged"), False)
        self.assertIs(self.w.STATE.get("control_unlocked"), False)
        self.assertRegex(self.w.STATE.get("message", ""), r"切断|断电|硬件急停")
        with self.assertRaises(RuntimeError):
            self.w.require_control_unlocked()

    def test_quick_stop_failure_retries_and_only_publishes_final_ack(self):
        self.disable.side_effect = [
            {"ok": False, "error": "模拟首轮缺少回包", "results": []},
            {"ok": True, "error": "", "results": []},
        ]
        self.request()
        self.assertIsNone(self.w.STATE.get("emergency_stop_acknowledged"))
        self.run_dispatch()
        self.assertEqual(self.disable.call_count, 2)
        self.assertIs(self.disable.call_args_list[0].kwargs.get("clear_timed_streams"), False)
        self.assertIsNot(self.disable.call_args_list[1].kwargs.get("clear_timed_streams"), False)
        self.assertIs(self.w.STATE.get("emergency_stop_acknowledged"), True)
        self.assertIs(self.w.STATE.get("control_unlocked"), False)

    def test_dispatch_thread_start_failure_remains_visible_and_locked(self):
        with patch.object(CapturedThread, "start", side_effect=RuntimeError("模拟线程启动失败")):
            with self.assertRaises(RuntimeError):
                self.w.request_emergency_stop()
        self.disable.assert_not_called()
        self.assertIs(self.w.STATE.get("emergency_stop_acknowledged"), False)
        self.assertIs(self.w.STATE.get("control_unlocked"), False)
        self.assertFalse(self.w.EMERGENCY_STOP_PENDING.is_set())
        self.assertRegex(self.w.STATE.get("message", ""), r"切断|断电|硬件急停")

    def test_dispatch_exception_is_visible_and_keeps_controls_locked(self):
        self.disable.side_effect = RuntimeError("模拟串口驱动异常")
        self.request()
        self.run_dispatch()
        self.assertIs(self.w.STATE.get("emergency_stop_acknowledged"), False)
        self.assertIs(self.w.STATE.get("control_unlocked"), False)
        self.assertRegex(self.w.STATE.get("message", ""), r"切断|断电|硬件急停")

    def test_pending_stop_rejects_new_auto_before_recorder_or_thread_creation(self):
        self.w.EMERGENCY_STOP_PENDING.set()
        self.w.STATE["control_unlocked"] = True  # Require the pending gate itself.
        with patch.object(self.w, "begin_manual_test_run") as begin:
            with self.assertRaises(RuntimeError):
                self.w._start_thread_locked(lambda: None)
        begin.assert_not_called()
        self.assertEqual(self.threads, [])

    def test_pending_stop_rejects_prepare_before_its_hardware_implementation(self):
        self.w.EMERGENCY_STOP_PENDING.set()
        with patch.object(self.w, "_prepare_system_impl") as prepare:
            with self.assertRaises(RuntimeError):
                self.w.prepare_system("MOCK-NO-HARDWARE", 921600)
        prepare.assert_not_called()

    def test_pending_stop_blocks_motion_even_with_stale_unlocked_state(self):
        self.w.EMERGENCY_STOP_PENDING.set()
        self.w.STATE["control_unlocked"] = True
        with self.assertRaises(RuntimeError):
            self.w.require_control_unlocked()


if __name__ == "__main__":
    unittest.main(verbosity=2)
