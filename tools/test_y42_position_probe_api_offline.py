"""Extracted production admission functions only: no server or serial import."""
import ast
from pathlib import Path
import unittest
from unittest.mock import patch
import sys
from types import SimpleNamespace

SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


class PositionProbeAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.started = []
        self.locked = False
        self.busy = False
        def unlocked():
            if self.locked:
                raise RuntimeError("locked")
        def confirmed(data):
            if data.get("confirmed") is not True:
                raise ValueError("confirmation missing")
        def forbidden(*args, **kwargs):
            raise AssertionError("Motor-angle diagnostic must not publish/use an XY model")
        self.worker = object()
        self.env = {"require_control_unlocked": unlocked,
                    "require_confirmed": confirmed,
                    "active_auto": lambda: self.busy,
                    "calibration_data": lambda: {"valid": False},
                    "start_thread": self.started.append,
                    "position_probe_worker": self.worker,
                    "require_internal_model_ready": forbidden,
                    "save_calibration_settings": forbidden}
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
                    and n.name == "_start_auto_impl")
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), self.env)
        self.run_action = self.env[node.name]

    def test_encoder_only_admission_does_not_write_or_require_xy_model(self):
        result = self.run_action({"action": "position-probe", "confirmed": True})
        self.assertTrue(result["ok"])
        self.assertEqual(self.started, [self.worker])

    def test_locked_unconfirmed_and_busy_all_reject_before_worker(self):
        self.locked = True
        with self.assertRaises(RuntimeError):
            self.run_action({"action": "position-probe", "confirmed": True})
        self.locked = False
        with self.assertRaises(ValueError):
            self.run_action({"action": "position-probe"})
        self.busy = True
        with self.assertRaises(RuntimeError):
            self.run_action({"action": "position-probe", "confirmed": True})
        self.assertEqual(self.started, [])

    def test_no_remote_amplitude_current_or_geometry_override(self):
        for field in ("amplitude", "current", "max_rpm", "segments", "calibration"):
            with self.assertRaises(ValueError):
                self.run_action({"action": "position-probe", "confirmed": True, field: 2})
        self.assertEqual(self.started, [])

    def test_faulted_torque_observer_remains_quarantined(self):
        with self.assertRaisesRegex(RuntimeError, "实测转速超过限制"):
            self.run_action({"action": "observe-torque", "confirmed": True})
        self.assertEqual(self.started, [])

    def test_zero_takeover_is_an_explicit_fixed_action(self):
        self.env["start_thread"] = lambda *args: self.started.append(args)
        self.run_action({"action": "position-zero-takeover", "confirmed": True})
        self.assertEqual(self.started, [(self.worker, True)])
        for data in ({"action": "position-zero-takeover"},
                     {"action": "position-zero-takeover", "confirmed": True, "current": 2000}):
            with self.assertRaises(ValueError): self.run_action(data)
        self.assertEqual(len(self.started), 1)

    def test_both_probe_sources_are_archived(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
                    and n.name == "begin_manual_test_run")
        source = ast.unparse(node)
        for name in ("position_probe_worker", "y42_position_probe.py", "y42_position_probe_io.py"):
            self.assertIn(name, source)


class PositionProbeWorkerStateTests(unittest.TestCase):
    def worker(self, adapter):
        state = {}; calls = []
        def set_state(**fields): state.update(fields)
        def recovery(*args, **kwargs):
            calls.append("disable")
            return {"ok": True}
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
                    and n.name == "position_probe_worker")
        env = {"__name__": "probe_test_api", "sys": SimpleNamespace(modules={"probe_test_api": object()}),
               "set_state": set_state, "STOP_EVENT": SimpleNamespace(set=lambda: calls.append("stop")),
               "disable_all_with_reconnect": recovery, "current_port": lambda: "FAKE",
               "current_baud": lambda: 921600, "manual_test_emit": lambda *a, **k: None}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), env)
        with patch.dict(sys.modules, {"y42_position_probe_io": SimpleNamespace(run_hardware_position_probe=adapter)}):
            result = env[node.name]()
        return result, state, calls

    def test_unexpected_exception_locks_and_requests_disable(self):
        def fail(api): raise RuntimeError("injected IO failure")
        result, state, calls = self.worker(fail)
        self.assertEqual(calls, ["stop", "disable"])
        self.assertEqual(state["auto"], "error")
        self.assertIs(state["control_unlocked"], False)
        self.assertIs(result["configuration_restored"], False)

    def test_missing_cleanup_or_axes_cannot_claim_success(self):
        for outcome in ({"success": True}, {"success": True, "final_disabled_confirmed": "yes"},
                        {"success": True, "final_disabled_confirmed": True, "configuration_restored": True}):
            result, state, _ = self.worker(lambda api: outcome)
            self.assertIs(result["success"], False)
            self.assertEqual(state["auto"], "error")

    def test_complete_result_is_idle_but_locked_and_full_trace_not_in_ui(self):
        outcome = {key: True for key in ("success", "final_hold_verified", "final_disabled_confirmed", "configuration_restored")}
        outcome.update(axis_results=[{"id": i} for i in range(1, 5)], commands=[{}], feedback=[{}])
        result, state, calls = self.worker(lambda api: outcome)
        self.assertEqual(state["auto"], "idle")
        self.assertIs(state["control_unlocked"], False)
        self.assertEqual(calls, [])
        self.assertIn("commands", result)
        self.assertNotIn("commands", state["position_probe"])


if __name__ == "__main__":
    unittest.main()
