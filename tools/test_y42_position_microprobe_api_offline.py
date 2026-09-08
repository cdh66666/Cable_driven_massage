"""Microprobe admission/state regressions; never import the real server."""
import ast
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import test_y42_position_probe_api_offline as admission_fixture

SOURCE, TREE = admission_fixture.SOURCE, admission_fixture.TREE


class MicroprobeAdmissionTests(unittest.TestCase):
    def setUp(self):
        admission_fixture.PositionProbeAdmissionTests.setUp(self)
        self.env["position_microprobe_worker"] = self.worker

    def test_fixed_action_needs_no_xy_model(self):
        result = self.run_action({"action": "position-microprobe", "confirmed": True})
        self.assertTrue(result["ok"])
        self.assertEqual(self.started, [self.worker])

    def test_no_parameter_or_permission_bypass(self):
        for key in ("current", "amplitude", "max_rpm", "axis_signs", "clock", "calibration"):
            with self.assertRaises(ValueError):
                self.run_action({"action": "position-microprobe", "confirmed": True, key: 1})
        with self.assertRaises(ValueError):
            self.run_action({"action": "position-microprobe"})
        self.locked = True
        with self.assertRaises(RuntimeError):
            self.run_action({"action": "position-microprobe", "confirmed": True})
        self.locked = False
        self.busy = True
        with self.assertRaises(RuntimeError):
            self.run_action({"action": "position-microprobe", "confirmed": True})
        self.assertEqual(self.started, [])

    def test_source_in_manifest(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
                    and n.name == "begin_manual_test_run")
        source = ast.unparse(node)
        for name in ("position_microprobe_worker", "y42_position_microprobe_io.py",
                     "y42_position_probe.py", "y42_position_probe_io.py"):
            self.assertIn(name, source)


class TorqueAdmissionTests(unittest.TestCase):
    def setUp(self):
        admission_fixture.PositionProbeAdmissionTests.setUp(self)
        self.env["torque_admission_worker"] = self.worker

    def test_fixed_action_needs_no_xy_model_or_old_calibration(self):
        result = self.run_action({"action": "torque-admission", "confirmed": True})
        self.assertTrue(result["ok"])
        self.assertEqual(result["action"], "torque-admission")
        self.assertEqual(self.started, [self.worker])

    def test_action_rejects_overrides_locked_busy_and_unconfirmed(self):
        for key in ("current", "max_rpm", "directions", "pull_directions", "calibration"):
            with self.assertRaises(ValueError):
                self.run_action({"action": "torque-admission", "confirmed": True, key: 1})
        with self.assertRaises(ValueError):
            self.run_action({"action": "torque-admission"})
        self.locked = True
        with self.assertRaises(RuntimeError):
            self.run_action({"action": "torque-admission", "confirmed": True})
        self.locked = False
        self.busy = True
        with self.assertRaises(RuntimeError):
            self.run_action({"action": "torque-admission", "confirmed": True})
        self.assertEqual(self.started, [])

    def test_torque_worker_and_shared_sources_are_archived(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
                    and n.name == "begin_manual_test_run")
        source = ast.unparse(node)
        for name in ("torque_admission_worker", "y42_position_microprobe_io.py",
                     "y42_position_probe.py", "y42_position_probe_io.py"):
            self.assertIn(name, source)


class MicroprobeWorkerTests(unittest.TestCase):
    def run_worker(self, adapter):
        state, calls = {}, []
        def recovery(*args, **kwargs):
            self.assertTrue(kwargs["clear_timed_streams"])
            calls.append("disable")
            return {"ok": True}
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
                    and n.name == "position_microprobe_worker")
        env = {"__name__": "isolated_api", "sys": SimpleNamespace(modules={"isolated_api": object()}),
               "set_state": lambda **fields: state.update(fields),
               "STOP_EVENT": SimpleNamespace(set=lambda: calls.append("stop")),
               "disable_all_with_reconnect": recovery, "current_port": lambda: "FAKE",
               "current_baud": lambda: 921600, "manual_test_emit": lambda *a, **k: None}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), env)
        fake_adapter = SimpleNamespace(run_hardware_position_microprobe=adapter, __file__=__file__)
        fake_loader = lambda: (fake_adapter, "fake-adapter-sha")
        with patch.dict(sys.modules, {
            "y42_adapter_versions": SimpleNamespace(load_microprobe_adapter=fake_loader),
        }):
            result = env[node.name]()
        return result, state, calls

    def test_exception_requests_stop_and_never_claims_restored(self):
        def fail(api):
            raise RuntimeError("injected failure")
        result, state, calls = self.run_worker(fail)
        self.assertEqual(calls, ["stop", "disable"])
        self.assertEqual(state["auto"], "error")
        self.assertIs(state["control_unlocked"], False)
        self.assertIs(result["configuration_restored"], False)

    def test_incomplete_result_is_not_success(self):
        for outcome in ({"success": True}, {"success": True, "final_disabled_confirmed": "yes"}):
            result, state, calls = self.run_worker(lambda api: outcome)
            self.assertIs(result["success"], False)
            self.assertEqual(state["auto"], "error")

    def test_structured_cleanup_failure_uses_independent_stop(self):
        result, state, calls = self.run_worker(lambda api: {
            "success": False, "reason": "injected lost stop ACK",
            "final_disabled_confirmed": False, "configuration_restored": False,
            "requires_hardware_stop": True})
        self.assertEqual(calls, ["stop", "disable"])
        self.assertTrue(result["disable_recovery"]["ok"])
        self.assertIs(result["final_disabled_confirmed"], False)
        self.assertIs(result["configuration_restored"], False)
        self.assertEqual(state["auto"], "error")

    def test_complete_result_stays_encoder_only_and_full_archive_not_ui(self):
        outcome = {key: True for key in ("success", "final_hold_verified", "final_disabled_confirmed", "configuration_restored")}
        outcome.update(axis_results=[{"id": i} for i in range(1, 5)], xyz_precision_measured=True,
                       probe_result={"large": []}, cadence={"commands": [{}], "measured_command_hz": 100})
        result, state, calls = self.run_worker(lambda api: outcome)
        self.assertEqual(state["auto"], "idle")
        self.assertIs(state["control_unlocked"], False)
        self.assertIs(result["xyz_precision_measured"], False)
        self.assertIn("probe_result", result)
        self.assertNotIn("probe_result", state["position_probe"])
        self.assertNotIn("commands", state["position_probe"]["cadence"])
        self.assertEqual(calls, [])


class TorqueAdmissionWorkerTests(unittest.TestCase):
    def run_worker(self, adapter):
        state, calls = {}, []

        def recovery(*args, **kwargs):
            self.assertTrue(kwargs["clear_timed_streams"])
            calls.append("disable")
            return {"ok": True}

        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
                    and n.name == "torque_admission_worker")
        env = {"__name__": "isolated_torque_api",
               "sys": SimpleNamespace(modules={"isolated_torque_api": object()}),
               "set_state": lambda **fields: state.update(fields),
               "STOP_EVENT": SimpleNamespace(set=lambda: calls.append("stop")),
               "disable_all_with_reconnect": recovery, "current_port": lambda: "FAKE",
               "current_baud": lambda: 921600, "manual_test_emit": lambda *a, **k: None}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), env)
        fake_adapter = SimpleNamespace(run_hardware_torque_admission=adapter, __file__=__file__)
        fake_loader = lambda: (fake_adapter, "fake-adapter-sha")
        with patch.dict(sys.modules, {
            "y42_adapter_versions": SimpleNamespace(load_microprobe_adapter=fake_loader),
        }):
            result = env[node.name]()
        return result, state, calls

    def test_exception_requests_stop_and_independent_disable_without_restore_claim(self):
        def fail(api):
            raise RuntimeError("injected torque failure")

        result, state, calls = self.run_worker(fail)

        self.assertGreaterEqual(calls.count("stop"), 1)
        self.assertEqual(calls[-2:], ["stop", "disable"])
        self.assertEqual(state["auto"], "error")
        self.assertIs(result["success"], False)
        self.assertIs(result["final_disabled_confirmed"], False)
        self.assertIs(result["configuration_restored"], False)
        self.assertEqual(result["disable_recovery"], {"ok": True})

    def test_structured_failure_keeps_restore_unknown_after_independent_disable(self):
        result, state, calls = self.run_worker(lambda api: {
            "success": False, "reason": "C5 ACK rejected",
            "final_disabled_confirmed": False, "configuration_restored": False,
        })

        self.assertEqual(calls, ["stop", "disable"])
        self.assertEqual(state["auto"], "error")
        self.assertIs(result["success"], False)
        self.assertIs(result["final_disabled_confirmed"], False)
        self.assertIs(result["configuration_restored"], False)

    def test_success_requires_exactly_four_ordered_axes_and_never_publishes_calibration(self):
        outcome = {key: True for key in
                   ("success", "final_hold_verified", "final_disabled_confirmed", "configuration_restored")}
        outcome.update(axis_results=[{"id": i} for i in range(1, 5)],
                       calibration_published=True, xyz_precision_measured=True,
                       probe_result={"raw": []}, cadence={"commands": [{}]})

        result, state, calls = self.run_worker(lambda api: outcome)

        self.assertEqual(calls, [])
        self.assertEqual(state["auto"], "idle")
        self.assertIs(state["control_unlocked"], False)
        self.assertIs(result["success"], True)
        self.assertIs(result["calibration_published"], False)
        self.assertIs(result["xyz_precision_measured"], False)
        self.assertEqual([row["id"] for row in result["axis_results"]], [1, 2, 3, 4])
        self.assertIn("torque_admission", state)
        self.assertNotIn("probe_result", state["torque_admission"])

        incomplete = dict(outcome, axis_results=[{"id": i} for i in range(1, 4)])
        result, state, calls = self.run_worker(lambda api: incomplete)
        self.assertIs(result["success"], False)
        self.assertEqual(state["auto"], "error")
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
