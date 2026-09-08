"""Exercise extracted admission functions without importing the server or serial."""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
QUARANTINE_REASON = "旧力矩标定缺少逐帧超速保护，尚未启动；不需要重复做电机自身校准。"


def extract_function(name, environment):
    node = next(node for node in TREE.body
                if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), environment)
    return environment[name]


class CalibrationAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.saved = {"valid": True, "strong_current": 900, "weak_current": 150,
                      "trajectory_current": 2000, "slope": [1, 2, 3, 4]}
        self.original_saved = copy.deepcopy(self.saved)
        self.state = {"calibration": self.saved, "auto": "idle"}
        self.started = []
        self.logs = []
        self.saved_ready = True
        self.saved_worker = object()

        def forbidden(*args, **kwargs):
            raise AssertionError("Quarantined calibration must not reset parameters or command motors")

        def confirmed(data):
            if data.get("confirmed") is not True:
                raise ValueError("confirmation missing")

        self.env = {
            "math": math,
            "require_control_unlocked": lambda: None,
            "require_confirmed": confirmed,
            "active_auto": lambda: False,
            "calibration_data": lambda: self.saved,
            "persisted_calibration_structure_ready": lambda data: (self.saved_ready, "missing saved datum"),
            "set_state": lambda **fields: self.state.update(fields),
            "start_thread": lambda *args: self.started.append(args),
            "append_process_log": lambda *args, **kwargs: self.logs.append((args, kwargs)),
            "enable_saved_calibration_worker": self.saved_worker,
            "calibration_worker": forbidden,
            "blank_calibration": forbidden,
            "persist_calibration": forbidden,
            "WORKSPACE_WIDTH_MM": 110.0,
            "WORKSPACE_HEIGHT_MM": 148.0,
            "DEFAULT_BALL_DIAMETER_MM": 40.0,
            "CALIBRATION_DEFAULT_STRONG_CURRENT": 900,
            "CALIBRATION_DEFAULT_WEAK_CURRENT": 150,
            "TRAJECTORY_DEFAULT_CURRENT": 2000,
            "OPERATING_CURRENT_MAX_MA": 2000,
            "PERSISTED_CALIBRATION_TARGET_SECONDS": 5.0,
        }
        self.run_action = extract_function("_start_auto_impl", self.env)

    def test_full_calibration_rejects_before_state_or_saved_parameter_changes(self):
        for ready in (True, False):
            with self.subTest(saved_ready=ready):
                self.saved_ready = ready
                state_before = copy.deepcopy(self.state)
                with self.assertRaisesRegex(RuntimeError, QUARANTINE_REASON):
                    self.run_action({"action": "calibrate", "force_full": True, "confirmed": True})
                self.assertEqual(self.state, state_before)
                self.assertEqual(self.saved, self.original_saved)
                self.assertEqual(self.started, [])
                self.assertEqual(self.logs, [])

    def test_full_calibration_rejects_before_parsing_motion_parameters(self):
        with self.assertRaisesRegex(RuntimeError, QUARANTINE_REASON):
            self.run_action({"action": "calibrate", "force_full": True, "confirmed": True,
                             "side": object(), "strong": object(), "weak": object()})
        self.assertEqual(self.state, {"calibration": self.original_saved, "auto": "idle"})
        self.assertEqual(self.started, [])

    def test_saved_activation_routes_remain_available_without_replacing_parameters(self):
        for extra in ({"activate_only": True}, {}):
            with self.subTest(extra=extra):
                result = self.run_action({"action": "calibrate", "confirmed": True, **extra})
                self.assertEqual(result["action"], "enable-saved-calibration")
                self.assertFalse(result["automatic_full_calibration"])
                self.assertEqual(self.started[-1], (self.saved_worker,))
                self.assertEqual(self.saved, self.original_saved)
                self.assertIs(self.state["calibration"], self.saved)
        self.assertEqual(len(self.started), 2)

    def test_missing_saved_datum_keeps_existing_rejection_without_starting_full_calibration(self):
        self.saved_ready = False
        with self.assertRaisesRegex(RuntimeError, "missing saved datum"):
            self.run_action({"action": "calibrate", "activate_only": True, "confirmed": True})
        self.assertEqual(self.started, [])
        self.assertEqual(self.saved, self.original_saved)
        self.assertEqual(self.state["calibration_run_mode"], "enable_saved")
        self.assertEqual(self.state["calibration_stage"], "rejected")


class CalibrationWorkerQuarantineTests(unittest.TestCase):
    def test_direct_worker_rejects_before_any_setup_state_or_hardware_effect(self):
        effects = []

        def forbidden(*args, **kwargs):
            effects.append((args, kwargs))
            raise AssertionError("Direct calibration call must stop before setup")

        environment = {
            "time": SimpleNamespace(perf_counter=forbidden),
            "set_state": forbidden,
            "enable_address": forbidden,
            "direct": SimpleNamespace(open_serial=forbidden),
            "disable_all_with_reconnect": forbidden,
            "STOP_EVENT": SimpleNamespace(is_set=forbidden, clear=forbidden),
        }
        worker = extract_function("calibration_worker", environment)
        with self.assertRaisesRegex(RuntimeError, QUARANTINE_REASON):
            worker(110.0, 148.0, 40.0, 900, 150)
        self.assertEqual(effects, [])


if __name__ == "__main__":
    unittest.main()
