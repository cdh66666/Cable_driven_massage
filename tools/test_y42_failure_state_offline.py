"""Execute production failure-state functions in isolation: no controller import."""
import ast
import copy
import itertools
from pathlib import Path
import threading
import types
import unittest


SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")


class FailureStateTests(unittest.TestCase):
    def setUp(self):
        wanted = {"set_state", "state_snapshot", "web_state_snapshot"}
        parsed = ast.parse(SOURCE.read_text(encoding="utf-8"))
        functions = [node for node in parsed.body if isinstance(node, ast.FunctionDef) and node.name in wanted]
        self.assertEqual({node.name for node in functions}, wanted)
        ticks = itertools.count(100)
        self.env = {
            "STATE_LOCK": threading.RLock(), "copy": copy,
            "time": types.SimpleNamespace(time_ns=lambda: next(ticks), time=lambda: 1788575561.4),
            "STATE": {"auto": "trajectory", "message": "正在接近起点",
                      "manual_test_recording": {"run_id": "failed-run-1"},
                      "calibration": {"valid": True, "groups": ["large-offline-data"]}},
            "persisted_calibration_structure_ready": lambda model: (True, "saved"),
        }
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE), "exec"), self.env)

    def update(self, **updates):
        self.env["set_state"](**updates)

    def test_failure_survives_idle_prepare_updates(self):
        self.update(auto="error", message="移动到轨迹起点超时：M4差36.77度")
        original = copy.deepcopy(self.env["STATE"]["last_failure"])
        self.update(auto="idle", message="自动准备完成，四台失能", control_unlocked=True)
        self.assertEqual(self.env["STATE"]["last_failure"], original)
        self.assertEqual(original["run_id"], "failed-run-1")
        self.assertFalse(original["acknowledged"])

    def test_safety_cleanup_updates_message_without_changing_failure_id(self):
        self.update(auto="error", message="移动到轨迹起点超时")
        failure_id = self.env["STATE"]["last_failure"]["id"]
        self.update(auto="error", message="移动到轨迹起点超时；已确认四台失能")
        self.assertEqual(self.env["STATE"]["last_failure"]["id"], failure_id)
        self.assertIn("四台失能", self.env["STATE"]["last_failure"]["message"])

    def test_new_run_same_cause_has_a_new_failure_identity(self):
        self.update(auto="error", message="移动到轨迹起点超时")
        first = self.env["STATE"]["last_failure"]["id"]
        self.update(auto="idle", message="自动准备完成")
        self.update(auto="trajectory", manual_test_recording={"run_id": "failed-run-2"})
        self.update(auto="error", message="移动到轨迹起点超时")
        latest = self.env["STATE"]["last_failure"]
        self.assertNotEqual(latest["id"], first)
        self.assertEqual(latest["run_id"], "failed-run-2")

    def test_both_snapshots_keep_failure_and_are_detached(self):
        self.update(auto="error", message="移动到轨迹起点超时")
        self.update(auto="idle", message="自动准备完成")
        for name in ("state_snapshot", "web_state_snapshot"):
            snapshot = self.env[name]()
            self.assertIn("起点超时", snapshot["last_failure"]["message"])
            snapshot["last_failure"]["message"] = "changed by caller"
            snapshot["last_failure"]["acknowledged"] = True
            self.assertIn("起点超时", self.env["STATE"]["last_failure"]["message"])
            self.assertFalse(self.env["STATE"]["last_failure"]["acknowledged"])
        self.assertEqual(self.env["STATE"]["calibration"]["groups"], ["large-offline-data"])

    def test_successful_prepare_does_not_create_a_failure(self):
        self.update(auto="idle", message="自动准备完成")
        self.assertIsNone(self.env["STATE"].get("last_failure"))


if __name__ == "__main__":
    unittest.main()
