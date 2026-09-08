"""Guard idle browser scanning without importing or connecting to the controller."""
import ast
from contextlib import nullcontext
from pathlib import Path
import threading
import types
import unittest


SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")


class IdleScanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)
        cls.functions = {
            node.name: node for node in cls.tree.body
            if isinstance(node, ast.FunctionDef)
        }
        cls.page = next(
            ast.literal_eval(node.value) for node in cls.tree.body
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "PAGE"
                    for target in node.targets)
        )

    def test_no_automatic_scan_dispatch_or_retry_promises(self):
        for removed in ("autoScanTick", "autoScanInFlight", "autoPrepareInFlight",
                        "每秒自动扫描", "等待自动扫描", "1 秒后继续重试",
                        "继续等待自动扫描", "将继续尝试自动扫描"):
            self.assertFalse(removed in self.page, f"Removed idle-scan text remains: {removed}")

    def test_manual_scan_and_normal_polling_remain(self):
        self.assertIn('onclick="scanIds()"', self.page)
        self.assertIn("async function scanIds()", self.page)
        self.assertIn("await post({action:'scan',start,end})", self.page)
        self.assertIn("function idConfigSafe(){return sessionSafetyConfirmed}", self.page)
        self.assertIn("autoPollLoop();setInterval(()=>{refreshStatus()},1200)", self.page)

    def test_backend_boot_and_status_recovery_remain(self):
        def calls(name):
            return [node for node in ast.walk(self.functions[name])
                    if isinstance(node, ast.Call)]

        self.assertTrue(any(isinstance(call.func, ast.Attribute)
                            and call.func.attr == "Thread"
                            and any(keyword.arg == "target"
                                    and isinstance(keyword.value, ast.Name)
                                    and keyword.value.id == "auto_prepare_on_boot"
                                    for keyword in call.keywords)
                            for call in calls("run")))
        for function, target in (
            ("auto_prepare_on_boot", "prepare_system"),
            ("auto_prepare_on_boot", "baud_sweep"),
            ("read_all_with_reconnect", "schedule_prepare_if_needed"),
            ("schedule_prepare_if_needed", "prepare_system"),
        ):
            self.assertTrue(any(isinstance(call.func, ast.Name)
                                and call.func.id == target for call in calls(function)),
                            (function, target))

    def test_missing_motor_only_scans_and_recommends_power_and_communication(self):
        operations = []
        state = {}
        connection = object()

        def scan(actual, start, end, verbose):
            self.assertIs(actual, connection)
            operations.append(("scan", start, end, verbose))
            return [1, 2, 3]

        serial_lock = threading.Lock()
        env = {
            "active_auto": lambda: False,
            "set_state": lambda **updates: state.update(updates),
            "SERIAL_LOCK": serial_lock,
            "MOTOR_IDS": (1, 2, 3, 4),
            "direct": types.SimpleNamespace(
                open_serial=lambda port, baud: nullcontext(connection), scan=scan),
        }
        isolated = ast.Module(body=[self.functions["_prepare_system_impl"]], type_ignores=[])
        exec(compile(isolated, str(SOURCE), "exec"), env)
        result = env["_prepare_system_impl"]("FAKE", 115200)
        self.assertEqual(operations, [("scan", 1, 4, False)])
        self.assertEqual(result["missing"], [4])
        self.assertFalse(result["ready"])
        self.assertFalse(state["control_unlocked"])
        self.assertFalse(serial_lock.locked())
        for expected in ("动力电源", "RS485 A/B", "接头", "重新点击"):
            self.assertIn(expected, result["message"])
        self.assertNotIn("ID 修改区", result["message"])
        self.assertNotIn("缺失 ID 时可直接在本区修改", self.page)


if __name__ == "__main__":
    unittest.main()
