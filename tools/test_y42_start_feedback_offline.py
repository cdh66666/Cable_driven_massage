"""AST-only production tests. No controller import, API, device, or serial I/O."""
import ast
import copy
from dataclasses import asdict, dataclass
from pathlib import Path
import threading
import types
import unittest


SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
SOURCE_TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


def selected_functions(names, env):
    nodes = [node for node in SOURCE_TREE.body if isinstance(node, ast.FunctionDef) and node.name in names]
    if {node.name for node in nodes} != set(names):
        raise AssertionError("Selected production functions not found")
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), env)
    return env


@dataclass
class Telemetry:
    address: int
    voltage: float | None = None
    bus_current: int | None = None
    phase_current: int | None = None
    rpm: float | None = None
    position: float | None = None
    temperature: int | None = None
    flags: int | None = None


class FreshStartFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.calls, self.sleeps, self.fail = [], [], set()
        self.env = {
            "STATE_LOCK": threading.RLock(), "copy": copy,
            "STATE": {"telemetry": {str(i): {"voltage": 24.0, "bus_current": 50,
                "phase_current": 150, "temperature": 26} for i in range(1, 5)}},
            "MOTOR_IDS": (1, 2, 3, 4),
            "time": types.SimpleNamespace(sleep=self.sleeps.append),
            "telemetry_dict": asdict,
            "valid_reply": lambda reply, address, command, minimum: len(reply) >= minimum
                and reply[0] == address and reply[1] == command and reply[-1] == 0x6B,
            "direct": types.SimpleNamespace(Telemetry=Telemetry, FRAME_END=0x6B,
                frame=lambda *values: bytes(values), transact=self.transact,
                read_u16=lambda reply, offset: int.from_bytes(reply[offset:offset + 2], "big"),
                read_u32=lambda reply, offset: int.from_bytes(reply[offset:offset + 4], "big")),
        }
        selected_functions({"read_trajectory_settle", "update_telemetry"}, self.env)

    def transact(self, connection, frame, **kwargs):
        address, command, _ = frame
        self.calls.append((address, command, kwargs))
        if (address, command) in self.fail:
            return b""
        payload = {
            0x35: bytes((int(address == 2),)) + (10 * address).to_bytes(2, "big"),
            0x36: bytes((int(address == 3),)) + (1000 * address).to_bytes(4, "big"),
            0x3A: b"\x03",
            0x27: (900 + address * 100).to_bytes(2, "big"),
            0x39: bytes((0, 30 + address)),
        }[command]
        return bytes((address, command)) + payload + b"\x6b"

    def read(self, **kwargs):
        return self.env["read_trajectory_settle"](object(), **kwargs)

    def test_default_keeps_twelve_queries_and_cached_load(self):
        values = self.read()
        self.assertEqual(len(self.calls), 12)
        self.assertEqual({command for _, command, _ in self.calls}, {0x35, 0x36, 0x3A})
        self.assertEqual([t.phase_current for t in values], [150] * 4)
        self.assertEqual([t.temperature for t in values], [26] * 4)
        self.assertEqual(values[1].rpm, -2.0)
        self.assertEqual(values[2].position, -300.0)

    def test_include_load_reads_twenty_queries_and_overwrites_old_cache(self):
        values = self.read(include_load=True)
        self.assertEqual(len(self.calls), 20)
        self.assertEqual([t.phase_current for t in values], [1000, 1100, 1200, 1300])
        self.assertEqual([t.temperature for t in values], [31, 32, 33, 34])
        self.assertEqual([t.voltage for t in values], [24.0] * 4)
        self.assertEqual(self.env["STATE"]["telemetry"]["4"]["phase_current"], 1300)

    def test_missing_load_never_returns_old_cached_value_after_retries(self):
        self.fail = {(4, 0x27), (2, 0x39)}
        values = self.read(include_load=True, attempts=2)
        self.assertEqual(len(self.calls), 40)
        self.assertEqual(self.sleeps, [0.006])
        self.assertIsNone(values[3].phase_current)
        self.assertIsNone(values[1].temperature)
        # Dashboard merging must not mutate the returned fresh readings.
        self.assertEqual(self.env["STATE"]["telemetry"]["4"]["phase_current"], 150)
        self.assertIsNone(values[3].phase_current)

    def test_missing_position_does_not_become_success_with_fresh_load(self):
        self.fail = {(1, 0x36)}
        values = self.read(include_load=True, attempts=2)
        self.assertEqual(len(self.calls), 40)
        self.assertIsNone(values[0].position)
        self.assertEqual(values[0].phase_current, 1000)

    def test_query_timeout_settings_are_forwarded_unchanged(self):
        self.read(include_load=True, first_byte_timeout=0.017, inter_byte_quiet=0.002)
        self.assertTrue(all(kwargs == {"first_byte_timeout": 0.017,
            "inter_byte_quiet": 0.002} for _, _, kwargs in self.calls))


class RequestAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.lock, self.calls = threading.Lock(), []
        self.env = {"AUTO_REQUEST_LOCK": self.lock,
            "_handle_path_locked": lambda data: self.calls.append(("path", data)) or {"ok": True},
            "_start_auto_impl": lambda data: self.calls.append(("auto", data)) or {"ok": True}}
        selected_functions({"handle_path", "start_auto"}, self.env)

    def test_busy_path_rejected_without_call_or_releasing_other_owner(self):
        self.lock.acquire()
        try:
            with self.assertRaisesRegex(RuntimeError, "请求正在处理"):
                self.env["handle_path"]({"points": []})
            self.assertEqual(self.calls, [])
            self.assertTrue(self.lock.locked())
        finally:
            self.lock.release()

    def test_success_calls_implementation_once_and_releases(self):
        payload = {"points": [{"x": 0.5, "y": 0.5}]}
        self.assertEqual(self.env["handle_path"](payload), {"ok": True})
        self.assertEqual(self.calls, [("path", payload)])
        self.assertFalse(self.lock.locked())

    def test_failed_implementation_releases_admission_lock(self):
        def fail(data):
            raise ValueError("invalid path")
        self.env["_handle_path_locked"] = fail
        with self.assertRaisesRegex(ValueError, "invalid path"):
            self.env["handle_path"]({})
        self.assertFalse(self.lock.locked())

    def test_pending_path_blocks_second_tab_auto_but_never_stop(self):
        entered, release = threading.Event(), threading.Event()
        failures = []
        def path_impl(data):
            entered.set()
            if not release.wait(2):
                raise AssertionError("test gate timeout")
            return {"ok": True}
        self.env["_handle_path_locked"] = path_impl
        def request():
            try:
                self.env["handle_path"]({})
            except BaseException as error:
                failures.append(error)
        worker = threading.Thread(target=request)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            with self.assertRaisesRegex(RuntimeError, "上一项请求"):
                self.env["start_auto"]({"action": "calibrate"})
            with self.assertRaisesRegex(RuntimeError, "请求正在处理"):
                self.env["handle_path"]({})
            for action in ("abort", "tracking-stop"):
                self.assertEqual(self.env["start_auto"]({"action": action}), {"ok": True})
            self.assertEqual([data["action"] for _, data in self.calls], ["abort", "tracking-stop"])
            self.assertTrue(self.lock.locked())
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        self.assertFalse(self.lock.locked())


if __name__ == "__main__":
    unittest.main()
