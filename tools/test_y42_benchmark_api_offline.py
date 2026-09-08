"""Production AST only; no live server, calibration import or motor IO."""
import ast
import copy
from pathlib import Path
import unittest

from y42_handwriting_benchmark import build_benchmark

SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


class BenchmarkIdentityTests(unittest.TestCase):
    def setUp(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == "validated_benchmark_identity")
        env = {}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), env)
        self.validate = env[node.name]
        self.b = build_benchmark()
        self.data = copy.deepcopy(self.b["payload"])
        self.data["benchmark"] = {"id": self.b["benchmark_id"],
                                  "geometry_sha256": self.b["geometry_sha256"], "scale": 1.0}

    def test_plain_handwriting_has_no_fixed_identity(self):
        self.assertIsNone(self.validate({"segments": [[{"x": .2, "y": .3}]]}))

    def test_exact_input_records_identity_without_mutating_request(self):
        before = copy.deepcopy(self.data)
        identity = self.validate(self.data)
        self.assertEqual(identity["geometry_sha256"], self.b["geometry_sha256"])
        self.assertEqual(len(identity["stroke_labels"]), 4)
        self.assertEqual(self.data, before)

    def test_changed_point_cannot_be_labelled_the_same_benchmark(self):
        self.data["segments"][0][1]["x"] += .000001
        with self.assertRaises(ValueError):
            self.validate(self.data)

    def test_changed_hash_or_order_is_rejected(self):
        original = copy.deepcopy(self.data)
        self.data["benchmark"]["geometry_sha256"] = "0" * 64
        with self.assertRaises(ValueError): self.validate(self.data)
        self.data = original
        self.data["segments"].reverse()
        with self.assertRaises(ValueError): self.validate(self.data)

    def test_nan_scale_is_rejected(self):
        self.data["benchmark"]["scale"] = float("nan")
        with self.assertRaises(ValueError): self.validate(self.data)

    def test_read_only_endpoint_has_no_controller_calls(self):
        handler = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == "Handler")
        def forbidden(*args, **kwargs):
            raise AssertionError("Preview must never access motor/controller state")
        env = {"BaseHTTPRequestHandler": object, "execute": forbidden,
               "web_state_snapshot": forbidden, "api_error": forbidden}
        exec(compile(ast.Module(body=[handler], type_ignores=[]), str(SOURCE), "exec"), env)
        h = env["Handler"](); h.path = "/api/benchmark"
        responses = []; h.send_json = responses.append
        h.do_GET()
        self.assertEqual(responses, [{"ok": True, **self.b}])
        self.assertNotIn("confirmed", responses[0]["payload"])
        self.assertNotIn("action", responses[0]["payload"])

    def test_motion_still_checks_lock_and_model_before_fixed_input(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == "_handle_path_locked")
        source = ast.unparse(node)
        for gate in ("require_control_unlocked()", "require_confirmed(data)", "require_internal_model_ready("):
            self.assertLess(source.index(gate), source.index("validated_benchmark_identity(data)"))
        self.assertIn("'benchmark': benchmark_identity", source)


if __name__ == "__main__":
    unittest.main()
