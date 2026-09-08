"""Pure benchmark checks; export tests use an in-memory fake filesystem."""
import ast
from contextlib import contextmanager
import copy
import io
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import y42_handwriting_benchmark as benchmark


class BenchmarkTests(unittest.TestCase):
    def setUp(self):
        self.value = benchmark.build_benchmark()

    def test_exact_four_strokes_and_endpoints(self):
        self.assertEqual(len(self.value["strokes"]), 4)
        self.assertEqual([len(s["points_mm"]) for s in self.value["strokes"]], [4, 65, 65, 4])
        self.assertEqual(self.value["strokes"][0]["points_mm"], [[20.0, 124.0], [90.0, 124.0], [90.0, 102.0], [55.0, 102.0]])
        self.assertEqual(self.value["strokes"][-1]["points_mm"], [[23.0, 28.0], [31.0, 28.0], [31.0, 34.0], [39.0, 34.0]])
        for stroke in self.value["strokes"][1:3]:
            self.assertEqual(stroke["points_mm"][0], stroke["points_mm"][-1])

    def test_finite_bounds_margin_and_normalized_coordinates(self):
        for stroke in self.value["strokes"]:
            for (x, y), normalized in zip(stroke["points_mm"], stroke["points_normalized"]):
                self.assertTrue(math.isfinite(x) and math.isfinite(y))
                self.assertLessEqual(18.0, x)
                self.assertLessEqual(x, 92.0)
                self.assertLessEqual(18.0, y)
                self.assertLessEqual(y, 130.0)
                self.assertTrue(0.0 <= normalized["x"] <= 1.0)
                self.assertTrue(0.0 <= normalized["y"] <= 1.0)
                self.assertAlmostEqual(x, normalized["x"] * 110.0, delta=0.00006)
                self.assertAlmostEqual(y, (1.0 - normalized["y"]) * 148.0, delta=0.00008)

    def test_long_line_exact_right_angles_and_return(self):
        points = self.value["strokes"][0]["points_mm"]
        vectors = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])]
        self.assertEqual(math.hypot(*vectors[0]), 70.0)
        for left, right in zip(vectors, vectors[1:]):
            self.assertEqual(sum(a * b for a, b in zip(left, right)), 0.0)
        self.assertLess(vectors[0][0] * vectors[-1][0], 0.0)

    def test_circle_has_fixed_radius_sampling_and_no_missing_closure(self):
        points = self.value["strokes"][1]["points_mm"]
        for point in points:
            self.assertAlmostEqual(math.dist(point, (35.0, 78.0)), 16.0, delta=0.000001)
        lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
        self.assertLess(max(lengths) - min(lengths), 0.000002)
        self.assertEqual(len(set(tuple(point) for point in points[:-1])), 64)

    def test_eight_has_true_shared_crossing_and_two_lobes(self):
        points = self.value["strokes"][2]["points_mm"]
        self.assertEqual(points[0], [77.0, 75.0])
        self.assertEqual(points[32], points[0])
        self.assertEqual(points[64], points[0])
        self.assertGreater(min(point[0] for point in points[1:32]), 77.0)
        self.assertLess(max(point[0] for point in points[33:64]), 77.0)
        first = [points[1][i] - points[0][i] for i in range(2)]
        second = [points[33][i] - points[32][i] for i in range(2)]
        self.assertNotAlmostEqual(first[0] * second[1] - first[1] * second[0], 0.0)

    def test_short_stroke_and_transfers_remain_separate(self):
        points = self.value["strokes"][3]["points_mm"]
        self.assertEqual([math.dist(a, b) for a, b in zip(points, points[1:])], [8.0, 6.0, 8.0])
        self.assertEqual(len(self.value["payload"]["segments"]), 4)
        self.assertEqual(sum(len(s) for s in self.value["payload"]["segments"]), 138)
        self.assertEqual(len(self.value["operator_labels"]), 4)
        for index, stroke in enumerate(self.value["strokes"]):
            self.assertEqual(self.value["payload"]["segments"][index], stroke["points_normalized"])

    def test_hash_determinism_payload_independence_and_geometry_sensitivity(self):
        self.assertEqual(self.value, benchmark.build_benchmark())
        self.assertEqual(len(self.value["geometry_sha256"]), 64)
        self.assertEqual(self.value["geometry_sha256"], "6c0b45bd429ebcccc49b5676bcc95b6cf4b48bdf9a8a908d9050644a3c7a1565")
        changed = copy.deepcopy(self.value["strokes"])
        changed[0]["points_mm"][0][0] += 0.001
        self.assertNotEqual(benchmark.geometry_sha256(changed, 1.0), self.value["geometry_sha256"])
        payload = benchmark.build_payload()
        payload["segments"][0][0]["x"] = 0.0
        self.assertEqual(self.value, benchmark.build_benchmark())

    def test_scaling_is_board_centred_bounded_and_deterministic(self):
        for scale in (0.25, 0.5, 0.75, 1.0):
            scaled = benchmark.build_benchmark(scale)
            self.assertEqual(scaled, benchmark.build_benchmark(scale))
            for original, changed in zip(self.value["strokes"], scaled["strokes"]):
                for (x, y), (sx, sy) in zip(original["points_mm"], changed["points_mm"]):
                    self.assertAlmostEqual(sx, 55.0 + scale * (x - 55.0), delta=0.000001)
                    self.assertAlmostEqual(sy, 74.0 + scale * (y - 74.0), delta=0.000001)
        self.assertNotEqual(self.value["geometry_sha256"], benchmark.build_benchmark(0.5)["geometry_sha256"])
        for invalid in (0.0, 0.249, 1.001, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                benchmark.build_benchmark(invalid)

    def test_payload_never_authorizes_execution_or_contains_control_actions(self):
        self.assertEqual(set(self.value["payload"]), {"segments", "speed", "repeat", "controller_mode"})
        self.assertEqual(self.value["payload"]["speed"], 50.0)
        self.assertFalse(self.value["payload"]["repeat"])
        self.assertEqual(self.value["payload"]["controller_mode"], "four_position")
        forbidden = {"action", "confirmed", "enabled", "enable", "execute", "start", "force_full", "port", "baud"}
        def check_keys(value):
            if isinstance(value, dict):
                self.assertTrue(set(value).isdisjoint(forbidden))
                for item in value.values():
                    check_keys(item)
            elif isinstance(value, list):
                for item in value:
                    check_keys(item)
        check_keys(self.value)

    def test_module_has_no_hardware_network_or_subprocess_dependencies(self):
        tree = ast.parse(Path(benchmark.__file__).read_text(encoding="utf-8"))
        forbidden = {"serial", "socket", "requests", "urllib", "http", "subprocess", "y42_pc_direct", "y42_pc_direct_web"}
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                self.assertTrue({name.name.split(".")[0] for name in node.names}.isdisjoint(forbidden))
            if isinstance(node, ast.ImportFrom):
                self.assertNotIn((node.module or "").split(".")[0], forbidden)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, {"post", "request", "transact", "open_serial", "send_fast", "start_thread"})

    def test_export_paths_are_exclusive_and_preserve_arbitrary_existing_files(self):
        files = {benchmark.EXPORT_ROOT / "precious_operator_data.json": "unchanged"}
        directories = []
        @contextmanager
        def memory_open(path, mode, **kwargs):
            self.assertEqual(mode, "x")
            self.assertTrue(path.is_relative_to(benchmark.EXPORT_ROOT))
            if path in files:
                raise FileExistsError(path)
            buffer = io.StringIO()
            yield buffer
            files[path] = buffer.getvalue()
        with patch.object(Path, "mkdir", autospec=True, side_effect=lambda path, **kwargs: directories.append((path, kwargs))), \
             patch.object(Path, "open", autospec=True, side_effect=memory_open):
            first = benchmark.export_benchmark()
            second = benchmark.export_benchmark()
        self.assertNotEqual(first["directory"], second["directory"])
        self.assertEqual(files[benchmark.EXPORT_ROOT / "precious_operator_data.json"], "unchanged")
        self.assertEqual(len(files), 5)
        self.assertEqual(json.loads(files[Path(first["files"]["benchmark.json"])])["geometry_sha256"], self.value["geometry_sha256"])
        for directory, kwargs in directories:
            self.assertTrue(directory.is_relative_to(benchmark.EXPORT_ROOT))
            if directory != benchmark.EXPORT_ROOT:
                self.assertFalse(kwargs["exist_ok"])

    def test_export_rejects_other_directories_before_any_write(self):
        with patch.object(benchmark, "EXPORT_ROOT", benchmark.EXPORT_ROOT.parent / "outside"), \
             patch.object(Path, "mkdir") as mkdir, patch.object(Path, "open") as opened:
            with self.assertRaises(ValueError):
                benchmark.export_benchmark()
            mkdir.assert_not_called()
            opened.assert_not_called()


if __name__ == "__main__":
    unittest.main()
