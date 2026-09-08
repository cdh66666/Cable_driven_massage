"""Pure AST planning replay: no server import, serial, network or saved data."""
import ast
import copy
import math
from pathlib import Path
import unittest


SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
NAMES = {"time_parameterized_path", "sample_cumulative_path", "custom_motion_phase",
         "resample_polyline", "plan_custom_motion_segments"}


def environment(source_tree=None):
    tree = TREE if source_tree is None else source_tree
    env = {"math": math}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name.startswith("CUSTOM_PATH_"):
                try:
                    env[name] = ast.literal_eval(node.value)
                except (ValueError, TypeError):
                    pass
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in NAMES]
    assert {node.name for node in nodes} == NAMES
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "<pure-motion-planning>", "exec"), env)
    return env


def old_duration(env, strokes, draw_speed=50.0, travel_speed=150.0):
    """Frozen old scheduling rules: default 20 mm/s boundaries + 0.24 s dwell."""
    seconds = 0.0
    previous = None
    for stroke in strokes:
        if previous is not None:
            distance = math.dist(previous, stroke[0])
            if distance > env["CUSTOM_PATH_DEDUP_DISTANCE_MM"]:
                transfer = env["resample_polyline"]([previous, stroke[0]])
                seconds += env["time_parameterized_path"](transfer, travel_speed, 600.0)[0][-1]
            seconds += 0.24
        seconds += env["time_parameterized_path"](stroke, draw_speed)[0][-1]
        previous = stroke[-1]
    return seconds


class MotionPhaseTests(unittest.TestCase):
    def setUp(self):
        self.env = environment()

    def refine(self, paths):
        return [self.env["resample_polyline"](path) for path in paths]

    def plan(self, strokes):
        return self.env["plan_custom_motion_segments"](strokes, 50.0, 150.0)

    def at(self, phase, seconds):
        duration = phase["end_s"] - phase["start_s"]
        return self.env["sample_cumulative_path"](
            phase["path"], phase["cumulative"], seconds / duration, phase["speeds"])

    def assert_phase_limits(self, phase):
        path, times, speeds = phase["path"], phase["cumulative"], phase["speeds"]
        cap, acceleration = phase["cruise_speed_mm_s"], phase["acceleration_mm_s2"]
        self.assertEqual(speeds[0], 0.0)
        self.assertEqual(speeds[-1], 0.0)
        for index, (left, right) in enumerate(zip(path, path[1:])):
            distance = math.dist(left, right)
            duration = times[index + 1] - times[index]
            if distance <= 1e-12:
                continue
            self.assertGreater(duration, 0.0)
            first, last = speeds[index:index + 2]
            if first + last <= 1e-9:
                peak = 2.0 * distance / duration
                accel = 4.0 * distance / duration ** 2
            else:
                peak = max(first, last)
                accel = abs(last - first) / duration
                self.assertAlmostEqual(0.5 * (first + last) * duration, distance, places=9)
            self.assertLessEqual(peak, cap + 1e-7)
            self.assertLessEqual(accel, acceleration + 1e-6)

    def test_default_profile_callers_keep_original_boundary_speed(self):
        path = self.refine([[(0.0, 0.0), (40.0, 0.0)]])[0]
        times, speeds = self.env["time_parameterized_path"](path, 50.0)
        self.assertEqual(speeds[0], 20.0)
        self.assertEqual(speeds[-1], 20.0)
        self.assertEqual(max(speeds), 50.0)
        self.assertGreater(times[-1], 40.0 / 50.0)

    def test_constant_acceleration_sampling_is_not_average_speed_lerp(self):
        sample = self.env["sample_cumulative_path"]
        path = [(0.0, 0.0), (1.0, 0.0)]
        self.assertEqual(sample(path, [0.0, 0.2], 0.5), (0.5, 0.0))
        self.assertEqual(sample(path, [0.0, 0.2], 0.5, [0.0, 10.0]), (0.25, 0.0))
        self.assertEqual(sample(path, [0.0, 0.2], 0.5, [10.0, 0.0]), (0.75, 0.0))

    def test_no_fixed_dwell_and_all_exact_phase_endpoints_join(self):
        strokes = self.refine([[(0.0, 0.0), (20.0, 0.0)],
                               [(25.0, 10.0), (25.0, 30.0)],
                               [(25.0, 30.0), (40.0, 30.0)]])
        original = copy.deepcopy(strokes)
        phases, _, duration, distance = self.plan(strokes)
        self.assertEqual([p["kind"] for p in phases], ["draw", "travel", "draw", "draw"])
        self.assertEqual(strokes, original)
        self.assertEqual([p["path"] for p in phases if p["kind"] == "draw"], original)
        self.assertAlmostEqual(distance, math.hypot(5.0, 10.0))
        self.assertEqual(phases[-1]["end_s"], duration)
        for phase in phases:
            self.assert_phase_limits(phase)
            self.assertEqual(self.at(phase, 0.0), phase["path"][0])
            self.assertEqual(self.at(phase, phase["end_s"] - phase["start_s"]), phase["path"][-1])
        for before, after in zip(phases, phases[1:]):
            self.assertEqual(before["end_s"], after["start_s"])
            self.assertEqual(before["path"][-1], after["path"][0])

    def test_draw_and_travel_acceleration_and_velocity_continuity(self):
        phases = self.plan(self.refine([[(0.0, 0.0), (40.0, 0.0)],
                                      [(70.0, 10.0), (70.0, 50.0)]]))[0]
        h = 1e-5
        for phase in phases:
            self.assert_phase_limits(phase)
            duration = phase["end_s"] - phase["start_s"]
            self.assertLess(math.dist(self.at(phase, h), self.at(phase, 0.0)) / h, 0.01)
            self.assertLess(math.dist(self.at(phase, duration), self.at(phase, duration - h)) / h, 0.01)
            # Numerical vector checks apply to these straight phases. Existing
            # sharp in-stroke vertices are retained, not falsely certified as
            # globally bounded vector acceleration by a tangential planner.
            for index in range(1, 1000):
                t = duration * index / 1000.0
                before, now, after = self.at(phase, t - h), self.at(phase, t), self.at(phase, t + h)
                velocity = math.dist(after, before) / (2.0 * h)
                accel = math.hypot(*(after[j] - 2.0 * now[j] + before[j] for j in range(2))) / h ** 2
                self.assertLessEqual(velocity, phase["cruise_speed_mm_s"] + 1e-4)
                self.assertLessEqual(accel, phase["acceleration_mm_s2"] + 0.05)

    def test_short_two_node_strokes_zero_to_zero_are_bounded_not_crawling(self):
        for distance in (0.001, 0.01, 0.05, 0.2, 100.0):
            with self.subTest(distance=distance):
                phase = self.plan([[(0.0, 0.0), (distance, 0.0)]])[0][0]
                self.assert_phase_limits(phase)
                duration = phase["end_s"]
                self.assertEqual(self.at(phase, duration), (distance, 0.0))
                self.assertAlmostEqual(self.at(phase, duration / 4.0)[0], distance / 8.0)
                self.assertLess(duration, distance / 0.1 + 0.02)

    def test_sub_dedup_transfer_is_not_an_uncommanded_position_jump(self):
        phases = self.plan([[(0.0, 0.0), (1.0, 0.0)], [(1.01, 0.0), (2.0, 0.0)]])[0]
        self.assertEqual([phase["kind"] for phase in phases], ["draw", "travel", "draw"])
        self.assertAlmostEqual(math.dist(*phases[1]["path"]), 0.01)
        self.assert_phase_limits(phases[1])

    def test_phase_sampler_never_marks_transfer_as_ink_and_reaches_last_point(self):
        phases, _, duration, _ = self.plan(self.refine([[(0.0, 0.0), (20.0, 0.0)],
                                                       [(30.0, 10.0), (50.0, 10.0)]]))
        milliseconds = math.ceil(duration * 1000.0)
        spec = {"motion_segments": phases, "duration_ms": milliseconds}
        for phase in phases:
            u = (phase["start_s"] + phase["end_s"]) / 2.0 / (milliseconds / 1000.0)
            actual = self.env["custom_motion_phase"](u, spec)
            self.assertEqual(actual["kind"], phase["kind"])
            self.assertEqual(actual["drawing"], phase["kind"] == "draw")
        self.assertEqual(self.env["custom_motion_phase"](1.0, spec)["xy"], phases[-1]["path"][-1])

    def test_exact_sharp_vertices_are_preserved_without_smoothing(self):
        strokes = self.refine([[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 0.0)]])
        phase = self.plan(strokes)[0][0]
        self.assertEqual(phase["path"], strokes[0])
        self.assert_phase_limits(phase)
        for point, stamp in zip(phase["path"], phase["cumulative"]):
            actual = self.at(phase, stamp)
            self.assertLess(math.dist(point, actual), 1e-9)

    def test_same_input_old_new_time_comparison(self):
        cases = {
            "six_separated_40mm_strokes": [[(0.0, y), (40.0, y)] for y in range(0, 60, 10)],
            "ten_short_strokes": [[(0.0, y), (4.0, y)] for y in range(0, 20, 2)],
            "three_touching_strokes": [[(0.0, 0.0), (20.0, 0.0)], [(20.0, 0.0), (20.0, 20.0)], [(20.0, 20.0), (40.0, 20.0)]],
        }
        for name, raw in cases.items():
            strokes = self.refine(raw)
            before = old_duration(self.env, strokes)
            phases, _, after, _ = self.plan(strokes)
            self.assertLess(after, before, name)
            for phase in phases:
                self.assert_phase_limits(phase)
            print(f"OFFLINE_PLAN {name}: old={before:.6f}s new={after:.6f}s saved={before-after:.6f}s fixed_dwell_removed={0.24*(len(strokes)-1):.2f}s")

    def test_no_cached_start_release_and_four_cb_subcommands_are_immediate(self):
        worker = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == "trajectory_worker")
        for node in ast.walk(worker):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "frame":
                prefix = [n.value if isinstance(n, ast.Constant) else None for n in node.args[:3]]
                self.assertNotEqual(prefix, [0x00, 0xFF, 0x66])
        from test_y42_aggregate_frames import environment as frame_environment
        frame = frame_environment()["four_motor_position_frame"]([0.0] * 4, [40.0] * 4, 300)
        for index in range(4):
            sub = frame[4 + index * 14:4 + (index + 1) * 14]
            self.assertEqual(sub[1], 0xCB)
            self.assertEqual(sub[9:11], b"\x01\x00", "absolute mode, no cached sync")


if __name__ == "__main__":
    unittest.main()
