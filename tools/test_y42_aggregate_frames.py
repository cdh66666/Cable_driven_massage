"""AST-only production frame tests; no web import, serial, config or data IO."""
import ast
from pathlib import Path
import types
import unittest


TOOLS = Path(__file__).parent
WEB_TREE = ast.parse((TOOLS / "y42_pc_direct_web.py").read_text(encoding="utf-8"))
DIRECT_TREE = ast.parse((TOOLS / "y42_pc_direct.py").read_text(encoding="utf-8"))
BUILDERS = ("four_motor_position_frame", "four_motor_torque_frame",
            "multi_motor_timed_feedback_frame", "mixed_position_torque_frame")
PULL = (0, 1, 1, 0)
ENCODER_POSITIVE = (0, 1, 0, 1)


def extract(tree, names, env):
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    assert {node.name for node in nodes} == set(names)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "<pure-production-builders>", "exec"), env)


def environment():
    direct = {"FRAME_END": 0x6B, "OPERATING_CURRENT_MAX_MA": 2000}
    extract(DIRECT_TREE, ("frame", "u16", "u32", "torque_frame"), direct)
    env = {"direct": types.SimpleNamespace(**direct), "MOTOR_IDS": (1, 2, 3, 4),
        "OPERATING_CURRENT_MAX_MA": 2000, "TRAJECTORY_MAX_RPM": 3000,
        "TRACKING_TORQUE_SLOPE_MA_PER_S": 400, "TRACKING_TORQUE_MAX_RPM": 8,
        "TIMED_FEEDBACK_INFO_CODE": 0x43, "TIMED_POSITION_INTERVAL_MS": 10,
        "trajectory_current_limit": lambda: 300,
        "normalized_trajectory_current_limits": lambda values: list(values) if isinstance(values, (list, tuple)) else [values] * 4,
        "encoder_positive_directions": lambda calibration=None: ENCODER_POSITIVE,
        "absolute_position_direction": lambda axis, target: int((target if ENCODER_POSITIVE[axis] == 0 else -target) < 0),
        "pull_directions": lambda: PULL}
    extract(WEB_TREE, BUILDERS + ("timed_feedback_frame",), env)
    return env


def expected_cb(axis, target, rpm, current):
    direction = int((target if ENCODER_POSITIVE[axis - 1] == 0 else -target) < 0)
    return bytes((axis, 0xCB, direction)) + round(rpm * 10).to_bytes(2, "big") + round(abs(target) * 10).to_bytes(4, "big") + b"\x01\x00" + current.to_bytes(2, "big") + b"\x6b"


def expected_c5(axis, current):
    return bytes((axis, 0xC5, PULL[axis - 1])) + b"\x01\x90" + current.to_bytes(2, "big") + b"\x00\x00\x50\x6b"


class AggregateFrameTests(unittest.TestCase):
    def setUp(self):
        self.env = environment()
        self.targets = [100.0, -200.0, -300.0, 400.0]
        self.rpms = [8.0, 12.0, 24.0, 36.0]
        self.currents = [150, 210, 330, 450]

    def assert_packet(self, frame, subframes, expected_length):
        self.assertEqual(len(frame), expected_length)
        self.assertEqual(int.from_bytes(frame[2:4], "big"), len(frame))
        self.assertEqual(frame[:2], b"\x00\xaa")
        self.assertEqual(frame[4:-1], b"".join(subframes), "Any subcommand byte changed")
        self.assertEqual(frame[-1], 0x6B)

    def test_manual_page36_golden_34_byte_packet_uses_production_header_tail(self):
        golden = bytes.fromhex("00 AA 00 22 02 FD 01 05 DC 08 00 00 7D 00 00 00 6B 03 FD 00 03 E8 0A 00 00 FA 00 01 01 6B 04 36 6B 6B")
        self.assertEqual(len(golden), 34)
        for builder in BUILDERS:
            with self.subTest(builder=builder):
                node = next(node for node in WEB_TREE.body if isinstance(node, ast.FunctionDef) and node.name == builder)
                # Exercise each actual production header/final-byte block with
                # the manufacturer's mixed Emm/query golden payload.
                start = next(i for i, statement in enumerate(node.body)
                    if isinstance(statement, ast.Assign) and isinstance(statement.targets[0], ast.Name)
                    and statement.targets[0].id == "length")
                block = node.body[start:start + 4]
                local = {"packet": bytearray(b"\x00\xaa\x00\x00" + golden[4:-1]),
                         "direct": types.SimpleNamespace(FRAME_END=0x6B)}
                exec(compile(ast.Module(body=block, type_ignores=[]), "<golden-production-tail>", "exec"), local)
                self.assertEqual(bytes(local["packet"]), golden)

    def test_four_position_61_preserves_targets_directions_current_and_subframe_flags(self):
        frame = self.env["four_motor_position_frame"](self.targets, self.rpms, self.currents)
        subframes = [expected_cb(i + 1, self.targets[i], self.rpms[i], self.currents[i]) for i in range(4)]
        self.assert_packet(frame, subframes, 61)

    def test_four_torque_49_preserves_all_c5_fields(self):
        frame = self.env["four_motor_torque_frame"](self.currents, torque_slope=400, torque_max_rpm=8)
        self.assert_packet(frame, [expected_c5(i + 1, current) for i, current in enumerate(self.currents)], 49)

    def test_four_timed_33_preserves_both_supported_query_codes_and_intervals(self):
        for info in (0x43, 0x36):
            for interval in (0, 10, 256, 65535):
                with self.subTest(info=info, interval=interval):
                    frame = self.env["multi_motor_timed_feedback_frame"](interval, info)
                    subframes = [bytes((axis, 0x11, 0x18, info)) + interval.to_bytes(2, "big") + b"\x6b" for axis in range(1, 5)]
                    self.assert_packet(frame, subframes, 33)

    def test_hybrid_all_14_valid_axis_selections_preserve_each_subcommand(self):
        for mask in range(1, 15):
            indices = tuple(i for i in range(4) if mask & (1 << i))
            with self.subTest(indices=indices):
                frame = self.env["mixed_position_torque_frame"](self.targets, indices,
                    self.currents, self.rpms, self.currents, torque_slope=400, torque_max_rpm=8)
                subframes = [expected_cb(i + 1, self.targets[i], self.rpms[i], self.currents[i])
                    if i in indices else expected_c5(i + 1, self.currents[i]) for i in range(4)]
                self.assert_packet(frame, subframes, 49 + 3 * len(indices))


if __name__ == "__main__":
    unittest.main()
