"""Fresh-cycle accounting and read-only review tests; no controller import/IO."""
import ast
import io
from pathlib import Path
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


def runtime():
    names = {"FourMotorFeedbackAssembler", "completed_feedback_cycles"}
    nodes = [n for n in TREE.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names]
    env = {"MOTOR_IDS": (1, 2, 3, 4), "TimedFeedbackRecord": object,
           "TRACKING_POSITION_COHERENCE_SECONDS": .025}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), env)
    return env["FourMotorFeedbackAssembler"](), env["completed_feedback_cycles"]


def cycle(start, offset=0):
    return [SimpleNamespace(address=i, sampled_at=start + i * .001,
                            position=offset + i) for i in range(1, 5)]


class FreshCycleTests(unittest.TestCase):
    def test_complete_cycle_survives_partial_next_cycle(self):
        assembler, collect = runtime()
        first, second = cycle(1), cycle(1.05, 10)
        self.assertEqual(collect(assembler, first + second[:1]), [first])
        self.assertEqual(collect(assembler, second[1:]), [second])

    def test_every_complete_cycle_in_one_drain_is_preserved(self):
        assembler, collect = runtime()
        first, second, third = cycle(1), cycle(1.05, 10), cycle(1.10, 20)
        self.assertEqual(collect(assembler, first + second + third[:2]), [first, second])
        self.assertEqual(collect(assembler, third[2:]), [third])

    def test_100_render_ticks_do_not_generate_samples(self):
        assembler, collect = runtime()
        self.assertEqual(len(collect(assembler, cycle(1))), 1)
        for _ in range(100):
            self.assertEqual(collect(assembler, []), [])

    def test_dropped_axis_cannot_be_filled_from_an_older_cycle(self):
        assembler, collect = runtime()
        self.assertEqual(collect(assembler, cycle(1)[:3]), [])
        new = cycle(1.05, 100)
        self.assertEqual(collect(assembler, new), [new])

    def test_worker_counts_sample_clock_not_command_ticks(self):
        worker = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == "trajectory_worker")
        fresh_block = next(n for n in ast.walk(worker) if isinstance(n, ast.If)
                           and "completed_snapshots" in ast.unparse(n.test)
                           and "feedback_snapshot is not None" in ast.unparse(n.test))
        code = ast.unparse(fresh_block)
        self.assertIn("feedback_batches = feedback_raw_updates", code)
        self.assertIn("feedback_hz = feedback_raw_hz", code)
        self.assertNotIn("feedback_batches +=", code)
        self.assertNotIn("TRAJECTORY_FEEDBACK_FILTER_ALPHA", code)

    def test_read_only_review_never_calls_controller(self):
        handler_node = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == "Handler")
        def forbidden(*args, **kwargs):
            raise AssertionError("Review must not access controller")
        env = {"BaseHTTPRequestHandler": object, "Path": Path, "__file__": str(SOURCE),
               "execute": forbidden, "web_state_snapshot": forbidden,
               "api_error": forbidden}
        exec(compile(ast.Module(body=[handler_node], type_ignores=[]), str(SOURCE), "exec"), env)
        handler = env["Handler"]()
        handler.path = "/effect-review"
        handler.wfile = io.BytesIO()
        statuses = []
        handler.send_response = statuses.append
        handler.send_header = lambda *args: None
        handler.end_headers = lambda: None
        handler.send_error = forbidden
        handler.do_GET()
        self.assertEqual(statuses, [200])
        self.assertIn("同一笔手绘".encode(), handler.wfile.getvalue())
        self.assertNotIn(b"<script", handler.wfile.getvalue())


if __name__ == "__main__":
    unittest.main()
