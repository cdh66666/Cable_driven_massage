"""Offline behavioral checks for cadence, cancellation and manual-path speed."""
import ast
from pathlib import Path
import queue
import threading
import time
import unittest

SOURCE = Path(__file__).with_name('y42_pc_direct_web.py')
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))

def load_parts(send):
    nodes = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
             and n.name in ('TrajectoryFrameWriter', 'trajectory_target_rpm', 'next_trajectory_deadline')]
    ns = dict(queue=queue, threading=threading, time=time, STOP_EVENT=threading.Event(),
              TRAJECTORY_INTERVAL=.01, _send_fast_sync=send, ESP32_SMOOTH_PROFILE='esp32_smooth_v1',
              TRAJECTORY_MAX_RPM=3000., ESP32_SMOOTH_MOTION_RPM=1000., TRAJECTORY_FEEDFORWARD=1.25)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), ns)
    return ns

class MotionRegression(unittest.TestCase):
    def test_manual_speed_follows_segment_and_does_not_change_sign_contract(self):
        ns = load_parts(lambda *_: None)
        speed = ns['trajectory_target_rpm']
        for delta in (.05, .4, 2.):
            expected = abs(delta) / .01 / 6 * 1.25
            self.assertAlmostEqual(speed(100., 100. + delta, .01, 'esp32_smooth_v1'), expected)
            self.assertAlmostEqual(speed(100., 100. - delta, .01, 'esp32_smooth_v1'), expected)
        self.assertEqual(speed(100, 100, .01, 'esp32_smooth_v1'), 1.)

    def test_burst_producer_preserves_order_without_wire_burst(self):
        sent = []
        ns = load_parts(lambda _, p: sent.append((p, time.perf_counter())))
        writer = ns['TrajectoryFrameWriter'](None)
        for i in range(25):
            writer.submit(bytes([i]))
        writer.close()
        self.assertEqual([p for p, _ in sent], [bytes([i]) for i in range(25)])
        self.assertGreater(sent[-1][1] - sent[0][1], .20)
        self.assertFalse(writer.thread.is_alive())

    def test_stop_cancels_unsent_frames(self):
        sent = []
        ns = load_parts(lambda _, p: (sent.append(p), ns['STOP_EVENT'].set()))
        writer = ns['TrajectoryFrameWriter'](None)
        for i in range(8):
            writer.submit(bytes([i]))
        with self.assertRaises(RuntimeError):
            writer.flush()
        self.assertEqual(sent, [b'\x00'])
        with self.assertRaises(RuntimeError):
            writer.close()
        self.assertFalse(writer.thread.is_alive())

if __name__ == '__main__':
    unittest.main()
