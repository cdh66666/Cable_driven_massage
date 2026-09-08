"""Exercise the real transact loop without opening a serial device."""
import time
import unittest
import y42_pc_direct as direct


class Port:
    def __init__(self, timeout=0.001, write_timeout=0.05):
        self._timeout, self._write_timeout = timeout, write_timeout
        self.updates = []
        self.rx = b""
    @property
    def timeout(self): return self._timeout
    @timeout.setter
    def timeout(self, value):
        self.updates.append(("read", value)); self._timeout = value
    @property
    def write_timeout(self): return self._write_timeout
    @write_timeout.setter
    def write_timeout(self, value):
        self.updates.append(("write", value)); self._write_timeout = value
    @property
    def in_waiting(self): return len(self.rx)
    def reset_input_buffer(self): self.rx = b""
    def write(self, data): self.rx = bytes.fromhex("013a026b"); return len(data)
    def read(self, size):
        result, self.rx = self.rx[:size], self.rx[size:]
        return result


class Timeouts(unittest.TestCase):
    def test_unchanged_timeouts_never_reconfigure_port(self):
        port = Port()
        result = direct.transact(port, bytes.fromhex("013a6b"), deadline=time.perf_counter()+0.35)
        self.assertEqual(result, bytes.fromhex("013a026b"))
        self.assertEqual(port.updates, [])

    def test_prepared_short_write_timeout_stays_fixed_across_transactions(self):
        """The real deadline wrapper must not rewrite an already bounded port."""
        port = Port(timeout=0.001, write_timeout=0.01)
        for _ in range(6):
            result = direct.transact(port, bytes.fromhex("013a6b"),
                                     deadline=time.perf_counter() + 0.35)
            self.assertEqual(result, bytes.fromhex("013a026b"))
        self.assertEqual(port.write_timeout, 0.01)
        self.assertEqual(port.updates, [])

    def test_unbounded_timeouts_are_bounded_and_restored(self):
        port = Port(None, None)
        direct.transact(port, bytes.fromhex("013a6b"), deadline=time.perf_counter()+0.35)
        self.assertIsNone(port.timeout)
        self.assertIsNone(port.write_timeout)
        self.assertEqual(len(port.updates), 4)
        self.assertGreater(port.updates[0][1], 0)
        self.assertLessEqual(port.updates[0][1], 0.35)


if __name__ == "__main__": unittest.main()
