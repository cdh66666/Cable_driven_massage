"""AST-only reconnect-preparation admission tests; never import the web app."""
import ast
from pathlib import Path
import types
import unittest


SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


class LockStub:
    def __init__(self, locked=False, acquire_result=True):
        self._locked = locked
        self.acquire_result = acquire_result
        self.acquire_calls = 0
        self.release_calls = 0

    def locked(self):
        return self._locked

    def acquire(self, blocking=True):
        self.acquire_calls += 1
        if not self.acquire_result:
            return False
        self._locked = True
        return True

    def release(self):
        self.release_calls += 1
        self._locked = False

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.release()
        return False


class EventStub:
    def __init__(self, set_=False):
        self.set_ = set_

    def is_set(self):
        return self.set_


class ThreadStub:
    instances = []

    def __init__(self, target, **kwargs):
        self.target = target
        self.kwargs = kwargs
        self.started = False
        self.__class__.instances.append(self)

    def start(self):
        self.started = True


class AliveThread:
    def __init__(self, alive=True):
        self.alive = alive

    def is_alive(self):
        return self.alive


class ReprepareGateTests(unittest.TestCase):
    def setUp(self):
        ThreadStub.instances = []
        self.state = {
            "control_unlocked": False,
            "auto": "idle",
            "message": "ready",
            "manual_test_recording": {"active": False},
        }
        self.prepare_calls = []
        self.prepare_exception = None

        def prepare_system(port, baud):
            self.prepare_calls.append((port, baud))
            if self.prepare_exception is not None:
                raise self.prepare_exception
            return {"ready": True}

        self.state_lock = LockStub()
        self.prepare_guard = LockStub()
        self.request_lock = LockStub()
        self.env = {
            "STATE_LOCK": self.state_lock,
            "STATE": self.state,
            "PREPARE_GUARD": self.prepare_guard,
            "AUTO_REQUEST_LOCK": self.request_lock,
            "AUTO_THREAD": None,
            "EMERGENCY_STOP_PENDING": EventStub(False),
            "threading": types.SimpleNamespace(Thread=ThreadStub),
            "prepare_system": prepare_system,
        }
        node = next(node for node in TREE.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "schedule_prepare_if_needed")
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"), self.env)
        self.schedule = self.env["schedule_prepare_if_needed"]

    def queued_worker(self):
        self.schedule("COM5", 921600)
        self.assertEqual(len(ThreadStub.instances), 1)
        thread = ThreadStub.instances[0]
        self.assertTrue(thread.started)
        return thread

    def test_active_thread_blocks_even_when_auto_is_error(self):
        self.state["auto"] = "error"
        self.env["AUTO_THREAD"] = AliveThread()

        self.schedule("COM5", 921600)

        self.assertEqual(ThreadStub.instances, [])
        self.assertEqual(self.prepare_calls, [])

    def test_active_recording_blocks_reprepare(self):
        self.state["manual_test_recording"] = {"active": True, "run_id": "micro-run"}

        self.schedule("COM5", 921600)

        self.assertEqual(ThreadStub.instances, [])
        self.assertEqual(self.prepare_calls, [])

    def test_queued_worker_rechecks_before_prepare(self):
        thread = self.queued_worker()
        self.env["AUTO_THREAD"] = AliveThread()
        self.state["auto"] = "trajectory"
        self.state["manual_test_recording"] = {"active": True, "run_id": "new-run"}

        thread.target()

        self.assertEqual(self.prepare_calls, [])
        self.assertEqual(self.request_lock.release_calls, 1)

    def test_prepare_exception_after_stop_state_change_does_not_overwrite(self):
        def prepare_after_stop(port, baud):
            self.prepare_calls.append((port, baud))
            self.state.update(auto="idle", message="急停已确认；等待重新准备")
            raise RuntimeError("stale reconnect failure")

        self.env["prepare_system"] = prepare_after_stop
        thread = self.queued_worker()
        thread.target()

        self.assertEqual(self.prepare_calls, [("COM5", 921600)])
        self.assertEqual(self.state["auto"], "idle")
        self.assertEqual(self.state["message"], "急停已确认；等待重新准备")
        self.assertEqual(self.request_lock.release_calls, 1)

    def test_idle_without_recording_starts_one_prepare(self):
        thread = self.queued_worker()
        thread.target()

        self.assertEqual(self.prepare_calls, [("COM5", 921600)])
        self.assertEqual(self.request_lock.release_calls, 1)


if __name__ == "__main__":
    unittest.main()
