"""Offline admission/worker tests for the bounded corner-stop diagnostic."""
import copy
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from test_y42_calibration_flow import isolated_web_module
from y42_adapter_versions import load_corner_probe_adapter


class CornerStopProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="y42-corner-stop-offline-")
        cls.web = isolated_web_module(cls.temporary.name)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def setUp(self):
        self.web = self.__class__.web
        self.original_state = copy.deepcopy(self.web.STATE)
        self.web.STATE["control_unlocked"] = True
        self.web.STATE["auto"] = "idle"
        self.web.EMERGENCY_STOP_PENDING.clear()
        self.web.STOP_EVENT.clear()
        self.started = []
        self.start_patch = patch.object(
            self.web, "start_thread", side_effect=lambda *args: self.started.append(args)
        )
        self.start_patch.start()
        self.addCleanup(self.start_patch.stop)
        self.addCleanup(self.web.EMERGENCY_STOP_PENDING.clear)
        self.addCleanup(self.web.STOP_EVENT.clear)
        self.addCleanup(lambda: self.web.STATE.update(self.original_state))

    def test_api_requires_unlock_confirmation_and_idle_state(self):
        self.web.STATE["control_unlocked"] = False
        with self.assertRaises(RuntimeError):
            self.web.start_auto({"action": "corner-stop-probe", "confirmed": True})
        self.assertEqual(self.started, [])

        self.web.STATE["control_unlocked"] = True
        with self.assertRaises(ValueError):
            self.web.start_auto({"action": "corner-stop-probe"})
        self.assertEqual(self.started, [])

        with patch.object(self.web, "active_auto", return_value=True):
            with self.assertRaises(RuntimeError):
                self.web.start_auto({"action": "corner-stop-probe", "confirmed": True})
        self.assertEqual(self.started, [])

    def test_api_rejects_all_parameter_overrides_and_starts_only_worker(self):
        fake_loader = Mock()
        fake_adapter_module = types.ModuleType("y42_adapter_versions")
        fake_adapter_module.load_corner_probe_adapter = fake_loader
        request = {"action": "corner-stop-probe", "confirmed": True, "limit_deg": 0.1}
        with patch.dict(sys.modules, {"y42_adapter_versions": fake_adapter_module}):
            with self.assertRaisesRegex(ValueError, "不接受参数覆盖"):
                self.web.start_auto(request)
            self.assertEqual(self.started, [])
            self.assertFalse(fake_loader.called)

            result = self.web.start_auto({"action": "corner-stop-probe", "confirmed": True})
        self.assertTrue(result["ok"])
        self.assertEqual(result["action"], "corner-stop-probe")
        self.assertEqual(self.started, [(self.web.corner_stop_probe_worker,)])
        self.assertFalse(fake_loader.called)

    def _run_worker(self, adapter_result=None, adapter_error=None):
        adapter = types.SimpleNamespace(
            run_hardware_corner_stop_probe=Mock(return_value=adapter_result)
        )
        if adapter_error is not None:
            adapter.run_hardware_corner_stop_probe.side_effect = adapter_error
        loader = Mock(return_value=(adapter, "corner-digest"))
        adapter_versions = types.ModuleType("y42_adapter_versions")
        adapter_versions.load_corner_probe_adapter = loader
        emitted = Mock()
        states = []
        recovery = Mock(return_value={"ok": True})
        with patch.dict(sys.modules, {"y42_adapter_versions": adapter_versions}), \
             patch.object(self.web, "manual_test_emit", emitted), \
             patch.object(self.web, "set_state", side_effect=lambda **fields: states.append(fields)), \
             patch.object(self.web, "disable_all_with_reconnect", recovery), \
             patch.object(self.web, "current_port", return_value="offline-port"), \
             patch.object(self.web, "current_baud", return_value=921600):
            result = self.web.corner_stop_probe_worker()
        return result, adapter, loader, emitted, states, recovery

    def test_worker_loads_adapter_at_worker_boundary_and_success_is_diagnostic_only(self):
        result, adapter, loader, emitted, states, recovery = self._run_worker(
            {"success": True, "final_disabled_confirmed": True,
             "configuration_restored": True, "reason": "synthetic"}
        )
        loader.assert_called_once_with()
        adapter.run_hardware_corner_stop_probe.assert_called_once_with(self.web)
        recovery.assert_not_called()
        self.assertTrue(result["success"])
        self.assertFalse(result["calibration_published"])
        self.assertFalse(result["xyz_precision_measured"])
        self.assertEqual(states[-1]["auto"], "idle")
        self.assertFalse(self.web.STOP_EVENT.is_set())

    def test_worker_exception_cannot_succeed_or_publish_calibration(self):
        result, _, loader, _, _, recovery = self._run_worker(
            adapter_error=RuntimeError("synthetic adapter failure")
        )
        loader.assert_called_once_with()
        recovery.assert_called_once_with(
            "offline-port", 921600, announce=True, clear_timed_streams=True
        )
        self.assertFalse(result["success"])
        self.assertFalse(result["final_disabled_confirmed"])
        self.assertFalse(result["calibration_published"])
        self.assertFalse(result["xyz_precision_measured"])
        self.assertTrue(self.web.STOP_EVENT.is_set())

    def test_worker_incomplete_cleanup_cannot_succeed_or_publish_calibration(self):
        result, _, _, _, _, recovery = self._run_worker(
            {"success": True, "final_disabled_confirmed": True,
             "configuration_restored": False, "reason": "cleanup incomplete"}
        )
        recovery.assert_not_called()
        self.assertFalse(result["success"])
        self.assertTrue(result["final_disabled_confirmed"])
        self.assertFalse(result["configuration_restored"])
        self.assertFalse(result["calibration_published"])
        self.assertFalse(result["xyz_precision_measured"])


class CornerStopAdapterLoaderTests(unittest.TestCase):
    def test_loader_reads_only_allowlisted_corner_worker_source(self):
        source = b'def run_hardware_corner_stop_probe(api): return {"success": True}\n'
        reads = Mock(return_value=source)
        with patch("y42_adapter_versions.Path.read_bytes", new=reads):
            module, digest = load_corner_probe_adapter()
        self.assertEqual(reads.call_count, 1)
        source_path = Path(module.__file__)
        self.assertIsInstance(source_path, Path)
        self.assertEqual(source_path.name, "y42_corner_stop_probe.py")
        self.assertTrue(module.__name__.startswith("_y42_corner_probe_"))
        self.assertEqual(len(digest), 64)
        self.assertTrue(callable(module.run_hardware_corner_stop_probe))


if __name__ == "__main__":
    unittest.main(verbosity=2)
