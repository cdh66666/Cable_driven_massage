import unittest
from unittest.mock import patch
from y42_adapter_versions import load_microprobe_adapter


class AdapterVersionsTests(unittest.TestCase):
    def test_version_replacement_keeps_existing_worker_module(self):
        one = b'def run_hardware_position_microprobe(api): return 1\n'
        two = b'def run_hardware_position_microprobe(api): return 2\n'
        with patch('y42_adapter_versions.Path.read_bytes', return_value=one):
            old, old_hash = load_microprobe_adapter()
        with patch('y42_adapter_versions.Path.read_bytes', return_value=two):
            new, new_hash = load_microprobe_adapter()
        self.assertNotEqual(old_hash, new_hash)
        self.assertEqual(old.run_hardware_position_microprobe(None), 1)
        self.assertEqual(new.run_hardware_position_microprobe(None), 2)

    def test_syntax_failure_does_not_replace_previous_version(self):
        source = b'def run_hardware_position_microprobe(api): return 3\n'
        with patch('y42_adapter_versions.Path.read_bytes', return_value=source):
            old, _ = load_microprobe_adapter()
        with patch('y42_adapter_versions.Path.read_bytes', return_value=b'def invalid!'):
            with self.assertRaises(SyntaxError):
                load_microprobe_adapter()
        self.assertEqual(old.run_hardware_position_microprobe(None), 3)

    def test_missing_entry_rejected(self):
        with patch('y42_adapter_versions.Path.read_bytes', return_value=b'x = 1\n'):
            with self.assertRaises(TypeError):
                load_microprobe_adapter()


if __name__ == '__main__':
    unittest.main()
