"""Offline recovery tests; no serial or real socket access."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import y42_restore_probe_envelope as recovery


class RestoreTests(unittest.TestCase):
    def setUp(self):
        self.originals, self.residuals, _ = recovery.load_archive()
        self.raw = SimpleNamespace(configs=dict(self.residuals))
        self.writes = []
        self.flags, self.rpm, self.ack = 2, 0, 2

    def transact(self, connection, packet, **kwargs):
        address, command = packet[:2]
        if command == 0x42:
            return connection.configs[address]
        if command == 0x3B:
            return bytes((address, command, 3, 0x6B))
        if command == 0x3A:
            return bytes((address, command, self.flags, 0x6B))
        if command == 0x35:
            return bytes((address, command, 0, 0, self.rpm, 0x6B))
        if command == 0x27:
            return bytes((address, command, 0, 22, 0x6B))
        if command == 0x48:
            self.writes.append(packet)
            self.assertEqual(packet[3], 0)
            connection.configs[address] = bytes((address, 0x42, 0x25, 0x18)) + packet[4:]
            return bytes((address, command, self.ack, 0x6B))
        self.fail("Unexpected command")

    def run_restore(self, apply=False):
        return recovery.restore_envelope(self.raw, self.originals, self.residuals,
            transact=self.transact, emit=lambda *a, **k: None, apply=apply)

    def test_archive_residual_chain(self):
        self.assertEqual([int.from_bytes(self.residuals[i][16:18], 'big') for i in recovery.IDS], [20, 0, 0, 0])

    def test_default_read_only(self):
        self.assertTrue(self.run_restore()['success'])
        self.assertEqual(self.writes, [])

    def test_only_caps_restored(self):
        result = self.run_restore(True)
        self.assertTrue(result['success'])
        self.assertEqual(result['restored'], [1, 2, 3, 4])
        self.assertEqual(self.raw.configs, self.originals)
        self.assertEqual(len(self.writes), 4)

    def test_enabled_or_moving_blocks_every_write(self):
        for flags, rpm in ((3, 0), (2, 1)):
            self.flags, self.rpm = flags, rpm
            self.assertFalse(self.run_restore(True)['success'])
        self.assertEqual(self.writes, [])

    def test_other_configuration_change_blocks(self):
        changed = bytearray(self.raw.configs[1]); changed[9] ^= 1
        self.raw.configs[1] = bytes(changed)
        self.assertFalse(self.run_restore(True)['success'])
        self.assertEqual(self.writes, [])

    def test_lost_ack_is_unknown_no_retry(self):
        self.ack = 0xE2
        result = self.run_restore(True)
        self.assertFalse(result['success'])
        self.assertEqual(result['unknown'], [1])
        self.assertEqual(len(self.writes), 1)

    def test_occupied_backend_port_blocks_entry(self):
        fake = SimpleNamespace(setsockopt=lambda *a: None,
            bind=lambda *a: (_ for _ in ()).throw(OSError('in use')), close=lambda: None)
        with patch.object(recovery.socket, 'socket', return_value=fake):
            with self.assertRaises(OSError):
                with recovery.exclusive_backend_guard():
                    self.fail('must not enter')


if __name__ == '__main__':
    unittest.main()
