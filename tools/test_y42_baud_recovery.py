import unittest
from y42_baud_recovery import restore_baud_snapshot


class RecoveryTests(unittest.TestCase):
    def run_case(self, bad_status=False, wrong_config=False):
        configs = {i: bytes((i, 0x42, 0x25, 0x18)) + bytes(32) + b'\x6b' for i in range(1, 5)}
        rates = {i: 512000 for i in configs}
        writes = []
        def exchange(baud, tx):
            i, command = tx[:2]
            if rates[i] != baud:
                return b''
            if command == 0x3A:
                return bytes((i, command, 3 if bad_status else 2, 0x6B))
            if command == 0x42:
                data = bytearray(configs[i])
                if wrong_config:
                    data[16] = 1
                return bytes(data)
            self.assertEqual(tx[:4], bytes((i, 0x48, 0xD1, 1)))
            self.assertEqual(tx[4:], configs[i][4:])
            writes.append(tx)
            rates[i] = 921600
            return b''  # Lost ACK must not be taken as success; readback decides.
        result = restore_baud_snapshot(configs, 921600, 512000, exchange)
        return result, writes

    def test_recovers_all_axes_despite_lost_write_ack(self):
        result, writes = self.run_case()
        self.assertTrue(result['restored'])
        self.assertEqual(len(writes), 4)

    def test_enabled_axis_never_written(self):
        result, writes = self.run_case(bad_status=True)
        self.assertFalse(result['restored'])
        self.assertEqual(writes, [])

    def test_original_full_config_must_match(self):
        result, _ = self.run_case(wrong_config=True)
        self.assertFalse(result['restored'])

    def test_invalid_snapshot_never_touches_bus(self):
        def exchange(*args):
            self.fail('Invalid snapshot must not touch hardware')
        with self.assertRaises(ValueError):
            restore_baud_snapshot({1: b'bad'}, 921600, 512000, exchange)


if __name__ == '__main__':
    unittest.main()
