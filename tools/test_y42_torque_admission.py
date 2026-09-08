"""Offline C5 torque-admission tests; no serial device or hardware is used."""
import unittest

import y42_position_microprobe_io as io
from test_y42_position_microprobe_io import FakeBus as MicroprobeFakeBus


class TorqueFakeBus(MicroprobeFakeBus):
    """Extend the microprobe bus with strict, low-force C5 behavior."""

    def __init__(self):
        super().__init__()
        self.c5_packets = []
        self.c5_config_snapshots = []
        self.torque_recovery_cb_packets = []
        self.reject_torque_axis = None
        self.torque_offsets = {address: 0.2 for address in io.IDS}
        self.torque_rpm = {address: 0.0 for address in io.IDS}
        self.torque_mode_active = False
        self.config_history = []

    def api(self):
        api = super().api()
        api.pull_directions = lambda: (0, 1, 0, 1)
        return api

    def write(self, packet):
        packet = bytes(packet)
        if len(packet) >= 2 and packet[1] == 0xC5:
            self.calls.append(packet)
            address = packet[0]
            self.assert_c5_contract(packet)
            self.c5_config_snapshots.append({
                axis: (int.from_bytes(config[16:18], "big"),
                       int.from_bytes(config[18:20], "big"))
                for axis, config in self.configs.items()
            })
            self.c5_packets.append(packet)
            self.torque_mode_active = True
            self.current[address] = 20
            self.positions[address] = self.origins[address] + self.torque_offsets[address]
            ack = 3 if address == self.reject_torque_axis else 2
            self.rx = bytes((address, 0xC5, ack, 0x6B))
            return len(packet)
        if len(packet) >= 2 and packet[1] in (0x45, 0x48):
            if packet[1] == 0x45:
                cap = int.from_bytes(packet[4:6], "big")
                rpm = int.from_bytes(self.configs[packet[0]][18:20], "big")
            else:
                cap = int.from_bytes(packet[16:18], "big")
                rpm = int.from_bytes(packet[18:20], "big")
            self.config_history.append((packet[0], packet[1], cap, rpm))
        return super().write(packet)

    def apply_cb(self, packet):
        super().apply_cb(packet)
        if self.torque_mode_active:
            self.torque_recovery_cb_packets.append(packet)
            self.positions[packet[0]] = self.last_targets[packet[0]]

    def record(self, address):
        row = super().record(address)
        if self.torque_mode_active:
            row.rpm = self.torque_rpm[address]
        return row

    def assert_c5_contract(self, packet):
        assert len(packet) == 11
        assert packet[1] == 0xC5
        assert int.from_bytes(packet[3:5], "big") == 200
        assert int.from_bytes(packet[5:7], "big") == 20
        assert packet[7] == 0
        assert int.from_bytes(packet[8:10], "big") == 10
        assert packet[-1] == 0x6B
        assert all(int.from_bytes(config[16:18], "big") <= 20
                   and int.from_bytes(config[18:20], "big") <= 1
                   for config in self.configs.values())

    def run(self, api=None):
        return io.run_hardware_torque_admission(self.api() if api is None else api,
            clock=self.clock, sleep=self.clock.sleep)


class TorqueAdmissionTests(unittest.TestCase):
    def test_four_low_force_c5_pulses_recover_with_twenty_ma_position_commands(self):
        bus = TorqueFakeBus()

        out = bus.run()

        self.assertTrue(out["success"], out["reason"])
        self.assertTrue(out["response_passed"])
        self.assertTrue(out["final_hold_verified"])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertTrue(out["configuration_restored"])
        self.assertLess(out["elapsed_seconds"], 8.0)
        self.assertEqual([packet[0] for packet in bus.c5_packets], [1, 2, 3, 4])
        self.assertEqual(len(out["probe_result"]["axis_results"]), 4)
        self.assertTrue(all(item["recovered"] for item in out["probe_result"]["axis_results"]))
        self.assertTrue(bus.c5_config_snapshots)
        self.assertTrue(all(cap <= 20 and rpm <= 1
                            for snapshot in bus.c5_config_snapshots
                            for cap, rpm in snapshot.values()))
        self.assertTrue(all(cap != 150 for _, _, cap, _ in bus.config_history))
        self.assertTrue(all(int.from_bytes(packet[5:7], "big") == 20
                            for packet in bus.c5_packets))
        self.assertTrue(all(int.from_bytes(packet[11:13], "big") <= 20
                            for packet in bus.position_cb_packets))
        self.assertFalse(any(packet[1] == 0xAA for packet in bus.calls))
        self.assertEqual(bus.configs, bus.original)
        self.assertEqual(bus.stream_calls, [("stop", 20)])
        counts = [bus.query_counts[address] for address in io.IDS]
        self.assertTrue(all(count > 20 for count in counts))
        self.assertLessEqual(max(counts) - min(counts), 1)

    def test_c5_ack_failure_stops_before_next_axis_and_restores(self):
        bus = TorqueFakeBus()
        bus.reject_torque_axis = 2

        out = bus.run()

        self.assertFalse(out["success"])
        self.assertIn("M2低力矩C5接管未确认", out["reason"])
        self.assertEqual([packet[0] for packet in bus.c5_packets], [1, 2])
        self.assertTrue(all(packet[0] < 3 for packet in bus.c5_packets))
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(all(cap != 150 for _, _, cap, _ in bus.config_history))
        self.assertEqual(bus.configs, bus.original)

    def test_c5_position_guard_stops_before_next_axis(self):
        bus = TorqueFakeBus()
        bus.torque_offsets[1] = 3.1

        out = bus.run()

        self.assertFalse(out["success"])
        self.assertIn("原始0x43位置越界", out["reason"])
        self.assertEqual([packet[0] for packet in bus.c5_packets], [1])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertTrue(out["configuration_restored"])
        self.assertEqual(bus.configs, bus.original)

    def test_c5_rpm_guard_stops_before_next_axis(self):
        bus = TorqueFakeBus()
        bus.torque_rpm[1] = 1.3

        out = bus.run()

        self.assertFalse(out["success"])
        self.assertIn("原始0x43转速越界", out["reason"])
        self.assertEqual([packet[0] for packet in bus.c5_packets], [1])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertTrue(out["configuration_restored"])
        self.assertEqual(bus.configs, bus.original)

    def test_c5_raw_overcurrent_stops_before_next_axis_and_restores(self):
        bus = TorqueFakeBus()
        injected = {"done": False}

        def overcurrent_once(reader, records):
            if reader.torque_mode_active and not injected["done"] and records:
                injected["done"] = True
                records[0].phase_current = 351
            return records

        bus.record_hook = overcurrent_once
        out = bus.run()

        self.assertFalse(out["success"])
        self.assertIn("原始0x43电流越界", out["reason"])
        self.assertEqual([packet[0] for packet in bus.c5_packets], [1])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertTrue(out["configuration_restored"])
        self.assertEqual(bus.configs, bus.original)


if __name__ == "__main__":
    unittest.main()
