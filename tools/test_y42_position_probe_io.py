"""Isolated fake-bus tests; never import the web controller or open serial."""
import contextlib
import copy
import threading
import types
import unittest

import y42_position_probe_io as io


class FakeBus:
    def __init__(self):
        self.configs = {}
        for address in range(1, 5):
            config = bytearray((address, 0x42, 0x25, 0x18)) + bytearray(range(4, 36)) + b"\x6b"
            config[5] = 1
            config[16:20] = bytes((0x0B, 0xB8, 0, 100))
            config[25:27] = bytes((1, 0))
            self.configs[address] = bytes(config)
        self.original = copy.deepcopy(self.configs)
        self.positions = {1: -12.3, 2: 11.2, 3: -2.7, 4: 4.1}
        self.flags = {address: 2 for address in range(1, 5)}
        self.rpm = {address: 0 for address in range(1, 5)}
        self.calls = []
        self.events = []
        self.reject_seed = None
        self.reject_enable = None
        self.allow_enable = False
        self.current = {address: 0 for address in range(1, 5)}
        self.reject_envelope = None
        self.reject_restore = None
        self.partial_command = None
        self.corrupt_readback = None
        self.restore_writes = []
        self.rx = b""
        self.flush_calls = 0
        self.write_timeout = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def read(self, size=1):
        out, self.rx = self.rx[:size], self.rx[size:]
        return out

    def write(self, packet):
        packet = bytes(packet)
        self.calls.append(packet)
        address, command = packet[:2]
        if command == 0x42:
            self.rx = self.configs[address]
        elif command == 0x48:
            updated = bytes((address, 0x42, 0x25, 0x18)) + packet[4:-1] + b"\x6b"
            restoring = int.from_bytes(updated[16:18], "big") > 20
            self.configs[address] = updated
            if restoring:
                self.restore_writes.append(address)
            rejected = address == (self.reject_restore if restoring else self.reject_envelope)
            self.rx = bytes((address, command, 0xE2 if rejected else 2, 0x6B))
            if not restoring and self.corrupt_readback == address:
                bad = bytearray(updated)
                bad[16:18] = bytes((0, 21))
                self.configs[address] = bytes(bad)
        elif command == 0xCB:
            self.rx = bytes((address, command, 0xE2 if address == self.reject_seed else 2, 0x6B))
        elif command == 0x36:
            position = self.positions[address]
            self.rx = (bytes((address, command, int(position < 0)))
                       + round(abs(position) * 10).to_bytes(4, "big") + b"\x6b")
        elif command == 0x35:
            rpm = self.rpm[address]
            self.rx = bytes((address, command, int(rpm < 0))) + round(abs(rpm) * 10).to_bytes(2, "big") + b"\x6b"
        elif command == 0x27:
            self.rx = bytes((address, command)) + self.current[address].to_bytes(2, "big") + b"\x6b"
        elif command == 0x39:
            self.rx = bytes((address, command, 0, 25, 0x6B))
        elif command == 0x3A:
            self.rx = bytes((address, command, self.flags[address], 0x6B))
        elif command in (0xF3, 0xFE):
            if command == 0xF3 and packet[3] != 0:
                if not self.allow_enable:
                    raise AssertionError("ENABLE IS FORBIDDEN")
                assert all(config[16:20] == bytes((0, 0, 0, 1)) for config in self.configs.values())
                self.flags[address] |= 1
                self.rx = bytes((address, command, 0xE2 if address == self.reject_enable else 2, 0x6B))
            else:
                self.rpm[address] = 0
                if command == 0xF3:
                    self.flags[address] &= ~1
                self.rx = bytes((address, command, 2, 0x6B))
        else:
            raise AssertionError(f"unexpected command {packet.hex()}")
        if command == self.partial_command:
            self.partial_command = None
            return len(packet) - 1
        return len(packet)

    def transact(self, connection, packet, **kwargs):
        assert kwargs["deadline"] > 0
        connection.write(packet)
        return connection.read(4096)

    def api(self):
        return types.SimpleNamespace(
            SERIAL_LOCK=contextlib.nullcontext(), STOP_EVENT=threading.Event(),
            direct=types.SimpleNamespace(open_serial=lambda *args: self, transact=self.transact),
            current_port=lambda: "FAKE", current_baud=lambda: 921600,
            calibration_data=lambda: {"encoder_positive_direction": [0, 0, 1, 1]},
            manual_test_emit=lambda event, **fields: self.events.append((event, fields)),
            command_frame=lambda address, action, data: (
                bytes((address, 0xFE, 0x98, 0, 0x6B)) if action == "stop"
                else bytes((address, 0xF3, 0xAB, 0, 0, 0x6B))))


class AdapterTests(unittest.TestCase):
    def test_golden_seed_keeps_20ma_and_absolute_sign(self):
        self.assertEqual(io.disabled_position_seed(1, -12.3, 0).hex(), "01cb01000a0000007b010000146b")
        self.assertEqual(io.disabled_position_seed(3, -12.3, 1).hex(), "03cb00000a0000007b010000146b")

    def test_successful_handshake_is_not_motion_success_and_restores_exactly(self):
        bus = FakeBus()
        out = io.run_hardware_position_probe(bus.api())
        self.assertTrue(out["handshake_passed"])
        self.assertFalse(out["success"])
        self.assertFalse(out["enable_sent"])
        self.assertFalse(out["motion_observed"])
        self.assertTrue(out["motion_command_attempted"])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertTrue(out["configuration_restored"])
        self.assertEqual(bus.configs, bus.original)
        self.assertEqual(bus.restore_writes, [1, 2, 3, 4])
        self.assertFalse(any(p[1] in (0xC5, 0xFF, 0x45) for p in bus.calls))
        self.assertTrue(all(p[3] == 0 for p in bus.calls if p[1] in (0x48, 0xF3)))
        first_cb = next(i for i, p in enumerate(bus.calls) if p[1] == 0xCB)
        self.assertEqual(sum(p[1] == 0x48 for p in bus.calls[:first_cb]), 4)
        writes = [f for e, f in bus.events if e == "position_probe_serial_write"]
        self.assertEqual(len(writes), len(bus.calls))
        self.assertTrue(all(f["bytes_written"] == f["expected_bytes"] for f in writes))

    def test_disabled_cb_rejection_never_enables_or_seeds_later_axes(self):
        bus = FakeBus()
        bus.reject_seed = 2
        out = io.run_hardware_position_probe(bus.api())
        self.assertFalse(out["handshake_passed"])
        self.assertEqual([p[0] for p in bus.calls if p[1] == 0xCB], [1, 2])
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertIn("拒绝", out["reason"])

    def test_initial_enabled_never_takes_ownership(self):
        bus = FakeBus()
        bus.flags[2] = 3
        out = io.run_hardware_position_probe(bus.api())
        self.assertFalse(out["final_disabled_confirmed"])
        self.assertFalse(any(p[1] in (0x48, 0xCB, 0xF3, 0xFE) for p in bus.calls))

    def test_unknown_flags_rejected_before_mutation(self):
        for flags in (0xE2, 0xEE, 0x80, 8):
            bus = FakeBus()
            bus.flags[1] = flags
            io.run_hardware_position_probe(bus.api())
            self.assertFalse(any(p[1] in (0x48, 0xCB, 0xF3, 0xFE) for p in bus.calls))

    def test_configuration_contract_mismatch_never_writes(self):
        for index, value in ((5, 0), (25, 2), (26, 1), (16, 0), (18, 0)):
            bus = FakeBus()
            bad = bytearray(bus.configs[1])
            bad[index] = value
            if index in (16, 18):
                bad[index + 1] = 0
            bus.configs[1] = bytes(bad)
            io.run_hardware_position_probe(bus.api())
            self.assertFalse(any(p[1] == 0x48 for p in bus.calls))

    def test_envelope_ack_unknown_stops_before_cb_and_restores_touched(self):
        bus = FakeBus()
        bus.reject_envelope = 2
        out = io.run_hardware_position_probe(bus.api())
        self.assertFalse(any(p[1] == 0xCB for p in bus.calls))
        self.assertEqual(bus.restore_writes, [1, 2])
        self.assertTrue(out["configuration_restored"])
        self.assertEqual(bus.configs, bus.original)

    def test_restore_unknown_does_not_continue_restoring_remaining_axes(self):
        bus = FakeBus()
        bus.reject_restore = 2
        out = io.run_hardware_position_probe(bus.api())
        self.assertFalse(out["configuration_restored"])
        self.assertEqual(bus.restore_writes, [1, 2])
        self.assertEqual(out["configuration_restore"], {"restored": [1], "unknown": [2], "not_attempted": [3, 4]})

    def test_partial_write_is_logged_and_never_treated_as_success(self):
        bus = FakeBus()
        bus.partial_command = 0x48
        out = io.run_hardware_position_probe(bus.api())
        failures = [f for e, f in bus.events if e == "position_probe_serial_write" and f["error"]]
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0]["bytes_written"], failures[0]["expected_bytes"] - 1)
        self.assertFalse(out["handshake_passed"])
        self.assertTrue(out["configuration_restored"])

    def test_readback_mismatch_never_sends_cb(self):
        bus = FakeBus()
        bus.corrupt_readback = 2
        out = io.run_hardware_position_probe(bus.api())
        self.assertFalse(any(p[1] == 0xCB for p in bus.calls))
        self.assertTrue(out["configuration_restored"])

    def test_seed_uses_latest_position_and_observes_response_before_stop(self):
        bus = FakeBus()
        original_transact = bus.transact
        def transact(connection, packet, **kwargs):
            if connection.phase == "before_disabled_seed" and packet[1] == 0x36:
                bus.positions[packet[0]] += 0.1
            reply = original_transact(connection, packet, **kwargs)
            if packet[:2] == bytes((4, 0xCB)):
                bus.positions[4] += 0.3
            return reply
        api = bus.api()
        api.direct.transact = transact
        out = io.run_hardware_position_probe(api)
        self.assertAlmostEqual(out["q0"][1], -12.2)
        self.assertFalse(out["handshake_passed"])
        self.assertTrue(out["motion_observed"])
        self.assertTrue(out["configuration_restored"])

    def test_pre_restore_read_failure_preserves_not_attempted_classification(self):
        bus = FakeBus()
        original_transact = bus.transact
        def transact(connection, packet, **kwargs):
            if connection.phase == "restore_original_before" and packet[0] == 2:
                raise RuntimeError("lost prewrite read")
            return original_transact(connection, packet, **kwargs)
        api = bus.api()
        api.direct.transact = transact
        out = io.run_hardware_position_probe(api)
        self.assertFalse(out["configuration_restored"])
        self.assertEqual(out["configuration_restore"], {"restored": [1], "unknown": [], "not_attempted": [2, 3, 4]})

    def test_write_does_not_extend_existing_tighter_timeout(self):
        bus = FakeBus()
        bus.write_timeout = 0.005
        connection = io._RecordedConnection(bus, lambda *args, **kwargs: None)
        connection.write(bytes((1, 0x3A, 0x6B)))
        self.assertEqual(bus.write_timeout, 0.005)

    def test_zero_current_takeover_seeds_immediately_and_never_raises_current(self):
        bus = FakeBus()
        bus.allow_enable = True
        out = io.run_hardware_position_probe(bus.api(), zero_current_takeover=True)
        self.assertTrue(out["handshake_passed"])
        self.assertFalse(out["success"])
        self.assertEqual(out["stage"], "zero_current_takeover")
        self.assertTrue(out["takeover_passed"])
        self.assertEqual(out["enable_attempted"], [1, 2, 3, 4])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertTrue(out["configuration_restored"])
        self.assertEqual(bus.configs, bus.original)
        for index, packet in enumerate(bus.calls):
            if packet[1] == 0xF3 and packet[3] == 1:
                self.assertEqual(bus.calls[index + 1][:2], bytes((packet[0], 0xCB)))
            self.assertNotIn(packet[1], (0x45, 0xC5, 0xFF))
        self.assertFalse(any(f.get("phase") == "disabled_cb_seed" for _, f in bus.events))

    def test_zero_current_enable_or_seed_rejection_stops_later_axes(self):
        for rejection in ("reject_enable", "reject_seed"):
            bus = FakeBus()
            bus.allow_enable = True
            setattr(bus, rejection, 2)
            out = io.run_hardware_position_probe(bus.api(), zero_current_takeover=True)
            self.assertFalse(out["handshake_passed"])
            self.assertEqual(out["enable_attempted"], [1, 2])
            self.assertTrue(out["final_disabled_confirmed"])
            self.assertTrue(out["configuration_restored"])

    def test_zero_current_feedback_gates_stop_before_next_enable(self):
        for failure in ("rpm", "position", "current", "unexpected_enabled"):
            bus = FakeBus()
            bus.allow_enable = True
            base_transact = bus.transact
            def transact(connection, packet, **kwargs):
                reply = base_transact(connection, packet, **kwargs)
                if packet[:2] == bytes((1, 0xCB)):
                    if failure == "rpm": bus.rpm[1] = 0.4
                    elif failure == "position": bus.positions[1] += 0.3
                    elif failure == "current": bus.current[1] = 51
                    else: bus.flags[2] |= 1
                return reply
            api = bus.api()
            api.direct.transact = transact
            out = io.run_hardware_position_probe(api, zero_current_takeover=True)
            self.assertFalse(out["handshake_passed"], failure)
            self.assertEqual(out["enable_attempted"], [1], failure)
            self.assertTrue(out["final_disabled_confirmed"], failure)
            self.assertTrue(out["configuration_restored"], failure)

    def test_zero_current_envelope_failure_never_enables(self):
        bus = FakeBus()
        bus.allow_enable = True
        bus.reject_envelope = 3
        out = io.run_hardware_position_probe(bus.api(), zero_current_takeover=True)
        self.assertFalse(out["enable_sent"])
        self.assertTrue(out["configuration_restored"])

    def test_zero_takeover_parameter_is_only_fixed_boolean(self):
        for invalid in (1, "true", 150, None):
            bus = FakeBus()
            with self.assertRaises(ValueError):
                io.run_hardware_position_probe(bus.api(), zero_current_takeover=invalid)
            self.assertEqual(bus.calls, [])

    def test_zero_actual_current_checked_before_first_enable(self):
        bus = FakeBus()
        bus.allow_enable = True
        base_transact = bus.transact
        def transact(connection, packet, **kwargs):
            if connection.phase == "before_zero_current_enable":
                bus.current[1] = 90
            return base_transact(connection, packet, **kwargs)
        api = bus.api()
        api.direct.transact = transact
        out = io.run_hardware_position_probe(api, zero_current_takeover=True)
        self.assertFalse(out["enable_sent"])
        self.assertFalse(out["takeover_passed"])
        self.assertTrue(out["configuration_restored"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
