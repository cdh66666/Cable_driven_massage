"""Injected fake clock/bus/master-poll tests: no web import, serial, or hardware IO."""
import copy
import types
import unittest

import y42_position_microprobe_io as io
from test_y42_position_probe_io import FakeBus as AdmissionFakeBus


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeBus(AdmissionFakeBus):
    def __init__(self):
        self.write_timeout_updates = []
        super().__init__()
        self.clock = FakeClock()
        for address in io.IDS:
            cfg = bytearray(self.configs[address])
            cfg[16:20] = (3000).to_bytes(2, "big") * 2
            self.configs[address] = bytes(cfg)
        self.original = copy.deepcopy(self.configs)
        self.allow_enable = True
        self.timeout = 0.001
        self.record_hook = None
        self.feedback_tail = b""
        self.unsolicited_on_poll = b""
        self.aa_ack = bytes((1, 0xCB, 2, 0x6B))
        self.stop_stream_failure = False
        self.query_counts = {address: 0 for address in io.IDS}
        self.query_packets = []
        self.last_targets = {}
        self.cb_counts = {}
        self.command_currents = {}
        self.promotions = []
        self.aggregate_count = 0
        self.motion_active = False
        self.position_cb_packets = []
        self.motion_cb_packets = []
        self.motion_cb_indexes = []
        self.cadence_events = []
        self.command_delay = 0.0
        self.motion_axis_delay = 0.0
        self.transaction_delay = 0.0
        self.fail_cap = None
        self.no_response = False
        self.timer_calls = []
        self.partial_aggregate = False
        self.reject_restore_axis = None
        self.reject_motion_axis = None
        self.stream_calls = []
        self.disabled_stop_ack = 0xE2
        self.disable_ack = 2
        self.post_disable_rpm = 0.0
        self.disable_keeps_enabled = False

    def read(self, size=1):
        """Make the synchronous poll's 0.5 ms quiet window progress fake time."""
        out = super().read(size)
        if not out:
            self.clock.sleep(0.0005)
        return out

    @property
    def write_timeout(self):
        return self._write_timeout

    @write_timeout.setter
    def write_timeout(self, value):
        self.write_timeout_updates.append({"value": value,
            "aggregate_count": getattr(self, "aggregate_count", 0)})
        self._write_timeout = value

    def record(self, address):
        return types.SimpleNamespace(address=address, position=self.positions[address],
            rpm=self.rpm[address], phase_current=self.current[address], temperature=25,
            flags=self.flags[address], sampled_at=self.clock(), raw_packet_hex=f"{address:02x}43-fake")

    def reset_input_buffer(self):
        self.rx = b""

    @property
    def in_waiting(self):
        return len(self.rx)

    def write(self, packet):
        packet = bytes(packet)
        address, command = packet[:2]
        if command == 0x43:
            assert packet == bytes((address, 0x43, 0x7A, 0x6B))
            self.calls.append(packet)
            self.query_packets.append(packet)
            self.query_counts[address] += 1
            records = [self.record(address)]
            if self.record_hook:
                records = self.record_hook(self, records)
            if isinstance(records, (bytes, bytearray)):
                self.rx = bytes(records)
            elif not records:
                self.rx = b""
            else:
                self.rx = b"".join(self.feedback_frame(record) for record in records)
            if self.feedback_tail:
                self.rx += bytes(self.feedback_tail)
                self.feedback_tail = b""
            return len(packet)
        if command == 0xAA:
            self.calls.append(packet)
            self.aggregate_count += 1
            assert all(int.from_bytes(cfg[16:18], "big") == 150 for cfg in self.configs.values())
            assert len(packet) == int.from_bytes(packet[2:4], "big") == 61
            active = 0
            for start in range(4, 60, 14):
                cb = packet[start:start + 14]
                self.apply_cb(cb)
                active += abs(self.last_targets[cb[0]] - self.origins[cb[0]]) > 1e-9
            assert active <= 1, "more than one axis commanded away from q0"
            self.clock.sleep(self.command_delay)
            if self.partial_aggregate:
                self.partial_aggregate = False
                return len(packet) - 1
            self.rx = bytes(self.aa_ack)
            return len(packet)
        if command in (0x48, 0x45):
            self.calls.append(packet)
            if command == 0x45:
                assert len(packet) == 7 and packet[2:4] == b"\x66\x00"
                cfg = bytearray(self.configs[address]); cfg[16:18] = packet[4:6]
                cfg = bytes(cfg)
            else:
                assert len(packet) == 37 and packet[2:4] == b"\xd1\x00"
                cfg = bytes((address, 0x42, 0x25, 0x18)) + packet[4:-1] + b"\x6b"
            cap = int.from_bytes(cfg[16:18], "big")
            if cap in (20, 150):
                before = self.configs[address]
                assert cfg[:16] + cfg[18:] == before[:16] + before[18:], "promotion changed unrelated configuration"
                assert set(self.last_targets) == set(io.IDS)
                assert all(self.command_currents[axis] == 20 for axis in io.IDS)
                assert all(self.last_targets[axis] == self.origins[axis] for axis in io.IDS)
                self.promotions.append((address, cap, dict(self.last_targets), dict(self.command_currents)))
            if cap == 3000:
                assert all(not flags & 1 for flags in self.flags.values())
                self.restore_writes.append(address)
            self.configs[address] = cfg
            if cap == 150 and all(int.from_bytes(item[16:18], "big") == 150
                                  for item in self.configs.values()):
                self.motion_active = True
            failed = ((address, cap) == self.fail_cap
                      or cap == 3000 and address == self.reject_restore_axis)
            self.rx = bytes((address, command, 0xE2 if failed else 2, 0x6B))
            return len(packet)
        if command == 0xCB:
            self.apply_cb(packet)
            self.position_cb_packets.append(packet)
            if self.motion_active:
                self.clock.sleep(self.command_delay + self.motion_axis_delay)
        written = super().write(packet)
        if command == 0xCB and self.motion_active:
            self.motion_cb_packets.append(packet)
            self.motion_cb_indexes.append(len(self.calls) - 1)
            if address == self.reject_motion_axis:
                self.rx = bytes((address, command, 3, 0x6B))
        if command == 0xCB and self.motion_active and self.partial_aggregate:
            self.partial_aggregate = False
            return len(packet) - 1
        if command == 0xFE and not self.flags[address] & 1:
            self.rx = bytes((address, command, self.disabled_stop_ack, 0x6B))
        if command == 0xF3 and packet[3] == 0 and self.aggregate_count:
            self.rx = bytes((address, command, self.disable_ack, 0x6B))
            self.rpm[address] = self.post_disable_rpm
            if self.disable_keeps_enabled:
                self.flags[address] |= 1
        return written

    def apply_cb(self, packet):
        address = packet[0]
        assert packet[1] == 0xCB and packet[3:5] == b"\x00\x0a"
        assert packet[9:11] == b"\x01\x00" and packet[-1] == 0x6B
        magnitude = int.from_bytes(packet[5:9], "big") / 10.0
        direction = packet[2] ^ (0, 0, 1, 1)[address - 1]
        target = magnitude * (-1 if direction else 1)
        current = int.from_bytes(packet[11:13], "big")
        assert 20 <= current <= 150
        if not self.last_targets:
            self.origins = dict(self.positions)
        # The production adapter rebinds q0 after the newly enabled axis
        # settles, then sends a second CB seed. Mirror that accepted rebase so
        # promotion checks compare against the current settled reference.
        if not self.motion_active and self.cb_counts.get(address, 0) >= 1:
            self.origins[address] = target
        self.cb_counts[address] = self.cb_counts.get(address, 0) + 1
        self.last_targets[address] = target
        self.command_currents[address] = current
        if self.motion_active and not self.no_response:
            self.positions[address] = target

    def transact(self, connection, packet, **kwargs):
        self.clock.sleep(self.transaction_delay)
        return super().transact(connection, packet, **kwargs)

    def stop_stream(self, connection):
        self.stream_calls.append(("stop", int.from_bytes(self.configs[1][16:18], "big")))
        if self.unsolicited_on_poll:
            self.rx = bytes(self.unsolicited_on_poll)
        return not self.stop_stream_failure

    @staticmethod
    def feedback_frame(record):
        address = int(record.address)
        response = bytearray(37)
        response[:4] = bytes((address, 0x43, 0x25, 0x0C))
        response[8:10] = int(record.phase_current).to_bytes(2, "big")
        response[19] = int(float(record.rpm) < 0)
        response[20:22] = round(abs(float(record.rpm)) * 10).to_bytes(2, "big")
        response[22] = int(float(record.position) < 0)
        response[23:27] = round(abs(float(record.position)) * 10).to_bytes(4, "big")
        response[33] = int(record.temperature)
        response[35] = int(record.flags)
        response[36] = 0x6B
        return bytes(response)

    def api(self):
        api = super().api()
        api.direct.transact = self.transact
        api.stop_timed_feedback_stream = self.stop_stream
        api.set_high_resolution_timer = lambda value: self.timer_calls.append(value) or True
        base_emit = api.manual_test_emit
        def emit(event, **fields):
            if event == "position_microprobe_command_cadence":
                self.aggregate_count += 1
                self.cadence_events.append(fields)
            return base_emit(event, **fields)
        api.manual_test_emit = emit
        return api

    def run(self, api=None):
        return io.run_hardware_position_microprobe(self.api() if api is None else api,
            clock=self.clock, sleep=self.clock.sleep)


class MicroprobeAdapterTests(unittest.TestCase):
    def test_master_polling_has_no_reader_thread_or_autonomous_stream(self):
        bus = FakeBus()
        out = bus.run()
        self.assertTrue(out["success"], out["reason"])
        self.assertNotIn("reader_running", bus.__dict__)
        self.assertNotIn("stream_active", bus.__dict__)
        self.assertEqual(bus.stream_calls, [("stop", 150)])
        counts = [bus.query_counts[address] for address in io.IDS]
        self.assertTrue(all(count > 0 for count in counts))
        self.assertLessEqual(max(counts) - min(counts), 1)

    def test_unsolicited_rx_aborts_before_motion_without_clearing_tail(self):
        bus = FakeBus()
        bus.unsolicited_on_poll = bytes((1, 0x43, 0x25, 0x0C))
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertEqual(bus.aggregate_count, 0)
        self.assertIn("未请求回包", out["reason"])
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])

    def test_fixed_aggregate_golden_preserves_absolute_sign_and_caps(self):
        packet = io.microprobe_position_frame([-12.3, 11.2, -2.7, 4.1], 150, [0, 0, 1, 1])
        self.assertEqual(len(packet), 61)
        self.assertEqual(packet[:4], b"\x00\xaa\x00\x3d")
        self.assertEqual(packet[4:18].hex(), "01cb01000a0000007b010000966b")
        self.assertEqual(packet[32:46].hex(), "03cb00000a0000001b010000966b")
        for current in (19, 151, 300, True):
            with self.assertRaises(ValueError):
                io.microprobe_position_frame([0] * 4, current, [0, 0, 1, 1])

    def test_replacement_motor_uses_saved_direction_without_changing_caps(self):
        old = io.microprobe_position_frame([-12.3, 11.2, -2.7, 4.1], 150, [0, 0, 1, 1])
        new = io.microprobe_position_frame([-12.3, 11.2, -2.7, 4.1], 150, [0, 0, 1, 0])
        self.assertEqual([i for i, pair in enumerate(zip(old, new)) if pair[0] != pair[1]], [48])
        self.assertEqual(new[48], 0)
        for invalid in (None, [], [0, 0, 1], [0, 0, 1, 2], [0, 0, 1, True]):
            with self.assertRaises(ValueError):
                io.microprobe_position_frame([0] * 4, 150, invalid)

    def test_write_timeout_is_prepared_before_stream_and_not_reset_per_tick(self):
        bus = FakeBus()
        out = bus.run()
        self.assertTrue(out["success"], out["reason"])
        self.assertGreater(bus.aggregate_count, 100)
        self.assertEqual([row["value"] for row in bus.write_timeout_updates], [None, 0.01])
        self.assertTrue(all(row["aggregate_count"] == 0 for row in bus.write_timeout_updates))

    def test_preparing_stream_timeout_never_widens_an_existing_tighter_limit(self):
        bus = FakeBus()
        bus.write_timeout = 0.005
        before = list(bus.write_timeout_updates)
        out = bus.run()
        self.assertTrue(out["success"], out["reason"])
        self.assertEqual(bus.write_timeout_updates, before)
        self.assertEqual(bus.write_timeout, 0.005)

    def test_complete_probe_strict_admission_promotions_cadence_and_exact_restore(self):
        bus = FakeBus()
        out = bus.run()
        self.assertTrue(out["success"], out["reason"])
        self.assertTrue(out["takeover_passed"])
        self.assertTrue(out["final_hold_verified"])
        self.assertTrue(out["motion_observed"])
        self.assertFalse(out["unexpected_motion_observed"])
        self.assertTrue(out["response_passed"])
        self.assertTrue(out["cadence_passed"])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertTrue(out["configuration_restored"])
        self.assertEqual(bus.configs, bus.original)
        self.assertEqual(len(out["axis_results"]), 4)
        self.assertFalse(out["xyz_precision_measured"])
        self.assertFalse(out["tension_verified"])
        self.assertEqual([(axis, cap) for axis, cap, _, _ in bus.promotions],
            [(axis, cap) for cap in (20, 150) for axis in (2, 3, 4, 1)])
        self.assertEqual(bus.stream_calls, [("stop", 150)])
        self.assertEqual(bus.timer_calls, [True, False])
        for index, packet in enumerate(bus.calls):
            if packet[1] == 0xF3 and packet[3] == 1:
                self.assertEqual(bus.calls[index + 1][:2], bytes((packet[0], 0xCB)))
            self.assertNotIn(packet[1], (0xC5, 0xC6, 0xFF))
        writes = [fields for event, fields in bus.events if event == "position_probe_serial_write"]
        self.assertEqual(len(writes), len(bus.calls))
        self.assertEqual(out["cadence"]["completed_commands"], bus.aggregate_count)
        stamps = out["cadence"]["commands"]
        self.assertAlmostEqual(out["cadence"]["measured_command_hz"],
            (len(stamps) - 1) / (stamps[-1]["started_at"] - stamps[0]["started_at"]))
        commands = out["probe_result"]["commands"]
        self.assertEqual(commands[0]["current_ma"], 20)
        self.assertEqual(commands[-1]["current_ma"], 20)
        self.assertTrue(all(abs(row["observed_signed_response_deg"] - 0.5) < 1e-8 for row in out["axis_results"]))
        feedback_stats = out["stream_runs"][-1]["feedback_per_axis"]
        self.assertTrue(all(10 < values["measured_hz"] < 15 for values in feedback_stats.values()))
        self.assertTrue(all(abs(bus.query_counts[address] - bus.query_counts[1]) <= 1
                            for address in io.IDS))
        self.assertEqual(out["cleanup_stop_e2_axes"], [1, 2, 3, 4])
        self.assertTrue(out["cleanup_stop_e2_disabled_confirmed"])

    def test_enable_settling_accepts_one_axis_offset_and_reseeds_q0(self):
        bus = FakeBus()
        base_transact = bus.transact
        shifted = False

        def transact(connection, packet, **kwargs):
            nonlocal shifted
            reply = base_transact(connection, packet, **kwargs)
            if not shifted and packet[:2] == bytes((1, 0xCB)):
                bus.positions[1] += 0.6
                shifted = True
            return reply

        api = bus.api()
        api.direct.transact = transact
        out = bus.run(api)

        self.assertTrue(out["success"], out["reason"])
        settled = [fields for event, fields in bus.events
                    if event == "position_probe_enable_settled"]
        self.assertEqual(len(settled), 4)
        self.assertAlmostEqual(settled[0]["offset_deg"], 0.6)
        self.assertEqual(settled[0]["max_offset_deg"], 3.0)
        self.assertTrue(out["takeover_passed"])
        self.assertTrue(out["configuration_restored"])

    def test_enable_settling_rejects_new_axis_over_three_degrees_before_motion(self):
        bus = FakeBus()
        base_transact = bus.transact
        shifted = False

        def transact(connection, packet, **kwargs):
            nonlocal shifted
            reply = base_transact(connection, packet, **kwargs)
            if not shifted and packet[:2] == bytes((1, 0xCB)):
                bus.positions[1] += 3.1
                shifted = True
            return reply

        api = bus.api()
        api.direct.transact = transact
        out = bus.run(api)

        self.assertFalse(out["success"])
        self.assertIn("超过 3°", out["reason"])
        self.assertEqual(bus.aggregate_count, 0)
        self.assertEqual(bus.promotions, [])
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])

    def test_enable_settling_rejects_continuous_subthree_degree_drift(self):
        bus = FakeBus()
        base_transact = bus.transact

        def transact(connection, packet, **kwargs):
            reply = base_transact(connection, packet, **kwargs)
            if connection.phase == "enable_settling" and packet[:2] == bytes((1, 0x36)):
                bus.positions[1] += 0.04
            return reply

        api = bus.api()
        api.direct.transact = transact
        out = bus.run(api)

        self.assertFalse(out["success"])
        self.assertIn("2.5秒内未落稳", out["reason"])
        self.assertLess(abs(bus.positions[1] - (-12.3)), 3.0)
        self.assertEqual(bus.aggregate_count, 0)
        self.assertEqual(bus.promotions, [])
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])

    def test_enable_settling_keeps_other_axes_at_point_two_degree_guard(self):
        bus = FakeBus()
        base_transact = bus.transact
        shifted = False

        def transact(connection, packet, **kwargs):
            nonlocal shifted
            reply = base_transact(connection, packet, **kwargs)
            if not shifted and packet[:2] == bytes((1, 0xCB)):
                bus.positions[1] += 0.6
                bus.positions[2] += 0.3
                shifted = True
            return reply

        api = bus.api()
        api.direct.transact = transact
        out = bus.run(api)

        self.assertFalse(out["success"])
        self.assertIn("M2接管位置变化超过 0.2°", out["reason"])
        self.assertEqual(
            [packet[0] for packet in bus.calls
             if packet[1] == 0xF3 and packet[3] == 1], [1])
        self.assertEqual(bus.aggregate_count, 0)
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])

    def test_failure_to_apply_zero_never_enables(self):
        bus = FakeBus(); bus.fail_cap = (2, 0)
        out = bus.run()
        self.assertFalse(out["takeover_passed"])
        self.assertFalse(any(packet[1] == 0xF3 and packet[3] == 1 for packet in bus.calls))
        self.assertTrue(out["configuration_restored"])
        self.assertEqual(bus.configs, bus.original)

    def test_enable_or_cb_rejection_never_promotes(self):
        for rejection in ("reject_enable", "reject_seed"):
            bus = FakeBus(); setattr(bus, rejection, 2)
            out = bus.run()
            self.assertFalse(out["success"])
            self.assertEqual(bus.promotions, [])
            self.assertEqual([packet[0] for packet in bus.calls if packet[1] == 0xF3 and packet[3] == 1], [1, 2])
            self.assertTrue(out["configuration_restored"])

    def test_each_promotion_failure_prevents_pure_probe_and_restores(self):
        for cap in (20, 150):
            bus = FakeBus(); bus.fail_cap = (2, cap)
            out = bus.run()
            self.assertFalse(out["success"])
            self.assertEqual(bus.aggregate_count, 0)
            self.assertTrue(out["configuration_restored"])
            self.assertTrue(out["final_disabled_confirmed"])

    def test_unsafe_original_frame_followed_by_safe_frame_still_stops(self):
        for field, bad in (("rpm", 3.1), ("phase_current", 351), ("position", 999), ("flags", 0x80), ("temperature", 45)):
            bus = FakeBus()
            def hook(reader, records):
                if bus.aggregate_count >= 2:
                    bus.fault_command_count = bus.aggregate_count
                    corrupted = copy.copy(records[0]); setattr(corrupted, field, bad)
                    return [corrupted]
                return records
            bus.record_hook = hook
            out = bus.run()
            self.assertFalse(out["success"], field)
            self.assertEqual(bus.aggregate_count, bus.fault_command_count)
            self.assertTrue(out["final_disabled_confirmed"], out["reason"])
            self.assertTrue(out["configuration_restored"], out["reason"])
            self.assertIn("原始0x43", out["reason"])

    def test_extra_polled_frame_aborts_before_another_command(self):
        bus = FakeBus()
        def hook(reader, records):
            if bus.aggregate_count >= 2:
                bus.fault_command_count = bus.aggregate_count
                return FakeBus.feedback_frame(records[0]) + b"\x00"
            return records
        bus.record_hook = hook
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertEqual(bus.aggregate_count, bus.fault_command_count)
        self.assertIn("额外/错误帧", out["reason"])
        self.assertTrue(out["configuration_restored"])

    def test_autonomous_feedback_shutdown_is_required_before_motion(self):
        bus = FakeBus(); bus.stop_stream_failure = True
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertEqual(bus.aggregate_count, 0)
        self.assertTrue(all(value == 20 for value in bus.command_currents.values()))
        self.assertTrue(out["configuration_restored"])

    def test_no_response_stops_without_escalation(self):
        bus = FakeBus(); bus.no_response = True
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertIn("响应不超过0.1度", out["reason"])
        self.assertTrue(out["configuration_restored"])
        self.assertLess(out["probe_result"]["elapsed_seconds"], 8)
        self.assertTrue(all(20 <= row["current_ma"] <= 150 for row in out["probe_result"]["commands"]))

    def test_late_or_partial_writes_abort_and_archive_failure(self):
        for kind in ("late", "partial"):
            bus = FakeBus()
            if kind == "late": bus.command_delay = 0.011
            else: bus.partial_aggregate = True
            out = bus.run()
            self.assertFalse(out["success"])
            self.assertEqual(bus.aggregate_count, 0)
            self.assertEqual(len(bus.motion_cb_packets), 1)
            self.assertTrue(out["configuration_restored"])
            failed = [fields for event, fields in bus.events if event == "position_probe_serial_write" and fields["error"]]
            self.assertTrue(failed)
            self.assertEqual(bus.timer_calls, [True, False])

    def test_query_deadline_is_checked_even_if_injected_transport_returns_late(self):
        bus = FakeBus(); bus.transaction_delay = 0.351
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertFalse(any(packet[1] in (0x48, 0xCB, 0xAA) for packet in bus.calls))
        self.assertIn("截止时间", out["reason"])

    def test_missing_warmup_axis_never_starts_motion(self):
        bus = FakeBus()
        bus.record_hook = lambda reader, records: [] if records[0].address == 4 else records
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertEqual(bus.aggregate_count, 0)
        self.assertTrue(all(value == 20 for value in bus.command_currents.values()))
        self.assertIn("12毫秒", out["reason"])
        self.assertTrue(out["configuration_restored"])

    def test_missing_live_feedback_becomes_stale_and_stops(self):
        bus = FakeBus()
        bus.record_hook = lambda reader, records: [] if bus.aggregate_count >= 2 else records
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertIn("12毫秒", out["reason"])
        self.assertTrue(out["configuration_restored"])

    def test_extra_polled_tail_is_not_silently_dropped(self):
        bus = FakeBus()
        def hook(reader, records):
            if bus.aggregate_count >= 2:
                bus.fault_command_count = bus.aggregate_count
                return FakeBus.feedback_frame(records[0]) + b"\x01\x00\xee\x6b"
            return records
        bus.record_hook = hook
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertIn("额外/错误帧", out["reason"])
        self.assertTrue(out["configuration_restored"])

    def test_late_command_gap_aborts_without_catchup(self):
        bus = FakeBus()
        def oversleep(seconds):
            bus.clock.sleep(seconds + (0.081 if bus.aggregate_count >= 2 else 0))
        out = io.run_hardware_position_microprobe(bus.api(), clock=bus.clock, sleep=oversleep)
        self.assertFalse(out["success"])
        self.assertEqual(bus.aggregate_count, 2)
        self.assertIn("实际间隔", out["reason"])
        self.assertTrue(out["configuration_restored"])

    def test_admission_budget_is_not_extended_by_repeated_queries(self):
        # Four full replies replace twenty scalar queries; use a genuinely
        # over-budget transport delay rather than relying on the old count.
        bus = FakeBus(); bus.transaction_delay = 0.120
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertIn("时间预算", out["reason"])
        self.assertLessEqual(out["elapsed_seconds"], 45.1)

    def test_original_caps_must_match_exact_saved_baseline(self):
        for field in (16, 18):
            bus = FakeBus()
            cfg = bytearray(bus.configs[1]); cfg[field:field + 2] = (150).to_bytes(2, "big")
            bus.configs[1] = bytes(cfg)
            out = bus.run()
            self.assertFalse(out["success"])
            self.assertFalse(any(packet[1] == 0x48 for packet in bus.calls))

    def test_restore_rejection_preserves_uncertainty_and_stops_remaining_restore(self):
        bus = FakeBus(); bus.reject_restore_axis = 2
        out = bus.run()
        self.assertFalse(out["success"])
        self.assertFalse(out["configuration_restored"])
        self.assertEqual(out["configuration_restore"], {"restored": [1], "unknown": [2], "not_attempted": [3, 4]})
        self.assertTrue(out["final_disabled_confirmed"])

    def test_disabled_stop_e2_needs_disable_ack_and_exact_zero_rpm(self):
        for kind in ("disable_ack", "nonzero_rpm", "enabled_flag"):
            bus = FakeBus()
            if kind == "disable_ack": bus.disable_ack = 0xE2
            elif kind == "nonzero_rpm": bus.post_disable_rpm = 0.1
            else: bus.disable_keeps_enabled = True
            out = bus.run()
            self.assertFalse(out["success"], kind)
            self.assertFalse(out["configuration_restored"], kind)
            self.assertFalse(out["cleanup_stop_e2_disabled_confirmed"], kind)
            self.assertEqual(bus.restore_writes, [], kind)

    def test_other_stop_ack_is_not_accepted_as_disabled_e2(self):
        bus = FakeBus(); bus.disabled_stop_ack = 0xE3
        out = bus.run()
        self.assertFalse(out["success"])
        # A rejected STOP is still a failure, but does not skip the four
        # DISABLE attempts. Their ACKs + fresh off/zero permit restoration.
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])
        self.assertEqual(len(bus.restore_writes), 4)
        self.assertIn("stopACK未确认", out["reason"])

    def test_stable_20hz_response_is_not_25hz_success(self):
        bus = FakeBus()
        def coarse_sleep(seconds):
            bus.clock.sleep(max(seconds, 0.050) if bus.aggregate_count else seconds)
        out = io.run_hardware_position_microprobe(bus.api(), clock=bus.clock, sleep=coarse_sleep)
        self.assertTrue(out["response_passed"], out["reason"])
        self.assertFalse(out["cadence_passed"])
        self.assertFalse(out["success"])
        self.assertTrue(18.0 <= out["cadence"]["measured_command_hz"] <= 21.0)
        self.assertGreater(out["cadence"]["skipped_ticks"], 0)
        self.assertLessEqual(out["cadence"]["max_interval_seconds"], 0.12)
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])

    def test_one_missed_tick_is_not_hidden_by_average_near_25hz(self):
        bus = FakeBus()
        delayed = False
        def once_late(seconds):
            nonlocal delayed
            extra = 0.041 if bus.aggregate_count >= 20 and not delayed else 0
            delayed |= bool(extra)
            bus.clock.sleep(seconds + extra)
        out = io.run_hardware_position_microprobe(bus.api(), clock=bus.clock, sleep=once_late)
        self.assertTrue(out["response_passed"], out["reason"])
        self.assertTrue(22.5 <= out["cadence"]["measured_command_hz"] <= 27.5)
        self.assertGreater(out["cadence"]["skipped_ticks"], 0)
        self.assertFalse(out["cadence_passed"])
        self.assertFalse(out["success"])
        self.assertTrue(out["configuration_restored"])

    def test_user_stop_during_stream_stops_and_restores(self):
        bus = FakeBus(); api = bus.api()
        def hook(reader, records):
            if bus.aggregate_count >= 2:
                bus.fault_command_count = bus.aggregate_count
                api.STOP_EVENT.set()
            return records
        bus.record_hook = hook
        out = bus.run(api)
        self.assertFalse(out["success"])
        self.assertEqual(bus.aggregate_count, bus.fault_command_count)
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
