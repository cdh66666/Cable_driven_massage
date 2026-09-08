"""Exercise production observation adapter/API via AST and injected fake IO only."""
import ast
import copy
import math
from dataclasses import dataclass
from pathlib import Path
import threading
import types
import unittest

SOURCE = Path(__file__).with_name("y42_pc_direct_web.py")
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


@dataclass
class Telemetry:
    address: int
    position: float = 0.0
    rpm: float = 0.0
    phase_current: float = 150.0
    temperature: float = 30.0
    flags: int = 1
    voltage: float = 24.0
    bus_current: float = 20.0


def extract(names, env):
    nodes = [node for node in TREE.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    assert {node.name for node in nodes} == set(names)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), env)


class WorkerHarness:
    def __init__(self):
        self.now = 10.0
        self.calls, self.logs = [], []
        self.stop = threading.Event()
        self.state = {"calibration": {"valid": False, "coefficients": [1, 2, 3, 4]}}
        self.ack_failure = None
        self.initial_enabled = False
        self.startup_flags = 1
        self.final_rpm = 0.0
        self.final_flags = None
        self.stop_stream_ok = True
        self.disable_ok = True
        self.disable_error = False
        self.reader_stop_error = False
        self.sampler_stop = False
        self.read_calls = 0
        self.raw_seen = None
        self.before_enable_delay = 0.0
        self.stop_after_read = None
        self.stop_on_final_read = False
        self.stream_live = False
        self.query_during_stream = []
        self.enabled_axes = set()
        self.suppress_enable_axis = None
        self.final_hold_verified = True
        self.fail_config_phase = None
        self.fail_config_axis = 2
        self.fail_config_readback = False
        self.bad_status = None
        self.bad_status_after_restore_failure = False
        self.external_enabled_axis = None
        self.ignore_disable_axis = None
        self.stop_after_c5 = None
        self.mutate_other_field_before_restore = False
        self.inject_raw_speed_pulse = False
        self.raw_retained = []
        self.config_writes = []
        self.original_configs = {}
        for axis in range(1, 5):
            config = bytearray((axis, 0x42, 0x25, 0x18)) + bytearray((i + axis) % 256 for i in range(32)) + b"\x6b"
            config[5] = 1
            config[16:18] = (2000).to_bytes(2, "big")
            config[18:20] = (1000).to_bytes(2, "big")
            self.original_configs[axis] = bytes(config)
        self.configs = copy.deepcopy(self.original_configs)
        harness = self

        class Serial:
            def __enter__(self):
                harness.calls.append(("open",)); return self
            def __exit__(self, *args):
                harness.calls.append(("close",))

        class Reader:
            def __init__(self, connection, *, on_record=None):
                harness.calls.append(("reader_init",))
                self.on_record = on_record
            def start(self):
                harness.calls.append(("reader_start",))
            def check(self):
                pass
            def drain(self):
                harness.now += 0.025
                values = [types.SimpleNamespace(**vars(Telemetry(i, position=0.123 * i,
                    flags=harness.startup_flags if i in harness.enabled_axes else 0)),
                    sampled_at=harness.now - (4 - i) * 0.002) for i in range(1, 5)]
                if harness.inject_raw_speed_pulse and any(call[0] == "sampler" for call in harness.calls):
                    pulse = copy.deepcopy(values[1]); pulse.rpm = -101.7
                    pulse.sampled_at -= 0.020
                    values.insert(0, pulse)
                    harness.inject_raw_speed_pulse = False
                harness.raw_retained.extend(copy.deepcopy(values))
                if self.on_record is not None:
                    for value in values:
                        self.on_record(value)
                return values
            def stop(self):
                harness.calls.append(("reader_stop",))
                if harness.reader_stop_error:
                    raise RuntimeError("fake reader stop failure")

        class Assembler:
            def __init__(self, span):
                self.items = []; harness.calls.append(("assembler", span))
            def add(self, item):
                self.items.append(item)
                if len(self.items) != 4:
                    return None
                result, self.items = self.items, []
                return result

        module = types.SimpleNamespace(ObservationConfig=lambda **kwargs: types.SimpleNamespace(**kwargs),
            run_observation_sampler=self.sampler)
        safe_builtins = dict(vars(__import__("builtins")))
        def controlled_import(name, *args, **kwargs):
            if name == "y42_observation_sampler":
                return module
            raise AssertionError("unexpected production import: " + name)
        safe_builtins["__import__"] = controlled_import
        self.env = {"__builtins__": safe_builtins, "__name__": "observation_ast_isolation",
            "time": types.SimpleNamespace(perf_counter=lambda: self.now, sleep=self.sleep), "math": math, "copy": copy,
            "STOP_EVENT": self.stop, "SERIAL_LOCK": threading.RLock(), "MOTOR_IDS": (1, 2, 3, 4),
            "set_state": lambda **updates: self.state.update(copy.deepcopy(updates)),
            "append_process_log": lambda event, **fields: self.logs.append((event, copy.deepcopy(fields))),
            "direct": types.SimpleNamespace(Telemetry=Telemetry, open_serial=lambda *args: Serial(),
                FRAME_END=0x6B, frame=lambda *values: bytes(values),
                torque_frame=lambda address, direction, current, slope, rpm: bytes((address, 0xC5, direction))
                    + slope.to_bytes(2, "big") + current.to_bytes(2, "big") + b"\x00"
                    + round(rpm * 10).to_bytes(2, "big") + b"\x6b",
                transact=self.transact, command_accepted=lambda reply, address, command:
                    reply == bytes((address, command, 2, 0x6B))),
            "valid_reply": lambda reply, address, command, minimum: len(reply) >= minimum
                and reply[0] == address and reply[1] == command and reply[-1] == 0x6B,
            "current_port": lambda: "FAKE_NEVER_REAL", "current_baud": lambda: 921600,
            "read_firmware_status": lambda connection, address: {"firmware": "X"},
            "read_all_reliable": self.read_all, "telemetry_ready_for_calibration": lambda values: len(values) == 4,
            "calibration_feedback_error": self.feedback_error,
            "pull_directions": lambda: (0, 1, 0, 1), "enable_address": self.enable,
            "smooth_torque_vector_ramp": self.ramp,
            "start_timed_feedback_stream": self.start_stream,
            "stop_timed_feedback_stream": self.stop_stream,
            "AsyncTimedFeedbackReader": Reader, "FourMotorFeedbackAssembler": Assembler,
            "TIMED_FEEDBACK_INFO_CODE": 0x43,
            "send_fast": self.send_fast,
            "four_motor_torque_frame": lambda currents, **kwargs: ("vector", tuple(currents), kwargs),
            "command_frame": lambda address, action, data: (action, address),
            "update_telemetry": lambda values: self.calls.append(("telemetry", copy.deepcopy(values))),
            "disable_all_with_reconnect": self.disable,
        }
        extract({"observation_sampling_worker", "ObservationRawRecordGuard"}, self.env)

    def sleep(self, seconds):
        self.now += seconds

    def transact(self, connection, frame, **kwargs):
        if self.stream_live:
            self.query_during_stream.append("transact")
        address, command = frame[:2]
        self.calls.append(("transaction", bytes(frame)))
        if command == 0x42:
            return self.configs[address]
        if command == 0x3A:
            if self.external_enabled_axis is not None and not self.config_writes:
                self.enabled_axes.add(self.external_enabled_axis)
            status = int(address in self.enabled_axes)
            if self.bad_status is not None:
                status = self.bad_status
            if self.bad_status_after_restore_failure and self.fail_config_phase == "restore" and any(
                    row["phase"] == "restore" and row["address"] == self.fail_config_axis for row in self.config_writes):
                return b""
            return bytes((address, command, status, 0x6B))
        if command in (0x48, 0x45):
            old = self.configs[address]
            new = bytearray(old)
            if command == 0x48:
                new[4:36] = frame[4:36]
                phase = "entry" if int.from_bytes(new[16:18], "big") == 20 else "restore"
            else:
                new[16:18] = frame[4:6]
                phase = "raise"
            row = {"address": address, "phase": phase, "tx": bytes(frame), "before": old,
                   "after": bytes(new), "enabled": set(self.enabled_axes)}
            self.config_writes.append(row)
            self.calls.append(("config_write", address, phase))
            self.configs[address] = bytes(new)
            if phase == self.fail_config_phase and address == self.fail_config_axis:
                if self.fail_config_readback:
                    altered = bytearray(new); altered[16:18] = (19).to_bytes(2, "big")
                    self.configs[address] = bytes(altered)
                else:
                    return b""  # Applied, but ACK lost: outcome is unknown.
            return bytes((address, command, 2, 0x6B))
        if command == 0xF3:
            self.calls.append(("enable", address, copy.deepcopy(self.configs)))
            if address != self.suppress_enable_axis:
                self.enabled_axes.add(address)
            return bytes((address, command, 2, 0x6B))
        if command == 0xC5:
            current = int.from_bytes(frame[5:7], "big")
            self.calls.append(("preset", address, current, set(self.enabled_axes)))
            if address == self.stop_after_c5:
                self.stop.set()
            return bytes((address, command, 0xE2 if address == self.ack_failure else 2, 0x6B))
        raise AssertionError("unexpected fake transaction: " + frame.hex())

    def enable(self, connection, address):
        # Production enable_address calls status_query/transact and is not a
        # write-only command adapter. Its reply can collide with live 0x43.
        if self.stream_live:
            self.query_during_stream.append("enable_address")
        self.calls.append(("enable", address)); self.enabled_axes.add(address)
        return True, b"", b""

    def send_fast(self, connection, frame):
        self.calls.append(("write", frame))
        if isinstance(frame, bytes) and len(frame) >= 4 and frame[1] == 0xF3 and frame[3] == 1:
            self.enabled_axes.add(frame[0])
            self.calls.append(("enable", frame[0]))
        elif isinstance(frame, tuple) and frame[0] == "enable":
            if frame[1] != self.suppress_enable_axis:
                self.enabled_axes.add(frame[1])
            self.calls.append(("enable", frame[1]))
        elif isinstance(frame, tuple) and frame[0] == "disable" and frame[1] != self.ignore_disable_axis:
            self.enabled_axes.discard(frame[1])

    def ramp(self, connection, start, end, **kwargs):
        self.calls.append(("ramp", tuple(start), tuple(end), self.stream_live))
        if self.stop.is_set():
            raise RuntimeError("用户停止观察")
        self.now += 0.4
        return list(end)

    def read_all(self, connection, **kwargs):
        if self.stream_live:
            self.query_during_stream.append("read_all_reliable")
        self.read_calls += 1
        self.now += 0.05
        if self.read_calls == 1:
            self.now += self.before_enable_delay
        final_read = any(call[0] == "sampler" for call in self.calls)
        if final_read and self.mutate_other_field_before_restore:
            changed = bytearray(self.configs[3]); changed[22] ^= 1
            self.configs[3] = bytes(changed)
            self.mutate_other_field_before_restore = False
        rpm = self.final_rpm if final_read else 0
        if self.read_calls == self.stop_after_read or final_read and self.stop_on_final_read:
            self.stop.set()
        return [Telemetry(i, flags=(int(self.initial_enabled) if self.read_calls == 1 else
            self.final_flags if final_read and self.final_flags is not None else int(i in self.enabled_axes)), rpm=rpm)
            for i in range(1, 5)]

    def feedback_error(self, values, require_enabled=True, **kwargs):
        if require_enabled and any(t.flags is None or not t.flags & 1 for t in values):
            return "实时反馈失能"
        if any(t.flags is None or t.flags & 8 for t in values):
            return "驱动保护"
        return None

    def sampler(self, vectors, io, clock, stopped, **kwargs):
        self.calls.append(("sampler", kwargs["config"].total_budget_s))
        if self.sampler_stop:
            self.stop.set()
            io.emergency_stop("fake STOP")
            return {"status": "safety_stopped", "reason": "fake STOP", "samples": []}
        self.raw_seen = io.read_snapshot(deadline=clock.now() + 0.25)
        io.send_torque(vectors[0], max_rpm=8.0, deadline=clock.now() + 0.25)
        io.emit("fake_sampler_data", snapshot=self.raw_seen)
        return {"status": "observed", "samples": [{}] * 6,
            "calibration_published": False, "final_hold_verified": self.final_hold_verified}

    def start_stream(self, *args):
        self.stream_live = True
        self.calls.append(("stream_start",))

    def stop_stream(self, connection):
        self.calls.append(("stream_stop",))
        if self.stop_stream_ok:
            self.stream_live = False
        return self.stop_stream_ok

    def disable(self, *args, **kwargs):
        self.calls.append(("disable_confirm",))
        if self.disable_error:
            raise RuntimeError("fake reconnect raised")
        if self.disable_ok:
            self.enabled_axes = {axis for axis in self.enabled_axes if axis == self.ignore_disable_axis}
        return {"ok": self.disable_ok}

    def run(self):
        return self.env["observation_sampling_worker"]()


class ObservationWorkerTests(unittest.TestCase):
    def test_all_four_driver_caps_verified_before_enable_then_immediate_twenty_ma(self):
        h = WorkerHarness(); result = h.run()
        first_enable = next(i for i, call in enumerate(h.calls) if call[0] == "enable")
        self.assertEqual([call[1] for call in h.calls[:first_enable] if call[0] == "config_write"], [1, 2, 3, 4])
        self.assertFalse(any(call[0] == "preset" for call in h.calls[:first_enable]))
        for call in (call for call in h.calls if call[0] == "enable"):
            for config in call[2].values():
                self.assertEqual(config[16:20], b"\x00\x14\x00\x08")
        actions = [(call[0], call[1]) for call in h.calls if call[0] in ("enable", "preset")]
        self.assertEqual(actions, [(action, axis) for axis in range(1, 5) for action in ("enable", "preset")])
        self.assertEqual(result["status"], "observed")

    def test_failed_enabled_c5_never_raises_cap_or_enters_sampling(self):
        h = WorkerHarness(); h.ack_failure = 3; result = h.run()
        self.assertEqual([call[1] for call in h.calls if call[0] == "enable"], [1, 2, 3])
        self.assertFalse(any(row["phase"] == "raise" for row in h.config_writes))
        self.assertFalse(any(call[0] == "sampler" for call in h.calls))
        self.assertIn(("disable_confirm",), h.calls)
        self.assertNotEqual(result["status"], "observed")

    def test_already_enabled_entry_is_not_taken_over(self):
        h = WorkerHarness(); h.initial_enabled = True; h.run()
        self.assertFalse(any(call[0] in ("preset", "enable", "sampler", "write", "disable_confirm") for call in h.calls))

    def test_raw_snapshot_keeps_actual_angles_and_capture_span(self):
        h = WorkerHarness(); h.run()
        batch = h.raw_seen
        self.assertEqual([t.position for t in batch["motors"]], [0.123 * i for i in range(1, 5)])
        self.assertAlmostEqual(batch["captured_end"] - batch["captured_start"], 0.006)
        self.assertEqual(batch["clock_source"], "host_receive_not_device")
        self.assertEqual(h.state["calibration"], {"valid": False, "coefficients": [1, 2, 3, 4]})

    def test_normal_cleanup_stops_reader_before_stream_then_closes(self):
        h = WorkerHarness(); h.run()
        labels = [call[0] for call in h.calls]
        self.assertLess(labels.index("reader_stop"), labels.index("stream_stop"))
        self.assertLess(labels.index("stream_stop"), labels.index("close"))
        self.assertEqual(h.logs[-1][0], "torque_observation_end")
        self.assertFalse(h.logs[-1][1]["calibration_published"])

    def test_stop_emits_protective_commands_and_confirms_disable(self):
        h = WorkerHarness(); h.sampler_stop = True; result = h.run()
        writes = [call[1] for call in h.calls if call[0] == "write" and isinstance(call[1], tuple)
                  and call[1][0] in ("stop", "disable")]
        protective = [(action, axis) for axis in range(1, 5) for action in ("stop", "disable")]
        self.assertEqual(writes[:8], protective)
        self.assertTrue(all(frame in protective for frame in writes))
        self.assertIn(("disable_confirm",), h.calls)
        self.assertFalse(h.state["control_unlocked"])
        self.assertEqual(result["status"], "safety_stopped")

    def test_stream_cleanup_failure_cannot_archive_observed_success(self):
        h = WorkerHarness(); h.stop_stream_ok = False; result = h.run()
        self.assertIn(("disable_confirm",), h.calls)
        self.assertNotIn(result["status"], ("observed", "incomplete"))
        self.assertNotIn(h.logs[-1][1]["status"], ("observed", "incomplete"))

    def test_reader_cleanup_failure_also_confirms_disable(self):
        h = WorkerHarness(); h.reader_stop_error = True; result = h.run()
        self.assertIn(("close",), h.calls)
        self.assertIn(("disable_confirm",), h.calls)
        self.assertNotIn(result["status"], ("observed", "incomplete"))

    def test_unconfirmed_disable_demands_hardware_stop(self):
        h = WorkerHarness(); h.ack_failure = 1; h.disable_ok = False; h.ignore_disable_axis = 1; h.run()
        self.assertRegex(h.state["message"], "切断|硬件急停")

    def test_moving_final_hold_is_not_declared_stable_success(self):
        h = WorkerHarness(); h.final_rpm = 4.0; result = h.run()
        self.assertIn(("disable_confirm",), h.calls)
        self.assertNotIn(result["status"], ("observed", "incomplete"))

    def test_startup_protected_status_prevents_feedback_observation(self):
        h = WorkerHarness(); h.startup_flags = 9; h.run()
        self.assertFalse(any(call[0] == "sampler" for call in h.calls))

    def test_live_0x43_stream_is_not_mixed_with_synchronous_telemetry_queries(self):
        h = WorkerHarness(); h.run()
        self.assertEqual(h.query_during_stream, [],
            "direct.transact resets RX and cannot treat interleaved 0x43 bytes as a normal reply")

    def test_unconfirmed_fourth_enable_never_ramps_up_or_samples(self):
        h = WorkerHarness(); h.suppress_enable_axis = 4; result = h.run()
        self.assertFalse(any(call[0] in ("ramp", "sampler") for call in h.calls))
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn(("disable_confirm",), h.calls)
        self.assertLess(h.now - 10.0, 1.0)

    def test_missing_verified_final_window_cannot_publish_success(self):
        h = WorkerHarness(); h.final_hold_verified = False; result = h.run()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertFalse(result["final_hold_verified"])
        self.assertIn(("disable_confirm",), h.calls)

    def test_final_protection_cannot_archive_observed_success(self):
        h = WorkerHarness(); h.final_flags = 9; result = h.run()
        self.assertIn(("disable_confirm",), h.calls)
        self.assertNotIn(result["status"], ("observed", "incomplete"))

    def test_stop_arriving_on_final_read_is_archived_as_stopped(self):
        h = WorkerHarness(); h.stop_on_final_read = True; result = h.run()
        self.assertIn(("disable_confirm",), h.calls)
        self.assertNotIn(result["status"], ("observed", "incomplete"))
        self.assertFalse(h.state["control_unlocked"])

    def test_unexpected_reconnect_exception_still_logs_terminal_failure(self):
        h = WorkerHarness(); h.ack_failure = 1; h.disable_error = True; h.ignore_disable_axis = 1; result = h.run()
        self.assertEqual(h.logs[-1][0], "torque_observation_end")
        self.assertNotIn(result["status"], ("observed", "incomplete"))
        self.assertRegex(h.state["message"], "切断|硬件急停")

    def test_slow_read_only_start_does_not_enable(self):
        h = WorkerHarness(); h.before_enable_delay = 5.1; h.run()
        self.assertFalse(any(call[0] in ("preset", "enable") for call in h.calls))

    def test_volatile_configs_preserve_every_non_limit_byte_and_restore_exactly(self):
        h = WorkerHarness(); result = h.run()
        for row in h.config_writes:
            self.assertEqual(row["tx"][3], 0, "all writes must be volatile store=00")
            self.assertEqual(row["before"][:16] + row["before"][20:],
                             row["after"][:16] + row["after"][20:])
            if row["phase"] == "raise":
                self.assertEqual(row["tx"][1:4], b"\x45\x66\x00")
                self.assertEqual(row["after"][16:20], b"\x00\xdc\x00\x08")
            else:
                self.assertEqual(row["tx"][1:4], b"\x48\xd1\x00")
                self.assertEqual(row["enabled"], set(), "full config may only be written disabled")
        self.assertEqual(h.configs, h.original_configs)
        self.assertEqual(result["final_state"], "disabled_original_configuration_restored")
        self.assertTrue(result["final_disabled_confirmed"])
        self.assertTrue(result["sampling_hold_verified"])
        self.assertFalse(result["final_hold_verified"])
        self.assertFalse(h.state["control_unlocked"])

    def test_cap_raise_occurs_only_after_all_four_enabled_c5_acknowledgements(self):
        h = WorkerHarness(); h.run()
        raised_at = next(i for i, call in enumerate(h.calls) if call[0] == "config_write" and call[2] == "raise")
        self.assertEqual([call[1] for call in h.calls[:raised_at] if call[0] == "preset"], [1, 2, 3, 4])

    def test_entry_envelope_lost_ack_never_enables_even_if_write_applied(self):
        h = WorkerHarness(); h.fail_config_phase = "entry"; result = h.run()
        self.assertFalse(any(call[0] == "enable" for call in h.calls))
        self.assertEqual(result["status"], "safety_stopped")
        self.assertTrue(result["final_disabled_confirmed"])
        self.assertEqual(h.configs, h.original_configs)

    def test_entry_envelope_readback_mismatch_never_enables(self):
        h = WorkerHarness(); h.fail_config_phase = "entry"; h.fail_config_readback = True; result = h.run()
        self.assertFalse(any(call[0] == "enable" for call in h.calls))
        self.assertNotEqual(result["status"], "observed")

    def test_open_loop_or_lower_original_limits_are_not_silently_increased(self):
        for offset, replacement in ((5, b"\x00"), (16, b"\x00\xc8"), (18, b"\x00\x04")):
            h = WorkerHarness()
            bad = bytearray(h.configs[2]); bad[offset:offset + len(replacement)] = replacement
            h.configs[2] = bytes(bad)
            h.run()
            self.assertEqual(h.config_writes, [])
            self.assertFalse(any(call[0] in ("enable", "disable_confirm") for call in h.calls))

    def test_wrong_complete_configuration_header_is_rejected(self):
        h = WorkerHarness(); bad = bytearray(h.configs[1]); bad[3] = 0
        h.configs[1] = bytes(bad); h.run()
        self.assertEqual(h.config_writes, [])

    def test_restore_ack_loss_stops_remaining_high_limit_writes_and_records_unknown(self):
        h = WorkerHarness(); h.fail_config_phase = "restore"; result = h.run()
        restore = result["configuration_restore"]
        self.assertEqual(result["status"], "cleanup_failed")
        self.assertEqual(restore["restored"], [1])
        self.assertEqual(restore["unknown"], [2])
        self.assertEqual(restore["not_attempted"], [3, 4])
        self.assertEqual(restore["unconfirmed_limit_axes"], [2])
        self.assertEqual([row["address"] for row in h.config_writes if row["phase"] == "restore"], [1, 2])
        self.assertEqual(h.configs[3][16:20], b"\x00\xdc\x00\x08")
        self.assertFalse(h.state["control_unlocked"])

    def test_restore_readback_mismatch_stops_all_later_restore_writes(self):
        h = WorkerHarness(); h.fail_config_phase = "restore"; h.fail_config_readback = True; result = h.run()
        self.assertEqual(result["status"], "cleanup_failed")
        self.assertEqual(result["configuration_restore"]["unknown"], [2])
        self.assertEqual([row["address"] for row in h.config_writes if row["phase"] == "restore"], [1, 2])

    def test_missing_recovery_flags_cannot_reuse_earlier_disabled_confirmation(self):
        h = WorkerHarness(); h.fail_config_phase = "restore"; h.bad_status_after_restore_failure = True
        result = h.run()
        self.assertFalse(result["final_disabled_confirmed"])
        self.assertRegex(h.state["message"], "切断|硬件急停")

    def test_error_codes_are_not_misread_as_disabled_status(self):
        for error_code in (0xE2, 0xEE):
            h = WorkerHarness(); h.bad_status = error_code; h.run()
            self.assertEqual(h.config_writes, [])
            self.assertFalse(any(call[0] in ("enable", "disable_confirm") for call in h.calls))

    def test_restore_refuses_to_overwrite_changed_non_limit_configuration(self):
        h = WorkerHarness(); h.mutate_other_field_before_restore = True; result = h.run()
        self.assertEqual(result["status"], "cleanup_failed")
        self.assertEqual([row["address"] for row in h.config_writes if row["phase"] == "restore"], [1, 2])
        self.assertEqual(h.configs[3][22], h.original_configs[3][22] ^ 1)

    def test_enabled_axis_during_cleanup_prevents_all_original_limit_restores(self):
        h = WorkerHarness(); h.ignore_disable_axis = 3; result = h.run()
        self.assertFalse(result["final_disabled_confirmed"])
        self.assertEqual(result["status"], "cleanup_failed")
        self.assertFalse(any(row["phase"] == "restore" for row in h.config_writes))

    def test_stop_after_last_c5_never_proceeds_to_higher_current_cap(self):
        h = WorkerHarness(); h.stop_after_c5 = 4; result = h.run()
        self.assertFalse(any(row["phase"] == "raise" for row in h.config_writes))
        self.assertEqual(result["status"], "safety_stopped")
        self.assertEqual(h.configs, h.original_configs)

    def test_soft_return_to_twenty_runs_before_reader_shutdown_without_sync_rx(self):
        h = WorkerHarness(); h.run()
        ramps = [(i, call) for i, call in enumerate(h.calls) if call[0] == "ramp"]
        self.assertEqual(ramps[-1][1][1:3], ((150, 150, 150, 150), (20, 20, 20, 20)))
        self.assertTrue(ramps[-1][1][3])
        self.assertLess(ramps[-1][0], next(i for i, call in enumerate(h.calls) if call[0] == "reader_stop"))
        self.assertEqual(h.query_during_stream, [])

    def test_race_to_enabled_before_envelope_gate_preserves_existing_hold(self):
        h = WorkerHarness(); h.external_enabled_axis = 4; h.run()
        self.assertEqual(h.config_writes, [])
        self.assertFalse(any(call[0] in ("enable", "write", "disable_confirm") for call in h.calls))

    def test_original_configuration_and_every_configuration_tx_rx_are_recorded(self):
        h = WorkerHarness(); result = h.run()
        self.assertEqual(result["original_configurations_hex"], {str(k): v.hex() for k, v in h.original_configs.items()})
        records = [fields for event, fields in h.logs if event == "torque_observation_configuration_io"]
        self.assertTrue(records)
        self.assertTrue(all(record["tx_hex"] and "rx_hex" in record for record in records))
        self.assertEqual(h.logs[-1][1]["configuration_restore"]["restored"], [1, 2, 3, 4])

    def test_raw_pulse_archives_actual_cause_even_if_latest_feedback_is_normal(self):
        h = WorkerHarness(); h.inject_raw_speed_pulse = True; result = h.run()
        self.assertEqual(result["status"], "safety_stopped")
        self.assertIn("101.7", result["reason"])
        self.assertNotIn("用户停止", result["reason"])
        self.assertEqual(result["raw_guard_fault"]["record"]["rpm"], -101.7)
        self.assertTrue(any(item.rpm == -101.7 for item in h.raw_retained))
        self.assertEqual(h.raw_retained[-1].rpm, 0.0)
        self.assertEqual(h.logs[-1][1]["raw_guard_fault"]["record"]["rpm"], -101.7)
        self.assertIn(("disable_confirm",), h.calls)


class ObservationApiTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.locked, self.confirmed, self.busy = False, True, False
        def require_control():
            if self.locked: raise RuntimeError("控制锁定")
        def require_confirmation(data):
            if not self.confirmed: raise RuntimeError("没有安全确认")
        self.worker = object()
        self.env = {"require_control_unlocked": require_control, "require_confirmed": require_confirmation,
            "active_auto": lambda: self.busy, "calibration_data": lambda: {"valid": False},
            "observation_sampling_worker": self.worker, "start_thread": lambda worker: self.calls.append(worker)}
        extract({"_start_auto_impl"}, self.env)

    def test_observation_api_remains_disabled_even_with_confirmation_or_override(self):
        expected = "低力力矩观察暂已停用：实测转速超过限制，待核实固件限速/制动；原始数据已保存"
        for extra in ({}, {"confirmed": True}, {"confirmed": True, "override": True, "force": True}):
            with self.assertRaises(RuntimeError) as failure:
                self.env["_start_auto_impl"]({"action": "observe-torque", **extra})
            self.assertEqual(str(failure.exception), expected)
        self.assertEqual(self.calls, [])

    def test_locked_unconfirmed_and_busy_requests_do_not_start_worker(self):
        for name in ("locked", "confirmed", "busy"):
            self.locked, self.confirmed, self.busy = False, True, False
            setattr(self, name, name != "confirmed")
            with self.assertRaises(RuntimeError):
                self.env["_start_auto_impl"]({"action": "observe-torque"})
        self.assertEqual(self.calls, [])


if __name__ == "__main__":
    unittest.main()
