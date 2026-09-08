"""Offline fake-session tests for the bounded corner-stop candidate adapter."""
from __future__ import annotations

import copy
import unittest

import y42_corner_stop_probe as probe


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, float(seconds))


def raw_43(address, position, rpm=0.0, current=20, temperature=25, flags=3):
    packet = bytearray(37)
    packet[:4] = bytes((address, 0x43, 0x25, 0x0C))
    packet[8:10] = int(current).to_bytes(2, "big")
    packet[19] = int(float(rpm) < 0)
    packet[20:22] = round(abs(float(rpm)) * 10).to_bytes(2, "big")
    packet[22] = int(float(position) < 0)
    packet[23:27] = round(abs(float(position)) * 10).to_bytes(4, "big")
    packet[33] = int(temperature)
    packet[35] = int(flags)
    packet[36] = 0x6B
    return bytes(packet)


class FakeSession:
    corner_probe_axis_id = 1
    pull_directions = (0, 0, 1, 1)
    corner_probe_encoder_direction_sign = 1

    def __init__(self, clock, *, motion=True, fault=None):
        self.clock = clock
        self.motion = motion
        self.fault = fault
        self.positions = {1: 0.0, 2: 10.0, 3: -5.0, 4: 2.0}
        self.start_positions = copy.deepcopy(self.positions)
        self.feedback_count = 0
        self.commands = []
        self.events = []
        self.snapshot = {"configuration": "original"}
        self.stop_called = False
        self.restore_called = False
        self.disabled = False

    def capture_configuration(self):
        return copy.deepcopy(self.snapshot)

    def send_c5(self, frame):
        frame = bytes(frame)
        self.commands.append(frame)
        self.assert_single_axis_c5(frame)
        self.current_level = int.from_bytes(frame[5:7], "big")
        return {"accepted": True, "frame_hex": frame.hex()}

    @staticmethod
    def assert_single_axis_c5(frame):
        if len(frame) != 11 or frame[1] != 0xC5 or frame[-1] != 0x6B:
            raise AssertionError("expected one addressed C5 frame")

    def read_feedback_43(self):
        self.feedback_count += 1
        if self.motion and self.feedback_count <= 8:
            self.positions[1] += 0.20
        rows = []
        for address in probe.IDS:
            rpm = 0.0
            current = 20
            sampled_at = self.clock()
            flags = 8 if self.fault == "flags" and address == 1 else 3
            if self.fault == "current" and address == 1 and self.current_level == 60:
                current = 91
            elif self.fault == "rpm" and address == 1 and self.current_level == 40:
                rpm = 1.3
            elif self.fault == "stale" and address == 1:
                sampled_at -= 0.25
            elif self.fault == "travel" and address == 1 and self.feedback_count == 2:
                self.positions[1] = 31.0
            rows.append({
                "id": address,
                "position": self.positions[address],
                "rpm": rpm,
                "phase_current": current,
                "temperature": 25,
                "flags": flags,
                "sampled_at": sampled_at,
                "raw_packet_hex": raw_43(address, self.positions[address], rpm, current, flags=flags).hex(),
            })
        return rows

    def stop_disable(self):
        self.stop_called = True
        self.disabled = True
        return {"ok": True, "stopped": True, "disabled": True}

    def restore_configuration(self, snapshot):
        self.restore_called = snapshot == self.snapshot
        return {"ok": self.restore_called, "restored": self.restore_called}

    def emit(self, event, **fields):
        self.events.append((event, fields))


def run(session):
    return probe.run_hardware_corner_stop_probe(
        session,
        clock=session.clock,
        sleep=session.clock.sleep,
    )


class CornerStopProbeTests(unittest.TestCase):
    def test_c5_frame_is_fixed_20_to_60_ma_and_one_rpm(self):
        frame = probe.c5_frame(1, 0, 20)
        self.assertEqual(len(frame), 11)
        self.assertEqual(frame.hex(), "01c50000c8001400000a6b")
        with self.assertRaises(ValueError):
            probe.c5_frame(1, 0, 80)
        with self.assertRaises(ValueError):
            probe.c5_frame(1, 0, 20, max_rpm=1.1)

    def test_motion_then_final_plateau_is_only_a_candidate(self):
        session = FakeSession(FakeClock(), motion=True)
        result = run(session)
        self.assertTrue(result["success"], result)
        self.assertTrue(result["movement_observed"])
        self.assertTrue(result["candidate_stop"])
        self.assertEqual(result["evidence"], "motion_then_final_level_low_rpm_position_plateau")
        self.assertFalse(result["calibration_published"])
        self.assertTrue(result["requires_external_mechanical_validation"])
        self.assertEqual(
            [packet[0] for packet in session.commands],
            [1, 2, 3, 4, 1, 1, 1],
        )
        self.assertEqual(
            [int.from_bytes(packet[5:7], "big") for packet in session.commands],
            [20, 20, 20, 20, 20, 40, 60],
        )
        self.assertEqual(len(session.commands), 7)
        self.assertTrue(session.stop_called)
        self.assertTrue(session.restore_called)
        self.assertTrue(result["final_disabled_confirmed"])
        self.assertTrue(result["configuration_restored"])
        self.assertGreaterEqual(len(result["raw_feedback"]), 20)
        self.assertEqual(result["commands"][0]["currents_ma"], [20, 20, 20, 20])
        self.assertEqual(result["commands"][4]["currents_ma"], [20, 20, 20, 20])
        self.assertEqual(result["commands"][5]["currents_ma"], [40, 20, 20, 20])
        self.assertEqual(result["commands"][6]["currents_ma"], [60, 20, 20, 20])

    def test_no_motion_is_safe_but_never_a_stop_candidate(self):
        session = FakeSession(FakeClock(), motion=False)
        result = run(session)
        self.assertTrue(result["success"], result)
        self.assertFalse(result["movement_observed"])
        self.assertFalse(result["candidate_stop"])
        self.assertEqual(result["stop_reason"], "insufficient_motion")
        self.assertTrue(result["final_disabled_confirmed"])
        self.assertTrue(result["configuration_restored"])

    def test_phase_keeps_cleanup_with_the_callers_existing_session(self):
        session = FakeSession(FakeClock(), motion=True)
        result = probe.run_corner_probe_phase(
            session,
            clock=session.clock,
            sleep=session.clock.sleep,
        )
        self.assertTrue(result["success"], result)
        self.assertTrue(result["candidate_stop"])
        self.assertTrue(result["cleanup_owned_by_caller"])
        self.assertIsNone(result["final_disabled_confirmed"])
        self.assertIsNone(result["configuration_restored"])
        self.assertFalse(session.stop_called)
        self.assertFalse(session.restore_called)

    def test_current_overrun_stops_before_claiming_candidate(self):
        session = FakeSession(FakeClock(), motion=True, fault="current")
        result = run(session)
        self.assertFalse(result["success"])
        self.assertFalse(result["candidate_stop"])
        self.assertIn("相电流超过当前指令+30mA", result["failure_reason"])
        self.assertTrue(result["final_disabled_confirmed"])
        self.assertTrue(result["configuration_restored"])
        self.assertTrue(result["raw_feedback"])

    def test_rpm_overrun_stops_immediately(self):
        session = FakeSession(FakeClock(), motion=True, fault="rpm")
        result = run(session)
        self.assertFalse(result["success"])
        self.assertFalse(result["candidate_stop"])
        self.assertIn("转速超过1.2RPM", result["failure_reason"])
        self.assertTrue(result["final_disabled_confirmed"])

    def test_stale_feedback_stops_and_archives_the_bad_raw_frame(self):
        session = FakeSession(FakeClock(), motion=True, fault="stale")
        result = run(session)
        self.assertFalse(result["success"])
        self.assertIn("反馈陈旧", result["failure_reason"])
        self.assertTrue(result["raw_feedback"])
        self.assertTrue(any(row.get("id") == 1 for row in result["raw_feedback"]))
        self.assertTrue(result["final_disabled_confirmed"])

    def test_bad_first_frame_keeps_all_four_raw_records(self):
        session = FakeSession(FakeClock(), motion=True, fault="flags")
        result = run(session)
        self.assertFalse(result["success"])
        self.assertIn("保护状态无效", result["failure_reason"])
        self.assertEqual(len(result["raw_feedback"]), 4)
        self.assertTrue(result["final_disabled_confirmed"])

    def test_cumulative_travel_bound_stops_without_false_stop(self):
        session = FakeSession(FakeClock(), motion=True, fault="travel")
        result = run(session)
        self.assertFalse(result["success"])
        self.assertFalse(result["candidate_stop"])
        self.assertIn("角位移超过30度", result["failure_reason"])
        self.assertTrue(result["final_disabled_confirmed"])

    def test_one_argument_loader_requires_verified_config_in_api(self):
        session = FakeSession(FakeClock(), motion=False)
        # Use a tiny object with no config to prove the adapter never guesses
        # an axis.
        class NoConfig:
            capture_configuration = session.capture_configuration
            send_c5 = session.send_c5
            read_feedback_43 = session.read_feedback_43
            stop_disable = session.stop_disable
            restore_configuration = session.restore_configuration

        with self.assertRaises(ValueError):
            probe.run_hardware_corner_stop_probe(NoConfig())


if __name__ == "__main__":
    unittest.main(verbosity=2)
