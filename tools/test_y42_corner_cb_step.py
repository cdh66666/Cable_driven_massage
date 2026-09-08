"""Offline tests for the bounded CB M1 short-step phase."""
from __future__ import annotations

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


class FakeCBSession:
    corner_probe_axis_id = 1
    # This is deliberately different from the old static base constant.  The
    # phase must consume these runtime values to choose the positive target.
    pull_directions = (0, 1, 1, 0)
    encoder_positive_directions = (0, 0, 1, 1)

    def __init__(self, clock, *, mode="follow"):
        self.clock = clock
        self.mode = mode
        self.positions = {1: 5.0, 2: 10.0, 3: -5.0, 4: 2.0}
        self.pending_target = None
        self.pending_current = 20
        self.jump_pending = True
        self.current_commands = []
        self.cb_commands = []
        self.feedback_count = 0
        self.events = []

    def send_c5(self, frame):
        frame = bytes(frame)
        if len(frame) != 11 or frame[1] != 0xC5 or frame[-1] != 0x6B:
            raise AssertionError("not a bounded C5 frame")
        if frame[0] not in (2, 3, 4) or int.from_bytes(frame[5:7], "big") != 20:
            raise AssertionError("active M1 or non-20mA C5 was sent")
        self.current_commands.append(frame)
        return {"accepted": True, "tx_hex": frame.hex()}

    def send_cb(self, target_position_deg, current_ma):
        target = float(target_position_deg)
        self.cb_commands.append((target, int(current_ma), self.clock()))
        self.pending_target = target
        self.pending_current = int(current_ma)
        return {"accepted": True, "tx_hex": "01cb-fake"}

    def read_feedback_43(self):
        self.feedback_count += 1
        rpm = 0.0
        if self.pending_target is not None:
            if self.mode == "follow":
                self.positions[1] = self.pending_target
            elif self.mode == "jump_once":
                if self.jump_pending:
                    self.positions[1] += 0.2
                    self.jump_pending = False
                else:
                    self.positions[1] = self.pending_target
            elif self.mode == "reverse":
                self.positions[1] -= 0.1
            elif self.mode == "runaway":
                self.positions[1] = self.pending_target
                rpm = 1.3
        sampled_at = self.clock() - (0.25 if self.mode == "stale" else 0.0)
        rows = []
        for address in probe.IDS:
            current = self.pending_current if address == 1 and self.pending_target is not None else 20
            position = self.positions[address]
            rows.append(
                {
                    "id": address,
                    "position": position,
                    "rpm": rpm if address == 1 else 0.0,
                    "phase_current": current,
                    "temperature": 25,
                    "flags": 3,
                    "sampled_at": sampled_at,
                    "raw_packet_hex": raw_43(
                        address,
                        position,
                        rpm if address == 1 else 0.0,
                        current,
                    ).hex(),
                }
            )
        return rows

    def emit(self, event, **fields):
        self.events.append((event, fields))


def run(session):
    return probe.run_corner_cb_step_phase(
        session,
        clock=session.clock,
        sleep=session.clock.sleep,
    )


class CornerCBStepTests(unittest.TestCase):
    def test_following_position_steps_are_bounded_and_never_claim_corner(self):
        session = FakeCBSession(FakeClock(), mode="follow")
        result = run(session)
        self.assertTrue(result["success"], result)
        self.assertTrue(result["phase_passed"], result)
        self.assertTrue(result["movement_observed"])
        self.assertFalse(result["candidate_stop"])
        self.assertFalse(result["calibration_published"])
        self.assertTrue(result["requires_full_four_corner_calibration"])
        self.assertTrue(result["cleanup_owned_by_caller"])
        self.assertLessEqual(result["max_lead_deg"], 0.2 + 1e-9)
        self.assertLessEqual(result["max_target_travel_deg"], 2.0 + 1e-9)
        self.assertEqual([frame[0] for frame in session.current_commands], [2, 3, 4])
        self.assertTrue(session.cb_commands)
        self.assertTrue(all(current == 40 for _, current, _ in session.cb_commands))
        self.assertGreaterEqual(len(result["raw_feedback"]), 4)
        self.assertEqual(result["encoder_step_sign"], 1)

    def test_no_response_escalates_to_60mA_but_is_not_a_stop(self):
        session = FakeCBSession(FakeClock(), mode="stationary")
        result = run(session)
        self.assertTrue(result["success"], result)
        self.assertFalse(result["phase_passed"])
        self.assertFalse(result["movement_observed"])
        self.assertEqual(result["reason"], "insufficient_motion")
        self.assertFalse(result["candidate_stop"])
        self.assertIn(40, [current for _, current, _ in session.cb_commands])
        self.assertIn(60, [current for _, current, _ in session.cb_commands])
        self.assertFalse(any(frame[0] == 1 for frame in session.current_commands))
        targets = [target for target, _, _ in session.cb_commands]
        self.assertLessEqual(max(abs(target - 5.0) for target in targets), 0.2 + 1e-9)

    def test_one_quantised_feedback_jump_keeps_targets_at_one_count(self):
        session = FakeCBSession(FakeClock(), mode="jump_once")
        result = run(session)
        self.assertTrue(result["success"], result)
        targets = [target for target, _, _ in session.cb_commands]
        self.assertTrue(targets)
        self.assertTrue(
            all(abs(next_target - target) <= 0.1 + 1e-9
                for target, next_target in zip(targets, targets[1:]))
        )
        self.assertLessEqual(result["max_lead_deg"], 0.2 + 1e-9)

    def test_speed_overrun_aborts_after_first_active_feedback(self):
        session = FakeCBSession(FakeClock(), mode="runaway")
        result = run(session)
        self.assertFalse(result["success"])
        self.assertFalse(result["candidate_stop"])
        self.assertIn("转速超过1.2RPM", result["failure_reason"])
        self.assertEqual(len(session.cb_commands), 1)
        self.assertTrue(result["raw_feedback"])

    def test_reverse_feedback_is_rejected(self):
        session = FakeCBSession(FakeClock(), mode="reverse")
        result = run(session)
        self.assertFalse(result["success"])
        self.assertIn("方向与运行时拉绳方向相反", result["failure_reason"])
        self.assertFalse(result["candidate_stop"])

    def test_stale_feedback_is_rejected_and_archived(self):
        session = FakeCBSession(FakeClock(), mode="stale")
        result = run(session)
        self.assertFalse(result["success"])
        self.assertIn("反馈陈旧", result["failure_reason"])
        self.assertEqual(len(result["raw_feedback"]), 4)

    def test_missing_runtime_encoder_direction_never_falls_back_to_static(self):
        session = FakeCBSession(FakeClock())
        session.encoder_positive_directions = None
        with self.assertRaises(ValueError):
            probe.run_corner_cb_step_phase(session, clock=session.clock, sleep=session.clock.sleep)
        self.assertFalse(session.current_commands)
        self.assertFalse(session.cb_commands)


if __name__ == "__main__":
    unittest.main(verbosity=2)
