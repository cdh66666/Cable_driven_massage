"""Focused offline tests for the synchronous master-polled transport contract."""
import unittest

import y42_position_microprobe_io as io
from test_y42_position_microprobe_io import FakeBus


class PolledTransportContractTests(unittest.TestCase):
    def test_each_motion_tick_is_four_cb_acks_then_two_round_robin_43_queries(self):
        bus = FakeBus()
        out = bus.run()

        self.assertTrue(out["success"], out["reason"])
        motion_ticks = len(bus.cadence_events)
        self.assertGreater(motion_ticks, 100)
        self.assertEqual(len(bus.motion_cb_packets), 4 * motion_ticks)
        self.assertFalse(any(packet[1] == 0xAA for packet in bus.calls))
        warmup_queries = sum(packet[1] == 0x43 for packet in
                             bus.calls[:bus.motion_cb_indexes[0]])
        for tick in range(motion_ticks):
            cb_indexes = bus.motion_cb_indexes[tick * 4:(tick + 1) * 4]
            self.assertEqual([bus.calls[index][0] for index in cb_indexes], [1, 2, 3, 4])
            self.assertEqual([bus.calls[index][1] for index in cb_indexes], [0xCB] * 4)
            self.assertEqual([bus.calls[index][1] for index in
                              range(cb_indexes[-1] + 1, cb_indexes[-1] + 3)],
                             [0x43, 0x43])
        self.assertEqual(len(bus.query_packets), warmup_queries + 2 * motion_ticks)
        self.assertLessEqual(max(bus.query_counts.values()) - min(bus.query_counts.values()), 1)
        self.assertEqual(out["cadence"]["target_hz"], io.COMMAND_HZ)
        self.assertEqual(out["stream_runs"][-1]["mode"], "master_polled")
        self.assertTrue(all(event["transport"] == "addressed_cb"
                            and len(event["axes"]) == 4
                            for event in bus.cadence_events))

    def test_addressed_cb_requires_exact_axis_ack_before_any_feedback_poll(self):
        bus = FakeBus()
        bus.reject_motion_axis = 1
        out = bus.run()

        self.assertFalse(out["success"])
        self.assertEqual(bus.aggregate_count, 0)
        self.assertEqual(len(bus.motion_cb_packets), 1)
        first_motion_cb = bus.motion_cb_indexes[0]
        self.assertEqual(len(bus.query_packets),
                         sum(packet[1] == 0x43 for packet in bus.calls[:first_motion_cb]))
        self.assertFalse(any(packet[1] == 0x43 for packet in bus.calls[first_motion_cb + 1:]))
        self.assertIn("位置指令确认无效", out["reason"])
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])

    def test_partial_axis_ack_failure_does_not_send_the_next_axis(self):
        bus = FakeBus()
        bus.reject_motion_axis = 2
        out = bus.run()

        self.assertFalse(out["success"])
        self.assertEqual([packet[0] for packet in bus.motion_cb_packets], [1, 2])
        self.assertEqual(bus.aggregate_count, 0)
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])

    def test_four_axis_transport_at_one_tick_stops_before_next_group(self):
        bus = FakeBus()
        # Four 9.5 ms addressed transactions plus the required receive quiet
        # windows reach one 40 ms command period while each write stays within
        # its individual 10 ms deadline.
        bus.motion_axis_delay = 0.0095
        out = bus.run()

        self.assertFalse(out["success"])
        self.assertEqual(len(bus.motion_cb_packets), 4)
        self.assertLessEqual(bus.aggregate_count, 1)
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])

    def test_bad_polled_43_frame_stops_before_the_next_aa(self):
        bus = FakeBus()

        def corrupt_after_motion(fake, records):
            if fake.aggregate_count >= 2:
                frame = bytearray(FakeBus.feedback_frame(records[0]))
                frame[0] = 4 if frame[0] != 4 else 3
                fake.fault_command_count = fake.aggregate_count
                return bytes(frame)
            return records

        bus.record_hook = corrupt_after_motion
        out = bus.run()

        self.assertFalse(out["success"])
        self.assertEqual(bus.aggregate_count, bus.fault_command_count)
        self.assertIn("完整0x43反馈", out["reason"])
        self.assertTrue(out["configuration_restored"])
        self.assertTrue(out["final_disabled_confirmed"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
