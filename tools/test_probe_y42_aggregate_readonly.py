"""Pure fake transport and temporary archive; never imports pyserial."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import probe_y42_aggregate_readonly as probe


class FakeTransport:
    def __init__(self):
        self.sent = []
        self.flags = 2
        self.old_reply = False
        self.mutation = None

    def transact(self, tx):
        probe.validate_readonly_frame(tx)
        self.sent.append(tx)
        axis = tx[0] if len(tx) == 3 else tx[4]
        raw = bytes((axis, 0x3A, self.flags, 0x6B))
        if len(tx) == 8 and tx[3] == 6 and not self.old_reply:
            raw = b""
        if self.mutation:
            raw = self.mutation(tx, raw, len(self.sent))
        return {"rx_hex": raw.hex(), "rx_bytes": len(raw), "stale_rx_hex": ""}


class ProbeTests(unittest.TestCase):
    def test_exact_query_whitelist_rejects_every_control_variant(self):
        for address in probe.IDS:
            for length in (8, 6):
                frame = probe.aggregate_status_query(address, length)
                self.assertEqual(len(frame), 8)
                self.assertEqual(int.from_bytes(frame[2:4], "big"), length)
                self.assertEqual(probe.validate_readonly_frame(frame), frame)
            for command in (0xF3, 0xC5, 0xFE, 0x45, 0x48, 0xFF, 0x36, 0x11):
                with self.assertRaises(ValueError):
                    probe.validate_readonly_frame(bytes((address, command, 0x6B)))
                malformed = bytearray(probe.aggregate_status_query(address, 8)); malformed[5] = command
                with self.assertRaises(ValueError):
                    probe.validate_readonly_frame(malformed)
        with self.assertRaises(ValueError):
            probe.validate_readonly_frame(probe.status_query(1) + probe.status_query(2))

    def test_all_twenty_four_queries_and_two_repeats_per_length_axis(self):
        io, events = FakeTransport(), []
        summary = probe.run_probe(io, events.append)
        self.assertEqual(summary["status"], "completed")
        self.assertEqual(len(io.sent), 24)
        self.assertEqual(summary["lengths"]["8"]["valid_status_replies"], 8)
        self.assertEqual(summary["lengths"]["6"]["valid_status_replies"], 0)
        for axis in probe.IDS:
            self.assertEqual(io.sent.count(probe.status_query(axis)), 2)
            for length in (6, 8):
                self.assertEqual(io.sent.count(probe.aggregate_status_query(axis, length)), 2)
        for event in events[:-1]:
            self.assertIn("started_unix_ns", event)
            self.assertIn("completed_monotonic_ns", event)
            self.assertIn("rx_hex", event)

    def test_bad_or_enabled_preflight_never_sends_aggregate(self):
        for flags in (1, 0xE2, 0xEE):
            io = FakeTransport(); io.flags = flags
            result = probe.run_probe(io, lambda event: None)
            self.assertEqual(result["status"], "blocked")
            self.assertEqual(len(io.sent), 8)
            self.assertTrue(all(len(frame) == 3 for frame in io.sent))

    def test_enabled_during_comparison_halts_trials_but_still_reads_four_final_flags(self):
        io = FakeTransport()
        io.mutation = lambda tx, raw, count: bytes((1, 0x3A, 1, 0x6B)) if count == 5 else raw
        result = probe.run_probe(io, lambda event: None)
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(len(io.sent), 9)
        self.assertEqual(io.sent[-4:], [probe.status_query(axis) for axis in probe.IDS])

    def test_raw_echo_is_retained_but_invalid_status_cannot_pass(self):
        tx = probe.aggregate_status_query(2, 8)
        self.assertEqual(probe.decode_status_reply(tx + b"\x02\x3a\x02\x6b", 2, tx), {"flags": 2, "disabled": True})
        self.assertIsNone(probe.decode_status_reply(tx, 2, tx))
        self.assertIsNone(probe.decode_status_reply(b"\x02\x3a\xe2\x6b", 2, tx))

    def test_final_trial_enabled_cannot_be_hidden_by_disabled_postflight(self):
        io = FakeTransport()
        io.mutation = lambda tx, raw, count: b"\x04\x3a\x01\x6b" if count == 20 else raw
        result = probe.run_probe(io, lambda event: None)
        self.assertEqual(result["comparison_count"], 16)
        self.assertTrue(result["postflight_disabled"])
        self.assertEqual(result["status"], "blocked")

    def test_transport_error_keeps_raw_received_bytes_and_blocks(self):
        class FailedTransport(FakeTransport):
            def transact(self, tx):
                result = super().transact(tx)
                if len(self.sent) == 5:
                    result["transport_error"] = "fake timeout after partial RX"
                return result
        io, events = FailedTransport(), []
        result = probe.run_probe(io, events.append)
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(events[4]["rx_hex"], "013a026b")
        self.assertEqual(result["comparison_count"], 1)

    def test_transport_last_boundary_rejects_before_any_write(self):
        class NeverSerial:
            def __getattr__(self, name):
                raise AssertionError("Rejected frame touched serial: " + name)
        with self.assertRaises(ValueError):
            probe.SerialQueryTransport(NeverSerial()).transact(b"\x01\xf3\xab\x01\x00\x6b")

    def test_archive_uses_unique_directory_snapshot_and_matching_hash(self):
        with tempfile.TemporaryDirectory() as root:
            first = probe.ProbeArchive(root, probe.__file__)
            summary = probe.run_probe(FakeTransport(), first.emit)
            first.finish(summary)
            second = probe.ProbeArchive(root, probe.__file__); second.finish({"status": "test"})
            self.assertNotEqual(first.directory, second.directory)
            events = [json.loads(line) for line in (first.directory / "events.jsonl").read_text(encoding="utf-8").splitlines()]
            source = Path(events[0]["snapshot"]).read_bytes()
            self.assertEqual(events[0]["sha256"], hashlib.sha256(source).hexdigest())
            self.assertEqual(len(events), 26)
            self.assertEqual(json.loads((first.directory / "summary.json").read_text())["comparison_count"], 16)


if __name__ == "__main__":
    unittest.main()
