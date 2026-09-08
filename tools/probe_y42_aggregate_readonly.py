"""Standalone 0x3A-only diagnostic. Never enables, stops or configures a motor.

Run only after the web service has released COM7. Every transmitted byte string
is checked at the transport boundary; the deliberately old aggregate length is
tested only with one read-only status subcommand.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time
import uuid


IDS = (1, 2, 3, 4)


def status_query(address):
    if address not in IDS:
        raise ValueError("Only motor IDs 1..4 are allowed")
    return bytes((address, 0x3A, 0x6B))


def aggregate_status_query(address, declared_length):
    if declared_length not in (8, 6):
        raise ValueError("Only complete length 8 and historical length 6 are allowed")
    return bytes((0, 0xAA, 0, declared_length)) + status_query(address) + b"\x6b"


def validate_readonly_frame(frame):
    frame = bytes(frame)
    allowed = {status_query(axis) for axis in IDS}
    allowed.update(aggregate_status_query(axis, size) for axis in IDS for size in (8, 6))
    if frame not in allowed:
        raise ValueError("Refused: only exact 0x3A status queries are permitted")
    return frame


def decode_status_reply(raw, address, transmitted):
    """Permit a complete local TX echo, but never hide any raw bytes in logs."""
    raw = bytes(raw)
    if raw.startswith(transmitted):
        raw = raw[len(transmitted):]
    if len(raw) != 4 or raw[:2] != bytes((address, 0x3A)) or raw[-1] != 0x6B:
        return None
    flags = raw[2]
    if flags & ~0x3F:  # E2/EE are not disabled-state proof.
        return None
    return {"flags": flags, "disabled": not bool(flags & 1)}


def run_probe(transport, emit, now_ns=time.monotonic_ns, unix_ns=time.time_ns):
    """Injected IO runner; only this function schedules the 24 query frames."""
    transactions = []

    def query(axis, phase, declared_length=None, repeat=None):
        tx = validate_readonly_frame(status_query(axis) if declared_length is None
            else aggregate_status_query(axis, declared_length))
        row = {"event": "status_query", "phase": phase, "address": axis,
            "declared_length": declared_length, "repeat": repeat,
            "tx_hex": tx.hex(), "tx_bytes": len(tx), "started_unix_ns": unix_ns(),
            "started_monotonic_ns": now_ns()}
        try:
            reply = transport.transact(tx)
            row.update(reply)
            raw = bytes.fromhex(row.get("rx_hex", ""))
            row["decoded"] = decode_status_reply(raw, axis, tx)
        except Exception as error:
            row.update(rx_hex="", decoded=None, transport_error=f"{type(error).__name__}: {error}")
        row.update(completed_unix_ns=unix_ns(), completed_monotonic_ns=now_ns())
        transactions.append(row)
        emit(row)
        return row

    def disabled(rows):
        return all(not row.get("transport_error") and row["decoded"] is not None
                   and row["decoded"]["disabled"] for row in rows)

    preflight = [query(axis, "preflight") for axis in IDS]
    preflight_passed = disabled(preflight)
    comparisons = []
    if preflight_passed:
        interrupted = False
        for axis in IDS:
            for declared in (8, 6):
                for repeat in (1, 2):
                    row = query(axis, "comparison", declared, repeat)
                    comparisons.append(row)
                    # A valid enabled reply invalidates the bench premise. Do
                    # not send stop/disable: this probe has no such authority.
                    if row.get("transport_error") or row["decoded"] is not None and not row["decoded"]["disabled"]:
                        interrupted = True
                        break
                if interrupted:
                    break
            if interrupted:
                break
    postflight = [query(axis, "postflight") for axis in IDS]
    comparisons_safe = all(not row.get("transport_error") and
        (row["decoded"] is None or row["decoded"]["disabled"]) for row in comparisons)
    summary = {"event": "summary", "port": "COM7", "baud": 921600,
        "readonly_commands_only": True, "preflight_disabled": preflight_passed,
        "postflight_disabled": disabled(postflight), "comparison_count": len(comparisons),
        "status": "completed" if preflight_passed and disabled(postflight) and comparisons_safe and len(comparisons) == 16 else "blocked",
        "lengths": {str(size): {"attempted": sum(row["declared_length"] == size for row in comparisons),
            "valid_status_replies": sum(row["declared_length"] == size and row["decoded"] is not None
                                        and not row.get("transport_error") for row in comparisons)} for size in (8, 6)},
        "motion_or_configuration_commands_sent": False}
    emit(summary)
    return summary


class SerialQueryTransport:
    """Serial object injected after explicit CLI validation; no hidden opening."""
    def __init__(self, connection):
        self.connection = connection

    def transact(self, tx):
        tx = validate_readonly_frame(tx)  # Last boundary before serial.write.
        connection = self.connection
        stale, received = bytearray(), bytearray()
        result = {}
        deadline = time.monotonic() + 0.5
        try:
            # Preserve queued RX rather than silently dropping it with reset.
            while connection.in_waiting:
                if time.monotonic() >= deadline or len(stale) > 65536:
                    raise RuntimeError("RX is not quiet; service/timed stream may still be active")
                connection.timeout = min(0.01, max(0.001, deadline - time.monotonic()))
                stale.extend(connection.read(min(connection.in_waiting, 4096)))
            connection.write_timeout = 0.2
            written = connection.write(tx)
            result["written_bytes"] = written
            if written != len(tx):
                raise RuntimeError("Partial status-query write")
            last_byte = None
            while time.monotonic() < deadline:
                now = time.monotonic()
                if last_byte is not None and now - last_byte >= 0.025:
                    break
                connection.timeout = min(0.01, max(0.001, deadline - now))
                data = connection.read(min(max(1, connection.in_waiting), 4096))
                if data:
                    received.extend(data)
                    last_byte = time.monotonic()
                if len(received) > 65536:
                    raise RuntimeError("Unexpected continuous RX; refusing further trials")
        except Exception as error:
            result["transport_error"] = f"{type(error).__name__}: {error}"
        result.update(rx_hex=received.hex(), rx_bytes=len(received), stale_rx_hex=stale.hex(), stale_rx_bytes=len(stale))
        return result


class ProbeArchive:
    def __init__(self, root, source_path):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%fZ")
        self.directory = Path(root) / f"{stamp}_aggregate_status_readonly_{uuid.uuid4().hex}"
        self.directory.mkdir(parents=True, exist_ok=False)
        snapshot = self.directory / "source_snapshot"
        snapshot.mkdir(exist_ok=False)
        source = Path(source_path).read_bytes()
        destination = snapshot / Path(source_path).name
        with destination.open("xb") as handle:
            handle.write(source)
        self.events = (self.directory / "events.jsonl").open("x", encoding="utf-8")
        self.emit({"event": "source_snapshot", "source_file": str(Path(source_path).resolve()),
            "snapshot": str(destination), "sha256": hashlib.sha256(source).hexdigest(),
            "byte_count": len(source), "timestamp_unix_ns": time.time_ns()})

    def emit(self, event):
        self.events.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
        self.events.flush()
        os.fsync(self.events.fileno())

    def finish(self, summary):
        self.events.close()
        with (self.directory / "summary.json").open("x", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2, allow_nan=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, choices=["COM7"])
    parser.add_argument("--baud", required=True, type=int, choices=[921600])
    parser.add_argument("--output-root", type=Path, default=Path(__file__).with_name("manual_test_runs"))
    args = parser.parse_args(argv)
    archive = ProbeArchive(args.output_root, __file__)
    summary = {"status": "blocked", "readonly_commands_only": True,
        "port": args.port, "baud": args.baud, "motion_or_configuration_commands_sent": False}
    try:
        import serial  # Only CLI execution imports/opens serial, never tests.
        connection = serial.Serial(port=None, baudrate=args.baud, timeout=0.01,
            write_timeout=0.2, rtscts=False, dsrdtr=False)
        # Match the established service's DTR=False without an opening pulse.
        connection.dtr = False
        connection.port = args.port
        connection.open()
        with connection:
            summary = run_probe(SerialQueryTransport(connection), archive.emit)
    except Exception as error:
        summary["error"] = f"{type(error).__name__}: {error}"
        archive.emit({"event": "probe_error", **summary})
    finally:
        archive.finish(summary)
    print(json.dumps({"directory": str(archive.directory), **summary}, ensure_ascii=False))
    return 0 if summary["status"] == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
