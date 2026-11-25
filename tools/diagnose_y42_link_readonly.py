"""Independent raw serial status probe; never imports or runs the controller.

Only addressed 0x3A status queries are allowed. No enable, motion, broadcast,
firmware, baud-setting or other configuration frames can be transmitted.
All attempts and raw bytes are saved to a new timestamped diagnostic file.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import serial


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--bauds", type=int, nargs="+", default=[921600, 115200])
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    folder = Path(__file__).resolve().parents[1] / "qa" / "connection_probes"
    folder.mkdir(parents=True, exist_ok=True)
    output = folder / (stamp + "_readonly.json")
    report = {"started_utc": stamp, "port": args.port, "query_only": True, "attempts": []}
    try:
        for baud in args.bauds:
            for mode in ("auto", "static-rts-low", "rts-high", "rts-low"):
                with serial.Serial(args.port, baud, timeout=0.01, write_timeout=0.3,
                                   rtscts=False, dsrdtr=False) as connection:
                    connection.dtr = False
                    if mode == "static-rts-low":
                        connection.rts = False
                    time.sleep(0.15)
                    for address in (1, 2, 3, 4):
                        tx = bytes((address, 0x3A, 0x6B))
                        assert tx in {bytes((i, 0x3A, 0x6B)) for i in (1, 2, 3, 4)}
                        connection.reset_input_buffer()
                        if mode.startswith("rts-"):
                            connection.rts = mode == "rts-high"
                        item = {"baud": baud, "direction": mode, "address": address,
                                "tx_hex": tx.hex(), "written_bytes": 0, "rx_hex": ""}
                        report["attempts"].append(item)
                        started = time.perf_counter()
                        received = bytearray()
                        try:
                            item["written_bytes"] = connection.write(tx)
                            if mode.startswith("rts-"):
                                time.sleep(max(0.001, len(tx) * 10 / baud))
                                connection.rts = mode != "rts-high"
                            while time.perf_counter() - started < 0.3:
                                chunk = connection.read(min(connection.in_waiting or 1, 128))
                                received.extend(chunk)
                                if received and not chunk:
                                    break
                        finally:
                            item["rx_hex"] = received.hex()
                            item["elapsed_s"] = round(time.perf_counter() - started, 6)
                        print(json.dumps(item), flush=True)
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        with output.open("x", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2)
        print("Saved read-only connection evidence: " + str(output), flush=True)


if __name__ == "__main__":
    run()
