"""Recover the saved pre-experiment baud, with per-axis disabled/readback guards."""
import json
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
events = []

def raw(baud, frame):
    payload = dict(action="raw", baud=baud, hex_frame=frame.hex(" "), confirmed=True)
    request = urllib.request.Request("http://127.0.0.1:8787/api/command", json.dumps(payload).encode(), {"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=5) as response:
        result = json.load(response)
    events.append(dict(baud=baud, **result))
    return bytes.fromhex(result["rx"])

def main():
    snapshot = json.loads((ROOT / "qa/twin/baud-512000-change.json").read_text(encoding="utf-8-sig"))
    for item in snapshot["write_results"]:
        address = item["id"]
        restored = bytearray.fromhex(item["tx"])
        restored[22] = 8  # Manual: 921600 baud.
        expected = bytes([address, 0x42, 0x25, 0x18]) + restored[4:]
        for attempt in range(5):
            if raw(921600, bytes([address, 0x42, 0x6C, 0x6B])) == expected:
                break
            disabled = False
            for _ in range(8):
                reply = raw(512000, bytes([address, 0x3A, 0x6B]))
                if reply == bytes([address, 0x3A, 2, 0x6B]):
                    disabled = True
                    break
            if not disabled:
                raise RuntimeError(f"M{address}: cannot verify disabled; no write")
            raw(512000, restored)
            time.sleep(0.1)
        else:
            raise RuntimeError(f"M{address}: configuration restore not verified")
        print(f"M{address}: full original configuration verified at 921600", flush=True)

if __name__ == "__main__":
    try:
        main()
    finally:
        (ROOT / "qa/twin/baud-recovery-raw.json").write_text(json.dumps(events, indent=2), encoding="utf-8")
