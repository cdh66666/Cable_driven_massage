"""Compare addressed and AA-wrapped read-only queries; never sends motion."""
import json
import time
import urllib.request
from pathlib import Path
import serial

s = json.load(urllib.request.urlopen('http://127.0.0.1:8787/api/auto'))
assert s['auto'] in ('idle', 'error') and not s['manual_test_recording']['active']
assert s['position_probe']['final_disabled_confirmed'] is True
rows = []
with serial.Serial('COM5', 921600, timeout=.001, write_timeout=.05) as bus:
    assert not bus.in_waiting
    for packet in (bytes.fromhex('01437a6b'), bytes.fromhex('00aa000901437a6b6b'),
                   bytes.fromhex('01366b'), bytes.fromhex('00aa000801366b6b')):
        start = time.perf_counter()
        bus.write(packet)
        rx = bytearray()
        while time.perf_counter() - start < .1:
            rx.extend(bus.read(bus.in_waiting or 1))
        rows.append(dict(tx=packet.hex(), rx=rx.hex()))
path = Path(__file__).resolve().parents[1] / 'qa/twin/com5-readonly-aggregate-check.json'
path.write_text(json.dumps(rows, indent=2), encoding='utf-8')
print(json.dumps(rows))
