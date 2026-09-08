"""Read-only COM5 link diagnosis: status frames only, no motor writes."""
import json
import time
import urllib.request
from pathlib import Path
import y42_pc_direct as direct

def main():
    with urllib.request.urlopen('http://127.0.0.1:8787/api/auto', timeout=3) as response:
        state = json.load(response)
    if state['auto'] not in ('idle', 'error'):
        raise RuntimeError('Controller is active; no parallel serial diagnosis allowed')
    output = dict(port='COM5', started=time.time(), ports=[dict(device=p.device, description=p.description, hwid=p.hwid) for p in direct.list_serial_ports()], rows=[])
    path = Path(__file__).resolve().parents[1] / 'qa/twin/com5-readonly-link-matrix.json'
    try:
        for baud in (921600, 115200, 512000, 256000):
            for mode in ('auto', 'rts-high', 'rts-low'):
                row = dict(baud=baud, direction=mode, transactions=[])
                output['rows'].append(row)
                with direct.open_serial('COM5', baud, direction_mode=mode) as connection:
                    time.sleep(0.15)
                    for address in (1, 2, 3, 4):
                        tx = bytes((address, 0x3A, 0x6B))
                        start = time.perf_counter()
                        rx = direct.transact(connection, tx, first_byte_timeout=0.15, deadline=start+0.25)
                        row['transactions'].append(dict(tx=tx.hex(), rx=rx.hex(), seconds=time.perf_counter()-start))
                print(json.dumps(row), flush=True)
    finally:
        path.write_text(json.dumps(output, indent=2), encoding='utf-8')

if __name__ == '__main__':
    main()
