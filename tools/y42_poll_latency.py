"""Disabled-only read latency measurement; sends only addressed 0x43 reads."""
import ctypes
import json
import time
import urllib.request
from pathlib import Path
import y42_pc_direct as direct

with urllib.request.urlopen('http://127.0.0.1:8787/api/auto',timeout=3) as response:
    state=json.load(response)
if state['auto'] not in ('idle','error') or state['manual_test_recording']['active']:
    raise RuntimeError('Active worker; refuse separate serial access')
rows=[]
timer=ctypes.windll.winmm.timeBeginPeriod(1)==0
try:
    with direct.open_serial(state['port'],state['baud']) as connection:
        for i in range(40):
            address=i%4+1
            started=time.perf_counter()
            packet=bytes((address,0x43,0x7A,0x6B))
            connection.write(packet)
            buffer=bytearray()
            last=None
            while time.perf_counter()-started<.012:
                waiting=connection.in_waiting
                part=connection.read(min(waiting,128) if waiting else 1)
                if part:
                    buffer.extend(part);last=time.perf_counter()
                elif len(buffer)==37 and last is not None and time.perf_counter()-last>=.0005:
                    break
            reply=bytes(buffer)
            elapsed=time.perf_counter()-started
            rows.append(dict(id=address,seconds=elapsed,tx=packet.hex(),rx=reply.hex()))
            if len(reply)!=37 or reply[:4]!=bytes((address,0x43,0x25,0x0C)) or reply[-1]!=0x6B or reply[35]!=2:
                raise RuntimeError('Invalid or non-disabled feedback; abort latency test')
    times=sorted(row['seconds'] for row in rows)
    print(json.dumps(dict(reads=len(rows),min_ms=times[0]*1000,median_ms=times[len(times)//2]*1000,max_ms=times[-1]*1000,timer=timer)))
finally:
    if timer:
        ctypes.windll.winmm.timeEndPeriod(1)
    (Path(__file__).resolve().parents[1]/'qa/twin/com5-polled-read-latency.json').write_text(json.dumps(rows,indent=2),encoding='utf8')
