import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
camera=ROOT/'qa/camera_runs/20260908_120725_566245'
start=json.loads((camera/'manifest.json').read_text(encoding='utf-8'))['capture_launch_unix']
rows=[json.loads(s) for s in (ROOT/'tools/manual_test_runs/20260908_120752_372_all_test_patterns_worker_5eba41db/events.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
output=[]
for row in rows:
    t=row['unix_ns']/1e9-start
    if 39.0 <= t <= 40.3 and row.get('record_channel')=='process_log':
        output.append({'camera_approx_s':round(t,3),'event':row['event'],'data':{k:v for k,v in row.items() if k not in {'sequence','unix_ns','monotonic_ns','record_channel'}}})
frames=[r for r in rows if r.get('event')=='high_rate_tx_frame' and r.get('packet_bytes')==61 and 39<=r['unix_ns']/1e9-start<=40.3]
last_bucket=-1
for row in frames:
    t=row['unix_ns']/1e9-start
    bucket=int(t*10)
    if bucket==last_bucket:continue
    last_bucket=bucket
    packet=bytes.fromhex(row['raw_packet_hex']); motors=[]
    for i in range(4):
        b=packet[4+14*i:4+14*(i+1)]
        motors.append({'id':b[0],'target_driver_deg':int.from_bytes(b[5:9],'big')/10*(-1 if b[2] else 1),'rpm':int.from_bytes(b[3:5],'big')/10,'cap_ma':int.from_bytes(b[11:13],'big')})
    output.append({'camera_approx_s':round(t,3),'motors':motors})
(camera/'slack_window_commands.json').write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(output,ensure_ascii=False))
