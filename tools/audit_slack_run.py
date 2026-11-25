"""Read-only packet and local-datum audit of the recorded 100 Hz run."""
import json
import math
import tempfile
import sys
from pathlib import Path
from test_y42_calibration_flow import isolated_web_module

ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'tools/manual_test_runs'/ (sys.argv[1] if len(sys.argv)>1 else '20260908_135946_683_all_test_patterns_worker_fd96933d')
OUT=ROOT/'qa/slack_audit_20260908'
OUT.mkdir(exist_ok=True)
events=[json.loads(s) for s in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
manifest=json.loads((RUN/'manifest.json').read_text(encoding='utf-8'))
cal=manifest['calibration_before']
rebase=next(e for e in events if e['event']=='trajectory_start_local_rebase')
n=next(e['frames'] for e in events if e['event']=='trajectory_complete')
frames=[e for e in events if e['event']=='high_rate_tx_frame' and e.get('writer_thread')=='y42-trajectory-frame-writer'][:n]
errors=[]; decoded=[]
for frame in frames:
    p=bytes.fromhex(frame['raw_packet_hex'])
    if len(p)!=61 or p[:2]!=b'\x00\xaa' or int.from_bytes(p[2:4],'big')!=61 or p[-1]!=0x6b:
        errors.append(frame['sequence']);continue
    motors=[]
    for i in range(4):
        b=p[4+i*14:18+i*14]
        if b[0]!=i+1 or b[1]!=0xcb or b[-1]!=0x6b or b[9:11]!=b'\x01\x00':errors.append(frame['sequence'])
        pos=int.from_bytes(b[5:9],'big')/10*(-1 if b[2] else 1)
        if cal['encoder_positive_direction'][i]:pos=-pos
        motors.append({'target_deg':pos,'rpm':int.from_bytes(b[3:5],'big')/10,'current_cap_ma':int.from_bytes(b[11:13],'big')})
    decoded.append({'time':frame['unix_ns']/1e9,'axes':motors})
with tempfile.TemporaryDirectory() as d:
    w=isolated_web_module(d)
    slopes=w.effective_slopes(cal)
    bias_mm=[o/s for o,s in zip(rebase['offsets_deg'],slopes)]
    # Show a constant angle bias is not a constant-yaw cable correction.
    yaw=rebase['observed_yaw_deg']; x0,y0=rebase['absolute_start_xy']
    base=[a-b for a,b in zip(w.planar_platform_cable_lengths(x0,y0,yaw,cal),w.planar_platform_cable_lengths(x0,y0,0,cal))]
    drift=[]
    for x in [10,55,100]:
        for y in [10,74,138]:
            delta=[a-b-c for a,b,c in zip(w.planar_platform_cable_lengths(x,y,yaw,cal),w.planar_platform_cable_lengths(x,y,0,cal),base)]
            drift.append({'xy':[x,y],'constant_yaw_correction_change_mm':delta})
short=[0]*4
for previous,current in zip(decoded,decoded[1:]):
    for i,(a,b) in enumerate(zip(previous['axes'],current['axes'])):
        if abs(b['target_deg']-a['target_deg'])>b['rpm']*6*.01+1e-7:short[i]+=1
result={'run':RUN.name,'frames':len(frames),'malformed_frames':len(errors),'quantized_step_speed_insufficient_per_axis':short,'current_caps_per_axis':[sorted(set(f['axes'][i]['current_cap_ma'] for f in decoded)) for i in range(4)],'rpm_ranges':[[min(f['axes'][i]['rpm'] for f in decoded),max(f['axes'][i]['rpm'] for f in decoded)] for i in range(4)],'start_offsets_deg':rebase['offsets_deg'],'equivalent_constant_length_offsets_mm':bias_mm,'start_estimated_error_mm':rebase['absolute_start_error_mm'],'start_estimated_yaw_deg':yaw,'fixed_yaw_correction_variation':drift,'warning':'Yaw and start XY are encoder/model estimates, not optically measured. This quantifies model inconsistency, not measured physical slack. Valid host packets do not prove motor receipt.'}
(OUT/(RUN.name+'_audit.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
