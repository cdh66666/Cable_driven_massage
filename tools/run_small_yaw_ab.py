"""Bounded real-device A/B: one calibration, two identical central circles."""
import json
import math
import time
import urllib.request
from pathlib import Path

def api(route, data=None):
    body=json.dumps(data).encode() if data is not None else None
    request=urllib.request.Request('http://127.0.0.1:8787'+route,data=body,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=15) as response:return json.load(response)

def wait_idle(timeout):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        s=api('/api/auto')
        if s.get('auto')=='error':raise RuntimeError(s.get('message'))
        if s.get('auto')=='idle':return s
        time.sleep(.5)
    raise TimeoutError('Operation did not finish; no next motion submitted')

results=[]
api('/api/arm',{'armed':True,'confirmed':True})
print(api('/api/auto',{'action':'calibrate','force_full':True,'confirmed':True}),flush=True)
s=wait_idle(55)
if s.get('calibration_stage')!='complete':raise RuntimeError('Calibration incomplete')
points=[{'x':(55+15*math.cos(2*math.pi*i/96))/110,'y':1-(74+15*math.sin(2*math.pi*i/96))/148} for i in range(97)]
for trial in [False,True]:
    print(api('/api/path',{'points':points,'speed':30,'confirmed':True,'open_loop':True,'session_yaw_trial':trial}),flush=True)
    s=wait_idle(30)
    results.append({'trial':trial,'last_run':s.get('manual_test_last_run'),'message':s.get('message'),'session_offsets':s.get('trajectory_session_offset_deg')})
    print(results[-1],flush=True)
Path('qa/slack_audit_20260908/small_yaw_ab.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
