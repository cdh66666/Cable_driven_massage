"""Same-session, explicit trial comparison; no automatic recalibration."""
import json,time,urllib.request
from pathlib import Path
def api(route,data=None):
    r=urllib.request.Request('http://127.0.0.1:8787'+route,data=json.dumps(data).encode() if data is not None else None,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(r,timeout=15) as response:return json.load(response)
points=[{'x':x,'y':y} for x,y in [(.1,.1),(.9,.1),(.9,.9),(.1,.9),(.1,.1)]]
results=[]
for trial in [False,True]:
    print(api('/api/path',{'points':points,'speed':50,'confirmed':True,'open_loop':True,'session_yaw_trial':trial}),flush=True)
    deadline=time.monotonic()+45
    while True:
        s=api('/api/auto')
        if s.get('auto')=='error':raise RuntimeError(s.get('message'))
        if s.get('auto')=='idle':break
        if time.monotonic()>deadline:raise TimeoutError('No subsequent motion submitted')
        time.sleep(.5)
    results.append({'trial':trial,'last_run':s.get('manual_test_last_run'),'message':s.get('message')})
    print(results[-1],flush=True)
Path('qa/slack_audit_20260908/outer_yaw_ab.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
