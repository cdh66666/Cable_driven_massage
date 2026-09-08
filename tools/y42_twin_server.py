"""Local twin sidecar: cached HTTP state + archive tail; never owns serial.

Motor actions are forwarded only to the existing controller's guarded APIs.
Simulation and live read loops do not send status queries to the motor bus.
"""
import collections
import hashlib
import importlib.util
import json
import mimetypes
import threading
import sys
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse,parse_qs
from y42_drawing_archive import drawing_from_events

ROOT=Path(__file__).resolve().parents[1]
RUNS=ROOT/'tools/manual_test_runs'
OUT=ROOT/'qa/twin';OUT.mkdir(exist_ok=True,parents=True)
LOCK=threading.Lock();SIM_LOCK=threading.Lock()
LIVE={'connected':False,'state':{},'motors':{},'samples':collections.deque(maxlen=2400)}
MODEL_LOCK=threading.Lock()
MODEL_CACHE={}


def simulation_model():
    """Load a fresh pure-model generation; no controller or serial imports.

    Existing requests keep their old module objects. Failed loads restore the
    previous module registry and fail the preview, never silently use old code.
    """
    names=('y42_exact_path','y42_spool_geometry','y42_twin_model')
    with MODEL_LOCK:
        sources={name:(ROOT/'tools'/f'{name}.py').read_bytes() for name in names}
        version=hashlib.sha256(b''.join(name.encode()+sources[name] for name in names)).hexdigest()
        if MODEL_CACHE.get('version')==version:return MODEL_CACHE['module'],version
        previous={name:sys.modules.get(name) for name in names}
        try:
            for name in names:
                path=ROOT/'tools'/f'{name}.py'
                spec=importlib.util.spec_from_file_location(name,path)
                module=importlib.util.module_from_spec(spec)
                sys.modules[name]=module
                exec(compile(sources[name],str(path),'exec'),module.__dict__)
        except Exception:
            for name,old in previous.items():
                if old is None:sys.modules.pop(name,None)
                else:sys.modules[name]=old
            raise
        MODEL_CACHE.update(module=module,version=version)
        return module,version


def controller(path,body=None):
    data=None if body is None else json.dumps(body).encode()
    request=urllib.request.Request('http://127.0.0.1:8787'+path,data=data,headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(request,timeout=5) as response:return json.load(response)
    except urllib.error.HTTPError as error:
        return json.loads(error.read())


def raw_samples(e):
    if e.get('event') in ('position_microprobe_raw_record','position_probe_fresh_axis'):
        if 'position' in e:return [dict(e)]
    if e.get('event')=='timed_feedback_frames':
        return [dict(id=r['address'],position=r['position_deg'],rpm=r.get('rpm'),phase_current=r.get('phase_current_ma'),sampled_at=r['sampled_monotonic_s'],raw_packet_hex=r.get('raw_packet_hex')) for r in e.get('samples',[])]
    return []


def poll():
    filename=None;offset=0;remainder=b'';seen=set()
    while True:
        try:
            state=controller('/api/auto')
            active=state.get('manual_test_recording') or {}
            if not active.get('run_id'):active=state.get('manual_test_last_run') or {}
            run_id=active.get('run_id') or ''
            path=(RUNS/run_id/'events.jsonl').resolve()
            if run_id and path.parent.parent==RUNS.resolve() and path.exists():
                if filename!=path:
                    filename=path;offset=0;remainder=b'';seen=set()
                    with LOCK:LIVE['samples'].clear();LIVE['motors'].clear()
                with path.open('rb') as f:f.seek(offset);chunk=f.read();offset=f.tell()
                lines=(remainder+chunk).split(b'\n');remainder=lines.pop()
                for line in lines:
                    if not line:continue
                    try:e=json.loads(line)
                    except ValueError:continue
                    for r in raw_samples(e):
                        key=(r['id'],r.get('sampled_at'),r.get('raw_packet_hex'))
                        if key in seen:continue
                        seen.add(key)
                        r['run_id']=run_id
                        with LOCK:
                            LIVE['motors'][str(r['id'])]=r;LIVE['samples'].append(r)
            with LOCK:LIVE.update(connected=True,state=state,received_at=time.time(),error=None)
        except Exception as exc:
            with LOCK:LIVE.update(connected=False,error=str(exc))
        time.sleep(.5)


def archive(run_id):
    path=(RUNS/run_id).resolve()
    if path.parent!=RUNS.resolve():raise ValueError('invalid run')
    events=[]
    with (path/'events.jsonl').open(encoding='utf-8') as f:
        for line in f:
            if line.strip():events.append(json.loads(line))
    samples=[];seen=set();targets=[]
    for e in events:
        for r in raw_samples(e):
            k=(r['id'],r.get('sampled_at'),r.get('raw_packet_hex'))
            if k not in seen:seen.add(k);samples.append(r)
        if e.get('event')=='position_probe_serial_write' and e.get('phase')=='microprobe_command_stream':
            b=bytes.fromhex(e['attempted_tx_hex'])
            if len(b)==61 and e.get('bytes_written')==61 and not e.get('error'):
                q=[]
                for i in range(4):
                    p=b[4+14*i:18+14*i];q.append((1 if p[2]==[0,0,1,1][i] else -1)*int.from_bytes(p[5:9],'big')/10)
                targets.append({'t':e['started_monotonic_ns']/1e9,'q':q})
    end=next((e for e in reversed(events) if e.get('event')=='run_end'),{})
    return dict(run_id=run_id,samples=samples,targets=targets,drawing=drawing_from_events(events),result=end.get('result'),error=end.get('error'),source_sha256=hashlib.sha256((path/'events.jsonl').read_bytes()).hexdigest())


class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def send(self,data,status=200,kind='application/json; charset=utf-8'):
        body=json.dumps(data,ensure_ascii=False,allow_nan=False).encode() if not isinstance(data,bytes) else data
        self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers()
        try:self.wfile.write(body)
        except (BrokenPipeError,ConnectionResetError):pass
    def do_GET(self):
        parsed=urlparse(self.path)
        try:
            if parsed.path=='/':return self.send((ROOT/'tools/y42_twin.html').read_bytes(),kind='text/html; charset=utf-8')
            if parsed.path=='/api/live':
                with LOCK:
                    value={**LIVE,'motors':dict(LIVE['motors']),'samples':list(LIVE['samples'])}
                value['host_monotonic_s']=time.perf_counter()
                return self.send(value)
            if parsed.path=='/api/runs':return self.send([p.parent.name for p in sorted(RUNS.glob('*/events.jsonl'),reverse=True)][:150])
            if parsed.path=='/api/replay':return self.send(archive(parse_qs(parsed.query)['id'][0]))
            if parsed.path=='/api/defaults':return self.send(simulation_model()[0].DEFAULTS)
            if parsed.path=='/api/model-version':
                _,version=simulation_model()
                return self.send({'version':version,'hot_reload':True,'scope':'simulation_only'})
            self.send({'error':'not found'},404)
        except Exception as exc:self.send({'error':str(exc)},400)
    def do_POST(self):
        origin=self.headers.get('Origin')
        if origin and origin!='http://127.0.0.1:8788':return self.send({'error':'origin rejected'},403)
        try:
            size=int(self.headers.get('Content-Length','0'))
            if size>1_000_000:raise ValueError('request too large')
            data=json.loads(self.rfile.read(size) or '{}')
            if self.path=='/api/simulate':
                if not SIM_LOCK.acquire(False):return self.send({'error':'仿真正在计算'},409)
                try:
                    model,version=simulation_model()
                    result=model.simulate(data)
                    result['model_version']=version
                finally:SIM_LOCK.release()
                run_id=time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%1000000)
                (OUT/(run_id+'_simulation.json')).write_text(json.dumps({'input':data,'result':result},ensure_ascii=False),encoding='utf-8')
                result['simulation_id']=run_id;return self.send(result)
            if self.path=='/api/action':
                action=data.get('action')
                if action=='refresh-status':
                    result=controller('/api/status')
                    with LOCK:
                        for row in result.get('motors',[]):LIVE['motors'][str(row['id'])]={**row,'sampled_at':time.perf_counter() if not result.get('status_deferred') else None}
                    return self.send(result)
                if action not in ('position-microprobe','calibrate','stop'):raise ValueError('未允许的操作')
                body={'action':action,'confirmed':True}
                if action=='calibrate':body['force_full']=True
                return self.send(controller('/api/auto',body))
            if self.path=='/api/path':
                points=data.get('points')
                if not isinstance(points,list) or not 2<=len(points)<=6000:raise ValueError('轨迹点数2..6000')
                if any(len(p)!=3 or abs(p[2]-25)>.001 for p in points):raise ValueError('当前实机只接收接触面XY轨迹；抬高未获可观测/可控验证')
                # Controller preserves its independent calibration and motion gates.
                normalized=[{'x':p[0]/110,'y':1-p[1]/148} for p in points]
                return self.send(controller('/api/path',{'points':normalized,'speed':data.get('speed',10),'confirmed':True}))
            self.send({'error':'not found'},404)
        except Exception as exc:self.send({'error':str(exc)},400)


if __name__=='__main__':
    threading.Thread(target=poll,daemon=True).start()
    print('Twin: http://127.0.0.1:8788',flush=True)
    ThreadingHTTPServer(('127.0.0.1',8788),Handler).serve_forever()
