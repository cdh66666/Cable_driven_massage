"""Complete interrupted cap restoration, only after fresh disabled verification."""
import json
import urllib.request
from pathlib import Path

events=[]
def api(path, data=None):
    request=urllib.request.Request('http://127.0.0.1:8787'+path,
        None if data is None else json.dumps(data).encode(), {'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=5) as response:
        return json.load(response)

def raw(tx):
    result=api('/api/command',dict(action='raw',hex_frame=tx.hex(' '),confirmed=True))
    events.append(result)
    return bytes.fromhex(result['rx'])

def main():
    state=api('/api/auto')
    if state['auto'] not in ('idle','error') or state['manual_test_recording']['active']:
        raise RuntimeError('Worker still active')
    originals=state['position_probe']['original_configurations_hex']
    for key, value in originals.items():
        address=int(key)
        original=bytes.fromhex(value)
        if len(original)!=37 or original[:4]!=bytes((address,0x42,0x25,0x18)) or original[-1]!=0x6B:
            raise RuntimeError('Invalid original snapshot')
        status=api('/api/status')['motors']
        if len(status)!=4 or any(not m['online'] or m['rpm']!=0 or m['flags']!=2 for m in status):
            raise RuntimeError('Four motors must be freshly stopped and disabled')
        current=raw(bytes((address,0x42,0x6C,0x6B)))
        if current == original:
            continue
        if (len(current)!=37 or current[:16]+current[20:]!=original[:16]+original[20:]
                or int.from_bytes(current[16:18],'big') not in (0,20,150)
                or int.from_bytes(current[18:20],'big')!=1):
            raise RuntimeError(f'M{address} unexpected configuration; no overwrite')
        ack=raw(bytes((address,0x48,0xD1,0))+original[4:])
        if ack!=bytes((address,0x48,2,0x6B)):
            raise RuntimeError('Restore ACK not confirmed')
        if raw(bytes((address,0x42,0x6C,0x6B)))!=original:
            raise RuntimeError('Full configuration readback mismatch')
        print(f'M{address}: complete original configuration restored',flush=True)
    print('All four original configurations verified',flush=True)

if __name__=='__main__':
    try:
        main()
    finally:
        (Path(__file__).resolve().parents[1]/'qa/twin/com5-probe-cap-recovery.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
