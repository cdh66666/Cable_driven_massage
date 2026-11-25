"""Reassemble original RX chunks; report candidate invalid 0x43 frames by stage.
No controller imports or hardware IO. Candidates are not repaired or accepted.
"""
import json
import sys
from pathlib import Path

def analyze(path):
    events=[json.loads(s) for s in (path/'events.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
    chunks=[e for e in events if e.get('event')=='position_probe_raw_rx']
    buffer=bytearray();bad=[];valid=0
    for e in chunks:
        buffer.extend(bytes.fromhex(e['raw_packet_hex']))
        while len(buffer)>=37:
            if buffer[0] not in (1,2,3,4) or buffer[1]!=0x43:del buffer[0];continue
            p=bytes(buffer[:37])
            if p[-1]!=107:del buffer[0];continue
            reasons=[]
            if p[2:4]!=bytes.fromhex('250c'):reasons.append('length/type header')
            for i in (14,19,22,27,32):
                if p[i] not in (0,1):reasons.append(f'sign[{i}]={p[i]}')
            if p[35]&~63:reasons.append(f'flags={p[35]}')
            if int.from_bytes(p[4:6],'big')>60000:reasons.append('voltage>60V')
            if reasons:bad.append(dict(phase=e.get('phase'),monotonic_ns=e.get('monotonic_ns'),packet=p.hex(),reasons=reasons))
            else:valid+=1
            del buffer[:37]
    result=next((e for e in reversed(events) if e.get('event')=='run_end'),{}).get('result') or {}
    return dict(run=path.name,valid_structure_candidates=valid,invalid_structure_candidates=bad,reason=result.get('reason') or result.get('error'),final_disabled=result.get('final_disabled_confirmed'),restored=result.get('configuration_restored'))

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    print(json.dumps(analyze(Path(sys.argv[1])),ensure_ascii=False,indent=2))
