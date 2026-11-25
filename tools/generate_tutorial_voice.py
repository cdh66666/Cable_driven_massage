"""Use the authorized local voice reference; keep an audit of generated segments."""
import json
import shutil
import urllib.request
import uuid
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'qa'/'tutorial_landscape'
reference=Path(r'D:\HONOR Share\Honor Share\录音机-20260831-1746.m4a')
paragraphs=[s.strip() for s in (OUT/'narration_draft_zh.txt').read_text(encoding='utf-8').split('\n\n') if s.strip() and not s.startswith('本稿等待')]
audit=[]
for i,text in enumerate(paragraphs,1):
    dest=OUT/f'narration_{i:02d}.wav'
    if dest.exists():
        audit.append({'index':i,'text':text,'audio':str(dest),'reused':True})
        continue
    boundary='tutorial-'+uuid.uuid4().hex
    parts=[]
    for key,value in {'text':text,'cfg_value':'2.0','inference_timesteps':'10'}.items():
        parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="{key}"\r\n\r\n{value}\r\n'.encode('utf-8'))
    parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="reference_audio"; filename="user-reference.m4a"\r\nContent-Type: audio/mp4\r\n\r\n'.encode()+reference.read_bytes()+b'\r\n')
    parts.append(f'--{boundary}--\r\n'.encode())
    request=urllib.request.Request('http://127.0.0.1:8808/api/generate',data=b''.join(parts),headers={'Content-Type':f'multipart/form-data; boundary={boundary}'})
    with urllib.request.urlopen(request,timeout=900) as response:
        result=json.load(response)
    source=Path(r'D:\codex\voxcpm-voice-studio\data\outputs')/result['audio_url'].split('/')[-1]
    shutil.copyfile(source,dest)
    audit.append({'index':i,'text':text,'audio':str(dest),'engine_result':result})
    (OUT/'voice_generation_audit.json').write_text(json.dumps({'reference':str(reference),'segments':audit},ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'Generated narration segment {i}/{len(paragraphs)}',flush=True)
