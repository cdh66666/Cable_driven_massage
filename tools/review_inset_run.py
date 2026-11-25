import json,cv2,numpy as np,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[1]
cam=root/'qa/camera_runs/20260908_152413_534551'
run=root/'tools/manual_test_runs/20260908_152424_242_all_test_patterns_worker_7749d006'
out=root/'qa/slack_audit_20260908/inset80';out.mkdir(exist_ok=True)
events=[json.loads(l) for l in (run/'events.jsonl').read_text(encoding='utf-8').splitlines()]
first=next(e for e in events if e.get('event')=='high_rate_tx_frame' and e.get('writer_thread')=='y42-trajectory-frame-writer')
offset=first['unix_ns']/1e9-json.loads((cam/'manifest.json').read_text())['capture_launch_unix']
for page in range(5):
    sheet=np.full((4*250,3*400,3),245,np.uint8)
    for k in range(12):
        t=page*12+k
        if t>55:break
        frame=cv2.imread(str(cam/f'frame_{round((offset+t)*10)+1:05d}.jpg'))
        if frame is None:continue
        tile=cv2.resize(frame,(400,225));y=(k//3)*250;x=(k%3)*400
        sheet[y:y+225,x:x+400]=tile
        cv2.putText(sheet,f'Motion ~{t:.1f}s / inset80 / 50mm/s',(x+8,y+243),cv2.FONT_HERSHEY_SIMPLEX,.47,(20,20,20),1)
    cv2.imwrite(str(out/f'page_{page+1}.jpg'),sheet)
(out/'alignment.json').write_text(json.dumps({'offset':offset,'note':'Approximate host/camera start alignment; actual video, not measured tension'},indent=2))
ff=r'D:\codex\tools\ffmpeg\ffmpeg-8.1.2-full_build\bin\ffmpeg.exe'
subprocess.run([ff,'-hide_banner','-loglevel','error','-y','-ss',str(max(0,offset-1)),'-i',str(cam/'camera.mp4'),'-t','58','-an','-c:v','libx264','-preset','fast','-crf','20','-movflags','+faststart',str(out/'七图案_80范围_50速度_实机原速.mp4')],check=True)
print(str(out),flush=True)
