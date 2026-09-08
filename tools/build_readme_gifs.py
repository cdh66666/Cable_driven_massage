"""Small, real-speed optical GIFs, rotated to the user's confirmed motor view."""
import json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'qa/tutorial_landscape/real_lesson_20260908'
OUT=ROOT/'docs/media';OUT.mkdir(parents=True,exist_ok=True)
FF=r'D:\codex\tools\ffmpeg\ffmpeg-8.1.2-full_build\bin\ffmpeg.exe'
clips=[('test-patterns-a','六图案_实机原速.mp4',5,8),('test-patterns-b','六图案_实机原速.mp4',18,8),('test-patterns-c','六图案_实机原速.mp4',41,8),('xingmao-motion','星猫_实机原速.mp4',8,8)]
records=[]
for name,file,start,duration in clips:
    # Original camera: TL=3, TR=4, BL=2, BR=1. Rotate 180°, not a mirror.
    vf='hflip,vflip,scale=400:-2:flags=lanczos,fps=8'
    for label,x,y in [('ID1','10','8'),('ID2','w-tw-10','8'),('ID3','w-tw-10','h-th-8'),('ID4','10','h-th-8')]:
        vf+=f",drawtext=fontfile='C\\:/Windows/Fonts/arial.ttf':text='{label}':x={x}:y={y}:fontsize=17:fontcolor=white:box=1:boxcolor=black@0.65:boxborderw=4"
    vf+=",drawtext=fontfile='C\\:/Windows/Fonts/arial.ttf':text='1x | 50 mm/s':x=(w-tw)/2:y=8:fontsize=14:fontcolor=white:box=1:boxcolor=black@0.65:boxborderw=4"
    graph=f'[0:v]{vf},split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=none[v]'
    dest=OUT/(name+'.gif')
    subprocess.run([FF,'-hide_banner','-loglevel','error','-y','-ss',str(start),'-t',str(duration),'-i',str(SOURCE/file),'-filter_complex',graph,'-map','[v]','-loop','0',str(dest)],check=True)
    subprocess.run([FF,'-hide_banner','-loglevel','error','-y','-ss',str(start+2),'-i',str(SOURCE/file),'-vf',vf,'-frames:v','1',str(OUT/(name+'-preview.jpg'))],check=True)
    records.append({'file':dest.name,'bytes':dest.stat().st_size,'source':str((SOURCE/file).relative_to(ROOT)),'start_seconds':start,'duration_seconds':duration,'speed':1,'rotation_degrees':180,'fps':8,'motor_view':{'top_left':1,'top_right':2,'bottom_right':3,'bottom_left':4}})
    print(name,round(dest.stat().st_size/1024),'KiB',flush=True)
(OUT/'capture-provenance.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
