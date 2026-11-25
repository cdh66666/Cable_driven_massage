"""Same-session browser + camera evidence, local authorized narration."""
import json,subprocess,textwrap,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'qa/tutorial_landscape/real_lesson_20260908'
CAM=ROOT/'qa/camera_runs/20260908_161454_018795'
FF=r'D:\codex\tools\ffmpeg\ffmpeg-8.1.2-full_build\bin\ffmpeg.exe'
timing=json.loads((OUT/'screen_timing.json').read_text())
voice=json.loads((OUT/'voice_manifest.json').read_text(encoding='utf-8'))['segments']
camera_start=json.loads((CAM/'manifest.json').read_text())['capture_launch_unix']
names=['connect','identity','direction','calibrate','patterns','words','finish']
titles=['连接：读取四台','编号：4 → 5 → 4 回读','方向：四台正反测试并恢复','校准：完整四角','六图案：只保留一个矩形','星猫：20笔实机轨迹','完成：保持位置']
events=timing['events'];starts=[next(e['time']/1000 for e in events if e['name']==n) for n in names]
ends=starts[1:]+[starts[-1]+5.4]
rendered=[];cursor=0
def ass_time(s):return f'{int(s)//3600}:{int(s)//60%60:02}:{s%60:05.2f}'
for i,(start,end,v) in enumerate(zip(starts,ends,voice)):
    raw=end-start;duration=max(raw,v['duration_seconds']+.7);preroll=0
    if i==0:
        start=timing['started_unix_ms']/1000
        raw=.85
        preroll=3.0
    pad=duration-raw-preroll
    ass='''[Script Info]
ScriptType: v4.00+
PlayResX: 1920
PlayResY: 1080
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Default,Microsoft YaHei,30,&H00FFFFFF,&H00FFFFFF,&H00202020,&H00202020,0,0,0,0,100,100,0,0,1,2,0,2,30,30,24,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
'''
    def line(a,b,body):return f'Dialogue: 0,{ass_time(a)},{ass_time(b)},Default,,0,0,0,,{body}\n'
    ass+=line(0,duration,r'{\an7\pos(28,18)\fs34\b1}'+titles[i])
    ass+=line(0,duration,r'{\an7\pos(995,20)\fs28}实机相机 · 同次录制 / 时钟近似对齐')
    ass+=line(0,duration,r'{\an7\pos(995,610)\fs22}'+('卷盘局部放大' if i==2 else '中部局部放大 · 全景保留在上方'))
    words=v['text'].replace('。','。\n').strip().splitlines()
    total=sum(len(w) for w in words);st=0
    for phrase in words:
        en=st+v['duration_seconds']*len(phrase)/total
        ass+=line(st,en,r'{\an2\pos(960,1060)\fs31}'+phrase)
        st=en
    if preroll:ass+=line(0,preroll,r'{\an7\pos(1000,95)\fs22}讲解停帧')
    if pad>.1:ass+=line(raw+preroll,duration,r'{\an7\pos(1000,95)\fs22}讲解停帧')
    assfile=OUT/f'chapter_{i+1:02d}.ass';assfile.write_text(ass,encoding='utf-8')
    cx='480';cy='250'
    if i==2:
        ds=[e for e in events if e['name']=='motor_direction']
        a,b,c=[e['time']/1000-start for e in ds[1:]]
        cx=f'if(gte(t,{c}),0,if(gte(t,{a}),960,0))'
        cy=f'if(gte(t,{b}),680,0)'
    screen_crop='crop=960:960:0:0,scale=960:960' if i==0 else 'crop=960:800:0:175,scale=960:900'
    filt=(f'[0:v]trim=duration={raw},setpts=PTS-STARTPTS,{screen_crop},setsar=1,fps=30,tpad=start_mode=clone:start_duration={preroll}:stop_mode=clone:stop_duration={pad},pad=1920:1080:0:80:color=0x102436[base];'
          f'[1:v]trim=duration={raw},setpts=PTS-STARTPTS,fps=30,tpad=start_mode=clone:start_duration={preroll}:stop_mode=clone:stop_duration={pad},split[full][detail];'
          '[full]scale=960:540,setsar=1[cam];'
          f"[detail]crop=960:400:x='{cx}':y='{cy}',scale=900:375,setsar=1[zoom];"
          '[base][cam]overlay=960:65[m];[m][zoom]overlay=990:640[n];'
          f'[n]ass=chapter_{i+1:02d}.ass[out];[2:a]apad[a]')
    dest=OUT/f'chapter_{i+1:02d}.mp4'
    if '--first-only' not in sys.argv or i==0:
        subprocess.run([FF,'-hide_banner','-loglevel','error','-y','-ss',str(max(0,start-timing['started_unix_ms']/1000)),'-i',str(OUT/'real_screen.webm'),'-ss',str(max(0,start-camera_start)),'-i',str(CAM/'camera.mp4'),'-i',str(OUT/f'voice_{i+1:02d}.wav'),'-filter_complex',filt,'-map','[out]','-map','[a]','-t',str(duration),'-c:v','libx264','-preset','veryfast','-threads','4','-crf','21','-pix_fmt','yuv420p','-c:a','aac','-b:a','128k',str(dest)],cwd=OUT,check=True)
    rendered.append({'chapter':names[i],'title':titles[i],'start':cursor,'duration':duration,'source_start_unix':start,'source_duration':raw,'freeze_seconds':pad});cursor+=duration
    print('Rendered',names[i],round(duration,2),flush=True)
(OUT/'concat.txt').write_text(''.join(f"file 'chapter_{i:02d}.mp4'\n" for i in range(1,8)))
subprocess.run([FF,'-hide_banner','-loglevel','error','-y','-f','concat','-safe','0','-i',str(OUT/'concat.txt'),'-c','copy','-movflags','+faststart',str(OUT/'新手实机教程_网页相机同屏_本人声音.mp4')],check=True)
(OUT/'edit_manifest.json').write_text(json.dumps({'chapters':rendered,'duration':cursor,'screen':'real_screen.webm','camera':str(CAM/'camera.mp4'),'note':'Real clicks and same-session optical video; approximate clock alignment. End-of-chapter freeze frames explicitly labelled. No synthetic device feedback.'},ensure_ascii=False,indent=2),encoding='utf-8')
for label,idx in [('六图案_实机原速',4),('星猫_实机原速',5)]:
    subprocess.run([FF,'-hide_banner','-loglevel','error','-y','-ss',str(starts[idx]-camera_start),'-i',str(CAM/'camera.mp4'),'-t',str(ends[idx]-starts[idx]),'-an','-c:v','libx264','-preset','veryfast','-crf','20','-movflags','+faststart',str(OUT/(label+'.mp4'))],check=True)
for n,t in enumerate([2,rendered[1]['start']+3,rendered[2]['start']+8,rendered[5]['start']+7]):
    subprocess.run([FF,'-hide_banner','-loglevel','error','-y','-ss',str(t),'-i',str(OUT/'新手实机教程_网页相机同屏_本人声音.mp4'),'-frames:v','1',str(OUT/f'preview_{n+1}.jpg')],check=True)
print('Complete',cursor,flush=True)
