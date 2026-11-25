import json
import re
import subprocess
import wave
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'qa/tutorial_landscape'
FF=r'D:\codex\tools\ffmpeg\ffmpeg-8.1.2-full_build\bin\ffmpeg.exe'
audit=json.loads((OUT/'voice_generation_audit.json').read_text(encoding='utf-8'))
durations=json.loads((OUT/'narration_durations.json').read_text())
intro=sum(durations[:6]); total=intro+37
with wave.open(str(OUT/'tutorial_voice_merged.wav'),'wb') as merged:
    params=None
    for i in range(1,8):
        with wave.open(str(OUT/f'narration_{i:02d}.wav'),'rb') as source:
            current=(source.getnchannels(),source.getsampwidth(),source.getframerate())
            if params is None:
                params=current;merged.setnchannels(params[0]);merged.setsampwidth(params[1]);merged.setframerate(params[2])
            assert current==params
            merged.writeframes(source.readframes(source.getnframes()))
    merged.writeframes(b'\0'*int(max(0,total-sum(durations))*params[2])*params[0]*params[1])
def stamp(t):
    c=round(t*100);return f'{c//360000}:{c//6000%60:02}:{c//100%60:02}.{c%100:02}'
header='''[Script Info]
ScriptType: v4.00+
PlayResX: 1920
PlayResY: 1080
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Default,Microsoft YaHei,30,&H00FFFFFF,&H00FFFFFF,&H001F172A,&H001F172A,0,0,0,0,100,100,0,0,1,1,0,2,100,100,42,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
'''
lines=[header];start=0
for i,segment in enumerate(audit['segments']):
    text=segment['text']; chunks=re.findall(r'[^。！？]+[。！？]?',text)
    weight=sum(len(c) for c in chunks)
    for c in chunks:
        end=start+durations[i]*len(c)/weight
        wrapped='\\N'.join(c[j:j+48] for j in range(0,len(c),48))
        lines.append(f'Dialogue: 1,{stamp(start)},{stamp(end)},Default,,0,0,0,,{wrapped}\n')
        start=end
lines.append(f'Dialogue: 1,0:00:00.00,{stamp(total)},Default,,0,0,0,,{{\\an7\\pos(22,22)\\fs22}}调试版 · 本人录音参考合成配音\n')
lines.append(f'Dialogue: 1,0:00:00.00,{stamp(intro)},Default,,0,0,0,,{{\\an2\\pos(960,950)\\fs24}}网页章节：离线演示，不驱动实机\n')
lines.append(f'Dialogue: 1,{stamp(intro)},{stamp(total)},Default,,0,0,0,,{{\\an2\\pos(960,950)\\fs24}}实机章节：原速录像，未做硬件曝光同步\n')
(OUT/'voiced_tutorial.ass').write_text(''.join(lines),encoding='utf-8')
vf=f'[0:v]tpad=stop_mode=clone:stop_duration=2,trim=duration={intro},setpts=PTS-STARTPTS,fps=30,setsar=1[a];[1:v]trim=duration=37,setpts=PTS-STARTPTS,fps=30,setsar=1[b];[a][b]concat=n=2:v=1:a=0,scale=1600:900,pad=1920:1080:160:0:color=0x101c30,ass=voiced_tutorial.ass[v]'
target='横屏入门与实测对照_本人音色配音.mp4'
subprocess.run([FF,'-hide_banner','-loglevel','error','-y','-i','voice_matched_onboarding.webm','-i','线性模型与单时钟_实拍对照.mp4','-i','tutorial_voice_merged.wav','-filter_complex',vf,'-map','[v]','-map','2:a','-t',str(total),'-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-c:a','aac','-b:a','160k','-movflags','+faststart',target],cwd=OUT,check=True)
print(str(OUT/target))
