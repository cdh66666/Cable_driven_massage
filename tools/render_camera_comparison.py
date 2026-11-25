"""Landscape evidence reel; clocks approximately aligned at host command start."""
import json
import argparse
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FFMPEG = r'D:\codex\tools\ffmpeg\ffmpeg-8.1.2-full_build\bin\ffmpeg.exe'
OUT = ROOT/'qa'/'tutorial_landscape'
OUT.mkdir(parents=True,exist_ok=True)
parser=argparse.ArgumentParser()
parser.add_argument('--before-camera',default='20260908_114238_896673')
parser.add_argument('--before-run',default='20260908_114249_228_all_test_patterns_worker_cd2fb57f')
parser.add_argument('--after-camera',default='20260908_120725_566245')
parser.add_argument('--after-run',default='20260908_120752_372_all_test_patterns_worker_5eba41db')
parser.add_argument('--output',default='实拍对照_无配音草稿.mp4')
parser.add_argument('--duration',type=float,default=37.0)
args=parser.parse_args()
sources = [(args.before_camera,args.before_run),(args.after_camera,args.after_run)]
inputs=[]
alignment=[]
for camera,run in sources:
    directory=ROOT/'qa'/'camera_runs'/camera
    manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    events=[json.loads(s) for s in (ROOT/'tools'/'manual_test_runs'/run/'events.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
    first=next(e for e in events if e.get('event')=='high_rate_tx_frame' and e.get('writer_thread')=='y42-trajectory-frame-writer')
    offset=max(0,first['unix_ns']/1e9-manifest['capture_launch_unix'])
    inputs += ['-ss',str(offset),'-i',str(directory/'camera.mp4')]
    alignment.append({'camera':camera,'run':run,'offset_seconds':offset,'alignment':'Approximate host-clock alignment, not synchronized hardware exposures'})
ass='''[Script Info]
ScriptType: v4.00+
PlayResX: 1920
PlayResY: 1080
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Default,Microsoft YaHei,34,&H00FFFFFF,&H00FFFFFF,&H001F172A,&H001F172A,0,0,0,0,100,100,0,0,1,0,0,2,40,40,35,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
Dialogue: 0,0:00:00.00,0:00:37.00,Default,,0,0,0,,{\\an7\\pos(64,55)\\fs46\\b1}实机录像对照 · 同一套七种图案
Dialogue: 0,0:00:00.00,0:00:37.00,Default,,0,0,0,,{\\an7\\pos(64,126)\\fs25}软件修订前后 · 原速播放 · 横屏 1920 × 1080 · 真实录像证据
Dialogue: 0,0:00:00.00,0:00:37.00,Default,,0,0,0,,{\\an7\\pos(40,196)}之前的实测
Dialogue: 0,0:00:00.00,0:00:37.00,Default,,0,0,0,,{\\an7\\pos(1000,196)}本轮实测（尚未验收通过）
Dialogue: 0,0:00:00.00,0:00:37.00,Default,,0,0,0,,{\\an7\\pos(40,820)\\fs28}主机发帧间隔 P95：11.78ms\\N超过15ms：31次；最大26.50ms
Dialogue: 0,0:00:00.00,0:00:37.00,Default,,0,0,0,,{\\an7\\pos(1000,820)\\fs28}主机发帧间隔 P95：10.87ms\\N超过15ms：15次；最大22.79ms
Dialogue: 0,0:00:00.00,0:00:37.00,Default,,0,0,0,,{\\an2\\pos(960,1005)\\fs26}两次独立实测不能证明改善全由修改导致。保留松绳与抖动，不伪造通过。\\N按主机命令时间近似对齐；相机未做硬件同步。数字不是实际球心误差。
'''
metrics=[]
for _,run in sources:
    metrics.append(json.loads((ROOT/'tools/manual_test_runs'/run/'host_timing_analysis.json').read_text(encoding='utf-8')))
ass=ass.replace('11.78ms','__BEFORE_P95__').replace('10.87ms','__AFTER_P95__')
ass=ass.replace('__BEFORE_P95__',f"{metrics[0]['intervals']['p95_ms']:.2f}ms").replace('31次；最大26.50ms',f"{metrics[0]['intervals_over_15ms']}次；最大{metrics[0]['intervals']['max_ms']:.2f}ms")
ass=ass.replace('__AFTER_P95__',f"{metrics[1]['intervals']['p95_ms']:.2f}ms").replace('15次；最大22.79ms',f"{metrics[1]['intervals_over_15ms']}次；最大{metrics[1]['intervals']['max_ms']:.2f}ms")
if args.duration != 37.0:
    seconds=max(1.0,min(300.0,args.duration))
    ass=ass.replace('0:00:37.00',f'{int(seconds)//3600}:{int(seconds)//60%60:02}:{seconds%60:05.2f}')
    ass=ass.replace('按主机命令时间近似对齐；相机未做硬件同步。数字不是实际球心误差。',
                    '只对齐开始时刻：转角减速后各图案时刻不同。数字是主机时序，不是球心误差。')
(OUT/'comparison.ass').write_text(ass,encoding='utf-8')
filters='[0:v]scale=960:540,setsar=1,fps=30[a];[1:v]scale=960:540,setsar=1,fps=30[b];[a][b]hstack=inputs=2,pad=1920:1080:0:250:color=0x101c30,ass=comparison.ass[v]'
subprocess.run([FFMPEG,'-hide_banner','-loglevel','error','-y',*inputs,'-filter_complex',filters,'-map','[v]','-an','-t',str(args.duration),'-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',args.output],cwd=OUT,check=True)
(OUT/(Path(args.output).stem+'_alignment.json')).write_text(json.dumps(alignment,ensure_ascii=False,indent=2),encoding='utf-8')
print(str(OUT/args.output))
