"""Render an original-speed, landscape calibration and playback evidence clip."""
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CAM = ROOT / 'qa/camera_runs/20260908_135905_590369'
RUN = ROOT / 'tools/manual_test_runs/20260908_135946_683_all_test_patterns_worker_fd96933d'
OUT = ROOT / 'qa/tutorial_landscape'
FFMPEG = r'D:\codex\tools\ffmpeg\ffmpeg-8.1.2-full_build\bin\ffmpeg.exe'

def stamp(t):
    t = max(0, t)
    return f'{int(t)//3600}:{int(t)//60%60:02}:{t%60:05.2f}'

manifest = json.loads((CAM/'manifest.json').read_text(encoding='utf-8'))
metrics = json.loads((RUN/'host_timing_analysis.json').read_text(encoding='utf-8'))
rows = [json.loads(line) for line in (CAM/'web_state.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
ass = r'''[Script Info]
ScriptType: v4.00+
PlayResX: 1920
PlayResY: 1080
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Default,Microsoft YaHei,32,&H00FFFFFF,&H00FFFFFF,&H001A1620,&H001A1620,0,0,0,0,100,100,0,0,1,1,0,2,40,40,30,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
'''
def caption(start, end, text):
    global ass
    ass += f'Dialogue: 0,{stamp(start)},{stamp(end)},Default,,0,0,0,,{text}\n'

caption(0,100,r'{\an7\pos(48,28)\fs40}四角校准 → 七种图案连续绘制｜真实相机 · 原速')
caption(0,100,r'{\an2\pos(960,1060)\fs26}先预计算全部 3695 帧；921600 baud；目标 100 Hz。画面未经运动平滑。')
for i, row in enumerate(rows):
    start = row['received_unix'] - manifest['capture_launch_unix']
    end = rows[i+1]['received_unix'] - manifest['capture_launch_unix'] if i+1<len(rows) else 100
    state = row.get('state', {})
    mode = state.get('auto','unknown')
    label = {'idle':'待机 / 完成保持','all-test-patterns':'七种测试图案：连续绘制','calibrate':'四角校准'}.get(mode,mode)
    caption(start,end,r'{\an7\pos(48,88)\fs28}阶段：'+str(label))
m=metrics['intervals']
caption(0,100,r'{\an2\pos(960,1014)\fs28}本轮主机发送：平均 '+f"{metrics['average_hz']:.3f} Hz / P95 {m['p95_ms']:.3f} ms / 最长 {m['max_ms']:.2f} ms"+r'；不是电机到位频率')
(OUT/'precomputed_demo.ass').write_text(ass,encoding='utf-8')
output=OUT/'预计算轨迹_四角校准与七图案实拍.mp4'
subprocess.run([FFMPEG,'-hide_banner','-loglevel','error','-y','-i',str(CAM/'camera.mp4'),'-vf','scale=1536:864,pad=1920:1080:192:132:color=0x101c30,ass=precomputed_demo.ass','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],cwd=OUT,check=True)
print(output)
