"""Record real camera video, 10 Hz review frames and read-only web state.

No motion commands or serial reads are issued by this recorder.
"""
import argparse
import datetime as dt
import json
from pathlib import Path
import subprocess
import threading
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--camera', default='2K Camera')
    parser.add_argument('--ffmpeg', default=r'D:\codex\tools\ffmpeg\ffmpeg-8.1.2-full_build\bin\ffmpeg.exe')
    parser.add_argument('--url', default='http://127.0.0.1:8787')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error('seconds must be between 1 and 300')
    folder = Path(__file__).resolve().parents[1] / 'qa' / 'camera_runs' / dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    stop = threading.Event()
    started = time.time()

    def sample():
        with (folder / 'web_state.jsonl').open('w', encoding='utf-8') as out:
            while not stop.is_set():
                requested = time.time()
                try:
                    # /api/auto returns cached controller state; /api/status
                    # can initiate serial traffic and must not be polled here.
                    with urllib.request.urlopen(args.url + '/api/auto', timeout=2) as response:
                        state = json.load(response)
                    row = {'requested_unix': requested, 'received_unix': time.time(), 'state': state}
                except Exception as exc:
                    row = {'requested_unix': requested, 'received_unix': time.time(), 'error': str(exc)}
                out.write(json.dumps(row, ensure_ascii=False) + '\n')
                out.flush()
                stop.wait(max(0, .5 - (time.time() - requested)))

    thread = threading.Thread(target=sample, daemon=True)
    thread.start()
    command = [args.ffmpeg, '-hide_banner', '-y', '-f', 'dshow', '-i', 'video=' + args.camera,
               '-t', str(args.seconds), '-an', '-c:v', 'libx264', '-preset', 'ultrafast',
               '-crf', '22', '-pix_fmt', 'yuv420p', str(folder / 'camera.mp4'),
               '-t', str(args.seconds), '-an', '-vf', 'fps=10,scale=960:-2',
               '-q:v', '3', '-update', '1', '-atomic_writing', '1', str(folder / 'live.jpg')]
    print(str(folder), flush=True)
    try:
        with (folder / 'capture.log').open('w', encoding='utf-8') as log:
            result = subprocess.run(command, stdout=log, stderr=log, timeout=args.seconds + 20)
    finally:
        stop.set()
        thread.join(timeout=3)
    if result.returncode:
        raise RuntimeError('Camera capture failed; see ' + str(folder / 'capture.log'))
    subprocess.run([args.ffmpeg, '-hide_banner', '-loglevel', 'error', '-i', str(folder / 'camera.mp4'),
                    '-vf', 'fps=10,scale=960:-2', '-q:v', '3', str(folder / 'frame_%05d.jpg')], check=True)
    manifest = {'camera': args.camera, 'capture_launch_unix': started, 'capture_end_unix': time.time(),
                'review_fps': 10, 'frame_count': len(list(folder.glob('frame_*.jpg'))),
                'timing_note': 'Video PTS starts at first captured frame; process launch is not hardware exposure synchronization.',
                'measurement_note': 'Real optical video; no calibrated XY measurement or tension measurement yet.'}
    (folder / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    (folder / 'review.html').write_text('''<!doctype html><meta charset="utf-8"><title>实机录像回放</title>
<style>body{background:#f5f7fa;color:#223;font:18px system-ui;max-width:1100px;margin:30px auto}video,img{width:100%;border-radius:14px}button{padding:12px;margin:8px}input{width:70%}</style>
<h1>实机录像回放</h1><p>真实相机画面 · 每秒 10 帧复查。尚未标定图像坐标，不将像素轨迹称为毫米实测。</p>
<video id="v" src="camera.mp4" controls></video><p><button onclick="step(-.1)">上一帧</button><button onclick="step(.1)">下一帧</button><span id="t"></span></p>
<script>let v=document.getElementById('v');function step(d){v.pause();v.currentTime=Math.max(0,Math.min(v.duration,v.currentTime+d))}v.ontimeupdate=()=>document.getElementById('t').textContent=v.currentTime.toFixed(2)+' 秒';</script>
<p><a href="manifest.json">录像信息</a> · <a href="web_state.jsonl">带时间戳的控制状态</a></p>''', encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
