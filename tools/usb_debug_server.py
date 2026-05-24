from __future__ import annotations

import json
import mimetypes
import re
import struct
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import serial
from serial.tools import list_ports


HOST = "127.0.0.1"
PORT = 8765
BAUD = 460800
CANVAS_MM = 150.0
MAX_POINTS = 200000
SERIAL_BOOT_WAIT = 3.0


_serial_lock = threading.Lock()
_serial_conn: serial.Serial | None = None
_serial_port = ""


def find_web_html() -> Path | None:
    candidates = [
        Path(__file__).with_name("usb_debug_web.html"),
        Path.cwd() / "tools" / "usb_debug_web.html",
        Path(sys.executable).resolve().parent / "tools" / "usb_debug_web.html",
        Path(sys.executable).resolve().parent.parent / "tools" / "usb_debug_web.html",
    ]
    bundle_dir = getattr(sys, "_MEIPASS", "")
    if bundle_dir:
        candidates.append(Path(bundle_dir) / "tools" / "usb_debug_web.html")
    return next((path for path in candidates if path.exists()), None)


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>USB轨迹调试端</title>
<style>
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:#eef0f3;color:#111}
main{max-width:1180px;margin:0 auto;padding:22px;display:grid;grid-template-columns:300px 1fr;gap:20px}
.panel{background:#fff;border:1px solid #d7dce2;border-radius:8px;padding:14px}
h1{font-size:22px;margin:0 0 14px}
button,label select,input{width:100%;font-size:15px}
button{height:42px;border:0;border-radius:7px;background:#111;color:#fff;font-weight:650;margin:6px 0}
button.secondary{background:#4b5563}
button.stop{background:#991b1b}
button.green{background:#047857}
select,input{height:36px;margin:6px 0 12px}
#pad{width:100%;aspect-ratio:1/1;background:#fff;border:2px solid #111;display:block}
#status{white-space:pre-wrap;line-height:1.45;font-size:14px;color:#222;margin-top:10px}
.row{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.muted{color:#555;font-size:13px;line-height:1.45}
</style>
</head>
<body>
<main>
  <section class="panel">
    <h1>USB轨迹调试端</h1>
    <label>串口</label>
    <select id="ports"></select>
    <button id="refresh" class="secondary">刷新串口</button>
    <label>轨迹 TXT / JSON</label>
    <input id="file" type="file" accept=".txt,.json,text/plain,application/json">
    <button id="upload">上传轨迹到板子</button>
    <div class="row">
      <button id="start" class="green">开始</button>
      <button id="stop" class="stop">停止</button>
    </div>
    <div class="row">
      <button id="loopOn" class="secondary">循环开</button>
      <button id="loopOff" class="secondary">循环关</button>
    </div>
    <button id="statusBtn" class="secondary">读取状态</button>
    <p class="muted">这个页面运行在电脑本地，通过 USB 串口和板子通信，不需要手机连接热点。</p>
    <div id="status">等待操作。</div>
  </section>
  <section class="panel">
    <canvas id="pad" width="900" height="900"></canvas>
  </section>
</main>
<script>
const ports=document.getElementById('ports');
const statusEl=document.getElementById('status');
const fileInput=document.getElementById('file');
const pad=document.getElementById('pad');
const ctx=pad.getContext('2d');
let strokes=[];
function setStatus(t){statusEl.textContent=t;}
function draw(){
  ctx.fillStyle='#fff'; ctx.fillRect(0,0,pad.width,pad.height);
  ctx.strokeStyle='#111'; ctx.lineWidth=2; ctx.strokeRect(34,34,pad.width-68,pad.height-68);
  ctx.save(); ctx.translate(34,34);
  const s=(pad.width-68)/150;
  ctx.strokeStyle='#111'; ctx.lineWidth=1.1; ctx.lineJoin='round'; ctx.lineCap='round';
  for(const stroke of strokes){
    if(stroke.length<2)continue;
    ctx.beginPath();
    ctx.moveTo(stroke[0].x*s,(150-stroke[0].y)*s);
    for(const p of stroke.slice(1))ctx.lineTo(p.x*s,(150-p.y)*s);
    ctx.stroke();
  }
  ctx.restore();
}
function parsePoint(p){
  if(Array.isArray(p))return {x:+p[0],y:+p[1]};
  return {x:+p.x,y:+p.y};
}
function parseTrajectory(text){
  const t=text.trim();
  if(!t)return [];
  if(t[0]==='['){
    return JSON.parse(t).map(s=>s.map(parsePoint)).filter(s=>s.length>=2);
  }
  return t.split(/\r?\n/).map(line=>line.trim()).filter(Boolean).map(line=>
    line.split(';').map(pair=>{
      const [x,y]=pair.split(',').map(Number);
      return {x,y};
    }).filter(p=>Number.isFinite(p.x)&&Number.isFinite(p.y))
  ).filter(s=>s.length>=2);
}
function bodyForCommand(cmd){
  return JSON.stringify({port:ports.value,command:cmd});
}
async function refreshPorts(){
  const res=await fetch('/api/ports');
  const data=await res.json();
  ports.innerHTML='';
  for(const p of data.ports){
    const opt=document.createElement('option');
    opt.value=p.device; opt.textContent=p.label;
    ports.appendChild(opt);
  }
  setStatus(data.ports.length?'已找到串口。':'没有找到可用串口。');
}
async function upload(){
  const file=fileInput.files[0];
  if(!file){setStatus('请先选择轨迹 TXT 或 JSON。');return;}
  const text=await file.text();
  strokes=parseTrajectory(text);
  draw();
  const points=strokes.reduce((sum,s)=>sum+s.length,0);
  setStatus(`正在上传 ${strokes.length} 段，${points} 点...`);
  const res=await fetch('/api/upload',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({port:ports.value,strokes})});
  const data=await res.json();
  setStatus(data.ok?`上传完成：${data.message}`:`上传失败：${data.error}`);
}
async function command(cmd){
  const res=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:bodyForCommand(cmd)});
  const data=await res.json();
  setStatus(data.ok?data.message:`失败：${data.error}`);
}
document.getElementById('refresh').onclick=refreshPorts;
document.getElementById('upload').onclick=upload;
document.getElementById('start').onclick=()=>command('webstart');
document.getElementById('stop').onclick=()=>command('webstop');
document.getElementById('loopOn').onclick=()=>command('loopon');
document.getElementById('loopOff').onclick=()=>command('loopoff');
document.getElementById('statusBtn').onclick=()=>command('usbstatus');
fileInput.onchange=async()=>{
  const file=fileInput.files[0]; if(!file)return;
  strokes=parseTrajectory(await file.text()); draw();
  setStatus(`已载入文件预览：${strokes.length} 段，${strokes.reduce((a,s)=>a+s.length,0)} 点。`);
};
draw(); refreshPorts();
</script>
</body>
</html>
"""


def port_score(port: object) -> tuple[int, str]:
    device = str(getattr(port, "device", ""))
    desc = str(getattr(port, "description", ""))
    text = f"{device} {desc}".lower()
    score = 100
    if "bluetooth" in text or "蓝牙" in text:
        score += 100
    if "ch343" in text or "usb-enhanced" in text:
        score -= 40
    if "usb" in text:
        score -= 20
    return score, device


def available_ports() -> list[dict[str, str]]:
    out = []
    for port in sorted(list_ports.comports(), key=port_score):
        device = str(port.device)
        desc = str(port.description or "")
        out.append({"device": device, "label": f"{device} - {desc}" if desc else device})
    return out


def close_serial() -> None:
    global _serial_conn, _serial_port
    if _serial_conn is not None:
        try:
            if _serial_conn.is_open:
                _serial_conn.close()
        finally:
            _serial_conn = None
            _serial_port = ""


def get_serial(port: str) -> serial.Serial:
    global _serial_conn, _serial_port
    if _serial_conn is not None and _serial_conn.is_open and _serial_port == port:
        return _serial_conn

    close_serial()
    print(f"Opening serial {port}...", flush=True)
    ser = serial.Serial(port, BAUD, timeout=0.25, write_timeout=0, rtscts=False, dsrdtr=False)
    ser.dtr = False
    ser.rts = False
    _serial_conn = ser
    _serial_port = port

    # Opening the USB CDC/serial port can reset the ESP32-S3. Keep this handle
    # alive so upload -> status/start uses the same boot session.
    time.sleep(SERIAL_BOOT_WAIT)
    ser.reset_input_buffer()
    print(f"Serial {port} ready.", flush=True)
    return ser


def flatten_strokes(strokes: list[list[dict[str, float]]]) -> bytes:
    points = bytearray()
    count = 0
    for stroke in strokes:
        if len(stroke) < 2:
            continue
        for index, p in enumerate(stroke):
            x100 = max(0, min(int(round(float(p["x"]) * 100.0)), int(CANVAS_MM * 100)))
            y100 = max(0, min(int(round(float(p["y"]) * 100.0)), int(CANVAS_MM * 100)))
            points.extend(struct.pack("<HHB", x100, y100, 0 if index == 0 else 1))
            count += 1
    if count < 2:
        raise ValueError("轨迹太短")
    if count > MAX_POINTS:
        raise ValueError(f"点数 {count} 超过板子上限 {MAX_POINTS}")
    return bytes(points)


def send_command(port: str, command: str, wait_prefixes: tuple[str, ...], timeout: float = 5.0) -> str:
    with _serial_lock:
        ser = get_serial(port)
        ser.reset_input_buffer()
        ser.write((command.strip() + "\n").encode("ascii"))
        lines: list[str] = []
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            lines.append(text)
            if text.startswith(wait_prefixes):
                return text
        return "\n".join(lines[-8:]) or "未收到板子回复"


def upload_binary(port: str, strokes: list[list[dict[str, float]]]) -> str:
    payload = flatten_strokes(strokes)
    count = len(payload) // 5
    with _serial_lock:
        ser = get_serial(port)
        ser.reset_input_buffer()
        print(f"Uploading {count} binary points...", flush=True)
        ser.write(f"USBBIN {count}\n".encode("ascii"))
        ready = ""
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if text.startswith("USBBIN READY"):
                ready = text
                break
            if text.startswith("USBBIN ERROR"):
                raise RuntimeError(text)
        if not ready:
            raise RuntimeError("板子没有进入二进制接收模式")
        chunk = 4096
        for start in range(0, len(payload), chunk):
            ser.write(payload[start : start + chunk])
            time.sleep(0.002)
        print("Payload sent, waiting for board ack...", flush=True)
        end = time.monotonic() + 12.0
        lines: list[str] = []
        while time.monotonic() < end:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            lines.append(text)
            if text.startswith("USBBIN OK"):
                return text
            if text.startswith("USBBIN ERROR"):
                raise RuntimeError(text)
        raise RuntimeError("\n".join(lines[-8:]) or "上传后未收到确认")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        return

    def send_json(self, obj: object, status: int = 200) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        return json.loads(body or "{}")

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            html_path = find_web_html()
            body = html_path.read_bytes() if html_path else INDEX_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/ports":
            self.send_json({"ports": available_ports()})
            return
        self.send_error(404)

    def do_POST(self) -> None:
        try:
            data = self.read_json()
            path = urlparse(self.path).path
            if path == "/api/upload":
                msg = upload_binary(str(data["port"]), data["strokes"])
                self.send_json({"ok": True, "message": msg})
                return
            if path == "/api/command":
                cmd = str(data["command"])
                prefixes = ("USBSTATUS", "Web playback", "Web single", "Web playback stopped", "No playable", "USBSTATUS loop")
                msg = send_command(str(data["port"]), cmd, prefixes)
                self.send_json({"ok": True, "message": msg})
                return
            self.send_error(404)
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, 400)


def run(open_browser: bool = True) -> None:
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    url = f"http://{HOST}:{PORT}/"
    if open_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    print(f"USB debug web server: {url}")
    server.serve_forever()


if __name__ == "__main__":
    run()
