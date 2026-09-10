# -*- coding: utf-8 -*-
"""打包版 EXE 的交付前验证：一个单文件，换到任何目录双击都得能用。

覆盖 5 种场景：
  T1  换到全新的、带中文名的目录             -> 必须照常起来
  T2  放到「下载」文件夹（用户实际遇到的位置） -> 必须照常起来
  T3  已经有一份在跑，又双击另一份            -> 必须把已有页面调出来，不能默默退出
  T4  8787 被别的程序占着                    -> 必须自动换端口，浏览器地址正确
  T5  EXE 放在没有写权限的目录                -> 数据目录自动退到 %LOCALAPPDATA%

T3 不靠"看浏览器"，而是让占位的假调试台记录每个请求的 User-Agent：
页面本身是 Python 请求的，浏览器来访问一定带 Mozilla —— 这就可断言了。
"""
import json
import os
import shutil
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

REPO = r"D:\Cable_driven_massage-main"
PACK = os.path.join(REPO, "packaging")
EXE = os.path.join(REPO, "Y42调试台.exe")
DL = r"C:\Users\admin\Downloads"
CN_ROOT = r"D:\移动测试_中文目录"
CN_DIR = os.path.join(CN_ROOT, "新建文件夹")
RO_ROOT = r"D:\只读测试目录"
UA_LOG = os.path.join(PACK, "_ua_log.txt")

OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def sh(*args):
    try:
        r = subprocess.run(list(args), capture_output=True)
    except FileNotFoundError:
        return ""
    return (r.stdout or b"").decode("gbk", "replace") + (r.stderr or b"").decode("gbk", "replace")


def y42_pids():
    out = sh("tasklist", "/FO", "CSV")
    pids = []
    for line in out.splitlines():
        cells = line.split('","')
        if "Y42" in line and len(cells) > 2:
            try:
                pids.append(int(cells[1].strip('"')))
            except ValueError:
                pass
    return pids


def kill_all():
    for _ in range(8):
        pids = y42_pids()
        if not pids:
            return
        for pid in pids:
            sh("taskkill", "/PID", str(pid), "/T", "/F")
        time.sleep(1.1)


def port_listening(port):
    return bool([l for l in sh("netstat", "-ano").splitlines()
                 if f"127.0.0.1:{port} " in l and "LISTENING" in l])


def fetch(port, path="/", timeout=1.0):
    return OPENER.open(f"http://127.0.0.1:{port}{path}", timeout=timeout).read().decode("utf-8", "replace")


def wait_real_page(port, timeout=45):
    t0 = time.perf_counter()
    tb = None
    while time.perf_counter() - t0 < timeout:
        el = (time.perf_counter() - t0) * 1000
        try:
            html = fetch(port, "/", 0.4)
            if tb is None and "正在启动，页面会自动刷新" in html:
                tb = el
            if 'id="livePorts"' in html:
                return tb, el, html
        except Exception:
            pass
        time.sleep(0.03)
    return tb, None, ""


def tail_log(folder, n=6):
    p = os.path.join(folder, "Y42调试台_数据", "Y42调试台_日志.txt")
    if not os.path.exists(p):
        return []
    return open(p, encoding="utf-8", errors="replace").read().splitlines()[-n:]


def run_exe(exe_path, args=(), wait_exit=0):
    proc = subprocess.Popen([exe_path, *args], cwd=os.path.dirname(exe_path))
    if wait_exit:
        try:
            proc.wait(timeout=wait_exit)
        except subprocess.TimeoutExpired:
            pass
    return proc


def browser_hit():
    """假调试台有没有收到带浏览器 UA 的请求 —— 证明"浏览器真的被打开了"。"""
    if not os.path.exists(UA_LOG):
        return None
    for line in open(UA_LOG, encoding="utf-8", errors="replace").read().splitlines():
        parts = line.split("\t")
        if len(parts) >= 3 and "Mozilla" in parts[2]:
            return parts
    return None


print("=" * 70)
print("准备：结束所有调试台实例")
kill_all()
print("  Y42 进程:", y42_pids() or "无")
print("  8787:", "占用中" if port_listening(8787) else "空闲")
for f in (UA_LOG,):
    if os.path.exists(f):
        os.remove(f)

# ------------------------------------------------------------------ T1 中文目录
print()
print("=" * 70)
print(f"T1  全新中文名目录：{CN_DIR}")
os.makedirs(CN_DIR, exist_ok=True)
cn_exe = os.path.join(CN_DIR, "Y42调试台.exe")
shutil.copy2(EXE, cn_exe)
run_exe(cn_exe, ["--no-browser"])
tb, tr, html = wait_real_page(8787)
print(f"  占位页 {tb:.0f} ms | 真实页 {tr:.0f} ms" if tr else "  失败：没等到真实页面")
print(f"  页面 {len(html)} bytes | 串口选择器={'id=\"quickPort\"' in html} | 四页导航={'workflowNav' in html}")
print(f"  数据目录建在同级: {os.path.isdir(os.path.join(CN_DIR, 'Y42调试台_数据'))}")
kill_all()
time.sleep(0.8)

# ------------------------------------------------------------------ T2 下载文件夹
print()
print("=" * 70)
print(f"T2  「{DL}」文件夹（用户实际遇到的位置）")
dl_exe = os.path.join(DL, "Y42调试台.exe")
try:
    shutil.copy2(EXE, dl_exe)
except PermissionError:
    print("  (副本被占用，跳过覆盖，用现有副本)")
run_exe(dl_exe, ["--no-browser"])
tb, tr, html = wait_real_page(8787)
print(f"  占位页 {tb:.0f} ms | 真实页 {tr:.0f} ms" if tr else "  失败：没等到真实页面")
print(f"  页面 {len(html)} bytes | 串口选择器={'id=\"quickPort\"' in html} | 四页导航={'workflowNav' in html}")
kill_all()
time.sleep(0.8)


# ------------------------------------------------------------------ T3 已有实例在跑
class LoggingConsole(BaseHTTPRequestHandler):
    """假调试台：只回答 /api/auto，其余给个普通页面，并记录每个请求的 UA。"""

    def do_GET(self):
        ua = self.headers.get("User-Agent", "")
        with open(UA_LOG, "a", encoding="utf-8") as fh:
            fh.write(f"{time.time():.3f}\t{self.path}\t{ua}\n")
        if self.path.startswith("/api/auto"):
            body = json.dumps({"auto": "idle"}).encode()
            ctype = "application/json"
        else:
            body = b"<html><body>fake console</body></html>"
            ctype = "text/html"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


print()
print("=" * 70)
print("T3  已经有一份在跑，再双击另一份（放在别的目录里）")
if os.path.exists(UA_LOG):
    os.remove(UA_LOG)
squatter = ThreadingHTTPServer(("127.0.0.1", 8787), LoggingConsole)
threading.Thread(target=squatter.serve_forever, daemon=True).start()
print("  假调试台已占住 8787")

t0 = time.perf_counter()
proc_b = run_exe(cn_exe, wait_exit=40)
el = (time.perf_counter() - t0) * 1000
print(f"  B 退出耗时 {el:.0f} ms，退出码 {proc_b.returncode}")
for line in tail_log(CN_DIR, 3):
    print("     ", line)
hit = browser_hit()
time.sleep(1.0)
print(f"  发给浏览器后的首个浏览器请求: {hit if hit else '（没有浏览器来访问）'}")
print(f"  结论: {'✅ 浏览器被真正拉起（已有页面被调出）' if hit else '❌ 浏览器没有被拉起'}")
squatter.shutdown()
squatter.server_close()
time.sleep(0.6)

# ------------------------------------------------------------------ T4 端口被别的程序占用
print()
print("=" * 70)
print("T4  8787 被一个「不是调试台」的程序占着")


class NotConsole(BaseHTTPRequestHandler):
    def do_GET(self):
        body = b"nope"
        self.send_response(404)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


occupier = ThreadingHTTPServer(("127.0.0.1", 8787), NotConsole)
threading.Thread(target=occupier.serve_forever, daemon=True).start()
print("  已用一个假程序占住 8787")
run_exe(cn_exe, ["--no-browser"])
time.sleep(6)
lines = tail_log(CN_DIR, 8)
port = None
for line in lines:
    if "改用端口" in line:
        try:
            port = int(line.rsplit("端口", 1)[1].strip())
        except ValueError:
            pass
print("  EXE 日志:")
for line in lines:
    print("     ", line)
ok = False
if port:
    try:
        html = fetch(port, "/", 3.0)
        ok = 'id="livePorts"' in html
    except Exception as e:
        print("  取页面失败:", e)
print(f"  自动端口 = {port}（已避开被占的 8787）| 页面可访问 = {ok}")
print(f"  结论: {'✅ 自动换端口且页面正常' if ok else '❌ 没有正确回退'}")
kill_all()
occupier.shutdown()
occupier.server_close()
time.sleep(0.8)

# ------------------------------------------------------------------ T5 只读目录
print()
print("=" * 70)
print(f"T5  EXE 放在没有写权限的目录：{RO_ROOT}")
shutil.rmtree(RO_ROOT, ignore_errors=True)
os.makedirs(RO_ROOT, exist_ok=True)
ro_exe = os.path.join(RO_ROOT, "Y42调试台.exe")
shutil.copy2(EXE, ro_exe)
print("  设置拒绝写入:", sh("icacls", RO_ROOT, "/deny", f"{os.environ.get('USERNAME','admin')}:(W,AD)").strip().splitlines()[-1:] or "")
squatter2 = ThreadingHTTPServer(("127.0.0.1", 8787), NotConsole)
threading.Thread(target=squatter2.serve_forever, daemon=True).start()   # 排除端口回退干扰
run_exe(ro_exe, ["--no-browser"])
time.sleep(6)
local = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Y42Console")
print("  同级数据目录可写:", os.path.isdir(os.path.join(RO_ROOT, "Y42调试台_数据")))
print(f"  退回 {local}: {os.path.isdir(local)}")
ok5 = os.path.isdir(local) and not os.path.isdir(os.path.join(RO_ROOT, "Y42调试台_数据"))
print(f"  结论: {'✅ 自动退回可写目录' if ok5 else '❌ 没有正确退回'}")
kill_all()
squatter2.shutdown()
squatter2.server_close()
print("  恢复权限:", sh("icacls", RO_ROOT, "/reset").strip()[:60].splitlines()[-1:] if os.path.isdir(RO_ROOT) else "")
time.sleep(0.6)

# ------------------------------------------------------------------ T6/T7 随机端口下的重复启动
def console_listen_ports():
    """当前调试台进程正在监听的端口集合。"""
    pids = set(y42_pids())
    ports = set()
    for line in sh("netstat", "-ano").splitlines():
        if "LISTENING" not in line:
            continue
        parts = line.split()
        if len(parts) < 5:
            continue
        local, pid = parts[1], parts[-1]
        try:
            pid = int(pid)
        except ValueError:
            continue
        if pid in pids and local.startswith("127.0.0.1:"):
            ports.add(int(local.rsplit(":", 1)[1]))
    return sorted(ports)


print()
print("=" * 70)
print("T6  8787 被占（A 用随机端口）时，再双击另一目录里的副本")
stale = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Y42Console", "server.json")
if os.path.exists(stale):
    os.remove(stale)
squatter3 = ThreadingHTTPServer(("127.0.0.1", 8787), NotConsole)
threading.Thread(target=squatter3.serve_forever, daemon=True).start()
print("  假程序占住 8787")

run_exe(dl_exe, ["--no-browser"])          # A：下载文件夹那份
time.sleep(6)
a_log = tail_log(DL, 8)
a_port = None
for line in a_log:
    if "改用端口" in line:
        a_port = int(line.rsplit("端口", 1)[1].strip())
print(f"  A 落在随机端口 {a_port}；当前调试台监听端口: {console_listen_ports()}")

run_exe(cn_exe, ["--no-browser"])          # B：中文目录那份（不同路径 → 单实例锁挡不住）
time.sleep(5)
b_log = tail_log(CN_DIR, 4)
pids_after = y42_pids()
ports_after = console_listen_ports()
deferred = any("检测到调试台已在运行" in l for l in b_log)
print("  B 的日志:")
for line in b_log:
    print("     ", line)
ok6 = deferred and ports_after == [a_port]
print(f"  监听端口（应仍只有 A 那一个）: {ports_after}")
print(f"  结论: {'✅ B 认出了已在跑的 A，没有起第二个实例' if ok6 else '❌ 起了第二个实例'}")
kill_all()
squatter3.shutdown()
squatter3.server_close()
time.sleep(0.8)

# ------------------------------------------------------------------ T7 同一份文件重复双击（随机端口）
print()
print("=" * 70)
print("T7  同一份文件、在随机端口上重复双击（单实例锁路径）")
squatter4 = ThreadingHTTPServer(("127.0.0.1", 8787), NotConsole)
threading.Thread(target=squatter4.serve_forever, daemon=True).start()
run_exe(dl_exe, ["--no-browser"])
time.sleep(6)
a2_log = tail_log(DL, 8)
a2_port = None
for line in a2_log:
    if "改用端口" in line:
        a2_port = int(line.rsplit("端口", 1)[1].strip())
mark = len(open(os.path.join(DL, "Y42调试台_数据", "Y42调试台_日志.txt"), encoding="utf-8", errors="replace").read().splitlines())
proc_b2 = run_exe(dl_exe, ["--no-browser"], wait_exit=40)
new_lines = open(os.path.join(DL, "Y42调试台_数据", "Y42调试台_日志.txt"), encoding="utf-8", errors="replace").read().splitlines()[mark:]
print(f"  A 的端口 {a2_port}；第二次双击新增日志:")
for line in new_lines:
    print("     ", line)
ok7 = any(f":{a2_port}/" in l for l in new_lines) and console_listen_ports() == [a2_port]
print(f"  监听端口: {console_listen_ports()}")
print(f"  结论: {'✅ 认出了自己已经在跑，并给出正确地址' if ok7 else '❌ 地址不对或起了第二个实例'}")
kill_all()
squatter4.shutdown()
squatter4.server_close()
time.sleep(0.8)

# ------------------------------------------------------------------ T8 残留登记清理
print()
print("=" * 70)
print("T8  上次被强杀留下的端口登记，不该拖慢以后的每次启动")
shared = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Y42Console")
os.makedirs(shared, exist_ok=True)
with open(os.path.join(shared, "server.json"), "w", encoding="utf-8") as fh:
    json.dump({"port": 59999, "pid": 999999, "started": 0}, fh)
print("  已伪造一条指向 59999 / 死进程的残留登记")
cn_log = os.path.join(CN_DIR, "Y42调试台_数据", "Y42调试台_日志.txt")
mark8 = len(open(cn_log, encoding="utf-8", errors="replace").read().splitlines()) if os.path.exists(cn_log) else 0
run_exe(cn_exe, ["--no-browser"])
tb, tr, _ = wait_real_page(8787)
print(f"  真实页 {tr:.0f} ms（占位页 {tb:.0f} ms）" if tr else "  失败：没等到真实页面")
lines8 = open(cn_log, encoding="utf-8", errors="replace").read().splitlines()[mark8:]
cleaned = any("清理上次遗留的端口登记" in l for l in lines8)
for line in lines8[:6]:
    print("     ", line)
print(f"  结论: {'✅ 识别并清掉了残留登记' if cleaned else '❌ 没有清理残留登记'}")
kill_all()
time.sleep(0.8)

# ------------------------------------------------------------------ 收尾
print()
print("=" * 70)
print("收尾：清理测试目录")
kill_all()
shutil.rmtree(RO_ROOT, ignore_errors=True)
shutil.rmtree(CN_ROOT, ignore_errors=True)
print("  已删除:", RO_ROOT, "|", CN_ROOT)
print("  剩余 Y42 进程:", y42_pids() or "无")
print("  8787:", "占用中" if port_listening(8787) else "空闲")

