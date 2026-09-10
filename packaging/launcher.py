# -*- coding: utf-8 -*-
"""Y42 四点绳驱调试台 —— 单文件 EXE 启动器（无控制台窗口 / 托盘常驻）。

职责只有四件事：

1. 定位内置的调试台主程序 ``tools/y42_pc_direct_web.py``（随 EXE 一起分发）；
2. 把“会被写入”的文件（校准参数、过程日志、串口设置、手工测试记录）
   重定向到 EXE 同级的可写数据目录 —— 内置资源目录是临时目录，退出即删；
3. 启动本地网页调试台，自动打开浏览器 http://127.0.0.1:8787/；
4. 在系统托盘常驻一个图标，用于再次打开页面或退出程序（不弹黑框）。

除此之外不修改任何原有控制逻辑：四页调试台（改 ID / 连接 / 校准 / 绘制）、
安全门、急停、协议帧全部沿用仓库原实现。
"""

from __future__ import annotations

import hashlib
import importlib
import json
import os
import socket
import sys
import threading
import time
import traceback
import webbrowser
from pathlib import Path

APP_TITLE = "Y42 四点绳驱调试台"
DATA_DIR_NAME = "Y42调试台_数据"
SHARED_DIR_NAME = "Y42Console"      # %LOCALAPPDATA% 下所有副本共用的登记目录
LOG_NAME = "Y42调试台_日志.txt"
SERVER_INFO_NAME = "server.json"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8787
ENTRY_MODULE = "y42_pc_direct_web"

# 主程序内“会写盘”的模块级常量 -> 落盘文件名（重定向到可写数据目录）
PERSISTENT_FILES = {
    "CALIBRATION_FILE": "y42_pc_calibration.json",
    "PROCESS_LOG_FILE": "y42_pc_process_log.jsonl",
    "SERIAL_SETTINGS_FILE": "y42_pc_serial.json",
    "MANUAL_TEST_RUNS_ROOT": "manual_test_runs",
}
PERSISTED_STATE_LOADERS = ("load_calibration", "load_saved_baud", "load_last_stability")

LOG_PATH: Path | None = None
SERVER = None
LOG_MAX_BYTES = 1_000_000


# ---------------------------------------------------------------- 输出 / 报错

def has_console() -> bool:
    return sys.stdout is not None and sys.stderr is not None


def log(message: str) -> None:
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {message}"
    if has_console():
        try:
            print(line, flush=True)
        except Exception:
            pass
    if LOG_PATH is not None:
        try:
            if LOG_PATH.exists() and LOG_PATH.stat().st_size > LOG_MAX_BYTES:
                LOG_PATH.write_text("", encoding="utf-8")
            with LOG_PATH.open("a", encoding="utf-8") as handle:
                handle.write(line + "\n")
        except OSError:
            pass


def message_box(message: str, title: str = APP_TITLE, *, error: bool = False) -> None:
    """无控制台时唯一的反馈通道。"""
    if has_console():
        log(message)
    try:
        import ctypes

        flags = 0x00000010 if error else 0x00000040  # MB_ICONERROR / MB_ICONINFORMATION
        ctypes.windll.user32.MessageBoxW(None, str(message), str(title), flags | 0x00000000)
    except Exception:
        pass


# ---------------------------------------------------------------- 路径解析

def is_frozen() -> bool:
    return bool(getattr(sys, "frozen", False))


def bundled_tools_dir() -> Path:
    """调试台代码与页面资源所在目录（EXE 内置资源或仓库 tools 目录）。"""
    if is_frozen():
        base = Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent))
        return base / "tools"
    return Path(__file__).resolve().parent.parent / "tools"


def writable_data_dir(tools_dir: Path) -> Path:
    """持久化数据目录：打包后为 EXE 同级目录，源码方式运行时沿用 tools/。"""
    if not is_frozen():
        return tools_dir

    candidates = [Path(sys.executable).resolve().parent / DATA_DIR_NAME]
    local_appdata = os.environ.get("LOCALAPPDATA")
    if local_appdata:
        candidates.append(Path(local_appdata) / "Y42Console")
    for candidate in candidates:
        try:
            candidate.mkdir(parents=True, exist_ok=True)
            probe = candidate / ".write_check"
            probe.write_bytes(b"0")
            probe.unlink()
            return candidate
        except OSError:
            continue
    raise SystemExit("无法创建可写数据目录，调试台未启动。")


# ---------------------------------------------------------------- 主程序装载

def load_console_module(tools_dir: Path):
    """按真实磁盘路径加载调试台主程序。

    必须“从磁盘导入”而不是打进 EXE：主程序在运行时用 ``Path(__file__)``
    定位自身源码与页面资源，只有真实文件路径才能让这些读取正常工作。
    """
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    return importlib.import_module(ENTRY_MODULE)


def redirect_persistent_files(module, target_dir: Path) -> None:
    for attribute, filename in PERSISTENT_FILES.items():
        setattr(module, attribute, target_dir / filename)


def reload_persisted_state(module) -> None:
    for name in PERSISTED_STATE_LOADERS:
        loader = getattr(module, name, None)
        if callable(loader):
            try:
                loader()
            except Exception:
                pass


# ---------------------------------------------------------- 单实例 / 服务信息
#
# 早期版本靠“连一下 8787 端口，连不上就认为没在跑”来判断重复启动。这个做法在本机
# 上会白等约 2 秒：本机（以及不少装了安全软件/防火墙的机器）连一个“没人监听”的
# 本地端口不会立刻收到 RST，而是要等约 2 秒才报连接被拒。启动路径上白等 2 秒，
# 用户看到的就是“双击半天没反应”。
#
# 现在改成两件事，都是毫秒级：
#   1. 用 Windows 命名互斥体判定“是不是已经有一个我在跑”——创建即返回，不做网络 I/O；
#   2. 正在跑的实例把端口写进 数据目录/server.json，第二个实例直接读文件拿地址。
# 端口探测只作为兜底，而且超时压到 0.25 秒。

MUTEX_HANDLE = None
MUTEX_NAME_FALLBACK = "Y42Console_SingleInstance"
ERROR_ALREADY_EXISTS = 183


def single_instance_mutex_name() -> str:
    """互斥体名字带上 EXE 自己的路径，避免不同副本之间互相“劫持”。

    同一份程序重复双击 -> 第二个实例只负责把页面调到前台；
    放在别的目录／别的版本的副本互不影响，不会被老实例拦住。
    """
    try:
        anchor = Path(sys.executable).resolve() if is_frozen() else Path(__file__).resolve()
        digest = hashlib.sha1(str(anchor).lower().encode("utf-8")).hexdigest()[:16]
    except Exception:
        digest = "default"
    return f"Local\\{MUTEX_NAME_FALLBACK}_{digest}"


def acquire_single_instance() -> bool:
    """抢占单实例锁：True=本次是第一个实例，False=已有实例在跑。

    进程退出（包括被强杀）时 Windows 会自动释放句柄，所以不会留下死锁。
    """
    global MUTEX_HANDLE
    try:
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.windll.kernel32
        kernel32.CreateMutexW.restype = wintypes.HANDLE
        kernel32.CreateMutexW.argtypes = (wintypes.LPCVOID, wintypes.BOOL, wintypes.LPCWSTR)
        handle = kernel32.CreateMutexW(None, True, single_instance_mutex_name())
        if not handle:
            return True  # 拿不到锁就照常启动，绝不把用户挡在门外
        MUTEX_HANDLE = handle
        return kernel32.GetLastError() != ERROR_ALREADY_EXISTS
    except Exception:
        log("单实例锁不可用，按独立实例启动：" + traceback.format_exc(limit=1))
        return True


def server_info_path(data_dir: Path) -> Path:
    return data_dir / SERVER_INFO_NAME


def shared_data_dir() -> Path | None:
    """所有副本共用的登记位置（%LOCALAPPDATA%\\Y42Console）。

    单实例锁是按"这一份文件"算的：放在不同目录的两份副本各自都觉得自己是第一份。
    把"我在哪个端口"同时登记到这里，任何副本都能发现"本机已经有一个在跑了"，
    不会出现两个实例同时抢同一个串口的危险情况。
    """
    base = os.environ.get("LOCALAPPDATA")
    if not base:
        return None
    return Path(base) / SHARED_DIR_NAME


def _write_json_atomic(target: Path, payload: dict) -> None:
    try:
        temporary = target.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        os.replace(temporary, target)
    except OSError:
        pass


def write_server_info(data_dir: Path, port: int) -> None:
    payload = {"port": int(port), "pid": os.getpid(), "started": time.time()}
    _write_json_atomic(server_info_path(data_dir), payload)
    shared = shared_data_dir()
    if shared is not None and shared != data_dir:
        try:
            shared.mkdir(parents=True, exist_ok=True)
            _write_json_atomic(server_info_path(shared), payload)
        except OSError:
            pass


def read_server_info(data_dir: Path) -> dict:
    try:
        payload = json.loads(server_info_path(data_dir).read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def clear_server_info(data_dir: Path) -> None:
    for folder in (data_dir, shared_data_dir()):
        if folder is None:
            continue
        recorded = read_server_info(folder)
        if recorded.get("pid") not in (None, os.getpid()):
            continue  # 不是本实例写的，别去动别人的登记
        try:
            server_info_path(folder).unlink(missing_ok=True)
        except OSError:
            pass


def port_open(port: int, timeout: float = 0.25) -> bool:
    """毫秒级判断端口上是否已有服务在监听；超时刻意压得很短，不拖慢启动。"""
    if not port:
        return False
    probe = socket.socket()
    probe.settimeout(timeout)
    try:
        probe.connect((DEFAULT_HOST, int(port)))
        return True
    except OSError:
        return False
    finally:
        try:
            probe.close()
        except OSError:
            pass


def recorded_ports(data_dir: Path) -> list[int]:
    """本副本自己的登记 + 共用登记里的端口（去重、保持顺序）。"""
    return [port for _, port in recorded_entries(data_dir)]


def recorded_entries(data_dir: Path) -> list[tuple[Path, int]]:
    entries: list[tuple[Path, int]] = []
    seen: list[int] = []
    for folder in (data_dir, shared_data_dir()):
        if folder is None:
            continue
        port = read_server_info(folder).get("port")
        if isinstance(port, int) and port > 0 and port not in seen:
            seen.append(port)
            entries.append((folder, port))
    return entries


def process_alive(pid: int) -> bool:
    """那个进程还在不在？判断不了就当它还活着 —— 宁可多等一次，也不误删登记。"""
    if not pid:
        return False
    try:
        import ctypes
        from ctypes import wintypes

        STILL_ACTIVE = 259
        PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
        kernel32 = ctypes.windll.kernel32
        kernel32.OpenProcess.restype = wintypes.HANDLE
        handle = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, int(pid))
        if not handle:
            return False
        try:
            code = wintypes.DWORD()
            if not kernel32.GetExitCodeProcess(handle, ctypes.byref(code)):
                return True
            return code.value == STILL_ACTIVE
        finally:
            kernel32.CloseHandle(handle)
    except Exception:
        return True


def live_console_port(data_dir: Path, timeout: float = 0.25) -> int | None:
    """本机是否已经有一个调试台在跑（可能在别的目录、也可能是随机端口）？

    只按"自己登记过"的端口去问，而且必须同时满足"端口在监听"和
    "它确实是个调试台（/api/auto 有回应）"才算数，绝不会误判到别的程序头上。
    没有任何登记时不做任何网络动作，所以首次启动不会被拖慢。
    """
    entries = recorded_entries(data_dir)
    for _, port in entries:
        if port_open(port, timeout) and y42_console_port(port):
            return port

    # 登记里的端口全都没人应答：那是被强杀的上一次留下的残渣。顺手清掉，
    # 否则以后每次启动都要为它白等一次 0.25 秒的连接超时。只有确认登记的主人
    # 已经不在进程表里才删 —— 端口瞬时没应答、但进程还活着的情况绝不能误删。
    for folder, _ in entries:
        pid = read_server_info(folder).get("pid")
        if isinstance(pid, int) and pid != os.getpid() and not process_alive(pid):
            try:
                server_info_path(folder).unlink(missing_ok=True)
                log(f"清理上次遗留的端口登记：{server_info_path(folder)}")
            except OSError:
                pass
    return None


def discover_running_url(data_dir: Path, fallback_port: int, timeout: float = 10.0) -> str:
    """已有实例在跑时，尽快拿到它的地址：优先读登记，其次试默认端口。"""
    deadline = time.monotonic() + timeout
    while True:
        candidates = recorded_ports(data_dir)
        if fallback_port not in candidates:
            candidates.append(fallback_port)
        for port in candidates:
            if port_open(port, 0.25):
                return f"http://{DEFAULT_HOST}:{port}/"
        if time.monotonic() >= deadline:
            return f"http://{DEFAULT_HOST}:{fallback_port}/"
        time.sleep(0.12)


def make_http_server(port: int, handler):
    """独占端口的 HTTP 服务器。

    Windows 上 SO_REUSEADDR（Python http.server 默认打开）会允许第二个进程绑定
    同一个端口 —— 于是“端口被占就换端口”的判断永远不会触发，两个实例抢同一个
    端口、页面时好时坏。这里改用 SO_EXCLUSIVEADDRUSE：端口被占时立刻返回
    WSAEADDRINUSE，才能正确回退到随机端口。
    """
    from http.server import ThreadingHTTPServer

    class ConsoleHTTPServer(ThreadingHTTPServer):
        allow_reuse_address = False
        daemon_threads = True

        def server_bind(self) -> None:
            if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                try:
                    self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                except OSError:
                    pass
            super().server_bind()

    return ConsoleHTTPServer((DEFAULT_HOST, port), handler)


def y42_console_port(port: int, timeout: float = 0.8) -> bool:
    """端口上是不是已经有一个调试台在跑？用 /api/auto 快速问一句。

    只会在“绑定失败（端口被占）”之后调用 —— 目标端口本来就在监听，所以这次
    HTTP 探测是毫秒级的，不会碰到本机连空闲端口要等约 2 秒的坑。
    """
    import urllib.request

    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(f"http://{DEFAULT_HOST}:{int(port)}/api/auto", timeout=timeout) as response:
            payload = json.loads(response.read().decode("utf-8", "replace"))
    except Exception:
        return False
    return isinstance(payload, dict) and "auto" in payload


# ---------------------------------------------------------------- 托盘

def build_icon_image():
    """托盘与 EXE 图标：四角圆点代表四台电机，中心小球代表笔尖。"""
    from PIL import Image, ImageDraw

    size = 256
    image = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    draw.rounded_rectangle([6, 6, size - 7, size - 7], radius=56, fill=(20, 126, 87, 255))
    dot = 20
    for cx, cy in ((58, 58), (198, 58), (198, 198), (58, 198)):
        draw.ellipse([cx - dot, cy - dot, cx + dot, cy + dot], fill=(255, 255, 255, 255))
    draw.ellipse([98, 98, 158, 158], fill=(255, 255, 255, 255))
    draw.ellipse([110, 110, 146, 146], fill=(20, 126, 87, 255))
    return image


def open_page(url: str, wait: float = 0.0) -> bool:
    """打开浏览器，返回是否真的拉起来了。

    ``wait > 0`` 表示"必须等打开动作执行完才允许返回"。这个参数用在
    “已有实例在运行 → 本次进程立刻结束”的路径上：那种路径下如果只是把打开动作
    丢给一个后台线程，解释器退出会把还没执行完的请求直接丢弃，用户看到的就是
    “双击了却什么都没发生”。把程序挪到别的目录后双击像“打不开”，原因就在这里。
    """
    outcome = {"ok": False}

    def worker() -> None:
        try:
            outcome["ok"] = bool(webbrowser.open(url))
        except Exception:
            log("打开浏览器失败：" + traceback.format_exc(limit=1))

    thread = threading.Thread(target=worker, name="y42-open-browser", daemon=True)
    thread.start()
    if wait > 0:
        thread.join(wait)
    return outcome["ok"]


def open_page_or_warn(url: str, note: str) -> None:
    """打开页面；万一浏览器起不来，也要弹框把地址交给用户，别让双击毫无反馈。"""
    if open_page(url, wait=8.0):
        return
    log("浏览器未能自动打开，改为弹窗提示地址：" + url)
    message_box(f"{note}\n\n请在浏览器中打开：\n{url}", "Y42 调试台访问地址")


def run_tray(url: str, data_dir: Path, on_exit) -> bool:
    """托盘常驻；返回 False 表示托盘不可用（调用方需自行阻塞）。"""
    try:
        import pystray
    except Exception:
        log("托盘组件不可用，改为静默常驻：" + traceback.format_exc(limit=1))
        return False

    def show_address(icon=None, item=None) -> None:
        message_box(f"调试台地址：{url}\n\n数据目录：{data_dir}", "Y42 调试台访问地址")

    def open_folder(icon=None, item=None) -> None:
        try:
            if is_frozen():
                os.startfile(str(data_dir))  # noqa: S606 - 打开自己的数据目录
            else:
                import subprocess

                subprocess.Popen(["explorer", str(data_dir)])
        except Exception:
            log("打开数据目录失败：" + traceback.format_exc(limit=1))

    def quit_app(icon=None, item=None) -> None:
        log("托盘退出：正在停止调试台")
        try:
            on_exit()
        finally:
            try:
                icon.stop()
            except Exception:
                pass

    menu = pystray.Menu(
        pystray.MenuItem("打开调试台", lambda *_: open_page(url), default=True),
        pystray.MenuItem("显示访问地址", show_address),
        pystray.MenuItem("打开数据目录", open_folder),
        pystray.Menu.SEPARATOR,
        pystray.MenuItem("退出调试台", quit_app),
    )
    icon = pystray.Icon("y42_console", build_icon_image(), f"{APP_TITLE} · {url}", menu)
    try:
        icon.run()
    except Exception:
        log("托盘启动失败：" + traceback.format_exc())
        return False
    return True


# ---------------------------------------------------------------- 启动占位页
#
# 单文件 EXE 每次启动都要先自解包（约 1 秒），启动器再装载 2 万多行的业务模块
# （约 0.7 秒）。这段时间里用户什么都看不到，体感就是“双击了没反应”。
# 所以先绑好端口、立刻把浏览器点亮，给一个“正在启动…”的占位页；业务模块装好后
# 再把请求处理类换成真正的调试台页面。用户在大约 1 秒就能看到界面，而不是 4 秒。

PLACEHOLDER_PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta http-equiv="refresh" content="1">
<title>Y42 四点绳驱调试台 · 正在启动</title>
<style>
 html,body{height:100%;margin:0}
 body{display:flex;align-items:center;justify-content:center;background:#f5f6f8;
      color:#1f2430;font-family:"Microsoft YaHei","PingFang SC",system-ui,sans-serif}
 .card{text-align:center;padding:38px 56px;border-radius:14px;background:#fff;
       box-shadow:0 8px 30px rgba(20,40,80,.10)}
 h1{margin:0 0 12px;font-size:19px;font-weight:600}
 p{margin:0;color:#5c6577;font-size:14px}
 .dot{display:inline-block;width:9px;height:9px;margin-right:8px;border-radius:50%;
      background:#147e57;animation:blink 1s infinite}
 @keyframes blink{0%,100%{opacity:1}50%{opacity:.25}}
</style></head>
<body><div class="card">
<h1>Y42 四点绳驱调试台</h1>
<p><span class="dot"></span>正在启动，页面会自动刷新…</p>
</div></body></html>
"""

ERROR_PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>Y42 调试台 · 启动失败</title>
<style>
 html,body{height:100%;margin:0}
 body{display:flex;align-items:center;justify-content:center;background:#fdf3f3;
      color:#3a1f22;font-family:"Microsoft YaHei",system-ui,sans-serif}
 .card{max-width:860px;padding:32px 40px;border-radius:14px;background:#fff;
       box-shadow:0 8px 30px rgba(120,20,20,.12)}
 h1{margin:0 0 14px;font-size:19px;color:#b3261e}
 pre{margin:0;padding:14px;border-radius:8px;background:#f7f7f9;overflow:auto;
     font-size:12px;line-height:1.6;max-height:52vh}
</style></head>
<body><div class="card">
<h1>调试台主程序加载失败</h1>
<pre>{detail}</pre>
</div></body></html>
"""


def boot_error_page(detail: str) -> str:
    from html import escape

    return ERROR_PAGE.replace("{detail}", escape(str(detail)))


def make_boot_handler(page_provider):
    """生成“占位页”请求处理器；业务模块装好后会被真正的 Handler 替换掉。"""
    from http.server import BaseHTTPRequestHandler

    class BootHandler(BaseHTTPRequestHandler):
        server_version = "Y42Boot/1.0"

        def _send(self) -> None:
            body = str(page_provider()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store, must-revalidate")
            self.end_headers()
            try:
                self.wfile.write(body)
            except OSError:
                pass

        def do_GET(self) -> None:  # noqa: N802
            self._send()

        def do_POST(self) -> None:  # noqa: N802
            self._send()

        def log_message(self, *args) -> None:  # 启动噪声不进日志
            pass

    return BootHandler


# ---------------------------------------------------------------- 启动

def parse_arguments(argv: list[str]) -> tuple[int, bool]:
    port = DEFAULT_PORT
    open_browser = True
    index = 0
    while index < len(argv):
        argument = argv[index]
        if argument == "--port" and index + 1 < len(argv):
            port = int(argv[index + 1])
            index += 2
            continue
        if argument.startswith("--port="):
            port = int(argument.split("=", 1)[1])
            index += 1
            continue
        if argument == "--no-browser":
            open_browser = False
            index += 1
            continue
        index += 1
    return port, open_browser


def main(argv: list[str] | None = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            if stream is not None:
                stream.reconfigure(errors="replace")  # type: ignore[union-attr]
        except Exception:
            pass

    arguments = list(sys.argv[1:] if argv is None else argv)
    try:
        requested_port, open_browser = parse_arguments(arguments)
    except ValueError:
        requested_port, open_browser = DEFAULT_PORT, True

    started_at = time.perf_counter()

    tools_dir = bundled_tools_dir()
    if not (tools_dir / f"{ENTRY_MODULE}.py").is_file():
        message_box(f"找不到调试台主程序：\n{tools_dir}", error=True)
        return 2

    try:
        data_dir = writable_data_dir(tools_dir)
    except SystemExit as exc:
        message_box(str(exc), error=True)
        return 2
    globals()["LOG_PATH"] = data_dir / LOG_NAME

    log("=" * 56)
    log(f"{APP_TITLE} 启动")
    log(f"程序目录：{tools_dir}")
    log(f"数据目录：{data_dir}")

    # ---- 单实例判定：毫秒级，不再用“连端口等超时”的方式白等 ----
    if not acquire_single_instance():
        url = discover_running_url(data_dir, requested_port)
        log(f"调试台已在运行，打开页面：{url}")
        if open_browser:
            open_page_or_warn(url, "检测到调试台已经在运行（同一份程序已开了一个）。")
        return 0

    boot_state = {"error": ""}

    def boot_page() -> str:
        detail = boot_state["error"]
        return boot_error_page(detail) if detail else PLACEHOLDER_PAGE

    # ---- 本机可能已经有一份「别的副本」在跑：放在另一个目录、或者当初默认端口被占而
    # 用了随机端口。单实例锁只认"这一份文件"，挡不住这种情况；而调试台必须只有一份 ——
    # 两个实例会抢同一个串口。所以先按登记过的端口问一句，已经在跑就直接把页面调出来。
    running_port = live_console_port(data_dir)
    if running_port is not None:
        url = f"http://{DEFAULT_HOST}:{running_port}/"
        log(f"检测到调试台已在运行（端口 {running_port}），打开页面：{url}")
        if open_browser:
            open_page_or_warn(url, "检测到本机已经有调试台在运行（可能是放在别处的另一份）。")
        return 0

    # ---- 先把端口占住、把浏览器点亮，再去装载 2 万行的业务模块 ----
    try:
        server = make_http_server(requested_port, make_boot_handler(boot_page))
    except OSError as first_error:
        # 默认端口被别的程序（或一份更早版本的调试台）占着。先快速问一句：是不是
        # 已经有调试台在那儿跑？是的话直接把它的页面打开，不要另起一个抢端口。
        if y42_console_port(requested_port):
            url = f"http://{DEFAULT_HOST}:{requested_port}/"
            log(f"端口 {requested_port} 上已有调试台在运行，打开页面：{url}")
            if open_browser:
                open_page_or_warn(url, f"检测到已经有调试台在运行（占用端口 {requested_port}）。")
            return 0
        try:
            server = make_http_server(0, make_boot_handler(boot_page))
        except OSError:
            log(f"端口监听失败：{first_error}")
            message_box(f"无法监听端口 {requested_port}：\n{first_error}", error=True)
            return 4
        log(f"端口 {requested_port} 被占用，改用端口 {server.server_address[1]}")

    globals()["SERVER"] = server
    port = server.server_address[1]
    url = f"http://{DEFAULT_HOST}:{port}/"
    write_server_info(data_dir, port)
    threading.Thread(target=server.serve_forever, name="y42-http", daemon=True).start()
    log(f"启动页已就绪：{url}（{time.perf_counter() - started_at:.2f}s）")

    if open_browser:
        # 端口此刻已在监听，浏览器马上就能看到“正在启动…”，不用再 sleep 等服务器
        open_page(url)

    console = None
    try:
        console = load_console_module(tools_dir)
        redirect_persistent_files(console, data_dir)
        reload_persisted_state(console)
        server.RequestHandlerClass = console.Handler
        log(f"调试台已就绪：{url}（{time.perf_counter() - started_at:.2f}s）")
    except Exception:
        detail = traceback.format_exc()
        boot_state["error"] = detail
        log("主程序加载失败：\n" + detail)
        message_box("调试台主程序加载失败：\n\n" + detail[-1200:], error=True)

    def shutdown() -> None:
        try:
            SERVER.shutdown()
            SERVER.server_close()
        except Exception:
            pass
        clear_server_info(data_dir)

    if console is not None:
        # 与原始 run() 一致：启动时做一次“非运动”安全准备（扫描 / 固件校验 /
        # 锁按键 / STOP+DISABLE），不会使能或移动电机。
        threading.Thread(
            target=console.auto_prepare_on_boot, name="y42-boot-prepare", daemon=True
        ).start()

    if run_tray(url, data_dir, shutdown):
        log("托盘已退出，调试台停止")
        return 0

    # 托盘不可用时的兜底：静默常驻，直到进程被结束。
    log("无托盘：调试台将在后台运行，需从任务管理器结束进程。")
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
