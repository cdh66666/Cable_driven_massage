# -*- coding: utf-8 -*-
"""把 Y42 电脑直连调试台打成单文件 Windows EXE。

产出：仓库根目录下 ``Y42调试台.exe`` —— 双击即启动本地网页调试台并自动打开浏览器。

打包思路（与仓库原实现零改动配合）：
  * ``tools/`` 下的源码与页面资源以“数据文件”形式随 EXE 分发，运行时落在一个
    临时资源目录里，是真实文件路径 —— 主程序用 ``Path(__file__)`` 定位自身源码、
    ``y42_four_pages.js``、``y42_real_lesson.html``、``xingmao_path.json``，
    以及若干兄弟模块，只有真实文件才能让这些读取正常工作。
  * PyInstaller 只负责打包解释器 + 标准库 + pyserial；调试台的业务模块在运行时
    从磁盘导入，因此这里用 AST 扫描生成依赖清单，确保运行时用到的标准库都被收进去。
"""

from __future__ import annotations

import ast
import importlib.util
import shutil
import subprocess
import sys
from pathlib import Path

PACKAGING_DIR = Path(__file__).resolve().parent
REPO_ROOT = PACKAGING_DIR.parent
TOOLS_DIR = REPO_ROOT / "tools"
BUILD_DIR = PACKAGING_DIR / "_build"
STAGE_TOOLS = BUILD_DIR / "stage" / "tools"
DIST_DIR = REPO_ROOT / "dist"
APP_NAME = "Y42Console"
FINAL_EXE_NAME = "Y42调试台.exe"

# 随 EXE 分发的资源类型；运行时生成的文件绝不打包
BUNDLED_SUFFIXES = {".py", ".js", ".html", ".json"}
GENERATED_NAMES = {
    "y42_pc_calibration.json",
    "y42_pc_serial.json",
    "y42_pc_process_log.jsonl",
    "server.json",            # 启动器运行期写出的“当前实例端口”，绝不能打包
}
SKIPPED_DIRS = {"__pycache__", "manual_test_runs"}
# 只有调试台真正用到的第三方运行时依赖
EXTRA_HIDDEN_IMPORTS = (
    "serial",
    "serial.tools",
    "serial.tools.list_ports",
    "pystray",
    "pystray._win32",
    "PIL.Image",
    "PIL.ImageDraw",
    "ctypes",          # 启动器用它创建单实例命名互斥体
    "ctypes.wintypes",
    "http.server",
)
EXCLUDED_MODULES = ("numpy", "cv2", "matplotlib", "pandas", "scipy", "tkinter")
ICON_PATH = BUILD_DIR / "app.ico"


def log(message: str) -> None:
    print(f"[build] {message}", flush=True)


def clean_previous() -> None:
    for path in (BUILD_DIR, DIST_DIR):
        if path.exists():
            shutil.rmtree(path, ignore_errors=True)
    BUILD_DIR.mkdir(parents=True, exist_ok=True)


def stage_tools() -> list[Path]:
    """把需要分发的 tools 资源复制到构建暂存目录。"""
    STAGE_TOOLS.mkdir(parents=True, exist_ok=True)
    staged: list[Path] = []
    for source in sorted(TOOLS_DIR.iterdir()):
        if source.is_dir():
            continue
        if source.suffix.lower() not in BUNDLED_SUFFIXES:
            continue
        if source.name in GENERATED_NAMES:
            continue
        if any(part in SKIPPED_DIRS for part in source.parts):
            continue
        destination = STAGE_TOOLS / source.name
        shutil.copy2(source, destination)
        staged.append(destination)
    log(f"暂存 {len(staged)} 个资源文件 -> {STAGE_TOOLS}")
    return staged


def collect_import_roots(staged: list[Path]) -> list[str]:
    """AST 扫描暂存源码，收集所有可导入的顶层模块名。"""
    local_modules = {path.stem for path in staged if path.suffix == ".py"}
    roots: set[str] = set()
    for path in staged:
        if path.suffix != ".py":
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    roots.add(alias.name.split(".", 1)[0])
            elif isinstance(node, ast.ImportFrom):
                if node.level or not node.module:
                    continue
                roots.add(node.module.split(".", 1)[0])

    selected = []
    for name in sorted(roots):
        if name in local_modules or name in {"__future__"}:
            continue
        try:
            if importlib.util.find_spec(name) is None:
                continue
        except (ImportError, ValueError, ModuleNotFoundError):
            continue
        selected.append(name)
    return selected


def write_dependency_stub(modules: list[str]) -> Path:
    stub = BUILD_DIR / "_stdlib_bundle.py"
    lines = [
        '"""构建期生成：仅为让 PyInstaller 收集运行时依赖，无业务逻辑。"""',
        "",
    ]
    lines.extend(f"import {name}  # noqa: F401" for name in modules)
    lines.append("")
    stub.write_text("\n".join(lines), encoding="utf-8")
    log(f"生成依赖清单：{len(modules)} 个模块 -> {stub.name}")
    return stub


def write_icon() -> Path:
    """用托盘同款图案生成 EXE 图标。"""
    if str(PACKAGING_DIR) not in sys.path:
        sys.path.insert(0, str(PACKAGING_DIR))
    from launcher import build_icon_image

    sizes = [(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)]
    build_icon_image().save(ICON_PATH, format="ICO", sizes=sizes)
    log(f"生成图标 -> {ICON_PATH.name}")
    return ICON_PATH


def write_entry_script() -> Path:
    shutil.copy2(PACKAGING_DIR / "launcher.py", BUILD_DIR / "launcher.py")
    entry = BUILD_DIR / "_entry.py"
    entry.write_text(
        "import _stdlib_bundle  # noqa: F401\n"
        "from launcher import main\n"
        "\n"
        "raise SystemExit(main())\n",
        encoding="utf-8",
    )
    return entry


def build(entry: Path) -> Path:
    command = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--clean",
        "--onefile",
        "--windowed",
        "--noupx",
        "--icon",
        str(ICON_PATH),
        "--name",
        APP_NAME,
        "--distpath",
        str(DIST_DIR),
        "--workpath",
        str(BUILD_DIR / "work"),
        "--specpath",
        str(BUILD_DIR),
        "--add-data",
        f"{STAGE_TOOLS};tools",
    ]
    for name in EXTRA_HIDDEN_IMPORTS:
        command += ["--hidden-import", name]
    for name in EXCLUDED_MODULES:
        command += ["--exclude-module", name]
    command.append("--collect-submodules")
    command.append("serial")
    command.append(str(entry))

    log("开始打包（首次编译约 1-3 分钟）…")
    result = subprocess.run(command, cwd=str(BUILD_DIR))
    if result.returncode != 0:
        raise SystemExit(f"PyInstaller 打包失败，退出码 {result.returncode}")

    produced = DIST_DIR / f"{APP_NAME}.exe"
    if not produced.is_file():
        raise SystemExit(f"未找到打包产物：{produced}")
    return produced


def publish(produced: Path) -> Path:
    final = REPO_ROOT / FINAL_EXE_NAME
    shutil.copy2(produced, final)
    size_mb = final.stat().st_size / (1024 * 1024)
    log(f"完成：{final}（{size_mb:.1f} MB）")
    return final


def main() -> int:
    if importlib.util.find_spec("PyInstaller") is None:
        print("缺少 PyInstaller，请先执行：python -m pip install pyinstaller pyserial")
        return 2
    if not TOOLS_DIR.is_dir():
        print(f"找不到 tools 目录：{TOOLS_DIR}")
        return 2

    clean_previous()
    write_icon()
    staged = stage_tools()
    modules = collect_import_roots(staged)
    write_dependency_stub(modules)
    entry = write_entry_script()
    produced = build(entry)
    publish(produced)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
