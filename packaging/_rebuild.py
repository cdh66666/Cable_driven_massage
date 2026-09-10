# -*- coding: utf-8 -*-
"""无清理重建：复用 build_exe 的暂存/图标/依赖收集逻辑，但在全新的目录里打包，
绝不删除任何已有文件（旧 _build / dist 原样保留）。"""

import shutil
import subprocess
import sys
from pathlib import Path

PACK = Path(__file__).resolve().parent
sys.path.insert(0, str(PACK))
import build_exe as b  # noqa: E402

NEW_BUILD = b.PACKAGING_DIR / "_build2"
NEW_STAGE = NEW_BUILD / "stage" / "tools"
NEW_DIST = b.REPO_ROOT / "dist_new"

# 把打包器指向新目录；除了 PyInstaller 自己的输出目录，不产生任何删除动作。
b.BUILD_DIR = NEW_BUILD
b.STAGE_TOOLS = NEW_STAGE
b.DIST_DIR = NEW_DIST
b.ICON_PATH = NEW_BUILD / "app.ico"

NEW_BUILD.mkdir(parents=True, exist_ok=True)
NEW_DIST.mkdir(parents=True, exist_ok=True)

b.write_icon()
staged = b.stage_tools()
modules = b.collect_import_roots(staged)
b.write_dependency_stub(modules)
entry = b.write_entry_script()

command = [
    sys.executable, "-m", "PyInstaller",
    "--noconfirm", "--onefile", "--windowed", "--noupx",
    "--icon", str(b.ICON_PATH),
    "--name", b.APP_NAME,
    "--distpath", str(NEW_DIST),
    "--workpath", str(NEW_BUILD / "work"),
    "--specpath", str(NEW_BUILD),
    "--add-data", f"{NEW_STAGE};tools",
]
for name in b.EXTRA_HIDDEN_IMPORTS:
    command += ["--hidden-import", name]
for name in b.EXCLUDED_MODULES:
    command += ["--exclude-module", name]
command += ["--collect-submodules", "serial", str(entry)]

print("[rebuild] 开始打包（新目录，不做任何清理）…", flush=True)
result = subprocess.run(command, cwd=str(NEW_BUILD))
print("[rebuild] PyInstaller 退出码 =", result.returncode, flush=True)
if result.returncode != 0:
    raise SystemExit(result.returncode)

produced = NEW_DIST / f"{b.APP_NAME}.exe"
if not produced.is_file():
    raise SystemExit(f"未找到产物：{produced}")

final = b.REPO_ROOT / b.FINAL_EXE_NAME
shutil.copy2(produced, final)
print(f"[rebuild] 完成 -> {final}（{final.stat().st_size / 1048576:.1f} MB）", flush=True)
