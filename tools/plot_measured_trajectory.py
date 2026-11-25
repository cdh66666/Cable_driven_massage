"""Plot a completed real-device trajectory against its commanded polyline.

The actual series is the controller's four-encoder feedback passed through the
saved internal geometry inverse. It is explicitly not an external camera pose.
"""
from __future__ import annotations

import json
import math
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "qa" / "twin"
PNG = OUT / "measured-trajectory-comparison.png"
HTML = Path(r"C:\Users\1\.codex\visualizations\2026\09\05\01a07265-d186-76f1-9473-dfe48fb159d1") / "measured-trajectory.html"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run", nargs="?", help="manual_test_runs目录名；省略则选最新完整实机绘制")
    args = parser.parse_args()
    if args.run:
        run = ROOT / "tools" / "manual_test_runs" / args.run
    else:
        candidates = []
        for path in (ROOT / "tools" / "manual_test_runs").glob("*"):
            state_path = path / "final_state.json"
            if not state_path.exists():
                continue
            try:
                state_probe = json.loads(state_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if (len(state_probe.get("trajectory_target_path", [])) >= 2
                    and len(state_probe.get("trajectory_actual_feedback_path", [])) >= 2
                    and "trajectory_worker" in path.name):
                candidates.append(path)
        if not candidates:
            raise RuntimeError("没有找到包含完整目标和实机反馈反解的绘制记录")
        run = max(candidates, key=lambda path: path.stat().st_mtime_ns)
    state = json.loads((run / "final_state.json").read_text(encoding="utf-8"))
    target = state["trajectory_target_path"]
    actual = state["trajectory_actual_feedback_path"]
    if len(target) < 2 or len(actual) < 2:
        raise RuntimeError("需要完整目标轨迹和至少两帧实机反馈反解轨迹")
    plt.rcParams.update({"font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
                         "axes.unicode_minus": False})
    fig, ax = plt.subplots(figsize=(9.2, 8.0), dpi=180, constrained_layout=True)
    tx, ty = zip(*target)
    ax.plot(tx, ty, color="#6b7280", linewidth=1.0, alpha=.72, label="目标手绘轨迹（电脑发送）")
    ax.scatter(tx[::max(1, len(target)//36)], ty[::max(1, len(target)//36)],
               s=10, color="#9ca3af", alpha=.65)
    axx, axy = zip(*actual)
    ax.plot(axx, axy, color="#d94841", linewidth=2.0, marker="o", markersize=3.0,
            alpha=.96, label="实机反馈反解轨迹（四轴编码器）")
    ax.scatter([tx[0], tx[-1]], [ty[0], ty[-1]], color="#374151", s=28, zorder=5)
    ax.scatter([axx[0], axx[-1]], [axy[0], axy[-1]], color="#d94841", s=28, zorder=6)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-2, 112); ax.set_ylim(-2, 150)
    ax.set_xlabel("X / mm"); ax.set_ylabel("Y / mm")
    ax.grid(alpha=.22); ax.legend(loc="best")
    ax.set_title(f"实机手绘轨迹对比\n{state.get('port','?')} · {state.get('baud','?')} · 四轴反馈反解 · 不是仿真")
    ax.text(.02, .02,
            f"反馈点 {len(actual)} / 目标点 {len(target)}\n"
            f"记录终点误差 {state.get('trajectory_fit_error_mm'):.2f} mm\n"
            "实测含义：编码器反馈 + 已保存几何模型反解；无外部视觉绝对基准",
            transform=ax.transAxes, ha="left", va="bottom", fontsize=9,
            bbox={"facecolor": "white", "alpha": .86, "edgecolor": "#d1d5db"})
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(PNG, dpi=180)
    plt.close(fig)

    # Compact inline SVG: preserve the same measured/target data for the user.
    width, height = 760, 560
    def sx(x): return 48 + float(x) / 110 * 680
    def sy(y): return 510 - float(y) / 148 * 450
    def path(points):
        return " ".join(("M" if i == 0 else "L") + f"{sx(x):.2f},{sy(y):.2f}" for i, (x, y) in enumerate(points))
    target_path, actual_path = path(target), path(actual)
    html = f'''<div id="measured-trajectory" style="width:100%;font-family:system-ui,sans-serif;color:var(--foreground);">
<div style="font-size:18px;font-weight:500;margin-bottom:8px;">实机手绘轨迹对比</div>
<svg viewBox="0 0 {width} {height}" role="img" aria-label="目标手绘轨迹与四轴编码器反馈反解轨迹对比" style="width:100%;height:auto;display:block;">
<rect x="48" y="60" width="680" height="450" fill="none" stroke="currentColor" opacity=".28"/>
<path d="{target_path}" fill="none" stroke="#8b929c" stroke-width="1.2" opacity=".8"/>
<path d="{actual_path}" fill="none" stroke="#d94841" stroke-width="2.4"/>
<text x="48" y="40" font-size="13" fill="currentColor">{state.get('port','?')} · {state.get('baud','?')} · 反馈点 {len(actual)} · 目标点 {len(target)} · 终点误差 {state.get('trajectory_fit_error_mm'):.2f} mm</text>
<line x1="48" y1="535" x2="78" y2="535" stroke="#8b929c" stroke-width="2"/><text x="84" y="539" font-size="12" fill="currentColor">目标手绘轨迹</text>
<line x1="205" y1="535" x2="235" y2="535" stroke="#d94841" stroke-width="3"/><text x="241" y="539" font-size="12" fill="currentColor">实机反馈反解</text>
<text x="48" y="555" font-size="11" fill="currentColor">实测含义：四轴编码器反馈 + 已保存几何模型反解；不是外部视觉绝对位置</text>
</svg></div>'''
    HTML.parent.mkdir(parents=True, exist_ok=True)
    HTML.write_text(html, encoding="utf-8")
    print(PNG)
    print(HTML)


if __name__ == "__main__":
    main()
