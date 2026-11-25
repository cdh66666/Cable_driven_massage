"""Render saved offline command roundtrips; never access a serial port."""
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'qa/twin'


def render(report):
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    cases = report['cases']
    fig, axes = plt.subplots(2, len(cases), figsize=(14, 8), constrained_layout=True)
    fig.suptitle('原始轨迹 → 电机目标量化 → 同模型反算\n离线软件验收；不是实机小球轨迹', fontsize=17)
    for i, case in enumerate(cases):
        p, q = np.asarray(case['source_xy_mm']), np.asarray(case['decoded_xy_mm'])
        e = np.linalg.norm(q-p, axis=1)
        ax = axes[0, i]
        ax.plot(p[:, 0], p[:, 1], color='#2477c9', lw=3, label='原始折线 / 细分')
        ax.plot(q[:, 0], q[:, 1], color='#d36e28', lw=1.2, ls='--', label='0.1°量化后反算')
        ax.set(xlim=(0,110), ylim=(0,148), title=case['label'], xlabel='X / mm', ylabel='Y / mm')
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(fontsize=8)
        ax = axes[1, i]
        ax.plot(e, color='#d36e28', lw=.8)
        ax.set(xlabel='轨迹采样序号', ylabel='软件往返误差 / mm',
               title=f"最大 {max(e):.5f} mm；未量化 {case['unquantized_max_mm']:.2e} mm")
        ax.grid(alpha=.2)
    fig.savefig(OUT/'model-command-roundtrip.png', dpi=160)
    plt.close(fig)

    # Compact native SVG for the existing page, which reloads HTML on every GET.
    # The data is an explicit offline report, independent of the live feedback.
    svg=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 990 420" role="img" aria-label="离线目标量化反算轨迹" style="width:100%;height:auto;background:#fbfdfe">']
    for i, case in enumerate(cases):
        left=35+i*330
        svg.append(f'<text x="{left}" y="25" font-size="16" fill="#17343d">{html.escape(case["label"])}</text>')
        svg.append(f'<rect x="{left}" y="42" width="220" height="296" fill="none" stroke="#bacdd2"/>')
        for key, color, width, dash in [('source_xy_mm','#2477c9',3,''),('decoded_xy_mm','#d36e28',1.2,'stroke-dasharray="4 3"')]:
            points=' '.join(f'{left+2*x:.3f},{338-2*y:.3f}' for x,y in case[key])
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width}" {dash}/>')
        svg.append(f'<text x="{left}" y="365" font-size="13" fill="#17343d">量化后最大误差 {case["quantized_max_mm"]:.5f} mm</text>')
        svg.append(f'<text x="{left}" y="386" font-size="12" fill="#58747c">未量化 {case["unquantized_max_mm"]:.2e} mm</text>')
    svg.append('</svg>')
    content='\n'.join(svg)
    (OUT/'model-command-roundtrip.svg').write_text(content,encoding='utf-8')
    panel='''<!-- model-roundtrip-report:start -->
<section class="card wide"><h2>软件验收 · 原始轨迹与目标反算</h2>
<p class="muted">蓝：原始折线；橙色虚线：电机目标按 0.1° 量化后，使用同一名义几何模型反算。两线接近重合才说明目标换算自洽；这里没有使用实机反馈。</p>
'''+content+'''<p class="muted">固定平面 z=25 mm、偏航=0；半径12 mm及绕向仍是模型假设。已有标定文件未改写，旧控制器未因此获得新的实机验收结果。</p></section>
<!-- model-roundtrip-report:end -->'''
    page=ROOT/'tools/y42_twin.html'
    text=page.read_text(encoding='utf-8')
    start='<!-- model-roundtrip-report:start -->'; end='<!-- model-roundtrip-report:end -->'
    if start in text:
        before,rest=text.split(start,1); _,after=rest.split(end,1); text=before+panel+after
    else:
        text=text.replace('<section class="card wide"><h2>设备原始反馈', panel+'\n<section class="card wide"><h2>设备原始反馈',1)
    page.write_text(text,encoding='utf-8')


if __name__=='__main__':
    render(json.loads((OUT/'model-command-roundtrip.json').read_text(encoding='utf-8')))
    print(OUT/'model-command-roundtrip.png')
