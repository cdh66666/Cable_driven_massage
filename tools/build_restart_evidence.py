"""Freeze existing code/geometry and plot recorded microprobes, without hardware IO.

Run with D:/codex/.cad_env/Scripts/python.exe. Creates a new timestamped bundle.
Original archives and controller configuration are never modified.
"""
import csv
import hashlib
import html
import json
import shutil
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

ROOT = Path(__file__).resolve().parents[1]
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(run, destination):
    events = [json.loads(line) for line in (run / 'events.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8'))
    directions = manifest['state_before'].get('calibration', {}).get('encoder_positive_direction')
    # This fixed probe explicitly requires this mapping; reject other mappings.
    if directions is None:
        candidates = [v for v in manifest.values() if isinstance(v, dict) and 'encoder_positive_direction' in v]
        directions = candidates[0]['encoder_positive_direction'] if candidates else None
    if directions != [0, 0, 1, 1]:
        raise ValueError(f'{run.name}: missing or unsupported archived encoder sign mapping: {directions}')
    tx, feedback, seen = [], [], set()
    for e in events:
        if e.get('event') == 'position_probe_serial_write' and e.get('phase') == 'microprobe_command_stream':
            b = bytes.fromhex(e['attempted_tx_hex'])
            if e.get('error') or e.get('bytes_written') != len(b):
                continue
            if len(b) != 61 or b[:4] != bytes.fromhex('00aa003d') or b[-1] != 107:
                raise ValueError('unexpected probe frame')
            values = []
            for i in range(4):
                part = b[4+14*i:18+14*i]
                assert part[0] == i+1 and part[1] == 203 and part[-1] == 107
                values.append((1 if part[2] == directions[i] else -1)*int.from_bytes(part[5:9], 'big')/10)
            tx.append((e['started_monotonic_ns']/1e9, values))
        if e.get('event') == 'position_microprobe_raw_record':
            key = (e['id'], e['sampled_at'], e['raw_packet_hex'])
            if key not in seen:
                seen.add(key)
                feedback.append(e)
    ending = next((e for e in reversed(events) if e.get('event') == 'run_end'), {})
    result = ending.get('result') or {}
    gaps = [(b[0]-a[0])*1000 for a,b in zip(tx,tx[1:])]
    stats = dict(run_id=run.name, source_events_sha256=digest(run/'events.jsonl'),
                 successful=result.get('success') is True, command_frames=len(tx), raw_samples=len(feedback),
                 host_hz=(len(tx)-1)/(tx[-1][0]-tx[0][0]) if len(tx)>1 else None,
                 max_gap_ms=max(gaps) if gaps else None,
                 reason=result.get('reason') or result.get('error') or ending.get('error'),
                 final_disabled=result.get('final_disabled_confirmed'), configuration_restored=result.get('configuration_restored'),
                 xy_measured=False)
    out = destination/run.name
    out.mkdir()
    write_json(out/'summary.json', stats)
    with (out/'raw-feedback.csv').open('w', newline='', encoding='utf-8-sig') as f:
        keys=['id','sampled_at','position','rpm','phase_current','temperature','flags','raw_packet_hex']
        writer=csv.DictWriter(f, fieldnames=keys, extrasaction='ignore'); writer.writeheader(); writer.writerows(feedback)
    with (out/'written-targets.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer=csv.writer(f); writer.writerow(['host_monotonic_s','M1_deg','M2_deg','M3_deg','M4_deg'])
        writer.writerows([[t,*q] for t,q in tx])
    fig, axes=plt.subplots(2,2, figsize=(12,7), sharex=True, sharey=True)
    t0=tx[0][0] if tx else (feedback[0]['sampled_at'] if feedback else 0)
    for i,ax in enumerate(axes.flat):
        rows=[r for r in feedback if r['id']==i+1 and (not tx or tx[0][0]-.1<=r['sampled_at']<=tx[-1][0]+.2)]
        baseline=tx[0][1][i] if tx else (rows[0]['position'] if rows else 0)
        if tx: ax.step([t-t0 for t,q in tx], [q[i]-baseline for t,q in tx],where='post',color='#1674c6',label='实际写入目标')
        if rows: ax.scatter([r['sampled_at']-t0 for r in rows],[r['position']-baseline for r in rows],s=9,color='#e05d31',label='原始编码器反馈')
        if not tx: ax.text(.5,.5,'未进入连续运动段',ha='center',transform=ax.transAxes)
        ax.set(title=f'M{i+1}',xlabel='相对首个运动指令 / s',ylabel='相对角度 / °',ylim=(-.4,.9)); ax.grid(alpha=.2)
    if tx or feedback: axes[0,0].legend(fontsize=9)
    fig.suptitle(run.name[:23]+'  实测电机往返记录',fontsize=16)
    fig.text(.5,.01,'原始采样点，不插值；从初始角度增加0.5°再返回。不是球心 XY 实测轨迹。',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.035,1,.95)); fig.savefig(out/'encoder-traces.png',dpi=150); plt.close(fig)
    return stats


def main():
    bundle=ROOT/'restart'/datetime.now().strftime('%Y%m%d_%H%M%S')
    (bundle/'code_snapshot').mkdir(parents=True)
    config=json.loads((ROOT/'tools/y42_pc_calibration.json').read_text(encoding='utf-8'))
    fields=['geometry_version','coordinate_frame','workspace_width','workspace_height','anchor_width','anchor_height',
            'motor_center_coordinates_mm','ball','spool_winding_radius_mm','spool_reference_radius_mm','spool_winding_z_range_mm',
            'ball_center_contact_z_mm','ball_attachment_z_mm','ball_attachment_span_x_mm','ball_attachment_span_y_mm',
            'encoder_positive_direction']
    geometry={k:config[k] for k in fields}
    geometry.update(units='mm unless specified', source='existing CAD-derived constants and saved configuration; no fresh physical measurement',
        motor_order=['左上','右上','右下','左下'], spool_outer_radius_mm=20,
        fixed_cable_exit=False, effective_winding_radius_verified=False,
        note='12.5mm为上缘参考半径，不是固定出绳孔。110×148为约定球心绘图区，并非已验证可达/精度范围。')
    write_json(bundle/'hardware_geometry.json',geometry)
    inventory=[]
    for source in sorted((ROOT/'tools').iterdir()):
        if source.is_file() and source.suffix in ('.py','.cjs','.bat') or source.is_file() and source.name.startswith('requirements'):
            target=bundle/'code_snapshot'/source.name; shutil.copy2(source,target)
            inventory.append(dict(source=str(source),file=str(target.relative_to(bundle)),sha256=digest(target)))
    shutil.copy2(ROOT/'tools/y42_pc_calibration.json',bundle/'legacy_calibration_INVALID.json')
    write_json(bundle/'code_manifest.json',inventory)
    fig,ax=plt.subplots(figsize=(10,7))
    ax.add_patch(Rectangle((0,0),110,148,facecolor='#eef5fc',edgecolor='#3074ae',linestyle='--'))
    for i,(x,y) in enumerate(geometry['motor_center_coordinates_mm']):
        ax.add_patch(Circle((x,y),12,facecolor='#9ab5d4',edgecolor='#304c72'))
        ax.text(x,y+17,f'M{i+1} ({x:g},{y:g})',ha='center',fontsize=10)
    ax.add_patch(Circle((55,74),20,facecolor='#f8d5b1',edgecolor='#c47d38'))
    sx,sy=geometry['ball_attachment_span_x_mm']/2,geometry['ball_attachment_span_y_mm']/2
    for x,y in [(55-sx,74+sy),(55+sx,74+sy),(55+sx,74-sy),(55-sx,74-sy)]: ax.plot(x,y,'o',color='#9c3428')
    ax.text(55,110,'球 Ø40 mm\n连接跨度 30.534 × 22.823 mm',ha='center')
    ax.text(55,40,'球心绘图区 110 × 148 mm\n轴心框 198 × 148 mm',ha='center')
    ax.set(xlim=(-70,180),ylim=(-30,185),xlabel='X / mm',ylabel='Y / mm',title='继承的真实硬件几何参数 · 顶视尺寸图')
    ax.set_aspect('equal');ax.grid(alpha=.15)
    fig.text(.5,.025,'按既有 CAD 提取值绘制；球置中仅作示意。无固定导绳孔，实际切点/排线待验证。',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.045,1,1));fig.savefig(bundle/'hardware-geometry.png',dpi=160);plt.close(fig)
    results=[]
    for run in sorted((ROOT/'tools/manual_test_runs').glob('*position_microprobe_worker*')):
        if (run/'events.jsonl').exists(): results.append(analyze(run,bundle))
    write_json(bundle/'experiment_index.json',results)
    fig,axes=plt.subplots(1,2,figsize=(12,4.8))
    labels=[r['run_id'][9:15] for r in results]
    axes[0].bar(labels,[r['command_frames'] for r in results],color='#417cb6')
    axes[0].set(title='实际连续位置帧数',ylabel='帧数',xlabel='历史运行时间 HHMMSS')
    axes[1].bar(labels,[r['raw_samples'] for r in results],color='#df794c')
    axes[1].set(title='去重原始编码器样本数（四轴合计）',ylabel='样本数',xlabel='历史运行时间 HHMMSS')
    for ax,field in zip(axes,['command_frames','raw_samples']):
        for i,r in enumerate(results):ax.text(i,r[field],str(r[field]),ha='center',va='bottom')
        ax.grid(axis='y',alpha=.2)
    fig.suptitle('五次历史实机尝试的证据变化 · 全部未整轮通过',fontsize=16)
    fig.text(.5,.01,'样本增加代表记录覆盖增加，不代表轨迹精度改善；失败保留，零帧不记为零误差。',ha='center')
    fig.tight_layout(rect=(0,.04,1,.94));fig.savefig(bundle/'experiment-progress.png',dpi=150);plt.close(fig)
    cards=[]
    for r in reversed(results):
        cards.append(f'<section><h2>{html.escape(r["run_id"])}</h2><p>连续运动帧 {r["command_frames"]} · 去重反馈 {r["raw_samples"]} · 整轮未通过</p><p>{html.escape(str(r["reason"]))}</p><img src="{r["run_id"]}/encoder-traces.png"><p><a href="{r["run_id"]}/raw-feedback.csv">原始反馈 CSV</a> · <a href="{r["run_id"]}/written-targets.csv">实际写入目标 CSV</a></p></section>')
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>按摩设备 · 真实证据图册</title><style>body{margin:30px auto;max-width:1200px;padding:0 24px;background:#f2f5fa;color:#203247;font:17px/1.7 "Microsoft YaHei",sans-serif}section{background:white;padding:24px;margin:24px 0;border-radius:16px}h2{font-size:20px;overflow-wrap:anywhere}img{width:100%;height:auto}a{color:#1964a5}.note{border-left:5px solid #d57e31;padding:15px;background:#fff4e5}</style><h1>按摩设备 · 真实证据图册</h1><p class="note">本次重建历史实测基线，没有发起新运动。蓝线是实际写入目标，橙点是编码器反馈。球心XY尚未独立测量；五次微幅测试均未整轮通过。</p><p><a href="../../RESTART.md">精简目标与实施计划</a> · <a href="hardware_geometry.json">硬件几何参数</a> · <a href="code_manifest.json">代码快照索引与SHA256</a></p><section><h2>硬件几何基线</h2><img src="hardware-geometry.png"></section><section><h2>历史尝试变化</h2><img src="experiment-progress.png"></section>'''+''.join(cards)+'</html>'
    (bundle/'index.html').write_text(page,encoding='utf-8')
    shutil.copy2(ROOT/'RESTART.md',bundle/'PLAN.md')
    print(bundle)


if __name__=='__main__': main()
