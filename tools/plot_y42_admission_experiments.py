"""Plot original admission feedback and error timing; never synthesize XY."""
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
paths = sorted((ROOT/'tools/manual_test_runs').glob('20260906_02*/events.jsonl'))
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False
fig, axes = plt.subplots(len(paths), 1, figsize=(12, max(5, 2.35*len(paths))), squeeze=False)
report = []
for path, ax in zip(paths, axes[:, 0]):
    raw = path.read_bytes()
    events = [json.loads(line) for line in raw.splitlines() if line]
    start = events[0]['monotonic_ns']/1e9
    result = next((e['result'] for e in reversed(events) if e['event']=='run_end'), {})
    samples = [e for e in events if e['event'] in ('position_probe_fresh_axis','position_microprobe_raw_record')]
    tails = [e for e in events if e['event']=='position_probe_transaction' and e.get('rx_hex','').endswith('0100ee6b')]
    for address in range(1,5):
        rows = [e for e in samples if e.get('id')==address]
        if rows:
            q0 = rows[0]['position']
            ax.scatter([(e['monotonic_ns']/1e9-start) for e in rows], [e['position']-q0 for e in rows], s=13, label=f'M{address}', alpha=.8)
    for e in tails:
        ax.axvline(e['monotonic_ns']/1e9-start, color='#d23442', ls='--', alpha=.8)
    short = path.parent.name.split('_position')[0]
    ax.set_title(f"{short} · {result.get('reason', '进行中')} · 零电流接管 {'通过' if result.get('takeover_passed', result.get('success')) else '未通过'}", fontsize=10)
    ax.set_ylabel('相对角度 / °'); ax.set_xlabel('运行时间 / s'); ax.grid(alpha=.2)
    ax.set_ylim(-.35,.35); ax.legend(loc='upper right', ncol=4, fontsize=8)
    report.append({'run_id':path.parent.name,'events_sha256':hashlib.sha256(raw).hexdigest(),
        'result':result,'error_transactions':tails,'feedback_samples':len(samples)})
fig.suptitle('真实接管对照：编码器变化与 M1 错误帧时刻', fontsize=16)
fig.text(.5,.008,'点为本轮原始反馈；红虚线为实际收到 01 00 EE 6B。接管检查不等于轨迹通过；无球心或高度测量。', ha='center', fontsize=10)
fig.tight_layout(rect=(0,.022,1,.975))
out=ROOT/'qa/twin'; fig.savefig(out/'admission-experiments.png', dpi=150)
(out/'admission-experiments.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps([{'run':r['run_id'],'reason':r['result'].get('reason'),'disabled':r['result'].get('final_disabled_confirmed'),'restored':r['result'].get('configuration_restored'),'tail_count':len(r['error_transactions'])} for r in report], ensure_ascii=False))
