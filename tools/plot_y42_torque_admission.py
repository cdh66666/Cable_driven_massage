"""Plot actual C5 admission feedback; explicitly not a Cartesian trajectory."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
RUN='20260907_004824_326_torque_admission_worker_d5d04b3c'
events=[json.loads(line) for line in (ROOT/'tools/manual_test_runs'/RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
rows=sorted([e for e in events if e.get('event') in
             ('position_probe_fresh_axis','position_microprobe_raw_record') and 'sampled_at' in e],
            key=lambda row:row['sampled_at'])
start=min(r['sampled_at'] for r in rows)
plt.rcParams['font.sans-serif']=['Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True,constrained_layout=True)
for i,color in enumerate(['#2477c9','#159080','#df783c','#8d61ad'],1):
    data=[r for r in rows if r['id']==i]
    t=[r['sampled_at']-start for r in data]
    for ax,key in zip(axes,['position','rpm','phase_current']):
        base=data[0][key] if key=='position' else 0
        ax.plot(t,[r[key]-base for r in data],'.',markersize=2,color=color,label=f'M{i}')
for ax,label in zip(axes,['相对初始角度 / °','转速 / RPM','相电流 / mA']):
    ax.set_ylabel(label);ax.grid(alpha=.2)
axes[0].legend(ncol=4);axes[-1].set_xlabel('时间 / s')
fig.suptitle('实机四轴低力矩接管检查 · 全部通过\n20 mA / 1 RPM 短时接管与恢复；尚未执行四角校准或球心轨迹',fontsize=15)
out=ROOT/'qa/twin/torque-admission-20260907.png'
fig.savefig(out,dpi=150)
print(out)
