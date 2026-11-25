import json,hashlib
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from y42_drawing_archive import drawing_from_events

root=Path(__file__).resolve().parents[1]
paths=sorted((root/'tools/manual_test_runs').glob('20260904_200*trajectory*/events.jsonl'))
plt.rcParams['font.sans-serif']=['Microsoft YaHei'];plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(1,len(paths),figsize=(16,5.4));result=[]
for ax,p in zip(axes,paths):
    raw=p.read_bytes();events=[json.loads(x) for x in raw.splitlines() if x]
    d=drawing_from_events(events);d.update(run_id=p.parent.name,events_sha256=hashlib.sha256(raw).hexdigest())
    target=d['target'];feedback=d['feedback']
    ax.plot(*zip(*target),color='#287fce',label='原始绘制路径',lw=2)
    if feedback:ax.plot(*zip(*(r['xy'] for r in feedback)),'.--',color='#db7134',label=f'历史反馈反算（{len(feedback)}组）',lw=1,ms=7)
    ax.set(title=p.parent.name[9:15],xlabel='X / mm',ylabel='Y / mm',aspect='equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    result.append(d)
fig.suptitle('绘制轨迹对照：原图形与历史设备反馈反算',fontsize=16)
fig.text(.5,.025,'历史记录，非本轮新实测。旧标定与零偏航近似未经外部验证；橙色点间虚线只帮助辨认顺序，不代表连续测量。',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.07,1,.92));out=root/'qa/twin'
fig.savefig(out/'historical-drawing-comparison.png',dpi=160)
(out/'historical-drawing-comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print([(r['run_id'],len(r['feedback'])) for r in result])
