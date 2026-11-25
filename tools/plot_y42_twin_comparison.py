"""Save explicit model-only comparison and new real-run diagnostic summaries."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from y42_twin_model import simulate
from diagnose_twin_raw_frames import analyze

root=Path(__file__).resolve().parents[1];out=root/'qa/twin'
plt.rcParams['font.sans-serif']=['Microsoft YaHei'];plt.rcParams['axes.unicode_minus']=False
cases=[('圆形 · 10 mm/s',{'shape':'circle'}),('圆形 · 50 mm/s',{'shape':'circle','parameters':{'speed':50}}),('蛇形 · 目标抬高5 mm',{'shape':'snake','parameters':{'height':5}})]
fig,ax=plt.subplots(2,3,figsize=(15,8));metrics=[]
for i,(name,params) in enumerate(cases):
    r=simulate(params);f=r['frames'];metrics.append({'case':name,'metrics':r['metrics'],'parameters':r['parameters']})
    ax[0,i].plot([p[0] for p in r['path']],[p[1] for p in r['path']],color='#2477c9',label='目标')
    ax[0,i].plot([p['pose'][0] for p in f],[p['pose'][1] for p in f],color='#159080',label='模型结果')
    ax[0,i].set(xlim=(0,110),ylim=(0,148),xlabel='X / mm',ylabel='Y / mm',title=name);ax[0,i].set_aspect('equal');ax[0,i].legend()
    ax[1,i].plot([p['t'] for p in f],[p['target'][2] for p in f],color='#2477c9',label='目标z')
    ax[1,i].plot([p['t'] for p in f],[p['pose'][2] for p in f],color='#159080',label='模型z')
    ax[1,i].set(xlabel='时间 / s',ylabel='高度z / mm',ylim=(24,31),title=f"仿真XY RMS {r['metrics']['xy_rms_mm']:.2f} mm");ax[1,i].legend()
    for a in ax[:,i]:a.grid(alpha=.2)
fig.suptitle('物理假设下的轨迹变化 · 全部为仿真，不是实机精度',fontsize=18)
fig.text(.5,.012,'同一几何；速度增加可见滞后增大。抬高目标无法获得向上绳力，球留在z=25接触面。质量/弹性/阻尼待实测辨识。',ha='center')
fig.tight_layout(rect=(0,.035,1,.95));fig.savefig(out/'simulation-comparison.png',dpi=160)
(out/'simulation-comparison.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding='utf-8')
runs=['20260905_230801_088_position_microprobe_worker_317e0355','20260906_005510_724_position_microprobe_worker_2a0283cb','20260906_010812_880_position_microprobe_worker_afb9e7ec']
(out/'real-run-diagnostics.json').write_text(json.dumps([analyze(root/'tools/manual_test_runs'/r) for r in runs],ensure_ascii=False,indent=2),encoding='utf-8')
print(out/'simulation-comparison.png')
