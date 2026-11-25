"""Compare archived encoder feedback; never label it ball-position metrology."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('runs',nargs='+')
args=parser.parse_args()
plt.rcParams['font.sans-serif']=['Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,len(args.runs),figsize=(7*len(args.runs),8),squeeze=False,constrained_layout=True)
for col,run in enumerate(args.runs):
    events=[json.loads(line) for line in (ROOT/'tools/manual_test_runs'/run/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    rows=[e for e in events if e.get('event')=='position_microprobe_raw_record']
    start=min(e['sampled_at'] for e in rows)
    result=next(e['result'] for e in reversed(events) if e.get('event')=='corner_stop_probe_result')
    for axis,color in enumerate(['#2477c9','#159080','#df783c','#8d61ad'],1):
        data=[r for r in rows if r['id']==axis]
        for row,key in enumerate(['position','rpm','phase_current']):
            base=data[0][key] if key=='position' else 0
            axs[row,col].plot([e['sampled_at']-start for e in data],[e[key]-base for e in data],'.',ms=3,color=color,label=f'M{axis}')
    axs[0,col].set_title(run.split('_corner')[0]+'\n'+(result.get('probe_result') or {}).get('failure_reason',result['reason']),fontsize=11)
    axs[0,col].legend(ncol=4)
    axs[1,col].axhline(1,color='#c53b42',ls='--',lw=1,label='设定 1 RPM')
    for row,label in enumerate(['相对角度 / °','转速 / RPM','相电流 / mA']):
        axs[row,col].set_ylabel(label);axs[row,col].grid(alpha=.2)
    axs[2,col].set_xlabel('运动反馈采集时间 / s')
fig.suptitle('实机寻角试验：编码器 / 转速 / 电流原始反馈\n尚未完成四角校准；此图不是球心 XY 轨迹',fontsize=15)
out=ROOT/'qa/twin/corner-attempt-comparison.png'
fig.savefig(out,dpi=150)
print(out)
