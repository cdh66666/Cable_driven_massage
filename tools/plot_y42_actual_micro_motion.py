import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parents[1]
run=root/'tools/manual_test_runs'/ (sys.argv[1] if len(sys.argv)>1 else '20260906_193209_749_position_microprobe_worker_3eb8b245')
rows=[json.loads(line) for line in (run/'events.jsonl').read_text(encoding='utf8').splitlines()]
commands=[r for r in rows if r['event']=='position_probe_command']
if not commands:
    raise SystemExit('本轮没有已完成的运动指令，不能绘制运动成果图')
result=next((r['result'] for r in reversed(rows) if r['event']=='position_microprobe_adapter_result'),{})
completed={r['id'] for r in result.get('axis_results',[])}
start=commands[0]['host_time']
q0=commands[0]['targets_deg']
records={i:{} for i in range(1,5)}
for r in rows:
    if r['event']=='position_microprobe_raw_record' and r['sampled_at']>=start:
        records[r['id']][r['sampled_at']]=r['position']
plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(2,2,figsize=(13,7.5),sharex=True,sharey=True)
fig.suptitle('实机已经进入运动：0.5°目标与编码器反馈',fontsize=18)
for i,ax in enumerate(axes.flat,1):
    ax.step([r['host_time']-start for r in commands],[r['targets_deg'][i-1]-q0[i-1] for r in commands],where='post',color='#267bd1',label='实际发送目标')
    points=sorted(records[i].items())
    ax.plot([t-start for t,p in points],[p-q0[i-1] for t,p in points],'.-',markersize=3,color='#d87528',label='原始编码器反馈')
    ax.set_title(f'M{i} · '+('往返完成' if i in completed else '本轴往返未完成'))
    ax.set_xlabel('运动阶段时间（秒）');ax.set_ylabel('相对角度（°）')
    ax.set_ylim(-.25,.7);ax.grid(alpha=.18)
axes[0,0].legend()
cadence=result.get('cadence',{})
hz=cadence.get('measured_command_hz')
status='全流程通过' if result.get('success') else '本轮未通过'
fig.text(.5,.025,f"{len(commands)}次四轴目标 · {hz:.2f}Hz · {status}。原始编码器图，非球心XY实测；无平滑。" if hz else f'{len(commands)}次四轴目标 · {status}；原始编码器图，非球心XY实测。',ha='center',fontsize=11)
fig.tight_layout(rect=(0,.065,1,.94))
target=root/'qa/twin'/f'{run.name}-motion.png'
fig.savefig(target,dpi=160)
print(target)
