"""Use saved user input and the actual production conversion, no hardware IO."""
import ast,hashlib,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from y42_handwriting_plan_replay import conversion_environment

root=Path(__file__).resolve().parents[1]
source=root/'tools/y42_pc_direct_web.py'
archive=root/'tools/manual_test_runs/20260904_200343_291_trajectory_worker_f0e8cc3a/events.jsonl'
events=[json.loads(x) for x in archive.read_bytes().splitlines() if x]
spec=next(e for e in events if e['event']=='run_start')['arguments'][1]
raw=spec['operator_input_normalized_segments']
raw=[[dict(x=p[0],y=p[1]) if isinstance(p,list) else p for p in stroke] for stroke in raw]
env=conversion_environment(ast.parse(source.read_text(encoding='utf-8')))
planned,original,count,refined=env['convert'](raw,{})
phases,_,duration,_=env['plan_custom_motion_segments'](original,50,150,exact_vertices=True)
clock_spec=dict(motion_segments=phases,duration_ms=round(duration*1000))
clock_targets=[env['custom_motion_phase'](min(1,index*.01/duration),clock_spec)['xy'] for index in range(round(duration/.01)+1)]
cursor=0
for stroke in original:
    for point in stroke:
        while cursor<len(clock_targets) and clock_targets[cursor]!=point:cursor+=1
        if cursor==len(clock_targets):raise AssertionError('100Hz command missed original vertex')
        cursor+=1
length=lambda stroke:sum(math.dist(a,b) for a,b in zip(stroke,stroke[1:]))
checks=[]
for before,after in zip(original,planned):
    cursor=0
    for point in before:
        while cursor<len(after) and after[cursor]!=point:cursor+=1
        if cursor==len(after):raise AssertionError('lost original vertex')
        cursor+=1
    checks.append(dict(original_points=len(before),planned_points=len(after),all_vertices_exact_and_ordered=True,length_difference_mm=length(after)-length(before)))
plt.rcParams['font.sans-serif']=['Microsoft YaHei'];plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(1,2,figsize=(11,5.2))
for ax in axes:
    for before,after in zip(original,planned):
        ax.plot(*zip(*before),color='#444b52',lw=4,label='保存的原始手绘折线')
        ax.plot(*zip(*after),color='#178aa7',lw=1.6,ls='--',label='修改后规划线：只插点')
        ax.scatter(*zip(*before),s=12,color='#444b52')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('X / mm');ax.set_ylabel('Y / mm')
axes[0].set_title('原始手绘与规划线完整重合');axes[0].legend(fontsize=9)
p=np.asarray(original[0]);i=int(np.argmin(p[:,1]));x,y=p[i];axes[1].set_xlim(x-4,x+4);axes[1].set_ylim(y-2,y+6);axes[1].set_title('原始拐点放大：不圆角、不删点')
fig.suptitle(f'原始手绘保真检查 · {count} 个原始点全部按原顺序保留',fontsize=15)
fig.text(.5,.015,'来自保存的用户笔画。这里只验证数字路径，不代表设备已经无误差走完。',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.05,1,.92));out=root/'qa/twin';fig.savefig(out/'exact-handwriting.png',dpi=170)
report=dict(input_archive=str(archive),input_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),controller_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_points=original,planned_points=planned,checks=checks,command_100hz_targets=clock_targets,all_original_vertices_on_command_clock=True,exact_plan_duration_s=duration,hardware_executed=False)
(out/'exact-handwriting.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(checks))
