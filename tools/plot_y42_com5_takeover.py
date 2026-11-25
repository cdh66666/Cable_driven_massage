"""Plot measured encoder admission trace; deliberately not a ball XY claim."""
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'tools/manual_test_runs' / (sys.argv[1] if len(sys.argv)>1 else '20260906_174544_540_position_microprobe_worker_6ca6717c')
events = [json.loads(line) for line in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
start = events[0]['monotonic_ns']/1e9
samples = {i: [] for i in range(1,5)}
seeds = {}
settled = {}
failure = None
for event in events:
    t = event['monotonic_ns']/1e9-start
    if event['event'] == 'position_probe_zero_current_seed':
        seeds[event['id']] = (t, event['q0'])
    if event['event'] == 'position_probe_enable_settled':
        settled[event['id']] = (t, event['settled_q0'])
    if event['event'] == 'position_microprobe_failed':
        failure = t
    if event['event'] != 'position_probe_transaction':
        continue
    tx = bytes.fromhex(event['tx_hex'])
    rx = bytes.fromhex(event['rx_hex'])
    if len(tx) == 3 and tx[1] == 0x36 and len(rx) == 8 and rx[:2] == tx[:2] and rx[-1] == 0x6B and rx[2] in (0,1):
        value = int.from_bytes(rx[3:7], 'big')/10*(-1 if rx[2] else 1)
        samples[tx[0]].append((t,value))
plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus']=False
fig, axes=plt.subplots(2,2,figsize=(13,7.5),sharex=True)
fig.suptitle('COM5 实测：四轴落稳通过，随后通信错误停止' if len(settled)==4 else 'COM5 实机接管反馈：M3 在接管后偏移，保护停止', fontsize=18)
for i,ax in enumerate(axes.flat,1):
    points=samples[i]
    reference=seeds.get(i,(0,points[0][1]))[1]
    ax.plot([p[0] for p in points],[p[1]-reference for p in points],'.-',color='#dc7027',label='实测编码器相对 q0')
    if i in settled:
        t,q=settled[i]
        ax.plot([0,t,t,points[-1][0]],[0,0,q-reference,q-reference],color='#267bd1',linestyle='--',label='参考点：落稳后更新')
        ax.axvline(t,color='#289663',alpha=.7,label='落稳通过')
    else:
        ax.axhline(0,color='#267bd1',linestyle='--',label='接管目标：保持 q0')
    ax.axhspan(-0.2,0.2,color='#267bd1',alpha=.08)
    if i in seeds:
        ax.axvline(seeds[i][0],color='#267bd1',alpha=.7,label='CB 接管指令')
    if failure:
        ax.axvline(failure,color='#bb3344',linestyle=':',label='保护停止')
    ax.set_title(f'M{i} · q0={reference:.1f}°')
    ax.set_ylabel('编码器偏差（°）')
    ax.set_xlabel('本轮时间（秒）')
    ax.grid(alpha=.15)
    ax.set_ylim(-.35,.85)
axes[0,0].legend(fontsize=9,loc='upper left')
fig.text(.5,.015,'原始 0x36 回包；未平滑。此图是电机接管诊断，尚未执行球心 XY 绘制轨迹。四台最终失能、原配置恢复。',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.05,1,.94))
target=ROOT/'qa/twin'/('com5-settling-measured.png' if settled else 'com5-takeover-measured.png')
fig.savefig(target,dpi=160)
print(target)
