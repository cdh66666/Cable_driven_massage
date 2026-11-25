"""Compare two actual motion archives; never synthesize missing samples."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
runs = ['20260906_200637_917_position_microprobe_worker_9c85f471',
        '20260906_200913_695_position_microprobe_worker_95b34b0f']
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, sharey=True)
for ax, run, title in zip(axes, runs, ['修复前：运动往返通过，阶段切换多等一个周期', '修复后：完整基线通过，沿用统一指令时钟']):
    events = [json.loads(s) for s in (root/'tools/manual_test_runs'/run/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    commands = [r for r in events if r['event'] == 'position_microprobe_command_cadence']
    origin = commands[0]['started_at']
    result = next(r['result'] for r in reversed(events) if r['event']=='position_microprobe_adapter_result')
    pairs = [(r['started_at']-origin, r['interval_seconds']*1000) for r in commands if r['interval_seconds'] is not None]
    ax.plot([p[0] for p in pairs], [p[1] for p in pairs], '.-', color='#267bd1', markersize=3, label='真实四轴指令间隔')
    ax.axhline(40, color='#399365', linestyle='--', label='25 Hz 目标：40 ms')
    ax.axhline(60, color='#d16b45', linestyle=':', label='本轮节拍上限：60 ms')
    ax.set_title(title+f" · {result['cadence']['measured_command_hz']:.3f} Hz")
    ax.set_ylabel('间隔（毫秒）')
    ax.set_ylim(35, 75)
    ax.grid(alpha=.15)
axes[0].legend(loc='upper right')
axes[1].set_xlabel('运动阶段时间（秒）')
fig.suptitle('真实设备：节拍修复前后对比', fontsize=18)
fig.tight_layout()
path = root/'qa/twin/com5-polled-cadence-progress.png'
fig.savefig(path, dpi=160)
print(path)
