"""Compare actual logged host serial-write times, without claiming motor feedback."""
import argparse
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('run', type=Path)
args = p.parse_args()
rows = [json.loads(line) for line in (args.run / 'events.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
frames = [r for r in rows if r.get('event') == 'high_rate_tx_frame' and r.get('packet_bytes') == 61 and r.get('writer_thread') == 'y42-trajectory-frame-writer']
all_writer_count=len(frames)
completion=next((r for r in rows if r.get('event')=='trajectory_complete' and isinstance(r.get('frames'),int)),None)
if completion is not None:
    frames=frames[:completion['frames']]
def stats(values):
    ordered = sorted(values)
    if not ordered:
        return None
    return {'n':len(values), 'mean_ms':sum(values)/len(values), 'p50_ms':ordered[len(ordered)//2], 'p95_ms':ordered[min(len(ordered)-1,int(len(ordered)*.95))], 'max_ms':max(ordered)}
starts = [r['started_monotonic_ns'] for r in frames]
gaps = [(b-a)/1e6 for a,b in zip(starts,starts[1:])]
durations = [(r['completed_monotonic_ns']-r['started_monotonic_ns'])/1e6 for r in frames]
report = {'source':str(args.run), 'measurement':'Host serial write start/completion; scheduled motion prefix from trajectory_complete.frames; excludes post-motion holding frames; not driver execution or camera XY measurement', 'frame_count':len(frames), 'all_writer_frames':all_writer_count, 'baud_values':sorted({r['baud'] for r in frames}), 'intervals':stats(gaps), 'writes':stats(durations), 'intervals_over_15ms':sum(v>15 for v in gaps), 'intervals_over_20ms':sum(v>20 for v in gaps), 'gaps_ms':gaps}
report['average_hz']=(len(starts)-1)*1e9/(starts[-1]-starts[0]) if len(starts)>1 else None
(args.run/'host_timing_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='gaps_ms'},ensure_ascii=False))
