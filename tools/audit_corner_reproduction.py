"""Read-only comparison with the exact calibration snapshot used in the run."""
import json,tempfile
from pathlib import Path
from test_y42_calibration_flow import isolated_web_module
ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'tools/manual_test_runs/20260908_143641_606_all_test_patterns_worker_549bed30'
events=[json.loads(l) for l in (RUN/'events.jsonl').read_text(encoding='utf-8').splitlines()]
cal=json.loads((RUN/'manifest.json').read_text(encoding='utf-8'))['calibration_before']
rebase=next(e for e in events if e['event']=='trajectory_start_local_rebase')
out=[]
with tempfile.TemporaryDirectory() as directory:
    w=isolated_web_module(directory)
    slopes=w.effective_slopes(cal)
    for xy,samples in zip(zip(*w.workspace_corner_coordinates(cal)),cal['samples']):
        nominal=[w.motor_degrees(i,*xy,cal) for i in range(4)]
        commanded=[a+b for a,b in zip(nominal,rebase['offsets_deg'])]
        out.append({'xy_mm':xy,'calibrated_angles_deg':samples,'nominal_angles_deg':nominal,
                    'session_command_angles_deg':commanded,
                    'model_minus_calibration_equivalent_mm':[(a-b)/s for a,b,s in zip(nominal,samples,slopes)],
                    'command_minus_calibration_equivalent_mm':[(a-b)/s for a,b,s in zip(commanded,samples,slopes)]})
result={'run':RUN.name,'note':'Positive means more payout than that saved corner encoder sample under the fitted model. This is not a measured slack length; recorded shapes are inset from calibration corners and loads/pose may differ.','corners':out}
(ROOT/'qa/slack_audit_20260908/full_review/corner_reproduction.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('Saved four-corner reproduction audit')
