"""Historical drawing diagnostics only; no controller import or hardware IO.

Reproduce the old zero-yaw equivalent-anchor approximation from archived rope
lengths. The 12.5mm datum is NOT a real fixed fairlead. These XY estimates are
not externally measured and must never be used to authorize motion.
"""
import numpy as np
from scipy.optimize import least_squares


def drawing_from_events(events):
    start = next((e for e in events if e.get('event') == 'run_start'), {})
    args = start.get('arguments', [])
    if len(args) < 2 or not isinstance(args[1], dict): return None
    path = args[1].get('path')
    if not path: return None
    targets = np.asarray(path, dtype=float)
    if targets.ndim != 2 or targets.shape[1] != 2 or not np.isfinite(targets).all(): return None
    # Exact v8 nominal reference used by these archived runs, retained only to
    # expose their old model assumptions rather than silently modernize them.
    axes = np.array([[-44.,148.],[154.,148.],[154.,0.],[-44.,0.]])
    exits = axes + np.array([[12.5,0],[-12.5,0],[-12.5,0],[12.5,0]])
    offsets = np.array([[-15.267,11.4115],[15.267,11.4115],[15.267,-11.4115],[-15.267,-11.4115]])
    anchors = exits-offsets
    rows=[]; guess=targets[0].copy()
    for e in events:
        if e.get('event') != 'trajectory_feedback_batch': continue
        lengths=np.asarray(e.get('relative_rope_lengths_mm', []), dtype=float)
        if lengths.shape != (4,) or not np.isfinite(lengths).all(): continue
        if e.get('sample_span_ms',float('inf')) > 100: continue
        fit=least_squares(lambda xy:np.linalg.norm(xy-anchors,axis=1)-lengths,guess)
        if not fit.success or not np.isfinite(fit.x).all(): continue
        guess=fit.x
        rows.append(dict(t=e['sampled_at'],xy=fit.x.tolist(),
            rope_residual_rms_mm=float(np.sqrt(np.mean(fit.fun**2))),
            raw_positions_deg=e.get('raw_positions_deg'),
            relative_rope_lengths_mm=lengths.tolist(),source_sequence=e['sequence']))
    return dict(target=targets.tolist(),feedback=rows,
        label='历史设备反馈反算 · 旧模型近似，非外部实测球心',
        model='archived_v8_equivalent_anchor_zero_yaw',
        limitations=['旧标定未经外部验证，现已失效','12.5mm是旧计算基准，非固定导绳孔','反馈稀疏；点间连线不代表完整真实轨迹','不能用于标定或放行运动'])
