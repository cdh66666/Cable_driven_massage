"""Offline nominal geometry roundtrip, with quantised motor targets archived.

No controller import, serial access, calibration mutation or hardware motion.
The nominal angle datum/signs below are explicit software fixtures, not a live
encoder calibration. Actual saved calibration readiness is reported separately.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from y42_twin_model import geometry, preset, refine
from y42_xy_inverse import solve_fixed_plane_xy

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'qa/twin'


def build():
    reference=geometry([55.,74.,25.,0.])[0]
    slopes=np.asarray([-1.,1.,-1.,1.])*180/(math.pi*12.)
    def forward(x,y):
        return ((geometry([x,y,25.,0.])[0]-reference)*slopes).tolist()
    paths=[('方框',preset('square')),('蛇形',preset('snake')),
           ('不规则折线',[[20,30,25],[30,110,25],[52,65,25],[72,118,25],
                        [88,35,25],[55,48,25],[20,30,25]])]
    cases=[]
    for label,original in paths:
        points=refine(original,.75)[:,:2]
        decoded=[];targets=[];quantized=[];unquantized=[];residuals=[]
        guess=(55.,74.)
        for point in points:
            q=forward(*point)
            exact=solve_fixed_plane_xy(q,forward,initial_xy=guess,
                                      residual_tolerance=1e-6,step_tolerance=1e-9)
            if not exact['ok']:
                raise RuntimeError(f'{label} exact inverse failed: {exact}')
            # Same signed magnitude / 0.1 degree rounding used by X CB framing.
            encoded=[math.copysign(round(abs(v)*10)/10,v) for v in q]
            result=solve_fixed_plane_xy(encoded,forward,initial_xy=guess,
                                       residual_tolerance=.06,step_tolerance=1e-9)
            if not result['ok']:
                raise RuntimeError(f'{label} quantised inverse failed: {result}')
            guess=result['xy_mm']
            decoded.append(guess); targets.append(q);quantized.append(encoded)
            unquantized.append(math.dist(point,exact['xy_mm']))
            residuals.append(result['residual_max'])
        errors=[math.dist(p,q) for p,q in zip(points,decoded)]
        cases.append(dict(label=label,original_vertices_mm=original,source_xy_mm=points.tolist(),
                          decoded_xy_mm=decoded,motor_targets_deg=targets,encoded_motor_targets_deg=quantized,
                          unquantized_max_mm=max(unquantized),quantized_max_mm=max(errors),
                          quantized_rms_mm=math.sqrt(sum(e*e for e in errors)/len(errors)),
                          inverse_max_residual_deg=max(residuals)))
    cal_path=ROOT/'tools/y42_pc_calibration.json'
    cal=json.loads(cal_path.read_text(encoding='utf-8'))
    return dict(schema='y42.offline_model_roundtrip.v1',cases=cases,hardware_executed=False,
                physical_accuracy_validated=False,model='shared fixed-exit cylinder approximation',
                assumptions=dict(z_mm=25,yaw_rad=0,radius_mm=12,exit_z_mm=10.5,
                                 angle_quantum_deg=.1,inverse_residual_tolerance_deg=.06,
                                 unquantized_inverse_residual_tolerance_deg=1e-6),
                saved_calibration=dict(sha256=hashlib.sha256(cal_path.read_bytes()).hexdigest(),
                                       valid=cal.get('valid'),modified=False),
                source_sha256={name:hashlib.sha256((ROOT/'tools'/name).read_bytes()).hexdigest()
                               for name in ['y42_twin_model.py','y42_spool_geometry.py','y42_xy_inverse.py']})


if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True)
    report=build()
    (OUT/'model-command-roundtrip.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps([{k:c[k] for k in ('label','unquantized_max_mm','quantized_max_mm','quantized_rms_mm')}
                      for c in report['cases']],ensure_ascii=False))
