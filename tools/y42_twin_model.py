"""Reduced cable dynamics, planning, and original-vertex-preserving refinement.

mm, seconds, Newtons, kg. This is an explicit hypothesis model, not calibrated
metrology. Fixed axial exit approximation; cylinder tangent + wrapped arc;
rigid ball has xyz/yaw, floor reaction, unilateral cable springs and damping.
Actuator targets are quantised to 0.1 degrees and held between command ticks;
the smaller integration step is not a claim about the hardware control rate.
"""
import math
import numpy as np
try:
    from y42_spool_geometry import platform_payouts_fixed_z
    from y42_exact_path import exact_vertex_profile
except ModuleNotFoundError:  # Support import as tools.y42_twin_model.
    from tools.y42_spool_geometry import platform_payouts_fixed_z
    from tools.y42_exact_path import exact_vertex_profile

AXES=np.array([[-44.,148.],[154.,148.],[154.,0.],[-44.,0.]])
OFFSETS=np.array([[-15.267,11.4115],[15.267,11.4115],[15.267,-11.4115],[-15.267,-11.4115]])
BRANCH=np.array([1.,-1.,1.,-1.])
REF=np.arctan2(74+OFFSETS[:,1]-AXES[:,1],55+OFFSETS[:,0]-AXES[:,0])
DEFAULTS=dict(speed=10.,spacing=.25,mass=.12,stiffness=.8,preload=.5,lag=.06,
              damping=12.,radius=12.,exit_z=10.5,acceleration=80.,max_rpm=40.,height=0.,control_hz=25.)


def geometry(q, radius=12.,exit_z=10.5):
    x,y,z,yaw=q
    result=platform_payouts_fixed_z(
        x,y,z,yaw,
        AXES.tolist(),OFFSETS.tolist(),[float(radius)]*4,BRANCH.tolist(),REF.tolist(),
        exit_z_mm=exit_z,
    )
    return (
        np.asarray(result['lengths_mm'],dtype=float),
        np.asarray(result['jacobian_xyz_yaw_rad'],dtype=float),
        np.asarray(result['tangent_xy_mm'],dtype=float),
        np.asarray(result['endpoints_xy_mm'],dtype=float),
    )


def preset(name):
    if name=='line': return [[35,74,25],[75,74,25],[35,74,25]]
    if name=='square': return [[30,49,25],[80,49,25],[80,99,25],[30,99,25],[30,49,25]]
    if name=='circle': return [[55+25*math.cos(a),74+25*math.sin(a),25] for a in np.linspace(0,2*math.pi,129)]
    if name=='snake':
        result=[]
        for i,y in enumerate(np.linspace(30,118,7)):
            for x in ([25,85] if i%2==0 else [85,25]):result.append([x,float(y),25])
        return result
    raise ValueError('未知轨迹')


def refine(points, spacing):
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or not 2<=len(p)<=6000 or not np.all(np.isfinite(p)):
        raise ValueError('轨迹需2..6000个有限XYZ点')
    if np.any(p[:,:2]<[0,0]) or np.any(p[:,:2]>[110,148]) or np.any(p[:,2]<25) or np.any(p[:,2]>40):raise ValueError('轨迹超出预览范围')
    out=[p[0]]
    for a,b in zip(p,p[1:]):
        distance=float(np.linalg.norm(b-a))
        if distance==0:continue
        count=max(1,math.ceil(distance/spacing))
        out.extend(a+(b-a)*i/count for i in range(1,count))
        out.append(b.copy())
    if len(out)>30000:raise ValueError('轨迹过密，最多30000个细化点')
    if len(out)<2:raise ValueError('轨迹长度为零')
    return np.array(out)


def plan(points, speed,spacing,acceleration):
    path=refine(points,spacing);ds=np.linalg.norm(np.diff(path,axis=0),axis=1)
    v=np.full(len(path),speed,dtype=float);v[0]=v[-1]=0
    original=np.asarray(points,dtype=float)
    original=original[np.r_[True,np.linalg.norm(np.diff(original,axis=0),axis=1)>0]]
    original_ds=np.linalg.norm(np.diff(original,axis=0),axis=1)
    directions=np.diff(original,axis=0)/original_ds[:,None]
    vertex=0
    for i in range(1,len(original)-1):
        vertex+=max(1,math.ceil(original_ds[i-1]/spacing))
        angle=math.acos(float(np.clip(directions[i-1]@directions[i],-1,1)))
        if angle>math.radians(30):v[vertex]=0
        elif angle>1e-5:v[vertex]=min(speed,math.sqrt(acceleration*min(original_ds[i-1],original_ds[i])/angle))
    for i in range(1,len(v)):v[i]=min(v[i],math.sqrt(v[i-1]**2+2*acceleration*ds[i-1]))
    for i in range(len(v)-2,-1,-1):v[i]=min(v[i],math.sqrt(v[i+1]**2+2*acceleration*ds[i]))
    times=[0.]
    for i,d in enumerate(ds):
        total=v[i]+v[i+1]
        times.append(times[-1]+(2*d/total if total>1e-9 else 2*math.sqrt(d/acceleration)))
    return path,np.array(times),v


def simulate(data):
    p={**DEFAULTS,**data.get('parameters',{})}
    ranges=dict(speed=(1,50),spacing=(.1,2),mass=(.03,.5),stiffness=(.1,3),preload=(.05,2),lag=(0,.3),damping=(1,40),radius=(10,16),exit_z=(5,16),acceleration=(10,200),max_rpm=(1,100),height=(0,10),control_hz=(25,100))
    for key,(lo,hi) in ranges.items():
        p[key]=float(p[key])
        if not math.isfinite(p[key]) or not lo<=p[key]<=hi:raise ValueError(f'{key} 超范围 {lo}..{hi}')
    if p['control_hz'] not in (25.,50.,100.):raise ValueError('控制周期仅支持25/50/100 Hz')
    control_tick=1/p['control_hz']
    points=data.get('points') or preset(data.get('shape','circle'))
    points=[[float(a),float(b),float(z)+p['height']] for a,b,z in points]
    display_path=refine(points,p['spacing'])
    path=np.asarray(points,dtype=float)
    times,vel,planned_peak=exact_vertex_profile(path,p['speed'],p['acceleration'],tick=control_tick)
    times=np.asarray(times);vel=np.asarray(vel)
    if times[-1]>180:raise ValueError('单轮预览限180秒，请提高速度或缩短轨迹')
    dt=.002; q=np.array([*path[0],0.]); q[2]=25
    velocity=np.zeros(4);rest=geometry(q,p['radius'],p['exit_z'])[0]-p['preload']/p['stiffness']
    history=[];index=0;max_tension=0;slack_steps=0
    command_reference=rest.copy();desired=rest.copy();next_command=0.;command_index=0
    command_target=path[0].copy();command_records=[]
    payout_quantum=p['radius']*math.pi/180*.1
    max_cable_speed=p['max_rpm']*2*math.pi*p['radius']/60
    for tick in range(int(math.ceil((times[-1]+.6)/dt))+1):
        t=tick*dt
        while index<len(times)-2 and t>times[index+1]:index+=1
        elapsed=min(max(0,t-times[index]),times[index+1]-times[index])
        interval=times[index+1]-times[index]
        distance=np.linalg.norm(path[index+1]-path[index])
        if vel[index]+vel[index+1]<1e-9:
            fraction=(2*(elapsed/interval)**2 if elapsed<=interval/2 else 1-2*(1-elapsed/interval)**2)
        else:
            fraction=(vel[index]*elapsed+.5*(vel[index+1]-vel[index])/interval*elapsed**2)/distance
        fraction=min(1,max(0,fraction))
        target=path[index]+(path[index+1]-path[index])*fraction
        target_q=np.array([*target,0.])
        if t+1e-10>=next_command:
            command_target=target.copy()
            raw_desired=geometry(target_q,p['radius'],p['exit_z'])[0]-p['preload']/p['stiffness']
            desired=command_reference+np.round((raw_desired-command_reference)/payout_quantum)*payout_quantum
            command_records.append(dict(t=round(t,6),target=command_target.tolist(),rest_lengths_mm=desired.tolist()))
            command_index+=1;next_command=command_index*control_tick
        rest_rate=np.clip((desired-rest)/max(dt,p['lag']),-max_cable_speed,max_cable_speed)
        rest+=rest_rate*dt
        lengths,jac,_,_=geometry(q,p['radius'],p['exit_z'])
        extension=lengths-rest
        tension=np.where(extension>0,np.maximum(0,p['stiffness']*extension+.001*(jac@velocity-rest_rate)),0)
        force=-(jac.T@tension)
        acc=force*1000/np.array([p['mass']]*3+[.4*p['mass']*20**2])
        acc[2]-=9810;acc-=p['damping']*velocity
        velocity+=acc*dt;q+=velocity*dt
        if q[2]<25:q[2]=25;velocity[2]=max(0,velocity[2])
        if not np.all(np.isfinite(q)) or max(abs(q[0]),abs(q[1]))>500:raise ValueError('仿真发散，不可作为规划通过')
        max_tension=max(max_tension,float(max(tension)));slack_steps+=bool(np.any(tension<=1e-8))
        if tick%5==0:history.append(dict(t=round(t,4),target=target.tolist(),command_target=command_target.tolist(),pose=q.round(5).tolist(),tension=tension.round(5).tolist()))
    errors=[float(np.linalg.norm(np.array(h['pose'][:2])-h['target'][:2])) for h in history]
    return dict(parameters=p,source_points=points,path=display_path.tolist(),frames=history,commands=command_records,duration=times[-1],point_count=len(display_path),planned_peak_speed_mm_s=planned_peak,path_policy='exact_vertices_configured_clock_rest_to_rest',command_hz=p['control_hz'],angle_quantum_deg=.1,
                metrics=dict(xy_rms_mm=float(np.sqrt(np.mean(np.square(errors)))),xy_max_mm=max(errors),max_tension_n=max_tension,slack_fraction=slack_steps/(tick+1)),
                physical_scope='XYZ+yaw，接触面z=25，切点+平面包角/固定出绳高度近似，单边弹性绳，重力与电机长度滞后；质量/刚度/阻尼/预紧未实测辨识；无滚转俯仰/乱排线/摩擦辨识',
                hardware_validated=False)
