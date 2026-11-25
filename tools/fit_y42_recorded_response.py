"""Fit a low-amplitude response hypothesis to real command/encoder samples.

This fits archived 150mA microprobe dynamics, not cable geometry or XY accuracy.
Raw feedback is plotted as points; only the candidate model is continuous.
"""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from y42_twin_server import archive

ROOT=Path(__file__).resolve().parents[1]
RUN='20260905_230801_088_position_microprobe_worker_317e0355'
data=archive(RUN);commands=data['targets'];origin=commands[0]['t']
times=np.array([c['t']-origin for c in commands]);targets=np.array([c['q'] for c in commands])
grid=np.arange(0,times[-1]+.001,.002)
results=[]
plt.rcParams['font.sans-serif']=['Microsoft YaHei'];plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(1,2,figsize=(12,4.8))
for axis in range(2):
    samples=[s for s in data['samples'] if s['id']==axis+1 and origin<=s['sampled_at']<=commands[-1]['t']]
    t=np.array([s['sampled_at']-origin for s in samples]);actual=np.array([s['position']-targets[0,axis] for s in samples])
    target=targets[:,axis]-targets[0,axis]
    def predict(params):
        tau,delay,bias=params
        idx=np.clip(np.searchsorted(times,grid-delay,side='right')-1,0,len(times)-1)
        u=target[idx];u[grid<delay]=0
        y=np.empty(len(grid));y[0]=bias;blend=1-np.exp(-.002/tau)
        for k in range(1,len(y)):y[k]=y[k-1]+blend*(u[k]+bias-y[k-1])
        return y
    def residual(params):return np.interp(t,grid,predict(params))-actual
    fit=least_squares(residual,[.08,.08,0],bounds=([.003,0,-.15],[.6,.4,.15]),diff_step=.02)
    pred=predict(fit.x);rmse=float(np.sqrt(np.mean(residual(fit.x)**2)))
    results.append(dict(axis=axis+1,tau_s=float(fit.x[0]),delay_s=float(fit.x[1]),bias_deg=float(fit.x[2]),fit_rmse_deg=rmse,sample_count=len(samples),scope='historical 0.5deg low-current probe; quantized 0.1deg; no held-out validation; not general hardware identification'))
    ax=axes[axis];ax.step(times,target,where='post',label='实际写入目标',color='#2477c9');ax.scatter(t,actual,s=10,label='原始编码器反馈',color='#df783c');ax.plot(grid,pred,label='一阶滞后+纯延迟拟合',color='#159080')
    ax.set(title=f'M{axis+1} · 拟合残差 {rmse:.3f}°',xlabel='相对首条运动指令 / s',ylabel='相对角度 / °',ylim=(-.2,.7));ax.grid(alpha=.2);ax.legend(fontsize=9)
fig.suptitle('用真实运动反馈校验仿真响应假设',fontsize=16)
fig.text(.5,.012,'仅识别历史低电流微幅响应；不是新实机通过，也不是球心精度。M2归程记录不完整。',ha='center')
fig.tight_layout(rect=(0,.04,1,.93));out=ROOT/'qa/twin';fig.savefig(out/'measured-response-fit.png',dpi=160)
(out/'measured-response-fit.json').write_text(json.dumps({'source_run':RUN,'sha256':data['source_sha256'],'fits':results},ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False))
