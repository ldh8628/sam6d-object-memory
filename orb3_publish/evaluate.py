#!/usr/bin/env python3
"""SE(3) trajectory evaluation: no scale fit; GT interpolation; gap-aware RPE."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def evaluate(estimated,groundtruth):
    est=np.atleast_2d(estimated); gt=np.atleast_2d(groundtruth)
    est=est[(est[:,0]>=gt[0,0])&(est[:,0]<=gt[-1,0])]
    if len(est)<3: return dict(associated_poses=len(est),error='Insufficient valid poses for trajectory evaluation')
    # Ground truth samples must bracket the estimate within 50 ms.
    indexes=np.searchsorted(gt[:,0],est[:,0],side='right').clip(1,len(gt)-1)
    keep=(gt[indexes,0]-gt[indexes-1,0])<=0.05
    est=est[keep]
    if len(est)<3: return dict(associated_poses=len(est),error='Insufficient ground truth coverage')
    t=est[:,0]; expected=np.column_stack([np.interp(t,gt[:,0],gt[:,k]) for k in (1,2,3)])
    rg=Slerp(gt[:,0],Rotation.from_quat(gt[:,4:8]))(t)
    re=Rotation.from_quat(est[:,4:8]); positions=est[:,1:4]
    a=positions-positions.mean(0); b=expected-expected.mean(0)
    u,_,vt=np.linalg.svd(a.T@b); adjust=np.eye(3); adjust[2,2]=np.linalg.det(vt.T@u.T)
    align=vt.T@adjust@u.T
    aligned=a@align.T+expected.mean(0)
    ate=np.linalg.norm(aligned-expected,axis=1)
    dt=np.diff(t); usable=(dt>0)&(dt<=0.1)
    dg=rg[:-1].inv()*rg[1:]; de=re[:-1].inv()*re[1:]
    rotation_error=np.rad2deg((dg.inv()*de).magnitude())
    angular_speed=np.rad2deg(dg.magnitude())/dt
    translation_g=rg[:-1].inv().apply(np.diff(expected,axis=0))
    translation_e=re[:-1].inv().apply(np.diff(positions,axis=0))
    translation_error=np.linalg.norm(translation_g-translation_e,axis=1)
    high=usable&(angular_speed>=30)
    def rms(values): return float(np.sqrt(np.mean(values**2))) if len(values) else None
    return dict(associated_poses=len(est),ate_rmse_m=rms(ate),ate_p99_m=float(np.quantile(ate,.99)),
                relative_pairs=int(usable.sum()),rpe_rotation_rmse_deg=rms(rotation_error[usable]),
                rpe_translation_rmse_m=rms(translation_error[usable]),
                fast_rotation_threshold_deg_s=30,fast_rotation_pairs=int(high.sum()),
                fast_rotation_rpe_rmse_deg=rms(rotation_error[high]),
                fast_rotation_rpe_p99_deg=float(np.quantile(rotation_error[high],.99)) if high.any() else None,
                groundtruth_interpolation='linear translation and SLERP; bracket <=50ms',
                alignment='rigid SE3 for ATE only; no scale fit; RPE independent of alignment')


def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('run',type=Path); ap.add_argument('groundtruth',type=Path)
    args=ap.parse_args()
    path=args.run/'trajectory.tum'
    if not path.stat().st_size: report=dict(associated_poses=0,error='No valid poses')
    else: report=evaluate(np.loadtxt(path),np.loadtxt(args.groundtruth))
    (args.run/'accuracy.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(args.run.name,json.dumps(report))

if __name__=='__main__': main()
