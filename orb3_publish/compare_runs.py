#!/usr/bin/env python3
"""Compare accuracy on common valid timestamps so missing poses cannot improve the score."""
import argparse,json
from pathlib import Path
import numpy as np
from evaluate import evaluate

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline',type=Path);p.add_argument('candidate',type=Path);p.add_argument('groundtruth',type=Path)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    old=np.atleast_2d(np.loadtxt(a.baseline/'trajectory.tum'))
    new=np.atleast_2d(np.loadtxt(a.candidate/'trajectory.tum'));gt=np.loadtxt(a.groundtruth)
    common=np.intersect1d(old[:,0],new[:,0])
    old_score=evaluate(old[np.isin(old[:,0],common)],gt)
    new_score=evaluate(new[np.isin(new[:,0],common)],gt)
    result=dict(common_valid_timestamps=len(common),baseline=old_score,candidate=new_score,
                method='Same valid timestamps for both runs; missing frames still count against per-run availability')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
