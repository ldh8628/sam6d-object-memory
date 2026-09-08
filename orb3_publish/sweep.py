#!/usr/bin/env python3
"""Compare feature/thread settings on the same dataset or SDK recording."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--atlas',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--dataset',type=Path,default=Path('/opt/orb-live/benchmarks/rgbd_dataset_freiburg1_rpy'))
    ap.add_argument('--realsense-bag',type=Path,help='Use SDK replay instead of the dataset; no ground-truth scoring')
    ap.add_argument('--cases',default='1000:2,1000:4,1250:1,1500:1,2000:1')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    reports=[]
    for case in args.cases.split(','):
        features,threads=map(int,case.split(':'));out=args.output/f'f{features}_t{threads}'
        command=[sys.executable,str(ROOT/'run_orb3_publish.py'),'--localhost-only','--atlas',str(args.atlas),
                 '--settings',str(ROOT/'orb3_publish/config/TUM1.yaml'),
                 '--realsense-bag' if args.realsense_bag else '--dataset',str(args.realsense_bag or args.dataset),
                 '--output',str(out),'--features',str(features),'--opencv-threads',str(threads)]
        with (args.output/f'{out.name}.log').open('w') as log:
            p=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        if p.returncode==0 and not args.realsense_bag:
            subprocess.run([sys.executable,str(ROOT/'orb3_publish/evaluate.py'),str(out),str(args.dataset/'groundtruth.txt')],check=True)
        report=json.loads((out/'summary.json').read_text());report.update(features=features,threads=threads)
        if (out/'accuracy.json').exists():report.update(json.loads((out/'accuracy.json').read_text()))
        reports.append(report);print(out.name,json.dumps(report),flush=True)
    (args.output/'sweep.json').write_text(json.dumps(reports,indent=2)+'\n')

if __name__=='__main__':main()
