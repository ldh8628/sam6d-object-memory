#!/usr/bin/env python3
"""TUM rotation replay using an xyz-built map. Run in the sourced Jazzy environment."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from run_orb3_publish import load_settings,write_opencv_settings,native_executable,digest


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data',type=Path,default=Path('/opt/orb-live/benchmarks'))
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--repeats',type=int,default=3)
    ap.add_argument('--skip-map',action='store_true')
    args=ap.parse_args(); out=args.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    settings=ROOT/'orb3_publish/config/TUM1.yaml'; vocabulary=ROOT/'orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt'
    mapout=out/'mapping_xyz'; atlas=mapout/'benchmark_map.osa'
    if not args.skip_map:
        mapout.mkdir(exist_ok=False)
        cfg=load_settings(settings); cfg['Camera.RGB']=0; cfg['System.SaveAtlasToFile']='benchmark_map'
        write_opencv_settings(mapout/'settings.yaml',cfg)
        import yaml
        params=dict(settings=str(mapout/'settings.yaml'),vocabulary=str(vocabulary),output=str(mapout),
                    dataset=str(args.data/'rgbd_dataset_freiburg1_xyz'),benchmark_mapping=True,
                    replay_rate=0.5,queue_capacity=120,opencv_threads=1)
        (mapout/'parameters.yaml').write_text(yaml.safe_dump({'orb3_live_publish':{'ros__parameters':params}}))
        command=[str(native_executable()),'--ros-args','--params-file',str(mapout/'parameters.yaml')]
        with (mapout/'console.log').open('w') as log:
            subprocess.run(command,cwd=mapout,stdout=log,stderr=subprocess.STDOUT,check=True)
        if not atlas.is_file() or atlas.stat().st_size<10000: raise RuntimeError('Benchmark mapping failed')
    for repeat in range(args.repeats):
        # Alternate order to reduce systematic thermal / cache effects.
        for mode in (('off','on') if repeat%2==0 else ('on','off')):
            result=out/f'rpy_{mode}_{repeat+1}'
            command=[sys.executable,str(ROOT/'run_orb3_publish.py'),'--localhost-only','--atlas',str(atlas),'--settings',str(settings),
                     '--dataset',str(args.data/'rgbd_dataset_freiburg1_rpy'),'--output',str(result),
                     '--rotation-fallback',mode,'--opencv-threads','1','--features','0']
            with (out/f'{result.name}.log').open('w') as log:
                subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
            subprocess.run([sys.executable,str(ROOT/'orb3_publish/evaluate.py'),str(result),
                            str(args.data/'rgbd_dataset_freiburg1_rpy/groundtruth.txt')],check=True)
    print('Benchmark reports:',out,'Atlas SHA256:',digest(atlas))

if __name__=='__main__': main()
