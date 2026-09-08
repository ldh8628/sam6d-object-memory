#!/usr/bin/env python3
"""Exercise blank tracking, malformed input, invalid Atlas, and orderly SIGINT."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from run_orb3_publish import load_settings,write_opencv_settings,native_executable


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--atlas',type=Path,required=True)
    ap.add_argument('--dataset',type=Path,required=True)
    ap.add_argument('--bag',type=Path,help='Also interrupt the direct SDK capture path')
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    original=hashlib.sha256(a.atlas.read_bytes()).hexdigest()
    blank=a.output/'blank_input';blank.mkdir()
    cv2.imwrite(str(blank/'rgb.png'),np.zeros((480,640,3),dtype=np.uint8))
    cv2.imwrite(str(blank/'depth.png'),np.full((480,640),5000,dtype=np.uint16))
    lines=[f'{1300000000+i/30:.9f} rgb.png {1300000000+i/30:.9f} depth.png\n' for i in range(60)]
    (blank/'associations.txt').write_text(''.join(lines))
    duplicate=a.output/'duplicate_input';duplicate.mkdir()
    (duplicate/'rgb.png').symlink_to(blank/'rgb.png');(duplicate/'depth.png').symlink_to(blank/'depth.png')
    (duplicate/'associations.txt').write_text(lines[0]+lines[0])
    invalid=a.output/'invalid.osa';invalid.write_bytes(b'not a serialized ORB Atlas\n')
    # Produce a structurally valid but empty Atlas through the actual mapper.
    # This also covers normal empty-map saving without a shutdown hang.
    empty_dir=a.output/'empty_mapping';empty_dir.mkdir()
    settings=load_settings(ROOT/'orb3_publish/config/TUM1.yaml')
    settings['Camera.RGB']=0;settings['System.SaveAtlasToFile']='empty_map'
    write_opencv_settings(empty_dir/'settings.yaml',settings)
    import yaml
    params=dict(settings=str(empty_dir/'settings.yaml'),vocabulary=str(ROOT/'orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
                output=str(empty_dir),dataset=str(blank),benchmark_mapping=True,replay_rate=0.0)
    (empty_dir/'parameters.yaml').write_text(yaml.safe_dump({'orb3_live_publish':{'ros__parameters':params}}))
    with (empty_dir/'console.log').open('w') as log:
        subprocess.run([str(native_executable()),'--ros-args','--params-file',str(empty_dir/'parameters.yaml')],
                       cwd=empty_dir,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=90,
                       env=dict(os.environ,ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='89',RMW_IMPLEMENTATION='rmw_fastrtps_cpp'))
    empty_atlas=empty_dir/'empty_map.osa';assert empty_atlas.is_file() and empty_atlas.stat().st_size>0
    results=[]
    cases=[('blank',blank,a.atlas,5),('duplicate',duplicate,a.atlas,3),
           ('invalid_atlas',blank,invalid,None),('empty_atlas',blank,empty_atlas,2),('sigint',a.dataset,a.atlas,0)]
    if a.bag:cases.append(('sdk_sigint',a.bag,a.atlas,0))
    for name,dataset,atlas,expected in cases:
        out=a.output/name
        command=[sys.executable,str(ROOT/'run_orb3_publish.py'),'--localhost-only',
                 '--atlas',str(atlas),'--settings',str(ROOT/'orb3_publish/config/TUM1.yaml'),
                 '--realsense-bag' if name=='sdk_sigint' else '--dataset',str(dataset),'--output',str(out)]
        with (a.output/f'{name}.log').open('w') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            if name in ('sigint','sdk_sigint'):
                deadline=time.monotonic()+90
                while time.monotonic()<deadline and process.poll() is None:
                    path=out/'frames.csv'
                    if path.exists() and path.stat().st_size>16384:
                        break
                    time.sleep(.1)
                assert process.poll() is None,'Publisher ended before SIGINT'
                stopped=time.monotonic();process.send_signal(signal.SIGINT)
                process.wait(timeout=45)
                shutdown_seconds=time.monotonic()-stopped
            else:process.wait(timeout=90)
        result=json.loads((out/'summary.json').read_text())
        assert result['atlas_unchanged'],name
        if expected is None:
            assert process.returncode!=0 and result['processed_frames']==0
        else:assert process.returncode==expected,(name,process.returncode,result)
        if name=='blank':
            assert result['processed_frames']==60 and result['valid_poses']==0
            assert result['complete_input_processing'] and not result['local_33ms_met']
            assert not (out/'trajectory.tum').read_text().strip()
        elif name=='duplicate':
            assert 'timestamp reversal or duplicate' in result['error']
        elif name=='empty_atlas':
            assert result['processed_frames']==0
            assert 'Atlas has no usable keyframes' in (out/'console.log').read_text()
        elif name in ('sigint','sdk_sigint'):
            assert result['complete_input_processing'] and result['processed_frames']>0
            assert result['processed_frames']==result['accepted_pairs']
            assert result['valid_poses']>0 and shutdown_seconds<10
            result['shutdown_seconds']=shutdown_seconds
        assert not result['full_requirement_verified']
        results.append(dict(name=name,**result));print(name,'PASS',flush=True)
    assert hashlib.sha256(a.atlas.read_bytes()).hexdigest()==original
    (a.output/'lifecycle.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
