#!/usr/bin/env python3
"""Verify built ORB core and native ROS input without cameras, bags or SAM weights."""
from pathlib import Path
import subprocess
import sys
import tempfile

from camera_extrinsic_localization import ROOT, ORB_INSTALL, write_settings


def main():
    subprocess.run(['ctest', '--test-dir', str(ROOT / 'orbslam_ws/build_jazzy/orbslam3_core'),
                    '--output-on-failure', '--no-tests=error'], check=True)
    parent = ROOT / 'output/deployment_self_test'
    parent.mkdir(parents=True, exist_ok=True)
    # Retain failed run evidence; the generated input is tiny and contains no images on disk.
    output = Path(tempfile.mkdtemp(prefix='orb_', dir=parent))
    camera = dict(width=640, height=480, K=[500,0,320,0,500,240,0,0,1], D=[0]*5)
    write_settings({'SLAM':dict(camera=camera, color_hz=30)}, 'SLAM', output/'settings.yaml',
                    'System.SaveAtlasToFile', 'test_atlas', .095)
    subprocess.run([sys.executable, str(ROOT / 'orbslam_ws/src/orbslam3_ros2/tests/check_input_ros.py'),
        '--executable', str(ORB_INSTALL / 'orbslam3_ros2/lib/orbslam3_ros2/rgbd_node'),
        '--vocabulary', str(ROOT / 'orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
        '--settings', str(output / 'settings.yaml'), '--output', str(output / 'native_input'),
        '--modes', 'native', 'errors', 'qos'], check=True)
    print(f'ORB runtime self-test PASS: {output}')

if __name__ == '__main__':
    main()
