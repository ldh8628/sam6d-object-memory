#!/usr/bin/env python3
"""Collect small reports; leave images, Atlas and full CSV data outside version control."""
import argparse,hashlib,json,platform,shutil
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,default=Path('/opt/orb-live/benchmarks'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    groups=['orb3_publish_v3','orb3_sweep_final','orb3_soak_20000','orb3_sweep_bounded','orb3_soak_bounded',
            'orb3_sweep_margin','orb3_soak_margin','orb3_sdk_soak','orb3_sweep_converged',
            'orb3_sweep_converged_1250','orb3_sweep_grid','orb3_sdk_grid','orb3_sdk_grid_threads',
            'orb3_sdk_grid_repeat','orb3_sweep_grid_t3','orb3_sdk_release','orb3_soak_grid','orb3_soak_grid_t3',
            'orb3_zero_counter','orb3_sdk_verified','orb3_soak_verified','orb3_faults_verified',
            'orb3_sdk_converged','orb3_soak_converged','orb3_faults_final','orb3_sdk_final','orb3_lifecycle_final']
    index={}
    for group in groups:
        source=a.base/group
        if not source.exists():continue
        target=a.output/group;target.mkdir(exist_ok=True)
        for name in ('summary.json','native_summary.json','run.json','integration.json','faults.json','sweep.json','common_accuracy.json','lifecycle.json','latency_analysis.json'):
            if (source/name).is_file():shutil.copy2(source/name,target/name)
        for child in source.iterdir():
            if not child.is_dir() or child.name=='mapping_xyz':continue
            files=[child/name for name in ('summary.json','accuracy.json','run.json') if (child/name).is_file()]
            if files:
                dest=target/child.name;dest.mkdir(exist_ok=True)
                for path in files:shutil.copy2(path,dest/path.name)
        index[group]=str(source)
    (a.output/'index.json').write_text(json.dumps(index,indent=2)+'\n')
    root=Path(__file__).resolve().parents[1]
    sources=['run_orb3_publish.py','orbslam_ws/src/orbslam3_ros2/src/orb3_live_node.cpp',
             'orbslam_ws/src/orbslam3_ros2/CMakeLists.txt','orbslam_ws/src/ORB_SLAM3/CMakeLists.txt',
             'orbslam_ws/src/ORB_SLAM3/src/Tracking.cc','orbslam_ws/src/ORB_SLAM3/src/MLPnPsolver.cpp',
             'orbslam_ws/src/ORB_SLAM3/src/System.cc','orbslam_ws/src/ORB_SLAM3/include/Tracking.h',
             'orbslam_ws/src/ORB_SLAM3/src/Optimizer.cc','orbslam_ws/src/ORB_SLAM3/include/Optimizer.h',
             'orbslam_ws/src/ORB_SLAM3/src/Frame.cc','orbslam_ws/src/ORB_SLAM3/src/KeyFrame.cc',
             'orbslam_ws/src/ORB_SLAM3/include/System.h','orbslam_ws/src/ORB_SLAM3/lib/libORB_SLAM3.so',
             'orbslam_ws/install_jazzy/orbslam3_ros2/lib/orbslam3_ros2/orb3_live_node']
    manifest=dict(platform=platform.platform(),python=platform.python_version(),
                  sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in sources})
    (a.output/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    data={}
    for name in ('rgbd_dataset_freiburg1_xyz','rgbd_dataset_freiburg1_rpy'):
        directory=a.base/name;report=directory/'association_report.json'
        if report.is_file():
            data[name]=dict(association=json.loads(report.read_text()),
                           associations_sha256=hashlib.sha256((directory/'associations.txt').read_bytes()).hexdigest())
    (a.output/'data_sources.json').write_text(json.dumps(data,indent=2)+'\n')
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import PoseStamped
    from distributed_slam_interfaces.msg import State
    pose=PoseStamped();pose.header.frame_id='map';pose.pose.orientation.w=1.
    state=State();state.header=pose.header;state.pose=pose.pose;state.tracking=True
    state.map_id='a'*64;state.session_id='b'*32
    sizes=dict(pose_bytes=len(serialize_message(pose)),state_bytes=len(serialize_message(state)))
    sizes.update(combined_payload_bytes_per_second_at_30hz=30*sum(sizes.values()),
                 excludes_dds_udp_ip_ethernet_overhead=True,packet_capture_performed=False)
    (a.output/'wire_sizes.json').write_text(json.dumps(sizes,indent=2)+'\n')
if __name__=='__main__':main()
