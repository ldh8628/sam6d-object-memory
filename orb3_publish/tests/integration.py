#!/usr/bin/env python3
"""Actual native ORB + independent ROS subscriber; no camera or remote claim."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from distributed_slam_interfaces.msg import State,Session

ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--atlas',type=Path,required=True); ap.add_argument('--dataset',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True); ap.add_argument('--max-frames',type=int,default=180)
    ap.add_argument('--features',type=int,default=1200);ap.add_argument('--opencv-threads',type=int,default=3)
    args=ap.parse_args(); out=args.output.resolve(); out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_DOMAIN_ID']='88';os.environ.pop('ROS_DISCOVERY_SERVER',None)
    rclpy.init(); node=rclpy.create_node('orb3_publish_integration_receiver')
    states=[];poses=[];sessions=[]
    qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
    def receive_state(msg):
        states.append(dict(sequence=msg.sequence,capture_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                           map_id=msg.map_id,session_id=msg.session_id,tracking=msg.tracking,
                           enqueue_ns=msg.enqueue_ns,publish_ns=msg.publish_ns,receive_ns=time.time_ns()))
    node.create_subscription(State,'/distributed_slam/state',receive_state,qos)
    node.create_subscription(PoseStamped,'/orbslam3/pose',lambda msg:poses.append(msg),qos)
    node.create_subscription(Session,'/distributed_slam/session',lambda msg:sessions.append(msg),
                             QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    command=[sys.executable,str(ROOT/'run_orb3_publish.py'),'--localhost-only','--atlas',str(args.atlas),
             '--settings',str(ROOT/'orb3_publish/config/TUM1.yaml'),'--dataset',str(args.dataset),
             '--output',str(out/'publisher'),'--domain-id','88','--max-frames',str(args.max_frames),
             '--features',str(args.features),'--opencv-threads',str(args.opencv_threads)]
    with (out/'launcher.log').open('w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        start=time.monotonic(); graph=[]
        while process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.02)
            if states and not graph: graph=node.get_topic_names_and_types()
            if time.monotonic()-start>max(180,args.max_frames/10+120):
                process.terminate();process.wait(timeout=50);raise RuntimeError('Publisher exceeded test timeout')
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    assert process.returncode==0, (process.returncode,out/'launcher.log')
    native=json.loads((out/'publisher/summary.json').read_text())
    # Only image-associated state messages belong to the input sequence.
    frame_states=[row for row in states if row['enqueue_ns']>0]
    sequences=[row['sequence'] for row in frame_states]
    with (out/'received.csv').open('w') as f:
        if states:
            writer=csv.DictWriter(f,fieldnames=list(states[0]));writer.writeheader();writer.writerows(states)
    print('expected',native['processed_frames'],'states',len(frame_states),'poses',len(poses),
          'missing',sorted(set(range(1,native['processed_frames']+1))-set(sequences)),flush=True)
    assert sequences==list(range(1,native['processed_frames']+1)), 'Dropped or reordered state messages'
    assert len(poses)==native['valid_poses'], 'Pose messages differ from valid tracking count'
    assert sessions and all(row['map_id']==sessions[-1].map_id for row in states)
    assert all(row['session_id']==sessions[-1].session_id for row in states)
    assert all(abs(sum(v*v for v in (p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w))-1)<1e-5 for p in poses)
    assert not any('sensor_msgs/msg/Image' in types or 'sensor_msgs/msg/PointCloud2' in types for _,types in graph)
    by_stamp={p.header.stamp.sec*10**9+p.header.stamp.nanosec:p for p in poses}
    with (out/'publisher/frames.csv').open() as f:
        for row in csv.DictReader(f):
            if row['tracking']!='1':continue
            p=by_stamp[int(row['capture_ns'])]
            actual=[p.pose.position.x,p.pose.position.y,p.pose.position.z,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]
            expected=[float(row[k]) for k in ('tx','ty','tz','qx','qy','qz','qw')]
            assert np.allclose(actual,expected,rtol=0,atol=1e-8),'Wire pose differs from the tracked pose'
            assert p.header.frame_id=='map'
    local_ms=[(row['receive_ns']-row['enqueue_ns'])/1e6 for row in frame_states if row['tracking']]
    report=dict(processed_frames=native['processed_frames'],received_frame_states=len(frame_states),
                received_valid_poses=len(poses),image_state_messages_only=len(states)==len(frame_states),wire_poses_match_tracking=True,
                session_received=bool(sessions),last_session_mode=sessions[-1].mode,all_sequences_received_in_order=True,no_image_topics=True,
                atlas_unchanged=native['atlas_unchanged'],local_process_receive_p99_ms=float(sorted(local_ms)[int(np.ceil(.99*len(local_ms)))-1]) if local_ms else None,
                remote_lan_verified=False,source='dataset',topics=graph)
    (out/'integration.json').write_text(json.dumps(report,indent=2)+'\n')
    with (out/'received.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(states[0]));writer.writeheader();writer.writerows(states)
    node.destroy_node();rclpy.shutdown();print(json.dumps(report,indent=2))

if __name__=='__main__':main()
