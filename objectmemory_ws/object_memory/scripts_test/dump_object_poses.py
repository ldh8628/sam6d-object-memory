#!/usr/bin/env python3
"""Run single-camera object memory and dump full 4x4 T_map_obj per landmark.
Output lines: object_id object_name status obs  m00 m01 ... m33 (16 floats).
"""
import os, sys, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from adapters.bag_frame_index import load_frame_timestamps
from adapters.sam6d_pem import load_pem_detections
from adapters.slam_trajectory import load_slam_trajectory
from pipeline.object_memory_runner import run_object_memory

ap = argparse.ArgumentParser()
ap.add_argument("--slam-traj", required=True)
ap.add_argument("--pem-dir", required=True)
ap.add_argument("--bag", required=True)
ap.add_argument("--time-tolerance", type=float, default=0.1)
ap.add_argument("--assoc-trans-gate", type=float, default=0.35)
ap.add_argument("--out", required=True)
a = ap.parse_args()

poses = load_slam_trajectory(a.slam_traj, source_slam_id="orbslam3_noimu")
ts = load_frame_timestamps(a.bag, color_topic_hint="/camera/camera/color/image_raw")
dets = load_pem_detections(a.pem_dir, timestamps=ts)
res = run_object_memory(poses, dets, ts, time_tolerance=a.time_tolerance,
                        assoc_trans_gate_m=a.assoc_trans_gate, assoc_rot_gate_deg=-1.0)
lines = []
for lm in sorted(res.store.all_landmarks(), key=lambda x: x.object_id):
    T = lm.T_map_obj
    flat = " ".join(f"{T[r][c]:.6f}" for r in range(4) for c in range(4))
    st = lm.status.value if hasattr(lm.status, "value") else lm.status
    lines.append(f"{lm.object_id} {lm.object_name} {st} {lm.observation_count} {flat}")
open(a.out, "w").write("\n".join(lines) + "\n")
print(f"dumped {len(lines)} landmarks -> {a.out}")
for l in lines:
    p = l.split(); print(f"  #{p[0]} {p[1]:22} {p[2]:10} obs={p[3]}  t=({float(p[7]):.2f},{float(p[11]):.2f},{float(p[15]):.2f})")
