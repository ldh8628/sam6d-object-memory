#!/usr/bin/env python3
"""Object Memory fusion WITH an explicit SLAM<-SAM camera extrinsic.

X = T_camSLAM_camSAM is applied to the POSE stream, not to the detections:
    T_map_camSAM(t) = T_map_camSLAM(t) * X
so the pipeline still computes the same placement
    T_map_obj = T_map_camSLAM * X * T_camSAM_obj,
but every OTHER consumer of the pose now sees the camera that actually made the
detection. That matters for the existence filter: P_D asks "should this landmark
be visible right now?" by projecting it into the pose's frustum. Feeding it the
SLAM camera's pose while the objects were seen by a camera yawed 90 deg to the
side makes every landmark look permanently out of view, so nothing is ever
confirmed. Measured on 260804 longcircle2 (289 detections):
    X on detections (pose = SLAM cam) -> 66 instances,  1 survivor
    X on poses      (pose = SAM cam)  -> 29 instances, 11 survivors
The second matches the single-camera reference run (28 instances, 10 survivors).
`--x-on-detections` restores the old behaviour for comparison.

Extrinsic (this rig): SAM camera relative to SLAM camera =
  down 380mm, right 215mm, then yawed 90 deg to the right.
  optical frame: X right, Y down, Z forward.
  t_X = (0.215, 0.380, 0.0) m
  R_X = [[0,0,1],[0,1,0],[-1,0,0]]  (SAM forward -> SLAM right; down shared)
"""
import os, sys, dataclasses, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from adapters.bag_frame_index import load_frame_timestamps
from adapters.sam6d_pem import load_pem_detections
from adapters.slam_trajectory import load_slam_trajectory
from pipeline.object_memory_runner import run_object_memory
from core.transforms import make_transform, compose_transform

# import the report helper + camera loader from the sibling runner
import importlib.util
spec = importlib.util.spec_from_file_location("_rom", os.path.join(HERE, "run_object_memory.py"))
_rom = importlib.util.module_from_spec(spec); spec.loader.exec_module(_rom)


def build_extrinsic(rx_deg_right=90.0, right=0.215, down=0.380, forward=0.0, invert=False):
    # yaw-right 90 deg about the down (Y) axis, in optical frame
    R = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]
    t = [right, down, forward]
    X = make_transform(R, t)
    if invert:
        from core.transforms import invert_transform
        X = invert_transform(X)
    return X


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slam-traj", required=True)
    ap.add_argument("--pem-dir", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--time-tolerance", type=float, default=0.2)
    ap.add_argument("--assoc-trans-gate", type=float, default=0.20)
    ap.add_argument("--invert-extrinsic", action="store_true",
                    help="use X^-1 instead of X (flip if map still scatters)")
    ap.add_argument("--x-on-detections", action="store_true",
                    help="legacy: left-multiply X onto every detection instead of onto the "
                         "pose stream. Same T_map_obj, but the visibility/P_D model then runs "
                         "on the SLAM camera's frustum instead of the SAM camera's (see module "
                         "docstring); kept only for comparison.")
    ap.add_argument("--x-npy", default=None,
                    help="load a 4x4 extrinsic X from .npy (e.g. BA-refined) instead of the built-in estimate")
    ap.add_argument("--output-report", default=None)
    ap.add_argument("--list-out", default=None, help="write instance list here")
    a = ap.parse_args()

    if a.x_npy:
        import numpy as _np
        Xm = _np.load(a.x_npy)
        X = tuple(tuple(float(v) for v in row) for row in Xm)
        print(f"loaded extrinsic X from {a.x_npy}")
    else:
        X = build_extrinsic(invert=a.invert_extrinsic)
    print("extrinsic X (T_camSLAM_camSAM):")
    for row in X:
        print("   ", "  ".join(f"{v:7.3f}" for v in row))

    poses = load_slam_trajectory(a.slam_traj, source_slam_id="orbslam3_noimu")
    timestamps = load_frame_timestamps(a.bag, color_topic_hint=a.color_topic)
    detections = load_pem_detections(a.pem_dir, timestamps=timestamps)
    if a.x_on_detections:
        # legacy: T_camSLAM_obj = X * T_camSAM_obj  (detections is {frame_idx: [det,...]})
        detections = {fi: [dataclasses.replace(d, T_cam_obj=compose_transform(X, d.T_cam_obj))
                           for d in dets]
                      for fi, dets in detections.items()}
        print("extrinsic applied to DETECTIONS (legacy); visibility uses the SLAM camera pose")
    else:
        # default: move the pose stream to the camera that made the detections
        poses = [dataclasses.replace(p, T_map_cam=compose_transform(p.T_map_cam, X))
                 for p in poses]
        print("extrinsic applied to POSES: T_map_camSAM = T_map_camSLAM * X")
    cam_K, img_size = _rom._load_camera(a.pem_dir)

    result = run_object_memory(
        poses, detections, timestamps, time_tolerance=a.time_tolerance,
        assoc_trans_gate_m=a.assoc_trans_gate, assoc_rot_gate_deg=-1.0,
        cam_K=cam_K, img_size=img_size, quality_weighting=True,
    )
    lms = sorted(result.store.all_landmarks(), key=lambda x: x.object_id)
    print(f"objects: {len(lms)}  frames_fused: {result.frames_with_slam}/{result.frames_total}"
          f"  detections: {result.detections_total}")
    lines = []
    for i, lm in enumerate(lms, 1):
        t = lm.T_map_obj
        x, y, z = t[0][3], t[1][3], t[2][3]
        st = lm.status.value if hasattr(lm.status, "value") else lm.status
        s = (f"  #{lm.object_id} {lm.object_name} [{st}] "
             f"obs={lm.observation_count} xyz=({x:.3f}, {y:.3f}, {z:.3f})")
        print(s); lines.append(s)
    if a.list_out:
        open(a.list_out, "w").write("\n".join(lines) + "\n")
    if a.output_report:
        _rom.write_report(result, a.output_report, {
            "slam_traj": a.slam_traj, "pem_dir": a.pem_dir, "bag": a.bag,
            "n_poses": len(poses), "n_timestamps": len(timestamps)})
        print(f"report: {a.output_report}")


if __name__ == "__main__":
    main()
