"""Story 5 before/after: legacy hit-counter vs quality-weighted confidence,
on the 4 SAM datasets. Reports the metrics that map to the diagnosed problems:

 (a) camera-overlap survivors  : surviving landmarks sitting <0.15 m from the
                                 camera trajectory (degenerate z~0 poses).
 (b) ghost lifespan            : detection-frames a spawned-then-deleted track
                                 lived (how long a false object lingers).
 story4 no-regression          : surviving instances per present class (target 1).
 circle over-promotion         : classes with >1 surviving instance.
"""
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "src")))

from adapters.slam_trajectory import load_slam_trajectory       # noqa: E402
from adapters.bag_frame_index import load_frame_timestamps      # noqa: E402
from adapters.sam6d_pem import load_pem_detections              # noqa: E402
from pipeline.object_memory_runner import run_object_memory     # noqa: E402
from run_object_memory import _load_camera                      # noqa: E402
from core.state_machine import ObjectStatus                     # noqa: E402

DATASETS = ["SAM_circle", "SAM_loop1", "SAM_loop2", "SAM_occlusion"]
IN = os.path.abspath(os.path.join(HERE, "..", "..", "input"))
SURVIVE = {ObjectStatus.active, ObjectStatus.remembered}
CAM_OVERLAP_M = 0.15


def load(ds):
    poses = load_slam_trajectory(f"{IN}/{ds}/slam/CameraTrajectory.txt")
    ts = load_frame_timestamps(f"{IN}/{ds}/bag")
    dets = load_pem_detections(f"{IN}/{ds}/pem", timestamps=ts)
    cam_K, img = _load_camera(f"{IN}/{ds}/pem")
    return poses, dets, ts, cam_K, img


def metrics(res, poses):
    cam_xyz = [(p.T_map_cam[0][3], p.T_map_cam[1][3], p.T_map_cam[2][3])
               for p in poses]

    def min_cam_dist(lm):
        x, y, z = lm.T_map_obj[0][3], lm.T_map_obj[1][3], lm.T_map_obj[2][3]
        return min(((x-cx)**2+(y-cy)**2+(z-cz)**2) ** 0.5
                   for cx, cy, cz in cam_xyz) if cam_xyz else 9e9

    surv_by_cls = defaultdict(int)
    cam_overlap = 0
    for lm in res.store.all_landmarks():
        if lm.status in SURVIVE:
            surv_by_cls[lm.object_name] += 1
            if min_cam_dist(lm) < CAM_OVERLAP_M:
                cam_overlap += 1
    surv = sum(surv_by_cls.values())
    multi = [c for c, n in surv_by_cls.items() if n > 1]

    # ghost lifespan (detection-frames) for spawned-then-deleted tracks
    ordinal = {fr.frame_idx: k for k, fr in enumerate(res.frame_results)}
    birth = {}
    for fr in res.frame_results:
        for dcn in fr.decisions:
            if dcn.decision_type.value == "new_tentative":
                birth.setdefault(dcn.object_id, ordinal[fr.frame_idx])
    stamp_ord = {round(fr.stamp, 6): k for k, fr in enumerate(res.frame_results)
                 if fr.stamp is not None}
    lifespans = []
    n_deleted = 0
    for oid, tr in res.store.transition_log():
        if tr.to_status.value == "deleted":
            n_deleted += 1
            d = stamp_ord.get(round(tr.stamp, 6)) if tr.stamp is not None else None
            b = birth.get(oid)
            if b is not None and d is not None:
                lifespans.append(d - b)
    lifespans.sort()
    med = lifespans[len(lifespans)//2] if lifespans else 0
    mx = lifespans[-1] if lifespans else 0
    return dict(spawned=len(birth), surv=surv, deleted=n_deleted,
                cam_overlap=cam_overlap, multi=multi,
                life_med=med, life_max=mx, life_n=len(lifespans))


def main():
    cache = {ds: load(ds) for ds in DATASETS}
    for ds in DATASETS:
        poses, dets, ts, cam_K, img = cache[ds]
        print(f"\n================= {ds} =================")
        print(f"{'mode':>8} {'spawned':>7} {'surv':>4} {'del':>4} "
              f"{'camOvlp':>7} {'lifeMed':>7} {'lifeMax':>7} {'multiSurv'}")
        for label, qw in (("legacy", False), ("quality", True)):
            res = run_object_memory(poses, dets, ts, assoc_trans_gate_m=0.20,
                                    assoc_rot_gate_deg=-1.0, cam_K=cam_K,
                                    img_size=img, quality_weighting=qw)
            m = metrics(res, poses)
            print(f"{label:>8} {m['spawned']:7d} {m['surv']:4d} {m['deleted']:4d} "
                  f"{m['cam_overlap']:7d} {m['life_med']:7d} {m['life_max']:7d} "
                  f"  {m['multi']}")


if __name__ == "__main__":
    main()
