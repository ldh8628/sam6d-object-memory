"""Multi-instance association eval: per-class instance counts by terminal
status across radii, for the 4 SAM datasets. GT-free (datasets are physically
single-instance-per-class, so the target is 1 surviving instance per present
class + phantom control)."""
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

DATASETS = ["SAM_circle", "SAM_loop1", "SAM_loop2", "SAM_occlusion"]
IN = os.path.abspath(os.path.join(HERE, "..", "..", "input"))
RADII = [0.10, 0.15, 0.20, 0.30]
SURVIVE = {"active", "remembered"}


def load(ds):
    poses = load_slam_trajectory(f"{IN}/{ds}/slam/CameraTrajectory.txt")
    ts = load_frame_timestamps(f"{IN}/{ds}/bag")
    dets = load_pem_detections(f"{IN}/{ds}/pem", timestamps=ts)
    cam_K, img = _load_camera(f"{IN}/{ds}/pem")
    return poses, dets, ts, cam_K, img


def summarize(result):
    by_cls = defaultdict(lambda: defaultdict(int))
    dom = defaultdict(int)   # max obs of any instance of a class
    for lm in result.store.all_landmarks():
        by_cls[lm.object_name][lm.status.value] += 1
        by_cls[lm.object_name]["_ids"] += 1
        dom[lm.object_name] = max(dom[lm.object_name], lm.observation_count)
    return by_cls, dom


def main():
    cache = {ds: load(ds) for ds in DATASETS}
    for ds in DATASETS:
        poses, dets, ts, cam_K, img = cache[ds]
        print(f"\n================= {ds} =================")
        print(f"{'radius':>7} {'ids':>4} {'surv':>5} {'active':>6} "
              f"{'remem':>6} {'deleted':>7} {'classes_present':>15} "
              f"{'multi_surv_classes'}")
        for r in RADII:
            res = run_object_memory(
                poses, dets, ts, assoc_trans_gate_m=r, assoc_rot_gate_deg=-1.0,
                cam_K=cam_K, img_size=img)
            by_cls, dom = summarize(res)
            total_ids = sum(c["_ids"] for c in by_cls.values())
            n_active = sum(c.get("active", 0) for c in by_cls.values())
            n_remem = sum(c.get("remembered", 0) for c in by_cls.values())
            n_del = sum(c.get("deleted", 0) for c in by_cls.values())
            surv = n_active + n_remem
            present = [cls for cls, c in by_cls.items()
                       if (c.get("active", 0) + c.get("remembered", 0)) > 0]
            multi = [cls for cls in present
                     if (by_cls[cls].get("active", 0)
                         + by_cls[cls].get("remembered", 0)) > 1]
            print(f"{r:7.2f} {total_ids:4d} {surv:5d} {n_active:6d} "
                  f"{n_remem:6d} {n_del:7d} {len(present):15d}   {multi}")
        # per-class dominant-obs at production radius 0.20
        res = run_object_memory(
            poses, dets, ts, assoc_trans_gate_m=0.20, assoc_rot_gate_deg=-1.0,
            cam_K=cam_K, img_size=img)
        by_cls, dom = summarize(res)
        print("  per-class @0.20  (surviving_instances / total_ids / dom_obs):")
        for cls in sorted(by_cls):
            c = by_cls[cls]
            s = c.get("active", 0) + c.get("remembered", 0)
            print(f"    {cls:22s} surv={s} ids={c['_ids']} dom_obs={dom[cls]}")


if __name__ == "__main__":
    main()
