"""Phase C1 -- live long-term memory map on the 4 bags.

Feeds the FULL dense SLAM pose stream (all poses, not stride-10 sync points) plus
the periodic SAM-6D detections through the streaming engine in arrival order,
with a realistic measured SAM latency (ISM ~0.12s + PEM ~0.065s/det ~= 0.2s), and
records the resulting persistent long-term map (active + remembered landmarks) --
the SLAM-corrected object poses.

This is the runnable form of the "real-time, all frames" experiment on currently
available SAM density. Raising SAM density to true stride-1 needs a SAM-pipeline
re-run (sam6d_ws) and is deferred; the SLAM side already streams every pose here.
"""
import os
import sys

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "src")))

from run_realtime_object_memory import run_streaming            # noqa: E402
from eval_confidence import DATASETS, load, metrics            # noqa: E402
from core.state_machine import ObjectStatus                    # noqa: E402

SURVIVE = {ObjectStatus.active, ObjectStatus.remembered}
SAM_LATENCY_S = 0.2   # measured: ISM 0.12 + PEM ~0.065/det (warm persistent)
COMMON = dict(assoc_trans_gate_m=0.20, assoc_rot_gate_deg=-1.0,
              quality_weighting=True)


def main():
    for ds in DATASETS:
        poses, dets, ts, cam_K, img = load(ds)
        res = run_streaming(poses, dets, ts, latency=SAM_LATENCY_S,
                            time_align=True, interpolate=True,
                            cam_K=cam_K, img_size=img, **COMMON)
        m = metrics(res, poses)
        print(f"\n================= {ds}  (live, L={SAM_LATENCY_S}s) "
              f"=================")
        print(f"  SLAM poses streamed : {len(poses)}")
        print(f"  SAM frames (periodic): {res.frames_total}  "
              f"fused={res.frames_with_slam}  dropped={res.frames_without_slam}")
        print(f"  ghost lifespan med/max: {m['life_med']}/{m['life_max']} frames "
              f"  camOverlap={m['cam_overlap']}")
        print(f"  --- LONG-TERM MEMORY MAP (active + remembered) ---")
        print(f"  {'id':>3} {'name':<16} {'status':<11} {'obs':>4} "
              f"{'conf':>5}  xyz (m)")
        rows = [lm for lm in res.store.all_landmarks() if lm.status in SURVIVE]
        for lm in sorted(rows, key=lambda x: (x.object_name, x.object_id)):
            x, y, z = (lm.T_map_obj[0][3], lm.T_map_obj[1][3], lm.T_map_obj[2][3])
            print(f"  {lm.object_id:>3} {lm.object_name:<16} {lm.status.value:<11} "
                  f"{lm.observation_count:>4} {lm.confidence:>5.2f}  "
                  f"({x:.3f}, {y:.3f}, {z:.3f})")
        act = sum(1 for lm in rows if lm.status is ObjectStatus.active)
        rem = sum(1 for lm in rows if lm.status is ObjectStatus.remembered)
        print(f"  => {len(rows)} landmarks  (active={act} remembered={rem})")


if __name__ == "__main__":
    main()
