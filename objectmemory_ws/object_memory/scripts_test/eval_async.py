"""Phase A verification: streaming (real-time) vs offline batch on the 4 bags.

Three claims, quantified:
  1. PARITY      -- streaming with interpolation OFF and a realistic latency
                    reproduces the batch result exactly (same surviving
                    landmarks, same poses). Proves the refactor + engine are
                    behaviour-preserving.
  2. ABLATION    -- turning time alignment OFF (fuse at ARRIVAL pose instead of
                    CAPTURE pose) smears landmarks away from their batch
                    position. Demonstrates why time alignment matters (concept 3).
  3. LATENCY SWEEP -- with time alignment ON, landmark drift stays flat as SAM
                    latency grows (robust); the ablation grows with latency.
"""
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "src")))

from pipeline.object_memory_runner import run_object_memory     # noqa: E402
from core.state_machine import ObjectStatus                     # noqa: E402
from run_realtime_object_memory import run_streaming            # noqa: E402
from eval_confidence import DATASETS, load, metrics             # noqa: E402

SURVIVE = {ObjectStatus.active, ObjectStatus.remembered}
# shared params for every run so batch and streaming are compared apples-to-apples
COMMON = dict(assoc_trans_gate_m=0.20, assoc_rot_gate_deg=-1.0,
              quality_weighting=True)


def survivors(res):
    """List of surviving landmarks as (name, x, y, z, status)."""
    out = []
    for lm in res.store.all_landmarks():
        if lm.status in SURVIVE:
            out.append((lm.object_name,
                        lm.T_map_obj[0][3], lm.T_map_obj[1][3],
                        lm.T_map_obj[2][3], lm.status.value))
    return out


def signature(res, nd=4):
    return sorted((n, round(x, nd), round(y, nd), round(z, nd), s)
                  for n, x, y, z, s in survivors(res))


def match_drift(ref, other):
    """Max/mean position delta matching each 'other' survivor to the nearest
    same-name 'ref' survivor (a proxy for landmark smearing)."""
    ref_by = defaultdict(list)
    for n, x, y, z, _ in survivors(ref):
        ref_by[n].append((x, y, z))
    deltas = []
    for n, x, y, z, _ in survivors(other):
        cands = ref_by.get(n)
        if not cands:
            continue
        d = min(((x-rx)**2+(y-ry)**2+(z-rz)**2) ** 0.5 for rx, ry, rz in cands)
        deltas.append(d)
    if not deltas:
        return 0.0, 0.0
    return max(deltas), sum(deltas) / len(deltas)


def main():
    cache = {ds: load(ds) for ds in DATASETS}

    print("\n########## 1. PARITY  (streaming interp=OFF, align=ON, L=0.15s "
          "== batch?) ##########")
    print(f"{'bag':>14} {'batchSurv':>9} {'streamSurv':>10} {'sigMatch':>8} "
          f"{'maxDrift(m)':>11}")
    all_match = True
    refs = {}
    for ds in DATASETS:
        poses, dets, ts, cam_K, img = cache[ds]
        ref = run_object_memory(poses, dets, ts, cam_K=cam_K, img_size=img,
                                **COMMON)
        refs[ds] = ref
        st = run_streaming(poses, dets, ts, latency=0.15, time_align=True,
                           interpolate=False, cam_K=cam_K, img_size=img,
                           **COMMON)
        match = signature(ref) == signature(st)
        all_match = all_match and match
        mx, _ = match_drift(ref, st)
        print(f"{ds:>14} {len(survivors(ref)):9d} {len(survivors(st)):10d} "
              f"{'YES' if match else 'NO':>8} {mx:11.5f}")
    print(f"  => PARITY {'HOLDS (streaming == batch)' if all_match else 'BROKEN'}")

    print("\n########## 2. INTERPOLATION effect (interp=ON vs batch) ##########")
    print(f"{'bag':>14} {'streamSurv':>10} {'maxDrift(m)':>11} {'meanDrift(m)':>12}")
    for ds in DATASETS:
        poses, dets, ts, cam_K, img = cache[ds]
        st = run_streaming(poses, dets, ts, latency=0.15, time_align=True,
                           interpolate=True, cam_K=cam_K, img_size=img, **COMMON)
        mx, mn = match_drift(refs[ds], st)
        print(f"{ds:>14} {len(survivors(st)):10d} {mx:11.5f} {mn:12.5f}")

    print("\n########## 3. ABLATION  (time alignment OFF -> smearing) ##########")
    print(f"{'bag':>14} {'alignSurv':>9} {'ablSurv':>7} {'ablMaxDrift':>11} "
          f"{'ablMeanDrift':>12} {'ablCamOvlp':>10}")
    for ds in DATASETS:
        poses, dets, ts, cam_K, img = cache[ds]
        on = run_streaming(poses, dets, ts, latency=0.15, time_align=True,
                           interpolate=True, cam_K=cam_K, img_size=img, **COMMON)
        off = run_streaming(poses, dets, ts, latency=0.15, time_align=False,
                            interpolate=True, cam_K=cam_K, img_size=img, **COMMON)
        mx, mn = match_drift(refs[ds], off)
        m_off = metrics(off, poses)
        print(f"{ds:>14} {len(survivors(on)):9d} {len(survivors(off)):7d} "
              f"{mx:11.5f} {mn:12.5f} {m_off['cam_overlap']:10d}")

    print("\n########## 4. LATENCY SWEEP (align ON, interp ON): drift vs L "
          "##########")
    print(f"{'bag':>14} " + "".join(f"{('L='+str(L)):>12}" for L in
                                     (0.0, 0.2, 0.5, 1.0)))
    for ds in DATASETS:
        poses, dets, ts, cam_K, img = cache[ds]
        row = []
        for L in (0.0, 0.2, 0.5, 1.0):
            st = run_streaming(poses, dets, ts, latency=L, time_align=True,
                               interpolate=True, cam_K=cam_K, img_size=img,
                               **COMMON)
            mx, _ = match_drift(refs[ds], st)
            row.append(f"{mx:12.5f}")
        print(f"{ds:>14} " + "".join(row))


if __name__ == "__main__":
    main()
