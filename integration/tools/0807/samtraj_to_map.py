#!/usr/bin/env python3
"""SAM 카메라 궤적을 SLAM 카메라의 map 프레임으로 옮긴다 (review 영상용).

rig_offset.py 가 준 M = T_mapA_mapB 를 곱하면 된다:

    T_mapSLAM_camSAM(t) = M · T_mapSAM_camSAM(t)

    python3 samtraj_to_map.py --pair 0807_185223_circle8
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]   # integration/tools/<x>/ -> 레포 루트
OUT = ROOT / "integration" / "output"


def q2R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
        qq = [0.0, 0.0, 0.0]
        qq[i] = 0.25 * s
        qq[j] = (R[j, i] + R[i, j]) / s
        qq[k] = (R[k, i] + R[i, k]) / s
        q = qq + [(R[k, j] - R[j, k]) / s]
    q = np.array(q, float)
    return q / np.linalg.norm(q)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pair", required=True)
    a = ap.parse_args()
    o = OUT / a.pair
    rig = json.loads((o / "rig_offset.json").read_text())
    M = np.array(rig["M_mapA_mapB"], float)
    src = OUT / (a.pair + "_samcam") / "slam_orb3" / "trajectory.txt"
    if not src.is_file():
        src = OUT / (a.pair + "_samcam") / "slam" / "trajectory.txt"
    T = np.loadtxt(src)
    rows = []
    for r in T:
        P = np.eye(4); P[:3, :3] = q2R(r[4:8]); P[:3, 3] = r[1:4]
        Q = M @ P
        q = R2q(Q[:3, :3])
        rows.append(f"{r[0]:.9f} {Q[0,3]:.6f} {Q[1,3]:.6f} {Q[2,3]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")
    dst = o / "slam_orb3" / "samcam_trajectory_in_slam_map.txt"
    dst.write_text("\n".join(rows) + "\n")
    print(f"{len(rows)} poses -> {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
