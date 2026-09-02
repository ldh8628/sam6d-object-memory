#!/usr/bin/env python3
"""RTAB-Map / hdl_graph_slam 궤적을 프레임레이트로 조밀화한다.

두 백엔드 모두 저장되는 것은 **그래프 노드**(RTAB ~1 Hz, hdl 키프레임)라서 그대로
객체 융합에 넣으면 검출 대부분이 시간 허용치(0.2 s) 밖으로 떨어진다. 두 경우 다
같은 방법으로 고친다 — 노드에서 보정량을 뽑아 프레임레이트 odometry 에 입힌다:

    C_k = optimized_pose_k · odometry_pose_k^-1        (노드마다)
    trajectory(t) = interp(C, t) · odometry(t)

구성은 data_slam/0724_chungbuk/densify_rtabmap_trajectory.py 와 동일하며 그 헬퍼를
그대로 재사용한다. 출력만 integration/output/<pair>/slam_<method>/ 로 향한다.

    conda activate hdl_graph_slam_humble   # rosbag2_py + numpy 있는 환경이면 됨
    python3 densify_0807.py --pair 0807_185223_circle8 --method rtabmap
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트
OUT_ROOT = ROOT / "integration" / "output"
# 조밀화 헬퍼는 0724 작업 때 만든 것을 그대로 쓴다 (2026-08-25 tar 에서 회수)
H0724 = ROOT / "integration" / "tools" / "0724"
sys.path.insert(0, str(H0724))

from densify_rtabmap_trajectory import (node_odom_poses,        # noqa: E402
                                        optimized_poses, read_odom,
                                        quat_to_R, R_to_quat, slerp)


def densify(cts, cq, ct, ots, oT) -> list[str]:
    rows = []
    for t, O in zip(ots, oT):
        j = int(np.searchsorted(cts, t))
        if j <= 0:
            q, tr = cq[0], ct[0]
        elif j >= len(cts):
            q, tr = cq[-1], ct[-1]
        else:
            span = cts[j] - cts[j - 1]
            u = 0.0 if span <= 0 else float((t - cts[j - 1]) / span)
            q = slerp(cq[j - 1], cq[j], u)
            tr = ct[j - 1] + u * (ct[j] - ct[j - 1])
        C = np.eye(4)
        C[:3, :3] = quat_to_R(q)
        C[:3, 3] = tr
        M = C @ O
        qq = R_to_quat(M[:3, :3])
        rows.append(f"{t:.9f} {M[0,3]:.6f} {M[1,3]:.6f} {M[2,3]:.6f} "
                    f"{qq[0]:.6f} {qq[1]:.6f} {qq[2]:.6f} {qq[3]:.6f}")
    return rows


def corrections(pairs):
    cts = np.array([p[0] for p in pairs])
    cq, ct = [], []
    for _s, T_opt, T_odom in pairs:
        C = T_opt @ np.linalg.inv(T_odom)
        cq.append(R_to_quat(C[:3, :3]))
        ct.append(C[:3, 3])
    return cts, np.array(cq), np.array(ct)


def run_rtabmap(d: Path) -> bool:
    db, bag, opt_f = d / "rtabmap.db", d / "odom_bag", d / "rtabmap_dense_poses.txt"
    missing = [p.name for p in (db, bag, opt_f) if not p.exists()]
    if missing:
        print(f"없는 파일: {missing}", file=sys.stderr)
        return False
    odom_by_id = node_odom_poses(db)
    opt = optimized_poses(opt_f)
    pairs = [(s, T, odom_by_id[i]) for s, i, T in opt if i in odom_by_id]
    if not pairs:
        print("export 와 db 사이에 일치하는 노드가 없다", file=sys.stderr)
        return False
    cts, cq, ct = corrections(pairs)
    ots, oT = read_odom(bag)
    if not len(ots):
        print("/rtabmap/odom 이 비었다", file=sys.stderr)
        return False
    rows = densify(cts, cq, ct, ots, oT)
    (d / "trajectory.txt").write_text("\n".join(rows) + "\n")
    report(d, rows, ots, len(pairs))
    return True


def report(d: Path, rows, ots, n_nodes: int) -> None:
    xyz = np.array([[float(v) for v in r.split()[1:4]] for r in rows])
    path = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
    span = float(ots[-1] - ots[0])
    print(f"{d.name}: {len(rows)} poses @ {len(ots)/max(span,1e-9):.1f} Hz "
          f"(노드 보정 {n_nodes}개, 최대 간격 {np.diff(ots).max():.2f} s, "
          f"경로 {path:.2f} m) -> {d/'trajectory.txt'}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pair", required=True)
    ap.add_argument("--method", choices=["rtabmap"], default="rtabmap",
                    help="hdl 은 dump_hdl_graph.py 가 이미 odom+보정을 함께 내므로 "
                         "여기서 다루지 않는다")
    a = ap.parse_args()
    d = OUT_ROOT / a.pair / f"slam_{a.method}"
    if not d.is_dir():
        print(f"없음: {d}", file=sys.stderr)
        return 2
    return 0 if run_rtabmap(d) else 1


if __name__ == "__main__":
    raise SystemExit(main())
