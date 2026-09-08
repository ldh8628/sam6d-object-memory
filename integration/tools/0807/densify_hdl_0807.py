#!/usr/bin/env python3
"""hdl_graph_slam 궤적을 스캔매칭 odometry 레이트로 조밀화하고, 카메라 프레임으로 옮긴다.

hdl 이 저장하는 것은 키프레임(여기선 203개 ≈ 0.7 Hz)이라 객체 융합의 시간 허용치
(0.2 s)에 대부분 걸리지 않는다. 스캔매칭 odometry 는 9.9 Hz 로 돌지만 드리프트하므로,
키프레임마다 그래프가 적용한 보정량을 뽑아 odometry 에 입힌다:

    C_k = estimate_k · odom_k^-1
    trajectory(t) = interp(C, t) · odom(t)

구성은 data_slam/0724_chungbuk/densify_lidar_trajectory.py 와 동일하고 그 헬퍼를 쓴다.

그 다음, hdl 은 **LiDAR 프레임**이라 카메라 검출과 융합할 수 없다. T_velo_cam 은
0724·260804 에서도 캘리브로 확정되지 않았으므로, ORB 카메라 궤적과 hand-eye(AX=ZB)
를 풀어 추정한다. **캘리브가 아니라 궤적 정합이다** — 결과 객체맵은 ORB 궤적에 부분적
으로 의존한다.

    conda activate hdl_graph_slam_humble
    python3 densify_hdl_0807.py --pair 0807_185223_circle8
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트
OUT_ROOT = ROOT / "integration" / "output"
# 조밀화 헬퍼는 0724 작업 때 만든 것을 그대로 쓴다 (2026-08-25 tar 에서 회수)
H0724 = ROOT / "integration" / "tools" / "0724"
sys.path.insert(0, str(H0724))

from densify_lidar_trajectory import (read_odom, read_corrections,   # noqa: E402
                                      quat_to_R, slerp)
from dump_hdl_graph import R_to_quat                                 # noqa: E402


def densify(d: Path) -> int:
    ots, oT = read_odom(d / "hdl_recorded_bag")
    cts, cq, ct = read_corrections(d / "graph_dump")
    if not len(ots) or not len(cts):
        print(f"odometry {len(ots)} / corrections {len(cts)} — 하나가 비었다", file=sys.stderr)
        return 0
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
    out = d / "trajectory_velo_dense.txt"
    out.write_text("\n".join(rows) + "\n")
    xyz = np.array([[float(v) for v in r.split()[1:4]] for r in rows])
    path = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
    span = float(ots[-1] - ots[0])
    print(f"조밀화: {len(rows)} poses @ {len(ots)/max(span,1e-9):.1f} Hz "
          f"(키프레임 보정 {len(cts)}개, 경로 {path:.2f} m) -> {out.name}")
    return len(rows)


def to_camera_frame(d: Path, orb_traj: Path) -> dict:
    """hand-eye 로 X = T_cam_velo 를 풀고 궤적을 카메라 프레임으로 옮긴다."""
    velo = d / "trajectory_velo_dense.txt"
    rig_json = d / "rig_offset_velo_cam.json"
    cmd = [sys.executable, str(ROOT / "integration" / "rig_offset.py"),
           "--a", str(orb_traj), "--b", str(velo),
           "--label", "orb_cam <- hdl_velo", "--out", str(rig_json)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout.strip() or r.stderr.strip())
    if not rig_json.exists():
        raise SystemExit("hand-eye 실패")
    info = json.loads(rig_json.read_text())
    X = np.array(info["X_camA_camB"])            # T_cam_velo
    Xinv = np.linalg.inv(X)                      # T_velo_cam
    # rig_offset 규약: B(t) 는 A(t + tau) 와 짝이다. 즉 LiDAR 스탬프를 카메라 시계로
    # 옮기려면 tau 를 더한다. 여기서 tau 는 -0.1 s 근처로 나오는데, 이는 Velodyne 이
    # 스윕 '끝'에 스탬프를 찍고 10 Hz 스윕이 0.1 s 길다는 것과 정확히 맞는다.
    tau = float(info.get("tau_s", 0.0))
    T = np.loadtxt(velo)
    rows = []
    for r_ in T:
        P = np.eye(4)
        P[:3, :3] = quat_to_R(r_[4:8])
        P[:3, 3] = r_[1:4]
        Q = P @ Xinv                             # T_mapHDL_cam = T_mapHDL_velo · T_velo_cam
        q = R_to_quat(Q[:3, :3])
        rows.append(f"{r_[0] + tau:.9f} {Q[0,3]:.6f} {Q[1,3]:.6f} {Q[2,3]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")
    print(f"시간 보정 tau = {tau:+.3f} s 를 LiDAR 스탬프에 적용")
    (d / "trajectory.txt").write_text("\n".join(rows) + "\n")
    info["note"] = ("T_velo_cam 은 ORB 카메라 궤적과의 hand-eye 정합으로 추정한 값이다. "
                    "캘리브가 아니므로 이 객체맵은 ORB 궤적에 부분적으로 의존한다.")
    rig_json.write_text(json.dumps(info, indent=1, ensure_ascii=False))
    print(f"카메라 프레임 궤적 {len(rows)} poses -> {d/'trajectory.txt'}")
    return info


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pair", required=True)
    ap.add_argument("--orb-traj", default="")
    a = ap.parse_args()
    d = OUT_ROOT / a.pair / "slam_hdl"
    orb = Path(a.orb_traj) if a.orb_traj else OUT_ROOT / a.pair / "slam_orb3" / "trajectory.txt"
    if not densify(d):
        return 1
    to_camera_frame(d, orb)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
