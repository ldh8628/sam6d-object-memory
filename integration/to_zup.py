#!/usr/bin/env python3
"""카메라 광학 프레임(x=오른쪽, y=아래, z=앞) 점군을 ROS z-up(REP-103)으로 바꾼다.

ORB-SLAM3 / RTAB-Map 의 map 프레임은 첫 카메라의 **광학 프레임**이라 y 가 아래를
가리킨다. CloudCompare·MeshLab·Open3D 는 보통 y-up 이나 z-up 을 가정하므로, 그대로
열면 바닥과 천장이 뒤집혀 보인다. 데이터가 틀린 게 아니라 규약이 다른 것이다.

    x_ros = +z_cam      (앞)
    y_ros = -x_cam      (왼쪽)
    z_ros = -y_cam      (위)

점군(.pcd/.ply)과 TUM 궤적 모두 같은 회전을 적용한다.

    python3 integration/to_zup.py integration/output/<pair>
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# 광학 -> ROS
R = np.array([[0.0, 0.0, 1.0],
              [-1.0, 0.0, 0.0],
              [0.0, -1.0, 0.0]])


def convert_pcd(src: Path, dst: Path) -> int:
    head, n = [], 0
    with src.open() as f, dst.open("w") as o:
        for ln in f:
            head.append(ln)
            o.write(ln)
            if ln.startswith("DATA"):
                break
        if "ascii" not in head[-1]:
            raise SystemExit(f"{src.name}: ascii PCD 만 지원한다")
        for ln in f:
            v = ln.split()
            if len(v) < 3:
                continue
            try:
                p = R @ np.array([float(v[0]), float(v[1]), float(v[2])])
            except ValueError:
                continue
            o.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f}"
                    + ("" if len(v) == 3 else " " + " ".join(v[3:])) + "\n")
            n += 1
    return n


def convert_ply(src: Path, dst: Path) -> int:
    n = 0
    with src.open() as f, dst.open("w") as o:
        for ln in f:
            o.write(ln)
            if ln.strip() == "end_header":
                break
        for ln in f:
            v = ln.split()
            if len(v) < 3:
                continue
            try:
                p = R @ np.array([float(v[0]), float(v[1]), float(v[2])])
            except ValueError:
                continue
            rest = v[3:]
            # normals(nx,ny,nz)가 있으면 같이 돌린다
            if len(rest) >= 3:
                try:
                    nvec = R @ np.array([float(rest[0]), float(rest[1]), float(rest[2])])
                    rest = [f"{nvec[0]:.5f}", f"{nvec[1]:.5f}", f"{nvec[2]:.5f}"] + rest[3:]
                except ValueError:
                    pass
            o.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f}"
                    + ("" if not rest else " " + " ".join(rest)) + "\n")
            n += 1
    return n


def convert_traj(src: Path, dst: Path) -> int:
    T = np.loadtxt(src)
    if T.ndim == 1:
        T = T[None, :]
    rows = []
    for r in T:
        p = R @ r[1:4]
        x, y, z, w = r[4:8]
        M = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        M = R @ M
        t = np.trace(M)
        if t > 0:
            s = np.sqrt(t + 1) * 2
            q = [(M[2, 1] - M[1, 2]) / s, (M[0, 2] - M[2, 0]) / s,
                 (M[1, 0] - M[0, 1]) / s, 0.25 * s]
        else:
            i = int(np.argmax(np.diag(M))); j, k = (i + 1) % 3, (i + 2) % 3
            s = np.sqrt(M[i, i] - M[j, j] - M[k, k] + 1) * 2
            qq = [0.0, 0.0, 0.0]
            qq[i] = 0.25 * s
            qq[j] = (M[j, i] + M[i, j]) / s
            qq[k] = (M[k, i] + M[i, k]) / s
            q = qq + [(M[k, j] - M[j, k]) / s]
        q = np.array(q) / np.linalg.norm(q)
        rows.append(f"{r[0]:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")
    dst.write_text("\n".join(rows) + "\n")
    return len(rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", help="integration/output/<pair>")
    ap.add_argument("--backends", nargs="*", default=["orb3", "rtabmap"],
                    help="hdl 은 이미 ROS z-up 이라 기본에서 뺀다")
    a = ap.parse_args()
    root = Path(a.out_dir)
    if not root.is_dir():
        print(f"없음: {root}", file=sys.stderr)
        return 2
    for b in a.backends:
        d = root / f"slam_{b}"
        if not d.is_dir():
            continue
        for p in sorted(d.glob("*.pcd")) + sorted(d.glob("*.ply")) + [d / "trajectory.txt"]:
            if not p.is_file() or p.stem.endswith("_zup"):
                continue
            dst = p.with_name(p.stem + "_zup" + p.suffix)
            n = (convert_pcd(p, dst) if p.suffix == ".pcd" else
                 convert_ply(p, dst) if p.suffix == ".ply" else
                 convert_traj(p, dst))
            print(f"{b}: {p.name} -> {dst.name}  ({n} rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
