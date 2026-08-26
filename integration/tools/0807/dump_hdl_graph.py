#!/usr/bin/env python3
"""hdl_graph_slam 의 **최적화된** 그래프를 받아 TUM 궤적으로 푼다.

    python3 dump_hdl_graph.py --dest <graph_dump 디렉터리> \
        --out trajectory_velo.txt [--odom-out odometry_velo.txt]

노드를 죽이기 전에 불러야 한다 — 죽은 뒤에는 최적화 결과가 사라진다.
`/hdl_graph_slam/dump_graph` (hdl_graph_slam/srv/DumpGraph) 를 호출하면 노드가
`<dest>/{graph.g2o, special_nodes.csv, 000000/data, 000001/data, ...}` 를 쓰고,
각 키프레임 `data` 는 다음 형식이다(src/hdl_graph_slam/keyframe.cpp:22):

    stamp <sec> <nsec>
    estimate\n<4x4 최적화 pose>
    odom\n<4x4 스캔매칭 odometry>
    accum_distance <d>
    [floor_coeffs ...] [utm_coord ...]
    id <i>

`estimate` 는 루프클로저·바닥구속이 반영된 최적화 pose, `odom` 은 보정 전 값이다.
둘 다 **velodyne 프레임**이다.
"""
import argparse
import sys
from pathlib import Path

import numpy as np


def quat_from_R(R):
    """회전행렬 -> (qx,qy,qz,qw). 수치적으로 안전한 분기형."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


def call_dump_service(dest: Path, timeout: float = 60.0) -> bool:
    import rclpy
    from rclpy.node import Node
    from hdl_graph_slam.srv import DumpGraph

    rclpy.init()
    node = Node("dump_hdl_graph")
    try:
        cli = node.create_client(DumpGraph, "/hdl_graph_slam/dump_graph")
        if not cli.wait_for_service(timeout_sec=timeout):
            print("dump_graph 서비스가 없다 — 노드가 이미 죽었나?", file=sys.stderr)
            return False
        req = DumpGraph.Request()
        req.destination = str(dest)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            print("dump_graph 응답 없음", file=sys.stderr)
            return False
        return bool(fut.result().success)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_keyframe(path: Path):
    """<dump>/NNNNNN/data -> (stamp_sec, T_estimate 4x4, T_odom 4x4)"""
    tok = path.read_text().split("\n")
    stamp, est, odom = None, None, None
    i = 0
    while i < len(tok):
        line = tok[i].strip()
        if line.startswith("stamp"):
            _, sec, nsec = line.split()
            stamp = int(sec) + int(nsec) * 1e-9
        elif line == "estimate":
            est = np.array([[float(x) for x in tok[i + 1 + r].split()] for r in range(4)])
            i += 4
        elif line == "odom":
            odom = np.array([[float(x) for x in tok[i + 1 + r].split()] for r in range(4)])
            i += 4
        i += 1
    return stamp, est, odom


def write_tum(rows, out: Path):
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        for t, T in rows:
            q = quat_from_R(T[:3, :3])
            f.write(f"{t:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dest", required=True, help="그래프를 덤프받을 디렉터리")
    ap.add_argument("--out", required=True, help="최적화 궤적 TUM 출력")
    ap.add_argument("--odom-out", default="", help="보정 전 odometry TUM 출력")
    ap.add_argument("--no-service", action="store_true",
                    help="서비스를 부르지 않고 이미 있는 --dest 를 파싱만 한다")
    a = ap.parse_args()

    dest = Path(a.dest).expanduser().resolve()
    if not a.no_service:
        if not call_dump_service(dest):
            print("dump_graph 실패", file=sys.stderr)
            return 1

    kfs = sorted(d for d in dest.glob("[0-9]" * 6) if (d / "data").is_file())
    if not kfs:
        print(f"키프레임이 없다: {dest}", file=sys.stderr)
        return 1

    traj, odom = [], []
    for d in kfs:
        t, est, od = parse_keyframe(d / "data")
        if t is None or est is None:
            continue
        traj.append((t, est))
        if od is not None:
            odom.append((t, od))

    write_tum(traj, Path(a.out))
    if a.odom_out and odom:
        write_tum(odom, Path(a.odom_out))
    print(f"keyframes={len(kfs)} trajectory={len(traj)} odometry={len(odom)} -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
