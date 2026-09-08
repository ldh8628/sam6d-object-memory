#!/usr/bin/env python3
"""Pull hdl_graph_slam's optimised pose graph out and write a TUM trajectory.

`export_hdl_results.sh` in this repo shells out to a C++ client that used to
live in /tmp and is long gone, so this replaces it with a plain rclpy client for
the `/hdl_graph_slam/dump_graph` service.

Calling it while the node is still alive is important: the graph only exists in
memory, and it is the *optimised* estimate (loop closures applied) rather than
the raw scan-matching odometry that we want as a reference trajectory. The
service writes one directory per keyframe, each holding a `data` file:

    stamp <sec> <nsec>
    estimate
    <4x4 row-major matrix, one row per line>
    odom
    <4x4 ...>
    accum_distance <m>
    ...

which we turn into `timestamp tx ty tz qx qy qz qw` (TUM), matching what the
RGB-D runs already write so the existing evaluation code just works.

    python3 dump_hdl_graph.py --dest <dir>                # call service + parse
    python3 dump_hdl_graph.py --parse-only --dest <dir>   # parse an earlier dump
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def parse_matrix(lines: list[str], start: int) -> tuple[np.ndarray, int]:
    rows = []
    i = start
    while i < len(lines) and len(rows) < 4:
        parts = lines[i].split()
        if len(parts) == 4:
            rows.append([float(x) for x in parts])
        i += 1
    return np.array(rows), i


def parse_keyframe(data_file: Path) -> dict | None:
    lines = data_file.read_text().splitlines()
    out: dict = {}
    i = 0
    while i < len(lines):
        tok = lines[i].split()
        if not tok:
            i += 1
            continue
        if tok[0] == "stamp" and len(tok) >= 3:
            out["stamp"] = int(tok[1]) + int(tok[2]) * 1e-9
            i += 1
        elif tok[0] == "estimate":
            out["estimate"], i = parse_matrix(lines, i + 1)
        elif tok[0] == "odom":
            out["odom"], i = parse_matrix(lines, i + 1)
        elif tok[0] == "accum_distance" and len(tok) >= 2:
            out["accum_distance"] = float(tok[1])
            i += 1
        else:
            i += 1
    if "stamp" not in out or out.get("estimate") is None or out["estimate"].shape != (4, 4):
        return None
    return out


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> (qx, qy, qz, qw), branchful form for numerical safety."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def write_tum(dest: Path, out_path: Path, key: str = "estimate") -> int:
    kfs = []
    for d in sorted(dest.iterdir()):
        f = d / "data"
        if d.is_dir() and f.exists():
            kf = parse_keyframe(f)
            if kf is not None:
                kfs.append(kf)
    if not kfs:
        print(f"!! no keyframes parsed under {dest}")
        return 0
    kfs.sort(key=lambda k: k["stamp"])
    rows = []
    for kf in kfs:
        T = kf[key]
        q = R_to_quat(T[:3, :3])
        rows.append(f"{kf['stamp']:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(rows) + "\n")
    span = kfs[-1]["stamp"] - kfs[0]["stamp"]
    print(f"-> {out_path}  ({len(rows)} keyframes, {span:.1f} s)")
    return len(rows)


def call_dump(dest: Path, timeout: float) -> bool:
    import rclpy
    from rclpy.node import Node
    from hdl_graph_slam.srv import DumpGraph

    rclpy.init()
    node = Node("hdl_graph_dump_client")
    cli = node.create_client(DumpGraph, "/hdl_graph_slam/dump_graph")
    ok = False
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            print("!! /hdl_graph_slam/dump_graph never appeared")
        else:
            req = DumpGraph.Request()
            req.destination = str(dest)
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
            if fut.done() and fut.result() is not None:
                ok = bool(fut.result().success)
                print(f"dump_graph success={ok} dest={dest}")
            else:
                print("!! dump_graph call timed out")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dest", required=True, type=Path,
                    help="directory the service writes the graph into")
    ap.add_argument("--out", type=Path, default=None,
                    help="TUM trajectory path (default <dest>/../trajectory.txt)")
    ap.add_argument("--parse-only", action="store_true")
    ap.add_argument("--odom-out", type=Path, default=None,
                    help="also write the raw scan-matching odometry for comparison")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    if not args.parse_only and not call_dump(args.dest, args.timeout):
        return 1
    if not args.dest.exists():
        print(f"!! {args.dest} does not exist")
        return 1

    out = args.out or args.dest.parent / "trajectory.txt"
    n = write_tum(args.dest, out, "estimate")
    if args.odom_out:
        write_tum(args.dest, args.odom_out, "odom")
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
