#!/usr/bin/env python3
"""Densify a 260804 hdl_graph_slam run: per-scan trajectory + full-resolution map.

hdl_graph_slam leaves two things behind that are each half of what we want:

  * `graph_dump/<id>/data` — 113 keyframes carrying both the pose the graph
    optimisation settled on (`estimate`) and the odometry pose it came from
    (`odom`). Loop-corrected, but only 1.6 Hz, and hdl gates keyframes on
    distance travelled, so a stationary stretch produces none at all.
  * `hdl_recorded_bag` — `/odom` from scan-matching at the scan rate (10 Hz),
    which drifts because nothing corrects it.

`C_k = estimate_k * odom_k^-1` is the correction the graph applied at keyframe k;
interpolating C between keyframes (slerp + lerp) and applying it to the dense
odometry gives a loop-corrected trajectory at the scan rate. This is exactly
data_slam/0724_chungbuk/densify_lidar_trajectory.py, re-pointed at the 260804
output layout instead of slam_comparison/output/.

`--map` then reuses those dense poses to rebuild the map from the *raw*
`/velodyne_points` in the source bag. build_hdl_map.py can only use the clouds in
the dump, which hdl has already prefiltered down to ~5 k points per keyframe;
going back to the bag gives all 27 k points of every one of the 721 sweeps.

    conda activate hdl_graph_slam_humble
    python3 densify_hdl_260804.py longcircle2 --map
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트
OUT_ROOT = ROOT / "output_slam" / "260804_office" / "hdl_graph_slam"
MANIFEST = HERE / "manifest_260804.json"

sys.path.insert(0, str(ROOT / "data_slam" / "0724_chungbuk"))
from densify_lidar_trajectory import quat_to_R, slerp          # noqa: E402
from dump_hdl_graph import parse_keyframe, R_to_quat           # noqa: E402
from build_hdl_map import voxel_downsample, write_pcd          # noqa: E402


def bag_reader(path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    return reader


def read_odom(bag: Path):
    reader = bag_reader(bag)
    ts, T = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/odom":
            continue
        m = deserialize_message(data, Odometry)
        p, o = m.pose.pose.position, m.pose.pose.orientation
        if not np.isfinite([p.x, p.y, p.z, o.x, o.y, o.z, o.w]).all():
            continue
        M = np.eye(4)
        M[:3, :3] = quat_to_R([o.x, o.y, o.z, o.w])
        M[:3, 3] = [p.x, p.y, p.z]
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        T.append(M)
    order = np.argsort(ts)
    return np.array(ts)[order], np.array(T)[order]


def read_corrections(dump: Path):
    """C_k = estimate_k * odom_k^-1, as (stamp, quaternion, translation)."""
    stamps, quats, trans = [], [], []
    for d in sorted(dump.iterdir()):
        f = d / "data"
        if not (d.is_dir() and f.exists()):
            continue
        kf = parse_keyframe(f)
        if kf is None or kf.get("odom") is None:
            continue
        C = kf["estimate"] @ np.linalg.inv(kf["odom"])
        stamps.append(kf["stamp"])
        quats.append(R_to_quat(C[:3, :3]))
        trans.append(C[:3, 3])
    order = np.argsort(stamps)
    return (np.array(stamps)[order], np.array(quats)[order],
            np.array(trans)[order])


def interp_correction(t: float, cts, cq, ct) -> np.ndarray:
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
    return C


# --------------------------------------------------------------------------
def cloud_to_xyzi(msg: PointCloud2) -> tuple[np.ndarray, np.ndarray | None]:
    """Decode a PointCloud2 into (N,3) float xyz and (N,) intensity."""
    np_type = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
               5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.name in names:
            continue
        names.append(f.name)
        formats.append(np_type[f.datatype])
        offsets.append(f.offset)
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                   "itemsize": msg.point_step})
    arr = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    inten = arr["intensity"].astype(np.float64) if "intensity" in names else None
    return xyz, inten


def build_map(src_bag: Path, lidar_topic: str, cts, cq, ct,
              out_pcd: Path, leaf: float, max_range: float,
              min_range: float) -> dict:
    reader = bag_reader(src_bag)
    chunks_xyz, chunks_i, used = [], [], 0
    raw_n = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != lidar_topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        xyz, inten = cloud_to_xyzi(msg)
        keep = np.isfinite(xyz).all(axis=1)
        rng = np.linalg.norm(xyz, axis=1)
        keep &= rng > min_range                 # drop returns off the platform
        if max_range > 0:
            keep &= rng <= max_range
        xyz = xyz[keep]
        if xyz.shape[0] == 0:
            continue
        inten = inten[keep] if inten is not None else np.zeros(xyz.shape[0])
        # the dense trajectory is the odometry pose corrected by the graph, so
        # apply the same construction here instead of nearest-neighbour lookup
        T = interp_correction(t, cts, cq, ct) @ odom_at(t)
        chunks_xyz.append(xyz @ T[:3, :3].T + T[:3, 3])
        chunks_i.append(inten)
        raw_n += xyz.shape[0]
        used += 1
    if not chunks_xyz:
        return {"ok": False, "reason": f"no {lidar_topic} in {src_bag}"}
    xyz = np.concatenate(chunks_xyz)
    inten = np.concatenate(chunks_i)
    if leaf > 0:
        xyz, inten = voxel_downsample(xyz, inten, leaf)
    write_pcd(out_pcd, xyz, inten)
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    return {"ok": True, "scans": used, "raw_points": int(raw_n),
            "points": int(xyz.shape[0]), "leaf": leaf, "max_range": max_range,
            "min_range": min_range,
            "extent_m": [round(float(v), 2) for v in (hi - lo)],
            "file": str(out_pcd), "mb": round(out_pcd.stat().st_size / 1e6, 1)}


# module-level so build_map can reach the odometry interpolator
_ODOM_TS = _ODOM_T = None


def odom_at(t: float) -> np.ndarray:
    """Odometry pose at t, slerp/lerp between the two bracketing samples."""
    ts, T = _ODOM_TS, _ODOM_T
    j = int(np.searchsorted(ts, t))
    if j <= 0:
        return T[0]
    if j >= len(ts):
        return T[-1]
    span = ts[j] - ts[j - 1]
    u = 0.0 if span <= 0 else float((t - ts[j - 1]) / span)
    q = slerp(R_to_quat(T[j - 1][:3, :3]), R_to_quat(T[j][:3, :3]), u)
    M = np.eye(4)
    M[:3, :3] = quat_to_R(q)
    M[:3, 3] = T[j - 1][:3, 3] + u * (T[j][:3, 3] - T[j - 1][:3, 3])
    return M


def main() -> int:
    global _ODOM_TS, _ODOM_T
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--map", action="store_true",
                    help="also rebuild the map from the raw bag scans")
    ap.add_argument("--leaf", type=float, default=0.03)
    ap.add_argument("--max-range", type=float, default=30.0)
    ap.add_argument("--min-range", type=float, default=0.9,
                    help="drop returns closer than this. Something rigid sits "
                         "0.51-0.72 m behind the Velodyne (mount/operator): its "
                         "returns are ~9 %% of every sweep and, being behind, "
                         "land on the path just travelled, so accumulating them "
                         "draws the trajectory into the map. The old 0.5 m cut "
                         "passed them. Nothing real can be that close either -- "
                         "the -15 deg bottom beam first hits the floor 2.4 m out "
                         "at this sensor height.")
    args = ap.parse_args()

    manifest = {d["name"]: d for d in json.loads(MANIFEST.read_text())}
    rc = 0
    for ses in args.sessions:
        run_dir = OUT_ROOT / ses
        bag, dump = run_dir / "hdl_recorded_bag", run_dir / "graph_dump"
        if not (bag.exists() and dump.exists()):
            print(f"{ses}: missing hdl_recorded_bag or graph_dump", file=sys.stderr)
            rc = 1
            continue

        ots, oT = read_odom(bag)
        cts, cq, ct = read_corrections(dump)
        if not len(ots) or not len(cts):
            print(f"{ses}: no odometry ({len(ots)}) or corrections ({len(cts)})",
                  file=sys.stderr)
            rc = 1
            continue
        _ODOM_TS, _ODOM_T = ots, oT

        rows = []
        for t, O in zip(ots, oT):
            M = interp_correction(t, cts, cq, ct) @ O
            q = R_to_quat(M[:3, :3])
            rows.append(f"{t:.9f} {M[0,3]:.6f} {M[1,3]:.6f} {M[2,3]:.6f} "
                        f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")
        out = run_dir / "trajectory_dense.txt"
        out.write_text("\n".join(rows) + "\n")
        xyz = np.array([[float(v) for v in r.split()[1:4]] for r in rows])
        path = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
        span = ots[-1] - ots[0]
        print(f"{ses}: {len(rows)} poses at {len(ots)/max(span,1e-9):.1f} Hz "
              f"from {len(cts)} keyframe corrections, path {path:.2f} m "
              f"-> {out.name}")

        if args.map:
            ds = manifest.get(ses)
            if ds is None:
                print(f"{ses}: not in manifest, map skipped", file=sys.stderr)
                rc = 1
                continue
            info = build_map(Path(ds["bag"]), ds.get("lidar", "/velodyne_points"),
                             cts, cq, ct, run_dir / "hdl_lidar_map.pcd",
                             args.leaf, args.max_range, args.min_range)
            if info["ok"]:
                print(f"  map: {info['scans']} scans  {info['raw_points']:,} -> "
                      f"{info['points']:,} pts  {info['mb']} MB  "
                      f"extent {info['extent_m']}")
                (run_dir / "hdl_lidar_map.json").write_text(json.dumps(info, indent=1))
            else:
                print(f"  map FAILED: {info['reason']}", file=sys.stderr)
                rc = 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
