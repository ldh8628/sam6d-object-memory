#!/usr/bin/env python3
"""Cross-check the three SLAM trajectories for the 260804 office sessions.

Each method reports poses in its own frame — ORB-SLAM3 and RTAB-Map in the
colour camera's optical frame (x right, y down, z forward), hdl_graph_slam in the
LiDAR/base frame (z up) — so nothing can be compared component-wise. Instead each
trajectory is reduced to two frame-independent quantities:

  * rotation accumulated about its own dominant rotation axis, which for these
    sessions is the vertical, and which the Xsens measures directly;
  * the path projected onto the plane perpendicular to that axis (the top view).

Both are invariant to how the method happened to orient its world frame, so the
Xsens becomes an external check on all three at once.

Run under `conda activate rtabmap` (numpy + matplotlib + rosbag2_py).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트
OUT = ROOT / "output_slam" / "260804_office"
METHODS = [("orb3_slam", "ORB-SLAM3 (RGB-D)", "tab:green"),
           ("rtabmap", "RTAB-Map (RGB-D)", "tab:orange"),
           ("hdl_graph_slam", "hdl_graph_slam (LiDAR)", "tab:purple")]


def quat_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def log_so3(R: np.ndarray) -> np.ndarray:
    """rotation matrix -> axis*angle."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = float(np.arccos(c))
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * np.sin(th)) * th


def load_tum(path: Path):
    rows = []
    for line in path.read_text(errors="replace").splitlines():
        p = line.split()
        if len(p) >= 8 and not line.startswith("#"):
            try:
                rows.append([float(v) for v in p[:8]])
            except ValueError:
                continue
    if not rows:
        return None
    a = np.asarray(rows)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def reduce_traj(t, xyz, quat):
    """-> (t, cumulative rotation about dominant axis [deg], top-view xy [m])."""
    Rs = [quat_to_R(q) for q in quat]
    steps = np.array([log_so3(Rs[i].T @ Rs[i + 1]) for i in range(len(Rs) - 1)])
    if not len(steps):
        return t, np.zeros(len(t)), xyz[:, :2]
    # dominant axis = principal direction of the incremental rotation vectors
    u, s, vt = np.linalg.svd(steps - 0.0, full_matrices=False)
    axis = vt[0] / np.linalg.norm(vt[0])
    yaw = np.concatenate([[0.0], np.cumsum(steps @ axis)])
    # sign convention: make the net rotation positive so curves are comparable
    if yaw[-1] < 0:
        yaw, axis = -yaw, -axis
    # top view: basis of the plane orthogonal to the rotation axis
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(axis @ tmp) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    e1 = np.cross(axis, tmp); e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    rel = xyz - xyz[0]
    return t, np.degrees(yaw), np.stack([rel @ e1, rel @ e2], axis=1)


def fit_planar_yaw(t_ref, xy_ref, t, xy):
    """Rotation (about the vertical) that puts `xy` in the reference's frame.

    The top-view basis of reduce_traj() is orthogonal to the rotation axis but
    its in-plane orientation is arbitrary, so two methods of the same run come
    out rotated against each other by an unknown angle. That angle is the yaw
    part of the unmeasured LiDAR<->camera extrinsic, and planar motion observes
    it well (DESIGN_lidar_pseudo_gt.md 5.2), so it is fitted from the data
    rather than assumed: with both paths starting at the origin the least
    squares solution is a single closed-form angle.

    Rotation only — no scale (both are metric) and no reflection (that would
    flip handedness, which is a bug to expose, not to hide). The reflected fit
    is reported alongside so a frame that really is mirrored cannot pass
    unnoticed.
    """
    lo, hi = max(t_ref[0], t[0]), min(t_ref[-1], t[-1])
    m = (t >= lo) & (t <= hi)
    if m.sum() < 3:
        return 0.0, None, None
    p = xy[m]
    q = np.stack([np.interp(t[m], t_ref, xy_ref[:, k]) for k in range(2)], axis=1)
    th = float(np.arctan2(np.sum(p[:, 0] * q[:, 1] - p[:, 1] * q[:, 0]),
                          np.sum(p[:, 0] * q[:, 0] + p[:, 1] * q[:, 1])))
    c, s = np.cos(th), np.sin(th)
    rms = float(np.sqrt(np.mean(np.sum((p @ np.array([[c, s], [-s, c]]) - q) ** 2, axis=1))))
    pm = p * np.array([1.0, -1.0])                       # same fit, mirrored
    thm = float(np.arctan2(np.sum(pm[:, 0] * q[:, 1] - pm[:, 1] * q[:, 0]),
                           np.sum(pm[:, 0] * q[:, 0] + pm[:, 1] * q[:, 1])))
    cm, sm = np.cos(thm), np.sin(thm)
    rms_m = float(np.sqrt(np.mean(np.sum((pm @ np.array([[cm, sm], [-sm, cm]]) - q) ** 2, axis=1))))
    return th, rms, rms_m


def rotate2d(xy, th):
    c, s = np.cos(th), np.sin(th)
    return xy @ np.array([[c, s], [-s, c]])


def imu_yaw(npz: Path):
    """Xsens gyro integrated about its vertical axis. Accepts either key set
    (`imu_t/imu_w` from the sync dumps, `t/w` from the GT-eval dumps) and uses
    the real sample spacing rather than a nominal one."""
    z = np.load(npz)
    t = z["imu_t"] if "imu_t" in z else z["t"]
    w = (z["imu_w"] if "imu_w" in z else z["w"])[:, 2]
    dt = np.diff(t, prepend=t[0])
    yaw = np.degrees(np.cumsum(w * dt))
    return t, (-yaw if yaw[-1] < 0 else yaw)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", nargs="+", default=["longcircle2", "stablization"])
    ap.add_argument("--imu-npz", nargs="*", default=[],
                    help="session=path/to/sync_<s>.npz for the Xsens reference")
    ap.add_argument("--out", default=str(OUT / "verification"))
    ap.add_argument("--no-align", dest="align", action="store_false",
                    help="draw every top view in its own arbitrary in-plane "
                         "orientation (the raw frames, before yaw fitting)")
    args = ap.parse_args()
    imu_map = dict(kv.split("=", 1) for kv in args.imu_npz)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    report = {}
    for session in args.sessions:
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        srep = {}
        series = []
        for key, label, colour in METHODS:
            p = OUT / key / session / "trajectory.txt"
            data = load_tum(p) if p.exists() else None
            if data is None:
                srep[key] = {"error": f"missing/empty {p}"}
                continue
            t, xyz, quat = data
            t, yaw, xy = reduce_traj(t, xyz, quat)
            series.append([key, label, colour, t, yaw, xy])
            path_len = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
            srep[key] = {
                "poses": int(len(t)),
                "duration_s": round(float(t[-1] - t[0]), 2),
                "pose_rate_hz": round(len(t) / max(t[-1] - t[0], 1e-9), 2),
                "path_length_m": round(path_len, 2),
                "net_rotation_deg": round(float(yaw[-1]), 1),
                "extent_m": [round(float(xy[:, 0].ptp()), 2),
                             round(float(xy[:, 1].ptp()), 2)],
                "start_end_gap_m": round(float(np.linalg.norm(xyz[-1] - xyz[0])), 3),
            }
            axes[0].plot(t - t[0], yaw, color=colour, label=f"{label} ({len(t)} poses)")

        # every top view is now in the reference method's frame, so the three
        # paths can be read as one picture instead of three rotated copies
        if series and args.align:
            _, _, _, t_ref, _, xy_ref = series[0]
            for s in series[1:]:
                th, rms, rms_m = fit_planar_yaw(t_ref, xy_ref, s[3], s[5])
                s[5] = rotate2d(s[5], th)
                srep[s[0]]["frame_alignment"] = {
                    "reference": series[0][0],
                    "yaw_deg": round(float(np.degrees(th)), 1),
                    "residual_rms_m": None if rms is None else round(rms, 3),
                    "mirrored_residual_rms_m": None if rms_m is None else round(rms_m, 3),
                }
                s[1] = f"{s[1]}  [yaw {np.degrees(th):+.0f}° fitted]"

        for key, label, colour, t, yaw, xy in series:
            axes[1].plot(xy[:, 0], xy[:, 1], color=colour, lw=1.4, label=label)
            axes[1].plot(xy[0, 0], xy[0, 1], "o", color=colour, ms=7)

        if session in imu_map:
            it, iy = imu_yaw(Path(imu_map[session]))
            axes[0].plot(it - it[0], iy, "k--", lw=1.6, label="Xsens integrated (ref)")
            srep["xsens_reference"] = {"net_rotation_deg": round(float(iy[-1]), 1)}
            for key, _, _ in METHODS:
                if "net_rotation_deg" in srep.get(key, {}):
                    srep[key]["rotation_err_vs_xsens_deg"] = round(
                        srep[key]["net_rotation_deg"] - float(iy[-1]), 1)

        axes[0].set_xlabel("t [s]"); axes[0].set_ylabel("rotation about dominant axis [deg]")
        axes[0].set_title(f"{session}: accumulated rotation")
        axes[0].grid(alpha=0.3); axes[0].legend(fontsize=8)
        axes[1].set_aspect("equal"); axes[1].grid(alpha=0.3)
        axes[1].set_xlabel("[m]"); axes[1].set_ylabel("[m]")
        axes[1].set_title(f"{session}: top view " + ("(yaw fitted to "
                          f"{series[0][0] if series else '-'}, origin aligned)"
                          if args.align else "(each in its own frame, origin aligned)"))
        axes[1].legend(fontsize=8)
        fig.tight_layout()
        png = outdir / f"{session}_check.png"
        fig.savefig(png, dpi=130)
        plt.close(fig)
        srep["figure"] = str(png)
        report[session] = srep
        print(f"-> {png}")

    (outdir / "verification.json").write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
