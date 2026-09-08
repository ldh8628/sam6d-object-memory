#!/usr/bin/env python3
"""Collect the three SLAM results for one 260804 session into a share-ready folder.

Each method leaves its map and its trajectory in a different place, a different
file format and a different coordinate convention. This gathers them into

    output_slam/260804_office/unity_export/<session>/
        <method>/map_ros.ply       binary little-endian PLY, x y z + rgb
        <method>/map_unity.ply     same points, Unity's left-handed Y-up frame
        <method>/poses_ros.tum     t tx ty tz qx qy qz qw  (source convention)
        <method>/poses_unity.csv   the same poses in Unity's frame
        <method>/info.json         counts, extent, path length, provenance
        preview_<method>.png       top-down map + trajectory, for eyeballing
        README.md, manifest.json

Binary little-endian is deliberate: it is the only PLY flavour Unity's usual
point-cloud importer (Pcx) reads, and it is a third the size of the ascii the
SLAM tools emit.

Frame conversion. Every source frame here is right-handed, Unity is left-handed,
so the change of basis matrix S has det = -1 and a rotation converts as
R_unity = S R_src S^T (the quaternion is then read off R_unity with the ordinary
formula — Unity's quaternion-to-matrix algebra is the same as ROS's, the
handedness lives in the axes, not in the extraction).

  * ORB-SLAM3 and RTAB-Map work in the colour camera's optical frame
    (x right, y down, z forward), so S = diag(1, -1, 1): flip Y and the scene is
    upright in Unity.
  * hdl_graph_slam works in the Velodyne frame (x forward, y left, z up), so
    S maps (x, y, z) -> (-y, z, x).

    conda activate hdl_graph_slam_humble
    python3 make_unity_export.py longcircle2
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트
OUT_ROOT = ROOT / "output_slam" / "260804_office"
EXPORT_ROOT = OUT_ROOT / "unity_export"

# S per source frame; det(S) = -1 for a faithful right-handed source (the
# handedness flip Unity needs).
S_OPTICAL = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, 1.0]])
S_FLU = np.array([[0, -1.0, 0], [0, 0, 1.0], [1.0, 0, 0]])

# The /velodyne_points in this converted bag are mirrored about a vertical plane
# with respect to the real room: over longcircle2 the platform turns +1066.8°
# (counter-clockwise seen from above) according to the Xsens gravity vector and
# gyro, and +1070°/+1069° according to ORB-SLAM3 and RTAB-Map, but −1069.7°
# according to hdl_graph_slam — and the raw sweeps themselves, matched azimuth to
# azimuth without hdl in the loop, also say negative. Same magnitude to 0.3 %,
# opposite sign: a reflection, not an error. hdl reconstructed its input
# faithfully; the input is what is mirrored, so the fix belongs here.
# Vertical is unaffected (its floor detection is fine), so undoing it is a sign
# flip on one horizontal axis; which one is free, since hdl's absolute yaw is
# arbitrary anyway and the alignment fit absorbs the difference.
MIRROR_Y = np.diag([1.0, -1.0, 1.0])
S_FLU_UNMIRRORED = S_FLU @ MIRROR_Y            # det = +1: two flips cancel


# --------------------------------------------------------------------------
# readers
# --------------------------------------------------------------------------
def read_pcd_ascii(path: Path) -> dict[str, np.ndarray]:
    fields: list[str] = []
    with path.open("r", errors="replace") as fh:
        for line in fh:
            tok = line.split()
            if not tok:
                continue
            if tok[0].upper() == "FIELDS":
                fields = tok[1:]
            elif tok[0].upper() == "DATA":
                if tok[1].lower() != "ascii":
                    raise ValueError(f"{path}: only ascii PCD is handled here")
                break
        data = np.loadtxt(fh, dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return {name: data[:, i] for i, name in enumerate(fields)}


def read_ply_ascii(path: Path) -> dict[str, np.ndarray]:
    props: list[str] = []
    n_vertex = 0
    in_vertex = False
    with path.open("r", errors="replace") as fh:
        for line in fh:
            tok = line.split()
            if not tok:
                continue
            if tok[0] == "format" and tok[1] != "ascii":
                raise ValueError(f"{path}: only ascii PLY is handled here")
            if tok[0] == "element":
                in_vertex = tok[1] == "vertex"
                if in_vertex:
                    n_vertex = int(tok[2])
            elif tok[0] == "property" and in_vertex:
                props.append(tok[-1])
            elif tok[0] == "end_header":
                break
        data = np.loadtxt(fh, dtype=np.float64, max_rows=n_vertex)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return {name: data[:, i] for i, name in enumerate(props)}


def cloud_xyz_rgb(fields: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """(N,3) float32 xyz and (N,3) uint8 rgb from a parsed PCD/PLY."""
    xyz = np.stack([fields["x"], fields["y"], fields["z"]], axis=1)
    if {"red", "green", "blue"} <= fields.keys():
        rgb = np.stack([fields["red"], fields["green"], fields["blue"]], axis=1)
    elif "rgb" in fields:
        # PCL packs r,g,b into the bit pattern of a float32
        packed = fields["rgb"].astype(np.float32).view(np.uint32)
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255],
                       axis=1).astype(np.float64)
    elif "intensity" in fields:
        rgb = intensity_to_rgb(fields["intensity"])
    else:
        rgb = np.full((xyz.shape[0], 3), 200.0)
    keep = np.isfinite(xyz).all(axis=1)
    return (xyz[keep].astype(np.float32),
            np.clip(rgb[keep], 0, 255).astype(np.uint8))


def intensity_to_rgb(inten: np.ndarray) -> np.ndarray:
    """LiDAR intensity -> a readable colour ramp (dark blue -> yellow)."""
    lo, hi = np.percentile(inten, [2, 98])
    t = np.clip((inten - lo) / max(hi - lo, 1e-9), 0, 1)
    r = np.clip(1.6 * t - 0.2, 0, 1)
    g = np.clip(1.3 * t, 0, 1)
    b = np.clip(0.9 - 1.1 * t, 0, 1) + 0.15
    return np.stack([r, g, b], axis=1) * 255.0


def read_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = [l.split() for l in path.read_text().splitlines()
            if l.strip() and not l.startswith("#")]
    a = np.array([[float(v) for v in r[:8]] for r in rows], dtype=np.float64)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


# --------------------------------------------------------------------------
# writers / maths
# --------------------------------------------------------------------------
def write_ply_binary(path: Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    n = xyz.shape[0]
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {n}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n")
    rec = np.empty(n, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                      ("red", "u1"), ("green", "u1"),
                                      ("blue", "u1")]))
    rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rec["red"], rec["green"], rec["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with path.open("wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(rec.tobytes())


def quat_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R: np.ndarray) -> np.ndarray:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def to_unity(xyz: np.ndarray, quats: np.ndarray | None, S: np.ndarray):
    pos = xyz @ S.T
    if quats is None:
        return pos, None
    out = np.empty_like(quats)
    for i, q in enumerate(quats):
        out[i] = R_to_quat(S @ quat_to_R(q) @ S.T)
    return pos, out


def path_length(xyz: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))


# --------------------------------------------------------------------------
def preview(png: Path, xyz: np.ndarray, rgb: np.ndarray, traj: np.ndarray,
            title: str, up_axis: int) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plane = [a for a in (0, 1, 2) if a != up_axis]
    step = max(1, xyz.shape[0] // 400_000)
    fig, ax = plt.subplots(figsize=(7.5, 7.0), dpi=130)
    ax.scatter(xyz[::step, plane[0]], xyz[::step, plane[1]], s=0.4,
               c=rgb[::step] / 255.0, linewidths=0)
    ax.plot(traj[:, plane[0]], traj[:, plane[1]], "-", color="#ff2d55", lw=1.6,
            label=f"trajectory ({len(traj)} poses)")
    ax.plot(traj[0, plane[0]], traj[0, plane[1]], "o", color="#00e0ff", ms=7,
            label="start")
    ax.plot(traj[-1, plane[0]], traj[-1, plane[1]], "s", color="#ffd500", ms=7,
            label="end")
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xlabel("xyz"[plane[0]] + " [m]")
    ax.set_ylabel("xyz"[plane[1]] + " [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(png)
    plt.close(fig)


def overlay_preview(png: Path, tracks: dict[str, np.ndarray], ses: str) -> None:
    """The three trajectories in the camera frame, hdl brought over by the fit."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"orb3_slam": "#ff2d55", "rtabmap": "#0aa6ff",
              "hdl_graph_slam (fitted)": "#22c55e"}
    fig, ax = plt.subplots(figsize=(7.5, 7.0), dpi=130)
    for label, xyz in tracks.items():
        ax.plot(xyz[:, 0], xyz[:, 2], "-", lw=1.5,
                color=colors.get(label, "#888"), label=f"{label} ({len(xyz)})")
    ax.plot(0, 0, "o", color="k", ms=6, label="origin (first camera frame)")
    ax.set_aspect("equal")
    ax.set_title(f"{ses} — trajectories in the camera frame (top view)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(png)
    plt.close(fig)


def export_one(name: str, cloud: Path, poses: Path, S: np.ndarray,
               up_axis: int, dest: Path, note: str) -> dict:
    dest.mkdir(parents=True, exist_ok=True)
    fields = read_ply_ascii(cloud) if cloud.suffix == ".ply" else read_pcd_ascii(cloud)
    xyz, rgb = cloud_xyz_rgb(fields)
    write_ply_binary(dest / "map_ros.ply", xyz, rgb)
    uxyz, _ = to_unity(xyz, None, S)
    write_ply_binary(dest / "map_unity.ply", uxyz.astype(np.float32), rgb)

    ts, txyz, tq = read_tum(poses)
    (dest / "poses_ros.tum").write_text(
        "# timestamp tx ty tz qx qy qz qw   (source frame, right-handed)\n" +
        "".join(f"{t:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n"
                for t, p, q in zip(ts, txyz, tq)))
    upos, uq = to_unity(txyz, tq, S)
    (dest / "poses_unity.csv").write_text(
        "timestamp,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w\n" +
        "".join(f"{t:.9f},{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},"
                f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}\n"
                for t, p, q in zip(ts, upos, uq)))

    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    inside = ((txyz >= lo - 1.0) & (txyz <= hi + 1.0)).all(axis=1).mean()
    info = {
        "method": name, "points": int(xyz.shape[0]), "poses": int(len(ts)),
        "duration_sec": round(float(ts[-1] - ts[0]), 2),
        "pose_rate_hz": round(len(ts) / max(float(ts[-1] - ts[0]), 1e-9), 2),
        "path_length_m": round(path_length(txyz), 2),
        "map_extent_m": [round(float(v), 2) for v in (hi - lo)],
        "traj_inside_map_frac": round(float(inside), 4),
        "source_cloud": str(cloud.relative_to(ROOT)),
        "source_poses": str(poses.relative_to(ROOT)),
        "note": note,
    }
    (dest / "info.json").write_text(json.dumps(info, indent=1))
    preview(dest.parent / f"preview_{name}.png", xyz, rgb, txyz,
            f"{name} — {info['points']:,} pts, {info['poses']} poses, "
            f"{info['path_length_m']} m", up_axis)
    print(f"{name:<16} {info['points']:>9,} pts  {info['poses']:>5} poses "
          f"@{info['pose_rate_hz']:>5.1f} Hz  path {info['path_length_m']:>6.2f} m "
          f"extent {info['map_extent_m']}")
    return info


def interp_positions(ts: np.ndarray, xyz: np.ndarray,
                     query: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Linear position interpolation, plus the mask of queries inside the span."""
    inside = (query >= ts[0]) & (query <= ts[-1])
    q = query[inside]
    out = np.stack([np.interp(q, ts, xyz[:, k]) for k in range(3)], axis=1)
    return out, inside


def plane_normal(xyz: np.ndarray) -> tuple[np.ndarray, float]:
    """Normal of the best-fit plane through a point set, and its flatness."""
    c = xyz - xyz.mean(axis=0)
    _, s, Vt = np.linalg.svd(c, full_matrices=False)
    return Vt[2], float(s[2] / s[0])


def fit_planar_alignment(src: np.ndarray, dst: np.ndarray, up_src: np.ndarray,
                         up_dst: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Rigid fit of dst ~ R @ src + t with the vertical correspondence pinned.

    A plain Umeyama fit is wrong here. The platform drove on a floor, so both
    trajectories are essentially planar (third singular value ~1 % of the first),
    which leaves the SVD free to pick either sign for the out-of-plane axis: the
    two solutions have the *same* residual and one of them is upside down. It
    picked the upside-down one.

    So the vertical is imposed rather than fitted: rotate up_src onto up_dst,
    then solve the one remaining degree of freedom — the yaw about that shared
    vertical — in closed form, together with the translation.
    """
    a = up_src / np.linalg.norm(up_src)
    b = up_dst / np.linalg.norm(up_dst)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-12:                    # already (anti)parallel
        R0 = np.eye(3) if c > 0 else -np.eye(3) + 2 * np.outer(b, b)
    else:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R0 = np.eye(3) + K + K @ K / (1 + c)
    s = src @ R0.T

    # in-plane basis orthogonal to the shared vertical
    e1 = np.array([1.0, 0, 0]) - b * (b @ np.array([1.0, 0, 0]))
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.array([0, 1.0, 0]) - b * (b @ np.array([0, 1.0, 0]))
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(b, e1)

    sc, dc = s - s.mean(axis=0), dst - dst.mean(axis=0)
    s2 = np.stack([sc @ e1, sc @ e2], axis=1)
    d2 = np.stack([dc @ e1, dc @ e2], axis=1)
    theta = float(np.arctan2((s2[:, 0] * d2[:, 1] - s2[:, 1] * d2[:, 0]).sum(),
                             (s2 * d2).sum()))
    ct, st = np.cos(theta), np.sin(theta)
    K = np.array([[0, -b[2], b[1]], [b[2], 0, -b[0]], [-b[1], b[0], 0]])
    R_yaw = np.eye(3) + st * K + (1 - ct) * (K @ K)
    R = R_yaw @ R0
    t = dst.mean(axis=0) - R @ src.mean(axis=0)
    resid = dst - (src @ R.T + t)
    return R, t, float(np.sqrt((resid ** 2).sum(axis=1).mean()))


def trajectory_alignments(dest_root: Path, ses: str) -> dict:
    """How the three results sit relative to each other, measured on the poses.

    ORB-SLAM3 and RTAB-Map both start from identity at the same first camera
    frame, so they are already in one frame and only need a residual; hdl works
    in the Velodyne frame and needs a rigid transform. That transform is fitted
    to the trajectories, which is NOT the same thing as a calibrated
    camera-LiDAR extrinsic: the two sensors sit at different points on the
    platform, so a lever arm is folded into the fit and it is only as good as
    the trajectories themselves. It is here to overlay the maps, not to
    calibrate anything.
    """
    def load(m):
        return read_tum(dest_root / m / "poses_ros.tum")

    ts_o, xyz_o, _ = load("orb3_slam")
    out: dict = {"reference": "orb3_slam (camera optical frame)"}

    ts_r, xyz_r, _ = load("rtabmap")
    ref, inside = interp_positions(ts_o, xyz_o, ts_r)
    d = np.linalg.norm(xyz_r[inside] - ref, axis=1)
    out["rtabmap_vs_orb3"] = {
        "shared_frame": True, "samples": int(inside.sum()),
        "rms_m": round(float(np.sqrt((d ** 2).mean())), 3),
        "max_m": round(float(d.max()), 3),
        "note": "same origin already — this is the disagreement between the two "
                "camera methods, not a residual after fitting.",
    }

    ts_l, xyz_l, _ = load("hdl_graph_slam")
    ref, inside = interp_positions(ts_o, xyz_o, ts_l)
    # hdl is gravity-aligned by its floor detection, so its up is exactly +z.
    # The camera's up is taken from the plane its own trajectory sweeps out and
    # signed by the optical convention (+y points down, so up is -y) — verified
    # against the map itself: floor -1.15 m, desk tops -0.30 m and ceiling
    # +1.25 m relative to the camera, i.e. a 2.4 m office ceiling the right way up.
    up_cam, flatness = plane_normal(xyz_o)
    if up_cam @ np.array([0, -1.0, 0]) < 0:
        up_cam = -up_cam
    # fit on the un-mirrored LiDAR positions, matching what the *_unity files hold
    R, t, rms = fit_planar_alignment(xyz_l[inside] @ MIRROR_Y.T, ref,
                                     np.array([0, 0, 1.0]), up_cam)
    up_check = float((R @ np.array([0, 0, 1.0])) @ np.array([0, -1.0, 0]))
    R_u = S_OPTICAL @ R @ S_FLU.T
    q_u = R_to_quat(R_u)
    t_u = S_OPTICAL @ t
    out["hdl_to_camera_frame"] = {
        "fitted_on_poses": True, "samples": int(inside.sum()),
        "fit_rms_m": round(rms, 3),
        "vertical_pinned": True,
        "camera_up_from_trajectory_plane": [round(float(v), 6) for v in up_cam],
        "camera_trajectory_flatness": round(flatness, 5),
        "up_maps_to_up": round(up_check, 4),
        "ros": {"R": [[round(float(v), 6) for v in row] for row in R],
                "t": [round(float(v), 6) for v in t]},
        "unity": {"position": [round(float(v), 6) for v in t_u],
                  "rotation_xyzw": [round(float(v), 6) for v in q_u]},
        "applies_to": "the mirror-corrected LiDAR coordinates, i.e. what "
                      "hdl_graph_slam/map_unity.ply and poses_unity.csv hold. "
                      "`ros` applies to MIRROR_Y @ (hdl's own output).",
        "note": "Set the hdl_graph_slam GameObject's local position/rotation to "
                "`unity` to overlay it on the two camera maps. Trajectory fit, "
                "not a calibrated extrinsic — a camera/LiDAR lever arm is "
                "absorbed into it. The vertical is imposed, not fitted, because "
                "the trajectory is planar and a free fit is upside-down "
                "ambiguous; only the yaw and the translation come from the data.",
    }
    print(f"hdl up -> camera up dot = {up_check:+.4f} (must be close to +1)")
    overlay_preview(dest_root / "preview_overlay.png",
                    {"orb3_slam": xyz_o, "rtabmap": xyz_r,
                     "hdl_graph_slam (fitted)":
                         (xyz_l @ MIRROR_Y.T) @ R.T + t}, ses)
    (dest_root / "alignment.json").write_text(json.dumps(out, indent=1))
    print(f"\nrtabmap vs orb3 (shared frame): rms {out['rtabmap_vs_orb3']['rms_m']} m, "
          f"max {out['rtabmap_vs_orb3']['max_m']} m")
    print(f"hdl -> camera frame fit: rms {rms:.3f} m over "
          f"{int(inside.sum())} poses")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    args = ap.parse_args()
    ses = args.session

    rtab_dense = OUT_ROOT / "rtabmap_dense" / ses
    use_dense_rtab = (rtab_dense / "trajectory_dense.txt").exists()
    specs = [
        dict(name="orb3_slam",
             cloud=OUT_ROOT / "orb3_slam" / ses / "orbslam3_dense_map.pcd",
             poses=OUT_ROOT / "orb3_slam" / ses / "trajectory.txt",
             S=S_OPTICAL, up_axis=1,
             note="RGB-D. Map and poses are ORB-SLAM3's own dense map and "
                  "per-frame camera trajectory (30 Hz), colour camera optical "
                  "frame, origin at the first frame."),
        dict(name="rtabmap",
             cloud=(rtab_dense / "rtabmap_dense_cloud.ply") if use_dense_rtab
                   else (OUT_ROOT / "rtabmap" / ses / "rtabmap_3d_map_cloud.ply"),
             poses=(rtab_dense / "trajectory_dense.txt") if use_dense_rtab
                   else (OUT_ROOT / "rtabmap" / ses / "trajectory.txt"),
             S=S_OPTICAL, up_axis=1,
             note=("RGB-D. Poses are rgbd_odometry at frame rate with the pose "
                   "graph correction interpolated onto it; map exported from the "
                   "same run's database." if use_dense_rtab else
                   "RGB-D. Poses are pose-graph nodes only (~1 Hz).")),
        dict(name="hdl_graph_slam",
             cloud=OUT_ROOT / "hdl_graph_slam" / ses / "hdl_lidar_map.pcd",
             poses=OUT_ROOT / "hdl_graph_slam" / ses / "trajectory_dense.txt",
             S=S_FLU_UNMIRRORED, up_axis=2,
             note="Velodyne LiDAR. Poses are scan-matching odometry with the "
                  "graph correction interpolated onto it (10 Hz); map rebuilt "
                  "from the raw sweeps using those poses. Velodyne frame, "
                  "origin at the first sweep — NOT the camera frame. The *_unity "
                  "files also undo the horizontal mirror the raw sweeps carry "
                  "(see MIRROR_Y in make_unity_export.py); the *_ros files are "
                  "hdl's output untouched and are therefore still mirrored."),
    ]

    dest_root = EXPORT_ROOT / ses
    dest_root.mkdir(parents=True, exist_ok=True)
    infos, rc = [], 0
    for sp in specs:
        if not sp["cloud"].exists() or not sp["poses"].exists():
            missing = [str(p) for p in (sp["cloud"], sp["poses"]) if not p.exists()]
            print(f"{sp['name']}: SKIPPED, missing {missing}", file=sys.stderr)
            rc = 1
            continue
        infos.append(export_one(sp["name"], sp["cloud"], sp["poses"], sp["S"],
                                sp["up_axis"], dest_root / sp["name"], sp["note"]))

    align = trajectory_alignments(dest_root, ses) if len(infos) == 3 else {}
    (dest_root / "manifest.json").write_text(json.dumps(
        {"session": ses, "dataset": "data_slam/260804_office", "methods": infos,
         "alignment": align}, indent=1))
    print(f"\n-> {dest_root}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
