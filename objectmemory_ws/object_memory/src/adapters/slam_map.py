"""Load a SLAM point-cloud map (ASCII PCD) as a list of (x, y, z) points.

Used as the background of the 2D BEV panel: the ORB-SLAM3 dense map projected
onto the map plane gives an actual top-down view of the environment. Pure
stdlib (no numpy/open3d); large clouds are strided down to a point cap.
"""

from __future__ import annotations

import os
import struct
from typing import List, Tuple

from adapters.cad_extents import _PLY_TYPES, _read_ply_header

Point = Tuple[float, float, float]


def load_pcd_xyz(path: str, max_points: int = 200000) -> List[Point]:
    """Return up to ``max_points`` (x, y, z) points from an ASCII PCD file."""
    path = os.path.expanduser(path)
    fields: List[str] = []
    n_points = 0
    with open(path, "r", encoding="latin-1") as f:
        # header
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"unexpected EOF in PCD header: {path}")
            parts = line.split()
            if not parts:
                continue
            key = parts[0].upper()
            if key == "FIELDS":
                fields = parts[1:]
            elif key == "POINTS":
                n_points = int(parts[1])
            elif key == "DATA":
                if parts[1].lower() != "ascii":
                    raise ValueError(f"only ASCII PCD supported: {parts[1]}")
                break
        try:
            ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
        except ValueError:
            raise ValueError(f"PCD has no x/y/z fields: {fields}")

        stride = max(1, n_points // max_points) if max_points else 1
        pts: List[Point] = []
        for i, line in enumerate(f):
            if stride > 1 and (i % stride):
                continue
            p = line.split()
            try:
                x, y, z = float(p[ix]), float(p[iy]), float(p[iz])
            except (IndexError, ValueError):
                continue
            # drop NaN/inf
            if x == x and y == y and z == z:
                pts.append((x, y, z))
    return pts


def load_ply_xyz(path: str, max_points: int = 200000) -> List[Point]:
    """Return up to ``max_points`` (x, y, z) from an ASCII or binary PLY."""
    path = os.path.expanduser(path)
    with open(path, "rb") as f:
        fmt, n, props = _read_ply_header(f)
        names = [p[0] for p in props]
        try:
            ix, iy, iz = names.index("x"), names.index("y"), names.index("z")
        except ValueError:
            raise ValueError(f"PLY has no x/y/z: {names}")
        stride = max(1, n // max_points) if max_points else 1
        pts: List[Point] = []
        if fmt == "ascii":
            for i in range(n):
                line = f.readline()
                if stride > 1 and (i % stride):
                    continue
                p = line.split()
                try:
                    pts.append((float(p[ix]), float(p[iy]), float(p[iz])))
                except (IndexError, ValueError):
                    continue
        elif fmt == "binary_little_endian":
            fmt_chars = "".join(_PLY_TYPES[t][0] for _, t in props)
            rec = struct.Struct("<" + fmt_chars)
            buf = f.read(rec.size * n)
            for i, v in enumerate(rec.iter_unpack(buf[: rec.size * n])):
                if stride > 1 and (i % stride):
                    continue
                pts.append((v[ix], v[iy], v[iz]))
        else:
            raise ValueError(f"unsupported PLY format: {fmt}")
    return pts


def load_map_points(path: str, max_points: int = 200000) -> List[Point]:
    """Load a SLAM map cloud from a .pcd or .ply file."""
    if path.lower().endswith(".ply"):
        return load_ply_xyz(path, max_points)
    return load_pcd_xyz(path, max_points)


def default_map_path(slam_traj_path: str) -> str:
    """Given a CameraTrajectory.txt path, guess the dense map pcd next to it."""
    d = os.path.dirname(os.path.abspath(slam_traj_path))
    for name in ("orbslam3_dense_map.pcd", "orbslam3_map_points.pcd",
                 "map_points.pcd"):
        cand = os.path.join(d, name)
        if os.path.isfile(cand):
            return cand
    return ""
