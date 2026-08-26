"""Axis-aligned CAD extents (mm) for projecting 3D object boxes.

SAM-6D CAD meshes are ASCII PLY files in millimetres (matching detection ``t``).
We only need each mesh's axis-aligned bounding box, so we stream the vertex
block in pure Python (no numpy/trimesh) and cache the result to JSON, since the
meshes have ~10^5 vertices and we re-run the visualizer often.

Object name -> CAD folder is resolved leniently: SAM-6D object names such as
``Mugcup_high`` / ``choco_hazelnut_high`` map to ``Mugcup_color_high`` /
``choco_hazelnut_color_high`` folders, and ``milk`` lives under ``fail/milk``.
"""

from __future__ import annotations

import glob
import json
import os
import struct
from typing import Dict, Optional, Tuple

# PLY scalar type -> (struct format char, byte size)
_PLY_TYPES = {
    "char": ("b", 1), "int8": ("b", 1),
    "uchar": ("B", 1), "uint8": ("B", 1),
    "short": ("h", 2), "int16": ("h", 2),
    "ushort": ("H", 2), "uint16": ("H", 2),
    "int": ("i", 4), "int32": ("i", 4),
    "uint": ("I", 4), "uint32": ("I", 4),
    "float": ("f", 4), "float32": ("f", 4),
    "double": ("d", 8), "float64": ("d", 8),
}

Extent = Tuple[Tuple[float, float, float], Tuple[float, float, float]]  # (min, max)


def _base_token(object_name: str) -> str:
    """First meaningful token of an object name, lowercased ('Mugcup_high'->'mugcup')."""
    return object_name.split("_")[0].lower()


def find_cad_ply(cad_root: str, object_name: str) -> Optional[str]:
    """Locate the active (non-.orig_meter) PLY for an object name, or None."""
    cad_root = os.path.expanduser(cad_root)
    token = _base_token(object_name)
    candidates = []
    for path in glob.glob(os.path.join(cad_root, "**", "*.ply"), recursive=True):
        if path.endswith(".orig_meter") or ".orig_meter" in path:
            continue
        # match against the containing folder names and the file name.
        hay = path.lower()
        if token and token in hay:
            candidates.append(path)
    if not candidates:
        return None
    # Prefer the shortest path (closest/most specific folder match).
    return sorted(candidates, key=len)[0]


def _read_ply_header(f):
    """Parse a PLY header (binary file object at start). Returns
    (fmt, n_vertex, vertex_props) where vertex_props is [(name, type), ...] in
    order (only scalar props of the first 'vertex' element)."""
    if f.readline().strip() != b"ply":
        raise ValueError("not a PLY file")
    fmt = f.readline().split()[1].decode()  # ascii | binary_little_endian | ...
    n_vertex = 0
    props = []
    cur = None
    while True:
        line = f.readline()
        if not line:
            raise ValueError("unexpected EOF in PLY header")
        parts = line.split()
        if not parts:
            continue
        kw = parts[0]
        if kw == b"element":
            cur = parts[1].decode()
            if cur == "vertex":
                n_vertex = int(parts[2])
        elif kw == b"property" and cur == "vertex":
            # 'property <type> <name>' (list properties don't occur on vertices)
            props.append((parts[2].decode(), parts[1].decode()))
        elif kw == b"end_header":
            break
    return fmt, n_vertex, props


def parse_ply_extent(ply_path: str) -> Extent:
    """Return ((xmin,ymin,zmin),(xmax,ymax,zmax)) over a PLY's vertices.

    Supports ASCII and binary_little_endian PLY (pure stdlib)."""
    with open(ply_path, "rb") as f:
        fmt, n_vertex, props = _read_ply_header(f)
        names = [p[0] for p in props]
        try:
            ix, iy, iz = names.index("x"), names.index("y"), names.index("z")
        except ValueError:
            raise ValueError(f"PLY vertex has no x/y/z: {ply_path}")

        xmin = ymin = zmin = float("inf")
        xmax = ymax = zmax = float("-inf")

        if fmt == "ascii":
            for _ in range(n_vertex):
                parts = f.readline().split()
                x, y, z = float(parts[ix]), float(parts[iy]), float(parts[iz])
                if x < xmin: xmin = x
                if y < ymin: ymin = y
                if z < zmin: zmin = z
                if x > xmax: xmax = x
                if y > ymax: ymax = y
                if z > zmax: zmax = z
        elif fmt == "binary_little_endian":
            fmt_chars = ""
            for _, t in props:
                if t not in _PLY_TYPES:
                    raise ValueError(f"unsupported PLY property type: {t}")
                fmt_chars += _PLY_TYPES[t][0]
            rec = struct.Struct("<" + fmt_chars)
            buf = f.read(rec.size * n_vertex)
            for v in rec.iter_unpack(buf[: rec.size * n_vertex]):
                x, y, z = v[ix], v[iy], v[iz]
                if x < xmin: xmin = x
                if y < ymin: ymin = y
                if z < zmin: zmin = z
                if x > xmax: xmax = x
                if y > ymax: ymax = y
                if z > zmax: zmax = z
        else:
            raise ValueError(f"unsupported PLY format: {fmt}")

    return ((xmin, ymin, zmin), (xmax, ymax, zmax))


def load_extents(
    cad_root: str,
    object_names,
    cache_path: Optional[str] = None,
) -> Dict[str, Extent]:
    """Return {object_name: extent_mm} for the given names, using a JSON cache.

    Names whose CAD cannot be found are omitted (caller falls back to axes-only).
    """
    cache: Dict[str, Extent] = {}
    if cache_path and os.path.isfile(cache_path):
        with open(cache_path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        for k, v in raw.items():
            cache[k] = (tuple(v[0]), tuple(v[1]))

    out: Dict[str, Extent] = {}
    dirty = False
    for name in object_names:
        if name in cache:
            out[name] = cache[name]
            continue
        ply = find_cad_ply(cad_root, name)
        if ply is None:
            continue
        ext = parse_ply_extent(ply)
        out[name] = ext
        cache[name] = ext
        dirty = True

    if cache_path and dirty:
        os.makedirs(os.path.dirname(os.path.abspath(cache_path)), exist_ok=True)
        with open(cache_path, "w", encoding="utf-8") as f:
            json.dump({k: [list(v[0]), list(v[1])] for k, v in cache.items()}, f)
    return out
