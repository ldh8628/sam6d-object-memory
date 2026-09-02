#!/usr/bin/env python3
"""객체별 CAD 바운딩박스(mm)를 한 번 계산해 캐시한다.

review 영상에 3D BBox 를 그리려면 객체 좌표계에서의 크기가 필요하다. SAM-6D 가 쓰는
CAD 경로는 sam6d_ws/configs/yolo_ism_objects.yaml 의 `cad_ply` 에 적혀 있다.
CAD 는 mm 단위다(BOP/SAM-6D 규약).

큰 PLY(최대 386 MB)를 매번 읽지 않도록 결과를 integration/cad_extents.json 에 캐시한다.

    python3 integration/cad_extents.py [--force]
"""
from __future__ import annotations

import argparse
import json
import re
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "sam6d_ws" / "configs" / "yolo_ism_objects.yaml"
SAM6D = ROOT / "sam6d_ws"
CACHE = Path(__file__).resolve().parent / "cad_extents.json"


def parse_header(f):
    """-> (fmt, n_vertex, props, header_len). props = [(type, name), ...]"""
    props, n, fmt = [], 0, None
    in_vertex = False
    while True:
        line = f.readline().decode("ascii", "replace").strip()
        if line.startswith("format"):
            fmt = line.split()[1]
        elif line.startswith("element vertex"):
            n = int(line.split()[2]); in_vertex = True
        elif line.startswith("element"):
            in_vertex = False
        elif line.startswith("property") and in_vertex:
            p = line.split()
            if p[1] != "list":
                props.append((p[1], p[2]))
        elif line == "end_header":
            return fmt, n, props, f.tell()


SZ = {"float": 4, "float32": 4, "double": 8, "float64": 8, "uchar": 1, "uint8": 1,
      "char": 1, "int8": 1, "short": 2, "ushort": 2, "int": 4, "uint": 4}
FM = {"float": "f", "float32": "f", "double": "d", "float64": "d", "uchar": "B",
      "uint8": "B", "char": "b", "int8": "b", "short": "h", "ushort": "H",
      "int": "i", "uint": "I"}


def extents(path: Path, max_pts: int = 400000):
    with path.open("rb") as f:
        fmt, n, props, off = parse_header(f)
        names = [p[1] for p in props]
        ix, iy, iz = names.index("x"), names.index("y"), names.index("z")
        if fmt == "ascii":
            stride = max(1, n // max_pts)
            lo = np.full(3, np.inf); hi = np.full(3, -np.inf)
            for i, line in enumerate(f):
                if i >= n:
                    break
                if i % stride:
                    continue
                v = line.split()
                try:
                    p = np.array([float(v[ix]), float(v[iy]), float(v[iz])])
                except (ValueError, IndexError):
                    continue
                lo = np.minimum(lo, p); hi = np.maximum(hi, p)
            return lo, hi, n
        # binary
        rec = "".join(FM[t] for t, _ in props)
        rsz = sum(SZ[t] for t, _ in props)
        endian = "<" if "little" in fmt else ">"
        st = struct.Struct(endian + rec)
        buf = f.read(rsz * n)
        lo = np.full(3, np.inf); hi = np.full(3, -np.inf)
        stride = max(1, n // max_pts)
        for i in range(0, n, stride):
            v = st.unpack_from(buf, i * rsz)
            p = np.array([v[ix], v[iy], v[iz]])
            lo = np.minimum(lo, p); hi = np.maximum(hi, p)
        return lo, hi, n


def cad_paths() -> dict[str, Path]:
    txt = CFG.read_text(encoding="utf-8")
    out, name = {}, None
    for line in txt.splitlines():
        m = re.match(r"\s*-?\s*name:\s*(\S+)", line)
        if m:
            name = m.group(1).strip('"\'')
        m = re.match(r"\s*cad_ply:\s*(\S+)", line)
        if m and name:
            out[name] = SAM6D / m.group(1)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    cache = json.loads(CACHE.read_text()) if CACHE.exists() and not a.force else {}
    for name, p in sorted(cad_paths().items()):
        if name in cache or not p.is_file():
            if not p.is_file():
                print(f"  {name}: CAD 없음 {p}")
            continue
        lo, hi, n = extents(p)
        cache[name] = {"min_mm": [round(float(v), 2) for v in lo],
                       "max_mm": [round(float(v), 2) for v in hi],
                       "size_mm": [round(float(v), 2) for v in (hi - lo)],
                       "vertices": n, "cad": str(p)}
        print(f"  {name:<24} size {np.round(hi - lo, 1)} mm  ({n} verts)")
    CACHE.write_text(json.dumps(cache, indent=1))
    print(f"-> {CACHE}  ({len(cache)} objects)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
