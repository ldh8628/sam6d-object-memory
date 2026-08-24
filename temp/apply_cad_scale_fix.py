#!/usr/bin/env python3
"""apply_cad_scale_fix.py — 인형 CAD 가 실물보다 작은 것을 바로잡는다.

배율 근거(세 가지가 서로 맞는다):
  · 마스크∧depth 로 잰 실제 높이 / CAD 높이   Bear 1.16 · Dinosaur 1.19 · Rabbit 1.08 (가림 때문에 하한)
  · CAD 투영 넓이 / 마스크 넓이의 역수(선형)   Bear 1.16~1.19 · Dinosaur 1.27 · Rabbit 1.16~1.30
  · 배율 스윕에서 pose_score·뒤집힘 최적점     Bear 1.15 · Dinosaur 1.15~1.45 · Rabbit 1.30
→ 채택: Bear 1.16 · Dinosaur 1.25 · Rabbit 1.25   (자로 잰 값이 있으면 그 값으로 바꿀 것)

바꾸는 것:
  1) 원본 CAD  sam6d_ws/data/cad/<obj>/*.ply           (원본은 .orig 로 백업)
  2) 템플릿 xyz  <repo>/template/<obj>/templates/xyz_*.npy   (렌더된 표면점 = 물체 좌표계)
  3) PEM 자산  assets/model_points/<obj>.npy, assets/pem_templates/<obj>.pt 의 tp
외형(RGB·마스크·ISM 특징·HSV)은 건드리지 않는다 — 크기만 바뀐다.
"""
import argparse, shutil, sys
from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[1]
WS = REPO.parent / "sam6d_ws"
SCALE = {"Bear": 1.16, "Dinosaur": 1.25, "Rabbit": 1.25, "choco_hazelnut_high": 0.81,
         # 우유팩만 **축별로 다르다** — 순수한 크기 오류가 아니라 모양이 다르다.
         #   밑면: 사용자 실측 70 mm 정사각 (CAD 73.8) -> ×0.949
         #        측정도 일치: 마스크 가로 p10 75 mm (정면 70 ~ 45° 99 사이여야 함)
         #   높이: 실측 중앙 233 mm (0807) · 232 mm (260804), CAD 195 -> ×1.18
         #   두 값이 다르므로 균일 배율로는 못 맞춘다(게이블탑 윗부분이 CAD 에 없는 듯).
         #   근본 해결은 CAD 교체다. 여기서는 측정에 맞춰 축별로 늘린다.
         "milk": (0.949, 0.949, 1.18)}
# 초코하임은 반대로 CAD 가 크다. 마스크로 잰 실물 앞면이
#   0807  207 × 151 mm  (CAD 대비 0.83 × 0.80)
#   260804 216 × 152 mm (        0.86 × 0.80)   ← 두 데이터셋·두 변이 모두 같은 배율
# CAD 는 250 × 189 mm. 가로세로 비(1.34 vs 1.32)가 같으므로 순수한 크기 오류다.
CAD_DIR = {"Bear": "Bear", "Dinosaur": "Dinosaur", "Rabbit": "Rabbit",
           "choco_hazelnut_high": "choco_hazelnut_color_high", "milk": "milk"}
TEM_DIR = {"choco_hazelnut_high": "choco_hazelnut_color_high"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="실제로 쓴다 (없으면 미리보기만)")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()

    for nm, s in SCALE.items():
        s = np.array(s, dtype=np.float64) if isinstance(s, tuple) else np.array([s, s, s])
        print(f"\n=== {nm}  ×{np.round(s,3).tolist()} ===")
        # 1) 원본 CAD
        for ply in sorted((WS / "data" / "cad" / CAD_DIR.get(nm, nm)).glob("*.ply")):
            bak = ply.with_suffix(".ply.orig")
            if a.revert:
                if bak.is_file():
                    shutil.copy2(bak, ply); print(f"  [revert] {ply.name}")
                continue
            if not bak.is_file() and a.apply:
                shutil.copy2(ply, bak)
            # PLY 의 좌표 세 개만 곱한다. 법선·uv·정점색·면은 그대로 둔다.
            # (mesh 라이브러리로 다시 내보내면 정점이 병합되고 텍스처가 사라진다 — 실제로 그랬다.)
            # ascii / binary_little_endian 둘 다 처리한다.
            src = bak if bak.is_file() else ply
            with open(src, "rb") as fh:
                raw = fh.read()
            hend = raw.index(b"end_header") + len(b"end_header")
            while raw[hend:hend + 1] in (b"\r", b"\n"):
                hend += 1
            head = raw[:hend].decode("ascii", "replace")
            fmt = "ascii" if "format ascii" in head else "binary"
            nvert = int([l for l in head.splitlines()
                         if l.startswith("element vertex")][0].split()[2])
            props = []
            seen_vertex = False
            for l in head.splitlines():
                if l.startswith("element vertex"):
                    seen_vertex = True; continue
                if l.startswith("element") and seen_vertex:
                    break
                if seen_vertex and l.startswith("property"):
                    props.append(l.split()[1:])          # [type, name]
            if fmt == "ascii":
                lines = raw[hend:].decode("ascii", "replace").split("\n")
                out, lo, hi = [], np.full(3, np.inf), np.full(3, -np.inf)
                for i in range(nvert):
                    tok = lines[i].split()
                    v = np.array(tok[:3], dtype=np.float64)
                    lo = np.minimum(lo, v); hi = np.maximum(hi, v)
                    tok[:3] = [f"{c:.6f}" for c in v * s]
                    out.append(" ".join(tok))
                body = ("\n".join(out) + "\n" + "\n".join(lines[nvert:])).encode("ascii")
            else:
                TY = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
                      "uchar": "u1", "uint8": "u1", "char": "i1", "int8": "i1",
                      "ushort": "<u2", "uint16": "<u2", "short": "<i2", "int16": "<i2",
                      "uint": "<u4", "uint32": "<u4", "int": "<i4", "int32": "<i4"}
                dt = np.dtype([(pn, TY[pt]) for pt, pn in props])
                arr = np.frombuffer(raw[hend:hend + nvert * dt.itemsize], dtype=dt).copy()
                lo = np.array([arr["x"].min(), arr["y"].min(), arr["z"].min()], float)
                hi = np.array([arr["x"].max(), arr["y"].max(), arr["z"].max()], float)
                for k, ax in enumerate("xyz"):
                    arr[ax] = (arr[ax].astype(np.float64) * s[k]).astype(arr.dtype[ax])
                body = arr.tobytes() + raw[hend + nvert * dt.itemsize:]
            print(f"  ply {ply.name} [{fmt}]: {(hi-lo).round(1)} -> {((hi-lo)*s).round(1)} mm "
                  f"(정점 {nvert}, 속성 보존)")
            if a.apply:
                with open(ply, "wb") as fh:
                    fh.write(raw[:hend]); fh.write(body)
        # 2) 템플릿 xyz
        xs = sorted((REPO / "template" / TEM_DIR.get(nm, nm) / "templates").glob("xyz_*.npy"))
        if xs:
            print(f"  템플릿 xyz {len(xs)} 개")
            for x in xs:
                bak = x.with_suffix(".npy.orig")
                if a.revert:
                    if bak.is_file():
                        shutil.copy2(bak, x)
                    continue
                if not bak.is_file() and a.apply:
                    shutil.copy2(x, bak)
                if a.apply:
                    np.save(x, np.load(bak if bak.is_file() else x) * s.astype(np.float32))
        # 3) PEM 자산
        mp = REPO / "assets" / "model_points" / f"{nm}.npy"
        tpf = REPO / "assets" / "pem_templates" / f"{nm}.pt"
        for f in (mp, tpf):
            bak = f.with_suffix(f.suffix + ".orig")
            if a.revert:
                if bak.is_file():
                    shutil.copy2(bak, f); print(f"  [revert] {f.name}")
                continue
            if not bak.is_file() and a.apply:
                shutil.copy2(f, bak)
        if a.revert:
            continue
        P = np.load(mp.with_suffix(".npy.orig") if mp.with_suffix(".npy.orig").is_file() else mp)
        print(f"  model_points: {(P.max(0)-P.min(0)).round(1)} -> {((P.max(0)-P.min(0))*s).round(1)} mm")
        if a.apply:
            np.save(mp, (P * s).astype(np.float32))
            src = tpf.with_suffix(".pt.orig") if tpf.with_suffix(".pt.orig").is_file() else tpf
            b = torch.load(src, map_location="cpu")
            b["tp"] = b["tp"] * torch.tensor(s, dtype=b["tp"].dtype)
            torch.save(b, tpf)
            print(f"  pem_templates tp 적용")
    if not a.apply and not a.revert:
        print("\n(미리보기였다. 실제로 쓰려면 --apply)")


if __name__ == "__main__":
    main()
