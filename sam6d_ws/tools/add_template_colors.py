#!/usr/bin/env python3
"""템플릿 캐시(assets/pem_templates/<obj>.pt)에 점별 색 `tc` 를 추가한다.

Stage V 의 원색 채널은 "관측 화소색 ↔ 그 후보 포즈가 주장하는 모델점의 색"을 맞댄다.
관측 쪽 색은 이미 들어와 있는 crop 에서 그 자리에서 뽑지만(자산 불필요), 모델 쪽 색은
템플릿 렌더에서 미리 뽑아 두어야 한다.

★ tp / tf 는 절대 건드리지 않는다.
  템플릿 점 표본추출(`_get_template` 의 np.random.choice)은 씨앗이 없어 **재현되지 않는다**.
  그래서 tp 를 다시 만들지 않고, **이미 저장된 tp 를 질의점으로 삼아** 42 뷰의 원해상도
  마스크 화소(=색이 있는 조밀 점군)에서 색을 찾아온다. 결과적으로 tf 와의 점 대응이
  정의상 그대로 유지된다.

  한 점이 여러 뷰에서 보이면 그 뷰들의 색을 평균한다(뷰별 음영·정반사가 상쇄된다.
  choco 정면처럼 광원이 카메라에 있어 포화되는 뷰가 있으므로 평균이 안전하다).

채널 순서: PEM 은 관측·템플릿 모두 imread 결과를 [:, :, ::-1] 로 뒤집어 쓴다.
  관측(get_instance_data)은 RGB→BGR, 템플릿(_get_template)은 imageio(RGB)→BGR.
  즉 **양쪽 다 BGR** 이다. 여기서도 BGR 로 저장해야 화이트닝 후 채널이 맞는다.

사용: python tools/add_template_colors.py [--objects milk,choco_hazelnut_high] [--eps-mm 2] [--force]
"""
import argparse
import glob
import os
import shutil
import sys

import imageio.v2 as imageio
import numpy as np
import torch

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEM_DIR = os.path.join(REPO, "assets", "pem_templates")
N_VIEW = 42


def _resolve(tdir):
    """blob 에 박힌 절대경로가 다른 기계 것이면 이 저장소 기준으로 되돌린다."""
    if os.path.isdir(tdir):
        return tdir
    i = tdir.find("/template/")
    if i >= 0:
        alt = os.path.join(REPO, tdir[i + 1:])
        if os.path.isdir(alt):
            return alt
    return tdir


def _view_cloud(tdir, v):
    """한 뷰의 원해상도 색점군 (xyz[m], bgr[0..1])."""
    rgb = imageio.imread(os.path.join(tdir, f"rgb_{v}.png"))[:, :, :3]
    mask = imageio.imread(os.path.join(tdir, f"mask_{v}.png")) == 255
    xyz = np.load(os.path.join(tdir, f"xyz_{v}.npy")).astype(np.float32) / 1000.0
    if mask.ndim == 3:
        mask = mask[:, :, 0]
    sel = mask.reshape(-1)
    pts = xyz.reshape(-1, 3)[sel]
    col = rgb.reshape(-1, 3)[sel][:, ::-1].astype(np.float32) / 255.0   # RGB -> BGR
    keep = np.isfinite(pts).all(1) & (np.abs(pts).sum(1) > 0)
    return pts[keep], np.ascontiguousarray(col[keep])


def build(name, eps_m, device, force):
    path = os.path.join(TEM_DIR, f"{name}.pt")
    blob = torch.load(path, map_location="cpu")
    if "tc" in blob and not force:
        print(f"  {name}: 이미 tc 있음 — 건너뜀 (--force 로 재생성)")
        return None
    tdir = _resolve(str(blob["tdir"]))
    if not os.path.isdir(tdir):
        print(f"  {name}: 템플릿 디렉터리 없음 — {tdir}")
        return None

    tp = blob["tp"].to(device).float()[0]                       # [2048,3] (m)
    best_d = torch.full((tp.shape[0],), float("inf"), device=device)
    acc = torch.zeros_like(tp)                                   # 가시 뷰 색의 합
    cnt = torch.zeros(tp.shape[0], device=device)
    near = torch.zeros_like(tp)                                  # 최근접 뷰의 색(대비책)

    n_view = 0
    for v in range(N_VIEW):
        if not os.path.isfile(os.path.join(tdir, f"xyz_{v}.npy")):
            continue
        n_view += 1
        pts, col = _view_cloud(tdir, v)
        pts = torch.from_numpy(pts).to(device)
        col = torch.from_numpy(col).to(device)
        d, j = torch.cdist(tp.unsqueeze(0), pts.unsqueeze(0))[0].min(1)
        c = col[j]
        vis = d < eps_m
        acc[vis] += c[vis]
        cnt += vis.float()
        upd = d < best_d
        best_d = torch.where(upd, d, best_d)
        near[upd] = c[upd]

    seen = cnt > 0
    tc = torch.where(seen.unsqueeze(1), acc / cnt.clamp(min=1).unsqueeze(1), near)
    stat = dict(views=n_view,
                seen_frac=float(seen.float().mean()),
                med_min_mm=float(best_d.median()) * 1000.0,
                p95_min_mm=float(best_d.quantile(0.95)) * 1000.0,
                mean_views=float(cnt[seen].mean()) if bool(seen.any()) else 0.0)

    if not os.path.isfile(path + ".pre_tc"):
        shutil.copy2(path, path + ".pre_tc")
    out = dict(blob)                       # tp / tf 는 같은 텐서 객체를 그대로 넘긴다
    out["tc"] = tc.unsqueeze(0).cpu()
    tmp = path + ".tmp"
    torch.save(out, tmp)
    os.replace(tmp, path)

    chk = torch.load(path, map_location="cpu")
    assert torch.equal(chk["tp"], blob["tp"]) and torch.equal(chk["tf"], blob["tf"]), \
        f"{name}: tp/tf 가 바뀌었다 — 중단"
    return stat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objects", default="", help="쉼표로 구분. 비우면 전부")
    ap.add_argument("--eps-mm", type=float, default=2.0,
                    help="이 거리 안에 있는 뷰를 '그 점이 보이는 뷰'로 본다")
    ap.add_argument("--device", default="cuda:0" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()

    names = ([s for s in a.objects.split(",") if s] or
             sorted(os.path.splitext(os.path.basename(p))[0]
                    for p in glob.glob(os.path.join(TEM_DIR, "*.pt"))))
    print(f"템플릿 색 추출 — {len(names)} 객체, eps={a.eps_mm}mm, device={a.device}")
    bad = 0
    for n in names:
        s = build(n, a.eps_mm / 1000.0, torch.device(a.device), a.force)
        if s is None:
            continue
        print(f"  {n}: 뷰 {s['views']}, 색 찾은 점 {s['seen_frac']*100:.1f}%, "
              f"점당 평균 {s['mean_views']:.1f}뷰, 최근접거리 중앙 {s['med_min_mm']:.2f}mm "
              f"/ p95 {s['p95_min_mm']:.2f}mm")
        if s["seen_frac"] < 0.95:
            bad += 1
            print(f"    ⚠ {n}: 색을 못 찾은 점이 5% 넘는다 — eps 를 키우거나 템플릿을 확인할 것")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
