#!/usr/bin/env python3
"""view_separability.py — 두 질문을 분리해서 잰다.

  Q-A. **렌더끼리** 시점이 구별되는가?  (물체 자체의 성질)
       42 장 템플릿의 특징을 서로 비교해, 어떤 뷰의 '가장 닮은 다른 뷰'가
       실제로 가까운 시점인지 아니면 반대편인지 본다.
  Q-B. **실사가 옳은 렌더에 붙는가?** (렌더↔실사 도메인 갭)
       실제 관측 crop 의 특징을 42 장과 비교해 봉우리가 뾰족한지 평평한지 본다.

  A 가 나쁘면 물체가 원래 어느 쪽에서 봐도 같은 것이고(사진을 더 줘도 소용없음),
  A 는 좋은데 B 가 나쁘면 도메인 갭이므로 **실사 몇 장으로 보정하는 방향이 맞다.**

특징은 PEM 이 실제로 쓰는 그 ViT 특징(`feature_extraction.get_img_feats`)을 그대로 쓴다.
"""
import argparse, json, os, sys
from pathlib import Path

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP

REPO = Path(__file__).resolve().parents[1]
TEM_DIR = {"choco_hazelnut_high": "choco_hazelnut_color_high", "Mugcup_high": "Mugcup_color_high"}


def desc_from(net, rgb_t, choose_t):
    with torch.inference_mode():
        f = net.feature_extraction.get_img_feats(rgb_t, choose_t)   # (1, N, C)
    d = f.mean(1)[0].float().cpu().numpy()
    return d / (np.linalg.norm(d) + 1e-9)


def real_crop(ric, cfg, frame, obj, device):
    meta = json.load(open(frame / "meta.json"))
    oi = meta["objects"].index(obj)
    lab = cv2.imread(str(frame / "mask.png"), 0)
    rgb = ric.load_im(str(frame / "rgb.png")).astype(np.uint8)
    m = (lab == oi + 1)
    if m.sum() < 400:
        return None
    y1, y2, x1, x2 = ric.get_bbox(m)
    mc = m[y1:y2, x1:x2]
    rc = rgb[:, :, ::-1][y1:y2, x1:x2, :]
    if cfg.test_dataset.rgb_mask_flag:
        rc = rc * (mc[:, :, None] > 0).astype(np.uint8)
    rc = cv2.resize(rc, (cfg.test_dataset.img_size,) * 2, interpolation=cv2.INTER_LINEAR)
    rc = ric.rgb_transform(np.array(rc))
    ch = (mc > 0).astype(np.float32).flatten().nonzero()[0]
    n = cfg.test_dataset.n_sample_template_point
    ch = ch[np.random.choice(np.arange(len(ch)), n, replace=len(ch) <= n)]
    rch = ric.get_resize_rgb_choose(ch, [y1, y2, x1, x2], cfg.test_dataset.img_size)
    return (torch.FloatTensor(rc).unsqueeze(0).to(device),
            torch.IntTensor(rch).long().unsqueeze(0).to(device))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    ap.add_argument("--objects", default="Rabbit,Bear,Dinosaur,saffron,choco_hazelnut_high,Sikhye_high")
    a = ap.parse_args()
    os.makedirs(PP.TMP, exist_ok=True)
    objs = a.objects.split(",")
    ric, cfg, net, _ = PP.load_pem("cuda:0", objs)
    print(f"{'객체':>22} {'렌더끼리 최고유사도':>18} {'그 상대가 60°+ 먼 뷰':>20} "
          f"{'실사 봉우리 뾰족함':>18} {'상위5뷰 각퍼짐':>15} {'n실사':>6}")
    for o in objs:
        tdir = REPO / "template" / TEM_DIR.get(o, o) / "templates"
        tem = ric.get_templates(str(tdir), cfg.test_dataset)          # rgb, pts, choose
        nv = len(tem[0])
        D = np.stack([desc_from(net, tem[0][k], tem[2][k]) for k in range(nv)])
        VD = np.stack([(lambda q: q / (np.linalg.norm(q) + 1e-9))(
            tem[1][k][0].cpu().numpy().mean(0)) for k in range(nv)])
        S = D @ D.T
        np.fill_diagonal(S, -1)
        best = S.argmax(1)
        ang = np.degrees(np.arccos(np.clip((VD * VD[best]).sum(1), -1, 1)))
        conf = float((ang > 60).mean())
        # 실사
        sharp, spread, n = [], [], 0
        for p in sorted(Path(a.probe).iterdir()):
            if not (p / "meta.json").is_file():
                continue
            m = json.load(open(p / "meta.json"))
            if o not in m["objects"]:
                continue
            rc = real_crop(ric, cfg, p, o, "cuda:0")
            if rc is None:
                continue
            d = desc_from(net, rc[0], rc[1])
            s = D @ d
            sharp.append(float((s.max() - s.mean()) / (s.std() + 1e-9)))
            top = np.argsort(s)[::-1][:5]
            v = VD[top].mean(0)
            spread.append(float(np.degrees(np.arccos(np.clip(
                np.percentile((VD[top] @ (v / (np.linalg.norm(v) + 1e-9))), 10), -1, 1)))))
            n += 1
        # 렌더 자기끼리의 봉우리 뾰족함(기준선)
        rs = []
        for k in range(nv):
            s = D @ D[k]
            s[k] = -1
            rs.append(float((s.max() - s.mean()) / (s.std() + 1e-9)))
        print(f"{o:>22} {S.max(1).mean():>18.3f} {conf:>20.2f} "
              f"{(np.median(sharp) if sharp else float('nan')):>18.2f} "
              f"{(np.median(spread) if spread else float('nan')):>15.0f} {n:>6}"
              f"   (렌더 기준선 {np.median(rs):.2f})")
    print("\n· '렌더끼리 최고유사도'가 1 에 가깝고 '그 상대가 60°+ 먼 뷰' 비율이 높으면")
    print("  → 물체가 어느 쪽에서 봐도 같다 = 사진을 더 줘도 안 된다.")
    print("· '실사 봉우리 뾰족함'이 렌더 기준선보다 크게 낮으면 → 도메인 갭이 문제다.")


if __name__ == "__main__":
    main()
