#!/usr/bin/env python3
"""point_distinctiveness.py — 초기 포즈의 진짜 재료를 잰다: **표면 점이 서로 구별되는가**.

PEM 의 초기 포즈는 "관측 점 196 개 ↔ 모델 점 196 개"의 대응확률에서 나온다. 대응이 서게 하려면
모델의 각 표면 점이 **다른 표면 점과 구별**되어야 한다. 토끼처럼 표면이 균일하면
"이 흰 곡면 조각"이 몸통 앞·뒤·머리 어디에도 똑같이 잘 붙는다 → 대응이 흩어진다
→ 6,000 개 가설이 사실상 무작위 분포에서 뽑힌다.

두 가지를 잰다.
  (1) **retrieval@10**: 각 모델 점에 대해 특징이 가장 닮은 10 개를 뽑았을 때,
      그중 실제로 **공간적으로 가까운**(지름의 15% 이내) 점의 비율. 1 이면 완벽히 구별,
      0 이면 엉뚱한 곳과 헷갈린다.
  (2) **혼동 거리**: 가장 닮은 '멀리 있는' 점까지의 3D 거리(지름 대비)와 그때의 유사도.

⚠ 참고: 모델 점의 '출처 뷰' 라벨은 의미가 없다. 42 뷰가 겹쳐 같은 표면을 여러 번 담고,
FPS 는 그중 하나만 임의로 남기므로 라벨은 사실상 무작위다. (그래서 뷰별 가중치 히스토그램은
정상 동작할 때도 평평하다 — 11 절의 해석은 이 스크립트로 철회한다.)
"""
import argparse, sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP

REPO = Path(__file__).resolve().parents[1]
TEM_DIR = {"choco_hazelnut_high": "choco_hazelnut_color_high", "Mugcup_high": "Mugcup_color_high"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objects",
                    default="Rabbit,Bear,Dinosaur,saffron,choco_hazelnut_high,Sikhye_high,milk,Febreze_high")
    a = ap.parse_args()
    objs = a.objects.split(",")
    ric, cfg, net, _ = PP.load_pem("cuda:0", objs)
    sys.path.insert(0, str(REPO / "sam6d_master/SAM-6D/Pose_Estimation_Model/utils"))
    from model_utils import sample_pts_feats
    print(f"{'객체':>22} {'retrieval@10':>13} {'혼동 유사도':>11} {'혼동 거리(지름대비)':>18} "
          f"{'뷰라벨 무의미도':>15}")
    for o in objs:
        tdir = REPO / "template" / TEM_DIR.get(o, o) / "templates"
        rgbs, ptss, chs = ric.get_templates(str(tdir), cfg.test_dataset)
        nv, npv = len(rgbs), ptss[0].shape[1]
        with torch.inference_mode():
            fs = [net.feature_extraction.get_img_feats(rgbs[k], chs[k]) for k in range(nv)]
            P = torch.cat(ptss, 1)
            F = torch.cat(fs, 1)
            dpo, dfo, idx = sample_pts_feats(P, F, net.feature_extraction.npoint, True)
        pts = dpo[0].cpu().numpy()
        f = dfo[0].float().cpu().numpy()
        f = f / (np.linalg.norm(f, axis=1, keepdims=True) + 1e-9)
        dia = float(np.linalg.norm(pts.max(0) - pts.min(0)))
        S = f @ f.T
        np.fill_diagonal(S, -2)
        D = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
        near = D < 0.15 * dia
        top = np.argsort(-S, axis=1)[:, :10]
        ret = np.take_along_axis(near, top, 1).mean()
        Sfar = np.where(near, -2, S)
        j = Sfar.argmax(1)
        conf_sim = Sfar[np.arange(len(f)), j]
        conf_dist = D[np.arange(len(f)), j] / dia
        # 뷰 라벨이 얼마나 무의미한가: 같은 표면 위치를 여러 뷰가 담고 있는 비율
        vlab = idx[0].cpu().numpy() // npv
        allv = (np.arange(P.shape[1]) // npv)
        Pn = P[0].cpu().numpy()
        # 각 선택된 점 근처(1% 지름)에 몇 개 뷰의 점이 있었나
        k = np.random.RandomState(0).choice(len(pts), 200, replace=False)
        nviews = []
        for i in k:
            d = np.linalg.norm(Pn - pts[i], axis=1)
            nviews.append(len(np.unique(allv[d < 0.01 * dia])))
        print(f"{o:>22} {ret:>13.3f} {np.median(conf_sim):>11.3f} {np.median(conf_dist):>18.2f} "
              f"{np.median(nviews):>15.0f} 개 뷰가 같은 점 보유")
    print("\nretrieval@10 이 높을수록 '표면의 각 부분이 서로 구별된다'는 뜻이다.")
    print("낮으면 관측 조각이 물체의 엉뚱한 부위에도 똑같이 잘 붙어 대응이 흩어진다.")


if __name__ == "__main__":
    main()
