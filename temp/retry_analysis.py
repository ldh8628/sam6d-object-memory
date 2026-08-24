#!/usr/bin/env python3
"""retry_analysis.py — "불리한 표본을 뽑았을 때 되돌릴 장치가 없어서" 가설을 냉정하게 시험한다.

SAM-6D 의 초기 포즈 생성은 이렇게 되어 있다(`model_utils.py:189-248`).
  ① 대응확률로 3 점 대응을 **6,000 벌** 뽑는다 (torch.rand, 씨앗 없음)
  ② 3 점 잔차로 상위 **300 개**만 남긴다
  ③ 남은 300 개를 **관측 196 점 ↔ 모델 1024 점의 역평균거리** 로 채점해 1 개를 고른다
  ④ 정제는 그 1 개를 시작점으로 국소 보정만 한다 (실측 ≤50°)

즉 **선택 장치는 있다.** 없는 것은 (i) 결과가 나쁠 때 다시 뽑는 **외부 루프**,
(ii) 여러 실행/여러 프레임에 걸친 **합의**, (iii) 채점에 쓰는 **더 강한 기준**이다.

세 가지를 나눠 잰다.
  A) 가설 수를 6,000 → 24,000 → 96,000 으로 늘리면 결과가 수렴하는가?
     수렴하면 '표본 부족'이 맞고, 안 하면 '목적함수가 평평'한 것이다.
  B) 한 번의 실행에서 살아남은 300 개 후보는 서로 얼마나 다른가? 1 등과 10 등의 점수 차는?
     점수가 다닥다닥 붙어 있으면 채점 기준이 승자를 못 가르는 것이다.
  C) 여러 번 돌려 상위 후보를 모으면 지배적인 답이 생기는가? (외부 루프의 상한)
"""
import argparse, json, os, sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP
from segment_anything.utils.amg import mask_to_rle_pytorch

REPO = Path(__file__).resolve().parents[1]


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def coarse_candidates(net, ep, nprop1, nprop2, sample_pts_feats, cfs, pairwise_distance,
                      WeightedProcrustes):
    """compute_coarse_Rt 를 그대로 재현하되 **후보 전체와 점수**를 돌려준다."""
    with torch.inference_mode():
        dpm, dfm, dpo, dfo, rad = net.feature_extraction(dict(ep))
        bg = torch.ones(1, 1, 3).float().cuda() * 100
        sp_m, sf_m, _ = sample_pts_feats(dpm, dfm, net.coarse_npoint, True)
        gm = net.geo_embedding(torch.cat([bg, sp_m], 1))
        sp_o, sf_o, _ = sample_pts_feats(dpo, dfo, net.coarse_npoint, True)
        go = net.geo_embedding(torch.cat([bg, sp_o], 1))
        cm = net.coarse_point_matching
        f1 = torch.cat([cm.bg_token, cm.in_proj(sf_m)], 1)
        f2 = torch.cat([cm.bg_token, cm.in_proj(sf_o)], 1)
        for i in range(cm.nblock):
            f1, f2 = cm.transformers[i](f1, gm, f2, go)
        atten = cfs(cm.out_proj(f1), cm.out_proj(f2), cm.cfg.sim_type, cm.cfg.temp,
                    cm.cfg.normalize_feat)
        pts1, pts2 = sp_m, sp_o
        model_pts = ep["model"] / (rad.reshape(-1, 1, 1) + 1e-6)
        B, N1, _ = pts1.size(); N2 = pts2.size(1)
        WSVD = WeightedProcrustes()
        ps = torch.softmax(atten, 2) * torch.softmax(atten, 1)
        l1 = torch.max(ps[:, 1:, :], 2)[1]; l2 = torch.max(ps[:, :, 1:], 1)[1]
        w1 = (l1 > 0).float(); w2 = (l2 > 0).float()
        ps = ps[:, 1:, 1:].contiguous() * w1.unsqueeze(2) * w2.unsqueeze(1)
        ps = ps.reshape(B, N1 * N2) ** 1.5
        cw = torch.cumsum(ps, 1); cw = cw / (cw[:, -1].unsqueeze(1) + 1e-8)
        idx = torch.searchsorted(cw, torch.rand(B, nprop1 * 3, device=pts1.device))
        i1 = torch.clamp(idx.div(N2, rounding_mode='floor'), max=N1 - 1).unsqueeze(2).repeat(1, 1, 3)
        i2 = torch.clamp(idx % N2, max=N2 - 1).unsqueeze(2).repeat(1, 1, 3)
        p1 = torch.gather(pts1, 1, i1).reshape(B * nprop1, 3, 3)
        p2 = torch.gather(pts2, 1, i2).reshape(B * nprop1, 3, 3)
        Rs, ts = WSVD(p2, p1, None)
        Rs = Rs.reshape(B, nprop1, 3, 3); ts = ts.reshape(B, nprop1, 1, 3)
        p1 = p1.reshape(B, nprop1, 3, 3); p2 = p2.reshape(B, nprop1, 3, 3)
        d = torch.norm((p1 - ts) @ Rs - p2, dim=3).mean(2)
        sel = torch.topk(d, nprop2, dim=1, largest=False)[1]
        Rs = torch.gather(Rs, 1, sel.reshape(B, nprop2, 1, 1).repeat(1, 1, 3, 3))
        ts = torch.gather(ts, 1, sel.reshape(B, nprop2, 1, 1).repeat(1, 1, 1, 3))
        tp = (pts1.unsqueeze(1) - ts) @ Rs
        emp = model_pts.unsqueeze(1).repeat(1, nprop2, 1, 1).reshape(B * nprop2, -1, 3)
        dd = torch.sqrt(pairwise_distance(tp.reshape(B * nprop2, -1, 3), emp))
        dd = dd.min(2)[0].reshape(B, nprop2, -1)
        sc = w1.unsqueeze(1).sum(2) / ((dd * w1.unsqueeze(1)).sum(2) + 1e-8)
    return (Rs[0].cpu().numpy(), ts[0].cpu().numpy(), sc[0].cpu().numpy(),
            float(rad[0].item()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    ap.add_argument("--objects", default="Rabbit,Bear,saffron")
    ap.add_argument("--reps", type=int, default=8)
    a = ap.parse_args()
    os.makedirs(PP.TMP, exist_ok=True)
    objs = a.objects.split(",")
    ric, cfg, net, tem = PP.load_pem("cuda:0", objs)
    sys.path.insert(0, str(REPO / "sam6d_master/SAM-6D/Pose_Estimation_Model/utils"))
    from model_utils import (sample_pts_feats, compute_feature_similarity as cfs,
                             pairwise_distance, WeightedProcrustes)
    frames = defaultdict(list)
    for p in sorted(Path(a.probe).iterdir()):
        if not (p / "meta.json").is_file():
            continue
        m = json.load(open(p / "meta.json"))
        for o in m["objects"]:
            if o in objs:
                frames[o].append(p)

    def make_ep(p, o):
        m = json.load(open(p / "meta.json"))
        K = np.array(m["cam_K"], float).reshape(3, 3)
        json.dump({"cam_K": K.flatten().tolist(), "depth_scale": 1.0},
                  open(f"{PP.TMP}/camera.json", "w"))
        lab = cv2.imread(str(p / "mask.png"), 0)
        msk = (lab == m["objects"].index(o) + 1)
        if msk.sum() < 400:
            return None
        h, w = msk.shape
        rle = mask_to_rle_pytorch(torch.from_numpy(msk).unsqueeze(0))[0]
        ys, xs = np.where(msk)
        json.dump([{"scene_id": 0, "image_id": 0, "category_id": 1,
                    "bbox": [int(xs.min()), int(ys.min()),
                             int(xs.max() - xs.min()), int(ys.max() - ys.min())],
                    "score": 1.0,
                    "segmentation": {"size": [h, w],
                                     "counts": [int(c) for c in rle["counts"]]}}],
                  open(f"{PP.TMP}/det.json", "w"))
        try:
            ep = ric.get_test_data(str(p / "rgb.png"), str(p / "depth.png"),
                                   f"{PP.TMP}/camera.json", o, f"{PP.TMP}/det.json",
                                   0.2, cfg.test_dataset)[0]
        except Exception:
            return None
        ep["dense_po"], ep["dense_fo"] = tem[o]
        return ep

    print("\n=== A) 가설 수를 늘리면 수렴하는가 (같은 입력 반복, 초기 포즈끼리 비교) ===")
    print(f"{'객체':>10} {'가설 수':>8} {'반복 Δrot 중앙':>14} {'p90':>8} {'>60° 비율':>10}")
    for o in objs:
        for np1 in (6000, 24000, 96000):
            sp = []
            for p in frames[o][:10]:
                ep = make_ep(p, o)
                if ep is None:
                    continue
                Rs = []
                for _ in range(a.reps):
                    R, t, sc, rad = coarse_candidates(net, ep, np1, 300, sample_pts_feats,
                                                      cfs, pairwise_distance, WeightedProcrustes)
                    Rs.append(R[int(sc.argmax())])
                dd = [geo(Rs[i], Rs[j]) for i in range(len(Rs)) for j in range(i + 1, len(Rs))]
                sp.append((float(np.median(dd)), float(np.mean(np.array(dd) > 60))))
            if sp:
                S = np.array(sp)
                print(f"{o:>10} {np1:>8} {np.median(S[:,0]):>14.1f} "
                      f"{np.percentile(S[:,0],90):>8.1f} {np.mean(S[:,1]):>10.2f}")

    print("\n=== B) 살아남은 300 후보의 상태 (한 번의 실행) ===")
    print(f"{'객체':>10} {'1등 vs 10등 점수비':>17} {'1등 vs 100등':>13} "
          f"{'상위10개 서로 Δrot':>18} {'300개 서로 Δrot':>15}")
    for o in objs:
        r1, r2, s10, s300 = [], [], [], []
        for p in frames[o][:10]:
            ep = make_ep(p, o)
            if ep is None:
                continue
            R, t, sc, rad = coarse_candidates(net, ep, 6000, 300, sample_pts_feats, cfs,
                                              pairwise_distance, WeightedProcrustes)
            k = np.argsort(-sc)
            r1.append(sc[k[9]] / (sc[k[0]] + 1e-9))
            r2.append(sc[k[99]] / (sc[k[0]] + 1e-9))
            top = [R[i] for i in k[:10]]
            s10.append(np.median([geo(top[i], top[j]) for i in range(10)
                                  for j in range(i + 1, 10)]))
            q = [R[i] for i in k[np.random.RandomState(0).choice(300, 20, replace=False)]]
            s300.append(np.median([geo(q[i], q[j]) for i in range(len(q))
                                   for j in range(i + 1, len(q))]))
        if r1:
            print(f"{o:>10} {np.median(r1):>17.3f} {np.median(r2):>13.3f} "
                  f"{np.median(s10):>18.1f} {np.median(s300):>15.1f}")
    print("\n1등 대비 10등 점수비가 1 에 가까우면 채점 기준이 승자를 못 가른다는 뜻이다.")
    paired(ric, cfg, net, tem, frames, objs, make_ep, a.reps,
           sample_pts_feats, cfs, pairwise_distance, WeightedProcrustes)


def paired(ric, cfg, net, tem, frames, objs, make_ep, reps,
           sample_pts_feats, cfs, pairwise_distance, WeightedProcrustes):
    """상위 후보 산포(한 번 계산, 공짜)가 실제 불안정성(여러 번 돌려야 앎)을 예측하는가."""
    print("\n=== C) '상위 10 후보 산포' 가 그 프레임의 불안정성을 예측하는가 ===")
    print(f"{'객체':>10} {'n':>4} {'상관계수':>8}   {'산포<10° 인 프레임':>18} {'산포>25° 인 프레임':>18}")
    for o in objs:
        X, Y = [], []
        for p in frames[o][:14]:
            ep = make_ep(p, o)
            if ep is None:
                continue
            R, t, sc, rad = coarse_candidates(net, ep, 6000, 300, sample_pts_feats, cfs,
                                              pairwise_distance, WeightedProcrustes)
            k = np.argsort(-sc)[:10]
            spread = float(np.median([geo(R[k[i]], R[k[j]]) for i in range(10)
                                      for j in range(i + 1, 10)]))
            Rs = []
            for _ in range(reps):
                R2, t2, sc2, _ = coarse_candidates(net, ep, 6000, 300, sample_pts_feats, cfs,
                                                   pairwise_distance, WeightedProcrustes)
                Rs.append(R2[int(sc2.argmax())])
            dd = float(np.median([geo(Rs[i], Rs[j]) for i in range(len(Rs))
                                  for j in range(i + 1, len(Rs))]))
            X.append(spread); Y.append(dd)
        if len(X) > 4:
            X, Y = np.array(X), np.array(Y)
            lo = Y[X < 10]; hi = Y[X > 25]
            print(f"{o:>10} {len(X):>4} {np.corrcoef(X, Y)[0,1]:>8.2f}   "
                  f"{'실제 Δrot '+format(np.median(lo),'.1f')+'°' if len(lo) else '-':>18} "
                  f"{'실제 Δrot '+format(np.median(hi),'.1f')+'°' if len(hi) else '-':>18}")


if __name__ == "__main__":
    main()
