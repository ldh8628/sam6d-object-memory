#!/usr/bin/env python3
"""correspondence_quality.py — 초기 포즈를 만드는 '대응' 자체의 품질을 실제 데이터에서 잰다.

초기 포즈는 관측 196 점 ↔ 모델 196 점의 대응확률에서 나온다. 그 대응이 좋은지는
**정답 포즈 없이도** 잴 수 있다. 강체 대응이면 **점 사이 거리가 보존**되어야 하기 때문이다.

  · 등거리 오차 : 무작위 두 관측점의 거리와, 그 둘이 매칭된 모델점 사이 거리의 차이(지름 대비).
                 0 에 가까우면 대응이 일관되고, 크면 관측 조각이 물체 여기저기로 흩어져 붙은 것이다.
  · 대응 엔트로피: 관측점 하나가 모델점 몇 개에 걸쳐 확률을 나눠 갖는지(유효 후보 수).
  · 매칭 산포   : 매칭된 모델점들이 물체 표면에서 얼마나 퍼져 있는지(한쪽 면이면 작아야 한다).

같은 지표를 **합성 이상 조건**(렌더를 관측으로 넣은 경우)에서도 재서 기준선으로 삼는다.
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
TEM_DIR = {"choco_hazelnut_high": "choco_hazelnut_color_high", "Mugcup_high": "Mugcup_color_high"}


def metrics(net, ep, sample_pts_feats, cfs, rs):
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
        at = cfs(cm.out_proj(f1), cm.out_proj(f2), cm.cfg.sim_type, cm.cfg.temp,
                 cm.cfg.normalize_feat)
        ps = (torch.softmax(at, 2) * torch.softmax(at, 1))[0, 1:, 1:].cpu().numpy()
    P = sp_m[0].cpu().numpy()          # 관측 sparse (정규화 좌표)
    Q = sp_o[0].cpu().numpy()          # 모델 sparse
    dia = float(np.linalg.norm(Q.max(0) - Q.min(0)))
    w = ps.sum(1)
    keep = w > np.percentile(w, 40)    # 배경 토큰에 흡수된 약한 점은 뺀다
    j = ps.argmax(1)
    i_ = np.nonzero(keep)[0]
    if len(i_) < 20:
        return None
    ii = rs.choice(i_, min(2000, len(i_) * 8))
    jj = rs.choice(i_, len(ii))
    ok = ii != jj
    ii, jj = ii[ok], jj[ok]
    d_obs = np.linalg.norm(P[ii] - P[jj], axis=1)
    d_mod = np.linalg.norm(Q[j[ii]] - Q[j[jj]], axis=1)
    iso = float(np.median(np.abs(d_obs - d_mod)) / dia)
    p = ps[keep] / (ps[keep].sum(1, keepdims=True) + 1e-12)
    ent = float(np.median(np.exp(-(p * np.log(p + 1e-12)).sum(1))))   # 유효 후보 수
    spread = float(np.linalg.norm(Q[j[i_]].std(0)) / dia)
    return iso, ent, spread


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    ap.add_argument("--objects", default="Rabbit,Bear,Dinosaur,saffron,choco_hazelnut_high")
    a = ap.parse_args()
    os.makedirs(PP.TMP, exist_ok=True)
    objs = a.objects.split(",")
    ric, cfg, net, tem = PP.load_pem("cuda:0", objs)
    sys.path.insert(0, str(REPO / "sam6d_master/SAM-6D/Pose_Estimation_Model/utils"))
    from model_utils import sample_pts_feats, compute_feature_similarity as cfs
    rs = np.random.RandomState(0)
    real, syn = defaultdict(list), defaultdict(list)

    # --- 실제 관측 ---
    for p in sorted(Path(a.probe).iterdir()):
        if not (p / "meta.json").is_file():
            continue
        m = json.load(open(p / "meta.json"))
        K = np.array(m["cam_K"], float).reshape(3, 3)
        json.dump({"cam_K": K.flatten().tolist(), "depth_scale": 1.0},
                  open(f"{PP.TMP}/camera.json", "w"))
        lab = cv2.imread(str(p / "mask.png"), 0)
        for oi, o in enumerate(m["objects"]):
            if o not in objs:
                continue
            msk = (lab == oi + 1)
            if msk.sum() < 400:
                continue
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
                continue
            tp, tf = tem[o]
            ep["dense_po"] = tp
            ep["dense_fo"] = tf
            r = metrics(net, ep, sample_pts_feats, cfs, rs)
            if r:
                real[o].append(r)

    # --- 합성 기준선 (렌더를 관측인 척) ---
    for o in objs:
        tdir = REPO / "template" / TEM_DIR.get(o, o) / "templates"
        rgbs, ptss, chs = ric.get_templates(str(tdir), cfg.test_dataset)
        nv = len(rgbs)
        MP = torch.FloatTensor(np.load(REPO / "assets" / "model_points" / f"{o}.npy")
                               .astype(np.float32) / 1000.0).cuda()
        tp, tf = tem[o]
        for k in np.linspace(0, nv - 1, 8).astype(int):
            idx = torch.randperm(ptss[k].shape[1])[:cfg.test_dataset.n_sample_observed_point]
            ep = {"rgb": rgbs[k], "rgb_choose": chs[k][:, idx], "pts": ptss[k][:, idx, :],
                  "dense_po": tp, "dense_fo": tf,
                  "model": MP[torch.randperm(MP.shape[0])[:1024]].unsqueeze(0),
                  "score": torch.ones(1).cuda()}
            r = metrics(net, ep, sample_pts_feats, cfs, rs)
            if r:
                syn[o].append(r)

    print(f"\n{'객체':>22} | {'실제 관측':>28} | {'합성(이상)':>26}")
    print(f"{'':>22} | {'등거리오차':>10} {'유효후보수':>9} {'매칭산포':>7} | "
          f"{'등거리오차':>10} {'유효후보수':>9} {'매칭산포':>6}")
    for o in objs:
        if o not in real or o not in syn:
            continue
        R = np.array(real[o]); S = np.array(syn[o])
        print(f"{o:>22} | {np.median(R[:,0]):>10.3f} {np.median(R[:,1]):>9.1f} "
              f"{np.median(R[:,2]):>7.3f} | {np.median(S[:,0]):>10.3f} "
              f"{np.median(S[:,1]):>9.1f} {np.median(S[:,2]):>6.3f}   (n={len(R)})")
    print("\n등거리오차: 강체 대응이면 0. 클수록 관측 조각이 물체 여기저기로 흩어져 붙은 것.")
    print("유효후보수: 관측점 하나가 모델점 몇 개에 확률을 나눠 갖는가. 1 이면 확정, 크면 헷갈림.")


if __name__ == "__main__":
    main()
