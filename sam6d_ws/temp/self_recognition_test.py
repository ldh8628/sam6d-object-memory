#!/usr/bin/env python3
"""self_recognition_test.py — 물체가 원래 시점을 알아볼 수 있게 생겼는가 (상한 시험).

렌더 템플릿 한 장을 **관측인 척** PEM 에 넣는다. 그 장의 점군은 물체 좌표계 그대로이므로
정답 회전은 **단위행렬**이다. 도메인 갭도, depth 잡음도, 마스크 오류도 없다.
그 장은 물체 모델에서 **빼고**(leave-one-view-out) 나머지 41 장으로 모델을 만든다.

  여기서도 못 맞히면 → 물체가 어느 쪽에서 봐도 같다. 실사 사진을 줘도 안 된다.
  여기서는 맞히는데 실제 영상에서 틀리면 → 렌더↔실사 갭 또는 depth 품질 문제.
"""
import argparse, json, os, sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP

REPO = Path(__file__).resolve().parents[1]
TEM_DIR = {"choco_hazelnut_high": "choco_hazelnut_color_high", "Mugcup_high": "Mugcup_color_high"}


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def concentration(net, ep, v_of_pt, vdir, nkeep, sample_pts_feats, cfs, ccR):
    """대응 가중치가 뷰 방향에 얼마나 몰리는가 (1=한 방향, 0=고루 퍼짐).
    §11 에서 실제 영상으로 잰 값(Rabbit 0.28, choco 0.39~0.44)과 같은 정의다."""
    with torch.inference_mode():
        dpm, dfm, dpo, dfo, rad = net.feature_extraction(dict(ep))
        bg = torch.ones(1, 1, 3).float().cuda() * 100
        sp_m, sf_m, _ = sample_pts_feats(dpm, dfm, net.coarse_npoint, True)
        gm = net.geo_embedding(torch.cat([bg, sp_m], 1))
        sp_o, sf_o, fo = sample_pts_feats(dpo, dfo, net.coarse_npoint, True)
        go = net.geo_embedding(torch.cat([bg, sp_o], 1))
        cm = net.coarse_point_matching
        f1 = torch.cat([cm.bg_token, cm.in_proj(sf_m)], 1)
        f2 = torch.cat([cm.bg_token, cm.in_proj(sf_o)], 1)
        for i in range(cm.nblock):
            f1, f2 = cm.transformers[i](f1, gm, f2, go)
        at = cfs(cm.out_proj(f1), cm.out_proj(f2), cm.cfg.sim_type, cm.cfg.temp,
                 cm.cfg.normalize_feat)
        ps = (torch.softmax(at, 2) * torch.softmax(at, 1))[0, 1:, 1:].cpu().numpy()
    wj = ps.sum(0)
    vj = v_of_pt[fo[0].cpu().numpy()]
    vw = np.zeros(nkeep)
    for j, v in enumerate(vj):
        vw[v] += wj[j]
    vw /= max(vw.sum(), 1e-9)
    return float(np.linalg.norm((vw[:, None] * vdir).sum(0)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objects", default="Rabbit,Bear,Dinosaur,saffron,choco_hazelnut_high,Sikhye_high,milk")
    ap.add_argument("--nview", type=int, default=14, help="시험할 뷰 수 (42 중 균등)")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--ablate", default="none",
                    help="none | norgb(외형 제거) | noise<mm>(점군에 잡음) | blur(외형 흐리게)")
    a = ap.parse_args()
    os.makedirs(PP.TMP, exist_ok=True)
    objs = a.objects.split(",")
    ric, cfg, net, _ = PP.load_pem("cuda:0", objs)
    sys.path.insert(0, str(REPO / "sam6d_master/SAM-6D/Pose_Estimation_Model/utils"))
    from model_utils import sample_pts_feats, compute_feature_similarity as cfs, compute_coarse_Rt as ccR
    NP = net.feature_extraction.npoint
    print(f"{'객체':>22} {'시험 뷰':>7} {'회전오차 중앙':>13} {'p90':>7} {'<15° 비율':>10} "
          f"{'>60° 비율':>10} {'score':>7} {'시점집중도':>10}")
    for o in objs:
        tdir = REPO / "template" / TEM_DIR.get(o, o) / "templates"
        rgbs, ptss, chs = ric.get_templates(str(tdir), cfg.test_dataset)
        nv = len(rgbs)
        with torch.inference_mode():
            feats = [net.feature_extraction.get_img_feats(rgbs[k], chs[k]) for k in range(nv)]
        MP = torch.FloatTensor(
            np.load(REPO / "assets" / "model_points" / f"{o}.npy").astype(np.float32) / 1000.0
        ).cuda()
        test = np.linspace(0, nv - 1, a.nview).astype(int)
        errs, scs, concs = [], [], []
        for k in test:
            keep = [i for i in range(nv) if i != k]
            with torch.inference_mode():
                tp_all = torch.cat([ptss[i] for i in keep], 1)
                tf_all = torch.cat([feats[i] for i in keep], 1)
                dpo, dfo, oidx = sample_pts_feats(tp_all, tf_all, NP, True)
                npv = ptss[keep[0]].shape[1]
                v_of_pt = (oidx[0].cpu().numpy() // npv)      # 2048 점 -> keep 안의 뷰 번호
                vdir = np.stack([(lambda q: q / (np.linalg.norm(q) + 1e-9))(
                    ptss[i][0].cpu().numpy().mean(0)) for i in keep])
                # 관측 = 뺀 그 뷰. 정답 회전 = I, 이동 = 0
                idx = torch.randperm(ptss[k].shape[1])[:cfg.test_dataset.n_sample_observed_point]
                pts = ptss[k][:, idx, :]
                rch = chs[k][:, idx]
                mi = torch.randperm(MP.shape[0])[:cfg.test_dataset.n_sample_model_point]
                rgb_in = rgbs[k]
                pts_in = pts
                if a.ablate == "norgb":                 # 외형을 지운다(회색 한 장)
                    rgb_in = torch.zeros_like(rgbs[k])
                elif a.ablate == "blur":                # 실제 영상처럼 흐리고 대비 낮게
                    import torch.nn.functional as F
                    kk = torch.ones(3, 1, 5, 5, device=rgbs[k].device) / 25.0
                    rgb_in = F.conv2d(rgbs[k], kk, padding=2, groups=3) * 0.6
                elif a.ablate.startswith("noise"):      # 점군에 **무상관** 잡음 [mm]
                    sd = float(a.ablate[5:]) / 1000.0
                    pts_in = pts + torch.randn_like(pts) * sd
                elif a.ablate.startswith("bumpy"):
                    # 털처럼 **상관된** 요철: 파장 ~3 cm 의 저주파 변위장을 표면 밖으로 준다.
                    # 무상관 잡음과 달리 평균으로 지워지지 않으므로 형상 자체가 달라진다.
                    A = float(a.ablate[5:]) / 1000.0
                    c = pts.mean(1, keepdim=True)
                    n = torch.nn.functional.normalize(pts - c, dim=2)
                    f = 2 * np.pi / 0.03
                    ph = torch.rand(3, device=pts.device) * 6.283
                    d = (torch.sin(f * pts[..., 0] + ph[0]) + torch.sin(f * pts[..., 1] + ph[1])
                         + torch.sin(f * pts[..., 2] + ph[2])) / 3.0
                    pts_in = pts + n * (A * d).unsqueeze(-1)
                elif a.ablate.startswith("swaprgb"):    # 기하는 이 뷰, 외형은 다른 뷰 (갭 모사)
                    off = int(a.ablate[7:])
                    rgb_in = rgbs[(k + off) % nv]
                elif a.ablate.startswith("squash"):
                    # 봉제인형이 눌려 모양이 달라진 경우 (비강체 차이) — 한 축을 N% 압축
                    r_ = 1.0 - float(a.ablate[6:]) / 100.0
                    c = pts.mean(1, keepdim=True)
                    sc_ = torch.tensor([1.0, 1.0, r_], device=pts.device)
                    pts_in = (pts - c) * sc_ + c
                elif a.ablate.startswith("half"):
                    # 아래쪽(선반에 가려지는 부분)이 안 보이는 경우
                    c = pts.mean(1, keepdim=True)
                    keepm = (pts[0, :, 1] < c[0, 0, 1]).nonzero()[:, 0]
                    if len(keepm) > 50:
                        rep = keepm[torch.randint(0, len(keepm), (pts.shape[1],))]
                        pts_in = pts[:, rep, :]
                        rch = chs[k][:, idx][:, rep]
                elif a.ablate.startswith("lowres"):
                    # 실제처럼 물체가 화면에서 작을 때: crop 을 N px 로 줄였다 다시 224 로.
                    import torch.nn.functional as F
                    n_ = int(a.ablate[6:])
                    rgb_in = F.interpolate(F.interpolate(rgbs[k], size=(n_, n_),
                                                         mode="area"),
                                           size=rgbs[k].shape[-2:], mode="bilinear",
                                           align_corners=False)
                elif a.ablate.startswith("fewpts"):
                    # 실제처럼 유효 화소가 적을 때: N 개만 남기고 2048 로 중복 복원
                    n_ = int(a.ablate[6:])
                    sel = torch.randperm(pts.shape[1])[:n_]
                    rep = sel[torch.randint(0, n_, (pts.shape[1],))]
                    pts_in = pts[:, rep, :]
                    rch = chs[k][:, idx][:, rep]
                for _ in range(a.reps):
                    ep = {"rgb": rgb_in, "rgb_choose": rch, "pts": pts_in,
                          "dense_po": dpo, "dense_fo": dfo,
                          "model": MP[mi].unsqueeze(0),
                          "score": torch.ones(1).cuda()}
                    out = net(ep)
                    R = out["pred_R"][0].cpu().numpy()
                    errs.append(geo(np.eye(3), R))
                    scs.append(float(out["pred_pose_score"][0].cpu().numpy()))
                    concs.append(concentration(net, ep, v_of_pt, vdir, len(keep),
                                               sample_pts_feats, cfs, ccR))
        e = np.array(errs)
        print(f"{o:>22} {len(test):>7} {np.median(e):>13.1f} {np.percentile(e,90):>7.1f} "
              f"{(e<15).mean():>10.2f} {(e>60).mean():>10.2f} {np.median(scs):>7.3f} "
              f"{np.median(concs):>10.2f}")
    print("\n정답은 단위행렬(0°). 렌더를 그대로 넣었는데도 회전오차가 크면 물체 자체가 시점을 못 알려주는 것이다.")


if __name__ == "__main__":
    main()
