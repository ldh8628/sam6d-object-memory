#!/usr/bin/env python3
"""inspect_pem_init.py — PEM 이 초기 포즈를 어떻게 만들고 무엇으로 정제하는지 한 프레임에서 해부한다.

Net.forward 를 그대로 다시 쓰되 중간값을 전부 붙잡는다.

  1) 42 장 템플릿 -> 각 뷰의 표면점 5,000 개씩 = 210,000 점을 **하나로 이어 붙인 뒤**
     FPS 로 2,048 점만 남긴다. 그 2,048 점이 '물체 모델'이다.
     여기서 각 점이 **몇 번 템플릿에서 왔는지**를 같이 들고 간다(원본에는 없는 계측).
  2) 관측/모델 각각에서 다시 FPS 로 196 점을 뽑아 트랜스포머로 대응확률 행렬을 만든다.
  3) 그 확률로 3점 대응을 6,000 벌 뽑아 SVD 로 포즈 후보를 만들고,
     3점 잔차 상위 300 개를 남긴 뒤, 전체 점을 가장 잘 설명하는 하나를 고른다 = **초기 포즈**.
  4) 초기 포즈를 시작점으로 2,048 점으로 다시 대응을 풀어 최종 포즈를 낸다 = **정제**.

그림: 관측 | 초기 포즈 | 최종 포즈 | 대응이 많이 걸린 템플릿 뷰들 | 뷰별 대응 가중치 막대.
"""
import argparse, json, os, sys
from pathlib import Path

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP
from segment_anything.utils.amg import mask_to_rle_pytorch

REPO = Path(__file__).resolve().parents[1]
TEM_DIR = {"choco_hazelnut_high": "choco_hazelnut_color_high",
           "Mugcup_high": "Mugcup_color_high"}


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", required=True, help="probe/<stamp> 폴더, 여러 개면 콤마")
    ap.add_argument("--object", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(PP.TMP, exist_ok=True)
    frames = [Path(x) for x in a.frame.split(",")]
    meta0 = json.load(open(frames[0] / "meta.json"))
    o = a.object
    names = sorted({x for f in frames for x in json.load(open(f / "meta.json"))["objects"]})
    ric, cfg, net, tem = PP.load_pem("cuda:0", names)
    sys.path.insert(0, str(REPO / "sam6d_master/SAM-6D/Pose_Estimation_Model/utils"))
    from model_utils import (sample_pts_feats, compute_feature_similarity,
                             compute_coarse_Rt)

    # ---------- 1) 42 장 템플릿 -> 물체 모델 (출처 뷰를 같이 들고 간다) ----------
    tdir = REPO / "template" / TEM_DIR.get(o, o) / "templates"
    all_tem, all_tem_pts, all_tem_choose = ric.get_templates(str(tdir), cfg.test_dataset)
    nview = len(all_tem)
    npv = all_tem_pts[0].shape[1]
    with torch.inference_mode():
        feats = [net.feature_extraction.get_img_feats(t, c)
                 for t, c in zip(all_tem, all_tem_choose)]
        tem_pts_all = torch.cat(all_tem_pts, dim=1)              # (1, 42*5000, 3)
        tem_feat_all = torch.cat(feats, dim=1)
        # ⚠ 정규화는 feature_extraction 이 다시 한다. 여기서 미리 나누면 두 번 나뉘어
        #    관측과 스케일이 어긋난다(실제로 그렇게 틀렸다). 원 단위 그대로 넘긴다.
        dense_po, dense_fo, obj_idx = sample_pts_feats(
            tem_pts_all, tem_feat_all, net.feature_extraction.npoint, return_index=True)
    view_of_point = (obj_idx[0].cpu().numpy() // npv)             # 2048 점 -> 뷰 번호
    # 각 템플릿 뷰가 물체를 '어느 방향에서' 본 것인지 (보이는 표면의 중심 방향)
    view_dir = np.stack([(lambda q: q / (np.linalg.norm(q) + 1e-9))(
        all_tem_pts[k][0].cpu().numpy().mean(0)) for k in range(nview)])
    print(f"[모델] 템플릿 {nview} 장 × {npv} 점 = {nview*npv} 점 -> FPS 2048 점")
    print(f"       그 2048 점이 나온 뷰의 개수: {len(set(view_of_point.tolist()))} 개 "
          f"(즉 한 장을 고르는 게 아니라 42 장을 섞어 쓴다)")

    results = []
    for FR in frames:
        meta = json.load(open(FR / "meta.json"))
        if o not in meta["objects"]:
            continue
        r = run_one(FR, meta, o, a, ric, cfg, net, tem, dense_po, dense_fo,
                    view_of_point, nview, tdir, view_dir, sample_pts_feats,
                    compute_feature_similarity, compute_coarse_Rt)
        results.append((FR.name,) + r)
    print(f"\n{'프레임':>14} {'초기->정제 회전':>15} {'위치mm':>8} {'score':>7} "
          f"{'시점집중도':>10} {'상위뷰비중':>10}")
    for n, dR, dt, sc, top, cc, mx in results:
        print(f"{n[-10:]:>14} {dR:>15.1f} {dt:>8.0f} {sc:>7.3f} {cc:>10.2f} {mx*100:>9.1f}%")
    import numpy as _np
    A = _np.array([[r[1], r[3], r[4]] for r in results])
    if len(A):
        print(f"\n초기->정제 회전 변화: 중앙 {_np.median(A[:,0]):.1f}° · p90 {_np.percentile(A[:,0],90):.1f}° "
              f"· 최대 {A[:,0].max():.1f}°   (score 중앙 {_np.median(A[:,1]):.2f})")
        big = A[:, 0] > 60
        print(f"정제가 60° 넘게 바꾼 프레임 비율 {big.mean():.2f}")
        print(f"시점 집중도: 중앙 {_np.median(A[:,2]):.2f}  (1=한 방향에 몰림, 0=42 뷰에 고루 퍼짐)")


def run_one(FR, meta, o, a, ric, cfg, net, tem, dense_po, dense_fo, view_of_point,
            nview, tdir, view_dir, sample_pts_feats, compute_feature_similarity, compute_coarse_Rt):
    import torch, numpy as np, cv2, json, os
    # ---------- 관측 입력 ----------
    K = np.array(meta["cam_K"], float).reshape(3, 3)
    json.dump({"cam_K": K.flatten().tolist(), "depth_scale": 1.0},
              open(f"{PP.TMP}/camera.json", "w"))
    lab = cv2.imread(str(FR / "mask.png"), 0)
    oi = meta["objects"].index(o)
    msk = (lab == oi + 1)
    h, w = msk.shape
    rle = mask_to_rle_pytorch(torch.from_numpy(msk).unsqueeze(0))[0]
    ys, xs = np.where(msk)
    json.dump([{"scene_id": 0, "image_id": 0, "category_id": 1,
                "bbox": [int(xs.min()), int(ys.min()),
                         int(xs.max() - xs.min()), int(ys.max() - ys.min())],
                "score": 1.0,
                "segmentation": {"size": [h, w], "counts": [int(c) for c in rle["counts"]]}}],
              open(f"{PP.TMP}/det.json", "w"))
    ep = ric.get_test_data(str(FR / "rgb.png"), str(FR / "depth.png"),
                           f"{PP.TMP}/camera.json", o, f"{PP.TMP}/det.json",
                           0.2, cfg.test_dataset)[0]

    # ---------- 2~4) Net.forward 를 손으로 다시 쓰며 중간값을 잡는다 ----------
    with torch.inference_mode():
        ep["dense_po"] = dense_po
        ep["dense_fo"] = dense_fo
        dense_pm, dense_fm, dense_po_n, dense_fo_n, rad = net.feature_extraction(ep)
        bg = torch.ones(1, 1, 3).float().cuda() * 100
        sp_m, sf_m, fps_m = sample_pts_feats(dense_pm, dense_fm, net.coarse_npoint, True)
        geo_m = net.geo_embedding(torch.cat([bg, sp_m], 1))
        sp_o, sf_o, fps_o = sample_pts_feats(dense_po_n, dense_fo_n, net.coarse_npoint, True)
        geo_o = net.geo_embedding(torch.cat([bg, sp_o], 1))

        cm = net.coarse_point_matching
        f1 = torch.cat([cm.bg_token, cm.in_proj(sf_m)], 1)
        f2 = torch.cat([cm.bg_token, cm.in_proj(sf_o)], 1)
        for i in range(cm.nblock):
            f1, f2 = cm.transformers[i](f1, geo_m, f2, geo_o)
        atten = compute_feature_similarity(cm.out_proj(f1), cm.out_proj(f2),
                                           cm.cfg.sim_type, cm.cfg.temp, cm.cfg.normalize_feat)
        init_R, init_t = compute_coarse_Rt(atten, sp_m, sp_o,
                                           ep["model"] / (rad.reshape(-1, 1, 1) + 1e-6),
                                           cm.cfg.nproposal1, cm.cfg.nproposal2)
        ep["init_R"], ep["init_t"] = init_R, init_t
        ep = net.fine_point_matching(dense_pm, dense_fm, geo_m, fps_m,
                                     dense_po_n, dense_fo_n, geo_o, fps_o, rad, ep)

    R_init = init_R[0].cpu().numpy()
    t_init = (init_t[0].cpu().numpy() * rad[0].item()) * 1000.0
    R_fin = ep["pred_R"][0].cpu().numpy()
    t_fin = ep["pred_t"][0].cpu().numpy() * 1000.0
    score = float(ep["pred_pose_score"][0].cpu().numpy())
    print(f"[초기 포즈] 6000 개 후보 중 선택.  [정제 후] 회전이 {geo(R_init, R_fin):.1f}° "
          f"움직였고 위치는 {np.linalg.norm(t_fin - t_init):.0f} mm 움직였다. score={score:.3f}")

    # 대응 가중치를 뷰별로 집계
    ps = (torch.softmax(atten, 2) * torch.softmax(atten, 1))[0, 1:, 1:].cpu().numpy()
    wj = ps.sum(0)                                    # 모델 sparse 196 점의 가중치
    vj = view_of_point[fps_o[0].cpu().numpy()]        # 그 196 점이 온 뷰
    vw = np.zeros(nview)
    for j, v in enumerate(vj):
        vw[v] += wj[j]
    vw = vw / max(vw.sum(), 1e-9)
    top = np.argsort(vw)[::-1][:5]
    print(f"[대응] 196 개 모델 점이 온 뷰 = {len(set(vj.tolist()))} 종류. "
          f"가중치 상위 뷰 {top.tolist()} (합쳐 {vw[top].sum()*100:.0f}%)")
    conc = float(np.linalg.norm((vw[:, None] * view_dir).sum(0)))   # 0=전 방향 분산, 1=한 방향 집중
    print(f"       상위 1개 비중 {vw.max()*100:.1f}%  ·  시점 집중도 {conc:.2f}")

    # ---------- 그림 ----------
    bgr = cv2.imread(str(FR / "rgb.png"))
    P = np.load(REPO / "assets" / "model_points" / f"{o}.npy").astype(np.float64)
    x1, y1, x2, y2 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
    pad = 30
    X1, Y1 = max(0, x1 - pad), max(0, y1 - pad)
    X2, Y2 = min(w, x2 + pad), min(h, y2 + pad)
    SZ = 230

    def panel(R, t, txt, col):
        im = bgr.copy()
        q = (R @ P.T).T + t
        q = q[q[:, 2] > 1]
        u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(int)
        v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(int)
        ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        im[v[ok], u[ok]] = col
        c = cv2.resize(im[Y1:Y2, X1:X2], (SZ, SZ), interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(c, (0, SZ - 20), (SZ, SZ), (0, 0, 0), -1)
        cv2.putText(c, txt, (4, SZ - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
        return c

    obs = bgr.copy()
    cs, _ = cv2.findContours(msk.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(obs, cs, -1, (0, 255, 255), 1)
    obs = cv2.resize(obs[Y1:Y2, X1:X2], (SZ, SZ), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(obs, (0, SZ - 20), (SZ, SZ), (0, 0, 0), -1)
    cv2.putText(obs, "observed + ISM mask", (4, SZ - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                (255, 255, 255), 1)
    row1 = [obs,
            panel(R_init, t_init, f"init (coarse)  ", (80, 160, 255)),
            panel(R_fin, t_fin, f"final (fine)  d{geo(R_init,R_fin):.0f}deg  s{score:.2f}",
                  (60, 230, 60))]
    # 상위 뷰 렌더
    for k in top[:3]:
        r = cv2.imread(str(tdir / f"rgb_{int(k)}.png"))
        m2 = cv2.imread(str(tdir / f"mask_{int(k)}.png"), 0)
        yy, xx = np.nonzero(m2 > 127)
        c = cv2.resize(r[yy.min():yy.max() + 1, xx.min():xx.max() + 1], (SZ, SZ))
        cv2.rectangle(c, (0, SZ - 20), (SZ, SZ), (0, 0, 0), -1)
        cv2.putText(c, f"template #{int(k)}  w={vw[k]*100:.0f}%", (4, SZ - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
        row1.append(c)
    W = SZ * len(row1)
    bar = np.full((110, W, 3), 22, np.uint8)
    cv2.putText(bar, "correspondence weight per template view (42)", (8, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
    for i in range(nview):
        x = int(20 + i * (W - 40) / nview)
        hgt = int(vw[i] / max(vw.max(), 1e-9) * 70)
        cv2.rectangle(bar, (x, 100), (x + max(2, int((W - 40) / nview) - 3), 100 - hgt),
                      (90, 200, 255) if i in top[:3] else (110, 110, 110), -1)
    if a.out:
        out = a.out if len(a.frame.split(",")) == 1 else f"{a.out[:-4]}_{FR.name[-8:]}.png"
        cv2.imwrite(out, np.vstack([np.hstack(row1), bar]))
        print(f"  그림: {out}")
    return (geo(R_init, R_fin), float(np.linalg.norm(t_fin - t_init)), score,
            top.tolist(), conc, float(vw.max()))




if __name__ == "__main__":
    main()
