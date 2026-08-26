#!/usr/bin/env python3
"""real_template_pilot.py — "실사 사진 몇 장을 템플릿에 넣으면 회전이 잡히는가" 파일럿.

토끼의 서로 다른 방향에서 찍힌 **실제 관측 몇 장**을 CAD 렌더 42 장과 같은 형식으로 만들어
물체 모델에 보태고, 나머지 프레임의 회전 안정성이 바뀌는지 잰다.

⚠ 실사 템플릿에는 정답 포즈가 없다. 그 프레임에서 PEM 이 낸 포즈를 임시 기준으로 써서
관측 점을 물체 좌표계로 옮긴다(= 'GT 아님'). 그래서 이 실험이 보는 것은
"실사 외형을 섞으면 대응이 한 답으로 모이는가" 하나다.

조건: A = CAD 42 장(현행) · B = CAD 42 + 실사 k 장 · C = 실사 k 장만
평가: 템플릿으로 쓴 프레임을 제외한 나머지에서 같은 입력 8 회 반복 -> 회전 산포/뒤집힘.
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


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def build_real_template(ric, cfg, frame, obj, device, geom="observed", model_pts=None):
    """geom='observed' : 3D 좌표를 관측 depth 에서 (털·노이즈·포즈오차가 섞인다)
       geom='cad'      : 3D 좌표는 CAD 표면에서 가져오고 **외형만 실사** (도메인 갭만 분리)"""
    meta = json.load(open(frame / "meta.json"))
    K = np.array(meta["cam_K"], float).reshape(3, 3)
    oi = meta["objects"].index(obj)
    lab = cv2.imread(str(frame / "mask.png"), 0)
    dep = cv2.imread(str(frame / "depth.png"), cv2.IMREAD_UNCHANGED).astype(np.float64)
    rgb = ric.load_im(str(frame / "rgb.png")).astype(np.uint8)
    det = [x for x in meta["dets"] if x["object"] == obj][0]
    R, t = np.array(det["R"]), np.array(det["t_mm"])
    m = (lab == oi + 1) & (dep > 0)
    if m.sum() < 500:
        return None
    H, W = m.shape
    ys, xs = np.nonzero(m)
    z = dep[ys, xs]
    P = np.stack([(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], 1)
    Q = (P - t) @ R
    xyz = np.zeros((H, W, 3), np.float32)
    if geom == "observed":
        xyz[ys, xs] = (Q / 1000.0).astype(np.float32)
    else:                                   # CAD 표면을 그 포즈로 투영해 화소마다 3D 를 준다
        q = (R @ model_pts.T).T + t
        q = q[q[:, 2] > 1]
        u = np.rint(K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(int)
        v = np.rint(K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(int)
        ok = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        u, v, q = u[ok], v[ok], q[ok]
        zbuf = np.full((H, W), np.inf)
        for i in np.argsort(-q[:, 2]):      # 먼 것부터 덮어써서 가까운 것만 남긴다
            if q[i, 2] < zbuf[v[i], u[i]]:
                zbuf[v[i], u[i]] = q[i, 2]
                xyz[v[i], u[i]] = (model_pts[np.arange(len(model_pts))[ok][i]] / 1000.0)
        m = m & (np.abs(xyz).sum(2) > 0)
        if m.sum() < 500:
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
    pts = xyz[y1:y2, x1:x2, :].reshape(-1, 3)[ch, :]
    rch = ric.get_resize_rgb_choose(ch, [y1, y2, x1, x2], cfg.test_dataset.img_size)
    return (torch.FloatTensor(rc).unsqueeze(0).to(device),
            torch.FloatTensor(pts).unsqueeze(0).to(device),
            torch.IntTensor(rch).long().unsqueeze(0).to(device))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    ap.add_argument("--object", default="Rabbit")
    ap.add_argument("--k", type=int, default=3)
    ap.add_argument("--reps", type=int, default=8)
    a = ap.parse_args()
    os.makedirs(PP.TMP, exist_ok=True)
    o = a.object
    dirs = [p for p in sorted(Path(a.probe).iterdir())
            if (p / "meta.json").is_file() and o in json.load(open(p / "meta.json"))["objects"]]
    names = sorted({x for p in dirs for x in json.load(open(p / "meta.json"))["objects"]})
    ric, cfg, net, _t = PP.load_pem("cuda:0", names)
    dev = "cuda:0"

    cand = []
    for p in dirs:
        d = [x for x in json.load(open(p / "meta.json"))["dets"] if x["object"] == o][0]
        cand.append((d["score"], (d.get("pem") or {}).get("mask_px", 0) or 0, p, np.array(d["R"])))
    cand.sort(key=lambda c: -(c[0] * min(c[1], 4000) / 4000))
    picked = []
    for sc, px, p, R in cand:
        if all(geo(R, r) > 45 for _, _, _, r in picked):
            picked.append((sc, px, p, R))
        if len(picked) >= a.k:
            break
    print("[실사 템플릿] " + ", ".join(f"{p.name[-8:]}(score {sc:.2f}, {px}px)"
                                       for sc, px, p, _ in picked))
    used = {p for _, _, p, _ in picked}
    ev = [p for p in dirs if p not in used]
    print(f"[평가 프레임] {len(ev)} 장")

    tdir = REPO / "template" / TEM_DIR.get(o, o) / "templates"
    cad = ric.get_templates(str(tdir), cfg.test_dataset)
    MPTS = np.load(REPO / "assets" / "model_points" / f"{o}.npy").astype(np.float64)
    real = [r for r in (build_real_template(ric, cfg, p, o, dev) for _, _, p, _ in picked)
            if r is not None]
    realc = [r for r in (build_real_template(ric, cfg, p, o, dev, "cad", MPTS)
                         for _, _, p, _ in picked) if r is not None]

    def feats(rgbs, ptss, chs):
        with torch.inference_mode():
            return net.feature_extraction.get_obj_feats(rgbs, ptss, chs)

    conds = {
        "A. CAD 42": (cad[0], cad[1], cad[2]),
        f"B. CAD 42 + real {len(real)}": (cad[0] + [r[0] for r in real],
                                          cad[1] + [r[1] for r in real],
                                          cad[2] + [r[2] for r in real]),
        f"C. real {len(real)} only": ([r[0] for r in real], [r[1] for r in real],
                                      [r[2] for r in real]),
        f"D. CAD 42 + real-appearance {len(realc)} (CAD 기하)":
            (cad[0] + [r[0] for r in realc], cad[1] + [r[1] for r in realc],
             cad[2] + [r[2] for r in realc]),
    }
    packs = {k: feats(*v) for k, v in conds.items()}
    res = defaultdict(list)
    for p in ev:
        meta = json.load(open(p / "meta.json"))
        K = np.array(meta["cam_K"], float).reshape(3, 3)
        json.dump({"cam_K": K.flatten().tolist(), "depth_scale": 1.0},
                  open(f"{PP.TMP}/camera.json", "w"))
        lab = cv2.imread(str(p / "mask.png"), 0)
        msk = (lab == meta["objects"].index(o) + 1)
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
        for cname, (tp, tf) in packs.items():
            Rs, sc = [], []
            for _ in range(a.reps):
                try:
                    inp = ric.get_test_data(str(p / "rgb.png"), str(p / "depth.png"),
                                            f"{PP.TMP}/camera.json", o, f"{PP.TMP}/det.json",
                                            0.2, cfg.test_dataset)[0]
                except Exception:
                    break
                n = inp["pts"].size(0)
                inp["dense_po"] = tp.repeat(n, 1, 1)
                inp["dense_fo"] = tf.repeat(n, 1, 1)
                with torch.inference_mode():
                    out = net(inp)
                Rs.append(out["pred_R"].detach().cpu().numpy()[0])
                sc.append(float(out["pred_pose_score"].detach().cpu().numpy()[0]))
            if len(Rs) < 3:
                continue
            dd = [geo(Rs[i], Rs[j]) for i in range(len(Rs)) for j in range(i + 1, len(Rs))]
            res[cname].append((float(np.median(dd)), float(max(dd) > 150), float(np.median(sc))))
    print(f"\n{'조건':>26} {'프레임':>6} {'반복 Δrot 중앙':>14} {'뒤집힘 비율':>11} {'pose_score':>11}")
    for k in conds:
        if k not in res:
            continue
        A = np.array(res[k])
        print(f"{k:>26} {len(A):>6} {np.median(A[:,0]):>14.1f} {np.mean(A[:,1]):>11.2f} "
              f"{np.median(A[:,2]):>11.3f}")


if __name__ == "__main__":
    main()
