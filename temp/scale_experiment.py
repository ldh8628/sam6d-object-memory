#!/usr/bin/env python3
"""scale_experiment.py — "인형 CAD 가 실물보다 작다" 가 흔들림의 원인인지 직접 시험한다.

CAD 점군과 PEM 템플릿 점을 **같은 배율로 키워** 가며 같은 프레임을 반복 처리한다.
배율이 맞아 들어가면 (1) 관측과의 잔차가 줄고 (2) 반복 간 회전 흔들림이 줄고
(3) PEM 자기 점수가 오른다. 셋 다 안 움직이면 크기는 원인이 아니다.

실행 환경: `sam6d`.
"""
import argparse, json, os, sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP
from segment_anything.utils.amg import mask_to_rle_pytorch

REPO = Path(__file__).resolve().parents[1]


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    ap.add_argument("--objects", default="Rabbit,Bear,Dinosaur,saffron,Sikhye_high")
    ap.add_argument("--scales", default="1.00,1.10,1.15,1.20,1.30")
    ap.add_argument("--repeat", type=int, default=6)
    a = ap.parse_args()
    want = set(a.objects.split(","))
    scales = [float(x) for x in a.scales.split(",")]
    os.makedirs(PP.TMP, exist_ok=True)

    dirs = sorted(p for p in Path(a.probe).iterdir() if (p / "meta.json").is_file())
    names = sorted({o for p in dirs for o in json.load(open(p / "meta.json"))["objects"]})
    ric, cfg, net, tem = PP.load_pem("cuda:0", names)
    base_pts = {n: np.load(REPO / "assets" / "model_points" / f"{n}.npy").astype(np.float32)
                for n in names}
    base_tem = {n: (tem[n][0].clone(), tem[n][1]) for n in names}

    out = defaultdict(list)
    for p in dirs:
        m = json.load(open(p / "meta.json"))
        if not (want & set(m["objects"])):
            continue
        K = np.array(m["cam_K"], float).reshape(3, 3)
        json.dump({"cam_K": K.flatten().tolist(), "depth_scale": 1.0},
                  open(f"{PP.TMP}/camera.json", "w"))
        lab = cv2.imread(str(p / "mask.png"), 0)
        dep = cv2.imread(str(p / "depth.png"), cv2.IMREAD_UNCHANGED).astype(np.float64)
        for oi, o in enumerate(m["objects"]):
            if o not in want:
                continue
            msk = (lab == oi + 1)
            if msk.sum() < 200:
                continue
            mm = msk & (dep > 0)
            ys, xs = np.nonzero(mm); z = dep[ys, xs]
            C = np.stack([(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], 1)
            h, w = msk.shape
            rle = mask_to_rle_pytorch(torch.from_numpy(msk).unsqueeze(0))[0]
            ys2, xs2 = np.where(msk)
            json.dump([{"scene_id": 0, "image_id": 0, "category_id": 1,
                        "bbox": [int(xs2.min()), int(ys2.min()),
                                 int(xs2.max() - xs2.min()), int(ys2.max() - ys2.min())],
                        "score": 1.0,
                        "segmentation": {"size": [h, w],
                                         "counts": [int(c) for c in rle["counts"]]}}],
                      open(f"{PP.TMP}/det.json", "w"))
            for s in scales:
                sp = base_pts[o] * s
                PP_pts = sp
                ric.trimesh.load_mesh = lambda key, *ar, **kw: type(
                    "S", (), {"sample": staticmethod(
                        lambda n, _p=PP_pts: _p if n == len(_p)
                        else _p[np.random.choice(len(_p), n, replace=(n > len(_p)))])})()
                tp = base_tem[o][0] * s
                tf = base_tem[o][1]
                tree = cKDTree(sp.astype(np.float64))
                Rs, sc, rs = [], [], []
                for _ in range(a.repeat):
                    try:
                        inp = ric.get_test_data(str(p / "rgb.png"), str(p / "depth.png"),
                                                f"{PP.TMP}/camera.json", o,
                                                f"{PP.TMP}/det.json", 0.2, cfg.test_dataset)[0]
                    except Exception:
                        break
                    n = inp["pts"].size(0)
                    inp["dense_po"] = tp.repeat(n, 1, 1)
                    inp["dense_fo"] = tf.repeat(n, 1, 1)
                    with torch.inference_mode():
                        r = net(inp)
                    R = r["pred_R"].detach().cpu().numpy()[0]
                    t = r["pred_t"].detach().cpu().numpy()[0] * 1000.0
                    Rs.append(R); rs.append(tree.query((C - t) @ R)[0])
                    sc.append(float(r["pred_pose_score"].detach().cpu().numpy()[0]))
                if len(Rs) < 2:
                    continue
                dd = [geo(Rs[i], Rs[j]) for i in range(len(Rs)) for j in range(i + 1, len(Rs))]
                dia = float(np.linalg.norm(sp.max(0) - sp.min(0)))
                out[(o, s)].append((float(np.median(dd)), float(max(dd) > 150),
                                    100 * float(np.median(np.concatenate(rs))) / dia,
                                    float(np.median(sc)),
                                    float(np.median(np.concatenate(rs)))))   # mm 절대값
    print(f"\n{'객체':>14} {'배율':>6} {'조합':>5} {'반복Δrot중앙':>13} {'뒤집힘':>7} "
          f"{'잔차mm':>8} {'잔차%':>7} {'pose_score':>11}")
    for o in sorted({k[0] for k in out}):
        for s in scales:
            g = out.get((o, s))
            if not g:
                continue
            A = np.array(g)
            print(f"{o:>14} {s:>6.2f} {len(A):>5} {np.median(A[:,0]):>13.1f} "
                  f"{np.mean(A[:,1]):>7.2f} {np.median(A[:,4]):>8.1f} {np.median(A[:,2]):>7.2f} "
                  f"{np.median(A[:,3]):>11.3f}")
        print()


if __name__ == "__main__":
    main()
