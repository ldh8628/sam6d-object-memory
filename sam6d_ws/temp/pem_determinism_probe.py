#!/usr/bin/env python3
"""pem_determinism_probe.py — 입력을 **비트 단위로 고정**하고 PEM forward 만 여러 번 돌린다.

pem_probe.py 는 매 호출마다 get_test_data 가 점을 새로 표본추출하므로 '입력이 달라서'
결과가 흔들릴 여지가 남는다. 여기서는 입력 텐서를 한 번 만들어 그대로 재사용한다.
그래도 결과가 달라지면 원인은 데이터가 아니라 **GPU 연산의 비결정성 + 해의 불안정성**이다.

실행 환경: `sam6d`.
"""
import argparse, hashlib, json, os, sys
from pathlib import Path

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP
from segment_anything.utils.amg import mask_to_rle_pytorch


def geo(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--reps", type=int, default=5)
    a = ap.parse_args()
    os.makedirs(PP.TMP, exist_ok=True)
    dirs = sorted(p for p in Path(a.probe).iterdir() if (p / "meta.json").is_file())
    names = sorted({o for p in dirs for o in json.load(open(p / "meta.json"))["objects"]})
    ric, cfg, net, tem = PP.load_pem("cuda:0", names)
    f = open(a.out, "w", encoding="utf-8")
    for p in dirs:
        m = json.load(open(p / "meta.json"))
        K = np.array(m["cam_K"], float).reshape(3, 3)
        json.dump({"cam_K": K.flatten().tolist(), "depth_scale": 1.0},
                  open(f"{PP.TMP}/camera.json", "w"))
        lab = cv2.imread(str(p / "mask.png"), cv2.IMREAD_GRAYSCALE)
        for oi, o in enumerate(m["objects"]):
            mask = (lab == oi + 1)
            if mask.sum() < 32:
                continue
            h, w = mask.shape
            rle = mask_to_rle_pytorch(torch.from_numpy(mask).unsqueeze(0))[0]
            ys, xs = np.where(mask)
            json.dump([{"scene_id": 0, "image_id": 0, "category_id": 1,
                        "bbox": [int(xs.min()), int(ys.min()),
                                 int(xs.max() - xs.min()), int(ys.max() - ys.min())],
                        "score": 1.0,
                        "segmentation": {"size": [h, w],
                                         "counts": [int(c) for c in rle["counts"]]}}],
                      open(f"{PP.TMP}/det.json", "w"))
            np.random.seed(7)
            try:
                inp = ric.get_test_data(str(p / "rgb.png"), str(p / "depth.png"),
                                        f"{PP.TMP}/camera.json", o, f"{PP.TMP}/det.json",
                                        0.2, cfg.test_dataset)[0]
            except Exception:
                continue
            hs = hashlib.md5(b"".join(inp[k].detach().cpu().numpy().tobytes()
                                      for k in ("pts", "rgb", "rgb_choose", "model"))).hexdigest()[:12]
            tp, tf = tem[o]
            Rs, sc = [], []
            for _ in range(a.reps):
                x = {k: (v.clone() if torch.is_tensor(v) else v) for k, v in inp.items()}
                n = x["pts"].size(0)
                x["dense_po"] = tp.repeat(n, 1, 1)
                x["dense_fo"] = tf.repeat(n, 1, 1)
                with torch.inference_mode():
                    out = net(x)
                Rs.append(out["pred_R"].detach().cpu().numpy()[0])
                sc.append(float(out["pred_pose_score"].detach().cpu().numpy()[0])
                          if "pred_pose_score" in out else None)
            dd = [geo(Rs[i], Rs[j]) for i in range(len(Rs)) for j in range(i + 1, len(Rs))]
            f.write(json.dumps({"stamp_ns": m["stamp_ns"], "object": o, "input_md5": hs,
                                "drot_med": round(float(np.median(dd)), 2),
                                "drot_max": round(float(max(dd)), 2),
                                "flip": bool(max(dd) > 150),
                                "scores": [round(s, 4) for s in sc if s is not None]}) + "\n")
        f.flush()
    f.close()
    R = [json.loads(l) for l in open(a.out, encoding="utf-8")]
    print(f"\n입력을 비트 단위로 고정하고 forward 만 {a.reps} 회 — {len(R)} 개 (프레임,객체) 조합")
    d = np.array([r["drot_med"] for r in R])
    print(f"   같은 입력인데도 Δrot 중앙 {np.median(d):.1f}° · p90 {np.percentile(d,90):.1f}° "
          f"· 최대 {max(r['drot_max'] for r in R):.1f}°")
    print(f"   150° 넘게 뒤집히는 조합 {np.mean([r['flip'] for r in R]):.2f}")
    print(f"\n{'객체':>22} {'조합':>5} {'Δrot중앙':>9} {'뒤집힘':>7}")
    for nm in sorted({r["object"] for r in R}):
        g = [r for r in R if r["object"] == nm]
        print(f"{nm:>22} {len(g):>5} {np.median([x['drot_med'] for x in g]):>9.1f} "
              f"{np.mean([x['flip'] for x in g]):>7.2f}")


if __name__ == "__main__":
    main()
