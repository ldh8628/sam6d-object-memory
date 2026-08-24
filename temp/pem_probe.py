#!/usr/bin/env python3
"""pem_probe.py — "PEM 결과가 왜 흔들리나"를 입력을 통제해서 가른다.

같은 프레임·같은 마스크를 **여러 번** 다시 넣고, 마스크를 1~3 px 만 깎거나 불려서 넣는다.
그러면 흔들림이 어디서 오는지가 갈린다.

  A) 같은 입력 반복(repeat)     -> 순수 내부 무작위성 (get_test_data 의 점 표본추출)
  B) 마스크 ±1~3 px            -> 마스크 경계 민감도
  C) 시간축 이웃 프레임          -> 시점·깊이 변화까지 포함한 실제 흔들림 (분석 스크립트에서)

실행 환경: `sam6d` (ROS 2 Jazzy). CAD 없이 assets/model_points 로 돈다.
"""
import argparse, json, os, sys, time
from pathlib import Path

import cv2
import numpy as np
import torch

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
PEM_DIR = REPO / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
TMP = f"/dev/shm/pem_probe_{os.getpid()}"


def load_pem(device, names):
    cwd = os.getcwd()
    os.chdir(PEM_DIR)
    for sub in ("provider", "utils", "model", "model/pointnet2"):
        sys.path.append(str(PEM_DIR / sub))
    sys.path.insert(0, str(PEM_DIR))
    import gorilla, importlib
    import run_inference_custom as ric
    cfg = gorilla.Config.fromfile(str(PEM_DIR / "config" / "base.yaml"))
    cfg.model_name = "pose_estimation_model"
    net = importlib.import_module(cfg.model_name).Net(cfg.model).to(device).eval()
    gorilla.solver.load_checkpoint(model=net,
                                   filename=str(PEM_DIR / "checkpoints" / "sam-6d-pem-base.pth"))

    class _Stub:
        def __init__(s, p): s._pts = p
        def sample(s, n):
            if n == len(s._pts):
                return s._pts
            return s._pts[np.random.choice(len(s._pts), n, replace=(n > len(s._pts)))]

    pts, tem = {}, {}
    for nm in names:
        pts[nm] = _Stub(np.load(REPO / "assets" / "model_points" / f"{nm}.npy").astype(np.float32))
        b = torch.load(REPO / "assets" / "pem_templates" / f"{nm}.pt", map_location=device)
        tem[nm] = (b["tp"].to(device), b["tf"].to(device))
    ric.trimesh.load_mesh = lambda key, *a, **k: pts[key]
    os.chdir(cwd)
    return ric, cfg, net, tem


def run_once(ric, cfg, net, tem, name, mask, K, rgb_p, dep_p, cam_p):
    from segment_anything.utils.amg import mask_to_rle_pytorch
    h, w = mask.shape
    rle = mask_to_rle_pytorch(torch.from_numpy(mask).unsqueeze(0))[0]
    ys, xs = np.where(mask)
    det = [{"scene_id": 0, "image_id": 0, "category_id": 1,
            "bbox": [int(xs.min()), int(ys.min()), int(xs.max() - xs.min()), int(ys.max() - ys.min())],
            "score": 1.0,
            "segmentation": {"size": [h, w], "counts": [int(c) for c in rle["counts"]]}}]
    dp = f"{TMP}/det.json"
    json.dump(det, open(dp, "w"))
    inp, _i, _wp, _mp, _d = ric.get_test_data(rgb_p, dep_p, cam_p, name, dp, 0.2, cfg.test_dataset)
    n = inp["pts"].size(0)
    tp, tf = tem[name]
    with torch.inference_mode():
        inp["dense_po"] = tp.repeat(n, 1, 1)
        inp["dense_fo"] = tf.repeat(n, 1, 1)
        out = net(inp)
    R = out["pred_R"].detach().cpu().numpy()[0]
    t = out["pred_t"].detach().cpu().numpy()[0] * 1000.0
    co = float(out["score"].detach().cpu().numpy()[0])
    ps = float(out["pred_pose_score"].detach().cpu().numpy()[0]) if "pred_pose_score" in out else None
    return R, t, co, ps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--repeat", type=int, default=10)
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()

    os.makedirs(TMP, exist_ok=True)
    dirs = sorted([p for p in Path(a.probe).iterdir() if (p / "meta.json").is_file()])
    names = sorted({o for p in dirs for o in json.load(open(p / "meta.json"))["objects"]})
    print(f"[probe] {len(dirs)} 프레임 · 객체 {names}")
    ric, cfg, net, tem = load_pem(a.device, names)

    f = open(a.out, "w", encoding="utf-8")
    for pi, p in enumerate(dirs):
        meta = json.load(open(p / "meta.json"))
        K = np.array(meta["cam_K"], dtype=float).reshape(3, 3)
        json.dump({"cam_K": K.flatten().tolist(), "depth_scale": 1.0},
                  open(f"{TMP}/camera.json", "w"))
        lab = cv2.imread(str(p / "mask.png"), cv2.IMREAD_GRAYSCALE)
        rgb_p, dep_p = str(p / "rgb.png"), str(p / "depth.png")
        for oi, name in enumerate(meta["objects"]):
            base = (lab == oi + 1)
            if base.sum() < 32:
                continue
            variants = [("same", base)]
            for k in (1, 2, 3):
                ker = np.ones((2 * k + 1, 2 * k + 1), np.uint8)
                variants.append((f"erode{k}", cv2.erode(base.astype(np.uint8), ker).astype(bool)))
                variants.append((f"dilate{k}", cv2.dilate(base.astype(np.uint8), ker).astype(bool)))
            for vname, m in variants:
                if m.sum() < 32:
                    continue
                reps = a.repeat if vname == "same" else 1
                for r in range(reps):
                    try:
                        R, t, co, ps = run_once(ric, cfg, net, tem, name, m, K,
                                                rgb_p, dep_p, f"{TMP}/camera.json")
                    except Exception as e:
                        f.write(json.dumps({"stamp_ns": meta["stamp_ns"], "object": name,
                                            "variant": vname, "rep": r,
                                            "error": f"{type(e).__name__}: {e}"}) + "\n")
                        continue
                    f.write(json.dumps({
                        "stamp_ns": meta["stamp_ns"], "object": name, "variant": vname,
                        "rep": r, "mask_px": int(m.sum()),
                        "R": [[round(float(v), 6) for v in row] for row in R],
                        "t_mm": [round(float(v), 3) for v in t],
                        "coarse": round(co, 5),
                        "pose_score": round(ps, 5) if ps is not None else None}) + "\n")
        f.flush()
        print(f"  [{pi+1}/{len(dirs)}] {p.name} done", flush=True)
    f.close()
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
