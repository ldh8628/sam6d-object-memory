#!/usr/bin/env python3
"""rabbit_mode_check.py — 크기를 고친 뒤에도 '다수결로 건질 수 있나'를 본다."""
import json, os, sys
from collections import defaultdict
from pathlib import Path
import cv2, numpy as np, torch
sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP
REPO = Path(__file__).resolve().parents[1]
os.makedirs(PP.TMP, exist_ok=True)
PROBE = Path(sys.argv[1]); REP = int(sys.argv[2]) if len(sys.argv) > 2 else 12
dirs = sorted(p for p in PROBE.iterdir() if (p / "meta.json").is_file())
names = sorted({o for p in dirs for o in json.load(open(p / "meta.json"))["objects"]})
ric, cfg, net, tem = PP.load_pem("cuda:0", names)
def geo(A,B): return float(np.degrees(np.arccos(np.clip((np.trace(A.T@B)-1)/2,-1,1))))
def cluster(Rs, thr=30.0):
    lab, reps = [], []
    for R in Rs:
        for k, r in enumerate(reps):
            if geo(r, R) < thr: lab.append(k); break
        else: reps.append(R); lab.append(len(reps)-1)
    return np.array(lab), reps
out = defaultdict(list)
for p in dirs:
    m = json.load(open(p / "meta.json"))
    K = np.array(m["cam_K"], float).reshape(3, 3)
    json.dump({"cam_K": K.flatten().tolist(), "depth_scale": 1.0}, open(f"{PP.TMP}/camera.json","w"))
    lab = cv2.imread(str(p / "mask.png"), 0)
    for oi, o in enumerate(m["objects"]):
        msk = (lab == oi + 1)
        if msk.sum() < 400: continue
        Rs, sc = [], []
        for _ in range(REP):
            try:
                R, t, co, ps = PP.run_once(ric, cfg, net, tem, o, msk, K,
                                           str(p/"rgb.png"), str(p/"depth.png"), f"{PP.TMP}/camera.json")
            except Exception: break
            Rs.append(R); sc.append(ps)
        if len(Rs) < REP//2: continue
        L, reps = cluster(Rs)
        sz = np.bincount(L)
        out[o].append((sz.max()/len(Rs), len(reps),
                       float(np.median([sc[i] for i in range(len(Rs)) if L[i]==int(np.argmax(sz))]))))
print(f"\n{'객체':>22} {'조합':>5} {'주모드 점유율':>13} {'모드 수':>8} {'주모드 점수':>11}")
for o in sorted(out):
    A = np.array(out[o])
    print(f"{o:>22} {len(A):>5} {np.mean(A[:,0]):>13.2f} {np.median(A[:,1]):>8.1f} {np.median(A[:,2]):>11.3f}")
print("\n주모드 점유율이 0.5 를 넘으면 여러 번 보고 다수결을 하면 건질 수 있다는 뜻이다.")
