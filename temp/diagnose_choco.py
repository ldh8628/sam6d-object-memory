#!/usr/bin/env python3
"""diagnose_choco.py — 초코하임이 왜 항상 90° 돌아가 나오는지.

실제로 찍힌 점군을 평면으로 보고 **면내 두 변의 길이**를 재서 CAD(250×189 mm)와 맞춰 본다.
그리고 PEM 이 낸 포즈와 그것을 판 법선축으로 90° 돌린 포즈 중 어느 쪽이 관측을 더 잘
설명하는지 직접 비교한다.

  90° 돌린 쪽이 더 잘 맞는다      -> PEM 이 더 나쁜 답을 골랐다 (추정 실패)
  둘이 비슷하다                  -> 관측이 면내 각도를 못 정한다 (모호)
  PEM 쪽이 더 잘 맞는다          -> 관측 자체가 세로로 길다 (마스크가 딴 것을 물었다)
"""
import argparse, json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[1]
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
NAME = "choco_hazelnut_high"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--object", default=NAME)
    a = ap.parse_args()
    run = Path(a.run)
    D = [json.loads(l) for l in open(run / "detections.jsonl", encoding="utf-8")]
    byst = defaultdict(list)
    for d in D:
        byst[d["stamp_ns"]].append(d)
    tgt = [d for d in D if d["object"] == a.object]
    want = {d["stamp_ns"] for d in tgt}
    P = np.load(REPO / "assets" / "model_points" / f"{a.object}.npy").astype(np.float64)
    dim = P.max(0) - P.min(0)
    tree = cKDTree(P)

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    K = None
    ij = Path(a.bag).parent / "info.json"
    if ij.is_file():
        K = np.array(json.load(open(ij))["sam"]["camera"]["K"], float).reshape(3, 3)
    rows = []
    with AnyReader([Path(a.bag)], default_typestore=ts) as rd:
        if K is None:
            ci = [c for c in rd.connections if c.topic.endswith("color/camera_info")]
            for c, _t, raw in rd.messages(connections=ci):
                K = np.array(rd.deserialize(raw, c.msgtype).k, float).reshape(3, 3); break
        dc = [c for c in rd.connections if c.topic == DEPTH]
        for c, _t, raw in rd.messages(connections=dc):
            m = rd.deserialize(raw, c.msgtype)
            ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if ns not in want:
                continue
            mpth = run / "masks" / f"{ns}.png"
            if not mpth.is_file():
                continue
            lab = cv2.imread(str(mpth), cv2.IMREAD_GRAYSCALE)
            objs = sorted({x["object"] for x in byst[ns]})
            if a.object not in objs:
                continue
            dep = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(float)
            msk = (lab == objs.index(a.object) + 1) & (dep > 0)
            if msk.sum() < 300:
                continue
            ys, xs = np.nonzero(msk); z = dep[ys, xs]
            C = np.stack([(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], 1)
            d = [x for x in byst[ns] if x["object"] == a.object][0]
            R = np.array(d["R"]); t = np.array(d["t_mm"])
            # 관측 점군의 주축 (평면 판이므로 앞 두 개가 면내 변)
            Cc = C - C.mean(0)
            w, V = np.linalg.eigh(np.cov(Cc.T))
            ext = []
            for k in (2, 1, 0):                       # 큰 축부터
                pr = Cc @ V[:, k]
                ext.append(np.percentile(pr, 98) - np.percentile(pr, 2))
            res0 = np.median(tree.query((C - t) @ R)[0])
            th = np.radians(90)
            Rz = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1.]])
            res90 = np.median(tree.query((C - t) @ (R @ Rz))[0])
            # 화면상 세로/가로
            rows.append((ext[0], ext[1], ext[2], res0, res90,
                         abs(R[1, 0]) > abs(R[0, 0]), msk.sum()))
    A = np.array([r[:5] + (float(r[5]), float(r[6])) for r in rows])
    print(f"{a.object}: 검출 {len(A)} 건   CAD = {dim[0]:.0f} × {dim[1]:.0f} × {dim[2]:.0f} mm\n")
    print(f"관측 점군의 면내 두 변 (중앙)   {np.median(A[:,0]):.0f} × {np.median(A[:,1]):.0f} mm"
          f"   (판 두께 {np.median(A[:,2]):.0f} mm)")
    print(f"   관측 긴변/짧은변 = {np.median(A[:,0])/max(np.median(A[:,1]),1):.2f}"
          f"   ·  CAD = {dim[0]/dim[1]:.2f}")
    print(f"\nPEM 이 '긴 변을 세로로' 놓은 비율 {A[:,5].mean():.2f}")
    print(f"\n관측 점군과의 잔차 (중앙, mm)")
    print(f"   PEM 이 낸 포즈        {np.median(A[:,3]):.1f}")
    print(f"   판 법선축으로 90° 회전 {np.median(A[:,4]):.1f}")
    better = (A[:, 4] < A[:, 3]).mean()
    print(f"   90° 돌린 쪽이 더 잘 맞는 비율 {better:.2f}")
    print(f"\n마스크 넓이 중앙 {np.median(A[:,6]):.0f} px")


if __name__ == "__main__":
    main()
