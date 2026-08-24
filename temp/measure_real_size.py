#!/usr/bin/env python3
"""measure_real_size.py — 실제 물체의 치수를 depth 로 재서 CAD 치수와 비교한다.

마스크 없이도 되도록, 검출 박스 안에서 '기록된 거리 ±120 mm' 인 화소의 최대 연결성분을
물체로 본다. 가려지거나 잘리면 작게 나오므로 **비율의 상위 백분위수**를 쓴다(작아지는 쪽으로만
편향되기 때문). 크기가 맞는 강체가 1.00 근처로 나오는지로 방법 자체를 검증한다.
"""
import argparse, json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]
COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--K", default="")
    a = ap.parse_args()
    D = [json.loads(l) for l in open(Path(a.run) / "detections.jsonl", encoding="utf-8")]
    want = {d["stamp_ns"] for d in D}
    by = defaultdict(list)
    for d in D:
        by[d["stamp_ns"]].append(d)

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    K = None
    for c in (Path(a.bag).parent / "info.json",):
        if c.is_file():
            K = np.array(json.load(open(c))["sam"]["camera"]["K"], float).reshape(3, 3)
    res = defaultdict(list)
    MP = {f.stem: np.load(f).astype(np.float64)
          for f in (REPO / "assets" / "model_points").glob("*.npy")}
    with AnyReader([Path(a.bag)], default_typestore=ts) as rd:
        if K is None:
            ci = [c for c in rd.connections if c.topic.endswith("color/camera_info")]
            for c, _t, raw in rd.messages(connections=ci):
                K = np.array(rd.deserialize(raw, c.msgtype).k, float).reshape(3, 3)
                break
        dcon = [c for c in rd.connections if c.topic == DEPTH]
        for c, _t, raw in rd.messages(connections=dcon):
            m = rd.deserialize(raw, c.msgtype)
            ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if ns not in want:
                continue
            dep = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(float)
            for d in by[ns]:
                z0 = (d.get("pem") or {}).get("z_med_mm")
                if not z0:
                    continue
                x1, y1, x2, y2 = d["bbox"]
                sub = dep[y1:y2, x1:x2]
                if sub.size < 200:
                    continue
                msk = ((np.abs(sub - z0) < 120) & (sub > 0)).astype(np.uint8)
                n, lab, st, _ = cv2.connectedComponentsWithStats(msk, 8)
                if n < 2:
                    continue
                k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
                if st[k, cv2.CC_STAT_AREA] < 200:
                    continue
                ys, xs = np.nonzero(lab == k)
                zz = sub[ys, xs]
                ys = ys + y1; xs = xs + x1
                X = (xs - K[0, 2]) * zz / K[0, 0]
                Y = (ys - K[1, 2]) * zz / K[1, 1]
                w = np.percentile(X, 99) - np.percentile(X, 1)
                h = np.percentile(Y, 99) - np.percentile(Y, 1)
                # 화면에 잘리지 않은 것만
                if x1 <= 1 or y1 <= 1 or x2 >= dep.shape[1] - 2 or y2 >= dep.shape[0] - 2:
                    continue
                res[d["object"]].append((w, h, float(np.median(zz))))
    print(f"{'객체':>22} {'n':>4} {'CAD (mm)':>22} {'실측 가로×세로 (p90)':>22} "
          f"{'실측대각/CAD대각':>16}")
    for nm in sorted(res):
        A = np.array(res[nm])
        if len(A) < 5:
            continue
        dim = MP[nm].max(0) - MP[nm].min(0)
        # 물체를 어느 방향에서 보든 화면상 최대 가능 크기 = 가장 큰 두 변의 대각
        d2 = np.sort(dim)[::-1]
        cad_diag = float(np.hypot(d2[0], d2[1]))
        obs = np.hypot(A[:, 0], A[:, 1])
        p90 = float(np.percentile(obs, 90))
        print(f"{nm:>22} {len(A):>4} {dim[0]:6.0f}×{dim[1]:5.0f}×{dim[2]:5.0f}{'':>4} "
              f"{np.percentile(A[:,0],90):9.0f}×{np.percentile(A[:,1],90):<8.0f} "
              f"{p90/cad_diag:>16.2f}")
    print("\n(실측은 가려지면 작아지므로 p90 을 쓴다. 크기가 맞는 강체가 1.00 근처면 방법이 타당하다.)")


if __name__ == "__main__":
    main()
