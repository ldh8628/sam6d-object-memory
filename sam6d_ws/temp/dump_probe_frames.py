#!/usr/bin/env python3
"""dump_probe_frames.py — 실시간 실행에서 처리된 프레임 중 일부를 원본 그대로 꺼낸다.

PEM 흔들림의 원인을 '같은 입력으로 다시 돌려서' 가르기 위한 재료를 만든다.
꺼내는 것: rgb.png · depth.png · mask.png(라벨) · meta.json(그때 나온 R,t,점수).

실행 환경: `sam_yolo` (rosbags).
"""
import argparse, json, os
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--n", type=int, default=40, help="뽑을 프레임 수")
    a = ap.parse_args()

    run, out = Path(a.run), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dets = [json.loads(l) for l in open(run / "detections.jsonl", encoding="utf-8")]
    by = defaultdict(list)
    for d in dets:
        by[d["stamp_ns"]].append(d)
    have = sorted(s for s in by if (run / "masks" / f"{s}.png").is_file())
    # 시간축에 고르게 n 장
    idx = np.linspace(0, len(have) - 1, min(a.n, len(have))).astype(int)
    pick = sorted({have[i] for i in idx})
    print(f"[pick] {len(pick)} / {len(have)} 프레임")

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    want = set(pick)
    got_c, got_d = {}, {}
    with AnyReader([Path(a.bag)], default_typestore=ts) as rd:
        con = [c for c in rd.connections if c.topic in (COLOR, DEPTH)]
        for c, _t, raw in rd.messages(connections=con):
            m = rd.deserialize(raw, c.msgtype)
            ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            if ns not in want:
                continue
            if c.topic == COLOR:
                b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                got_c[ns] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
            else:
                got_d[ns] = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()
    ij = Path(a.bag).parent / "info.json"
    if ij.is_file():
        K = json.load(open(ij))["sam"]["camera"]["K"]
    else:                                   # info.json 이 없으면 bag 의 camera_info
        with AnyReader([Path(a.bag)], default_typestore=ts) as rd:
            cc = [c for c in rd.connections if c.topic.endswith("color/camera_info")]
            for c, _t, raw in rd.messages(connections=cc):
                K = list(rd.deserialize(raw, c.msgtype).k); break
    n_ok = 0
    for ns in pick:
        if ns not in got_c or ns not in got_d:
            continue
        d = out / str(ns)
        d.mkdir(exist_ok=True)
        cv2.imwrite(str(d / "rgb.png"), got_c[ns])
        cv2.imwrite(str(d / "depth.png"), got_d[ns])
        lab = cv2.imread(str(run / "masks" / f"{ns}.png"), cv2.IMREAD_GRAYSCALE)
        cv2.imwrite(str(d / "mask.png"), lab)
        rows = sorted(by[ns], key=lambda r: r["object"])
        json.dump({"stamp_ns": ns, "cam_K": K,
                   "objects": [r["object"] for r in rows], "dets": rows},
                  open(d / "meta.json", "w"), ensure_ascii=False, indent=1)
        n_ok += 1
    print(f"[out] {n_ok} 프레임 -> {out}")


if __name__ == "__main__":
    main()
