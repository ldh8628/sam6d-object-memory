#!/usr/bin/env python3
"""render_test_video.py — 실시간 실행 결과를 좌우 비교 mp4 로 만든다.

    왼쪽    bag 이 실제로 흘려보낸 원본 영상 (모든 프레임, 실시간 순서)
    가운데  **PEM 이 실제로 처리한 그 프레임** + CAD 점군을 추정 포즈로 투영
    오른쪽  같은 프레임 + **3D 경계상자만** (축 색: X 빨강 / Y 초록 / Z 파랑)
           둘 다 다음 처리가 끝날 때까지 그대로 유지된다 (= 실시간에서 사람이 보는 화면).

오른쪽이 왼쪽보다 늦게, 띄엄띄엄 갱신되는 것이 곧 처리 지연이다.
아래 띠는 같은 객체의 연속 추정 사이 회전 변화량(Δrot) 이력이다 — 흔들림의 직접 지표.

노드가 `output.diagnostics: true` 로 남긴 것을 그대로 쓴다:
  frames.jsonl (t_done_ns = bag 시각 기준 처리 완료 시각) · detections.jsonl · masks/*.png

인코딩은 ffmpeg libx264(yuv420p) 로 한다. OpenCV 의 mp4v 는 브라우저·기본 플레이어에서
재생되지 않는 경우가 많다(그래서 이전 결과가 안 열렸다).

bag 을 다시 읽어 왼쪽 영상을 만들기 때문에 **bag 재생판에서만** 쓸 수 있다
(실제 카메라 운용에서는 영상이 저장돼 있지 않으므로 만들 수 없다).

실행 환경: `rosbags` 파이썬 패키지가 있는 env (예: `sam_yolo`).
"""
import argparse, json, os, subprocess, sys
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]
COLOR = "/camera/camera/color/image_raw"
PALETTE = [(80, 220, 90), (60, 165, 255), (240, 170, 60), (200, 100, 240),
           (80, 225, 225), (150, 150, 255), (120, 200, 160), (255, 205, 120),
           (175, 120, 255), (110, 240, 185)]


def geodesic_deg(A, B):
    c = np.clip((np.trace(np.asarray(A).T @ np.asarray(B)) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True, help="노드 출력 폴더 (frames.jsonl 이 있는 곳)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--step", type=int, default=2, help="원본 N 프레임마다 한 장")
    ap.add_argument("--start", type=float, default=0.0, help="bag 시작 후 [s]")
    ap.add_argument("--dur", type=float, default=0.0, help="길이 [s] (0=끝까지)")
    a = ap.parse_args()

    run = Path(a.run)
    frames = [json.loads(l) for l in open(run / "frames.jsonl", encoding="utf-8")]
    dets = [json.loads(l) for l in open(run / "detections.jsonl", encoding="utf-8")]
    by_stamp = defaultdict(list)
    for d in dets:
        by_stamp[d["stamp_ns"]].append(d)
    frames = [f for f in frames if f.get("t_done_ns")]
    frames.sort(key=lambda f: f["t_done_ns"])
    print(f"[run] 처리 프레임 {len(frames)} · 검출 {len(dets)}")

    # 객체별 Δrot (연속 추정 사이)
    prevR, hist = {}, defaultdict(list)
    for f in frames:
        for d in sorted(by_stamp.get(f["stamp_ns"], []), key=lambda d: d["object"]):
            nm, R = d["object"], np.array(d["R"])
            d["drot"] = geodesic_deg(prevR[nm], R) if nm in prevR else None
            prevR[nm] = R
            if d["drot"] is not None:
                hist[nm].append((f["t_done_ns"] * 1e-9, d["drot"]))

    # CAD 점 (mm)
    MP, cidx, BOX = {}, {}, {}
    pdir = REPO / "assets" / "model_points"
    for i, p in enumerate(sorted(pdir.glob("*.npy"))):
        full = np.load(p).astype(np.float32)
        MP[p.stem] = full[::8]
        cidx[p.stem] = i
        lo, hi = full.min(0), full.max(0)          # 물체 좌표계 축정렬 상자
        BOX[p.stem] = np.array([[x, y, z] for x in (lo[0], hi[0])
                                for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], np.float32)

    K = None
    for cand in (Path(a.bag).parent / "info.json", Path(a.bag).parent.parent / "info.json"):
        if cand.is_file():
            K = np.array(json.load(open(cand))["sam"]["camera"]["K"], dtype=float).reshape(3, 3)
            break
    if K is None:                      # info.json 이 없으면 bag 의 camera_info 를 쓴다
        from rosbags.highlevel import AnyReader as _AR
        from rosbags.typesys import Stores as _S, get_typestore as _gt
        with _AR([Path(a.bag)], default_typestore=_gt(_S.ROS2_HUMBLE)) as _rd:
            _c = [c for c in _rd.connections if c.topic.endswith("color/camera_info")]
            if _c:
                for c, _t, raw in _rd.messages(connections=_c):
                    K = np.array(_rd.deserialize(raw, c.msgtype).k, float).reshape(3, 3)
                    break
        print(f"[K] bag 의 camera_info 사용 fx={K[0,0]:.1f}" if K is not None else "[K] 없음")

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)

    t0_bag = frames[0]["stamp_ns"] * 1e-9
    lo = t0_bag + a.start
    hi = lo + a.dur if a.dur else frames[-1]["t_done_ns"] * 1e-9 + 2.0

    # bag 을 한 번만 읽는다. 처리된 프레임을 만나면 그 자리에서 오른쪽 화면을 그려
    # JPEG 로 접어 둔다(원본 그대로 들고 있으면 수백 MB 가 된다). 결과가 화면에 뜨는
    # 시각 t_done 은 항상 촬영 시각보다 뒤이므로, 왼쪽 영상을 앞으로 훑는 것만으로
    # "그 순간 가장 최근 결과"를 항상 손에 쥐고 있을 수 있다.
    want = {f["stamp_ns"]: f for f in frames}
    order = sorted(frames, key=lambda f: f["t_done_ns"])

    from rosbags.highlevel import AnyReader
    with AnyReader([Path(a.bag)], default_typestore=ts) as rd:
        con = [c for c in rd.connections if c.topic == COLOR]
        first = next(rd.messages(connections=con))[2]
        m0 = rd.deserialize(first, con[0].msgtype)
        h, w = m0.height, m0.width
    if K is None:
        K = np.array([[387.0, 0, w / 2], [0, 386.5, h / 2], [0, 0, 1]])
    BAR, STRIP = 34, 130
    H, W = BAR + h + STRIP, w * 3

    # 상자 12 모서리: 각 모서리가 어느 축 방향인지(0=X,1=Y,2=Z) 같이 든다.
    # 코너 순서는 (x,y,z) 를 lo/hi 로 도는 순서라 비트로 축을 알 수 있다.
    EDGES = [(i, i ^ b, {4: 0, 2: 1, 1: 2}[b])
             for i in range(8) for b in (4, 2, 1) if not (i & b)]
    AXCOL = [(70, 70, 255), (80, 220, 80), (255, 170, 60)]      # X 빨강 Y 초록 Z 파랑(BGR)

    def project(q, K, w, h):
        q = q[q[:, 2] > 1e-3]
        if not len(q):
            return None, None
        u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2])
        v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2])
        return u, v

    def make_pane(img, f):
        """(점군 오버레이, 3D 상자 오버레이) 두 장을 만든다."""
        ns = f["stamp_ns"]
        mp = run / "masks" / f"{ns}.png"
        lab = cv2.imread(str(mp), cv2.IMREAD_GRAYSCALE) if mp.is_file() else None
        box_img = img.copy()
        for j, d in enumerate(sorted(by_stamp.get(ns, []), key=lambda d: d["object"])):
            nm = d["object"]
            col = PALETTE[cidx.get(nm, j) % len(PALETTE)]
            R = np.array(d["R"]); t = np.array(d["t_mm"])
            P = MP.get(nm)
            if P is not None:
                u, v = project((R @ P.T).T + t, K, w, h)
                if u is not None:
                    ui, vi = u.astype(np.int32), v.astype(np.int32)
                    ok = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
                    img[vi[ok], ui[ok]] = col
            if nm in BOX:                       # --- 3D 경계상자 ---
                q = (R @ BOX[nm].T).T + t
                if (q[:, 2] > 1e-3).all():
                    u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(np.int32)
                    v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(np.int32)
                    for i0, i1, ax in EDGES:
                        cv2.line(box_img, (u[i0], v[i0]), (u[i1], v[i1]), AXCOL[ax], 2,
                                 cv2.LINE_AA)
            if lab is not None:                  # ISM 이 PEM 에 준 마스크 = 흰 윤곽
                cs, _ = cv2.findContours((lab == j + 1).astype(np.uint8),
                                         cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, cs, -1, (255, 255, 255), 1)
                cv2.drawContours(box_img, cs, -1, (210, 210, 210), 1)
            x1, y1, x2, y2 = d["bbox"]
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 1)
            lb = nm.replace("_high", "") + f" {d['score']:.2f}"
            if d.get("drot") is not None:
                lb += f"  drot {d['drot']:.0f}"
            for im in (img, box_img):
                cv2.putText(im, lb, (x1 + 2, max(11, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.40, col, 1, cv2.LINE_AA)
        return (cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1],
                cv2.imencode(".jpg", box_img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1])

    def bar(text, sub=""):
        b = np.full((BAR, w, 3), 26, np.uint8)
        cv2.putText(b, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (255, 255, 255), 1, cv2.LINE_AA)
        if sub:
            cv2.putText(b, sub, (max(4, w - 8 - 9 * len(sub)), 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (170, 210, 255), 1, cv2.LINE_AA)
        return b

    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
           "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
           "-r", f"{a.fps}", "-i", "-", "-c:v", "libx264", "-preset", "medium",
           "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out]
    enc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    T0, T1 = lo, hi
    blank = np.full((h, w, 3), 18, np.uint8)
    cv2.putText(blank, "waiting for first SAM-6D result ...", (30, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1, cv2.LINE_AA)
    panes, pi, cur, n_out, i = {}, 0, None, 0, -1
    with AnyReader([Path(a.bag)], default_typestore=ts) as rd:
        con = [c for c in rd.connections if c.topic == COLOR]
        for c, _t, raw in rd.messages(connections=con):
            m = rd.deserialize(raw, c.msgtype)
            ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            st = ns * 1e-9
            f = want.get(ns)
            if f is None and not (lo <= st <= hi):
                continue
            b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            img = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
            if f is not None:
                panes[ns] = make_pane(img.copy(), f)
            i += 1
            if not (lo <= st <= hi) or i % a.step:
                continue
            while pi < len(order) and order[pi]["t_done_ns"] * 1e-9 <= st:
                jp = panes.pop(order[pi]["stamp_ns"], None)
                if jp is not None:
                    cur = (order[pi]["t_done_ns"] * 1e-9, jp, order[pi])
                pi += 1
            if cur is None:
                mid = rgt = blank
                txt, sub = "SAM-6D  (no result yet)", ""
            else:
                mid = cv2.imdecode(cur[1][0], cv2.IMREAD_COLOR)
                rgt = cv2.imdecode(cur[1][1], cv2.IMREAD_COLOR)
                ff = cur[2]
                txt = f"SAM-6D  frame t={ff['stamp_ns']*1e-9 - t0_bag:6.1f}s  obj {ff['n_accept']}"
                sub = f"lat {ff['ms']['total']/1000:.2f}s  held {st - cur[0]:4.1f}s"
            canvas = np.empty((H, W, 3), np.uint8)
            canvas[:BAR, :w] = bar(f"LIVE  bag t={st - t0_bag:6.1f}s", "30 Hz")
            canvas[:BAR, w:2*w] = bar(txt + "   [CAD points]", sub)
            canvas[:BAR, 2*w:] = bar("3D bbox   X=red Y=green Z=blue", "")
            canvas[BAR:BAR + h, :w] = img
            canvas[BAR:BAR + h, w:2*w] = mid
            canvas[BAR:BAR + h, 2*w:] = rgt
            strip = np.full((STRIP, W, 3), 20, np.uint8)
            cv2.putText(strip, "PEM rotation change between consecutive estimates of the same object (deg)",
                        (8, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (205, 205, 205), 1, cv2.LINE_AA)
            for gv in (0, 60, 120, 180):
                gy = int(STRIP - 10 - gv / 180.0 * (STRIP - 34))
                cv2.line(strip, (58, gy), (W - 8, gy), (52, 52, 52), 1)
                cv2.putText(strip, f"{gv:3d}", (20, gy + 4), cv2.FONT_HERSHEY_SIMPLEX,
                            0.34, (115, 115, 115), 1, cv2.LINE_AA)
            for nm, hs in hist.items():
                col = PALETTE[cidx.get(nm, 0) % len(PALETTE)]
                for t, dv in hs:
                    if t > st or t < T0:
                        continue
                    x = int(58 + (t - T0) / max(T1 - T0, 1e-6) * (W - 66))
                    y = int(STRIP - 10 - min(dv, 180) / 180.0 * (STRIP - 34))
                    cv2.circle(strip, (x, y), 2, col, -1)
            x = int(58 + (st - T0) / max(T1 - T0, 1e-6) * (W - 66))
            cv2.line(strip, (x, 20), (x, STRIP - 8), (95, 95, 95), 1)
            canvas[BAR + h:] = strip
            enc.stdin.write(canvas.tobytes())
            n_out += 1
    enc.stdin.close()
    enc.wait()
    print(f"wrote {a.out}  ({n_out} frames, {n_out/a.fps:.0f}s @ {a.fps}fps)")


if __name__ == "__main__":
    main()
