#!/usr/bin/env python3
"""sam6d_viewer.py — 시각화 프로세스. **ROS 를 쓰지 않는다.**

수신 프로세스가 쓰는 프레임 공유메모리와 추론 프로세스가 쓰는 결과 공유메모리를
**그대로 읽어서** 그린다. 그래서

  · ROS 토픽을 하나도 늘리지 않는다 (30 Hz 영상을 또 구독하지 않는다)
  · 수신/추론과 별개 프로세스라 GIL 을 다투지 않는다 (NOTES 19~22절)
  · mp4 렌더와 **같은 화면**을 실시간으로 본다

화면은 세 칸이다.
  왼쪽   지금 들어오는 영상 (수신되는 대로)
  가운데 마지막으로 처리된 프레임 + CAD 점군 투영
  오른쪽 같은 프레임 + 3D 경계상자 (축 색 X 빨강 / Y 초록 / Z 파랑)
아래 띠는 같은 객체의 연속 추정 사이 회전 변화량(Δrot) 이력이다.

    conda activate sam6d          # (또는 numpy·opencv 가 있는 아무 env)
    python realtime/sam6d_viewer.py                 # 창으로 보기
    python realtime/sam6d_viewer.py --save live.mp4 # 보면서 녹화
    python realtime/sam6d_viewer.py --headless --save live.mp4   # 화면 없이 녹화만
"""
from __future__ import annotations

import argparse
import collections
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shm_channel import FrameReader, JsonReader          # noqa: E402

REPO = Path(__file__).resolve().parents[1]
PALETTE = [(80, 220, 90), (60, 165, 255), (240, 170, 60), (200, 100, 240),
           (80, 225, 225), (150, 150, 255), (120, 200, 160), (255, 205, 120),
           (175, 120, 255), (110, 240, 185)]
AXCOL = [(70, 70, 255), (80, 220, 80), (255, 170, 60)]      # X 빨강 Y 초록 Z 파랑
EDGES = [(i, i ^ b, {4: 0, 2: 1, 1: 2}[b]) for i in range(8) for b in (4, 2, 1) if not (i & b)]


def geodesic_deg(A, B):
    c = np.clip((np.trace(np.asarray(A).T @ np.asarray(B)) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


def load_models():
    MP, BOX, idx = {}, {}, {}
    d = REPO / "assets" / "model_points"
    for i, p in enumerate(sorted(d.glob("*.npy"))):
        full = np.load(p).astype(np.float32)
        MP[p.stem] = full[::8]
        idx[p.stem] = i
        lo, hi = full.min(0), full.max(0)
        BOX[p.stem] = np.array([[x, y, z] for x in (lo[0], hi[0])
                                for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], np.float32)
    return MP, BOX, idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default="", help="보이는 그대로 mp4 로 녹화")
    ap.add_argument("--fps", type=float, default=15.0, help="녹화 fps")
    ap.add_argument("--headless", action="store_true", help="창 없이 녹화만")
    ap.add_argument("--scale", type=float, default=1.0, help="창 배율")
    a = ap.parse_args()

    MP, BOX, cidx = load_models()
    print(f"[view] CAD 점군 {len(MP)} 객체")
    fr = jr = None
    for _ in range(600):                       # 수신·추론이 먼저 뜰 때까지 기다린다
        try:
            fr = fr or FrameReader()
            jr = jr or JsonReader()
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if fr is None or jr is None:
        raise SystemExit("[view] 공유메모리를 찾지 못했다. 수신/추론 프로세스를 먼저 띄울 것")

    cache = collections.OrderedDict()          # stamp -> bgr (처리된 프레임을 찾기 위한 최근 창)
    hist = collections.defaultdict(lambda: collections.deque(maxlen=300))
    prevR, last, live, K = {}, None, None, None
    enc, n_written, t0 = None, 0, None
    STRIP, BAR = 130, 34
    fps_in = collections.deque(maxlen=60)

    def draw(res, base):
        """가운데(점군)·오른쪽(3D 상자) 두 칸을 만든다."""
        mid, rgt = base.copy(), base.copy()
        h, w = base.shape[:2]
        for j, d in enumerate(sorted(res["dets"], key=lambda x: x["object"])):
            nm = d["object"]
            col = PALETTE[cidx.get(nm, j) % len(PALETTE)]
            R, t = np.array(d["R"]), np.array(d["t_mm"])
            P = MP.get(nm)
            if P is not None:
                q = (R @ P.T).T + t
                q = q[q[:, 2] > 1]
                if len(q):
                    u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(np.int32)
                    v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(np.int32)
                    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                    mid[v[ok], u[ok]] = col
            if nm in BOX:
                q = (R @ BOX[nm].T).T + t
                if (q[:, 2] > 1).all():
                    u = (K[0, 0] * q[:, 0] / q[:, 2] + K[0, 2]).astype(np.int32)
                    v = (K[1, 1] * q[:, 1] / q[:, 2] + K[1, 2]).astype(np.int32)
                    for i0, i1, ax in EDGES:
                        cv2.line(rgt, (u[i0], v[i0]), (u[i1], v[i1]), AXCOL[ax], 2, cv2.LINE_AA)
            lb = nm.replace("_high", "") + f" {d['score']:.2f}"
            if d.get("drot") is not None:
                lb += f"  drot {d['drot']:.0f}"
            uu = int(np.clip(K[0, 0] * t[0] / max(t[2], 1) + K[0, 2], 4, w - 120))
            vv = int(np.clip(K[1, 1] * t[1] / max(t[2], 1) + K[1, 2], 14, h - 6))
            for im in (mid, rgt):
                cv2.putText(im, lb, (uu, vv), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
        return mid, rgt

    print("[view] 대기 중 ... (창에서 q 또는 Ctrl-C 로 종료)")
    try:
        while True:
            got = fr.read_new()
            if got is not None:
                rgb, dep, K, stamp, recv = got
                live = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cache[stamp] = live
                while len(cache) > 150:
                    cache.popitem(last=False)
                fps_in.append(time.time())
            r = jr.read_new()
            if r is not None:
                for d in sorted(r["dets"], key=lambda x: x["object"]):
                    nm = d["object"]; R = np.array(d["R"])
                    d["drot"] = geodesic_deg(prevR[nm], R) if nm in prevR else None
                    prevR[nm] = R
                    if d["drot"] is not None:
                        hist[nm].append((time.time(), d["drot"]))
                base = cache.get(r["stamp_ns"])
                if base is None and cache:
                    # 그 프레임을 못 받았으면(뷰어가 잠깐 밀렸으면) 가장 가까운 것으로 대신한다
                    k = min(cache, key=lambda x: abs(x - r["stamp_ns"]))
                    if abs(k - r["stamp_ns"]) < 3e8:      # 0.3 초 이내면 인정
                        base = cache[k]
                if base is not None and K is not None:
                    last = (r, *draw(r, base), time.time())
            if live is None:
                time.sleep(0.005)
                continue

            h, w = live.shape[:2]
            H, W = BAR + h + STRIP, w * 3
            canvas = np.empty((H, W, 3), np.uint8)
            hz = (len(fps_in) - 1) / max(fps_in[-1] - fps_in[0], 1e-6) if len(fps_in) > 2 else 0

            def bar(txt, sub=""):
                b = np.full((BAR, w, 3), 26, np.uint8)
                cv2.putText(b, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                            (255, 255, 255), 1, cv2.LINE_AA)
                if sub:
                    cv2.putText(b, sub, (max(4, w - 8 - 9 * len(sub)), 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (170, 210, 255), 1, cv2.LINE_AA)
                return b

            if last is None:
                mid = rgt = np.full_like(live, 18)
                cv2.putText(mid, "waiting for SAM-6D result ...", (30, h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1, cv2.LINE_AA)
                rgt = mid.copy()
                txt, sub = "SAM-6D  (no result yet)", ""
            else:
                r, mid, rgt, tshow = last
                ms = r.get("ms", {})
                txt = f"SAM-6D  obj {r['n']}  proc {r.get('n_proc', 0)}"
                sub = (f"{ms.get('total', 0)}ms [y{ms.get('yolo',0)}/i{ms.get('ism',0)}/"
                       f"p{ms.get('pem',0)}]  held {time.time()-tshow:4.1f}s")
            canvas[:BAR, :w] = bar("LIVE", f"{hz:4.1f} Hz in")
            canvas[:BAR, w:2*w] = bar(txt + "   [CAD points]", sub)
            canvas[:BAR, 2*w:] = bar("3D bbox   X=red Y=green Z=blue", "")
            canvas[BAR:BAR + h, :w] = live
            canvas[BAR:BAR + h, w:2*w] = mid
            canvas[BAR:BAR + h, 2*w:] = rgt
            strip = np.full((STRIP, W, 3), 20, np.uint8)
            cv2.putText(strip, "PEM rotation change between consecutive estimates (deg)",
                        (8, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (205, 205, 205), 1, cv2.LINE_AA)
            now = time.time()
            for gv in (0, 60, 120, 180):
                gy = int(STRIP - 10 - gv / 180.0 * (STRIP - 34))
                cv2.line(strip, (58, gy), (W - 8, gy), (52, 52, 52), 1)
                cv2.putText(strip, f"{gv:3d}", (20, gy + 4), cv2.FONT_HERSHEY_SIMPLEX,
                            0.34, (115, 115, 115), 1, cv2.LINE_AA)
            for nm, hs in hist.items():                 # 최근 60 초
                col = PALETTE[cidx.get(nm, 0) % len(PALETTE)]
                for t, dv in hs:
                    if now - t > 60:
                        continue
                    x = int(58 + (1 - (now - t) / 60.0) * (W - 66))
                    y = int(STRIP - 10 - min(dv, 180) / 180.0 * (STRIP - 34))
                    cv2.circle(strip, (x, y), 2, col, -1)
            canvas[BAR + h:] = strip

            if a.save:
                if enc is None:
                    enc = subprocess.Popen(
                        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                         "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
                         "-r", f"{a.fps}", "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
                         "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.save],
                        stdin=subprocess.PIPE)
                    t0 = time.time()
                    n_written = 0
                # 실시간 흐름을 fps 에 맞춰 적는다(빠뜨리거나 겹쳐 적지 않게)
                want = int((time.time() - t0) * a.fps)
                while n_written < want:
                    enc.stdin.write(canvas.tobytes())
                    n_written += 1
            if not a.headless:
                sh = canvas if a.scale == 1.0 else cv2.resize(
                    canvas, None, fx=a.scale, fy=a.scale)
                cv2.imshow("SAM-6D  live | CAD points | 3D bbox", sh)
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                    break
            else:
                time.sleep(max(0.0, 1.0 / a.fps - 0.005))
    except KeyboardInterrupt:
        pass
    finally:
        if enc is not None:
            enc.stdin.close(); enc.wait()
            print(f"[view] 녹화 저장: {a.save}")
        if not a.headless:
            cv2.destroyAllWindows()
        fr.close(); jr.close()


if __name__ == "__main__":
    main()
