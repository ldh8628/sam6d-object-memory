#!/usr/bin/env python3
"""make_review.py — a video you can actually check the integration against.

Builds ONE synchronised composite video plus an HTML viewer:

    +-----------------+-----------------+---------------------+
    | SLAM camera RGB | SAM camera RGB  | SAM RGB + 3D BBox   |  what each camera saw
    +-----------------+-----------------+---------------------+
    |  2D map: BOTH camera positions + headings at that       |  where each camera was
    |  instant and only a SHORT TRAIL (no full path), the     |
    |  prior map points, and the fused objects as squares —   |
    |  filled while the object is being detected right now,   |
    |  hollow otherwise                                       |
    +---------------------------------------------------------+

Both trajectories must already live in the SAME map frame — i.e. the SAM bag was
relocalised inside the SLAM camera's Atlas (see orbslam_ws/output/*_gate). Frames
are paired by absolute timestamp, so a paused frame shows the two cameras at the
same instant, not merely at the same frame number.

Run with the sam_yolo python (needs rosbags + cv2); H.264 via ffmpeg.

    /home/ldh9501/miniconda3/envs/sam_yolo/bin/python integration/make_review.py \
        --pair 260804_longcircle2
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT_ROOT = HERE.parent
DATA_DIR = HERE / "data"
OUTPUT_DIR = HERE / "output"
GATE_DIR = PROJECT_ROOT / "orbslam_ws" / "output" / "260804_gate"

COLOR_TOPIC = "/camera/camera/color/image_raw"
PANEL_W, PANEL_H = 640, 480          # per-camera panel
MAP_H = 780                          # map panel height (drives the map scale)
BAR_H = 44                           # header bar

C_SLAM = (232, 118, 43)              # BGR — blue-ish, SLAM camera
C_SAM = (60, 60, 220)                # BGR — red-ish, SAM camera
C_OBJ = (40, 160, 40)
C_OBJ_LIVE = (60, 200, 60)           # 지금 검출 중인 객체(속 채움)
C_BG = (255, 255, 255)
C_GRID = (235, 235, 235)
C_PTS = (205, 205, 205)


def log(m):
    print(f"[review] {m}", flush=True)


# --------------------------------------------------------------------------
def read_tum(path: Path):
    """TUM trajectory -> (stamps, positions, rotation matrices)."""
    ts, pos, rot = [], [], []
    for line in path.read_text().splitlines():
        p = line.split()
        if len(p) != 8:
            continue
        ts.append(float(p[0]))
        pos.append([float(p[1]), float(p[2]), float(p[3])])
        x, y, z, w = (float(v) for v in p[4:8])
        n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        x, y, z, w = x / n, y / n, z / n, w / n
        rot.append([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return np.array(ts), np.array(pos), np.array(rot)


def read_pcd_xyz(path: Path, max_pts=25000):
    if not path.is_file():
        return np.zeros((0, 3))
    lines = path.read_text(errors="replace").splitlines()
    start = next((i + 1 for i, l in enumerate(lines) if l.startswith("DATA")), None)
    if start is None:
        return np.zeros((0, 3))
    pts = []
    for l in lines[start:]:
        p = l.split()
        if len(p) >= 3:
            try:
                pts.append([float(p[0]), float(p[1]), float(p[2])])
            except ValueError:
                pass
    a = np.array(pts).reshape(-1, 3)
    if len(a) > max_pts:
        a = a[np.linspace(0, len(a) - 1, max_pts).astype(int)]
    return a


def bag_color_stamps(bag_dir: Path, topic: str):
    """Color message timestamps in recorded order (index == SAM-6D frame index)."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag_dir], default_typestore=ts) as r:
        con = [c for c in r.connections if c.topic == topic]
        if not con:
            raise SystemExit(f"topic {topic} not in {bag_dir}")
        return np.array([t / 1e9 for _, t, _ in r.messages(connections=con)])


def decode_frames(bag_dir: Path, topic: str, wanted: set, quality=88):
    """JPEG-encode only the frames we need, so both bags fit in memory."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = {}
    with AnyReader([bag_dir], default_typestore=ts) as r:
        con = [c for c in r.connections if c.topic == topic]
        i = -1
        for c, _t, raw in r.messages(connections=con):
            i += 1
            if i not in wanted:
                continue
            m = r.deserialize(raw, c.msgtype)
            buf = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            img = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" \
                else buf
            ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                out[i] = enc
    return out


# --------------------------------------------------------------------------
def read_occupancy_grid(pgm: Path) -> dict:
    """map_2d.pgm + map_2d.yaml -> {grid, origin, res}. ROS map_server 규약."""
    raw = pgm.read_bytes()
    if not raw.startswith(b"P5"):
        raise SystemExit(f"P5 PGM 이 아니다: {pgm}")
    tok, i = [], 2
    while len(tok) < 3:                                   # width, height, maxval
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b"#":                          # 주석 줄은 통째로 건너뛴다
            i = raw.find(b"\n", i) + 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        tok.append(int(raw[i:j]))
        i = j
    w, h, _ = tok
    g = np.frombuffer(raw[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)
    yml = dict(ln.split(":", 1) for ln in pgm.with_suffix(".yaml").read_text().splitlines()
               if ":" in ln and not ln.startswith("#"))
    ox, oy, _ = json.loads(yml["origin"].strip())
    # PGM 은 위에서 아래로 저장하므로 격자 좌표계로 되돌린다
    return {"grid": np.flipud(g).copy(), "origin": (ox, oy),
            "res": float(yml["resolution"])}


class MapView:
    """map frame (x right, z up) -> map panel pixels, with a static backdrop."""

    def __init__(self, width, height, pts_xz, objects, traj_slam, traj_sam, margin=54,
                 legend=None, grid=None):
        self.legend = legend if legend is not None else (
            (C_SLAM, "SLAM camera (front)"), (C_SAM, "SAM camera (side)"),
            (C_OBJ_LIVE, "object (filled = detected now)"))
        self.w, self.h = width, height
        self.grid = grid
        # trajectories define the framing; map points carry far outliers
        focus = np.vstack([traj_slam[:, [0, 2]], traj_sam[:, [0, 2]]])
        lo, hi = focus.min(axis=0), focus.max(axis=0)
        pad = 0.18 * max(hi - lo).max()
        lo, hi = lo - pad, hi + pad
        span_x, span_z = np.maximum(hi - lo, 1e-3)
        if grid is None:
            self.scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_z)
        else:
            # 점유격자를 깔면 궤적만 꽉 채우는 것보다 방이 보이는 게 낫다. 세로를 기준으로
            # 최소 창(기본 12 m)을 두고, 가로는 패널 비율만큼 저절로 넓어진다.
            self.scale = (height - 2 * margin) / max(span_z, grid.get("min_span_m", 12.0))
        self.cx, self.cz = (lo + hi) / 2
        self.margin = margin
        self.base = self._backdrop(pts_xz, objects, traj_slam, traj_sam)

    def _paint_grid(self, img):
        """점유격자를 지도 패널 배경으로 깐다.

        격자는 z-up 프레임(x_ros, y_ros)에서 만들었고 이 패널은 광학 프레임(x, z)이다.
        두 프레임 모두 같은 원본 map 프레임에서 나왔으므로 대응은 한 가지뿐이다:
            x_ros = z_panel,   y_ros = -x_panel
        """
        g = self.grid["grid"]
        gx0, gy0 = self.grid["origin"]
        res = self.grid["res"]
        gh, gw = g.shape
        # 열은 x_panel 에만, 행은 z_panel 에만 의존하므로 1차원 두 개로 만든다
        X = self.cx + (np.arange(self.w) - self.w / 2) / self.scale       # (w,)
        Z = self.cz - (np.arange(self.h) - self.h / 2) / self.scale       # (h,)
        xi = np.floor((Z - gx0) / res).astype(np.int64)                   # (h,) 격자 열
        yi = np.floor((-X - gy0) / res).astype(np.int64)                  # (w,) 격자 행
        ok = ((xi >= 0) & (xi < gw))[:, None] & ((yi >= 0) & (yi < gh))[None, :]
        cell = g[np.clip(yi, 0, gh - 1)[None, :], np.clip(xi, 0, gw - 1)[:, None]]
        img[ok & (cell == 254)] = (255, 255, 255)
        img[ok & (cell == 205)] = (236, 236, 236)
        img[ok & (cell == 0)] = (70, 70, 70)
        return img

    def px(self, x, z):
        return (int(round(self.w / 2 + (x - self.cx) * self.scale)),
                int(round(self.h / 2 - (z - self.cz) * self.scale)))

    def _backdrop(self, pts_xz, objects, traj_slam, traj_sam):
        img = np.full((self.h, self.w, 3), C_BG, np.uint8)
        if self.grid is not None:
            self._paint_grid(img)
            pts_xz = np.empty((0, 3))            # 격자가 있으면 점군 배경은 겹쳐 그리지 않는다
        else:
            # 1 m grid — 점유격자를 깔았을 때는 그리지 않는다. cv2.line 이 색을 덮어써서
            # 벽(어두운 셀) 위를 지나가면 벽을 지워 버린다.
            for gx in range(int(math.floor(self.cx - 8)), int(math.ceil(self.cx + 8))):
                x, _ = self.px(gx, 0)
                if 0 <= x < self.w:
                    cv2.line(img, (x, 0), (x, self.h), C_GRID, 1)
            for gz in range(int(math.floor(self.cz - 8)), int(math.ceil(self.cz + 8))):
                _, y = self.px(0, gz)
                if 0 <= y < self.h:
                    cv2.line(img, (0, y), (self.w, y), C_GRID, 1)
        for x, _y, z in pts_xz if pts_xz.shape[1] == 3 else []:
            u, v = self.px(x, z)
            if 0 <= u < self.w and 0 <= v < self.h:
                img[v, u] = C_PTS
        # 전체 궤적은 그리지 않는다 (요청: 현재 위치 + 짧은 꼬리만).
        # 객체도 배경이 아니라 프레임마다 네모로 그린다 (현재 검출 여부를 칠로 구분).
        self.obj_label_slots = {}
        placed = []
        for o in sorted(objects, key=lambda o: o["xyz"][2], reverse=True):
            u, v = self.px(o["xyz"][0], o["xyz"][2])
            lu, lv = u + 13, v + 5
            while any(abs(lv - pv) < 15 and abs(lu - pu) < 190 for pu, pv in placed):
                lv += 15
            placed.append((lu, lv))
            self.obj_label_slots[o["object_id"]] = (lu, lv)
        # scale bar: 1 m
        x0, y0 = self.margin, self.h - 24
        cv2.line(img, (x0, y0), (x0 + int(self.scale), y0), (90, 90, 90), 2)
        cv2.putText(img, "1 m", (x0 + int(self.scale) + 8, y0 + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1, cv2.LINE_AA)
        cv2.putText(img, "map frame top view (x right, z up)", (x0, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1, cv2.LINE_AA)
        # legend, in the space the trajectory leaves empty on the right
        lx, ly = self.w - 300, 34
        for col, txt in self.legend:
            cv2.circle(img, (lx, ly - 5), 7, col, -1, cv2.LINE_AA)
            cv2.putText(img, txt, (lx + 15, ly), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (70, 70, 70), 1, cv2.LINE_AA)
            ly += 26
        cv2.putText(img, "arrow = viewing direction", (lx - 1, ly + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 140, 140), 1, cv2.LINE_AA)
        return img

    def frame(self, cams, trails, objects=(), visible_ids=frozenset()):
        """cams: [(pos, R, colour, label)], trails: [Nx3 recent positions].

        objects 는 융합 객체 목록, visible_ids 는 '지금 검출되고 있는' object_id 집합.
        검출 중이면 속을 채운 네모, 아니면 테두리만 있는 네모로 그린다.
        """
        img = self.base.copy()
        for o in sorted(objects, key=lambda o: o["xyz"][2], reverse=True):
            u, v = self.px(o["xyz"][0], o["xyz"][2])
            live = o["object_id"] in visible_ids
            r = 9 if live else 7
            if live:
                cv2.rectangle(img, (u - r, v - r), (u + r, v + r), C_OBJ_LIVE, -1, cv2.LINE_AA)
                cv2.rectangle(img, (u - r, v - r), (u + r, v + r), (255, 255, 255), 2, cv2.LINE_AA)
            else:
                cv2.rectangle(img, (u - r, v - r), (u + r, v + r), C_OBJ, 2, cv2.LINE_AA)
            lu, lv = self.obj_label_slots.get(o["object_id"], (u + 13, v + 5))
            if lv - v > 8:
                cv2.line(img, (u, v), (lu - 3, lv - 4), (170, 205, 170), 1, cv2.LINE_AA)
            cv2.putText(img, f'{o["object_name"]} #{o["object_id"]}', (lu, lv),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (20, 110, 20) if live else (120, 120, 120),
                        2 if live else 1, cv2.LINE_AA)
        # 꼬리(최근 몇 초)만 — 전체 경로는 그리지 않는다
        for tr, (_p, _R, col, _lab) in zip(trails, cams):
            n = len(tr)
            for i in range(1, n):
                a = self.px(*tr[i - 1][[0, 2]]) if False else self.px(tr[i - 1][0], tr[i - 1][2])
                b = self.px(tr[i][0], tr[i][2])
                f = i / max(n - 1, 1)                       # 오래된 쪽은 옅게
                c = tuple(int(f * ch + (1 - f) * 245) for ch in col)
                cv2.line(img, a, b, c, max(1, int(1 + 2 * f)), cv2.LINE_AA)
        for pos, R, col, lab in cams:
            u, v = self.px(pos[0], pos[2])
            fwd = R[:, 2]                      # camera optical +Z in map coords
            n = math.hypot(fwd[0], fwd[2]) or 1.0
            du, dv = fwd[0] / n, fwd[2] / n
            tip = (int(u + du * 46), int(v - dv * 46))
            cv2.arrowedLine(img, (u, v), tip, col, 3, cv2.LINE_AA, tipLength=0.32)
            cv2.circle(img, (u, v), 9, col, -1, cv2.LINE_AA)
            cv2.circle(img, (u, v), 9, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, lab, (u + 13, v - 11), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, col, 2, cv2.LINE_AA)
        return img


BBOX_EDGES = ((0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4),
              (0, 4), (1, 5), (2, 6), (3, 7))


def obj_colour(name: str):
    """이름마다 고정된 색 (BGR)."""
    h = (hash(name) & 0xFFFFFF)
    c = np.array([(h >> 16) & 255, (h >> 8) & 255, h & 255], float)
    c = 90 + 0.62 * c                       # 너무 어둡지 않게
    return tuple(int(v) for v in c)


def draw_bbox3d(img, det, cad, K):
    """객체 CAD 의 AABB 8꼭짓점을 카메라로 투영해 12개 모서리만 그린다."""
    ent = cad.get(det["object"]) if cad else None
    if ent is None:
        return False
    lo = np.array(ent["min_mm"], float)
    hi = np.array(ent["max_mm"], float)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    R = np.array(det["R"], float)
    t = np.array(det["t_mm"], float)
    pc = corners @ R.T + t                                  # 카메라 좌표 (mm)
    if np.any(pc[:, 2] <= 1.0):
        return False
    fx, fy, cx, cy = K
    uv = np.stack([fx * pc[:, 0] / pc[:, 2] + cx,
                   fy * pc[:, 1] / pc[:, 2] + cy], axis=1)
    if not np.isfinite(uv).all():
        return False
    col = obj_colour(det["object"])
    for i, j in BBOX_EDGES:
        cv2.line(img, tuple(np.round(uv[i]).astype(int)),
                 tuple(np.round(uv[j]).astype(int)), col, 2, cv2.LINE_AA)
    lab = np.round(uv.min(axis=0)).astype(int)
    # 점수 대신 카메라 원점에서 3D 박스 중심까지의 거리를 m 로 적는다. 점수는 PEM 의
    # 내부 신뢰도라 화면에서 해석하기 어렵지만, 거리는 박스가 놓인 위치가 맞는지
    # 눈으로 바로 검증할 수 있는 물리량이다.
    dist_m = float(np.linalg.norm(pc.mean(axis=0))) / 1000.0
    cv2.putText(img, f'{det["object"]} {dist_m:.2f} m',
                (int(lab[0]), max(14, int(lab[1]) - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 2, cv2.LINE_AA)
    return True


def put_panel_label(img, text, colour):
    cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(img, text, (12, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.rectangle(img, (0, 0), (img.shape[1] - 1, img.shape[0] - 1), colour, 3)
    return img


def nearest(sorted_ts, t):
    i = int(np.searchsorted(sorted_ts, t))
    cands = [k for k in (i - 1, i) if 0 <= k < len(sorted_ts)]
    return min(cands, key=lambda k: abs(sorted_ts[k] - t))


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--pair", default="260804_longcircle2")
    ap.add_argument("--slam-bag", default="")
    ap.add_argument("--sam-bag", default="")
    ap.add_argument("--slam-traj", default=str(GATE_DIR / "map_slam" / "CameraTrajectory.txt"),
                    help="SLAM camera trajectory (map frame)")
    ap.add_argument("--sam-traj", default=str(GATE_DIR / "reloc_sam" / "CameraTrajectory.txt"),
                    help="SAM camera trajectory RELOCALISED IN THE SAME map frame")
    ap.add_argument("--map-points", default=str(GATE_DIR / "map_slam" / "orbslam3_map_points.pcd"))
    ap.add_argument("--grid", default="",
                    help="2D 점유격자 map_2d.pgm (옆에 같은 이름 .yaml 필요). 주면 지도 "
                         "패널 배경이 희소 점군 대신 이 격자가 된다.")
    ap.add_argument("--object-map", default="",
                    help="fused object_map.json (default: <pair>/fused/object_map.json)")
    ap.add_argument("--out-dir", default="", help="default: <pair output>/review")
    ap.add_argument("--fps", type=float, default=15.0, help="output video frame rate")
    ap.add_argument("--start", type=float, default=0.0, help="seconds into the overlap")
    ap.add_argument("--duration", type=float, default=0.0, help="0 = to the end")
    ap.add_argument("--trail", type=float, default=4.0,
                    help="motion trail length [s] — the full path is NOT drawn any more")
    ap.add_argument("--pem-json", default="",
                    help="SAM-6D per-frame poses (default: <pair>/sam/sam_objects.json)")
    ap.add_argument("--cad-extents", default=str(HERE / "cad_extents.json"),
                    help="object bbox sizes; build with integration/cad_extents.py")
    ap.add_argument("--extrinsic", default="",
                    help="4x4 T_camSLAM_camSAM .npy (default: <pair>/X_camSLAM_camSAM.npy). "
                         "Used to place each detection in the map frame so it can be "
                         "matched to the fused instance it belongs to.")
    ap.add_argument("--bag-info", default="",
                    help="bag_info.json for the SAM camera intrinsics "
                         "(default: <pair>/bag_info.json)")
    ap.add_argument("--det-hold", type=float, default=0.35,
                    help="a detection keeps the object 'live' for this long [s]")
    ap.add_argument("--match-gate", type=float, default=0.40,
                    help="detection -> fused instance matching radius [m]")
    ap.add_argument("--crf", type=int, default=23)
    a = ap.parse_args()

    pair_dir = DATA_DIR / a.pair
    out_root = OUTPUT_DIR / a.pair
    slam_bag = Path(a.slam_bag) if a.slam_bag else pair_dir / "SLAM"
    sam_bag = Path(a.sam_bag) if a.sam_bag else pair_dir / "SAM"
    out_dir = Path(a.out_dir) if a.out_dir else out_root / "review"
    out_dir.mkdir(parents=True, exist_ok=True)
    obj_path = Path(a.object_map) if a.object_map else out_root / "fused" / "object_map.json"

    # ---- inputs ---------------------------------------------------------
    ts_slam, pos_slam, rot_slam = read_tum(Path(a.slam_traj))
    ts_sam, pos_sam, rot_sam = read_tum(Path(a.sam_traj))
    log(f"SLAM traj {len(ts_slam)} poses [{ts_slam[0]:.3f}, {ts_slam[-1]:.3f}]")
    log(f"SAM  traj {len(ts_sam)} poses [{ts_sam[0]:.3f}, {ts_sam[-1]:.3f}]")
    if min(ts_slam[-1], ts_sam[-1]) - max(ts_slam[0], ts_sam[0]) <= 0:
        raise SystemExit("the two trajectories do not overlap in time — is the SAM bag "
                         "clock-shifted and relocalised in the SLAM map?")

    objects = []
    if obj_path.is_file():
        om = json.loads(obj_path.read_text())
        objects = [o for o in om["objects"] if o["status"] != "deleted"]
        log(f"{len(objects)} fused objects from {obj_path}")
    else:
        log(f"no fused object map at {obj_path} (map panel will show cameras only)")

    map_pts = read_pcd_xyz(Path(a.map_points))
    log(f"{len(map_pts)} prior map points")

    cs_slam = bag_color_stamps(slam_bag, COLOR_TOPIC)
    cs_sam = bag_color_stamps(sam_bag, COLOR_TOPIC)
    log(f"bag color frames: SLAM {len(cs_slam)}, SAM {len(cs_sam)}")

    # ---- SAM-6D 검출 (3D BBox 오버레이 + 지도 네모용) ---------------------
    pem_path = Path(a.pem_json) if a.pem_json else out_root / "sam" / "sam_objects.json"
    cad_path = Path(a.cad_extents)
    xnpy = Path(a.extrinsic) if a.extrinsic else out_root / "X_camSLAM_camSAM.npy"
    binfo = Path(a.bag_info) if a.bag_info else out_root / "bag_info.json"
    dets_by_frame, cad, X, K = {}, {}, None, None
    if pem_path.is_file():
        pj = json.loads(pem_path.read_text())
        for d in pj.get("detections", []):
            if not d.get("pem_ok") or d.get("R") is None or d.get("t_mm") is None:
                continue
            dets_by_frame.setdefault(int(d["frame_index"]), []).append(d)
        log(f"{sum(len(v) for v in dets_by_frame.values())} PEM poses "
            f"over {len(dets_by_frame)} sampled frames")
    else:
        log(f"no PEM poses at {pem_path} — 3D BBox panel will be empty")
    if cad_path.is_file():
        cad = json.loads(cad_path.read_text())
    else:
        log(f"no CAD extents at {cad_path} — run integration/cad_extents.py")
    if xnpy.is_file():
        X = np.load(xnpy)
    if binfo.is_file():
        bi = json.loads(binfo.read_text())
        k = (bi.get("sam") or {}).get("camera", {}).get("K")
        if k:
            K = (k[0], k[4], k[2], k[5])          # fx, fy, cx, cy
    if K is None:
        K = (388.0, 387.5, 322.7, 244.8)
        log("SAM intrinsics not found in bag_info.json — using D455 defaults")

    # 검출마다 map 좌표를 구해 융합 인스턴스에 붙인다
    det_map_id = {}
    if dets_by_frame and X is not None and objects:
        obj_xyz = np.array([o["xyz"] for o in objects])
        obj_ids = [o["object_id"] for o in objects]
        obj_names = [o["object_name"] for o in objects]
        for fi, ds in dets_by_frame.items():
            if fi >= len(cs_sam):
                continue
            t_det = cs_sam[fi]
            i_s = nearest(ts_slam, t_det)
            if abs(ts_slam[i_s] - t_det) > 0.2:
                continue
            T_map_camS = np.eye(4)
            T_map_camS[:3, :3] = rot_slam[i_s]
            T_map_camS[:3, 3] = pos_slam[i_s]
            T_map_camSAM = T_map_camS @ X
            for d in ds:
                p_obj = np.array(d["t_mm"], float) / 1000.0
                p_map = T_map_camSAM[:3, :3] @ p_obj + T_map_camSAM[:3, 3]
                same = [j for j, n in enumerate(obj_names) if n == d["object"]]
                if not same:
                    continue
                dist = np.linalg.norm(obj_xyz[same] - p_map, axis=1)
                j = int(np.argmin(dist))
                if dist[j] <= a.match_gate:
                    det_map_id[(fi, d["object"], id(d))] = obj_ids[same[j]]
                    d["_map_id"] = obj_ids[same[j]]
        log(f"{sum(1 for v in dets_by_frame.values() for d in v if '_map_id' in d)}"
            f" detections matched to a fused instance (gate {a.match_gate} m)")

    # ---- common time grid ----------------------------------------------
    t0 = max(ts_slam[0], ts_sam[0], cs_slam[0], cs_sam[0]) + a.start
    t1 = min(ts_slam[-1], ts_sam[-1], cs_slam[-1], cs_sam[-1])
    if a.duration > 0:
        t1 = min(t1, t0 + a.duration)
    grid = np.arange(t0, t1, 1.0 / a.fps)
    log(f"{len(grid)} output frames @ {a.fps} fps ({grid[-1] - grid[0]:.1f}s)")

    idx_slam_img = [nearest(cs_slam, t) for t in grid]
    idx_sam_img = [nearest(cs_sam, t) for t in grid]
    idx_slam_pose = [nearest(ts_slam, t) for t in grid]
    idx_sam_pose = [nearest(ts_sam, t) for t in grid]

    log("decoding SLAM camera frames ...")
    fr_slam = decode_frames(slam_bag, COLOR_TOPIC, set(idx_slam_img))
    log("decoding SAM camera frames ...")
    fr_sam = decode_frames(sam_bag, COLOR_TOPIC, set(idx_sam_img))
    log(f"decoded {len(fr_slam)} + {len(fr_sam)} unique frames")

    # ---- video ----------------------------------------------------------
    W = PANEL_W * 3                      # SLAM RGB | SAM RGB | SAM RGB + 3D BBox
    H = BAR_H + PANEL_H + MAP_H
    view = MapView(W, MAP_H, map_pts, objects, pos_slam, pos_sam,
                   grid=read_occupancy_grid(Path(a.grid)) if a.grid else None)
    mp4 = out_dir / "sync_view.mp4"
    cmd = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
           "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
           "-r", f"{a.fps}", "-i", "-",
           "-c:v", "libx264", "-preset", "faster", "-crf", str(a.crf),
           "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    trail_n = int(a.trail * a.fps)
    # 비디오 프레임마다 '가장 가까운 샘플 프레임' — det_hold 초 안쪽이면 살아 있다고 본다
    sampled = np.array(sorted(dets_by_frame)) if dets_by_frame else np.zeros(0, int)
    det_frame_for = []
    for t in grid:
        if not len(sampled):
            det_frame_for.append(None); continue
        st = cs_sam[sampled]
        j = int(np.argmin(np.abs(st - t)))
        det_frame_for.append(int(sampled[j]) if abs(st[j] - t) <= a.det_hold else None)
    log(f"{sum(x is not None for x in det_frame_for)}/{len(grid)} video frames "
        f"carry a detection (hold {a.det_hold}s)")
    per_frame = []
    for k, t in enumerate(grid):
        canvas = np.full((H, W, 3), C_BG, np.uint8)
        # header
        cv2.rectangle(canvas, (0, 0), (W, BAR_H), (28, 28, 28), -1)
        rel = t - grid[0]
        cv2.putText(canvas, f"{a.pair}   t = {rel:6.2f}s   ({t:.3f})   frame {k + 1}/{len(grid)}",
                    (14, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 245, 245), 1, cv2.LINE_AA)
        # camera panels
        for j, (frames, iidx, col, name) in enumerate((
                (fr_slam, idx_slam_img, C_SLAM, "SLAM camera (front)"),
                (fr_sam, idx_sam_img, C_SAM, "SAM camera (side)"))):
            enc = frames.get(iidx[k])
            panel = (cv2.imdecode(enc, cv2.IMREAD_COLOR) if enc is not None
                     else np.full((PANEL_H, PANEL_W, 3), 60, np.uint8))
            if panel.shape[:2] != (PANEL_H, PANEL_W):
                panel = cv2.resize(panel, (PANEL_W, PANEL_H))
            dt = (cs_slam if j == 0 else cs_sam)[iidx[k]] - t
            put_panel_label(panel, f"{name}   bag frame {iidx[k]}   dt {dt * 1000:+.0f} ms", col)
            canvas[BAR_H:BAR_H + PANEL_H, j * PANEL_W:(j + 1) * PANEL_W] = panel
        # 3번째 패널: SAM RGB + 3D BBox (박스만)
        enc = fr_sam.get(idx_sam_img[k])
        det_panel = (cv2.imdecode(enc, cv2.IMREAD_COLOR) if enc is not None
                     else np.full((PANEL_H, PANEL_W, 3), 60, np.uint8))
        if det_panel.shape[:2] != (PANEL_H, PANEL_W):
            det_panel = cv2.resize(det_panel, (PANEL_W, PANEL_H))
        fi_near = det_frame_for[k]
        drawn, live_ids = 0, set()
        if fi_near is not None:
            for d in dets_by_frame.get(fi_near, []):
                if draw_bbox3d(det_panel, d, cad, K):
                    drawn += 1
                if "_map_id" in d:
                    live_ids.add(d["_map_id"])
        put_panel_label(det_panel,
                        f"SAM 6D pose (3D bbox)   {drawn} obj"
                        + ("" if fi_near is None else f"   sampled frame {fi_near}"),
                        C_OBJ_LIVE)
        canvas[BAR_H:BAR_H + PANEL_H, 2 * PANEL_W:3 * PANEL_W] = det_panel

        # map panel
        i_s, i_m = idx_slam_pose[k], idx_sam_pose[k]
        cams = [(pos_slam[i_s], rot_slam[i_s], C_SLAM, "SLAM"),
                (pos_sam[i_m], rot_sam[i_m], C_SAM, "SAM")]
        trails = [pos_slam[max(0, i_s - trail_n):i_s + 1],
                  pos_sam[max(0, i_m - trail_n):i_m + 1]]
        canvas[BAR_H + PANEL_H:, :] = view.frame(cams, trails, objects, live_ids)
        proc.stdin.write(canvas.tobytes())

        per_frame.append({
            "t": round(float(rel), 3), "abs": round(float(t), 3),
            "slam_frame": int(idx_slam_img[k]), "sam_frame": int(idx_sam_img[k]),
            "slam_xyz": [round(float(v), 3) for v in pos_slam[i_s]],
            "sam_xyz": [round(float(v), 3) for v in pos_sam[i_m]],
        })
        if (k + 1) % 200 == 0:
            log(f"  {k + 1}/{len(grid)} frames")
    proc.stdin.close()
    if proc.wait() != 0:
        raise SystemExit("ffmpeg failed")
    log(f"wrote {mp4} ({mp4.stat().st_size / 1e6:.1f} MB)")
    # poster: browsers show nothing until the first frame is decoded otherwise
    subprocess.run(["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
                    "-i", str(mp4), "-vframes", "1", "-q:v", "4",
                    str(out_dir / "poster.jpg")], check=False)

    meta = {
        "pair": a.pair, "fps": a.fps, "frames": len(grid),
        "duration_s": round(float(grid[-1] - grid[0]), 3),
        "sources": {"slam_bag": str(slam_bag), "sam_bag": str(sam_bag),
                    "slam_traj": a.slam_traj, "sam_traj": a.sam_traj,
                    "object_map": str(obj_path) if obj_path.is_file() else None},
        "objects": [{"id": o["object_id"], "name": o["object_name"],
                     "status": o["status"], "xyz": o["xyz"]} for o in objects],
        "per_frame": per_frame,
    }
    (out_dir / "sync_data.json").write_text(json.dumps(meta), encoding="utf-8")
    write_html(out_dir / "review.html", meta)
    log(f"open  {out_dir / 'review.html'}")


# --------------------------------------------------------------------------
def write_html(path: Path, meta: dict):
    objs = "".join(
        f'<tr><td>#{o["id"]}</td><td>{o["name"]}</td><td>{o["status"]}</td>'
        f'<td>{o["xyz"][0]:.2f}, {o["xyz"][1]:.2f}, {o["xyz"][2]:.2f}</td></tr>'
        for o in meta["objects"])
    src = meta["sources"]
    # inlined, NOT fetched: Chrome blocks fetch() of a sibling file over file://
    inline = json.dumps({"fps": meta["fps"], "per_frame": meta["per_frame"]},
                        separators=(",", ":"))
    path.write_text(f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<title>{meta['pair']} — SLAM + SAM 동기 확인</title>
<style>
 :root {{ color-scheme: light dark; }}
 body {{ font-family: system-ui, -apple-system, "Noto Sans KR", sans-serif;
        margin: 0; padding: 20px 24px 60px; background: #fbfbfc; color: #1a1a1a; }}
 h1 {{ font-size: 20px; margin: 0 0 4px; }}
 .sub {{ color: #666; font-size: 13px; margin-bottom: 16px; }}
 video {{ max-width: 100%; max-height: 86vh; display: block; margin: 0 auto;
          background: #000; border-radius: 8px; box-shadow: 0 2px 12px rgba(0,0,0,.12); }}
 .bar {{ display: flex; gap: 10px; align-items: center; flex-wrap: wrap;
         margin: 12px 0 18px; }}
 button {{ font: inherit; padding: 6px 12px; border-radius: 6px; cursor: pointer;
           border: 1px solid #ccc; background: #fff; }}
 button:hover {{ background: #f0f0f0; }}
 .read {{ font-family: ui-monospace, Menlo, monospace; font-size: 13px;
          background: #fff; border: 1px solid #e3e3e6; border-radius: 8px;
          padding: 10px 14px; display: inline-block; min-width: 520px; }}
 table {{ border-collapse: collapse; font-size: 13px; background: #fff; }}
 th, td {{ border: 1px solid #e3e3e6; padding: 4px 10px; text-align: left; }}
 th {{ background: #f4f4f6; }}
 .legend span {{ display: inline-block; margin-right: 16px; font-size: 13px; }}
 .dot {{ display: inline-block; width: 11px; height: 11px; border-radius: 50%;
         margin-right: 5px; vertical-align: -1px; }}
 details {{ margin-top: 22px; font-size: 13px; color: #444; }}
 code {{ background: #f0f0f2; padding: 1px 5px; border-radius: 4px; }}
</style></head><body>
<h1>{meta['pair']} — SLAM 카메라와 SAM 카메라 동기 확인</h1>
<div class="sub">위: 두 카메라가 <b>같은 순간</b>에 본 RGB · 아래: 같은 순간의 두 카메라 위치와 시선 방향(맵 좌표계 상면도).
{meta['frames']}프레임 / {meta['duration_s']}초 / {meta['fps']}fps</div>

<video id="v" src="sync_view.mp4" poster="poster.jpg" controls preload="auto"></video>

<div class="bar">
  <button onclick="step(-1)">◀ 1프레임</button>
  <button onclick="step(1)">1프레임 ▶</button>
  <button onclick="rate(0.25)">0.25x</button>
  <button onclick="rate(0.5)">0.5x</button>
  <button onclick="rate(1)">1x</button>
  <button onclick="rate(2)">2x</button>
  <span class="legend">
    <span><i class="dot" style="background:#2b76e8"></i>SLAM 카메라(정면)</span>
    <span><i class="dot" style="background:#dc3c3c"></i>SAM 카메라(측면)</span>
    <span><i class="dot" style="background:#28a028"></i>융합된 객체</span>
  </span>
</div>

<div class="read" id="read">재생하면 현재 시각의 값이 표시됩니다.</div>

<h3 style="font-size:15px;margin:26px 0 8px">융합된 객체 (map 좌표계)</h3>
<table><tr><th>id</th><th>이름</th><th>상태</th><th>xyz [m]</th></tr>{objs}</table>

<details><summary>입력 출처</summary>
<p>SLAM bag <code>{src['slam_bag']}</code><br>
SAM bag <code>{src['sam_bag']}</code><br>
SLAM 궤적 <code>{src['slam_traj']}</code><br>
SAM 궤적(같은 맵에 relocalize) <code>{src['sam_traj']}</code><br>
객체 <code>{src['object_map']}</code></p>
<p>두 영상은 <b>절대 타임스탬프</b>로 짝지었습니다(패널 라벨의 <code>dt</code>가 그 프레임의 시간 오차).
두 궤적은 같은 ORB-SLAM3 맵 좌표계이므로 아래 지도의 두 점 사이 간격이 곧 카메라 간 장착 오프셋입니다.</p>
</details>

<script>
const V = document.getElementById('v'), R = document.getElementById('read');
const D = {inline}, FPS = D.fps;
function fmt(a) {{ return a.map(v => v.toFixed(2).padStart(6)).join(', '); }}
function tick() {{
  if (D) {{
    const i = Math.max(0, Math.min(D.per_frame.length - 1, Math.round(V.currentTime * FPS)));
    const f = D.per_frame[i];
    R.textContent =
      `t = ${{f.t.toFixed(2)}} s   (프레임 ${{i + 1}}/${{D.per_frame.length}})\\n` +
      `SLAM  bag frame ${{String(f.slam_frame).padStart(4)}}   xyz = [${{fmt(f.slam_xyz)}}]\\n` +
      `SAM   bag frame ${{String(f.sam_frame).padStart(4)}}   xyz = [${{fmt(f.sam_xyz)}}]`;
    R.style.whiteSpace = 'pre';
  }}
  requestAnimationFrame(tick);
}}
requestAnimationFrame(tick);
function step(n) {{ V.pause(); V.currentTime = Math.max(0, V.currentTime + n / FPS); }}
function rate(r) {{ V.playbackRate = r; }}
document.addEventListener('keydown', e => {{
  if (e.key === 'ArrowRight') step(1);
  else if (e.key === 'ArrowLeft') step(-1);
  else if (e.key === ' ') {{ e.preventDefault(); V.paused ? V.play() : V.pause(); }}
}});
</script>
</body></html>
""", encoding="utf-8")


if __name__ == "__main__":
    main()
