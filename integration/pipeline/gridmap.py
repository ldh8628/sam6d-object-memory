#!/usr/bin/env python3
"""관측 단위 log-odds 점유격자 — ROS Navigation 에 쓸 수 있는 2D 맵을 만든다.

왜 다시 썼나
------------
처음에는 '합쳐진 점군 + 각 점을 가장 가까운 궤적 포즈에서 봤다고 가정'해서 광선을 쐈다.
그건 틀렸고, 세 백엔드에서 서로 다른 증상으로 드러났다.

  * hdl  : 점유 후보 셀의 38.8% 가 **키프레임 하나만 지지**하는 셀이었다(사람·원거리
           희소 반사·노이즈). 관측 원점이 틀리니 다른 키프레임의 광선이 그 셀을 지워
           주지 못해 전부 장애물로 남았다.
  * 카메라: 원점이 틀린 광선이 방사형 줄무늬를 만들고 자유 공간을 엉뚱하게 칠했다.

표준 방식은 관측(스캔/프레임)마다 **그 관측의 진짜 센서 원점**에서 각 끝점까지 광선을
쏘고, 지나간 셀에 miss, 끝점에 hit 를 누적해 log-odds 로 판정하는 것이다. 그러면 한 번만
찍힌 셀은 다른 관측의 광선이 통과하며 저절로 지워진다.

    L(cell) = n_hit * L_OCC + n_miss * L_FREE
    점유: L > +0.62 (p>0.65)  ·  자유: L < -1.41 (p<0.196)  ·  그 사이는 미지

관측 원본
---------
    hdl        graph_dump/<kf>/{cloud.pcd, data} — 키프레임마다 점군과 최적화 포즈가 있다
    ORB3/RTAB  표준 bag 의 aligned depth + 그 백엔드의 궤적 (합쳐진 dense 점군이 아니라)

카메라 쪽을 각 백엔드의 dense 점군 대신 depth 로 다시 투영하는 이유는 두 가지다.
(1) dense 점군에는 어느 프레임에서 봤는지가 없어 광선을 쏠 원점이 없다.
(2) 그래서 두 지도의 차이가 '점군 생성 방식'이 아니라 **궤적 차이만** 반영하게 되어
    백엔드 비교가 깨끗해진다.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common as C  # noqa: E402

import numpy as np  # noqa: E402

L_OCC, L_FREE = 0.85, -0.40           # 한 번 맞음 / 한 번 통과
L_OCC_T, L_FREE_T = 0.62, -1.41       # p 0.65 / 0.196 (ROS map_server 기본 문턱)
UNKNOWN, FREE, OCCUPIED = 205, 254, 0


class LogOddsGrid:
    """고정 경계 위의 log-odds 격자. 관측을 하나씩 흘려 넣는다(메모리 상수)."""

    def __init__(self, lo, hi, res: float):
        self.lo = np.asarray(lo, float)
        self.res = res
        self.W = int(np.ceil((hi[0] - self.lo[0]) / res))
        self.H = int(np.ceil((hi[1] - self.lo[1]) / res))
        self.hit = np.zeros((self.H, self.W), np.int32)
        self.miss = np.zeros((self.H, self.W), np.int32)
        self.n_obs = 0

    def _cells(self, xy):
        c = np.floor((xy - self.lo) / self.res).astype(np.int64)
        ok = (c[:, 0] >= 0) & (c[:, 0] < self.W) & (c[:, 1] >= 0) & (c[:, 1] < self.H)
        return c[ok]

    def add(self, origin_xy, pts_xy, max_range: float) -> None:
        """관측 하나: 원점에서 각 끝점까지 광선을 쏜다."""
        if len(pts_xy) == 0:
            return
        self.n_obs += 1
        o = np.asarray(origin_xy, float)
        v = pts_xy - o
        L = np.linalg.norm(v, axis=1)
        keep = (L > self.res) & (L < max_range)
        if not keep.any():
            return
        v, L = v[keep], L[keep]
        end = o + v
        c = self._cells(end)
        if len(c):
            np.add.at(self.hit, (c[:, 1], c[:, 0]), 1)
        # 끝점 직전까지 통과 셀을 찍는다. 원점이 하나라 덩어리로 벡터화된다.
        step = self.res * 0.7
        u = v / L[:, None]
        n = np.maximum((L / step).astype(np.int32) - 1, 0)
        nmax = int(n.max())
        if nmax <= 0:
            return
        for s in range(0, len(v), 20000):
            uu, nn = u[s:s + 20000], n[s:s + 20000]
            k = np.arange(nmax)[None, :]
            valid = k < nn[:, None]
            P = o[None, None, :] + uu[:, None, :] * (k[:, :, None] * step)
            cm = self._cells(P[valid])
            if len(cm):
                np.add.at(self.miss, (cm[:, 1], cm[:, 0]), 1)

    def finish(self, min_hits: int = 2) -> np.ndarray:
        """log-odds -> ROS 점유격자 값 (0 점유 / 254 자유 / 205 미지)."""
        Lv = self.hit * L_OCC + self.miss * L_FREE
        seen = (self.hit + self.miss) > 0
        occ = (Lv > L_OCC_T) & (self.hit >= min_hits)
        grid = np.full(Lv.shape, UNKNOWN, np.uint8)
        grid[seen] = FREE                       # 관측됐고 점유가 아니면 자유로 본다
        grid[occ] = OCCUPIED
        return grid


# --------------------------------------------------------------------------
# 관측 원본
# --------------------------------------------------------------------------
def hdl_observations(dump: Path):
    """graph_dump/<kf>/{cloud.pcd,data} -> (센서 원점, map 좌표 점군)."""
    for d in sorted(p for p in Path(dump).iterdir() if p.is_dir()):
        cloud, meta = d / "cloud.pcd", d / "data"
        if not (cloud.is_file() and meta.is_file()):
            continue
        lines = meta.read_text().splitlines()
        try:
            i = lines.index("estimate")
        except ValueError:
            continue
        T = np.array([[float(x) for x in lines[i + 1 + r].split()] for r in range(4)])
        p = C.read_pcd(cloud)
        yield T[:3, 3], p @ T[:3, :3].T + T[:3, 3]


def depth_observations(bag_dir: Path, traj_path: Path, depth_topic: str, K,
                       stride: int = 6, px_step: int = 4, max_pts: int = 8000,
                       d_min: float = 0.3, d_max: float = 5.0, tol: float = 0.05,
                       color_topic: str = "", zup: bool = True):
    """표준 bag 의 aligned depth + 궤적 -> (센서 원점, map 좌표 점군), 모두 z-up.

    depth 는 거리가 멀수록 오차가 제곱으로 커지므로 d_max 로 자른다. D455 스테레오는
    5 m 를 넘어가면 벽이 두껍게 번져 항법 지도로는 못 쓴다.

    color_topic 을 주면 (원점, 점군, rgb) 세 쌍을 낸다. depth 는 color 에 정렬돼 있고
    두 토픽의 스탬프가 같으므로(exact sync) 한 번의 순회로 짝지을 수 있다.
    """
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    ts, xyz, quat = C.read_tum(traj_path)
    fx, fy, cx, cy = K
    # 2D 격자는 z-up 이 필요하지만 3D PLY 는 trajectory.txt 와 같은 프레임이어야
    # 유니티에서 맵과 궤적이 겹친다. 그래서 회전을 선택으로 둔다.
    R = C.R_OPT2ROS if zup else np.eye(3)
    rng = np.random.default_rng(0)
    store = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(bag_dir)], default_typestore=store) as r:
        topics = [depth_topic] + ([color_topic] if color_topic else [])
        con = [c for c in r.connections if c.topic in topics]
        if not any(c.topic == depth_topic for c in con):
            C.die(f"{bag_dir} 에 {depth_topic} 이 없다")
        i, colors = -1, {}
        for c, _t, raw in r.messages(connections=con):
            m = r.deserialize(raw, c.msgtype)
            key = (m.header.stamp.sec, m.header.stamp.nanosec)
            if c.topic == color_topic:
                colors[key] = m
                if len(colors) > 40:                      # 오래된 것부터 버려 메모리 고정
                    colors.pop(next(iter(colors)))
                continue
            i += 1
            if i % stride:
                continue
            t = key[0] + key[1] * 1e-9
            j = int(np.argmin(np.abs(ts - t)))
            if abs(ts[j] - t) > tol:
                continue                                  # 포즈가 없는 프레임은 건너뛴다
            D = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            D = D[::px_step, ::px_step].astype(np.float32) * 1e-3
            vv, uu = np.nonzero((D > d_min) & (D < d_max))
            if len(vv) == 0:
                continue
            z = D[vv, uu]
            u = uu * px_step
            v = vv * px_step
            P = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], 1)
            rgb = None
            cm = colors.get(key)
            if cm is not None:
                img = np.frombuffer(cm.data, np.uint8).reshape(cm.height, cm.width, 3)
                rgb = img[v, u]
                if cm.encoding.lower() == "bgr8":
                    rgb = rgb[:, ::-1]
            if len(P) > max_pts:
                sel = rng.choice(len(P), max_pts, replace=False)
                P, rgb = P[sel], (rgb[sel] if rgb is not None else None)
            Rwc = C.quat_to_R(quat[j])
            Pm = (P @ Rwc.T + xyz[j]) @ R.T
            yield (R @ xyz[j], Pm) if rgb is None else (R @ xyz[j], Pm, rgb)


def velodyne_observations(bag_dir: Path, traj_path: Path, topic: str = "/velodyne_points",
                          stride: int = 2, max_range: float = 30.0, tol: float = 0.15):
    """Velodyne bag + hdl 궤적 -> (센서 원점, map 좌표 점군).

    hdl 의 keyframe 점군(graph_dump)이 없을 때 3D 지도를 만드는 경로다. 스캔 스탬프에
    가장 가까운 포즈를 쓴다 — hdl 궤적은 9.9 Hz 라 10 Hz 스캔과 거의 1:1 로 맞는다.
    """
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    ts, xyz, quat = C.read_tum(traj_path)
    store = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(bag_dir)], default_typestore=store) as r:
        con = [c for c in r.connections if c.topic == topic]
        if not con:
            C.die(f"{bag_dir} 에 {topic} 이 없다")
        i = -1
        for c, _t, raw in r.messages(connections=con):
            i += 1
            if i % stride:
                continue
            m = r.deserialize(raw, c.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            j = int(np.argmin(np.abs(ts - t)))
            if abs(ts[j] - t) > tol:
                continue
            off = {f.name: f.offset for f in m.fields}
            if not {"x", "y", "z"} <= set(off):
                continue
            buf = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
            P = np.stack([buf[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
                          for k in ("x", "y", "z")], 1).astype(np.float64)
            P = P[np.isfinite(P).all(1)]
            P = P[np.linalg.norm(P, axis=1) < max_range]
            if not len(P):
                continue
            yield xyz[j], P @ C.quat_to_R(quat[j]).T + xyz[j]


def export_ply(obs, dst: Path, voxel: float = 0.03, max_pts: int = 4_000_000) -> dict:
    """관측 흐름을 복셀로 솎아 binary little-endian PLY 로 쓴다.

    binary LE 인 이유는 Unity 의 흔한 점군 임포터(Pcx)가 읽는 유일한 형식이고 ascii 의
    1/3 크기이기 때문이다. 색이 있으면 x y z + rgb, 없으면 x y z 만 쓴다.
    """
    keys, pts, cols, chunks = None, [], [], 0
    have_rgb = None

    def compress(P, Cc):
        nonlocal keys
        q = np.floor(P / voxel).astype(np.int64)
        k = (q[:, 0] << 42) ^ (q[:, 1] << 21) ^ q[:, 2]
        _u, idx = np.unique(k, return_index=True)
        return P[idx], (Cc[idx] if Cc is not None else None), k[idx]

    acc_P, acc_C = [], []
    for item in obs:
        p = item[1]
        c = item[2] if len(item) > 2 else None
        if have_rgb is None:
            have_rgb = c is not None
        acc_P.append(p)
        if have_rgb:
            acc_C.append(c)
        chunks += 1
        if chunks % 200 == 0:                      # 주기적으로 눌러 메모리를 묶어둔다
            P = np.concatenate(acc_P)
            Cc = np.concatenate(acc_C) if have_rgb else None
            P, Cc, _k = compress(P, Cc)
            acc_P, acc_C = [P], ([Cc] if have_rgb else [])
    if not acc_P:
        C.die("점이 하나도 없다 — 궤적과 bag 이 맞는지 확인해라")
    P = np.concatenate(acc_P)
    Cc = np.concatenate(acc_C) if have_rgb else None
    P, Cc, _k = compress(P, Cc)
    if len(P) > max_pts:
        sel = np.random.default_rng(0).choice(len(P), max_pts, replace=False)
        P, Cc = P[sel], (Cc[sel] if have_rgb else None)

    dst.parent.mkdir(parents=True, exist_ok=True)
    head = ["ply", "format binary_little_endian 1.0",
            f"element vertex {len(P)}",
            "property float x", "property float y", "property float z"]
    if have_rgb:
        head += ["property uchar red", "property uchar green", "property uchar blue"]
    head += ["end_header", ""]
    if have_rgb:
        rec = np.zeros(len(P), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                      ("red", "u1"), ("green", "u1"), ("blue", "u1")])
        rec["red"], rec["green"], rec["blue"] = Cc[:, 0], Cc[:, 1], Cc[:, 2]
    else:
        rec = np.zeros(len(P), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    rec["x"], rec["y"], rec["z"] = P[:, 0], P[:, 1], P[:, 2]
    with dst.open("wb") as f:
        f.write("\n".join(head).encode())
        f.write(rec.tobytes())
    return {"points": int(len(P)), "voxel": voxel, "rgb": bool(have_rgb),
            "size_mb": round(dst.stat().st_size / 2**20, 1)}


# --------------------------------------------------------------------------
def build(obs, traj_xyz_zup: np.ndarray, res: float = 0.05, z_lo: float = 0.30,
          z_hi: float = 2.0, max_range: float = 12.0, min_hits: int = 2,
          margin: float = 2.0, floor: float | None = None) -> dict:
    """관측 흐름 -> 격자. 경계는 궤적 + margin + max_range 로 미리 잡는다."""
    T = traj_xyz_zup[:, :2]
    lo = T.min(0) - (max_range + margin)
    hi = T.max(0) + (max_range + margin)
    g = LogOddsGrid(lo, hi, res)

    # 바닥은 처음 N 개 관측을 모아 한 번만 검출하고, 그 N 개도 버리지 않고 함께 넣는다.
    N_PROBE = 60
    zs = float(np.median(traj_xyz_zup[:, 2]))
    probe, z0, z1, basis = [], None, None, "floor"

    def slice_and_add(origin, pts):
        m = (pts[:, 2] > z0) & (pts[:, 2] < z1)
        g.add(origin[:2], pts[m][:, :2], max_range)

    for origin, pts in obs:
        if z0 is None:
            probe.append((origin, pts))
            if len(probe) < N_PROBE:
                continue
            if floor is None:
                fc = C.detect_floor(np.concatenate([p[::7, 2] for _o, p in probe]))
                floor, basis = (fc[0], "floor") if fc else (zs, "sensor")
                top = fc[1] - 0.20 if fc else None
            else:
                top = None
            z0 = floor + z_lo
            z1 = floor + z_hi if top is None else min(floor + z_hi, top)
            C.log(f"  바닥 기준 {basis} {floor:+.2f} -> z 슬라이스 {z0:+.2f}~{z1:+.2f}")
            for o2, p2 in probe:
                slice_and_add(o2, p2)
            probe = []
            continue
        slice_and_add(origin, pts)

    if z0 is None:                                   # 관측이 N_PROBE 개보다 적었다
        if not probe:
            C.die("관측이 없어 격자를 만들 수 없다")
        if floor is None:
            fc = C.detect_floor(np.concatenate([p[::7, 2] for _o, p in probe]))
            floor, basis = (fc[0], "floor") if fc else (zs, "sensor")
        z0, z1 = floor + z_lo, floor + z_hi
        for o2, p2 in probe:
            slice_and_add(o2, p2)
    grid = g.finish(min_hits)
    # 실제로 쓰인 영역만 잘라낸다 (미지로만 채워진 테두리 제거)
    used = np.argwhere(grid != UNKNOWN)
    if len(used):
        y0, x0 = used.min(0)
        y1, x1 = used.max(0) + 1
        grid = grid[y0:y1, x0:x1]
        lo = lo + np.array([x0, y0]) * res
    return {"grid": grid, "origin": (float(lo[0]), float(lo[1])), "res": res,
            "floor": float(floor), "z_basis": basis,
            "z_slice": [float(z0), float(z1)],
            "observations": g.n_obs,
            "occupied": int((grid == OCCUPIED).sum()),
            "free": int((grid == FREE).sum())}


def load_map(stem: Path) -> dict:
    """이미 만들어 둔 map_2d.{pgm,yaml} 을 다시 읽는다 (융합 그림이 백엔드 지도와
    한 픽셀도 어긋나지 않게 하려고 점군에서 다시 만들지 않는다)."""
    pgm = stem.with_suffix(".pgm")
    raw = pgm.read_bytes()
    if not raw.startswith(b"P5"):
        C.die(f"P5 PGM 이 아니다: {pgm}")
    tok, i = [], 2
    while len(tok) < 3:
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b"#":
            i = raw.find(b"\n", i) + 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        tok.append(int(raw[i:j]))
        i = j
    w, h, _ = tok
    grid = np.flipud(np.frombuffer(raw[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)).copy()
    txt = stem.with_suffix(".yaml").read_text()
    yml = dict(ln.split(":", 1) for ln in txt.splitlines() if ":" in ln and not ln.startswith("#"))
    ox, oy, _ = json.loads(yml["origin"].strip())
    zs = [0.0, 0.0]
    for ln in txt.splitlines():
        if ln.startswith("# z 슬라이스"):
            p = ln.split()
            zs = [float(p[3]), float(p[5])]
    return {"grid": grid, "origin": (ox, oy), "res": float(yml["resolution"]),
            "z_slice": zs, "occupied": int((grid == OCCUPIED).sum()),
            "free": int((grid == FREE).sum())}


def write_map(g: dict, traj_zup: np.ndarray, stem: Path, title: str,
              objects: list | None = None) -> None:
    """격자를 map_2d.{pgm,yaml,png} 로 쓴다. objects 는 z-up 좌표."""
    stem.parent.mkdir(parents=True, exist_ok=True)
    C.save_grid(g, stem, title)
    C.render_map_png(g, traj_zup, stem.with_suffix(".png"), title, objects)
    C.log(f"  2D 맵 {g['grid'].shape[1]}x{g['grid'].shape[0]} @ {g['res']} m · "
          f"관측 {g['observations']} · 점유 {g['occupied']} · 자유 {g['free']} -> {stem}")


# 센서마다 믿을 수 있는 사거리가 다르다. D455 스테레오 depth 는 오차가 거리 제곱으로
# 커져 5 m 를 넘으면 벽이 두껍게 번지고, VLP-16 은 그 정도 거리는 정확하다.
RANGE = {"orb3": 5.0, "rtabmap": 5.0, "hdl": 15.0}

# hdl_graph_slam 이 자기 맵을 낼 때 쓰는 복셀 크기(map_cloud_resolution). 3D 맵을 공식
# 경로로 만들 때는 이 값을 따라간다 — 0.03 으로 더 잘게 쪼개봐야 키프레임 점군에는 그만한
# 정보가 없고, 공식 산출물(/hdl_graph_slam/map_points, save_map)과 달라지기만 한다.
HDL_MAP_CLOUD_RESOLUTION = 0.1


def backend_grid(backend: str, out: Path, data: Path, info: dict, res: float = 0.05,
                 max_range: float = 0.0, stride: int = 6, min_hits: int = 2) -> tuple:
    """백엔드 이름 하나로 관측 원본을 골라 격자를 만든다. 궤적도 z-up 으로 돌려 돌려준다."""
    rng = max_range or RANGE[backend]
    traj = out / backend / "trajectory.txt"
    _t, xyz, _q = C.read_tum(traj)
    zup = xyz @ C.map_rotation(backend).T
    if backend == "hdl":
        # Velodyne bag 재투영을 기본으로 쓴다. graph_dump 는 hdl 이 걸러낸 키프레임뿐이라
        # 관측이 훨씬 적고(185223 실측 201 vs 1462, 65 s 세션은 31개까지 떨어진다),
        # _stage 가 지워지면 사라져 세션마다 입력이 달라진다. 원본 스캔은 영구 보존된다.
        lidar = str(data / "lidar") if info.get("lidar") else ""
        if lidar and Path(lidar).is_dir():
            obs = velodyne_observations(Path(lidar), traj,
                                        (info.get("lidar") or {}).get("topic", "/velodyne_points"),
                                        stride=2, max_range=rng)
        else:
            C.log("  Velodyne bag 이 없어 graph_dump 키프레임으로 대체한다")
            obs = hdl_observations(out / "_stage" / "slam_hdl" / "graph_dump")
    else:
        K = info["SLAM"]["camera"]["K"]
        obs = depth_observations(data / "SLAM", traj, info["SLAM"]["depth_topic"],
                                 (K[0], K[4], K[2], K[5]), stride=stride, d_max=rng)
    return build(obs, zup, res=res, max_range=rng, min_hits=min_hits), zup


def backend_ply(backend: str, out: Path, data: Path, info: dict, voxel: float = 0.03,
                stride: int = 3) -> dict:
    """<출력>/<백엔드>/map_3d.ply — 유니티에 올릴 3D 점군.

    카메라 백엔드(ORB3/RTAB)는 dense 점군을 다시 만들지 않고 **그 백엔드의 궤적으로 bag 을
    재투영**한다. 점군과 궤적이 같은 실행에서 나오므로 정합이 보장되고 SLAM 재실행이 필요 없다.

    hdl 은 다르다. hdl_graph_slam 은 자기 맵 생성기(map_cloud_generator)를 갖고 있고,
    그 입력은 **prefiltering(거리 0.5~far, 0.1 복셀, outlier)을 통과한 키프레임 점군**을
    **그래프가 최적화한 pose(estimate)** 로 옮긴 것이다. bag 재투영은 그 필터와 키프레임
    선택을 전부 우회해 원시 스캔을 전량 쌓기 때문에(185223 실측 2924 장 vs 키프레임 203 개,
    0.1 복셀 환산 점유 467k vs 233k) 사람처럼 지나간 물체가 궤적을 따라 그대로 남는다.
    그래서 graph_dump 가 있으면 공식 경로를 쓰고, 없을 때만 bag 재투영으로 물러선다.
    """
    traj = out / backend / "trajectory.txt"
    if not traj.is_file():
        C.die(f"{traj} 가 없다 — {backend}.py 를 먼저 돌려라")
    if backend == "hdl":
        dump = out / "_stage" / "slam_hdl" / "graph_dump"
        if dump.is_dir():
            obs = hdl_observations(dump)
            voxel = max(voxel, HDL_MAP_CLOUD_RESOLUTION)
        else:
            C.log("  graph_dump 가 없어 Velodyne bag 을 재투영한다 — 공식 경로가 아니라서 "
                  "동적 객체와 근거리 잡점이 걸러지지 않는다")
            obs = velodyne_observations(Path(data / "lidar"), traj,
                                        (info.get("lidar") or {}).get("topic", "/velodyne_points"),
                                        stride=max(1, stride // 2))
    else:
        K = info["SLAM"]["camera"]["K"]
        obs = depth_observations(data / "SLAM", traj, info["SLAM"]["depth_topic"],
                                 (K[0], K[4], K[2], K[5]), stride=stride,
                                 px_step=3, max_pts=25000, d_max=RANGE[backend],
                                 color_topic=info["SLAM"].get("color_topic", ""),
                                 zup=False)
    return export_ply(obs, out / backend / "map_3d.ply", voxel=voxel)
