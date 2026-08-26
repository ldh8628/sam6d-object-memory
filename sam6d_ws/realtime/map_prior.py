#!/usr/bin/env python3
"""map_prior.py — ch2: SLAM Map 기준 객체 자세를 쌓아 두고, 가림·화면잘림으로 마스크가
나빠진 프레임에서 후보 자격을 되살린다.

ch1(마스크 비교)은 프레임 하나만 본다. 가림이나 화면 잘림이 있으면 정답 자세여도
실루엣이 안 맞으니 거절될 수 있다. 물체는 정지해 있으므로 `R_map_obj = R_map_cam·R_cam_obj`
가 상수다 — 이것이 프레임을 가로질러 수렴하면 등록해 두고, 그 뒤로는 등록 자세와 맞는
후보를 **마스크와 무관하게** 통과시킨다. 두 채널은 **합집합**이다(교집합이면 구제가 안 된다).

★ 자기강화 차단 — 이 파일에서 가장 중요한 규칙
  잘못 등록된 자세는 그 뒤로 틀린 후보를 영원히 들여보내는 고리가 된다. 그래서 투표를
  **쉬운 프레임에서만** 받는다: Stage V 판정이 PASS 이고, **승자가 ch1(마스크)으로 통과**
  했을 때만 표를 준다. ch2 로만 들어온 후보는 절대 ch2 에 투표하지 못한다.
  사전은 쉬운 프레임에서 배우고 어려운 프레임에서 쓴다.

★ 사전은 **자격만 주고 승자를 뽑지 않는다.** 승자는 여전히 생존 후보 중 텍스처 점수
  argmax 다. 그래서 잘못된 사전은 자격 범위를 넓힐 뿐 후보를 당선시키지 못한다.

⚠ 그래도 못 막는 것이 있다. 수렴 조건은 **무작위 오차를 거르지 계통 편향을 거르지 못한다.**
  milk 처럼 180° 앞뒤 뒤집힘이 프레임 다수이면 모드 다수결은 뒤집힌 자세를 등록한다.
  남은 완충은 좁은 admit_deg, 텍스처가 뽑는 승자, 그리고 강등 규칙뿐이다.
  감시 신호: 한 객체의 admit=="map" 비율이 100% 로 기어오르는데 승자의 cov 가 낮으면
  사전이 자기를 가둔 것이다.

알고리즘 출처(재구현이 아니라 이식):
  objectmemory_ws/object_memory/src/core/fusion.py
    align_to_symmetry(:82) · update_rotation_modes(:117) · mode_occupancy(:140)
  objectmemory_ws/object_memory/src/core/pose_buffer.py
    PoseBuffer(:81) · interpolate_pose(:53)
대칭축 표는 **새로 만들지 않는다** — verify_config.DEFAULT_VERIFY["symmetry_axes"] 가
이 저장소의 정본이고 model_utils._sym_group 이 이미 그것을 먹는다(자동 유도 금지:
소각 대칭이 곰·공룡에 가짜 대칭을 만든 전례가 있다).
"""
from __future__ import annotations

import bisect
import math

import numpy as np

try:
    from verify_config import DEFAULT_VERIFY
except ImportError:                                   # 단독 임포트 대비
    DEFAULT_VERIFY = {"symmetry_axes": {}, "sym_step_deg": 10}


# ----------------------------------------------------------------- 회전 유틸
def _axis_rotation(axis, deg):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    x, y, z = (float(c) / n for c in axis)
    a = math.radians(deg)
    c, s, k = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return np.array([[c + x * x * k, x * y * k - z * s, x * z * k + y * s],
                     [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
                     [z * x * k - y * s, z * y * k + x * s, c + z * z * k]])


_SYM_CACHE = {}


def symmetry_group(name, axes=None, step_deg=None):
    """그 물체의 동치 회전 목록. 대칭이 선언되지 않았으면 빈 튜플(=정렬 안 함)."""
    axes = DEFAULT_VERIFY.get("symmetry_axes", {}) if axes is None else axes
    step_deg = int(DEFAULT_VERIFY.get("sym_step_deg", 10) if step_deg is None else step_deg)
    if name not in axes:
        return ()
    key = (name, step_deg)
    if key not in _SYM_CACHE:
        ax = axes[name]
        _SYM_CACHE[key] = tuple(_axis_rotation(ax, d) for d in range(step_deg, 360, step_deg))
    return _SYM_CACHE[key]


def rel_angle_deg(A, B):
    """두 회전 사이의 각도. 자세는 물체→카메라이므로 동치는 R·S 규약이다."""
    tr = float(np.sum(A * B))                     # trace(AᵀB)
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) * 0.5))))


def align_to_symmetry(prev_R, meas_R, group):
    """meas_R 을 prev_R 에 가장 가까운 동치 자세로 바꾼다(대칭 없으면 그대로)."""
    if not len(group):
        return meas_R
    best, best_a = meas_R, rel_angle_deg(prev_R, meas_R)
    for S in group:
        cand = meas_R @ S
        a = rel_angle_deg(prev_R, cand)
        if a < best_a:
            best, best_a = cand, a
    return best


def sym_angle_deg(A, B, group):
    """대칭을 접은 최소 각도차."""
    if not len(group):
        return rel_angle_deg(A, B)
    return min([rel_angle_deg(A, B)] + [rel_angle_deg(A, B @ S) for S in group])


def _quat(R):
    """회전행렬 → 사원수 xyzw. 수치 안정을 위해 대각 최대 성분 기준으로 분기한다."""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                         (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2.0
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return q


def _rot(q):
    x, y, z, w = q / (np.linalg.norm(q) + 1e-12)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _nlerp(a, b, t):
    if float(np.dot(a, b)) < 0.0:
        b = -b
    q = a * (1.0 - t) + b * t
    return q / (np.linalg.norm(q) + 1e-12)


# ----------------------------------------------------------------- 포즈 버퍼
class PoseBuffer:
    """stamp(초) → T_map_cam(4x4). 없으면 **추측하지 않는다**.

    max_gap 보다 벌어진 구간은 보간하지 않는다 — 추적이 끊긴 구간을 조용히 메우면
    엉뚱한 자세로 지도에 올라간다.
    """

    def __init__(self, window_s=10.0, tol_s=0.05, max_gap_s=0.20):
        self.window, self.tol, self.max_gap = float(window_s), float(tol_s), float(max_gap_s)
        self._t, self._p, self._q = [], [], []

    def add(self, stamp_s, T):
        T = np.asarray(T, dtype=float)
        s = float(stamp_s)
        i = bisect.bisect_left(self._t, s)
        self._t.insert(i, s)
        self._p.insert(i, T[:3, 3].copy())
        self._q.insert(i, _quat(T[:3, :3]))
        cut = s - self.window
        while self._t and self._t[0] < cut:
            self._t.pop(0); self._p.pop(0); self._q.pop(0)

    def __len__(self):
        return len(self._t)

    def lookup(self, stamp_s):
        """capture stamp 기준 조회. 되돌림 (T 4x4 또는 None, 나이 ms 또는 None)."""
        if not self._t:
            return None, None
        s = float(stamp_s)
        i = bisect.bisect_left(self._t, s)
        if i == 0 or i == len(self._t):
            j = 0 if i == 0 else len(self._t) - 1
            if abs(self._t[j] - s) > self.tol:
                return None, None
            return self._compose(self._p[j], self._q[j]), abs(self._t[j] - s) * 1e3
        t0, t1 = self._t[i - 1], self._t[i]
        if (t1 - t0) > self.max_gap:
            j = i - 1 if (s - t0) <= (t1 - s) else i
            if abs(self._t[j] - s) > self.tol:
                return None, None
            return self._compose(self._p[j], self._q[j]), abs(self._t[j] - s) * 1e3
        r = (s - t0) / max(t1 - t0, 1e-9)
        p = self._p[i - 1] * (1 - r) + self._p[i] * r
        q = _nlerp(self._q[i - 1], self._q[i], r)
        return self._compose(p, q), min(s - t0, t1 - s) * 1e3

    @staticmethod
    def _compose(p, q):
        T = np.eye(4)
        T[:3, :3] = _rot(q)
        T[:3, 3] = p
        return T


# ----------------------------------------------------------------- 사전 본체
class _Inst:
    __slots__ = ("P", "n", "modes", "votes", "registered", "R_reg", "recent")

    def __init__(self, P, R):
        self.P = np.asarray(P, dtype=float)
        self.n = 1.0
        self.modes = [(_quat(R), 1.0)]
        self.votes = 1
        self.registered = False
        self.R_reg = None
        self.recent = []          # 최근 투표가 등록 자세와 맞았는지 (강등 판정용)


class MapPrior:
    """이름별 다중 인스턴스로 map 기준 자세를 쌓고, 등록되면 카메라계 참조를 돌려준다."""

    def __init__(self, cfg=None, log=print):
        c = dict(cfg or {})
        self.log = log
        self.enabled = bool(c.get("enabled", False))
        self.admit_deg = float(c.get("admit_deg", 20.0))
        self.tol_deg = float(c.get("tol_deg", 30.0))
        self.min_votes = int(c.get("min_votes", 20))
        self.min_share = float(c.get("min_share", 0.6))
        self.freeze = bool(c.get("freeze", False))
        self.demote_window = int(c.get("demote_window", 40))
        self.demote_share = float(c.get("demote_share", 0.35))
        self.pos_tol_m = float(c.get("pos_tol_m", 0.30))
        self.max_inst = dict(c.get("multi_instance", {}) or {})
        self.vote_min_px = int(c.get("vote_min_px", 40))
        self.vote_need_pass = bool(c.get("vote_need_pass", True))
        self.vote_need_mask = bool(c.get("vote_need_mask", True))
        self.reset_on_jump_m = float(c.get("reset_on_jump_m", 1.0))
        self.max_modes = 8
        self._inst = {}                    # name -> [_Inst, ...]
        self._last_cam = None
        self.reset()

    # ------------------------------------------------------------ 상태
    def reset(self, why="시작"):
        if self._inst:
            self.log(f"[map_prior] 초기화({why})")
        self._inst = {}
        self._last_cam = None

    def note_camera(self, T_map_cam):
        """SLAM 재위치추정으로 카메라가 튀면 지도 좌표가 통째로 무의미해진다."""
        if T_map_cam is None:
            return
        p = np.asarray(T_map_cam)[:3, 3]
        if self._last_cam is not None:
            d = float(np.linalg.norm(p - self._last_cam))
            if d > self.reset_on_jump_m:
                self.reset(f"SLAM 포즈 점프 {d:.2f} m")
        self._last_cam = p.copy()

    def _sym(self, name):
        return symmetry_group(name)

    # ------------------------------------------------------------ 조회
    def reference(self, name, T_map_cam, box=None, K=None):
        """이 검출에 쓸 카메라계 참조 자세. 되돌림 (R_ref 3x3 또는 None, 진단 dict).

        PEM 을 부르기 **전**이라 t_cam_obj 가 아직 없다. 그래서 지도 위치로 군집할 수
        없고, 등록된 인스턴스를 현재 카메라로 **역투영**해 검출 박스 안에 떨어지는지로
        연관한다. 이름당 등록 인스턴스가 하나뿐이면 무조건 연관한다.
        """
        if not self.enabled or T_map_cam is None:
            return None, {}
        reg = [(i, o) for i, o in enumerate(self._inst.get(name, [])) if o.registered]
        if not reg:
            return None, {}
        R_mc = np.asarray(T_map_cam)[:3, :3]
        P_mc = np.asarray(T_map_cam)[:3, 3]
        pick = None
        if len(reg) == 1:
            pick = reg[0]
        elif box is not None and K is not None:
            x1, y1, x2, y2 = [float(v) for v in box]
            best = None
            for i, o in reg:
                pc = R_mc.T @ (o.P - P_mc)
                if pc[2] <= 1e-3:
                    continue
                u = K[0, 0] * pc[0] / pc[2] + K[0, 2]
                v = K[1, 1] * pc[1] / pc[2] + K[1, 2]
                if x1 <= u <= x2 and y1 <= v <= y2:
                    d = (u - 0.5 * (x1 + x2)) ** 2 + (v - 0.5 * (y1 + y2)) ** 2
                    if best is None or d < best[0]:
                        best = (d, (i, o))
            pick = best[1] if best else None
        if pick is None:
            return None, {}
        i, o = pick
        R_ref = R_mc.T @ o.R_reg                       # map -> camera
        return R_ref, {"inst": i, "votes": o.votes, "share": self._share(o)}

    def _share(self, o):
        tot = sum(w for _q, w in o.modes)
        return round(float(o.modes[0][1] / tot) if tot > 0 else 0.0, 3)

    # ------------------------------------------------------------ 투표
    def vote(self, name, R_cam_obj, t_cam_obj_m, T_map_cam, *,
             bbox_px=None, verdict=None, admitted_by=None):
        """PEM 결과 하나를 지도에 올린다. **자격 미달이면 조용히 버린다.**

        되돌림: 갱신된 진단 dict (표를 안 줬으면 사유 포함).
        """
        if not self.enabled or T_map_cam is None:
            return {"voted": False, "why": "no_pose"}
        if self.vote_need_pass and verdict is not None and verdict != "PASS":
            return {"voted": False, "why": "not_pass"}
        if self.vote_need_mask and admitted_by is not None and admitted_by not in ("mask", "both"):
            # ★ 고리 차단기 — ch2 로만 들어온 후보는 ch2 에 투표하지 못한다.
            return {"voted": False, "why": "not_mask_admitted"}
        if bbox_px is not None and bbox_px < self.vote_min_px:
            return {"voted": False, "why": "too_small"}

        T = np.asarray(T_map_cam, dtype=float)
        R_mc, P_mc = T[:3, :3], T[:3, 3]
        R_map = R_mc @ np.asarray(R_cam_obj, dtype=float)
        P_map = R_mc @ np.asarray(t_cam_obj_m, dtype=float) + P_mc

        lst = self._inst.setdefault(name, [])
        cap = int(self.max_inst.get(name, 1))
        j = self._associate(lst, P_map, cap)
        if j is None:
            lst.append(_Inst(P_map, R_map))
            return {"voted": True, "inst": len(lst) - 1, "new": True}

        o = lst[j]
        o.P = o.P * (o.n / (o.n + 1.0)) + P_map / (o.n + 1.0)     # 위치는 평균(단봉)
        o.n += 1.0
        o.votes += 1
        self._update_modes(o, R_map, name)                        # 회전은 모드 다수결
        self._register(o, name)
        return {"voted": True, "inst": j, "votes": o.votes, "share": self._share(o),
                "registered": o.registered}

    def _associate(self, lst, P_map, cap):
        """가장 가까운 인스턴스. 없고 정원이 남으면 None(=새로 만든다)."""
        if not lst:
            return None
        d = [float(np.linalg.norm(o.P - P_map)) for o in lst]
        j = int(np.argmin(d))
        if d[j] <= self.pos_tol_m:
            return j
        if len(lst) < cap:
            return None
        return j                    # 정원이 찼으면 가장 가까운 곳에 흡수시킨다

    def _update_modes(self, o, R_map, name):
        grp = self._sym(name)
        bi, ba, bR = -1, self.tol_deg, None
        for i, (q, _w) in enumerate(o.modes):
            rep = _rot(q)
            cand = align_to_symmetry(rep, R_map, grp)
            a = rel_angle_deg(rep, cand)
            if a < ba:
                bi, ba, bR = i, a, cand
        if bi < 0:
            o.modes.append((_quat(R_map), 1.0))
        else:
            q, w = o.modes[bi]
            o.modes[bi] = (_nlerp(q, _quat(bR), 1.0 / (w + 1.0)), w + 1.0)
        o.modes.sort(key=lambda m: -m[1])
        del o.modes[self.max_modes:]

        if o.registered:
            ok = sym_angle_deg(o.R_reg, R_map, grp) <= self.tol_deg
            o.recent.append(bool(ok))
            del o.recent[:-self.demote_window]

    def _register(self, o, name):
        share = self._share(o)
        if not o.registered:
            if o.votes >= self.min_votes and share >= self.min_share:
                o.registered = True
                o.R_reg = _rot(o.modes[0][0])
                o.recent = []
                self.log(f"[map_prior] 등록 {name} 표{o.votes} 점유율{share:.2f}")
            return
        # 갱신: 최빈 모드가 바뀌면 등록 자세도 따라간다(freeze=false 기본).
        if not self.freeze:
            o.R_reg = _rot(o.modes[0][0])
        # 강등: 최근 독립 투표가 등록 자세와 계속 어긋나면 등록을 푼다.
        if len(o.recent) >= self.demote_window:
            agree = sum(o.recent) / float(len(o.recent))
            if agree < self.demote_share:
                o.registered, o.R_reg, o.recent = False, None, []
                self.log(f"[map_prior] DEMOTE {name} 최근일치 {agree:.2f}")


# ----------------------------------------------------------------- 포즈 공급
def load_extrinsic(path):
    """4x4 T_camSLAM_camSAM (.npy). 없으면 None."""
    if not path:
        return None
    X = np.load(str(path))
    X = np.asarray(X, dtype=float).reshape(4, 4)
    return X


def load_trajectory(path):
    """TUM 궤적(`t tx ty tz qx qy qz qw`) → PoseBuffer. bag 검증용."""
    pb = PoseBuffer(window_s=1e9)
    n = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.split()
            if len(v) < 8:
                continue
            T = np.eye(4)
            T[:3, :3] = _rot(np.array([float(v[4]), float(v[5]), float(v[6]), float(v[7])]))
            T[:3, 3] = [float(v[1]), float(v[2]), float(v[3])]
            pb.add(float(v[0]), T)
            n += 1
    return pb, n


class PoseSource:
    """설정에 따라 T_map_camSAM 을 만든다. 두 경로를 **동시에** 들고 서로를 감시한다.

    slam_extrinsic   : T_map_camSAM = T_map_camSLAM · X   (SLAM 카메라 + 캘리브)
    self_localization: SAM 카메라를 기존 맵에 only-localization 한 자세 그대로 (X = 항등)
    both             : self 가 유효하면 그것, 아니면 slam·X 로 폴백.
                       **두 값의 차이를 매 프레임 남긴다** — X 캘리브 상시 감시가 공짜다.
    trajectory_file  : TUM 궤적 파일(ROS 없이 ch2 전체를 재현할 수 있다)
    none             : ch2 비활성
    """

    def __init__(self, cfg=None, log=print):
        c = dict(cfg or {})
        self.log = log
        self.mode = str(c.get("pose_source", "none"))
        self.X = load_extrinsic(c.get("extrinsic"))
        w = float(c.get("buffer_window_s", 10.0))
        tol = float(c.get("pose_tol_s", 0.05))
        gap = float(c.get("max_gap_s", 0.20))
        self.slam = PoseBuffer(w, tol, gap)
        self.own = PoseBuffer(w, tol, gap)
        self.traj = None
        if self.mode == "trajectory_file" and c.get("trajectory"):
            self.traj, n = load_trajectory(c["trajectory"])
            self.log(f"[map_prior] 궤적 {n}개 적재: {c['trajectory']}")
        if self.mode in ("slam_extrinsic", "both") and self.X is None:
            self.log("[map_prior] ⚠ extrinsic 이 없다 — slam 경로는 쓰지 않는다"
                     " (self_localization 만 유효)")

    def add_slam(self, stamp_s, T):
        self.slam.add(stamp_s, T)

    def add_self(self, stamp_s, T):
        self.own.add(stamp_s, T)

    def lookup(self, stamp_s):
        """되돌림 (T_map_camSAM 또는 None, 진단 dict)."""
        if self.mode == "none":
            return None, {}
        if self.mode == "trajectory_file":
            if self.traj is None:
                return None, {}
            T, age = self.traj.lookup(stamp_s)
            return T, ({"src": "traj", "pose_age_ms": round(age, 1)} if T is not None else {})

        Ts, age_s = (self.own.lookup(stamp_s) if self.mode in ("self_localization", "both")
                     else (None, None))
        Tx, age_x = (None, None)
        if self.mode in ("slam_extrinsic", "both") and self.X is not None:
            Tm, age_x = self.slam.lookup(stamp_s)
            if Tm is not None:
                Tx = Tm @ self.X

        dg = {}
        if Ts is not None and Tx is not None:
            # 같은 순간의 두 추정이 얼마나 벌어지는가 = X 캘리브가 얼마나 맞는가.
            dg["dxy_extrinsic_m"] = round(float(np.linalg.norm(Ts[:3, 3] - Tx[:3, 3])), 4)
            dg["drot_extrinsic_deg"] = round(rel_angle_deg(Ts[:3, :3], Tx[:3, :3]), 2)
        if Ts is not None:
            return Ts, {"src": "self_loc", "pose_age_ms": round(age_s, 1), **dg}
        if Tx is not None:
            return Tx, {"src": "slam_x", "pose_age_ms": round(age_x, 1), **dg}
        return None, {}
