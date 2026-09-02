#!/usr/bin/env python3
"""객체 lifecycle — 있던 객체가 '사라졌다/움직였다'를 갱신하는 기능만 모은 모듈.

objectmemory_ws/object_memory 의 core + pipeline 에서 **판정에 실제로 쓰이는 함수만**
가져와 표준 라이브러리만으로 자립시킨 것이다. ROS 노드·시각화·어댑터 트리는 가져오지
않는다. 입력은 integration 이 이미 만들어 둔 산출물뿐이다:

    <출력>/sam/objects.json    SAM-6D 프레임별 (물체, 6D 포즈, 점수)   ← 검출
    <출력>/<백엔드>/trajectory.txt  SLAM 궤적 (TUM)                    ← 카메라 포즈
    <데이터>/calib/*.npy        X = T_camSLAM_camSAM (두 카메라 리그)
    <데이터>/info.json          SAM 카메라 K + 영상 크기 (P_D 시야 모델)
    <데이터>/sam                bag (프레임 인덱스 -> 타임스탬프)

포팅 범위(= '사라짐'과 '이동'을 모두 판정하는 데 필요한 최소 집합):

    존재확률 r      검출/미검출마다 베이즈 갱신           update_existence_*
    시야 P_D        랜드마크를 영상에 재투영해 시야 판정  expected_p_d
    품질 가중       검출 하나의 증거량(logLR)             detection_log_lr
    연관            같은 클래스의 가장 가까운 인스턴스     _assign_pairs
    이동            연관되면 포즈를 러닝 평균으로 갱신     fuse_pose
    분리            게이트 밖이면 새 인스턴스 spawn        _run_initiator
    사라짐          미검출 r 감쇠 -> lost/remembered/삭제  _run_deleter

핵심 규칙 두 가지만 기억하면 된다.

  1. 시야 밖(P_D = 0)이면 미검출에 벌점이 없다. 그래서 다른 곳을 보고 있는 동안에는
     객체가 절대 죽지 않고, **그 자리를 실제로 바라보는 동안에만** 사라짐 판정이 진행된다.
  2. 관측이 min_obs_longterm 이상 쌓인 객체는 r 이 바닥나도 삭제하지 않고 remembered 로
     보존한다. remembered 는 이후 감쇠 대상에서 제외된다(장기 기억).

동작은 objectmemory_ws 와 일치시키는 것을 기준으로 삼는다(parity). 진단용으로
프레임 스냅샷에 r 과 P_D 를 추가로 남기는 것만 다르다 — 판정에는 영향이 없다.
"""
from __future__ import annotations

import ast
import bisect
import glob
import json
import math
import os
import sqlite3
import struct
from dataclasses import dataclass, field, fields as dataclass_fields
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# 기본값 — objectmemory_ws 와 동일한 값. integration 의 fuse 단계는 여기에 더해
# assoc_trans_gate_m=0.20, assoc_rot_gate_deg=-1(끔), quality_weighting=True 를 쓴다.
# ---------------------------------------------------------------------------
PROMOTE_HITS = 2
ASSOC_TRANS_GATE_M = 0.20
ASSOC_ROT_GATE_DEG = -1.0        # <0 이면 회전 게이트를 쓰지 않는다 (PEM 회전은 잡음이 크다)
SCORE_FLOOR = 0.0
TENTATIVE_GATE_MULT = 3.0
PD_BASE = 0.5                    # 시야 안 기대 검출률 (9-bag 실측 recall 에 맞춘 값)
FOV_MARGIN = 0.05                # 영상 가장자리 5% 는 시야 밖으로 본다
FALLBACK_HALF_ANGLE_DEG = 35.0   # K 가 없을 때의 원뿔 근사
CLUTTER_RATIO = 0.1
R_INIT = 0.7
R_PROMOTE = 0.6
R_LOST = 0.3
R_RETIRE = 0.05
TENTATIVE_MAX_AGE = 100
MIN_OBS_LONGTERM = 5
R_PRIOR = 0.15
R_MAX = 0.99
R_MIN = 1e-6
TIME_TOLERANCE = 0.05

Matrix3 = tuple
Matrix4 = tuple


# ===========================================================================
# 1. 강체 변환 (표준 라이브러리만)
# ===========================================================================
def make_transform(rotation, translation) -> Matrix4:
    r, t = rotation, translation
    return (
        (float(r[0][0]), float(r[0][1]), float(r[0][2]), float(t[0])),
        (float(r[1][0]), float(r[1][1]), float(r[1][2]), float(t[1])),
        (float(r[2][0]), float(r[2][1]), float(r[2][2]), float(t[2])),
        (0.0, 0.0, 0.0, 1.0),
    )


def identity_transform() -> Matrix4:
    return ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def compose_transform(a: Matrix4, b: Matrix4) -> Matrix4:
    """a * b (b 를 먼저 적용)."""
    return tuple(
        tuple(a[i][0] * b[0][j] + a[i][1] * b[1][j]
              + a[i][2] * b[2][j] + a[i][3] * b[3][j] for j in range(4))
        for i in range(4)
    )


def invert_transform(T: Matrix4) -> Matrix4:
    rt = ((T[0][0], T[1][0], T[2][0]),
          (T[0][1], T[1][1], T[2][1]),
          (T[0][2], T[1][2], T[2][2]))
    t = (T[0][3], T[1][3], T[2][3])
    nt = tuple(-(rt[i][0] * t[0] + rt[i][1] * t[1] + rt[i][2] * t[2])
               for i in range(3))
    return make_transform(rt, nt)


def transform_distance_translation(a: Matrix4, b: Matrix4) -> float:
    return math.sqrt(sum((a[i][3] - b[i][3]) ** 2 for i in range(3)))


def transform_distance_rotation_deg(a: Matrix4, b: Matrix4) -> float:
    """R_a^T R_b 의 측지 각도 [deg]."""
    trace = sum(sum(a[k][i] * b[k][i] for k in range(3)) for i in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def quat_to_rotation(qx: float, qy: float, qz: float, qw: float) -> Matrix3:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        raise ValueError("zero-norm quaternion")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)),
        (2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)),
        (2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)),
    )


def make_transform_from_quat(translation, quat) -> Matrix4:
    return make_transform(quat_to_rotation(*quat), translation)


def rotation_to_quat(rotation: Matrix3):
    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw, qx, qy, qz = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw, qx, qy, qz = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw, qx, qy, qz = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw, qx, qy, qz = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        raise ValueError("degenerate rotation matrix")
    return (qx / n, qy / n, qz / n, qw / n)


def quat_nlerp(qa, qb, w: float):
    """qb 를 qa 반구로 뒤집은 뒤 정규화 선형보간 (w = qb 의 가중치)."""
    if sum(qa[i] * qb[i] for i in range(4)) < 0.0:
        qb = tuple(-v for v in qb)
    q = tuple(qa[i] * (1.0 - w) + qb[i] * w for i in range(4))
    n = math.sqrt(sum(c * c for c in q))
    return qa if n == 0.0 else tuple(c / n for c in q)


def compute_T_map_obj(T_map_cam: Matrix4, T_cam_obj: Matrix4) -> Matrix4:
    """T_map_obj = T_map_cam * T_cam_obj."""
    return compose_transform(T_map_cam, T_cam_obj)


# ===========================================================================
# 2. 포즈 융합 — '이동'을 반영하는 부분
# ===========================================================================
# --- 대칭 선언 -------------------------------------------------------------
# 물체 좌표계에서 **각도가 원리적으로 존재하지 않는 회전축**만 적는다. 여기 적힌 축으로
# 돌린 자세는 서로 다른 답이 아니라 같은 답이므로, 평균 내기 전에 한쪽으로 맞춘다.
# 그러지 않으면 nlerp 가 0° 와 120° 를 섞어 **한 번도 관측된 적 없는 중간 자세**를 만든다
# (실측: 우유팩 융합 결과가 모든 실제 관측과 24° 떨어진 유령 자세였다).
#
# 판정 근거 = 모델점을 그 축으로 돌렸을 때의 자기정합 오차(지름 대비, 10° 간격 전 각도):
#   Sikhye 0.63% · Sauce 1.54% · Mugcup 2.22%  → 회전체로 인정
#   (대조군) choco 8.53% → 인정 안 함. milk 1.93% 이지만 정사각 단면이라 연속이 아닌
#   90° 배수 대칭이고, 면마다 인쇄가 달라 원리적으로는 구별 가능하므로 **일부러 뺐다**.
#   인형류는 애초에 어떤 축으로도 자기정합이 안 된다.
SYMMETRY_ENABLED = True
SYMMETRY_STEP_DEG = 10
SYMMETRY_AXES = {                      # CAD 좌표계 기준 회전축(정규화)
    "Sikhye_high": (0.0, 0.0, 1.0),
    "Sauce_high": (0.0, 0.0, 1.0),
    "Mugcup_high": (0.1057, 0.0337, 0.9938),
}
_SYM_CACHE: dict = {}


def _axis_rotation(axis, deg: float):
    """축-각 회전행렬 (3x3 튜플)."""
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    x, y, z = (c / n for c in axis)
    a = math.radians(deg)
    c, s, k = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return ((c + x * x * k, x * y * k - z * s, x * z * k + y * s),
            (y * x * k + z * s, c + y * y * k, y * z * k - x * s),
            (z * x * k - y * s, z * y * k + x * s, c + z * z * k))


def symmetry_group(object_name: str):
    """그 물체의 동치 회전 목록. 대칭이 없으면 빈 목록(=정렬 안 함)."""
    if not SYMMETRY_ENABLED or object_name not in SYMMETRY_AXES:
        return ()
    if object_name not in _SYM_CACHE:
        ax = SYMMETRY_AXES[object_name]
        _SYM_CACHE[object_name] = tuple(
            _axis_rotation(ax, d) for d in range(SYMMETRY_STEP_DEG, 360, SYMMETRY_STEP_DEG))
    return _SYM_CACHE[object_name]


def _mat3_mul(A, B):
    return tuple(tuple(sum(A[r][k] * B[k][c] for k in range(3)) for c in range(3)) for r in range(3))


def _rel_angle_deg(A, B):
    """두 회전 사이의 각도(도). trace(AᵀB) 로 계산한다."""
    tr = sum(A[k][0] * B[k][0] + A[k][1] * B[k][1] + A[k][2] * B[k][2] for k in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


def align_to_symmetry(prev_R, meas_R, object_name: str):
    """meas_R 을 prev_R 에 가장 가까운 동치 자세로 바꾼다(대칭 없으면 그대로)."""
    group = symmetry_group(object_name)
    if not group:
        return meas_R
    best, best_a = meas_R, _rel_angle_deg(prev_R, meas_R)
    for S in group:
        cand = _mat3_mul(meas_R, S)
        a = _rel_angle_deg(prev_R, cand)
        if a < best_a:
            best, best_a = cand, a
    return best


# --- 모드 다수결 회전 융합 -------------------------------------------------
# SAM-6D 는 프레임마다 자세를 독립적으로 추정하고, 그 결과가 **이봉/다봉**이다
# (같은 면을 보고도 180° 뒤집힌 답이 섞여 나온다). 러닝 평균(nlerp)은 두 봉우리를
# 뭉개서 **한 번도 관측된 적 없는 중간 자세**를 만든다 — 이것이 지도가 흔들리는
# 직접 원인이었다. 그래서 회전만은 평균 대신 **가장 표가 많은 자세(모드)**를 쓴다.
#
# 실측(운영 무변경 재생, 4 bag): 지도 배회 1470° → 714° (-51%),
#   milk 유령 자세 11° → 1°, 관측 일치율 6% → 51%.
#   비대칭 객체에도 듣는다(Dinosaur -84%, Bear -47%) — 대칭 선언(-9%)보다 크다.
#   부산물로 모드 점유율이 그대로 그 자세의 신뢰도가 된다.
#
# 구현은 스트리밍 군집이다. 프레임마다 과거 전체를 다시 보면 O(n²) 이라 못 쓴다.
# 대표 자세 목록을 들고 다니며 들어온 측정을 가장 가까운 대표에 붙이고, 어디에도
# 안 붙으면 새 봉우리를 연다. 대칭이 선언된 물체는 붙이기 전에 동치 자세로 돌린다.
MODE_FUSION_ENABLED = True
MODE_FUSION_TOL_DEG = 30.0     # 이 안이면 같은 봉우리
MODE_FUSION_MAX_MODES = 8      # 표가 적은 봉우리는 버린다


def update_rotation_modes(modes, meas_R, object_name: str | None = None):
    """회전 모드 목록을 갱신하고 (갱신된 목록, 대표 회전) 을 돌려준다.

    modes 는 [(quat_xyzw, 표), ...] 로 표가 많은 순이다. 판정에 쓰이는 것은
    맨 앞 원소뿐이고, 나머지는 점유율(=신뢰도) 계산과 봉우리 전환을 위해 남긴다.
    """
    best_i, best_a, best_R = -1, MODE_FUSION_TOL_DEG, None
    for i, (q, _w) in enumerate(modes):
        rep = quat_to_rotation(*q)
        cand = align_to_symmetry(rep, meas_R, object_name) if object_name else meas_R
        a = _rel_angle_deg(rep, cand)
        if a < best_a:
            best_i, best_a, best_R = i, a, cand
    if best_i < 0:
        modes.append((rotation_to_quat(meas_R), 1.0))
    else:
        q, w = modes[best_i]
        modes[best_i] = (quat_nlerp(q, rotation_to_quat(best_R), 1.0 / (w + 1.0)), w + 1.0)
    modes.sort(key=lambda m: -m[1])
    del modes[MODE_FUSION_MAX_MODES:]
    return modes, quat_to_rotation(*modes[0][0])


def mode_occupancy(modes) -> float:
    """으뜸 모드의 표 점유율. 그 자세를 얼마나 믿을 수 있는지의 척도."""
    tot = sum(w for _q, w in modes)
    return (modes[0][1] / tot) if tot > 0 else 0.0


def fuse_pose(prev_T: Matrix4, n_prev: int, meas_T: Matrix4,
              object_name: str | None = None, rot_modes=None) -> Matrix4:
    """가중 1/(n+1) 의 러닝 평균. n_prev=0 이면 측정값이 그대로 씨앗이 된다.

    최신값 덮어쓰기가 아니라 평균이므로 SAM-6D 의 프레임별 지터는 눌리고, 게이트를
    통과한 진짜 이동만 서서히 반영된다. 큰 이상치는 애초에 연관 게이트가 막는다.

    `object_name` 이 SYMMETRY_AXES 에 있으면 평균 내기 전에 측정값을 현재 추정과
    같은 동치 자세로 돌려 놓는다(위 '대칭 선언' 참고). 표에 없는 물체는 영향이 없다.
    """
    rot = lambda T: tuple(tuple(T[r][c] for c in range(3)) for r in range(3))  # noqa: E731
    if rot_modes is not None and MODE_FUSION_ENABLED:
        # 회전은 모드 다수결, 위치는 지금까지대로 러닝 평균.
        # 씨앗(n_prev=0)일 때도 모드 목록을 열어 둬야 두 번째 측정이 붙을 곳이 생긴다.
        _m, mode_R = update_rotation_modes(rot_modes, rot(meas_T), object_name)
        if n_prev <= 0:
            t = tuple(meas_T[i][3] for i in range(3))
        else:
            w = 1.0 / (n_prev + 1)
            t = tuple(prev_T[i][3] * (1.0 - w) + meas_T[i][3] * w for i in range(3))
        return make_transform(mode_R, t)
    if n_prev <= 0:
        return meas_T
    w = 1.0 / (n_prev + 1)
    t = tuple(prev_T[i][3] * (1.0 - w) + meas_T[i][3] * w for i in range(3))
    meas_R = align_to_symmetry(rot(prev_T), rot(meas_T), object_name) if object_name else rot(meas_T)
    q = quat_nlerp(rotation_to_quat(rot(prev_T)), rotation_to_quat(meas_R), w)
    return make_transform_from_quat(t, q)


# ===========================================================================
# 3. 존재확률 r (베르누이 트랙 존재) — '사라짐'의 근거
# ===========================================================================
def update_existence_loglr(r: float, log_lr: float) -> float:
    """검출 수락: logit(r') = logit(r) + logLR."""
    r = min(max(r, R_MIN), R_MAX)
    return min(max(1.0 / (1.0 + math.exp(-(math.log(r / (1.0 - r)) + log_lr))), 0.0), R_MAX)


def update_existence_missed(r: float, p_d: float) -> float:
    """미검출: r' = r(1-P_D) / (1 - r·P_D). P_D = 0 이면 r 그대로(시야 밖)."""
    r = min(max(r, 0.0), R_MAX)
    p_d = min(max(p_d, 0.0), 1.0)
    return r * (1.0 - p_d) / (1.0 - r * p_d)


def update_existence_detected(r: float, p_d: float, clutter_ratio: float) -> float:
    """품질 가중을 쓰지 않을 때의 검출 갱신 (legacy 경로)."""
    r = min(max(r, 0.0), R_MAX)
    p_d = min(max(p_d, 0.0), 1.0)
    num = r * p_d
    return min(num / (num + (1.0 - r) * max(clutter_ratio, 1e-9)), R_MAX)


# ===========================================================================
# 4. 시야 P_D — '그 자리를 실제로 바라보고 있는가'
# ===========================================================================
def expected_p_d(
    T_map_cam: Matrix4,
    T_map_obj: Matrix4,
    cam_K: Optional[Sequence[Sequence[float]]] = None,
    img_size: Optional[Tuple[int, int]] = None,
    pd_base: float = PD_BASE,
    margin: float = FOV_MARGIN,
    fallback_half_angle_deg: float = FALLBACK_HALF_ANGLE_DEG,
) -> float:
    """랜드마크를 이 프레임의 카메라에 재투영해 기대 검출확률을 낸다 (시야 밖이면 0)."""
    dx = T_map_obj[0][3] - T_map_cam[0][3]
    dy = T_map_obj[1][3] - T_map_cam[1][3]
    dz = T_map_obj[2][3] - T_map_cam[2][3]
    x = T_map_cam[0][0] * dx + T_map_cam[1][0] * dy + T_map_cam[2][0] * dz
    y = T_map_cam[0][1] * dx + T_map_cam[1][1] * dy + T_map_cam[2][1] * dz
    z = T_map_cam[0][2] * dx + T_map_cam[1][2] * dy + T_map_cam[2][2] * dz
    if z <= 0.0:
        return 0.0
    if cam_K is not None and img_size is not None:
        w, h = img_size
        u = cam_K[0][0] * x / z + cam_K[0][2]
        v = cam_K[1][1] * y / z + cam_K[1][2]
        mu, mv = w * margin, h * margin
        return pd_base if (mu <= u <= w - mu and mv <= v <= h - mv) else 0.0
    return pd_base if math.degrees(math.atan2(math.hypot(x, y), z)) <= fallback_half_angle_deg else 0.0


# ===========================================================================
# 5. 검출 하나의 증거량 (품질 가중 logLR)
# ===========================================================================
Z_MIN_M = 0.05
Z_MAX_M = 5.0
W_SCORE = 2.0
W_DEPTH = 4.0
W_RESID = 0.0                 # 게이트 안 잔차는 추정 지터라 존재 증거가 아니다 (기본 끔)
LOGLR_CLAMP = (-4.0, 2.5)


@dataclass(frozen=True)
class QualityWeights:
    w_score: float = W_SCORE
    w_depth: float = W_DEPTH
    w_resid: float = W_RESID
    z_min: float = Z_MIN_M
    z_max: float = Z_MAX_M
    clamp_lo: float = LOGLR_CLAMP[0]
    clamp_hi: float = LOGLR_CLAMP[1]

    @classmethod
    def legacy(cls) -> "QualityWeights":
        return cls(w_score=0.0, w_depth=0.0, w_resid=0.0)


def detection_log_lr(
    score: float,
    z_cam: float,
    resid_trans: Optional[float],
    gate: float,
    p_d: float,
    clutter_ratio: float,
    weights: QualityWeights = QualityWeights(),
) -> float:
    """logLR = log(P_D/c) + w_score·(score-0.5) + w_depth·(깊이 이상) + w_resid·(잔차)."""
    base = math.log(min(max(p_d, 1e-9), 1.0) / max(clutter_ratio, 1e-9))
    phi_score = min(max(score, 0.0), 1.0) - 0.5
    phi_depth = -1.0 if (z_cam < weights.z_min or z_cam > weights.z_max) else 0.0
    phi_resid = (0.0 if (resid_trans is None or gate <= 0.0)
                 else -min(max(resid_trans / gate, 0.0), 1.0))
    log_lr = (base + weights.w_score * phi_score
              + weights.w_depth * phi_depth + weights.w_resid * phi_resid)
    return min(max(log_lr, weights.clamp_lo), weights.clamp_hi)


# ===========================================================================
# 6. 상태 기계
# ===========================================================================
class ObjectStatus(str, Enum):
    tentative = "tentative"
    active = "active"
    lost = "lost"
    remembered = "remembered"
    merged = "merged"
    deleted = "deleted"


@dataclass
class StateTransition:
    from_status: ObjectStatus
    to_status: ObjectStatus
    reason: str
    stamp: Optional[float] = None
    metadata: dict = field(default_factory=dict)


_ALLOWED = {
    (ObjectStatus.tentative, ObjectStatus.active),
    (ObjectStatus.tentative, ObjectStatus.deleted),
    (ObjectStatus.active, ObjectStatus.lost),
    (ObjectStatus.lost, ObjectStatus.active),
    (ObjectStatus.lost, ObjectStatus.remembered),
    (ObjectStatus.remembered, ObjectStatus.active),
    (ObjectStatus.active, ObjectStatus.merged),
    (ObjectStatus.remembered, ObjectStatus.merged),
}


def coerce_status(status) -> ObjectStatus:
    if isinstance(status, ObjectStatus):
        return status
    if isinstance(status, str):
        return ObjectStatus(status)
    raise ValueError(f"cannot coerce {status!r} to ObjectStatus")


def is_transition_allowed(from_status, to_status) -> bool:
    src, dst = coerce_status(from_status), coerce_status(to_status)
    return True if dst is ObjectStatus.deleted else (src, dst) in _ALLOWED


def build_transition(from_status, to_status, reason: str, stamp=None,
                     metadata=None) -> StateTransition:
    src, dst = coerce_status(from_status), coerce_status(to_status)
    if not reason or not str(reason).strip():
        raise ValueError("transition reason must be non-empty")
    if not is_transition_allowed(src, dst):
        raise ValueError(f"transition not allowed: {src.value} -> {dst.value}")
    return StateTransition(src, dst, str(reason), stamp,
                           dict(metadata) if metadata else {})


# ===========================================================================
# 7. 자료형
# ===========================================================================
@dataclass
class SlamCameraPose:
    stamp: float
    T_map_cam: Matrix4
    frame_id: str = "map"
    child_frame_id: str = "camera"
    source_slam_id: str = "orbslam3"


@dataclass
class Sam6DDetection:
    stamp: float
    detection_id: int
    object_name: str
    T_cam_obj: Matrix4
    score: float
    frame_id: str = "camera"
    bbox: Optional[tuple] = None
    class_id: Optional[int] = None


@dataclass
class FusedPoseSample:
    stamp: float
    observation_count: int
    xyz: tuple
    quat_xyzw: tuple


@dataclass
class ObjectLandmark:
    object_id: int
    object_name: str
    status: ObjectStatus
    T_map_obj: Matrix4
    confidence: float
    first_seen_time: float
    last_seen_time: float
    observation_count: int = 1
    missed_count: int = 0
    last_detection_id: Optional[int] = None
    last_sam6d_score: Optional[float] = None
    class_id: Optional[int] = None
    transition_history: List[StateTransition] = field(default_factory=list)
    pose_history: List[FusedPoseSample] = field(default_factory=list)
    rot_modes: List[tuple] = field(default_factory=list)   # [(quat_xyzw, 표), ...] 표 내림차순


@dataclass
class ObjectFrameSnapshot:
    """한 프레임에서 본 인스턴스 하나의 기억 상태 (시각화·진단용).

    objectmemory_ws 판에 없던 confidence / p_d 를 함께 남긴다. 어떤 프레임에 왜
    사라졌는지는 r 곡선과 '그때 시야 안이었는가(P_D)' 없이는 사후 확인이 불가능하다.
    판정에는 쓰이지 않는 순수 진단 필드다.
    """
    object_id: int
    object_name: str
    status: ObjectStatus
    T_map_obj: tuple
    visible: bool                      # 이 프레임에 측정을 받아들였는가
    confidence: float = 0.0            # 존재확률 r
    p_d: float = 0.0                   # 이 프레임의 기대 검출확률 (0 = 시야 밖)
    rejected_this_frame: bool = False
    observation_count: int = 0
    T_map_obj_meas: Optional[tuple] = None
    residual_t: Optional[float] = None
    residual_deg: Optional[float] = None


@dataclass
class AssociationDecision:
    decision_type: str                 # short_term_match / new_tentative / rejected
    detection_id: int
    score: float
    object_id: Optional[int] = None
    reject_reason: Optional[str] = None
    debug_info: dict = field(default_factory=dict)


@dataclass
class FrameResult:
    frame_idx: int
    stamp: Optional[float]
    slam_ok: bool
    decisions: List[AssociationDecision] = field(default_factory=list)
    camera_T_map_cam: Optional[tuple] = None
    objects: List[ObjectFrameSnapshot] = field(default_factory=list)


@dataclass
class RunResult:
    store: "ObjectMemoryStore"
    frame_results: List[FrameResult]
    live_ids_by_name: Dict[str, List[int]]
    object_ids_by_name: Dict[str, int]
    frames_total: int
    frames_with_slam: int
    frames_without_slam: int
    detections_total: int


# ===========================================================================
# 8. 저장소 — object_id 의 단일 소유자
# ===========================================================================
_LANDMARK_FIELDS = {f.name for f in dataclass_fields(ObjectLandmark)}


class ObjectMemoryStore:
    def __init__(self, starting_object_id: int = 1):
        self._next_object_id = starting_object_id
        self._landmarks: Dict[int, ObjectLandmark] = {}
        self._transition_log: List[Tuple[int, StateTransition]] = []

    def create_tentative(self, object_name: str, T_map_obj: Matrix4,
                         first_seen_time: float, confidence: float = 0.0,
                         class_id=None, last_detection_id=None,
                         last_sam6d_score=None) -> ObjectLandmark:
        object_id = self._next_object_id
        self._next_object_id += 1
        lm = ObjectLandmark(
            object_id=object_id, object_name=object_name,
            status=ObjectStatus.tentative, T_map_obj=T_map_obj,
            confidence=confidence, first_seen_time=first_seen_time,
            last_seen_time=first_seen_time, observation_count=1, missed_count=0,
            last_detection_id=last_detection_id, last_sam6d_score=last_sam6d_score,
            class_id=class_id,
        )
        self._landmarks[object_id] = lm
        return lm

    def get(self, object_id: int) -> Optional[ObjectLandmark]:
        return self._landmarks.get(object_id)

    def require(self, object_id: int) -> ObjectLandmark:
        if object_id not in self._landmarks:
            raise KeyError(f"unknown object_id: {object_id}")
        return self._landmarks[object_id]

    def all_landmarks(self) -> List[ObjectLandmark]:
        return list(self._landmarks.values())

    def by_status(self, status) -> List[ObjectLandmark]:
        target = coerce_status(status)
        return [lm for lm in self._landmarks.values() if lm.status is target]

    def update_landmark(self, object_id: int, **changes) -> ObjectLandmark:
        lm = self.require(object_id)
        if "object_id" in changes:
            raise ValueError("object_id cannot be changed")
        if "status" in changes:
            raise ValueError("status cannot be changed directly; use apply_transition")
        unknown = set(changes) - _LANDMARK_FIELDS
        if unknown:
            raise ValueError(f"unknown landmark fields: {sorted(unknown)}")
        for key, value in changes.items():
            setattr(lm, key, value)
        return lm

    def apply_transition(self, object_id: int, to_status, reason: str,
                         stamp=None, metadata=None) -> StateTransition:
        lm = self.require(object_id)
        tr = build_transition(lm.status, to_status, reason, stamp, metadata)
        lm.status = tr.to_status
        lm.transition_history.append(tr)
        self._transition_log.append((object_id, tr))
        return tr

    def transition_log(self) -> List[Tuple[int, StateTransition]]:
        return list(self._transition_log)


# ===========================================================================
# 9. 프레임 처리 — 연관 / spawn / 융합 / 감쇠·삭제
# ===========================================================================
def _snapshot(store, live_ids_by_name, seen_ids, meas_by_id=None,
              rejected_names=None, p_d_by_id=None):
    meas_by_id = meas_by_id or {}
    rejected_names = rejected_names or set()
    p_d_by_id = p_d_by_id or {}
    snap: List[ObjectFrameSnapshot] = []
    for name, ids in live_ids_by_name.items():
        for object_id in ids:
            lm = store.get(object_id)
            if lm is None or lm.status in (ObjectStatus.deleted, ObjectStatus.merged):
                continue
            m = meas_by_id.get(object_id)
            snap.append(ObjectFrameSnapshot(
                object_id=object_id, object_name=name, status=lm.status,
                T_map_obj=lm.T_map_obj, visible=object_id in seen_ids,
                confidence=lm.confidence, p_d=p_d_by_id.get(object_id, 0.0),
                rejected_this_frame=(name in rejected_names and object_id not in seen_ids),
                observation_count=lm.observation_count,
                T_map_obj_meas=m[0] if m else None,
                residual_t=m[1] if m else None,
                residual_deg=m[2] if m else None,
            ))
    return snap


def _run_initiator(store, live_ids_by_name, det, name, score, T_meas, ts, r_init):
    """게이트 안에서 짝을 못 찾은 측정을 새 tentative 인스턴스로 띄운다.

    첫 목격, 진짜 두 번째 개체, 오염된 첫 포즈에 가려진 올바른 군집 — 셋 다 여기로
    온다. 셋을 구별하지 않고 일단 띄운 뒤, 살아남는지는 존재 필터가 판정한다.
    """
    lm = store.create_tentative(
        object_name=name, T_map_obj=T_meas, first_seen_time=ts,
        confidence=r_init, class_id=det.class_id,
        last_detection_id=det.detection_id, last_sam6d_score=score,
    )
    live_ids_by_name.setdefault(name, []).append(lm.object_id)
    if MODE_FUSION_ENABLED:
        # 첫 측정도 한 표로 세어 둔다. 안 그러면 두 번째 측정이 첫 봉우리가 되어
        # 씨앗 자세가 통째로 버려진다.
        update_rotation_modes(lm.rot_modes,
                              tuple(tuple(T_meas[r][c] for c in range(3)) for r in range(3)),
                              name)
    return lm


def _run_deleter(store, live_ids_by_name, seen_ids, ts, pose, ages, cam_K,
                 img_size, pd_base, r_lost, r_retire, tentative_max_age,
                 min_obs_longterm, p_d_by_id):
    """이 프레임에 측정을 못 받은 인스턴스의 존재확률을 P_D 만큼 깎고 은퇴시킨다.

    시야 밖(P_D=0)이면 r 이 그대로라 다른 데를 보는 시간은 객체를 죽이지 않는다.
    삭제된 (이름, object_id) 를 돌려주어 호출자가 live 목록에서 빼게 한다.
    """
    released = []
    for name, ids in live_ids_by_name.items():
        for object_id in ids:
            if object_id in seen_ids:
                continue
            lm = store.get(object_id)
            if lm is None or lm.status in (ObjectStatus.deleted, ObjectStatus.merged,
                                           ObjectStatus.remembered):
                # remembered = 장기 기억. 더 이상 미검출을 세지 않는다.
                continue
            p_d = expected_p_d(pose.T_map_cam, lm.T_map_obj, cam_K, img_size,
                               pd_base=pd_base)
            p_d_by_id[object_id] = p_d
            r = update_existence_missed(lm.confidence, p_d)
            store.update_landmark(
                object_id,
                missed_count=lm.missed_count + (1 if p_d > 0.0 else 0),
                confidence=r,
            )
            if lm.status is ObjectStatus.active and r < r_lost:
                store.apply_transition(object_id, ObjectStatus.lost,
                                       "missed_threshold", stamp=ts)
            elif lm.status is ObjectStatus.tentative and r < r_retire:
                store.apply_transition(object_id, ObjectStatus.deleted,
                                       "existence_floor", stamp=ts)
                released.append((name, object_id))
            elif (lm.status is ObjectStatus.tentative
                    and ages.get(object_id, 0) >= tentative_max_age):
                store.apply_transition(object_id, ObjectStatus.deleted,
                                       "tentative_timeout", stamp=ts)
                released.append((name, object_id))
            elif lm.status is ObjectStatus.lost and r < r_retire:
                # 은퇴 분기: 실제로 자리잡았던 트랙만 장기 기억으로 남긴다.
                if lm.observation_count >= min_obs_longterm:
                    store.apply_transition(object_id, ObjectStatus.remembered,
                                           "long_term_memory", stamp=ts)
                else:
                    store.apply_transition(object_id, ObjectStatus.deleted,
                                           "low_evidence", stamp=ts)
                    released.append((name, object_id))
    return released


def _nearest_pose(stamps: List[float], poses: List[SlamCameraPose], ts: float,
                  tol: float) -> Optional[SlamCameraPose]:
    if not stamps:
        return None
    i = bisect.bisect_left(stamps, ts)
    best, best_dt = None, tol
    for j in (i - 1, i, i + 1):
        if 0 <= j < len(stamps):
            dt = abs(stamps[j] - ts)
            if dt <= best_dt:
                best_dt, best = dt, poses[j]
    return best


class StreamingObjectMemory:
    """프레임 하나씩 굴러가는 지속 객체 기억.

    step(dets, pose, ts, frame_idx) 이 전부다. 오프라인 배치와 실시간 스트리밍이
    같은 함수를 쓰기 때문에 '어떤 포즈를 어떻게 얻는가'만 다르고 판정은 동일하다.
    """

    def __init__(self, assoc_trans_gate_m: float = ASSOC_TRANS_GATE_M,
                 assoc_rot_gate_deg: Optional[float] = ASSOC_ROT_GATE_DEG,
                 score_floor: float = SCORE_FLOOR,
                 tentative_gate_mult: float = TENTATIVE_GATE_MULT,
                 pd_base: float = PD_BASE, clutter_ratio: float = CLUTTER_RATIO,
                 r_init: float = R_INIT, r_promote: float = R_PROMOTE,
                 r_lost: float = R_LOST, r_retire: float = R_RETIRE,
                 tentative_max_age: int = TENTATIVE_MAX_AGE,
                 min_obs_longterm: int = MIN_OBS_LONGTERM,
                 cam_K=None, img_size=None, quality_weighting: bool = True,
                 weights: Optional[QualityWeights] = None,
                 r_prior: float = R_PRIOR):
        self.assoc_trans_gate_m = assoc_trans_gate_m
        self.assoc_rot_gate_deg = assoc_rot_gate_deg
        self.rot_gate_off = assoc_rot_gate_deg is None or assoc_rot_gate_deg < 0.0
        self.score_floor = score_floor
        self.tentative_gate_mult = tentative_gate_mult
        self.pd_base = pd_base
        self.clutter_ratio = clutter_ratio
        self.r_init = r_init
        self.r_promote = r_promote
        self.r_lost = r_lost
        self.r_retire = r_retire
        self.tentative_max_age = tentative_max_age
        self.min_obs_longterm = min_obs_longterm
        self.cam_K = cam_K
        self.img_size = img_size
        self.quality_weighting = quality_weighting
        self.weights = weights if weights is not None else QualityWeights()
        self.logit_prior = math.log(r_prior / (1.0 - r_prior))

        self.store = ObjectMemoryStore()
        self.live_ids_by_name: Dict[str, List[int]] = {}
        self.ages: Dict[int, int] = {}
        self.detections_total = 0

    # -- SLAM 포즈가 없는 프레임: 검출을 버리고 기억을 얼린다 ------------------
    def step_no_pose(self, dets, ts, frame_idx: int) -> FrameResult:
        self.detections_total += len(dets)
        fr = FrameResult(frame_idx=frame_idx, stamp=ts, slam_ok=False)
        for det in dets:
            fr.decisions.append(AssociationDecision(
                decision_type="rejected", detection_id=det.detection_id,
                score=min(max(det.score, 0.0), 1.0),
                reject_reason="no SLAM pose within tolerance"))
        fr.objects = _snapshot(self.store, self.live_ids_by_name, set())
        return fr

    def step(self, dets, pose: SlamCameraPose, ts, frame_idx: int) -> FrameResult:
        self.detections_total += len(dets)
        store, live_ids_by_name, ages = self.store, self.live_ids_by_name, self.ages

        fr = FrameResult(frame_idx=frame_idx, stamp=ts, slam_ok=True)
        fr.camera_T_map_cam = pose.T_map_cam
        # seen = 이 프레임에 측정을 '받아들인' object_id. 자기 검출이 다른 인스턴스를
        # spawn 시키며 떠난 경우는 seen 이 아니다(아니면 영원히 얼어붙는다).
        seen_ids, rejected_names, meas_by_id, p_d_by_id = set(), set(), {}, {}

        # --- 1단계: 측정 생성 + 점수 하한 기각 ---------------------------------
        pending = []
        for det in dets:
            name = det.object_name
            score = min(max(det.score, 0.0), 1.0)
            T_meas = compute_T_map_obj(pose.T_map_cam, det.T_cam_obj)
            if score < self.score_floor:
                rejected_names.add(name)
                fr.decisions.append(AssociationDecision(
                    decision_type="rejected", detection_id=det.detection_id,
                    score=score, reject_reason="score below floor",
                    debug_info={"object_name": name}))
                continue
            pending.append((det, name, score, T_meas))

        # --- 2단계: 게이트 안의 (검출, 인스턴스) 후보쌍 -------------------------
        pairs = []
        for i, (det, name, score, T_meas) in enumerate(pending):
            for object_id in live_ids_by_name.get(name, []):
                lm = store.get(object_id)
                if lm is None or lm.status in (ObjectStatus.deleted, ObjectStatus.merged):
                    continue
                dt = transform_distance_translation(lm.T_map_obj, T_meas)
                dth = transform_distance_rotation_deg(lm.T_map_obj, T_meas)
                # tentative 는 아직 추정이 흔들리므로 게이트를 넓게 준다.
                mult = (self.tentative_gate_mult
                        if lm.status is ObjectStatus.tentative else 1.0)
                if dt > self.assoc_trans_gate_m * mult:
                    continue
                if not self.rot_gate_off and dth > self.assoc_rot_gate_deg * mult:
                    continue
                pairs.append((dt, i, object_id, dth))

        # 가까운 쌍부터 그리디로 1:1 배정
        pairs.sort(key=lambda p: p[0])
        assigned_det, claimed_ids = {}, set()
        for dt, i, object_id, dth in pairs:
            if i in assigned_det or object_id in claimed_ids:
                continue
            assigned_det[i] = (object_id, dt, dth)
            claimed_ids.add(object_id)

        # --- 3단계: 배정된 것은 수락(=이동 반영), 나머지는 spawn ----------------
        for i, (det, name, score, T_meas) in enumerate(pending):
            match = assigned_det.get(i)
            if match is None:
                if self.quality_weighting:
                    log_lr = detection_log_lr(
                        score, det.T_cam_obj[2][3], None, self.assoc_trans_gate_m,
                        self.pd_base, self.clutter_ratio, self.weights)
                    spawn_r = 1.0 / (1.0 + math.exp(-(self.logit_prior + log_lr)))
                else:
                    spawn_r = self.r_init
                lm = _run_initiator(store, live_ids_by_name, det, name, score,
                                    T_meas, ts, spawn_r)
                ages[lm.object_id] = 0
                seen_ids.add(lm.object_id)
                meas_by_id[lm.object_id] = (T_meas, 0.0, 0.0)
                p_d_by_id[lm.object_id] = self.pd_base
                fr.decisions.append(AssociationDecision(
                    decision_type="new_tentative", detection_id=det.detection_id,
                    object_id=lm.object_id, score=score,
                    debug_info={"object_name": name}))
                continue

            object_id, dt, dth = match
            lm = store.require(object_id)
            meas_by_id[object_id] = (T_meas, dt, dth)
            seen_ids.add(object_id)
            p_d_by_id[object_id] = self.pd_base
            n_prev = lm.observation_count
            T_fused = fuse_pose(lm.T_map_obj, n_prev, T_meas, name, lm.rot_modes)
            lm.pose_history.append(FusedPoseSample(
                stamp=det.stamp, observation_count=n_prev + 1,
                xyz=(T_fused[0][3], T_fused[1][3], T_fused[2][3]),
                quat_xyzw=tuple(rotation_to_quat(
                    tuple(tuple(T_fused[r][c] for c in range(3)) for r in range(3)))),
            ))
            if self.quality_weighting:
                log_lr = detection_log_lr(
                    score, det.T_cam_obj[2][3], dt, self.assoc_trans_gate_m,
                    self.pd_base, self.clutter_ratio, self.weights)
                new_r = update_existence_loglr(lm.confidence, log_lr)
            else:
                new_r = update_existence_detected(lm.confidence, self.pd_base,
                                                  self.clutter_ratio)
            store.update_landmark(
                object_id, T_map_obj=T_fused, last_seen_time=ts,
                observation_count=n_prev + 1, missed_count=0, confidence=new_r,
                last_detection_id=det.detection_id, last_sam6d_score=score)
            # 승격 / 재획득
            if self.quality_weighting:
                promote_ok = new_r >= self.r_promote
            else:
                promote_ok = (lm.observation_count >= PROMOTE_HITS
                              and new_r > self.r_promote)
            if lm.status in (ObjectStatus.lost, ObjectStatus.remembered):
                store.apply_transition(object_id, ObjectStatus.active,
                                       "redetected", stamp=ts)
            elif lm.status is ObjectStatus.tentative and promote_ok:
                store.apply_transition(object_id, ObjectStatus.active,
                                       "promoted", stamp=ts)
            fr.decisions.append(AssociationDecision(
                decision_type="short_term_match", detection_id=det.detection_id,
                object_id=object_id, score=score,
                debug_info={"object_name": name, "residual_t": round(dt, 4),
                            "residual_deg": round(dth, 2)}))

        for oid in list(ages):
            ages[oid] += 1

        released = _run_deleter(
            store, live_ids_by_name, seen_ids, ts, pose, ages, self.cam_K,
            self.img_size, self.pd_base, self.r_lost, self.r_retire,
            self.tentative_max_age, self.min_obs_longterm, p_d_by_id)
        for name, object_id in released:
            ids = live_ids_by_name.get(name)
            if ids and object_id in ids:
                ids.remove(object_id)
            if ids == []:
                del live_ids_by_name[name]

        fr.objects = _snapshot(store, live_ids_by_name, seen_ids, meas_by_id,
                               rejected_names, p_d_by_id)
        return fr

    def object_ids_by_name(self) -> Dict[str, int]:
        return {name: ids[0] for name, ids in self.live_ids_by_name.items() if ids}


def run_object_lifecycle(
    slam_poses: List[SlamCameraPose],
    detections_by_frame: Dict[int, list],
    timestamps: List[float],
    time_tolerance: float = TIME_TOLERANCE,
    **kwargs,
) -> RunResult:
    """프레임을 인덱스 순서로 훑으며 가장 가까운 SLAM 포즈에 검출을 융합한다."""
    poses = sorted(slam_poses, key=lambda p: p.stamp)
    stamps = [p.stamp for p in poses]
    mem = StreamingObjectMemory(**kwargs)

    frame_results, with_slam, without_slam = [], 0, 0
    for frame_idx in sorted(detections_by_frame.keys()):
        dets = detections_by_frame[frame_idx]
        if 0 <= frame_idx < len(timestamps):
            ts = timestamps[frame_idx]
        elif dets:
            ts = dets[0].stamp
        else:
            ts = None
        pose = _nearest_pose(stamps, poses, ts, time_tolerance) if ts is not None else None
        if pose is None:
            without_slam += 1
            fr = mem.step_no_pose(dets, ts, frame_idx)
        else:
            with_slam += 1
            fr = mem.step(dets, pose, ts, frame_idx)
        frame_results.append(fr)

    return RunResult(
        store=mem.store, frame_results=frame_results,
        live_ids_by_name=mem.live_ids_by_name,
        object_ids_by_name=mem.object_ids_by_name(),
        frames_total=len(detections_by_frame), frames_with_slam=with_slam,
        frames_without_slam=without_slam, detections_total=mem.detections_total,
    )


# ===========================================================================
# 10. 입력 로더 — integration 산출물만 읽는다
# ===========================================================================
def load_slam_trajectory(path: str, source_slam_id: str = "orbslam3") -> List[SlamCameraPose]:
    """TUM 궤적 (t tx ty tz qx qy qz qw) -> SlamCameraPose 목록 (시간 오름차순)."""
    poses: List[SlamCameraPose] = []
    with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
        for lineno, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                raise ValueError(f"{path}:{lineno}: TUM 8열이 아니다 ({len(parts)}열)")
            ts, tx, ty, tz, qx, qy, qz, qw = (float(p) for p in parts)
            poses.append(SlamCameraPose(
                stamp=ts, T_map_cam=make_transform_from_quat((tx, ty, tz), (qx, qy, qz, qw)),
                source_slam_id=source_slam_id))
    poses.sort(key=lambda p: p.stamp)
    return poses


def _resolve_db3(bag_path: str) -> str:
    bag_path = os.path.expanduser(bag_path)
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return bag_path
    cands = sorted(set(glob.glob(os.path.join(bag_path, "*.db3"))
                       + glob.glob(os.path.join(bag_path, "bag", "*.db3"))
                       + glob.glob(os.path.join(bag_path, "*", "*.db3"))))
    if not cands:
        raise FileNotFoundError(f"no .db3 found under: {bag_path}")
    return cands[0]


def load_frame_timestamps(bag_path: str,
                          color_topic_hint: str = "/camera/camera/color/image_raw"
                          ) -> List[float]:
    """bag 의 컬러 토픽 타임스탬프 [s]. N번째 메시지 = SAM-6D frame_index N."""
    conn = sqlite3.connect(f"file:{_resolve_db3(bag_path)}?mode=ro", uri=True)
    try:
        cur = conn.cursor()
        cur.execute("select id, name from topics")
        rows = cur.fetchall()
        topic_id = None
        for tid, name in rows:
            if name == color_topic_hint:
                topic_id = tid
                break
        if topic_id is None:
            for tid, name in rows:
                if color_topic_hint and color_topic_hint in name:
                    topic_id = tid
                    break
        if topic_id is None:
            for tid, name in rows:
                low = name.lower()
                if ("color" in low or "rgb" in low) and "image" in low and "camera_info" not in low:
                    topic_id = tid
                    break
        if topic_id is None:
            raise ValueError(f"no color image topic (hint={color_topic_hint!r}); "
                             f"topics={[n for _, n in rows]}")
        cur.execute("select timestamp from messages where topic_id = ? order by timestamp",
                    (topic_id,))
        stamps = [r[0] for r in cur.fetchall()]
    finally:
        conn.close()
    return [s / 1e9 for s in stamps]      # ros2 bag 은 ns 로 저장한다


def load_detections(objects_json: str, timestamps: Optional[List[float]] = None,
                    mm_to_m: bool = True) -> Dict[int, List[Sam6DDetection]]:
    """<출력>/sam/objects.json -> {frame_index: [Sam6DDetection, ...]}.

    PEM 입력 트리(_stage/sam/pem_inputs) 대신 보존본 하나만 읽는다. 프레임 안의
    순서는 물체 이름 오름차순으로 맞춘다 — PEM 트리를 sorted(glob) 로 훑던
    objectmemory_ws 어댑터와 같은 순서여야 spawn 순서(=object_id)가 일치한다.
    """
    with open(os.path.expanduser(objects_json), "r", encoding="utf-8") as f:
        payload = json.load(f)
    scale = 0.001 if mm_to_m else 1.0
    out: Dict[int, List[Sam6DDetection]] = {}
    for d in payload.get("detections", []):
        if not d.get("pem_ok") or d.get("R") is None or d.get("t_mm") is None:
            continue                      # 포즈가 없는 ISM-only 항목은 건너뛴다
        frame_idx = int(d["frame_index"])
        stamp = (timestamps[frame_idx]
                 if timestamps is not None and 0 <= frame_idx < len(timestamps)
                 else float(frame_idx))
        R = tuple(tuple(float(v) for v in row) for row in d["R"])
        t = tuple(float(v) * scale for v in d["t_mm"])
        out.setdefault(frame_idx, []).append(Sam6DDetection(
            stamp=stamp, detection_id=frame_idx * 1000,
            object_name=d["object"], T_cam_obj=make_transform(R, t),
            score=float(d.get("score", 0.0)),
            bbox=tuple(d["bbox"]) if d.get("bbox") is not None else None,
        ))
    for frame_idx, dets in out.items():
        dets.sort(key=lambda x: x.object_name)
        for local_id, det in enumerate(dets):
            det.detection_id = frame_idx * 1000 + local_id
    return out


def load_camera(info_json: str, which: str = "sam"):
    """<데이터>/info.json -> (cam_K 3x3, (width, height)). P_D 시야 모델의 입력."""
    with open(os.path.expanduser(info_json), "r", encoding="utf-8") as f:
        info = json.load(f)
    cam = info.get(which, {}).get("camera")
    if not cam or "K" not in cam:
        return None, None
    k = cam["K"]
    K = ((k[0], k[1], k[2]), (k[3], k[4], k[5]), (k[6], k[7], k[8]))
    return K, (int(cam["width"]), int(cam["height"]))


def load_npy_4x4(path: str) -> Matrix4:
    """4x4 .npy 를 numpy 없이 읽는다 (이 모듈은 표준 라이브러리만 쓴다)."""
    with open(os.path.expanduser(path), "rb") as f:
        if f.read(6) != b"\x93NUMPY":
            raise ValueError(f"not a .npy file: {path}")
        major = f.read(1)[0]
        f.read(1)                                     # minor
        hlen = int.from_bytes(f.read(2 if major == 1 else 4), "little")
        meta = ast.literal_eval(f.read(hlen).decode("latin1").strip())
        if tuple(meta["shape"]) != (4, 4):
            raise ValueError(f"{path}: shape {meta['shape']} — 4x4 가 아니다")
        code = {"<f8": "d", "<f4": "f", "|f8": "d", "=f8": "d"}.get(meta["descr"])
        if code is None:
            raise ValueError(f"{path}: dtype {meta['descr']} 는 지원하지 않는다")
        n = struct.calcsize(code)
        vals = struct.unpack("<" + code * 16, f.read(16 * n))
    if meta.get("fortran_order"):
        vals = [vals[c * 4 + r] for r in range(4) for c in range(4)]
    return tuple(tuple(float(vals[r * 4 + c]) for c in range(4)) for r in range(4))


# ===========================================================================
# 11. 결과 직렬화
# ===========================================================================
def landmark_to_dict(lm: ObjectLandmark, pose_history: bool = True) -> dict:
    T = lm.T_map_obj
    q = rotation_to_quat(tuple(tuple(T[r][c] for c in range(3)) for r in range(3)))
    d = {
        "object_id": lm.object_id,
        "object_name": lm.object_name,
        "status": lm.status.value,
        "confidence": round(lm.confidence, 4),
        "observations": lm.observation_count,
        "missed": lm.missed_count,
        "first_seen": lm.first_seen_time,
        "last_seen": lm.last_seen_time,
        "xyz": [round(T[i][3], 4) for i in range(3)],
        "quat_xyzw": [round(v, 6) for v in q],
        "T_map_obj": [[round(v, 6) for v in row] for row in T],
        "transitions": [{"to": tr.to_status.value, "reason": tr.reason,
                         "stamp": tr.stamp} for tr in lm.transition_history],
    }
    if pose_history:
        d["pose_history"] = [
            {"t": round(s.stamp, 6), "n": s.observation_count,
             "xyz": [round(v, 4) for v in s.xyz],
             "quat_xyzw": [round(v, 6) for v in s.quat_xyzw]}
            for s in lm.pose_history
        ]
    return d


def result_to_dict(result: RunResult, timeline: bool = True) -> dict:
    """최종 객체 목록 + (선택) 프레임별 상태 타임라인."""
    objects = [landmark_to_dict(lm)
               for lm in sorted(result.store.all_landmarks(), key=lambda x: x.object_id)]
    out = {
        "stats": {
            "frames_total": result.frames_total,
            "frames_with_slam": result.frames_with_slam,
            "frames_without_slam": result.frames_without_slam,
            "detections": result.detections_total,
            "instances": len(objects),
            "live_instances": sum(1 for o in objects if o["status"] != "deleted"),
        },
        "objects": objects,
        "transitions": [
            {"object_id": oid, "from": tr.from_status.value, "to": tr.to_status.value,
             "reason": tr.reason, "stamp": tr.stamp}
            for oid, tr in result.store.transition_log()
        ],
    }
    if timeline:
        out["timeline"] = [
            {
                "frame": fr.frame_idx,
                "stamp": fr.stamp,
                "slam_ok": fr.slam_ok,
                "objects": [
                    {"object_id": o.object_id, "name": o.object_name,
                     "status": o.status.value, "r": round(o.confidence, 4),
                     "p_d": round(o.p_d, 3), "seen": o.visible,
                     "obs": o.observation_count,
                     "xyz": [round(o.T_map_obj[i][3], 4) for i in range(3)]}
                    for o in fr.objects
                ],
            }
            for fr in result.frame_results
        ]
    return out
