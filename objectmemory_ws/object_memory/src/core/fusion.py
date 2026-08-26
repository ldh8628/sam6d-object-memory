"""Pose fusion for object_memory landmarks.

A landmark's authoritative ``T_map_obj`` is the running mean of its accepted
per-frame measurements rather than the latest overwrite:

    translation : incremental mean, weight 1/(n+1)
    rotation    : quaternion nlerp, weight 1/(n+1)

where ``n`` is the number of prior observations. This converges to the average
pose of a static object and damps per-frame SAM-6D jitter, while the caller's
association gate keeps gross outliers out of the fusion in the first place.
"""

from __future__ import annotations

import math

from core.transforms import (
    Matrix4,
    make_transform,
    make_transform_from_quat,
    quat_nlerp,
    quat_to_rotation,
    rotation_to_quat,
)


# ===========================================================================
# 대칭 선언 (AF1, 2026-08-19)
# ===========================================================================
# 물체 좌표계에서 **각도가 원리적으로 존재하지 않는 회전축**만 적는다. 여기 적힌 축으로
# 돌린 자세는 서로 다른 답이 아니라 같은 답이므로, 섞기 전에 한쪽으로 맞춘다.
# 그러지 않으면 nlerp 가 0° 와 120° 를 평균해 **한 번도 관측된 적 없는 중간 자세**를
# 만든다(실측: 우유팩 융합 결과가 모든 실제 관측과 24° 떨어진 유령 자세였다).
#
# 판정 근거 = 모델점을 그 축으로 돌렸을 때의 자기정합 오차(지름 대비, 10° 간격 전 각도):
#   Sikhye 0.63% · Sauce 1.54% · Mugcup 2.22%  → 회전체로 인정
#   (대조군) choco 8.53% 는 인정 안 함. milk 는 1.93% 이나 정사각 단면이라 연속이 아닌
#   90° 배수 대칭이고 면마다 인쇄가 다르므로 **일부러 뺐다**. 인형류는 어떤 축으로도
#   자기정합이 안 된다. 자동 유도는 소각(10~20°)이 절대임계를 통과해 곰·공룡에 가짜
#   대칭을 만들어 악화시켰으므로, 이 표는 손으로 쓴 보수적 목록이다.
SYMMETRY_ENABLED = True
SYMMETRY_STEP_DEG = 10
SYMMETRY_AXES = {                      # CAD 좌표계 기준 회전축(정규화)
    "Sikhye_high": (0.0, 0.0, 1.0),
    "Sauce_high": (0.0, 0.0, 1.0),
    "Mugcup_high": (0.1057, 0.0337, 0.9938),
}
_SYM_CACHE: dict = {}


def _axis_rotation(axis, deg: float):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    x, y, z = (c / n for c in axis)
    a = math.radians(deg)
    c, s, k = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return ((c + x * x * k, x * y * k - z * s, x * z * k + y * s),
            (y * x * k + z * s, c + y * y * k, y * z * k - x * s),
            (z * x * k - y * s, z * y * k + x * s, c + z * z * k))


def symmetry_group(object_name):
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
    tr = sum(A[k][0] * B[k][0] + A[k][1] * B[k][1] + A[k][2] * B[k][2] for k in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


def align_to_symmetry(prev_R, meas_R, object_name):
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


# ===========================================================================
# 모드 다수결 회전 융합 (AF1, 2026-08-19)
# ===========================================================================
# SAM-6D 는 프레임마다 자세를 독립적으로 추정하고 그 결과가 **이봉/다봉**이다
# (같은 면을 보고도 180° 뒤집힌 답이 섞인다). 러닝 평균은 두 봉우리를 뭉개
# 실제로는 관측된 적 없는 중간 자세를 만든다 — 지도가 흔들리는 직접 원인이다.
# 그래서 회전만 평균 대신 **표가 가장 많은 자세(모드)** 를 쓴다. 위치는 그대로 평균이다
# (위치는 단봉이라 평균이 옳고, 평균이 잡음을 눌러 준다).
#
# 실측(운영 무변경 재생, 4 bag): 지도 배회 1470° → 714°(-51%), milk 유령 자세 11° → 1°,
#   관측 일치율 6% → 51%. **비대칭 객체에도 듣는다**(Dinosaur -84%, Bear -47%)는 점이
#   중요하다 — 대칭 선언(-9%)보다 크고, 손으로 쓴 표에 의존하지 않는다.
#
# 구현은 스트리밍 군집이다. 프레임마다 과거 전체를 다시 보면 O(n^2) 이라 못 쓴다.
# 대표 자세 목록을 들고 다니며 들어온 측정을 가장 가까운 대표에 붙이고, 어디에도 안
# 붙으면 새 봉우리를 연다. 대칭이 선언된 물체는 붙이기 전에 동치 자세로 돌린다.
MODE_FUSION_ENABLED = True
MODE_FUSION_TOL_DEG = 30.0     # 이 안이면 같은 봉우리
MODE_FUSION_MAX_MODES = 8      # 표가 적은 봉우리는 버린다


def update_rotation_modes(modes, meas_R, object_name=None):
    """회전 모드 목록을 제자리에서 갱신하고 (목록, 으뜸 모드의 회전) 을 돌려준다.

    modes 는 [(quat_xyzw, 표), ...] 로 표 내림차순이다. 판정에 쓰이는 것은 맨 앞
    원소뿐이고, 나머지는 점유율(=신뢰도)과 봉우리 전환을 위해 남긴다.
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
    """으뜸 모드의 표 점유율. 그 자세를 얼마나 믿을 수 있는지의 값(0~1)."""
    tot = sum(w for _q, w in modes)
    return (modes[0][1] / tot) if tot > 0 else 0.0


def _rotation_of(T: Matrix4):
    return ((T[0][0], T[0][1], T[0][2]),
            (T[1][0], T[1][1], T[1][2]),
            (T[2][0], T[2][1], T[2][2]))


def fuse_pose(prev_T: Matrix4, n_prev: int, meas_T: Matrix4,
              object_name=None, rot_modes=None) -> Matrix4:
    """Fuse ``meas_T`` into ``prev_T`` given ``n_prev`` prior observations.

    n_prev=0 returns the measurement unchanged (first observation seeds).

    ``rot_modes`` 를 주면 회전은 러닝 평균 대신 **모드 다수결**로 정해진다(위 설명).
    안 주면 예전 동작 그대로다 — 기존 호출자와 테스트는 영향을 받지 않는다.
    ``object_name`` 이 SYMMETRY_AXES 에 있으면 섞기 전에 동치 자세로 맞춘다.
    """
    if rot_modes is not None and MODE_FUSION_ENABLED:
        _m, mode_R = update_rotation_modes(rot_modes, _rotation_of(meas_T), object_name)
        if n_prev <= 0:
            t = (meas_T[0][3], meas_T[1][3], meas_T[2][3])
        else:
            w = 1.0 / (n_prev + 1)
            t = tuple(prev_T[i][3] * (1.0 - w) + meas_T[i][3] * w for i in range(3))
        return make_transform(mode_R, t)
    if n_prev <= 0:
        return meas_T
    w = 1.0 / (n_prev + 1)

    tp = (prev_T[0][3], prev_T[1][3], prev_T[2][3])
    tm = (meas_T[0][3], meas_T[1][3], meas_T[2][3])
    t = tuple(tp[i] * (1.0 - w) + tm[i] * w for i in range(3))

    meas_R = (align_to_symmetry(_rotation_of(prev_T), _rotation_of(meas_T), object_name)
              if object_name else _rotation_of(meas_T))
    qp = rotation_to_quat(_rotation_of(prev_T))
    qm = rotation_to_quat(meas_R)
    q = quat_nlerp(qp, qm, w)

    return make_transform_from_quat(t, q)
