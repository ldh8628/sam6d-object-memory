#!/usr/bin/env python3
"""Stage V(후보 텍스처 검증) 설정 정규화 — 노드와 split 프로세스가 같은 값을 쓰게 한다.

기본값이 두 군데로 갈라지면 "노드는 되는데 split 은 안 된다"가 반드시 생긴다.
그래서 기본값과 병합 규칙을 여기 한 곳에만 둔다.
"""
from __future__ import annotations

# 기하 1위를 검증할 후보 범위. 점수 합산/재정렬에는 사용하지 않는다.
DEFAULT_APPE = {"topk": 100, "stride": 1}

# `verify` 키가 아예 없을 때(=대부분의 옛 config)와 명시적으로 null 을 준 것을 가른다.
UNSET = object()

DEFAULT_VERIFY = {
    # 기하 300→1 선택은 항상 유지하며 아래 채널은 선택 후보를 검증만 한다.
    "enabled": True,
    "w_col": 1.0,          # 호환 키: 색상은 별도 진단값만 계산하며 가중치로 쓰지 않음
    "texture_max_rank": None,
    "texture_min_score": None,
    "size_ratio_max": 1.0, # input bbox area / rendered bbox area
    "iou_min": None,       # 보정 전에는 diagnostic-only
    "point_splat_radius_px": 2,
    "projection_chunk": 32,
    "publish": False,      # Detection3D.results[1] 로 신뢰도까지 발행할지
    "dump_topk": 0,        # >0 이면 후보 목록을 검출 행에 덤프(보고서용, 판정 무관)
    # 자동 유도 금지 — 소각 대칭이 곰·공룡에 가짜 대칭을 만들었던 전례가 있다.
    # objectmemory core/fusion.py 의 손으로 쓴 보수적 표를 그대로 옮긴 것이다.
    "symmetry_axes": {
        "Sikhye_high": [0.0, 0.0, 1.0],
        "Sauce_high": [0.0, 0.0, 1.0],
        "Mugcup_high": [0.1057, 0.0337, 0.9938],
    },
}


def build_appe_cfg(rt):
    """runtime 설정 → (PEM 에 넘길 appe_rerank dict 또는 None, verify dict 또는 None).

    * verify.enabled 가 켜져 있으면 독립 후보 계측 입력을 자동으로 켠다.
    * `verify` 키가 없으면 기본값(ON)이 적용된다. 끄려면 `verify: null` 또는
      `verify: {enabled: false}` 를 명시해야 한다.
    * 모든 경우 초기 pose는 원본 기하 argmax이며 verify는 그 순위를 바꾸지 않는다.
    """
    ar = rt.get("appe_rerank")
    ver = rt.get("verify", UNSET)
    if ver is None:                    # 명시적 null = 끄기
        ver = {"enabled": False}
    elif ver is UNSET:                 # 키가 없으면 기본값(=ON)
        ver = {}
    ver = dict(DEFAULT_VERIFY, **(dict(ver) if isinstance(ver, dict) else {}))
    on = bool(ver.get("enabled"))
    if not ar and not on:
        return None, ver
    ar = dict(DEFAULT_APPE, **(dict(ar) if isinstance(ar, dict) else {}))
    if ver is not None:
        ar["verify"] = ver
    return ar, ver


def describe(ar, ver):
    """기동 로그 한 줄."""
    if not ar:
        return "[pem] 기하-only 300→1 · 독립 검증 OFF"
    base = f"[pem] 기하-only 300→1 · 검증후보 topk={ar['topk']} stride={ar['stride']}"
    if not (ver and ver.get("enabled")):
        return base + " · 독립 검증 OFF"
    return (base + f" · texture/size/IoU 독립 검증 ON: "
            f"texture_rank≤{ver['texture_max_rank']} size_ratio≤{ver['size_ratio_max']} "
            f"iou≥{ver['iou_min']} publish={ver['publish']}")
