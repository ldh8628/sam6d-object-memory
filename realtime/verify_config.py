#!/usr/bin/env python3
"""Stage V(후보 텍스처 검증) 설정 정규화 — 노드와 split 프로세스가 같은 값을 쓰게 한다.

기본값이 두 군데로 갈라지면 "노드는 되는데 split 은 안 된다"가 반드시 생긴다.
그래서 기본값과 병합 규칙을 여기 한 곳에만 둔다.
"""
from __future__ import annotations

# 외형 재정렬(K2)의 운영 검증값(final3). 예전 기본값은 50/0.5/4 였다.
DEFAULT_APPE = {"topk": 100, "w_geo": 2.0, "stride": 1}

# `verify` 키가 아예 없을 때(=대부분의 옛 config)와 명시적으로 null 을 준 것을 가른다.
UNSET = object()

DEFAULT_VERIFY = {
    # 2026-08-21 기본 ON. 0807/185223 실측: 뒤집힘(>45°) 7.4% -> 5.1%, 교정 31 : 파손 7,
    # PEM 시간 66 -> 66 ms. 끄려면 config 에 `verify: {enabled: false}` 또는 `verify: null`.
    "enabled": True,
    "w_col": 1.0,          # 원색 채널 가중 (0 = 특징 채널만 = 현행 final3 와 동일)
    "mode_tol_deg": 30.0,  # 이 안이면 같은 봉우리 (ObjectMemory 모드 융합과 같은 값)
    "flip_deg": 45.0,      # 이보다 멀면 '다른 자세'로 보고 마진 계산에 넣는다
    "tau_pass": 0.8,       # ↓ 아래 넷은 초기치. 오프라인 보정 후 확정할 것
    "tau_corr": 1.0,
    "tau_scale": 1.0,      # conf = sigmoid(margin / tau_scale)
    "geo_guard": 0.9,      # 교정 후보의 기하 점수 하한 비율 (0 = 거르지 않음)
    "sym_step_deg": 10,    # 연속 대칭을 이 각도로 이산화
    "publish": False,      # Detection3D.results[1] 로 신뢰도까지 발행할지
    "dump_topk": 0,        # >0 이면 후보 목록을 검출 행에 덤프(보고서용, 판정 무관)
    "dump_quats": False,   # shortlist 전체를 사원수로 덤프(분석용, 판정 무관)
    # 자동 유도 금지 — 소각 대칭이 곰·공룡에 가짜 대칭을 만들었던 전례가 있다.
    # objectmemory core/fusion.py 의 손으로 쓴 보수적 표를 그대로 옮긴 것이다.
    "symmetry_axes": {
        "Sikhye_high": [0.0, 0.0, 1.0],
        "Sauce_high": [0.0, 0.0, 1.0],
        "Mugcup_high": [0.1057, 0.0337, 0.9938],
    },
}


# ── 마스크 게이트 (ch1) ────────────────────────────────────────────────────
# 기하 점수로 정렬한 뒤·텍스처 검증 앞에서, 후보 자세로 CAD 를 역투영한 실루엣과 ISM 마스크를
# 맞대어 후보 자격을 깎는다. **점수에 항을 더하지 않는다** — 색을 점수로 더했다가 FP 가 3~4 배
# 터지고 하드 게이트로 바꿔 −61% 를 얻은 전례(HSV)를 따른다.
#
# 단일 IoU 로는 두 가지를 못 가른다. 그래서 상보적인 항을 따로 둔다:
#   cov  = |Mo∩Mr|/|Mo|   렌더가 마스크를 설명하는가 (가림에 강건)
#   prec = |Mo∩Mr|/|Mr|   렌더가 마스크 밖으로 새는가
#   r_area = |Mr|/|Mo|    크기 정합 — '초코로 잘못 인식된 갈색 박스'(렌더가 확연히 작다)
#   dtheta = 주축 각도차   방향·형상 — 'saffron 축 어긋남'
# ★ 가림과 오인식은 r_area 를 **반대 방향**으로 민다(가림 ↑ / 더 큰 물체 위 오인식 ↓).
#   그래서 r_area 는 **하한만** 건다. 양쪽을 거는 순간 가림에서 무너진다.
DEFAULT_MASK_GATE = {
    "enabled": True,
    # -- 렌더 --
    "grid": 32,            # 스플랫 격자 G. CAD 1024 점 기준 칸당 ~2 점(0.4·G² 점유 가정)
    "dilate": 1,           # 3x3 max-pool 팽창 횟수 (스플랫 구멍 메우기)
    "topk": 100,           # 기하 상위 몇 개를 채점할지 (appe topk 와 맞춘다)
    "z_min": 1.0e-3,       # 이보다 앞이면 그 점은 버린다 (카메라 뒤/영점)
    "mask_occ": 0.0,       # 이 점유율을 넘는 칸을 관측 마스크로 본다 (0 = 조금이라도 덮였으면)
    # -- 결합 점수 S = cov · prec^w_p · shape --
    "score_terms": "cov,prec,shape",   # 스윕 결과에 따라 "cov" 단독으로도 갈 수 있게
    "w_p": 0.5,            # prec 지수
    "theta0": 30.0,        # shape = exp(-dtheta/theta0)
    "ecc_min": 1.6,        # 관측 마스크 이심률이 이 아래면 주축이 잡음 → dtheta 항 사용 안 함
    # -- (A) 후보 게이트: 상대 --
    "gate_guard": 0.85,    # S >= gate_guard * S.max() 인 후보만 통과
    "min_keep": 5,         # 그래도 S 상위 이만큼은 남긴다
    "min_bbox_px": 24,     # bbox 한 변이 이보다 작으면 게이트 해제(격자 양자화 잡음)
    # -- (B) ISM 오인식 판별: 절대. **기본은 표시만, 거절하지 않는다** --
    # 판별 근거는 **cov_best 와 r_area_best** 다. S 에는 prec 항이 있어 가림에 민감해서
    # 오인식과 섞인다(합성 실측: 오인식 cov .33/r_area .33, 가림 cov .93/r_area 1.71).
    "ism_reject": False,   # true 면 의심 검출을 실제로 버린다 (실측 보정 전엔 켜지 말 것)
    "ism_flag_below": 0.5,  # cov_best 하한
    "r_area_min": 0.5,      # r_area_best 하한 (상한은 걸지 않는다 — 가림이 올리는 방향)
    # -- 진단 --
    "dump": False,         # 후보별 항을 verify.cands 에 덤프(분석용, 판정 무관)
    "dump_cases": True,    # 후보가 지워진 케이스를 gate_cases.jsonl 로 보관(사후 분석용)
    "max_cases": 5000,
}

# ── Map 사전 게이트 (ch2) ──────────────────────────────────────────────────
# SLAM Map 기준으로 같은 객체의 자세가 수렴하면 등록하고, 그 뒤로는 가림·화면잘림으로 마스크가
# 나빠도 등록 자세와 맞으면 통과시킨다. ch1 과 **합집합**이다(교집합이면 구제가 안 된다).
#
# ★ 자기강화 차단: 투표는 '쉬운 프레임'에서만 받는다 — verdict PASS + 승자가 ch1 으로 통과했을 것.
#   ch2 로만 들어온 후보는 절대 ch2 에 투표하지 못한다.
# ★ 사전은 자격만 주고 승자를 뽑지 않는다. 승자는 여전히 텍스처 점수 argmax 다.
DEFAULT_MAP_PRIOR = {
    "enabled": True,
    # -- 포즈 공급 --
    "pose_source": "both",         # slam_extrinsic | self_localization | both | trajectory_file | none
    "pose_topic": "/orbslam3/pose",          # slam_extrinsic 용 (SLAM 카메라)
    "self_pose_topic": "/orbslam3_sam/pose", # self_localization 용 (SAM 카메라 only-loc)
    "extrinsic": "",               # 4x4 .npy 경로. T_map_camSAM = T_map_camSLAM · X
    "trajectory": "",              # TUM 궤적 파일 (trajectory_file 모드, bag 검증용)
    "buffer_window_s": 10.0,
    "pose_tol_s": 0.05,
    "max_gap_s": 0.20,             # 이보다 벌어진 구간은 보간하지 않는다(추적 끊김을 메우지 말 것)
    # -- 자격 부여 --
    "admit_deg": 20.0,             # 등록 자세와 이 안이면 통과. 사전은 fine, 후보는 coarse 라 여유가 필요
    # -- 투표 조건 (전부 만족해야 표 1) --
    "vote_min_px": 40,
    "vote_need_pass": True,        # Stage V verdict == PASS
    "vote_need_mask": True,        # ★ 승자가 ch1 으로 통과했을 것 (고리 차단기)
    # -- 등록/갱신/강등 --
    "tol_deg": 30.0,               # 같은 봉우리로 볼 각도 (ObjectMemory 모드 융합과 같은 값)
    "min_votes": 20,
    "min_share": 0.6,              # 최빈 모드 점유율
    "freeze": False,               # false = 갱신 가능(최빈 모드가 바뀌면 등록 자세도 바뀐다)
    "demote_window": 40,
    "demote_share": 0.35,
    "reset_on_jump_m": 1.0,        # SLAM 재위치추정 점프 감지 시 전체 초기화
    # -- 인스턴스 --
    "pos_tol_m": 0.30,
    "multi_instance": {"choco_hazelnut_high": 2},   # 실물이 2 개인 것은 choco 뿐(사용자 확인)
    "persist": "",                 # 기본 실행 단위. 경로를 줘야만 읽고 쓴다
}


def _merge(rt, key, defaults):
    """`verify` 와 **똑같은 3 분기 규칙**으로 블록 하나를 병합한다.

    키 없음 → 기본값 그대로 / 명시적 null → 끄기 / dict → 기본값 위에 덮어쓰기.
    세 블록이 같은 규칙을 써야 "노드는 되는데 split 은 안 된다"가 안 생긴다.
    """
    v = rt.get(key, UNSET)
    if v is None:                      # 명시적 null = 끄기
        v = {"enabled": False}
    elif v is UNSET:                   # 키가 없으면 기본값
        v = {}
    return dict(defaults, **(dict(v) if isinstance(v, dict) else {}))


def build_appe_cfg(rt):
    """runtime 설정 → (PEM 에 넘길 appe_rerank dict 또는 None, verify dict 또는 None).

    * verify / mask_gate / map_prior 중 하나라도 켜져 있으면 재정렬(appe_rerank)은 자동으로
      켜진다 — 셋 다 그 후보 목록 위에서 도는 단계라 따로 켤 이유가 없다.
    * 키가 없으면 기본값(ON)이 적용된다. 끄려면 `verify: null` / `mask_gate: null` /
      `map_prior: null` (또는 `{enabled: false}`) 를 명시해야 한다.
    * 전부 꺼지면 None 을 돌려주고, 그러면 PEM 은 예전 경로(기하 argmax) 그대로다.

    ⚠ **2-튜플 반환을 유지한다.** sam6d_core / sam6d_realtime_node / temp/verify_eval /
      temp/verify_report 네 곳이 두 개로 언팩한다. 새 블록은 ar 안에 실어 보낸다.
    """
    ar = rt.get("appe_rerank")
    ver = _merge(rt, "verify", DEFAULT_VERIFY)
    mg = _merge(rt, "mask_gate", DEFAULT_MASK_GATE)
    mp = _merge(rt, "map_prior", DEFAULT_MAP_PRIOR)
    on = bool(ver.get("enabled")) or bool(mg.get("enabled")) or bool(mp.get("enabled"))
    if not ar and not on:
        return None, ver
    ar = dict(DEFAULT_APPE, **(dict(ar) if isinstance(ar, dict) else {}))
    ar["verify"] = ver
    ar["mask_gate"] = mg
    ar["map_prior"] = mp
    return ar, ver


def _gate_clause(ar):
    """마스크 게이트 / Map 사전 부분의 로그 문구. split 과 노드가 같은 줄을 찍어야
    '노드는 되는데 split 은 안 된다'를 기동 즉시 알아챌 수 있다."""
    mg = (ar or {}).get("mask_gate") or {}
    mp = (ar or {}).get("map_prior") or {}
    out = ""
    if mg.get("enabled"):
        out += (f" · 마스크 게이트 ON: G={mg['grid']} topk={mg['topk']} "
                f"guard={mg['gate_guard']} min_keep={mg['min_keep']} terms={mg['score_terms']}"
                f" ism={'거절' if mg.get('ism_reject') else '표시만'}")
    else:
        out += " · 마스크 게이트 OFF"
    if mp.get("enabled"):
        out += (f" · Map 사전 ON: src={mp['pose_source']} admit={mp['admit_deg']}° "
                f"등록={mp['min_votes']}표/{mp['min_share']}")
    else:
        out += " · Map 사전 OFF"
    return out


def describe(ar, ver):
    """기동 로그 한 줄."""
    if not ar:
        return ("[pem] 외형 재정렬 OFF · 텍스처 검증 OFF · 마스크 게이트 OFF · Map 사전 OFF "
                "(기하 점수만 — 예전과 동일)")
    base = f"[pem] 외형 재정렬 ON: topk={ar['topk']} w_geo={ar['w_geo']} stride={ar['stride']}"
    if not (ver and ver.get("enabled")):
        return base + " · 텍스처 검증 OFF" + _gate_clause(ar)
    return (base + f" · 텍스처 검증 ON: w_col={ver['w_col']} geo_guard={ver['geo_guard']} "
            f"tau(pass/corr)={ver['tau_pass']}/{ver['tau_corr']} publish={ver['publish']}"
            + _gate_clause(ar))
