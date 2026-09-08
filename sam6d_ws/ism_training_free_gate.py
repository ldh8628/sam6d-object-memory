#!/usr/bin/env python3
"""ism_training_free_gate.py — training-free gate 결정 규칙 (운영 authority, 학습 0).

Phase 2.2 검증에서 채택된 규칙만 여기 둔다. 어떤 학습·GT·객체별 파라미터도 없다.
파라미터는 전부 {운영 threshold, 고정 비율}에서만 유도(코드에 상수로 고정).
운영 통합은 config-gated 기본 OFF. OFF/미지원 method → 항상 무개입(Phase 1C 유지).

지원 method:
  phase1c               (기본) 무개입.
  one_borderline_rescue 선택 후보가 정확히 1개 gate만 borderline-fail이고 나머지 2개 strong-pass
                        이면 rescue. HSV severe-fail은 절대 rescue 금지(FP 폭증 방지).
                        운영 경로 특성상 semantic-borderline은 후단 미도달 → appe/HSV borderline만 도달.
"""

BORDERLINE_RATIO = 0.90     # score >= ratio*thr → borderline(경계) fail
SEVERE_RATIO = 0.50         # score <  severe*thr → severe fail (rescue 금지)
STRONG_FRAC = 0.10          # 통과 gate가 thr + frac*(1-thr) 이상이면 strong
SCORE_MAX = 1.0
VERSION = "tf-gate-1.0-2026-07-22"


def _strong(score, thr):
    return score >= thr + STRONG_FRAC * (SCORE_MAX - thr)


def one_borderline_rescue(sem, appe, hsv, sim_thr, appe_gate, hsv_thr,
                          sem_ok, appe_ok, hsv_ok):
    """정확히 1개 gate만 borderline-fail & 나머지 2개 strong → True(rescue). 근거 문자열 반환.

    sem_ok/appe_ok/hsv_ok = 각 gate 통과여부(운영 판정과 동일 입력).
    반환 (rescue:bool, reason:str).
    """
    scores = {"S": (sem, sim_thr, sem_ok), "A": (appe, appe_gate, appe_ok), "H": (hsv, hsv_thr, hsv_ok)}
    fails = [g for g, (_, _, ok) in scores.items() if not ok]
    if len(fails) != 1:
        return False, f"no_rescue(fails={len(fails)})"
    g = fails[0]; sc, thr, _ = scores[g]
    if g == "H" and sc < SEVERE_RATIO * thr:
        return False, "no_rescue(hsv_severe)"
    if sc < SEVERE_RATIO * thr:
        return False, f"no_rescue(severe_{g})"
    if sc < BORDERLINE_RATIO * thr:
        return False, f"no_rescue(not_borderline_{g})"
    others_strong = all(_strong(scores[x][0], scores[x][1]) for x in ("S", "A", "H") if x != g)
    if others_strong:
        return True, f"rescue_borderline_{g}"
    return False, f"no_rescue(others_weak_{g})"


def decide(method, res, o):
    """운영 hook. res=recognize_frame 결과 dict, o=object config.
    method OFF/phase1c 또는 이미 accept면 무개입(False 반환=변경없음).
    반환 (changed:bool, reason:str). changed=True면 caller가 res를 accept로 바꾼다.
    """
    if method in (None, "", "phase1c"):
        return False, "off"
    if res.get("accepted"):
        return False, "already_accepted"
    if method == "one_borderline_rescue":
        # 운영에서 semantic-fail 후보는 masked_appe 미계산. fail-safe: 필수 score 없으면 무개입.
        if res.get("decision", "").startswith("no-object(below-sim)") or res.get("masked_appe") is None:
            return False, "unreachable(sem_stage)"
        sem = float(res.get("best_sem", 0.0)); appe = float(res.get("masked_appe", 0.0))
        hsv = float(res.get("hsv_score", 1.0))
        sim_thr = float(o.get("similarity_threshold", 0.35)); appe_gate = float(o.get("appe_gate", 0.55))
        hsv_thr = float(o.get("hsv_gate_threshold", 0.1214))
        sem_ok = sem >= sim_thr; appe_ok = appe >= appe_gate
        hsv_ok = hsv >= hsv_thr if o.get("hsv_gate_enabled") else True
        rescue, reason = one_borderline_rescue(sem, appe, hsv, sim_thr, appe_gate, hsv_thr,
                                               sem_ok, appe_ok, hsv_ok)
        return rescue, reason
    return False, f"unknown_method({method})"
