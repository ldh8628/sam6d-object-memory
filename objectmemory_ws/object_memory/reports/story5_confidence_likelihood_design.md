# Story 5 — Quality-Weighted Confidence (설계 + 구현 결과)

> 상태: **구현 완료 · 4-bag 검증 완료** · 2026-07-08 (승인: 결정1=사후튜닝, 결정2=MVP 3특징, 결정3=병행후전환)
> 선행 진단: gif 오탐지 문제 (a) 카메라-겹침, (b) 미소멸 → `reports/story4_*` 후속
> 대상 코드: `src/core/existence.py`, `src/pipeline/object_memory_runner.py`
> 원칙: 파이프라인 내부는 **연속 confidence 하나**, 임계값은 **출력단으로 이동**.

---

## 1. 동기 (무엇을 고치려는가)

진단된 두 문제의 **공통 뿌리는 하나**다: 현재 존재확률 r 갱신이 **검출 품질을 무시하는 히트 카운터**다.

- 모든 accepted 검출이 r을 **동일하게** 올린다 (`update_existence_detected`의 `P_D`, `c` 가 상수).
- 그래서 z≈0 퇴화 pose·저신뢰·큰잔차 검출도 정상 검출과 **같은 증거값**을 가진다.
- 승격/은퇴는 **관측 횟수 게이트**(`PROMOTE_HITS=2`, `MIN_OBS_LONGTERM=5`)에 의존 → 스치는 오검출 2번이면 active.

**목표**: r을 "이 랜드마크가 진짜 객체일 확률"로 만들고, **증거를 품질로 가중**해 오탐지 속에서 자기보정하게 한다.

---

## 2. 현재 모델 재정식화 (핵심 관찰)

현재 detected 업데이트는

```
r' = r·P_D / ( r·P_D + (1−r)·c )
```

이를 **odds** = r/(1−r) 로 쓰면

```
odds' = odds · (P_D / c)          # P_D=0.6, c=0.1  →  비율 6 (모든 검출 고정)
logit(r') = logit(r) + log(P_D/c) # 로그오즈는 가법적
```

즉 **모든 accepted 검출이 log-odds에 상수 log(6)≈1.79 를 더하고 있다.** miss 업데이트
`r' = r(1−P_D)/(1−r·P_D)` 는 P_D 가중 Bayes 감쇠로 **직교하며 그대로 둔다**(시야 밖 P_D=0 → 무감점, fix-r 유지).

---

## 3. 제안 모델 — 품질가중 가능도비 Λ

상수 log(6)을 **검출별 가능도비의 로그**로 교체한다. 로지스틱 형태(로그오즈가 특징의 선형결합):

```
logit(r') = logit(r) + logΛ_i
logΛ_i    = Σ_k  w_k · φ_k(z_i)        # 특징 φ, 가중 w
```

- `logΛ_i > 0` → 진짜 객체 증거 (r ↑)
- `logΛ_i ≈ 0` → 중립 (정보 없음)
- `logΛ_i < 0` → clutter 증거 (r ↓, **accepted여도 r을 깎을 수 있음** ← 자기보정의 핵심)

`logΛ_i` 는 `[logΛ_min, logΛ_max]` 로 클램프(단일 검출이 트랙을 지배/파괴하지 못하게).

### 3.1 특징 φ_k (MVP는 3개)

| φ | 정의 | 의미 | 부호 |
|---|---|---|---|
| **φ_visib** | `log(P_D / c)` (현행 항 유지) | 시야 안이면 기본 탐지 가능성 | 시야 밖이면 애초에 accepted 없음 |
| **φ_score** | `s − s_mid` (s = sam6d_score, s_mid=0.5) | 검출기 신뢰도 | s>0.5 → +, s<0.5 → − |
| **φ_depth** | z_cam 물리성: `z<z_min(0.05m)`거나 `z>z_max(5m)` → **−D_hard**, 정상 → 0 | 퇴화/비현실 pose 강한 clutter | z≈0 → 큰 − |
| **φ_resid** | 기존 트랙과의 잔차: `−clamp(d_trans / gate, 0, 1)` (신규 spawn은 트랙 없음 → 0) | 트랙과 어긋나면 신뢰 낮춤 | 잔차 클수록 − |

> **핵심 효과**: (a) z≈0 카메라-겹침 pose는 `φ_depth = −D_hard` 로 `logΛ_i ≪ 0` → **r이 쌓이기는커녕 하락 → 승격 불가 → 자기보정.** 하드 z-cut(A안)은 이 특징 하나의 특수 케이스.

### 3.2 신규 spawn = 낮은 prior + 첫 검출 품질

현재 `R_INIT=0.7`(무조건 높게 시작) → **모든 유령이 절반쯤 진짜로 태어남**. 변경:

```
r_prior = 0.15                       # 아직 미확증 단일 검출의 사전확률(낮게)
logit(r0) = logit(r_prior) + logΛ_1  # 첫 검출 품질이 시작점을 결정
```

- 좋은 첫 검출 → 쓸만한 r0
- z≈0·저신뢰 첫 검출 → r0 여전히 낮음 → 다음 프레임 즉시 소멸 후보 (문제 b 완화)

### 3.3 log-odds가 주는 비대칭 이력 (공짜)

로그오즈 누적은 자동으로:
- **약한 트랙(낮은 |logit|)** → 반대증거로 쉽게 뒤집힘 → **오탐지 즉시 사라짐**
- **확립된 트랙(높은 logit)** → 뒤집으려면 많은 반대증거 → **진짜 객체 안 깜빡임**

현행 "모두 R_INIT=0.7에서 동일하게 ~5 miss"의 대칭·부적절 이력을 대체.

---

## 4. confidence 밴드 — 카운트 게이트 제거

상태(active/lost/remembered/…)는 **유지하되 r의 순수 함수**로 파생. `PROMOTE_HITS`,
`MIN_OBS_LONGTERM` 제거.

| 전이 | 현행(카운트+임계) | 제안(순수 r 밴드) |
|---|---|---|
| tentative → active | obs≥2 **and** r>0.6 | **r ≥ τ_active(0.6)** (증거 질량 요건; 훌륭한 1검출 또는 준수한 2검출이면 통과, 쓰레기 2검출은 미달) |
| active → lost | r<0.3 (in-view miss) | **r < τ_lost(0.3)** (동일) |
| lost → remembered | 시야밖 **and** obs≥5 | **시야밖 지속 and r ≥ τ_remember(0.5)** (누적 증거로 대체) |
| any → deleted | r<0.05 등 | **r < τ_retire(0.05)** (+ tentative_max_age 안전캡 유지) |

**출력(소비자) 임계는 단 하나**: `published ⇔ r ≥ τ_show(0.5) ∧ status∈{active, remembered}`
→ viz/Unity는 이 뷰만 소비 (story: viz는 이미 active+remembered만 그림).

밴드 기본값(초안, 4-bag 튜닝 대상):
`τ_retire=0.05, τ_lost=0.30, τ_show=0.50, τ_active=0.60, τ_remember=0.50`
가중 초안: `w_visib=1.0(=현행), w_score=2.0, w_depth=1.0(D_hard=4.0), w_resid=1.5`
클램프: `logΛ ∈ [−4.0, +2.5]`

---

## 5. 유지되는 것 (변경 없음)

- **가시성 P_D (`visibility.py`)** — confidence와 **직교**. 시야 밖 무감점(fix-r) 그대로.
- **pose 융합 (`fusion.py`)** — 증가평균. 단, §7 참고(품질가중 융합은 후속).
- **다중 인스턴스 association** (story4) — 게이트/greedy 1:1 그대로. Λ는 accept 이후 r 갱신에만 관여.
- **INV-002**(id 소유·불변), **INV-008**(SLAM 없으면 동결).

---

## 6. 코드 변경 지도 (구현 시)

1. `core/existence.py`
   - 신규 `log_likelihood_ratio(features, weights) -> float` (§3.1) + 클램프.
   - `update_existence_detected` 를 `logit(r) + logΛ` 형태로 일반화(현행은 `logΛ=log(P_D/c)` 특수 케이스 → **하위호환**).
2. `core/features.py` (신규, 순수함수)
   - `detection_features(det, T_cam_obj, track_lm|None, p_d, clutter) -> dict(φ)`.
3. `pipeline/object_memory_runner.py`
   - accept 분기에서 Λ 계산 후 r 갱신.
   - spawn: `r_init` → prior+Λ_1.
   - 전이: 카운트 게이트 제거, r 밴드로 교체. `PROMOTE_HITS`, `MIN_OBS_LONGTERM` 제거(또는 deprecate).
   - 상수 → 파라미터로 노출(러너 시그니처 + CLI), **라이브러리 기본값은 현행 동작 보존**해 테스트 회귀 방지 후 단계적 전환.

---

## 7. 명시적 비목표 / 후속

- **식별(identity) confidence** — 오라벨(세제→우유) 보정은 별도 축. 이번은 **존재확률**만.
- **품질가중 pose 융합** — 저품질 검출을 융합에서도 낮게(현재 균등평균). Λ 재사용 가능하나 후속.
- **맵-표면 기하 일치 φ_geom** — 강력하나 SLAM 밀집맵 질의 필요 → 후속 특징.
- **Unity 연동** — 하지 않음.

---

## 8. 검증 설계

- **Prong 1 (실데이터 before/after)** — 4 SAM bag. 지표:
  (i) 카메라 0.15m 이내 생존 랜드마크 수(=문제 a) → **0 지향**,
  (ii) 유령 수명 분포(문제 b) → 저품질 spawn 수명 **↓**,
  (iii) 진짜 객체 생존/클래스=1 **무회귀**(story4 성과 보존),
  (iv) circle 과승격(Mugcup·saffron 2차 active) **해소**.
  도구: `eval_multi_instance.py` 확장(품질 지표 추가).
- **Prong 2 (유닛)** — 신규 `test_confidence_likelihood.py`:
  퇴화 pose가 승격 못 함 / 고품질 1검출이 저품질 2검출보다 r 높음 / 확립 트랙이 단발 반대증거에 안 죽음 / 시야 밖 무감점 유지 / 하위호환(logΛ=log(P_D/c)면 현행 수치 재현).
- 기존 70 pytest **green 유지**(라이브러리 기본값 보존 전략).

---

## 9. 리스크

- **GIGO** — 가능도 모델이 나쁘면 악화. → MVP는 특징 3개로 최소화, 4-bag로 가중 보정.
- **가중 과적합** — 4개 bag에 맞춰 튜닝 시 일반화 우려. → 물리적으로 해석 가능한 값으로 고정(예 z_min=0.05m는 데이터 아닌 물리), 데이터 튜닝은 소수만.
- **테스트 회귀** — 기본값 보존 + 단계적 전환으로 통제.

---

## 9.5 구현 결과 (4-bag before/after) — 승인 후 실측

구현: `core/features.py`(신규), `core/existence.py`(`update_existence_loglr`),
`pipeline/object_memory_runner.py`(`quality_weighting` 스위치). **하위호환 항등식
`update_existence_loglr(r, log(P_D/c)) == update_existence_detected` 유닛 검증**,
기존 70 + 신규 9 = **79 pytest green**. 도구 `scripts_test/eval_confidence.py`.

측정값 (assoc 0.20m, 회전 off; camOvlp=카메라 0.15m내 생존 랜드마크, lifeMed/Max=유령 수명(검출프레임)):

| bag | 모드 | 생존 | camOvlp | 유령수명(중앙/최대) | 다중생존(과승격) |
|---|---|--:|--:|--:|---|
| **circle** | legacy | 7 | **2** | 22 / 22 | **[Mugcup, saffron]** |
| **circle** | **quality** | 5 | **0** | **1** / 18 | **[]** |
| loop1 | legacy | 6 | 0 | 11 / 33 | [] |
| loop1 | quality | 6 | 0 | **3** / 30 | []  (choco 보존) |
| loop2 | legacy | 5 | 0 | 11 / 45 | [] |
| loop2 | quality | 5 | 0 | **3** / 40 | [] |
| occlusion | legacy | 2 | 0 | 5 / 5 | [] |
| occlusion | quality | 2 | 0 | 4 / 4 | [] |

**성과 (진단 문제 정면 해결)**:
- **(a) 카메라-겹침 제거** — circle **2→0**. z≈0 퇴화 pose가 `φ_depth`로 음의 증거 → 승격 불가.
- **(b) 유령 즉시 소멸** — 유령 수명 **중앙 11→3(loop), 22→1(circle)**. log-odds 비대칭 이력으로 약한 트랙이 빠르게 사망.
- **circle 과승격 해소** — Mugcup 2→1(obs=3 유령 제거, obs=18 진짜 보존), saffron 2→1(obs=2 유령 제거). **story4 최대 회귀 제거.**
- **무회귀** — occlusion·loop2 생존/클래스 유지, 70 기존 테스트 green.

**경계 사례 해소 — jitter는 존재 증거가 아니다 (사용자 통찰 반영, `w_resid` 기본 off)**:
- 1차 실측에서 loop1 choco가 active→lost로 강등됐다. 원인은 `φ_resid`가 **프레임간 pose
  jitter(잔차 최대 0.19m)를 벌점**으로 먹은 것. 그런데 choco는 **실물**(circle·loop1·loop2 모두 존재).
- **핵심 교정**: within-gate 잔차 = 위치추정 노이즈이지 "객체 존재" 증거가 아니다. jitter는 **pose 융합**에서
  평균으로 흡수하고, gross outlier는 **association 게이트**가 이미 처리한다. 따라서 존재확률은 잔차를 무시해야 한다.
  → **`w_resid` 기본 0**(feature는 opt-in으로 유지). 전역값이라 객체별 임계 불필요.
- 결과: loop1 choco **복구(생존=1)**, circle 과승격·camOvlp·유령수명 개선 **전부 유지**, 79 test green.
- **여전히 안 살아나는 choco는 상류 문제(D2, confidence 소관 아님)**:
  circle=검출 2개·score 0.16(SAM-6D recall 부족), loop2=검출 7개·**depth 2.53m 오위치**(PEM pose 오류).
  존재확률로 "만들어낼" 수 없는 상류 결함이므로 별도 트랙.

## 9.6 P_D 보정 — 검출기 recall 갭에서 실물이 Lost로 깜빡이는 문제

**증상**: loop1 choco가 시야 안에 있는데도 SAM-6D가 절반만 검출(**in-view recall=0.48**,
detected 56 / in-view-miss 61) → in-view miss로 confidence 감쇠 → active↔lost **16회 왕복** →
맵(active+remembered만)에서 깜빡임.

**원인**: `PD_BASE`(기대 in-view 검출률)가 **0.6**으로, 실측 0.48보다 **과대추정**. miss 벌점
`r'=r(1-P_D)/(1-rP_D)`은 P_D가 클수록 커지므로, 정상적인 검출 누락에 과벌점 → 조기 Lost.

**교정**: `PD_BASE 0.6 → 0.5`(실측에 맞춤). 1줄 전역값(객체별 임계 없음).
- 결과: choco 최종 r **0.64→0.92**, 깜빡임↓, loop1 생존 클래스 6→7(choco 안정 포함).
- **4-bag 무회귀**: circle 과승격·camOvlp 여전히 0, 삭제(유령) 개수 불변, 79 test green.
- **안전 근거**: 유령 억제는 quality weighting(검출 쪽)이 담당하므로 miss 쪽을 관대하게 해도
  유령이 안 늘어남(삭제수 불변으로 실증).

**한계**: P_D 보정만으로 깜빡임이 **완전히** 사라지진 않음(잔여). 완전 제거는 검출기 recall
자체를 올려야 함 → 아래 D2 백로그.

---

## 9.7 백로그 — SAM-6D recall 근본 개선 (D2, sam6d_ws, 예약)

objectmemory 소관 밖. choco 등 저-recall 객체를 시야에서 더 자주 검출하도록 SAM-6D ISM 개선:
- ISM 검출 임계값 하향(과거 감사: 0.4에서 FN 상당 회복; FN=proposal-miss 68%+cls-gate 32%).
- choco 프롬프트 색/형태 속성화(open-vocab 매칭 강화).
- loop bag ISM(+PEM) 재실행 → detection 재생성.
→ **다음 SAM-6D 재실행 시 묶어서 처리.** 지금은 §9.6 P_D 보정으로 증상 완화하고 실시간으로 진행.

---

## 10. 다음 액션 제안 (사용자 결정 필요)

**판정: MVP 구현·검증 완료, 경계사례 해소.** (a) 카메라-겹침 2→0, (b) 유령수명 중앙 11→3/22→1,
circle 과승격 해소, **loop1 choco 보존(jitter 벌점 제거)**, 무회귀, 79 test green. 가중은 전역(객체별 임계 없음).

선택지:
- **(A) quality를 정식 기본값으로 승격** *(추천)* — 검증 끝. 라이브러리 기본은 테스트 안전상 legacy 유지,
  CLI/viz는 이미 quality 기본. 문서/네이밍만 "opt-in→default"로 정리.
- **(B) 상류 SAM-6D 개선(D2)** — circle/loop2 choco는 confidence 소관 밖(검출부족·pose오류).
  진짜 회복하려면 SAM-6D recall/PEM pose 품질 개선 필요.
- **(C) 후속 특징** — φ_geom(맵-표면 일치)로 존재확률 강화, 품질가중 pose 융합.
- **(D) 실시간 비동기 시나리오** — 이전에 미뤄둔 SLAM/SAM 병렬·지연보상·object landmark 실시간화.

→ 제안: **A로 마무리** 후, 다음 큰 방향(B 상류품질 / D 실시간) 선택.
