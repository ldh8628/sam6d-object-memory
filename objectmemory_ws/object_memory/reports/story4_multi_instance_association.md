# Story 4 — Multi-Instance Association (same-class instances)

> 상태: 구현 완료 · 검증 완료(4 SAM bag + 유닛) · 2026-07-07
> 코드: `src/pipeline/object_memory_runner.py`, `src/core/memory_store.py`
> 검증 도구: `scripts_test/eval_multi_instance.py`, `scripts_test/test_multi_instance.py`

## 문제 (이전 MVP의 한계)

이전 러너는 클래스명당 **live 랜드마크 1개**만 유지했다(`object_ids_by_name: Dict[str,int]`).
결과로 (a) 같은 클래스의 두 물체가 공존 불가, (b) **첫 pose가 오염되면** 이후 정상
검출이 게이트에서 계속 reject → 랜드마크 소멸 → 동명 재시드가 반복되어 하나의 물체가
여러 id로 **파편화**되었다(loop1 milk id 1/5/8, loop2 choco id 3/10/11/13/14).

## 변경 (multi-instance, Story 4/5 최소형)

- 클래스명 → **live 인스턴스 id 리스트**(`live_ids_by_name: Dict[str,List[int]]`).
- 검출을 맵 공간에서 **같은 클래스의 최근접 live 인스턴스**에 결합(병진 `assoc_trans_gate_m`
  이내; 회전 게이트는 PEM 회전 노이즈로 **기본 off**). 프레임 내 **greedy 1:1 배정**
  (거리 오름차순, 인스턴스·검출 각 1회).
- **어떤 인스턴스에도 안 맞으면 reject가 아니라 새 tentative 인스턴스 spawn.**
  진짜 2번째 물체, 또는 오염 pose에 가려졌던 정상 클러스터가 자기 트랙을 형성한다.
  노이즈성 spawn은 existence 필터(Bernoulli r + FOV P_D)가 정리한다.
- lifecycle/existence/visibility·INV-008은 **인스턴스별로** 그대로 적용.
- 운영 기본값(사용자 결정): `assoc_trans_gate_m=0.20`, 회전 게이트 off.

## 검증 설계 (외부 GT 없음)

4개 SAM bag은 **물리적으로 클래스당 1인스턴스**다. 따라서 실데이터 목표는 "인스턴스를
늘리는 것"이 아니라 **재관측을 한 id로 유지(파편화↓) + phantom 통제 + 무회귀**다.
진짜 공존(2개 동시)은 데이터에 없으므로 **유닛/합성**으로 별도 검증한다.

- Prong 1 (실데이터): `eval_multi_instance.py`로 반경 0.10/0.15/0.20/0.30 스윕, 클래스별
  생존(active+remembered) 인스턴스 수·지배 인스턴스 관측수·총 id수 집계.
- Prong 2 (유닛): `test_multi_instance.py` 5종 — 2인스턴스 분리 유지, 프레임 내 1:1 배정,
  재방문 동일 id, 근접 재관측 파편화 없음, 회전 게이트 off. **전체 70 pass**.

## 결과 — 반경 0.20 (운영값), before=이전 fixr

| bag | 판정 | 근거 |
|---|---|---|
| **SAM_loop1** | ✅ 파편화 해소 | 모든 존재 클래스 **생존=1**(multi_surv=[]). milk **3 id→1 id**(obs 58→**82**), Febreze 4 id의 생존 1(dom obs 9→**15**), saffron dom 96→**109**, choco 생존 1(obs 54). |
| **SAM_loop2** | ✅ 파편화 해소 | 생존=1/클래스. Mugcup 4 id→**생존 1**(obs 36→**59**), Bear obs 5→**27**. choco는 상류 PEM 불량(z≈2.4~2.6)이라 **전부 deleted로 정확 기각**. |
| **SAM_occlusion** | ✅ 무회귀 | Bear/milk 각 **생존 1**. 0.20에서 총 id **3**(phantom 최소, 0.10/0.15에선 5). |
| **SAM_circle** | ⚠️ 경미한 과생성 | Bear/milk/Febreze 생존 1(Febreze는 전보다 개선). 단 **Mugcup·saffron가 생존 2** — 저관측(obs 2~3) 불량-pose 클러스터가 active로 승격. **모든 반경에서 동일** → 반경이 아니라 promotion 정책 이슈. |

### 반경 스윕 요지 (총 id = spawn된 전체, 대부분 deleted phantom)

| bag | 0.10 | 0.15 | 0.20 | 0.30 | 생존/클래스 |
|---|--:|--:|--:|--:|---|
| loop1 (총 id) | 24 | 22 | **18** | 16 | 전 반경 생존=1 (0.30이 phantom 최소) |
| loop2 (총 id) | 19 | 19 | **18** | 17 | 전 반경 생존=1 |
| occlusion (총 id) | 5 | 5 | **3** | 3 | 넓을수록 phantom↓ |
| circle multi_surv | Mug,saf | Mug,saf | Mug,saf | Mug,saf | 반경 무관(정책 문제) |

→ **0.20이 phantom(총 id)과 병합안전의 균형점**. 0.30은 loop1 phantom을 더 줄이나 진짜
2인스턴스 병합 위험 증가(현 데이터엔 진짜 공존 없어 미노출).

## 결론

- **핵심 성과**: "오염 첫 pose → 순차 재시드" 파편화가 loop1/loop2에서 해소됨. 지배 인스턴스가
  관측을 한 id에 집약하고, **클래스당 생존 인스턴스=1**로 수렴. choco처럼 상류 pose가 진짜
  불량인 클래스는 existence가 정확히 전량 기각.
- **남은 한계 (circle)**: 저관측(2~3 obs) 불량-pose 클러스터가 `PROMOTE_HITS=2`로 쉽게 active
  승격 → 단일 물체가 2개 active로 분리. **spawn-on-miss가 이 클러스터에 승격 기회를 더 줌.**
  반경으로는 해결 불가.

## 다음 액션 제안 (사용자 결정 필요)

**판정: GO** — 의도한 파편화 해소를 loop1/loop2에서 달성, occlusion 무회귀, 70 test green.
단 circle의 저관측 2차-인스턴스 과승격은 별도 튜닝 필요.

선택지:
- **(A) 2차 인스턴스 승격 강화** *(추천)* — 같은 클래스의 **2번째 이상 인스턴스**에 한해
  `PROMOTE_HITS`↑(예 3~4) 또는 생존에 최소 관측수 요구. circle 과생성만 억제하고 loop1/2
  성과는 보존하는지 재검증.
- **(B) 진짜 공존 데이터 확보** — 동일 클래스 2개를 실제 배치한 bag를 새로 수집해 공존
  분리 정확도를 실데이터로 검증(현재는 유닛/합성만).
- **(C) 현 상태 동결** — circle 과생성을 알려진 한계로 문서화하고 상류 PEM pose 품질(D2)
  개선을 우선.

→ 어느 쪽으로 진행할까요? (추천: A — 값싼 정책 변경으로 유일한 회귀를 제거)
