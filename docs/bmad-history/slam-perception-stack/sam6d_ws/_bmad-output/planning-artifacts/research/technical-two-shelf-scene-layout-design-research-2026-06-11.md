---
stepsCompleted: [1]
workflowType: 'research'
research_type: 'technical'
research_topic: 'Two-Shelf Object Placement & Scene Layout Design for SAM-6D + SLAM Experiments'
research_goals: '2개 매대 + 7객체 배치를 논문 Experiments(single/multi pose·no-object·occlusion·viewpoint·tracking·object-aware SLAM·loop closure·Husky nav)가 모두 가능하도록 scene layout·object placement·trajectory·split·GT·최소수집으로 설계'
user_name: 'Ldh9501'
date: '2026-06-11'
source_verification: true
---

# Technical Research — Two-Shelf Object Placement & Scene Layout Design

> **관점**: 보기 좋은 배치가 아니라 **논문 Experiments를 만들 수 있는** 배치. 이전 [데이터셋 설계 리서치](technical-experimental-dataset-design-sam6d-slam-research-2026-06-11.md)의 5레벨 taxonomy·RQ·GT 레시피를 물리 공간에 구체화.
> **확정 하드웨어 스펙**: Husky A200 **990×670×390mm**(폭 0.67m), D455(f) **min-Z ~0.52m, 이상범위 0.6~6m, 4m서 depth오차 <2%**. → 통로 ≥1.2m, 객체 관측거리 0.6~3m(0.5m 이내 금지).
> **범위 제외**: 6개 CAD 생성 계획(별도). 가정: 매대 ~1.0~1.2m(W)×0.4~0.5m(D)×~1.2~1.5m(H), 실내 개방공간 ≥5×5m.

---

## Executive Summary — 결론 5개

1. **단일 "예쁜" 배치로는 9개 실험을 못 덮는다 — 배치를 실험축에 매핑해야 한다.** 매대 layout은 SLAM(loop closure/LiDAR feature/navigation)을 결정하고, 매대 위 object placement는 인식(multi-object/occlusion/viewpoint/no-object)을 결정한다. 둘을 분리 설계 후 결합.
2. **추천 매대 배치 = "Two-Island 비대칭 L/대각"** — 두 매대를 벽에서 ≥1.2m 띄워 **각 매대를 360° orbit** 가능하게 하고, 두 매대를 잇는 경로가 **figure-8/loop**를 형성하도록 비대칭(서로 다른 각도·벽거리)으로 둔다. 비대칭은 **LiDAR SLAM의 무특징·대칭 함정을 회피**(제약 충족), 두 island는 자연스러운 재방문(loop closure)을 만든다. 통로 폭 ≥1.2m(Husky 0.67m + 양측 마진).
3. **object placement는 "난이도 분리 + 세션마다 재배치"가 핵심.** 한 매대=easy/medium, 다른 매대=hard(밀집·대칭·유사색 인접·원거리)로 난이도를 물리 분리하고, **빈 구역(empty bay)을 반드시 포함**(no-object rejection). **동일 배치 반복 금지** — 배치(arrangement)와 trajectory를 train/val/test로 나눠야 leakage가 없다(같은 bag 프레임 분할=시간상관 leakage).
4. **GT는 매대 좌표계 + AprilTag 보드로 앵커링.** 각 매대에 tag 보드를 고정해 **shelf frame**을 정의 → object 6D는 shelf frame 기준으로 1회 측정하면 카메라가 어디서 보든 재사용(재라벨링 0). 카메라/LiDAR 궤적은 LiDAR-SLAM ICP로, loop event는 재방문 타임스탬프로 기록.
5. **최소 수집 = ~10 bag(~25분).** calib 1 + 난이도 3(orbit) + navigation straight-pass 2 + figure-8 loop 1 + dynamic 1 + long-loop 1 + absent 1. 이걸로 9개 실험을 각각 최소 1회 검증. 추천 다음 단계 = **Quick-Dev(배치 SOP + AprilTag shelf-frame + 6D GT 파이프라인 PoC)**.

---

## Shelf Layout Comparison (Task 1)

평가축: Husky주행 / D455시야 / LiDAR feature / loop유도 / 재관측 / occlusion난이도 / multi-view pose / 안전. (◎우수 ○가능 △제한 ✗불리)

| 기준 | A 나란히(평행) | B L-shape | C 마주보기(통로) | D 분리구역 |
|---|---|---|---|---|
| Husky 주행 | ○ 직선 위주 | ◎ 코너 회전 | ◎ 통로 통과 | ◎ 장거리 |
| D455 시야 | ○ 한 면씩 | ◎ 두 면 각도 | ◎ 양측 동시 | ○ 구역별 |
| **LiDAR feature(비대칭)** | △ 평행=대칭 위험 | **◎ 코너=구조 특징** | △ 평행통로=대칭 위험 | ○ 거리차 |
| **loop closure 유도** | △ | ◎ 코너 루프 | △ 직선통로 약함 | **◎ 왕복 재방문** |
| object re-observation | ○ | ◎ | ◎ 양측 | ◎ |
| occlusion 난이도 조절 | ○ | ◎ | ◎ 깊이 가림 | ○ |
| multi-view pose(orbit) | △ 뒤 막힘 | **◎ 두 면+코너** | ○ 통로측만 | ◎ 각 360° |
| 안전/충돌 | ◎ 단순 | ○ 코너 주의 | △ 좁은통로 위험 | ◎ |

- **A(평행)**: 단순하나 평행=LiDAR 대칭/무특징 위험(제약 위반 소지), orbit 뒤쪽 막힘.
- **B(L-shape)**: 코너가 LiDAR 기하특징을 주고 loop·orbit·재관측을 동시에 지원 — 인식+SLAM 균형 최상.
- **C(마주보기)**: warehouse 통로형 → dense multi-object·양면 동시 관측 강점, 단 단일 직선통로는 loop closure 약하고 좁으면 충돌위험.
- **D(분리)**: 장거리·왕복 재방문(loop/long sequence) 강점, 단 각 scene이 단순해져 dense multi-object 약화.

→ **단일 우승 없음.** B의 인식+orbit 강점 + D의 loop/재방문 강점을 결합한 **Two-Island 비대칭 배치**가 9개 실험을 모두 덮는다(아래).

---

## Recommended Shelf Layout (Task 2)

**추천 = Two-Island 비대칭 L/대각 + 둘레 loop.** 두 매대를 개방공간 안 두 "섬"으로 두되 **평행을 피해**(LiDAR 대칭 회피) 서로 다른 각도·벽거리로 배치. 각 매대 전 둘레 ≥1.2m 확보 → 360° orbit. 두 매대를 잇는 경로가 figure-8을 형성.

**치수(권장)**:
- 매대 간 거리(중심): **2.5~3.0m** (사이를 straight-pass + figure-8 교차점으로 사용)
- 벽과의 거리: **≥1.2m** (Husky가 매대 뒤로 통과해 full-orbit; 0.67m+양측 마진)
- Husky 통과 폭(모든 통로): **≥1.2m** (최소 1.0m, 권장 1.2m)
- D455 객체 관측거리: **0.6~2.0m**(이상범위 내, 0.52m 이내 금지; 4m서도 <2%지만 작은객체는 ≤2m 권장)
- 카메라 마운트 높이: 매대 상판(객체 바닥) 높이 ±, **객체를 0.6~2m에서 약간 내려보는 각** (D455를 Husky 마스트 ~0.8~1.2m에)
- LiDAR: 매대(수직면)+벽+의도적 landmark(상자/의자 2~3개)로 **비대칭 기하특징** 확보; 완전 빈 대칭 복도 금지.

```
 Top view (개방공간 ≥ 5 x 5 m, 벽=#)
 ###############################################
 #                                             #
 #     [ Shelf A ]  (정면 남향)                 #     A: easy/medium 객체
 #     ===========                              #        앞 ≥1.2m, 뒤 ≥1.2m orbit
 #        |   ^orbit A                          #
 #        v        \                            #
 #   start o----->---\------+                   #     straight-pass (A→B 사이)
 #         \          \     |                   #     중앙 X = figure-8 교차 = loop closure
 #          +----X-----+    |                   #
 #         /     |      \   v                   #
 #        +------+-------> [ Shelf B ]          #     B: hard 객체(밀집·대칭·유사색)
 #          ^orbit B        =========           #        각도 틀어 배치(평행 회피)
 #                          (대각 회전)          #
 #   # landmark(box)              # chair       #     LiDAR feature용 비대칭 landmark
 ###############################################
   범례: o=시작/복귀(loop), X=figure-8 교차(재방문), 통로 폭 ≥1.2m
```

- **trajectory 후보(이 배치에서 전부 가능)**: straight-pass(A↔B 사이), orbit A, orbit B, **figure-8(A→X→B→X→A, 중앙 X 재방문=loop closure)**, approach-retreat(각 매대 정면), occlusion-side view(밀집측 측면 진입), long-mission(둘레 다회 루프).

---

## Object Placement Plan (Task 3)

7객체를 **난이도·축 다양성**으로 두 매대에 분배. (객체 축은 이전 리서치: 저텍스처박스=milk / 고텍스처박스 / 원통 축대칭 / 대칭무지 / 소형 / 대형 / 반사.)

배치안 비교:

| 안 | 구성 | 장점 | 단점 |
|---|---|---|---|
| A 4+3 균등 | 두 매대 비슷 | 단순, 균형 | 난이도 분리 약함 |
| B dense/sparse | 한 매대 밀집·한 매대 희소 | occlusion vs spacing 대조 | 축 다양성 분산 |
| **C easy/hard 분리** | A=easy/med, B=hard | **난이도 물리 분리=clean 실험군** | 매대별 객체 고정시 leakage 위험 |
| D 동일군 재배치 | 같은 객체 여러 난이도 | viewpoint/난이도 ablation | 세션 많이 필요 |

→ **추천 = C(난이도 분리) + D(세션마다 재배치) 결합.** 매대 역할(A=easy/med, B=hard)은 고정하되 **세션마다 객체 위치·이웃·각도를 바꿔** train/test 누수 방지.

**기본 배치(세션1 예시)**:
- **Shelf A (easy/medium)**: 고텍스처박스, 대형 비정형, 소형 — **간격 ≥30cm**, 정면 노출, 빈 bay 1칸(no-object).
- **Shelf B (hard)**: milk(저텍스처 흰박스), 대칭무지 원통, 반사캔, 축대칭 병 — **간격 <10cm 밀집 + 유사색 인접(milk와 흰 무지 나란히)** + 일부 상호 가림, 빈 bay 1칸.
- **false-positive 유도 배경**: 흰 박스류·유사 형태 비대상 물체를 매대 위/주변에 의도적으로 둠(no-object rejection·FP stress).

> 세션 재배치 규칙: 매 세션 객체 **위치 순서·회전·이웃쌍**을 변경(같은 6D pose 반복 금지) → pose-leakage 차단(아래 split).

---

## Difficulty Level Design (Task 4)

| 난이도 | 객체 간 거리 | 객체 수/장면 | 매대 | Husky trajectory | 추가 GT |
|---|---|---|---|---|---|
| **Easy** | ≥30cm | 2~3 | Shelf A | orbit, approach(1~1.5m) | 6D, ID, visible |
| **Medium** | 10~20cm, 부분 가림, 유사색 근접 | 3~5 | A+B 혼합 | orbit + straight-pass, 측면각 | + occlusion level(0~2) |
| **Hard** | <10cm 밀집, 강가림, 유사색/형태 인접, **원거리 2.5~3m** | 5~7(전 객체) | Shelf B | figure-8, occlusion-side, 원거리 pass | + occlusion level(0~3), 가림주체 ID |

- Easy=상한 성능(검출/pose 기준선), Medium=현실, Hard=논문의 "어려운 케이스" 주장 근거(BOP-occlusion/T-LESS-유사외관 정신). 원거리는 D455 0.6~3m 내에서 최대(≤3m).

---

## Husky Trajectory Plan (Task 5)

| trajectory | 패턴 | 검증 실험(metric) |
|---|---|---|
| **straight pass** | A↔B 사이 직선 통과 | navigation, odom **RPE**(국부 drift), 통과 중 검출 |
| **shelf orbit** | 한 매대 360° 회전 | **viewpoint robustness·multi-view 6D**(ADD-S vs yaw), 템플릿 시점 커버 |
| **figure-8** | A→X→B→X→A (중앙 X 교차) | **loop closure**(ATE 개선 on/off), object map 일관성 |
| **approach-and-retreat** | 매대 정면 1.5m→3m→0.6m | scale/거리별 검출·pose, **min-Z(0.52m) 경계** 행동 |
| **loop closure revisit** | 둘레 1바퀴 후 시작점 복귀 | 재방문 인식·relocalization, ATE |
| **occlusion-side view** | 밀집 매대 측면/저각 진입 | **occlusion robustness**(가림도별 pose), tracking 유지율 |

→ 6개 trajectory가 9개 실험을 커버: detection/pose(orbit·approach), no-object(straight-pass with empty bay), occlusion(occlusion-side), viewpoint(orbit), tracking(연속 전체), object-SLAM/loop(figure-8·revisit), navigation(straight·long).

---

## Dataset Split Design (Task 6) — leakage 방지

**원칙**: 같은 bag을 프레임 단위로 train/test 분할하면 시간상관 leakage. **arrangement(객체 배치)와 trajectory를 단위로 split.** 같은 객체 인스턴스라도 배치/pose가 다르면 pose leakage 없음(BOP도 객체는 공유, scene은 분리).

| split | 구성 | 목적 |
|---|---|---|
| **Train/Calib** | arrangement #1~2, orbit+approach (Easy/Med) | 캘리브·시점 커버·(필요시) fine-tune |
| **Validation** | arrangement #3, 다른 trajectory(straight) | 임계/하이퍼 선택 |
| **Test** | arrangement #4 (미사용 배치), orbit+pass | 주 성능 보고(unseen 배치) |
| **Hard Test** | arrangement #5, Shelf B 밀집·원거리·occlusion-side | 어려운 케이스 주장 |
| **Long Sequence** | figure-8 + 둘레 long-loop (배치 고정) | SLAM ATE/RPE·loop closure |

- 규칙: **test/hard-test의 객체 배치·pose는 train/val에 등장 금지.** trajectory도 가능하면 분리(train=orbit, test=pass 등)해 trajectory overfit 차단.

---

## GT / Annotation Plan (Task 7)

| GT | 방법/배치 |
|---|---|
| **shelf coordinate frame** | 각 매대에 **AprilTag 보드 고정**(상판 모서리 또는 후면판) → 매대별 원점/축 정의 |
| **object 6D pose (in shelf frame)** | CAD + 첫프레임 정렬 + **depth-ICP**로 shelf frame 기준 6D 1회 산출 → 카메라 시점 무관 재사용 |
| **object ID** | 배치표(arrangement manifest)에 매대·bay·객체ID 기록 |
| **object visibility / occlusion level** | per-frame 0/1 visible + 가림도 0~3(미가림/약/강/거의전부) + 가림주체 ID |
| **camera trajectory** | LiDAR-SLAM(ICP-to-map) 또는 mocap; cam은 cam↔LiDAR extrinsic으로 환산 |
| **LiDAR trajectory** | LiDAR odom/SLAM 결과(궤적 GT 기준) |
| **loop closure event** | figure-8 교차 X·시작점 복귀 타임스탬프 태깅 |

**AprilTag/marker 설치 권장 위치**:
- **매대마다 tag 보드 1개**(후면판 또는 상판 가장자리, 객체에 안 가리게) = shelf frame 앵커.
- **바닥/벽에 글로벌 tag 1~2개** = world frame + 두 shelf frame 정합 + 루프 재방문 relocalization 보조.
- **cam↔LiDAR extrinsic 캘리브용 보드** 1회(수집 전, 별도 calib bag).
- 주의: tag가 **평가용 RGB의 객체 영역을 가리지 않게**, 그리고 SAM-6D 입력에 tag가 FP를 유발하지 않게 객체에서 떨어뜨려 설치.

> 이 방식의 이점: object 6D를 **shelf frame에 한 번** 측정 → 모든 trajectory/세션에서 카메라 pose만 알면 객체 pose가 자동 도출 = **재라벨링 비용 ~0**(이전 리서치 GT 레시피의 매대-특화 구현).

---

## Minimal Bag Collection Plan (Task 8)

2매대·7객체로 9개 실험을 최소 1회씩 — **~10 bag, 총 ~25분**. (저장 ~4~5GB/분 + LiDAR → ~120~150GB.)

| # | bag 목적 | 배치/난이도 | trajectory | 길이 |
|---|---|---|---|---|
| 0 | **calib** | tag 보드만 | cam↔LiDAR extrinsic | ~1min |
| 1 | single-object 기준 | A: 1~2객체 isolated | orbit + approach | ~2min |
| 2 | multi-object Easy | A: 3 spaced + empty bay | orbit + straight-pass | ~2min |
| 3 | multi-object Medium | A+B 혼합 4~5 | orbit + 측면 | ~2.5min |
| 4 | multi-object Hard | B: 밀집 5~7 + 유사색 | occlusion-side + 원거리 pass | ~3min |
| 5 | no-object rejection | empty bay/FP 배경 위주 | straight-pass | ~2min |
| 6 | object tracking | Med 배치 | 연속 orbit→pass (끊김 없이) | ~3min |
| 7 | **loop closure** | 고정 배치 | **figure-8 (X 재방문)** | ~3min |
| 8 | dynamic robustness | Med 배치 + 사람 통행 | straight-pass + 사람 이동 | ~2.5min |
| 9 | **long sequence** | 고정 배치 | 둘레 long-loop 다회 | ~4min |

> 재배치는 위 split 규칙대로 bag 2~5에 서로 다른 arrangement(#1~5) 사용 → test/hard-test 누수 차단. bag 7·9는 SLAM용으로 배치 고정.

---

## 한계 / 가정(추측 금지)

1. 매대 치수·실내 면적은 가정값(≥5×5m, 매대 ~1.2m W). 실제 공간에 맞춰 거리·통로 재산정 필요.
2. Husky wheel-odom/IMU·LiDAR 모델 미확정 → 궤적 GT 정확도·저장량 변동. mocap 가용 시 GT 단순화.
3. D455(f) 0.52m min-Z는 해상도/모드 의존(0.4m 보고도 있음) → 객체를 ≥0.6m에서 관측 권장.
4. AprilTag가 SAM-6D 입력에 미치는 영향(FP) 미검증 → PoC에서 tag 마스킹 여부 확인 필요.
5. 7객체의 실제 축(대칭/반사 보유 여부)은 추정 — 실물 확인 후 매대 배정 확정.

---

## Recommended Next BMAD Step

**→ Quick-Dev** (PRD·추가 Research 아님).

이유: 배치 설계(이 보고서)는 "무엇을 어디에"를 정의했고, 남은 리스크는 **실행 가능성**(shelf frame 앵커링·6D GT 자동화·tag 영향)이다. 본격 수집 전 작은 PoC로 검증해야 모든 bag이 라벨 가능해진다. (이전 리서치의 "GT 파이프라인 PoC" 권고와 일치 — 매대 시나리오로 구체화.)

권장 Quick-Dev 범위:
1. **배치 SOP 문서** — 매대 거리/통로/관측거리/마운트 높이 체크리스트 + arrangement manifest 템플릿.
2. **AprilTag shelf-frame PoC** — tag 보드로 shelf frame 추정 + object 6D(shelf frame) 1회 등록 → 다른 시점에서 재투영 검증.
3. **6D GT 자동전파 PoC** — milk(보유 CAD)로 depth-ICP가 shelf frame 기준 per-frame pose를 내는지 + tag 마스킹이 SAM-6D FP에 주는 영향 1회 측정.

---

## 다음 액션 제안 (사용자 결정 필요)

> 정식 research .md 표준 — 이 절로 마무리.

**핵심 판정**: 9개 실험을 모두 덮으려면 **Two-Island 비대칭 L/대각 배치(orbit+figure-8 loop, 통로 ≥1.2m, 관측 0.6~2m)** + **난이도 분리·세션 재배치 object placement** + **shelf-frame AprilTag GT**가 필요. 최소 ~10 bag으로 검증 가능. 다음은 **Quick-Dev(배치 SOP + shelf-frame/6D GT PoC)** 권장.

선택지:
- **(A) Quick-Dev: 배치 SOP + AprilTag shelf-frame + 6D GT 전파 PoC** *(추천)* — 수집 전 라벨 실행가능성·tag 영향 확증(최대 리스크 해소).
- **(B) 추가 Technical Research: 실제 실내 공간 실측 기반 배치 확정** — 방 도면/매대 실측 후 거리·통로 수치 고정.
- **(C) PRD: 전체 데이터 수집 캠페인 + object-aware SLAM 시스템 통합** 문서화 — 큰 그림 우선(단 GT PoC 미검증 리스크 감수).

→ **어느 쪽으로 진행할까요?** (추천: A — shelf-frame GT 실행가능성이 매대 수집의 단일 최대 리스크)
