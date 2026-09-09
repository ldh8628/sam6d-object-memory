# Phase 2 ISM 병목 재검증 보고서
## Appearance Block 2·9 / Semantic Top-5·전체평균 / YOLO-World BBox 미생성

- 작성일: 2026-07-22 · 작성: Ldh9501 · 스킬: `/bmad-quick-dev`
- 성격: **검증 전용**. 운영 config·판정 로직 변경 없음. git 조작 없음.
- 산출물: `outputs/phase2_appearance_semantic_yolo_bbox/`
- 명세: `_bmad-output/implementation-artifacts/spec-phase2-appearance-semantic-yolo-bbox-validation.md`

---

## 1. Executive Summary

세 Workstream을 Phase 1C 동결 기준선(TP 622 / FP 86 / FN 313 / F1 0.7572, 사람 GT 935 positive cell) 위에서 dataset LODO로 검증했다. **통합 P0를 Phase 1C와 정확히 재현**(TP 622 / FP 86 / F1 0.7572)한 뒤 변형을 비교했다.

| Workstream | 핵심 결과 | 판정 |
|---|---|---|
| A. Appearance block | block2≈block9≈block11 단독 AUROC .80 (거의 동일). 파이프라인에서 **block9 게이트가 FP 86→54(-37%), F1 .7572→.7653**. 단 **class 편중**(dolls는 block2, choco는 block11) → H-A4 위배 | **조건부 후보 / MORE DATA** |
| B. Semantic 집계 | top5~allmean 상관 **.948**, 집계식 전부 AUROC .79~.81, **어떤 변형도 파이프라인 F1 미개선**. "Top-5×전체평균"(곱)은 오히려 악화(.7403) | **KEEP CURRENT (top-5)** |
| C. YOLO BBox 미생성 | 미검출 163셀 중 **140(86%)=shared-pass 교차클래스 NMS 억제(F2)**. per-prompt로 visibility recall **.826→.959**, 최종 파이프라인 **shared960 F1 .700 vs per-prompt(cur) .757 = +84 TP** | **per-prompt 보장(최우선) / 해상도는 MORE DATA** |

**채택 후보**: (1위) 배포 경로의 **per-prompt proposal 보장**(shared-pass면 +84 TP), (2위) **block9 appe 게이트**(FP-side, class 편중 주의).
**기각**: semantic 집계식 변경, perprompt1920/해상도 상향(FP·latency 대비 이득 미미), texture(Phase 1C에서 이미 기각).
**추가 데이터 필요**: 클래스별 appe block 선택, perprompt1280 downstream FP 관리.

---

## 2. Phase 1C Frozen Baseline (변경 없음)

| 항목 | 값 |
|---|---|
| HSV hard gate | enabled, t=0.1214, score<t → reject |
| Dinosaur hue 보정 | +9 OpenCV 단위(=+18°), 캐시 baked, runtime 분기 없음 |
| texture negative/rescue | 비활성(운영 미적용) |
| 성능(사람GT935) | A(HSV off) TP648/FP334/F1.676 · **C(운영) TP622/FP86/FN313/F1.7572** |

**회귀 확인(AC-01)**: `test_uniform_threshold` 5/5, `test_hsv_shadow` 13/13, `test_phase1c` 12/12 통과. config 값(t0.1214, Dino+9, gate enabled) 불변. **통합 P0가 622/86/.7572를 정확히 재현**하여 기준선 보존 검증.

---

## 3. 실제 Pipeline과 코드 흐름 (코드 실측, 추측 아님)

운영 `recognize_frame`(`yolo_ism_object_n.py:499`) 게이트 순서: **YOLO conf → semantic(hard) → mask → appe(hard) → HSV(hard)**.

| stage | 파일:함수 | 입력→출력 | score/gate | 최종판정 영향 |
|---|---|---|---|---|
| YOLO-World | `yolo_ism_object_n.py:677` predict | RGB→boxes | conf≥`score_threshold`0.02, imgsz960, ultralytics 기본 NMS **iou0.7 class-aware(yaml 미설정)** | O(후보) |
| top_k | `:520` | boxes→top3 | `top_k:3`, conf 내림차순 | O |
| semantic | `yolo_ism.py:238` `semantic_score` | CLS·42템플릿CLS→scalar | **mean(top-5 of 42), gate≥`similarity_threshold`0.35 HARD** | O |
| mask | `yolo_ism.py:289` `segment_boxes` | box→MobileSAM mask | — | fg 선택 |
| appe | `yolo_ism_object_n.py:296` `masked_appe_blocks` | patch·템플릿patch→scalar | **block11(0-based last) per-patch matched cosine, gate≥`appe_gate`0.55 HARD** | O |
| HSV | `yolo_ism_object_n.py:_apply_hsv_gate` | crop·mask→sim | Bhattacharyya, gate≥0.1214 HARD | O |

- backbone: **DINOv2 `dinov2_vits14`(ViT-S/14, 12 blocks, register 0)**, ImageNet norm, block2/9/11은 0-based 직접 인덱스.
- **단일 forward 확인**: `get_intermediate_layers(x, n=[2,9,11])` 한 번으로 3블록 동시 추출(+0.1%). 덤프 provenance `forward_calls==frames_processed==270`으로 증명. `nms_rank_blocks:[2,9]`가 운영에서 이미 다블록 추출 중.
- 검출기=**YOLO-World only**(yolov8m-worldv2). FastSAM은 vendored 트리에만 존재, 운영 미사용. MobileSAM은 박스 마스킹 전용(proposal 아님).

---

## 4. Workstream A — Appearance Block 2·9

**데이터**: `phase2_candidates.csv`(18060행, 6셋), 선택-best 후보 population(candidate 있는 (frame,object)) pos=897/neg=1244. score variant=`clstop1`(운영 appe와 동일 뷰).

**단독 판별력**:
| block | AUROC | AUPRC |
|---|---|---|
| block2 | 0.8002 | 0.787 |
| block9 | 0.7972 | 0.7822 |
| block11(운영) | 0.7957 | 0.7633 |

세 블록이 거의 동일(.80). 상관 b9-b11 **.852**(중복 큼), b2-b11 **.491**(block2가 가장 독립적). b9-semantic **.754**(block9는 semantic과 유사).

**파이프라인 기여(LODO, semantic+HSV 고정, appe 게이트만 교체)**:
| config | TP | FP | FN | F1 |
|---|---|---|---|---|
| A0 block11 @0.55(운영) | 623 | 86 | 312 | 0.7579 |
| A1 block2 (LODOthr) | 558 | 82 | 377 | 0.7086 |
| **A2 block9 (LODOthr .6008)** | **613** | **54** | 322 | **0.7653** |
| Ax block11 (LODOthr) | 574 | 43 | 361 | 0.7397 |
| A3 mean(b2,b9) | 615 | 75 | 320 | 0.7569 |
| A4 z-가중합(b2,b9) | 596 | 61 | 339 | 0.7487 |

- **block9 게이트가 최고 F1(.7653)**, FP 86→54. threshold 0.6008이 6폴드 전부 안정(민감도 낮음, A-8 #4 충족).
- fusion(A3/A4)은 block9 단독보다 못함 → 결합 불필요(H-A3 기각).

**가설 판정**:
- H-A1(block2가 texture/edge로 FP 유리): 부분참. block2 AUROC 최고이나 파이프라인에선 저조.
- H-A2(block9가 object-level로 TP 유리): 기각. block9는 오히려 FP 제거가 강점.
- H-A3(결합 개선): **기각**(fusion < block9 단독).
- H-A4(개선의 객체 무의존): **기각 — class 편중이 핵심 발견**.

**클래스별 AUROC(핵심)**:
| object | block2 | block9 | block11 |
|---|---|---|---|
| Bear | **0.749** | 0.518 | 0.542 |
| Dinosaur | **0.811** | 0.608 | 0.522 |
| choco_hazelnut | 0.410 | 0.747 | **0.795** |
| saffron | **0.965** | 0.905 | 0.933 |
| milk | 0.862 | 0.856 | 0.869 |

**dolls(Bear/Dinosaur)는 block9/11이 거의 무작위, block2가 압도**. 반대로 **choco(갈색박스)는 block11(texture)이 최고**. 즉 최적 block은 **클래스 의존적**이며, block9의 aggregate 승리는 FP-heavy 클래스에 기인. block9 blanket 교체는 dolls 위험(FN_new 40, 아래 §7).

**결론**: block9 게이트는 aggregate F1을 올리나(+0.008) 클래스 편중으로 A-8 #3("다수 객체 개선") 부분 미충족 → **조건부 후보**. 이상적 해법은 클래스별 block(dolls=block2, choco=block11, 그 외=block9)이며 추가 검증 필요.

**Latency/메모리(AC-02, A-8 #5·6)**: block2/9/11 동일 forward 동시 추출(재forward 0, provenance 증명). 추가 GPU 비용 ≈ block별 norm +0.1%, patch matmul +0.026ms/템플릿 수준 → **사실상 무료**. 운영 채택 시 forward 추가 없음.

---

## 5. Workstream B — Semantic Top-5·전체평균

**원본 정의**: semantic = 42-view 코사인 배열 → 운영은 **mean(top-5)**(match_topk:5). "Top-5×전체평균"은 문서에 정의 없어 곱(S5)·기하평균(S6)·가중합(S7=α·top5+(1-α)·all) 모두 비교.

**단독 판별력**:
| 집계 | AUROC | AUPRC |
|---|---|---|
| S1 top1 | 0.7914 | 0.7871 |
| S3 top5(운영) | 0.8001 | 0.7926 |
| S4 all-mean | **0.8103** | 0.7867 |
| S5 product | 0.8096 | 0.7954 |
| S6 geomean | 0.8096 | 0.7954 |
| S7 α가중 | 0.8087 | 0.7957 |

**top5~allmean 상관 = 0.948**(거의 중복). all-mean이 근소 우위이나 파이프라인에선 무의미.

**파이프라인 기여(LODO, selection=top5 고정, appe+HSV 고정)**:
| config | TP | FP | FN | F1 |
|---|---|---|---|---|
| S0 top5 @0.35(운영) | 623 | 86 | 312 | **0.7579** |
| S3 top5 (LODOthr) | 585 | 68 | 350 | 0.7368 |
| S4 all-mean (LODOthr) | 603 | 56 | 332 | 0.7566 |
| S5 product | 583 | 57 | 352 | 0.7403 |
| S7 α (LODO, α=0 전폴드) | 603 | 56 | 332 | 0.7566 |

- **어떤 변형도 운영 S0(.7579)를 넘지 못함**. all-mean이 근접(.7566)하나 TP 손실.
- S7 α-탐색이 **전폴드 α=0**(=순수 all-mean)으로 수렴 → top-5 가중은 불필요, 그러나 all-mean도 이득 없음.
- "Top-5×전체평균"(곱 S5=.7403)은 **오히려 악화**. geomean은 product의 sqrt(단조변환)이라 동일 결과.

**Top-5 필요성 검증(B-7)**: top5와 allmean이 .948로 강상관 → Top-5의 "상위 안정성"과 전체평균의 "일관성"이 사실상 같은 신호. 결합의 추가 정보 없음. **결론: KEEP CURRENT (top-5 @0.35)**. B-9 채택조건 전부 미충족.

---

## 6. Workstream C — YOLO-World BBox 미생성

**정의**: GT-visible인데 운영 shared-pass(single multi-label, imgsz960, conf0.02, 기본 NMS)가 박스 미생성. conf0.005 raw 덤프(339프레임, 6config) 오프라인 분석. GT는 frame-level visibility뿐 → **visibility recall**(IoU recall 불가).

**Failure stage 분류(935 visible cell)**:
| stage | 수 | 의미 |
|---|---|---|
| hit_operational | 772 | 정상 |
| **F2 shared NMS 억제** | **140** | per-prompt는 ≥0.02 박스 생성하나 shared-pass NMS/클래스경쟁이 제거 |
| F1 below confidence | 19 | <0.02 |
| F0 no raw prediction | 4 | prompt raw 0 |

**미검출 163 중 140(86%)이 F2** — 진짜 원인은 confidence·prompt·해상도가 아니라 **shared 단일패스의 교차클래스 NMS 억제**.

**보완책 비교(visibility recall / FP·frame)**:
| method | recall | FP/frame |
|---|---|---|
| C0 shared960@0.02(운영 main) | 0.8257 | 3.99 |
| shared960 agnostic NMS | 0.7027 | 3.13 |
| **perprompt960@0.02** | **0.9594** | 6.48 |
| perprompt1280@0.02 | 0.9711 | 7.52 |
| ext prompt(ext960) | 0.9209 | 7.12 |
| conf 0.01 / 0.005 | 0.847 / 0.860 | 5.83 / 8.79 |

- **per-prompt 별도 패스가 recall .826→.959**(F2 140셀 회복). agnostic NMS는 악화(.703). conf 하향은 recall 소폭·FP 급증. ext prompt는 이득 미미.
- **결정적**: Phase 1C `cur` 후보집합의 candidate visibility recall이 **정확히 0.9594** = perprompt960. 즉 **cur/평가·PEM 경로는 이미 per-prompt를 사용**(NMS 억제 해소됨). 반면 `yolo_ism_object_n` 코어 shared-pass는 .826에 머무름.

**원인 분석 요약**: NMS(iou0.7 class-aware)가 shared 패스에서 강 prompt가 약 prompt 박스를 억제(과거 milk↔dolls cross-talk). confidence·해상도는 부차적(F1 19·F0 4).

---

## 7. 통합 결과 (P0~P5)

| 구성 | TP | FP | FN | F1 | 비고 |
|---|---|---|---|---|---|
| **P0 Phase1C(cur=per-prompt)** | **622** | **86** | 313 | **0.7572** | 정확 재현 ✅ |
| P1 + block9 appe | 613 | 54 | 322 | 0.7653 | FP-side |
| P2 + best semantic | 622 | 86 | 313 | 0.7572 | 채택 없음=P0 |
| P4 block9+semantic | 613 | 54 | 322 | 0.7653 | =P1 |
| **P3 shared960(단일패스)** | **538** | 64 | 397 | **0.7001** | proposal 억제 실측 |
| P3 perprompt1280 | 633 | 97 | 302 | 0.7604 | 해상도 회복 |

**이득 중복(P0 vs P1 block9)**: FP 제거 55 · 신규 FP 23 · **신규 FN 40**(block9가 참후보 40 제거 — dolls 편중). 순 FP -32, 순 FN +9.

**핵심 통합 해석**:
1. **proposal 레버가 가장 큼**: shared-pass(.700) vs per-prompt/cur(.757) = **최종 +84 TP**. 단 이 이득은 cur/평가·PEM 경로엔 **이미 반영**. 코어 recognize가 shared-pass로 배포되면 per-prompt 전환이 최대 이득.
2. **block9 게이트(gate-side)와 per-prompt(proposal-side)는 직교** — 결합 시 가산적(단 perprompt1280은 block11만 스코어링, 조합 실측은 cur 한정).
3. perprompt1280은 TP+11이나 **FP도 +11**(F1 중립), latency ~2× → 이득 대비 비용 낮음.
4. **잔여 FN의 본질**: cur 후보recall .959 → 최종 recall .665. 즉 **참후보 897 중 275가 게이트에서 제거**. FN 병목은 proposal이 아니라 **게이트(sem/appe/HSV)의 참후보 과제거**(recall 상한을 게이트가 결정).

---

## 8. 대표 사례

`csv/phase2_per_candidate_debug.csv`(3390셀) + `yolo_bbox_failure_cases.csv`(163셀)로 추적 가능. 대표:
- **block2만 성공(doll)**: Bear/Dinosaur — block2 AUROC .75/.81 vs block9/11 ~.52 (green/brown domain, texture-late block이 무력).
- **block11만 성공(choco)**: 갈색박스는 block11 texture .795가 block2 .410 압도.
- **Semantic 집계 차이 무의미**: top5·allmean 상관 .948 대표 산점(figures/semantic).
- **YOLO F2(NMS 억제)**: perprompt_top_conf 높은데 shared 0 → failure_cases.csv `F2_shared_nms_suppression` 140건.
- **F0(4건)**: prompt raw 0 — 극소.
- **BBox 회복 후 재제거**: perprompt1280 신규 박스 다수가 게이트 통과하나 FP도 동반(P3 FP 86→97).

figures: `figures/{appearance,semantic,yolo_bbox,integrated,representative_cases}/` (별도 생성).

---

## 9. 성능과 비용

- **Appearance block2/9/11**: 동일 forward 동시 추출(재forward 0, provenance 증명). 추가 비용 ≈ block norm +0.1% + patch matmul 미미 → 무료.
- **YOLO per-prompt**: 프롬프트 10패스 = shared 1패스 대비 ~약 7배 추론(실측 shared960 5.5s vs perprompt960 36.4s/339프레임). perprompt1280은 유사(28s)·박스↑. → per-prompt는 recall 대폭↑이나 latency 대가 큼(이미 배치/PEM 경로는 수용 중).
- GPU: 덤프 270프레임 단일패스, 사후 전량 오프라인. peak memory 이슈 없음.

---

## 10. 권장 운영 변경

```
Appearance:  BLOCK 9 (조건부) / 이상적으로는 CLASS-WISE BLOCK — MORE DATA
Semantic:    KEEP CURRENT (top-5 @0.35)
YOLO BBox:   PER-PROMPT 보장 (shared-pass 배포 시 최우선) / 해상도·prompt = MORE DATA
```

---

## 11. 변경 파일

**신규(연구·평가 경로만)**:
- `_ism_research_2026_07/phase2_bottleneck/phase2_common.py`
- `.../probes/dump_phase2_candidates.py`(GPU 덤프)
- `.../eval_appearance_blocks.py`, `eval_semantic_aggregation.py`, `collect_yolo_bbox_diagnostics.py`, `eval_yolo_bbox_recovery.py`, `eval_phase2_integrated.py`, `make_phase2_figures.py`
- `tests/test_phase2_eval.py`
- `outputs/phase2_appearance_semantic_yolo_bbox/**`(csv/metrics/figures/raw/overlays/report/provenance)
- 본 보고서 + spec

**운영 변경**: 없음(config·`yolo_ism*.py`·`ism_hsv.py` 판정 로직 무변경). diagnostic/read-only만.
**rollback**: 산출물 폴더 삭제로 완전 원복(운영 영향 0).

---

## 12. 남은 위험

- **class 편중**(block9): dolls 위험 — 클래스별 block 미검증.
- **사람 GT frame-level뿐**: C의 per-box IoU recall 불가(visibility recall 하한).
- perprompt1280 FP 증가(+11), latency 2×.
- shared-pass vs per-prompt: 실제 배포 경로 확인 필요(코어 recognize=shared, PEM/eval=per-prompt).
- LODO 6폴드지만 객체 불균형(dolls 소표본) — dolls 결론은 표본 제약.
- perprompt1920 미측정(비용상 생략, 로그 명시).

---

## 13. 최종 구현 우선순위 (최대 1~2)

**① per-prompt proposal 보장 (proposal-side)**
- 예상 TP: shared-pass 배포 시 **+84**(538→622); 이미 per-prompt면 perprompt1280로 +11.
- 예상 FP: per-prompt 전환 시 게이트가 흡수(cur 기준 FP 86 유지); 1280은 +11.
- 예상 F1: .700→.757(전환) / .757→.760(1280).
- latency: ~7× 추론(단일→10패스). 구현 복잡도: 낮음(`build_ism_inputs_imu` 방식 존재). 회귀위험: FP·latency.
- 근거: 미검출 140/163이 shared NMS 억제, per-prompt가 recall .826→.959, 최종 +84 TP 실측.

**② block9 appe 게이트 (gate-side, 조건부)**
- 예상 TP -9, FP **-32**(86→54), F1 +0.008. 추가 latency ≈0(동일 forward). 구현 복잡도 낮음(config `appe_blocks:[9]`+threshold).
- 회귀위험: **dolls FN**(class 편중, 신규 FN 40). → 클래스별 block 또는 dolls 예외 선결 권장.
- 근거: LODO F1 .7653>.7579, threshold 0.6008 6폴드 안정.

**주의**: 본 검증만으로 운영 자동 변경하지 않음. 위 후보는 별도 구현 스토리에서 dolls 예외/배포경로 확인 후 진행.

---

## 부록 — 산출물 인덱스
- CSV: `csv/{appearance_*,semantic_*,yolo_*,integrated_pipeline_metrics,phase2_candidates,phase2_per_candidate_debug}.csv`
- Metrics: `metrics/{appearance,semantic,yolo_bbox,integrated,dump_provenance,yolo_diag_provenance}.json`
- Raw: `raw/view_sims_phase2.npz`, `raw/yolo_raw/<config>/<ds>.csv`
- Provenance: `provenance/provenance.txt`, `config_snapshots/yolo_ism_objects.yaml`
- 재현: `~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase2_bottleneck/{dump_phase2_candidates(GPU),eval_appearance_blocks,eval_semantic_aggregation,collect_yolo_bbox_diagnostics(GPU),eval_yolo_bbox_recovery,eval_phase2_integrated --with-gpu}.py`
