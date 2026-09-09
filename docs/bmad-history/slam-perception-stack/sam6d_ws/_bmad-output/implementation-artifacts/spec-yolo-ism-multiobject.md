---
title: 'YOLO-World multi-object ISM 인식 (config 기반 yolo_ism_object_n.py)'
type: 'feature'
created: '2026-06-17'
status: 'done'
baseline_commit: '7bc2cc26a058b042c8bff4ef8d23f92b855bc36a'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 기존 `yolo_ism.py`는 단일 객체(milk) 전용으로 prompt·템플릿이 하드코딩되어, 7종(변종 포함 12개) 객체를 한 번에 ISM 인식할 수 없다.

**Approach:** `yolo_ism.py`를 **수정하지 않고 import-재사용**하여 새 실행 파일 `yolo_ism_object_n.py`를 만든다. 객체별 YOLO-World prompt·템플릿·feature 캐시 경로를 `configs/yolo_ism_objects.yaml`에서 읽어, 프레임마다 enabled 객체들을 ISM(semantic cls + masked-appe) 게이트로 인식하고 객체별 결과를 저장한다. PEM/6D pose·geometric 축은 이번 범위에서 제외.

## Boundaries & Constraints

**Always:**
- `yolo_ism.py` 원본 무수정 — 함수/상수를 `import yolo_ism`으로 재사용.
- 객체 prompt·template_dir·cache 경로는 코드 하드코딩 금지, 전부 config에서 로드.
- 운영점 기본값 = milk ISM 운영점(conf0.02 + sim0.35 + MobileSAM masked-appe gate0.55, use_mask), config에서 객체별 override 허용.
- high/low 변종은 각각 별도 config 객체 항목으로 등록.

**Ask First:**
- config 스키마(필드명/구조)를 본 spec과 다르게 바꿔야 할 경우.
- 실행 모델을 per-object 다중 YOLO pass로 바꿔 비용이 크게 늘어나는 경우.

**Never:**
- PEM/6D pose 추정, geometric(depth) 축 구현.
- SAM-6D 원본 소스 수정.
- 정량 GT 평가/VLM 라벨링(이번엔 인식 결과 산출·육안 검증까지).

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| 정상 다객체 프레임 | bag 프레임 + config 12객체 | 프레임당 각 객체별 semantic→masked-appe 게이트 판정, 통과 객체 overlay+CSV 기록 | N/A |
| 특정 객체 proposal 0개 | 해당 class YOLO box 없음 | 그 객체 decision=`no-proposal`, 다른 객체는 정상 처리 | continue |
| template_dir 없음/rgb 0장 | config 경로 오기입 | 해당 객체 skip + 경고 로그, 나머지 진행 | warn, skip |
| enabled:false 객체 | config 플래그 | 로드/실행 모두 제외 | N/A |

</frozen-after-approval>

## Code Map

- `yolo_ism.py` -- 재사용 원천(무수정): `build_dinov2`, `build_template_cls`, `build_template_appe`, `semantic_score`, `masked_appe_score`, `build_segmentor`, `segment_box`, `masked_query_patches`, `dinov2_forward`, `resolve_frames`, `draw_result`, 상수 `PATCH`.
- `yolo_ism_object_n.py` -- 신규. config 로드 → DINOv2/MobileSAM/YOLO-World 1회 로드 → 객체별 템플릿 feature(cls+appe) 캐시 → 프레임 루프(멀티클래스 YOLO 1회 predict → class별 box 라우팅 → 객체별 ISM 게이트) → 객체별/통합 결과 저장.
- `configs/yolo_ism_objects.yaml` -- 신규. `defaults`(운영점) + `objects[]`(name, yolo_prompt, object_id, template_dir, cad_ply(reserved), enabled, 선택 override).
- `outputs/yolo_ism_object_n/<bag>/` -- 객체별 `<name>_results.csv`+overlay, 통합 `multiobject_summary.csv`.

## Tasks & Acceptance

**Execution:**
- [x] `configs/yolo_ism_objects.yaml` -- 12개 객체(milk, choco_hazelnut high/low, Febreze high/low, Mugcup high/low, saffron, Sauce high/low, Sikhye high/low) + defaults 운영점 정의 -- prompt/경로 config화 요구 충족.
- [x] `yolo_ism_object_n.py` -- `import yolo_ism`으로 함수 재사용, config 파싱(`--config`), 객체별 template feature 캐시(객체명 기반 경로), 멀티클래스 YOLO predict→class 라우팅→객체별 semantic+masked-appe 게이트, 객체별/통합 CSV·overlay 저장, SUMMARY 출력.
- [x] 파일럿 실행 -- `two_table_around` 1 bag(stride 10, 202프레임) 실행 후 객체별 결과 CSV·overlay 생성 확인. 결과: Febreze/Mugcup/Sauce/Sikhye 인식, milk/choco/saffron=0(YOLO-World recall + 객체별 게이트 미튜닝 — 라우팅 버그 아님, 진단 완료).

**Acceptance Criteria:**
- Given 원본 `yolo_ism.py`, when 본 작업 완료, then `git diff yolo_ism.py`가 비어 있다(무수정).
- Given config의 enabled 객체 N개, when `yolo_ism_object_n.py --config ...` 실행, then `outputs/yolo_ism_object_n/<bag>/`에 객체별 `<name>_results.csv` N개와 `multiobject_summary.csv`가 생성된다.
- Given config에서 한 객체 `yolo_prompt`/`enabled` 변경, when 재실행, then 코드 수정 없이 동작이 반영된다.
- Given two_table_around 파일럿, when 실행 종료, then 콘솔 SUMMARY에 객체별 최종 인식 프레임 수가 출력된다.

## Spec Change Log

## Design Notes

- **실행 모델(효율):** 매 프레임 `yolo.set_classes([enabled prompts])` 1회 predict 후 `boxes.cls`로 객체별 proposal 라우팅 → DINOv2/MobileSAM는 객체별 selected proposal에만 적용. (객체마다 YOLO 재호출하는 per-object loop 대비 YOLO 비용 1/N.)
- **재사용 패턴 예시:**
  ```python
  import yolo_ism as yi
  model = yi.build_dinov2(ckpt, device)
  tcls, _ = yi.build_template_cls(obj.template_dir, model, device, obj.cls_cache)
  tappe, _ = yi.build_template_appe(obj.template_dir, model, device, obj.appe_cache)
  sem = yi.semantic_score(cls, tcls, match_topk)            # 1차 게이트
  appe = yi.masked_appe_score(q_fg, tappe[best_t])          # 2차 게이트
  ```
- cache 경로는 config 미지정 시 `outputs/yolo_ism_object_n/template_features/<name>_{cls,appe}.pt`로 자동 유도.
- `cad_ply`는 향후 geometric/PEM 확장용 reserved 필드(이번엔 미사용).

## Verification

**Commands:**
- `python -c "import yolo_ism"` -- expected: import 에러 없음(재사용 가능 확인).
- `git diff --stat yolo_ism.py` -- expected: 출력 없음(원본 무수정).
- `python yolo_ism_object_n.py --config configs/yolo_ism_objects.yaml --bag data/ros2_bag/two_table_around --frames-dir outputs/yolo_test/two_table_around/frames --max-frames 5` -- expected: 에러 없이 종료, 객체별 CSV/overlay 생성.
- 파일럿: 위 명령에서 `--max-frames` 제거(stride 10 전체) -- expected: SUMMARY에 객체별 인식 프레임 수 출력.

**Manual checks:**
- `outputs/yolo_ism_object_n/two_table_around/`의 객체별 overlay 몇 장 육안 확인 — milk/Mugcup 등 박스·마스크가 타깃 객체에 합리적으로 맞는지.

## Suggested Review Order

**설계 진입점 — 멀티객체 라우팅**

- 핵심 통찰: high/low가 prompt를 공유하므로 YOLO는 고유 prompt로, ISM은 객체별 템플릿으로 분기
  [`yolo_ism_object_n.py:124`](../../yolo_ism_object_n.py#L124)

- 프레임당 YOLO 1회 predict → class별 box 라우팅 → 객체별 ISM (효율 1/N)
  [`yolo_ism_object_n.py:289`](../../yolo_ism_object_n.py#L289)

**객체별 ISM 게이트 (재사용)**

- semantic(cls) 선택 → masked-appe 2차 게이트; yolo_ism 함수 import 재사용
  [`yolo_ism_object_n.py:138`](../../yolo_ism_object_n.py#L138)

- 단일 공유 YOLO pass에서도 객체별 score_threshold/top_k override를 보장(리뷰 patch P3)
  [`yolo_ism_object_n.py:145`](../../yolo_ism_object_n.py#L145)

**Config 로드/검증 (리뷰 patch P2)**

- 필수키·중복 name·objects 타입 검증으로 잘못된 config를 명확히 거부
  [`yolo_ism_object_n.py:58`](../../yolo_ism_object_n.py#L58)

- 템플릿 빌드 실패 시 해당 객체만 skip (patch P4)
  [`yolo_ism_object_n.py:97`](../../yolo_ism_object_n.py#L97)

**Config (peripheral)**

- 12객체 prompt/경로 + defaults 운영점 정의
  [`yolo_ism_objects.yaml`](../../configs/yolo_ism_objects.yaml)
