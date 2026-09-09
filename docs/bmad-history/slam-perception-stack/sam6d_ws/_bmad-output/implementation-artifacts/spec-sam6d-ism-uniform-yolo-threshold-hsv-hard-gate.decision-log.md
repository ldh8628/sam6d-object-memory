# Decision Log — spec-sam6d-ism-uniform-yolo-threshold-hsv-hard-gate

## 2026-07-22 · create · self-validate verdict

**형식 결정:** 사용자가 명시 경로·18개 섹션·응답형식을 지정 → 기본 SPEC.md 커널 폴더 대신
저장소 house-style(`implementation-artifacts/spec-*.md` 평면 파일, 예: `spec-yolo-ism-multiobject.md`)로
작성. 사용자 지시가 기본 형식을 override.

**Authority 확인(추측 없음):** 운영 라인 앵커(config `:12,13,256,264,272`; 드라이버 게이트
`:286/336, :371/412, :446/514`, `_OVERRIDABLE :40`, `load_config :108`, `prepare_objects :193`,
`build_prompt_groups :259`, shared-min :583)와 HSV 파라미터(`eval_pipeline_end2end.py:39,54-59,97-98,123`;
`build_prototype_colors.py:115,132`; `fold_params.csv`)를 실제 파일에서 검증 후 고정.

**Pass 1 (Coherence):** 18개 섹션 전부 존재. AC-1~6 테스트 가능. Constraints(§4 out-of-scope,
§3 불변식)가 설계를 실제로 제약. 장식 없음. 통과.

**Pass 2 (Preservation):** 모든 load-bearing 원천 주장 반영 확인 — 성능표·Δ(§2), 객체 threshold
(§5/7/12), HSV 전 파라미터(§8), 3단계 Phase(§6), 로깅/진단 필드(§11), 파일변경 라인(§12), 테스트
(§13), B0/B1/B2+동결검증(§14), 롤백(§16), Story2/3/4(§18), 출력 emitter shadow 안전성(AC-5).

**정직성 플래그 2건(명세에 명시):**
1. t_hsv 는 코드 고정값이 아니라 fold별 캘리브(평균 ≈0.122) → §8.2/AC-2b/§17-R2/OQ-1 로 동결 절차화.
2. 저채도(Rabbit) 예외는 pinned mode-B 코드에 없음(리포트 이력만) → 예외 없음 채택, §8.3/§17-R3.
3. §2 "현행"(cur|R0)은 HSV 포함이라 오늘 운영(HSV 없음)과 다름 → 진짜 기준선 B0 를 §14 로 분리.

**개방질문:** OQ-1(배포 threshold 단일값 확정), OQ-2(진단 로그 포맷), OQ-3(HSV 캐시 도구 위치).

**Wrapper-only content:** 없음(드롭된 load-bearing 주장 없음).
