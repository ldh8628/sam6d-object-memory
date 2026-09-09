---
title: 'SAM-6D PPT FP Flow Revision'
type: 'chore'
created: '2026-06-11'
status: 'done'
route: 'one-shot'
---

# SAM-6D PPT FP Flow Revision

## Intent

**Problem:** 원본 업무보고 PPT의 전체 구조도에서 `FP 문제`가 `1-4. 후처리` 아래에 있어, FP가 후처리 단계에서 발생하거나 후처리만으로 해결되는 것처럼 보였다.

**Approach:** 원본은 백업으로 보존하고, 수정본의 실제 1페이지를 `Detection 후보 생성 -> Pose Estimation -> Score Gating / Reject FP -> Temporal Filtering -> Output 6D Pose` 흐름으로 재구성했다. FP 발생 위치는 detection 후보/pose scoring으로 옮기고, score-gating과 temporal filtering은 차단/완화 역할로 표현했다.

## Suggested Review Order

**구조도 의미**

- 1페이지 핵심 흐름과 FP 위치를 먼저 확인한다.
  [`rebuild_page1_fp_flow.py:172`](../ppt-review/rebuild_page1_fp_flow.py#L172)

- Detection, scoring, gating, temporal filtering의 역할 분리가 반영된 부분이다.
  [`rebuild_page1_fp_flow.py:178`](../ppt-review/rebuild_page1_fp_flow.py#L178)

- FP 발생 callout과 후처리 완화 callout이 분리되어 있다.
  [`rebuild_page1_fp_flow.py:191`](../ppt-review/rebuild_page1_fp_flow.py#L191)

**산출물 정리**

- 빈 첫 슬라이드와 중복 옛 구조도 제거로 page 1 혼선을 줄였다.
  [`rebuild_page1_fp_flow.py:257`](../ppt-review/rebuild_page1_fp_flow.py#L257)

- 수정 전후 판단과 검증 결과를 보고서에 정리했다.
  [`page1_structure_revision_report.md:18`](../ppt-review/page1_structure_revision_report.md#L18)

- 최종 산출물 경로와 렌더 파일을 보고서 첫 부분에서 확인한다.
  [`page1_structure_revision_report.md:3`](../ppt-review/page1_structure_revision_report.md#L3)
