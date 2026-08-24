---
title: '재사용 가능한 PEM 포즈 후보 HTML 탐색기'
type: 'feature'
created: '2026-08-22'
status: 'done'
baseline_commit: 'NO_VCS'
context:
  - 'output/longcircle2_experiments/20260822_SUMMARY.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 현재 longcircle2의 프레임별 PEM 결과, 객체 좌표축, 후보 단계별 정답 포함 여부와 기하·텍스처 점수 근거가 여러 JSONL·이미지·모델 내부에 흩어져 있어 한 화면에서 추적할 수 없다. 기존 후보 덤프는 상위 100개 자세와 스칼라 점수만 보존하며, 6,000→300 단계 및 점 단위 근거는 저장하지 않는다.

**Approach:** 진단 모드에서 필요한 후보 근거를 구조화해 내보내고, bag·검출 JSONL·SLAM pseudo-GT를 정적 report schema로 변환하는 생성기와 의존성 없는 HTML/CSS/JS 탐색기를 만든다. longcircle2 보고서를 실제 생성하되 다른 bag/result에도 같은 CLI와 schema를 재사용한다.

## Boundaries & Constraints

**Always:** Rabbit을 제외한 설정상의 9개 객체를 모든 프레임에 고정 슬롯으로 표시한다. 검출 없음, PEM 입력 탈락, GT 없음, 후보 진단 미수집을 서로 다른 상태로 표시한다. 정면/위 방향은 객체별 manifest 축 벡터로 정의하고 현재 데이터의 기본값도 설정 파일에서 수정 가능해야 한다. O/X는 회전·이동 임계값과 pseudo-GT 출처를 화면에 명시한다. 기존 실시간 경로는 진단 옵션이 꺼졌을 때 수치와 성능이 바뀌지 않아야 한다. HTML은 외부 CDN 없이 로컬 파일만으로 열린다.

**Ask First:** 객체별 CAD 정면축을 사람이 새로 확정해야 하는 경우, 외부 측정 GT를 도입하는 경우, 전체 6,000 후보의 원시 행렬을 영구 저장해 산출물 용량을 크게 늘리는 경우.

**Never:** pseudo-GT를 외부 정답처럼 표기하지 않는다. 미수집 값을 X로 간주하지 않는다. 6,000개 후보를 DOM에 한꺼번에 렌더링하지 않는다. Rabbit을 결과에 포함하지 않는다. 디버그 UI를 운영 추론의 기본 경로로 활성화하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 완전 진단 | bag + 9객체 결과 + 후보/GT/SLAM | 프레임 탐색, 축 오버레이, 후보 상세, 6000/300 OX, 상위 N 근거 이미지 | N/A |
| 부분 진단 | 기존 top-100 JSONL만 존재 | 가능한 점수·후보는 표시하고 6000/300은 `미수집` | 값 위조 없이 경고 배지 |
| 객체 미검출 | 해당 프레임의 객체 결과 없음 | 9개 슬롯을 유지하고 `검출 없음` 표시 | 상세 버튼 비활성화 |
| GT 불신/부재 | 객체 pseudo-GT가 trusted가 아님 | 정답 OX 대신 `GT 없음` | 후보 점수 탐색은 허용 |
| 잘못된 입력 | bag/JSONL/schema 불일치 | 생성 중단, 누락 경로·필드 보고 | 부분 산출물을 성공으로 표기하지 않음 |

</frozen-after-approval>

## Code Map

- `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py` -- 6,000→300 후보 생성, 기하·특징·색 점수의 유일한 계산 지점.
- `realtime/sam6d_core.py` -- 프레임/객체/마스크와 PEM 진단 결과를 결합하는 경계.
- `temp/verify_eval.py` -- 동일 프레임·동일 시드 longcircle2 진단 재실행기.
- `tools/evaluate_slam_pose.py` -- SLAM `T_wc`, 대칭 회전 오차, pseudo-GT 규약.
- `tools/build_pem_explorer.py` -- 신규 재사용 report 생성 CLI 및 schema 검증기.
- `tools/pem_explorer/` -- 신규 정적 HTML/CSS/JS 템플릿.
- `tests/test_pem_explorer.py` -- schema, 투영, 상태·OX 판정 회귀 테스트.

## Tasks & Acceptance

**Execution:**
- [x] `model_utils.py`, `sam6d_core.py`, `verify_eval.py` -- opt-in 후보 단계/점수 근거 진단을 추가하고 longcircle2를 재실행한다.
- [x] `tools/build_pem_explorer.py` -- 입력 검증, RGB 추출, 9객체 frame manifest, 축/GT/후보 판정과 근거 이미지를 생성한다.
- [x] `tools/pem_explorer/*` -- 프레임 슬라이더·검색, 9객체 카드, canvas 축, 후보 drawer, N 선택, OX/점수/근거 패널을 구현한다.
- [x] `tests/test_pem_explorer.py` -- 완전/부분/무GT/미검출/불량 schema를 자동 검증한다.
- [x] `output/longcircle2_pem_explorer/` -- longcircle2 HTML과 재생 명령·데이터 설명을 저장한다.

**Acceptance Criteria:**
- Given longcircle2 report를 열었을 때, when 임의 프레임을 이동하면, then 같은 원본 프레임 위에 최대 9객체의 PEM 포즈와 객체별 정면·위 화살표가 표시되고 9개 상태 카드가 동기화된다.
- Given 후보가 있는 객체 카드를 선택했을 때, when 상위 N을 바꾸면, then 후보 순위·R/t·기하/특징/색/결합 점수, 선택 정답 여부, 6,000/300 포함 OX 또는 미수집 상태, 기하 깊이 겹침·텍스처 겹침 이미지가 갱신된다.
- Given 다른 데이터셋의 유효 입력을 CLI에 넣었을 때, when 보고서를 생성하면, then 프런트엔드 수정 없이 동일 schema와 UI로 열린다.
- Given 진단 옵션이 꺼졌을 때, when 기존 parity 검사를 수행하면, then PEM 선택 결과가 변경되지 않는다.

## Spec Change Log

## Design Notes

보고서는 `index.html`, `report-data.js`, 상대경로 asset으로 구성해 `file://`에서도 작동시킨다. 6,000 후보는 브라우저에 모두 싣지 않고 단계별 성공 개수·최소 오차·OX와 선택 후보 식별자만 저장한다. 상위 후보만 자세/점수/시각 근거를 보존한다. 기하 이미지는 입력 depth와 투영 CAD의 일치/불일치 점을 색으로, 텍스처 이미지는 관측 RGB와 템플릿 색·특징 유사도를 heatmap으로 보여주며 정확 계산값과 시각화 proxy를 구분해 표기한다.

## Verification

**Commands:**
- `conda run -n sam6d python -m pytest -q tests/test_pem_explorer.py tests` -- 신규 및 기존 테스트 통과.
- `conda run -n sam6d python tools/build_pem_explorer.py --help` -- 재사용 CLI와 필수 입력 설명.
- longcircle2 생성 명령 -- schema 오류 없이 HTML·assets 생성, 내부 링크 누락 0건.

**Manual checks (if no CLI):**
- 검출/미검출/GT 없음/6000 미수집 프레임을 각각 열어 상태가 혼동되지 않는지, 화살표 방향과 후보 이미지가 bbox에 맞게 갱신되는지 확인한다.

## Suggested Review Order

**리포트 생성과 데이터 계약**

- 재사용 CLI가 입력 검증부터 정적 번들까지 전체 흐름을 조율한다.
  [`build_pem_explorer.py:275`](../tools/build_pem_explorer.py#L275)

- pseudo-GT 판정은 GT·SLAM 부재를 분리하고 임계값을 명시한다.
  [`build_pem_explorer.py:126`](../tools/build_pem_explorer.py#L126)

- ROS bag 추출은 명시적 RGB·camera-info topic과 정확 timestamp를 보존한다.
  [`build_pem_explorer.py:179`](../tools/build_pem_explorer.py#L179)

**PEM 진단 계측**

- 6000·300 단계에서 원시 행렬 저장 없이 정답 포함 여부를 집계한다.
  [`model_utils.py:269`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L269)

- 상위 후보의 실제 NN·특징 표본과 휴리스틱 겹침 근거를 분리한다.
  [`model_utils.py:309`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L309)

- 진단은 coarse 후보 선택 경계에서만 opt-in으로 활성화된다.
  [`model_utils.py:543`](../sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py#L543)

- realtime 경계가 객체·참조 자세·진단 출력을 연결한다.
  [`sam6d_core.py:174`](../realtime/sam6d_core.py#L174)

- 재실행기가 SLAM pseudo-GT를 프레임별 카메라 좌표로 변환한다.
  [`verify_eval.py:165`](../temp/verify_eval.py#L165)

**브라우저 탐색 UI**

- 상세 패널이 O/X·후보 점수·정확값과 proxy provenance를 함께 표시한다.
  [`app.js:64`](../tools/pem_explorer/app.js#L64)

- 프레임 캔버스가 manifest 정면·위 축을 동기화해 투영한다.
  [`app.js:46`](../tools/pem_explorer/app.js#L46)

- semantic 축 기본값은 검토 전 provisional 상태로 명시된다.
  [`default_axes.json:1`](../tools/pem_explorer/default_axes.json#L1)

**검증과 실제 산출물**

- 회귀 테스트가 전체·부분·무GT·미검출·불량 입력을 고정한다.
  [`test_pem_explorer.py:65`](../tests/test_pem_explorer.py#L65)

- 생성된 longcircle2 보고서는 외부 서버 없이 직접 열린다.
  [`index.html:1`](../output/longcircle2_pem_explorer/index.html#L1)
