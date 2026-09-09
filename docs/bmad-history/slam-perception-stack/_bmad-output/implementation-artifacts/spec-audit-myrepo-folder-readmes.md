---
title: 'MyRePo 17개 폴더 README 감사와 루트 인덱스'
type: 'chore'
created: '2026-08-05T00:00:00+09:00'
status: 'done'
baseline_commit: 'ee7bfd7f336d400bdbe27e44f5fa44b1f906da51'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** MyRePo `main`의 최상위 17개 프로젝트 폴더 중 10개에는 `README.md`가 없고, `DL_stitch/README.md`는 upstream `nie-lang/UnsupervisedDeepImageStitching` 문서와 SHA-256까지 동일하다. 저장소 루트 README도 없어 각 프로젝트의 목적과 문서 맞춤 여부를 한눈에 확인할 수 없다.

**Approach:** commit `ee7bfd7`의 실제 Git tree와 주요 진입점을 근거로 누락된 10개 README와 MyRePo 맞춤 `DL_stitch` README를 직접 작성한다. 이미 로컬 변경·입력·결과를 설명하는 6개 맞춤 README는 보존하고, 루트 `README.md`에 17개 폴더 링크·한 문장 소개·문서 판정을 표로 정리한다.

## Boundaries & Constraints

**Always:** 최상위 디렉터리는 `scripts`를 제외한 현재 17개를 기준으로 한다. 새 문서는 한국어로 직접 작성하고 목적, 주요 파일, 최소 실행법, 입력·출력, 확인된 제한을 해당 폴더의 실제 코드에 맞춘다. upstream 기반 코드는 출처를 밝히되 MyRePo의 포함 내용과 실행 현실을 먼저 설명한다. 폴더명에 공백이 있는 링크도 GitHub에서 동작해야 한다.

**Ask First:** 프로젝트 소스·데이터·모델·결과 파일 변경이나 삭제, 새 의존성 manifest 추가, GitHub push·PR·`main` 병합이 필요하면 먼저 확인한다.

**Never:** upstream README를 그대로 복사하거나 확인하지 않은 실행 성공·성능을 주장하지 않는다. `scripts`를 18번째 프로젝트로 표에 넣지 않고, 기존 6개 맞춤 README의 provenance·실행 결과를 근거 없이 축약하지 않는다. 문서 작업을 빌미로 대용량 데이터·가중치·캐시를 정리하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 기존 맞춤 문서 | 로컬 수정·환경·결과가 명시된 6개 README | 보존하고 루트 표에서 `맞춤 작성`으로 연결 | 사실 오류가 발견되면 해당 문장만 근거와 함께 수정 |
| upstream 동일 문서 | `DL_stitch/README.md`가 upstream과 byte-identical | MyRePo 포함 범위·실행·한계를 설명하는 새 문서로 교체하고 upstream 링크 유지 | 원본 저자·논문을 로컬 성과로 표현하지 않음 |
| README 누락 | 10개 폴더에 최상위 README 없음 | 각 코드·자산에 맞는 직접 작성 README 생성 | 단일 진입점이 없으면 스크립트별 역할과 선택 실행을 명시 |
| 공백 폴더명 | `Pattern Recognition` | 루트 표와 로컬 링크 검사가 모두 통과 | URL 인코딩 또는 유효한 Markdown 상대 링크 사용 |

</frozen-after-approval>

## Code Map

- `DL_stitch/README.md` -- upstream과 완전히 동일해 맞춤 재작성 대상인 유일한 기존 문서.
- `{Intelligence_capston_opencv,Pattern Recognition,Pattern_recognition_project,ViT_AutoEncoder,hangle_normalizer}/` -- README가 없는 영상·패턴인식·한국어 실험군.
- `{mesh_based_stitch,opencv_number,picamera,project_bank,ros2_image_subscribe}/` -- README가 없는 파노라마·GUI·하드웨어·웹·ROS 2 실험군.
- `{FoundPose,ImprovedFoundPose,ORB3-SLAM,HDL-GRAPH-SLAM,RTAB-MAP,SAM-6D}/README.md` -- 이미 MyRePo 환경과 provenance를 설명하는 보존 대상.
- `README.md` -- 17개 폴더 한 문장 요약과 README 감사 결과의 저장소 진입점.

## Tasks & Acceptance

**Execution:**
- [x] `DL_stitch/README.md` -- upstream 원문을 MyRePo 포함 코드·데이터 규모·실행 경로·출처·한계 중심 문서로 교체한다.
- [x] 누락 10개 `*/README.md` -- 실제 진입 파일과 입출력 계약에 기반한 맞춤 문서를 생성한다.
- [x] 기존 맞춤 6개 `*/README.md` -- upstream 복사본이 아님을 근거로 판정하고 불필요한 재작성 없이 유지한다.
- [x] `README.md` -- 17개 폴더의 링크, 한 문장 소개, README 상태를 빠짐없이 표로 작성한다.
- [x] 문서 검증 -- 17개 README 존재, 표의 17행·상대 링크, upstream byte-identical 해소, whitespace를 검사한다.

**Acceptance Criteria:**
- Given `main`의 최상위 프로젝트 폴더, when `scripts`를 제외해 열거하면, then 정확히 17개이며 모두 자기 폴더의 `README.md`를 가진다.
- Given 루트 README, when GitHub에서 표를 읽으면, then 각 폴더가 한 번씩 링크되고 한 문장만으로 목적을 구분할 수 있다.
- Given 각 새 README, when 실제 코드의 진입점·입출력·제약과 대조하면, then 존재하지 않는 명령·자산·성과를 주장하지 않는다.
- Given `DL_stitch`, when 공식 upstream README와 비교하면, then 더 이상 동일하지 않고 upstream 출처와 MyRePo 고유 안내가 함께 남는다.

## Spec Change Log

## Design Notes

README 상태는 `맞춤 작성`, `이번 감사에서 맞춤 작성` 두 값만 사용한다. 이는 코드의 독창성 판정이 아니라 최상위 문서가 이 저장소의 실제 내용과 실행 조건을 설명하는지에 대한 판정이다.

## Verification

**Commands:**
- `python3` 문서 감사 스크립트(임시 실행) -- `scripts` 제외 17개 폴더 모두 README 존재, 루트 표 링크가 실제 경로로 해석됨.
- `cmp DL_stitch/README.md <(curl .../UnsupervisedDeepImageStitching/main/README.md)` -- 동일하지 않음.
- `git diff --check && git status --short` -- 문서 외 변경과 whitespace 오류 없음.

## Suggested Review Order

**저장소 진입점과 감사 판정**

- 17개 프로젝트의 목적, 링크, 맞춤 문서 판정을 한 표에서 확인한다.
  [`README.md:1`](../../../foundpose/release/MyRePo-readme-audit/README.md#L1)

**Upstream 기반 실행 경계**

- UDIS 출처, 누락 가중치, 2단계 실행 계약을 로컬 tree 기준으로 재작성했다.
  [`DL_stitch/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/DL_stitch/README.md#L1)

**학습·평가 실험 안내**

- ViT reconstruction 실험의 필수 class와 checkpoint 복구 경계를 명시한다.
  [`Pattern Recognition/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/Pattern%20Recognition/README.md#L1)

- DRAEM·PatchCore·CycleGAN 비교의 대표 실행과 provenance 한계를 정리한다.
  [`ViT_AutoEncoder/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/ViT_AutoEncoder/README.md#L1)

- Notebook별 선형회귀와 Iris 분류 역할을 실제 포함 파일에 맞춘다.
  [`Pattern_recognition_project/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/Pattern_recognition_project/README.md#L1)

- Ollama 준비부터 한국어 정규화 평가까지 재현 순서를 연결한다.
  [`hangle_normalizer/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/hangle_normalizer/README.md#L1)

**영상·GUI·웹 실습 안내**

- Kaggle cache 영상과 OpenCV 고정 입력의 실제 준비 단계를 설명한다.
  [`Intelligence_capston_opencv/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/Intelligence_capston_opencv/README.md#L1)

- Mesh·DL stitching 경로를 완성도와 출력별로 구분한다.
  [`mesh_based_stitch/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/mesh_based_stitch/README.md#L1)

- 숫자 GUI의 조작법과 bundled pickle 신뢰 경계를 앞에 둔다.
  [`opencv_number/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/opencv_number/README.md#L1)

- Raspberry Pi 전용 카메라·cascade 의존성을 하드웨어 실행 관점에서 정리한다.
  [`picamera/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/picamera/README.md#L1)

- Flask 데모의 테스트 계정과 외부 노출 금지 조건을 명확히 한다.
  [`project_bank/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/project_bank/README.md#L1)

**ROS 2 멀티카메라 안내**

- 캡처·파노라마·탐지 진입점과 model provenance 한계를 구분한다.
  [`ros2_image_subscribe/README.md:1`](../../../foundpose/release/MyRePo-readme-audit/ros2_image_subscribe/README.md#L1)
