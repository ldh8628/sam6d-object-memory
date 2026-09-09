# Investigation: YOLO-World·DINOv2·MobileSAM 후보 필터링 파이프라인

## Hand-off Brief

1. **What happened.** 사용자는 현재 `sam6d_ws`가 YOLO-World bbox 후보, DINOv2 CLS semantic 필터, MobileSAM 분할, patch DINOv2 appearance 필터를 실행 가능한 형태로 구현했는지 확인을 요청했다.
2. **Where the case stands.** Active — ROS 추론 노드에는 semantic/appearance score 호출이 직접 확인되지만, 같은 노드의 로딩 로그는 현재 FastSAM + DINOv2를 명시해 사용자 설명과 불일치 가능성이 있다.
3. **What's needed next.** 실제 호출 체인·모델 자산·실행 환경·Git 상태를 조사하고, 상대 PC의 커밋/브랜치 정보가 없다는 비교 한계를 분리해 판정한다.

## Case Info

| Field            | Value |
| ---------------- | ----- |
| Ticket           | N/A |
| Date opened      | 2026-07-20 |
| Status           | Active |
| System           | Ubuntu 22.04; system Python 3.10.12; investigation env Python 3.11.15 |
| Evidence sources | Source code, configuration, generated outputs, version control |

## Problem Statement

현재 입력 이미지가 주어졌을 때 YOLO-World로 bbox 후보를 선정하고, 후보의 DINOv2 CLS feature와 SAM-6D 렌더링 템플릿 이미지 42장의 CLS feature를 비교해 semantic score로 1차 필터링한 뒤, MobileSAM으로 객체를 구별하고 patch 단위 DINOv2 appearance score로 2차 필터링하는 프로젝트가 실행 가능한지, 즉 다른 PC의 진행 상황과 동일한지 확인한다.

## Evidence Inventory

| Source   | Status | Notes |
| -------- | ------ | ----- |
| Source code | Partial | FastSAM ROS 경로와 MobileSAM standalone 경로, DINOv2 두 점수는 존재; YOLO-World 및 요청 순서의 통합 경로는 없음 |
| Runtime configuration | Partial | 기본 ROS 경로는 FastSAM, candidate filter 기본 false, semantic/appearance 최소값 0.0 |
| Model weights | Partial | FastSAM, DINOv2, PEM, MAE 존재; YOLO-World, MobileSAM, legacy SAM vit_h 누락 |
| Rendered templates | Available | 23세트, 각 RGB/mask/xyz 42개(총 2,898파일) 확인 |
| Dependency environments | Partial | `sam6d_test`에 핵심 FastSAM/DINOv2 패키지는 있으나 `mobile_sam`, YOLO-World 계열 의존성, xformers 누락 |
| Feature cache | Missing | CLS/patch descriptor disk cache 없음; MobileSAM runner는 시작 시 42-view를 재계산 |
| Generated outputs | Partial | FastSAM+DINOv2 실행 및 score 산출물은 있으나 요청된 YOLO-World+MobileSAM end-to-end 결과는 없음 |
| Tests/static analysis | Partial | 기존 오프라인 평가 보고서는 있으나 JUnit/pytest 결과, YOLO-World/MobileSAM 실행 로그 없음 |
| Diagnostic archives | Missing | 요청 파이프라인을 재현하는 별도 진단 아카이브 없음 |
| Issue tracker | Missing | 관련 티켓/PR 식별자 없음 |
| Version control | Partial | 로컬 HEAD `503e871d...`, branch `main`; 4 modified, 3 deleted, 17,798 untracked 파일로 dirty |
| Other-PC state | Missing | 상대 PC의 HEAD, branch, dirty diff 또는 전달 아카이브가 제공되지 않음 |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | ROS/standalone inference caller chain | High | Done | ROS=FastSAM; standalone MobileSAM은 mask→semantic→appearance; YOLO-World 없음 |
| 2 | 템플릿 수와 feature cache | High | Done | 23세트×42-view 확인; disk feature cache 없음 |
| 3 | 모델 코드·가중치·Python/CUDA 의존성 | High | Done | YOLO-World 전체 및 MobileSAM 실행 의존성 누락 확인 |
| 4 | 테스트·실행 로그·산출물 | High | Done | FastSAM 실행 증거만 있으며 요청 파이프라인 성공 증거 없음 |
| 5 | Git branch/HEAD/dirty/untracked/remote | High | Done | 로컬 식별자 산출; dirty worktree 때문에 HEAD만으로 비교 불가 |
| 6 | 사용자 가설의 원인/상태 판정 | High | Open | 증거 간 인과와 반증을 정리하여 최종 판정 |
| 7 | 다른 PC manifest 비교 | High | Blocked | 상대 PC의 Git/diff/asset/environment manifest 필요 |

## Timeline of Events

| Time | Event | Source | Confidence |
| ---- | ----- | ------ | ---------- |
| 2026-07-20 | 조사 시작, 프로젝트 후보를 `sam6d_ws`로 설정 | 사용자 요청 및 저장소 구조 | Deduced |
| 2026-07-20 | 로컬 Git HEAD와 branch 확인 | git | Confirmed |

## Confirmed Findings

### Finding 1: 기존 ROS 노드에는 DINOv2 semantic/appearance 단계가 있다

**Evidence:** `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:879`, `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:881`, `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:889`

**Detail:** query descriptor 생성 뒤 `compute_semantic_score()`와 `compute_appearance_score()`를 순차 호출한다.

### Finding 2: 현재 강한 진입점은 FastSAM을 명시한다

**Evidence:** `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:427`

**Detail:** 모델 로딩 로그가 `ISM (FastSAM + DINOv2)`를 명시하므로, 사용자 가설의 YOLO-World + MobileSAM 조합은 아직 확인되지 않았다.

### Finding 3: 42-view 템플릿 자산은 충분히 존재한다

**Evidence:** `output/ply_test/saffron/low_texture_around/run.log:17`, `output/ply_test/saffron/low_texture_around/run.log:18`, `output/ply_test/saffron/low_texture_around/run.log:19`

**Detail:** 런타임 로그에서 RGB/mask/xyz가 각각 42개로 확인되며, 파일시스템에는 같은 구조의 템플릿 세트가 23개 있다.

### Finding 4: YOLO-World 증거는 전 범위에서 누락됐다

**Evidence:** source/config/weight/dependency 전 범위 검색 결과 0건; 현재 설정은 `src/sam6d_ros/config/params.yaml:44`에서 FastSAM을 선택한다.

**Detail:** YOLO-World adapter, text prompt 설정, checkpoint, Python dependency, bbox-to-MobileSAM 연결이 모두 없다.

### Finding 5: MobileSAM은 standalone 소스만 있고 현재 실행 조건은 충족되지 않는다

**Evidence:** `sam6d_master/SAM-6D/run_batch_inference_mobileSam.py:61`, `sam6d_master/SAM-6D/run_batch_inference_mobileSam.py:93`, `sam6d_master/SAM-6D/run_batch_inference_mobileSam.py:648`

**Detail:** `mobile_sam` package와 `Instance_Segmentation_Model/checkpoints/mobile_sam/mobile_sam.pt`가 없고, loader가 먼저 요구하는 legacy SAM vit_h checkpoint도 없다. ROS 노드에는 MobileSAM 선택지가 연결되지 않았다.

### Finding 6: 작업공간은 커밋 상태와 크게 다르다

**Evidence:** Git status/diff inventory — modified 4, deleted 3, untracked 17,798; tracked diff 577 insertions/87 deletions.

**Detail:** `main`과 로컬 `origin/main` ref는 같은 HEAD지만 실제 실행 코드·가중치·템플릿의 상당 부분이 미커밋 또는 untracked 상태다.

## Deduced Conclusions

현재 단계에서는 사용자 설명 전체가 이 PC에 구현됐다고 결론 낼 수 없다. 기존 SAM-6D 계열 semantic/appearance 계산 경로는 확인됐지만 proposal/segmentation 모델이 요청 조합인지 추가 추적이 필요하다.

## Hypothesized Paths

### Hypothesis 1: 이 PC는 다른 PC와 동일한 YOLO-World + MobileSAM 2단계 파이프라인을 보유한다

**Status:** Open

**Theory:** 별도 모듈 또는 최근 변경분이 기존 ROS 노드에 연결되어 사용자 설명과 같은 파이프라인을 제공한다.

**Supporting indicators:** semantic/appearance score 호출과 DINOv2 feature 단계가 존재한다.

**Would confirm:** YOLO-World bbox 출력이 MobileSAM/DINOv2 단계에 전달되는 호출 체인, 42개 템플릿 feature, 필요한 자산과 성공 실행 결과를 확인하고 상대 PC Git 식별자가 일치함.

**Would refute:** 실제 활성 경로가 FastSAM proposal만 사용하거나 YOLO-World/MobileSAM 코드·자산·설정이 없고, 상대 PC 기준과 Git 상태가 다름.

**Resolution:** Open.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| 상대 PC Git HEAD/branch/status/diff | “동일” 여부를 확정할 수 없음 | 상대 PC에서 HEAD/branch/status와 `git diff --binary -- sam6d_ws` SHA-256 수집 |
| 상대 PC untracked asset manifest | checkpoint/template/실험 스크립트 동일성 확인 불가 | 정렬된 상대경로+SHA-256 manifest 수집 |
| 대표 입력 이미지 및 기대 출력 | end-to-end 재현 판정 제한 | 사용 중인 입력 이미지/launch 명령 확보 |
| YOLO-World 실행 코드·가중치·환경 | 요청 파이프라인의 첫 단계가 로컬에 없음 | 다른 PC 또는 전달 소스에서 확보 |
| MobileSAM 패키지·가중치 | standalone 스크립트 실행 불가 | `mobile_sam`, `mobile_sam.pt`, loader 의존 SAM weight/구조 수정 확보 |
| 요청 파이프라인 end-to-end 로그 | 정확한 단계 순서와 성공 여부 확인 불가 | 동일 입력으로 단계별 count/score/timing 로그 생성 |

## Source Code Trace

| Element       | Detail |
| ------------- | ------ |
| Error origin  | N/A — 기능/상태 탐색 |
| Trigger       | ROS inference node의 입력 프레임 처리 |
| Condition     | ISM 모델과 렌더링 템플릿이 로드된 상태 |
| Related files | `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py`, `src/sam6d_ros/config/params.yaml` |

## Conclusion

**Confidence:** Low

기존 DINOv2 semantic/appearance 계산은 Confirmed이나 YOLO-World + MobileSAM 조합, 정확한 42장 템플릿, 실행 준비 상태, 상대 PC와의 동일성은 아직 Open이다.

## Recommended Next Steps

### Diagnostic

소스·설정·자산·Git·기존 실행 산출물의 증거 범위를 매핑한다.

## Reproduction Plan

활성 launch/CLI 경로와 입력 자산을 확인한 뒤 dry import, 모델 자산 검사, 가능한 최소 입력 실행 순으로 검증한다.
