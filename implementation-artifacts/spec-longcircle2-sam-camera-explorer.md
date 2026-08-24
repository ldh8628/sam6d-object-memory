---
title: 'longcircle2 SAM 전용 RGB-D 결과 전환'
type: 'feature'
created: '2026-08-23'
status: 'done'
baseline_commit: 'NO_VCS'
context:
  - 'implementation-artifacts/spec-reusable-pem-pose-explorer.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** 현재 `data/longcircle2`와 기본 PEM HTML은 동시간대 SLAM 카메라 영상을 사용한다. 실제 SAM 전용 카메라의 RGB-D는 Windows의 `C:\LANShare\260804_SAM\longcircle2`에 별도 저장되어 있으며, RealSense SDK 원시 토픽이라 현 파이프라인이 직접 읽을 수 없다.

**Approach:** 원본은 읽기 전용 심볼릭 링크로 보존하고, 검증된 변환기로 color-aligned 표준 ROS2 bag을 로컬 `data/`에 생성한다. 이 bag으로 ISM/PEM 진단과 독립 HTML을 다시 만들고, 기존 SLAM 결과는 비교용으로 보존한 채 SAM 결과를 기본 진입점으로 제공한다.

## Boundaries & Constraints

**Always:** 원본 경로·파일 크기·메시지 수·2,130개 RGB-D pairing과 시계 보정 근거를 provenance에 기록한다. SAM 원본/변환본/SLAM 기존본을 이름으로 명확히 구분한다. 변환본은 표준 color, aligned-depth, CameraInfo 토픽을 제공하고 실행 전후 무결성을 검사한다. 기존 보고서와 데이터를 삭제하거나 덮어쓰지 않는다. 보고서에는 9개 non-Rabbit 객체, 후보·기하·텍스처 근거와 데이터셋/카메라 출처를 표시한다. 카메라 간 검증된 extrinsic이 없는 동안 pseudo-GT와 6,000/300 O/X는 `GT 없음(extrinsic_missing)`으로 표시한다.

**Ask First:** 추정한 SAM↔SLAM extrinsic을 정답 변환으로 채택하는 경우, 기존 산출물을 삭제하는 경우, Windows 원본을 수정하는 경우.

**Never:** SLAM 카메라 pseudo-GT를 SAM 카메라 pose에 직접 적용하지 않는다. 미수집·무GT를 X로 표기하지 않는다. Windows 마운트가 없거나 변환이 불완전한 상태를 성공으로 공개하지 않는다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 정상 변환 | SAM raw bag + association/manifest | `data/longcircle2_sam`에 2,130쌍의 표준 aligned RGB-D bag | 토픽·count·첫/끝 stamp 검증 |
| 마운트 부재 | 원본 symlink target 없음 | 기존 로컬 변환본/HTML은 계속 사용 가능 | 재변환만 중단하고 원인 출력 |
| 무GT 실행 | inter-camera extrinsic 없음 | 후보 점수·근거는 표시, O/X는 `GT 없음` | SLAM pseudo-GT 주입 거부 |
| 일부 변환 | DB/metadata/count 불일치 | 추론·HTML 생성을 시작하지 않음 | 불완전 산출물을 별도 표시 후 실패 |

</frozen-after-approval>

## Code Map

- `/media/etri/04D4788ED4788428/LANShare/260804_SAM/longcircle2/` -- 확정된 SAM 전용 원본(2,166 RGB, 2,130 depth, LiDAR/IMU 없음).
- `/media/etri/04D4788ED4788428/LDH_ws/ros2bag_convert_gui/convert_recording.py` -- SDK 원본을 aligned 표준 ROS2 bag으로 바꾸는 검증된 변환기.
- `data/longcircle2_sam/dataset_manifest.json` -- bag 무결성 invariant와 SAM 카메라 GT 정책을 코드가 신뢰하는 manifest.
- `tools/validate_rgbd_dataset.py` -- 추론과 report 생성 전에 manifest·metadata·SQLite topic/count/stamp를 fail-closed 검증하는 공통 게이트.
- `temp/verify_eval.py` -- 표준 bag의 ISM/PEM 진단 실행기.
- `tools/build_pem_explorer.py` -- 진단 결과와 bag을 정적 PEM explorer로 생성.
- `tools/pem_explorer/` -- 재사용 정적 UI.
- `output/pem_explorer/index.html` -- 데이터셋별 독립 보고서로 이동하는 기본 진입점.

## Tasks & Acceptance

**Execution:**
- [x] `data/longcircle2_sam_raw`, `data/longcircle2_sam/PROVENANCE.md`, `data/longcircle2_sam/dataset_manifest.json` -- 원본 링크와 재현 가능한 변환본을 구축하고 기계 검증 가능한 count/topic/stamp·GT 정책을 기록한다.
- [x] `data/longcircle2/dataset_manifest.json`, `output/longcircle2_pem_explorer/README.md` -- SLAM same-camera manifest와 재현 가능한 sam6d 환경 명령을 일치시킨다.
- [x] `tools/validate_rgbd_dataset.py`, `temp/verify_eval.py` -- camera role/relation과 GT policy의 교차 일관성 및 trusted reference pose의 유한 SO(3) 조건을 검증해 SAM pseudo-GT 우회를 차단한다.
- [x] `tools/build_pem_explorer.py`, `tools/pem_explorer/app.js` -- 같은 trusted pose·stage schema 검증과 기존 manifest identity/invalid-pose/안전 교체 동작을 유지한다.
- [x] `output/longcircle2_sam_diagnostic/` -- 2,130 프레임에 대해 무GT full ISM/PEM 후보 진단을 실행한다.
- [x] `output/pem_explorer/longcircle2_sam/` -- invalid-pose 상태를 포함해 SAM 카메라 HTML을 안전하게 재생성하고 브라우저 오류·누락 asset을 검사한다.
- [x] `output/pem_explorer/index.html` -- SAM 결과를 기본으로 열고 기존 SLAM 보고서를 명시적으로 연결한다.
- [x] `tests/test_pem_explorer.py`, `tests/test_rgbd_dataset.py` 및 데이터 검증 명령 -- distinct-camera/same-camera 모순, malformed trusted pose/stage/manifest, SAM/SLAM 재사용과 기존 회귀를 검증한다.

**Acceptance Criteria:**
- Given 기본 explorer를 열었을 때, when 대표 프레임을 확인하면, then SLAM 카메라가 아닌 SAM 전용 RGB 영상 위에 9객체 결과와 축이 표시된다.
- Given 객체 상세를 열었을 때, when 상위 N 후보를 선택하면, then pose·기하/텍스처 점수와 겹침 이미지가 동기화되고 O/X는 위조 없이 `GT 없음`이다.
- Given 기존 SLAM 결과가 필요할 때, when dataset 목록을 선택하면, then 별도 비교용 보고서로 접근 가능하다.

## Spec Change Log

- 2026-08-23: 확정 SAM 원본을 읽기 전용 링크로 연결하고 검증된 변환기로
  2,130쌍의 표준 aligned bag을 생성했다.
- 2026-08-23: 첫 color가 첫 depth보다 먼저 오는 bag에서 첫 프레임을 버리던
  `verify_eval.py`의 pending 경계를 수정하고 2,130프레임 full diagnostic을 재실행했다.
- 2026-08-23: report schema/UI에 dataset·camera source와
  `extrinsic_missing` 무GT 상태를 추가하고, 해당 상태에서 pseudo-GT 주입을 거부했다.
- 2026-08-23: 독립 SAM report와 SAM-default/SLAM-comparison landing을 생성했다.
- 2026-08-23 review loop 1: 리뷰에서 camera/GT 정책이 호출자 문자열에만 의존하고
  불완전 bag preflight가 없어서 frozen Never 및 partial-conversion 규칙을 우회할 수 있음이
  확인됐다. 신뢰 manifest와 추론/builder 공통 무결성 게이트, bounded pending 및 안전한
  publication 작업을 추가해 잘못된 SAM O/X와 부분 결과 공개를 방지한다. KEEP: 검증된
  2,130쌍 변환본, 2,979검출 무GT 진단, `extrinsic_missing` UI, 기존 SLAM 보존, SAM 기본
  landing, 19개 통과 테스트와 브라우저 preview는 유지한다.
- 2026-08-23 review loop 2: manifest 부재 시 generic 기본값으로 GT 정책과 pair-count가
  fail-open되고 CLI 출처 문자열이 manifest를 덮을 수 있으며, 88개 zero/unprojectable 최종
  pose가 일반 검출로 표시됨이 확인됐다. 실행 진입점의 in-bag manifest 필수화, 엄격한 GT
  boolean/policy, manifest 권위 identity, `pem_pose_invalid` 상태와 해당 회귀 검증을 추가해
  부분/오인 데이터 공개를 막는다. KEEP: loop 1의 정확한 SQLite/stamp-set validator,
  bounded sync, staged publication, 29개 통과 테스트, 모든 기존 대용량 산출물과 해시는 유지한다.
- 2026-08-23 review loop 3: mandatory manifest와 safe-replace 도입 후 기존 SLAM 비교
  README의 재생성 명령이 manifest 부재/replace 플래그 때문에 깨지고, duplicate/undeclared
  topic·비boolean trusted/stage 값과 임의 camera-topic override가 신뢰 경계를 흐릴 수 있음이
  확인됐다. SLAM same-camera manifest와 갱신 명령, manifest-declared auxiliary topics,
  strict boolean, canonical topic 강제, completed-report 교체 guard를 추가한다. KEEP: 38개
  테스트, SAM 2,130/2,979/89-invalid 결과, GT 전부 extrinsic_missing, missing asset 0,
  기존 diagnostic/SLAM report와 landing은 유지한다.
- 2026-08-23 review loop 4: distinct SAM camera manifest에서 GT block만
  `same_camera_reference`로 바꾸면 pseudo-GT 금지를 우회할 수 있음이 실제 재현됐다.
  camera role/relation↔GT policy 교차 검증, trusted reference pose의 finite SO(3), malformed
  stage/manifest의 통제된 거부와 재현 명령 환경을 추가해 자기모순 manifest와 잘못된 O/X를
  막는다. KEEP: 56개 테스트, 두 실제 manifest의 정확한 topic 통계, SAM/SLAM report-data와
  diagnostic 해시/mtime, 89 invalid-pose UI를 유지한다.

## Design Notes

SAM과 SLAM 첫 프레임은 manifest의 호스트 시계 오프셋 `+6,601,183,400 ns` 적용 후 약 `0.260 ms` 차이로 동시간대임이 확인된다. 그러나 시간 동기만으로 카메라 좌표계가 같아지지는 않으므로 O/X 활성화에는 별도의 검증된 `T_sam_from_slam`이 필요하다. 단일 거대 JS에서 데이터셋을 동적 교체하지 않고 독립 보고서와 가벼운 landing page를 사용해 기존 UI 상태·asset 오염을 피한다.

## Verification

**Commands:**
- `conda run -n sam6d python <converter> <raw-link> data/longcircle2_sam` -- 표준 bag 생성 완료.
- `ros2 bag info data/longcircle2_sam` -- RGB/depth/CameraInfo 각 2,130개와 예상 duration 확인.
- `conda run -n sam6d python -m pytest -q tests/test_pem_explorer.py` -- explorer 회귀 통과.
- HTML asset 검사 및 headless browser smoke test -- 깨진 링크·콘솔 오류 0건, SAM 프레임 확인.

**Actual results (2026-08-23):**

- converted bag: 8,520 messages; RGB/depth/두 CameraInfo topic 각각 2,130개,
  고유 stamp 2,130개와 동일 first/last stamp 확인.
- full diagnostic: frame 2,130, detection 2,979, detection당 후보 정확히 100개,
  pointwise record 39,331개; stage 6,000/300 모두 무GT 상태로 수집.
- explorer: frame 2,130, detection 2,979, candidate 297,900, 9-slot/frame,
  `extrinsic_missing` 외 GT 상태 없음, 누락 asset 0개. 최종 pose는 정상 `detected`
  2,890건과 `pem_pose_invalid` 89건으로 분리되며 invalid 89건도 후보 100개에 접근 가능하다.
- tests: 최종 review patch 이후 전체 `78 passed`; Chrome child/landing render exit 0,
  페이지 오류 0건. frame 148 preview에서 invalid 경고와 후보 100개를 함께 렌더했다.
- trusted dataset gate: 실제 SAM manifest/metadata/SQLite의 네 토픽 count·type·고유
  stamp·전체 stamp 집합이 2,130쌍으로 일치하며 pseudo-GT 정책 거부 확인. 기존 SLAM
  bag은 표준 네 토픽 각 2,166개와 manifest-declared auxiliary 8개(69,345 messages), 전체
  78,009 messages가 metadata/SQLite와 정확히 일치하고 same-camera pseudo-GT만 허용한다.
- bounded reader: 실제 bag에서 2,130 frames, unmatched 0, EOF pending 0 확인.
- safe publication: synthetic late I/O failure에서 기존 completed report 보존 및 staging
  debris 0; symlink/file/unrelated directory replacement 거부와 completed PEM marker target만
  교체 가능함을 확인했다.
- KEEP hashes: diagnostic dump `1ef48e8b...`, frame dump `3a5b25b6...`, 변환 bag DB
  `231cd372...`가 review loop 전 해시를 유지했다. 명세대로 report만 재생성해 report-data는
  `5785b5fc...`, Chrome preview는 `8de6cd2c...`로 갱신됐다.
- review loop 3 KEEP: SAM/SLAM report-data는 각각 `5785b5fc...`/`e9082baa...`,
  SAM diagnostic dump/frame dump는 `1ef48e8b...`/`3a5b25b6...`를 유지했다. 대용량
  report/diagnostic은 재생성하지 않고 SLAM README 명령과 두 bag manifest만 갱신했다.
- review loop 4 trust boundary: `same_camera_reference`는 명시적인
  `slam_reference_camera`/`same_camera` 조합에서만 허용하며 dedicated/distinct camera의
  policy mutation을 거부한다. 실제 SLAM pseudo-GT의 `trusted is True` 5객체는 finite SO(3)
  검증을 통과했고 malformed trusted pose/stage/manifest는 model·report output 전에 실패한다.
  SLAM README 명령은 절대 conda 경로와 `--no-capture-output -n sam6d`를 사용한다.
- final edge patch: SQLite orphan message, numeric overflow, non-object diagnostic을 통제된
  오류로 처리하며 두 기존 report와 신규 report에 `PEM_EXPLORER_COMPLETE_V1` ownership
  marker를 사용한다. 대용량 diagnostic/report-data 해시는 변경되지 않았다.

## Suggested Review Order

**신뢰 경계와 GT 정책**

- 공통 진입점은 manifest·metadata·SQLite·카메라/GT 정책을 함께 검증한다.
  [`validate_rgbd_dataset.py:299`](../tools/validate_rgbd_dataset.py#L299)

- Trusted reference pose는 finite SO(3)와 유한 translation만 허용한다.
  [`validate_rgbd_dataset.py:391`](../tools/validate_rgbd_dataset.py#L391)

- 추론은 모델 로드 전에 SAM pseudo-GT와 불완전 bag을 차단한다.
  [`verify_eval.py:267`](../temp/verify_eval.py#L267)

- Builder는 같은 검증을 거쳐 staging report만 안전하게 공개한다.
  [`build_pem_explorer.py:637`](../tools/build_pem_explorer.py#L637)

**결과 표현과 안전한 교체**

- 투영 불가능 최종 pose를 후보가 남는 별도 상태로 변환한다.
  [`build_pem_explorer.py:516`](../tools/build_pem_explorer.py#L516)

- Ownership marker가 있는 완성 PEM report만 교체할 수 있다.
  [`build_pem_explorer.py:589`](../tools/build_pem_explorer.py#L589)

- UI는 invalid pose와 extrinsic 없는 GT를 O/X와 구분한다.
  [`app.js:22`](../tools/pem_explorer/app.js#L22)

**데이터와 탐색 진입점**

- SAM manifest는 2,130쌍과 distinct-camera GT 금지를 고정한다.
  [`dataset_manifest.json:1`](../data/longcircle2_sam/dataset_manifest.json#L1)

- SLAM manifest는 2,166쌍과 선언된 보조 토픽을 고정한다.
  [`longcircle2/dataset_manifest.json:1`](../data/longcircle2/dataset_manifest.json#L1)

- Landing은 SAM을 기본으로 열고 기존 SLAM 비교를 보존한다.
  [`index.html:18`](../output/pem_explorer/index.html#L18)

**회귀 검증**

- Synthetic/실제 bag 검증이 manifest·topic·GT 우회를 고정한다.
  [`test_rgbd_dataset.py:222`](../tests/test_rgbd_dataset.py#L222)

- Explorer 검증이 invalid pose·안전 교체·재생성 계약을 고정한다.
  [`test_pem_explorer.py:400`](../tests/test_pem_explorer.py#L400)
