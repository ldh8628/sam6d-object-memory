---
title: '0729 RealSense bag 표준 RGB-D 변환'
type: 'feature'
created: '2026-07-30'
status: 'done'
baseline_commit: '1f221f6a944fc0e9ece77d87792c55dbef67fac9'
review_loop_iteration: 5
context:
  - 'docs/03_DATASETS.md'
  - '_bmad-output/implementation-artifacts/investigations/ros2-bag-slam-sam6d-compatibility-investigation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `data_slam/0729/`의 세 SDK-style bag에는 RGB·raw depth·문자열 보정값만 있어 ORB-SLAM3·RTAB-Map·SAM-6D full pose의 표준 입력 계약을 만족하지 않는다. 기존 변환기는 필요한 정렬 로직이 있지만 폴더명과 DB 파일명이 같다고 가정해 현재 구조에서 실패한다.

**Approach:** 기존 depth→color 재투영 변환기를 현재 구조에 맞게 안전하게 일반화하고, 세 세션을 표준 RGB-D bag과 세션별 ORB settings YAML로 변환한 뒤 데이터 계약을 검증한다.

## Boundaries & Constraints

**Always:** 원본 `0729_*`를 읽기 전용으로 유지한다. 출력은 `data_slam/0729/{converted,settings}/`에 쓴다. bag의 intrinsics, depth unit, depth→color extrinsic, host-epoch association timestamp만 사용한다. 출력은 표준 color, color-aligned depth, color/depth CameraInfo 4개 토픽이며 color=`bgr8`, depth=`16UC1` mm, frame=`camera_color_optical_frame`이다. 기존 출력은 덮어쓰지 않는다.

**Ask First:** DB가 없거나 복수인 경우, 필수 보정·association이 없는 경우, 디스크가 부족한 경우, 원본 수정이 필요한 경우.

**Never:** LiDAR·IMU·TF 또는 임의 보정값을 합성하지 않는다. HDL 호환을 주장하지 않는다. 모델 자산 복원, workspace 빌드, SLAM 추론, Git commit/push는 범위 밖이다.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|---------------|----------------------------|----------------|
| 정상 | 단일 DB와 필수 보정·association | 동기 RGB·aligned depth·CameraInfo와 ORB YAML | N/A |
| 이름 불일치 | `0729_*` 안의 `recording_*.db3` | 단일 DB 자동 탐색 | 폴더명 기반 경로를 강제하지 않음 |
| 모호/충돌 | DB 0개·복수 또는 기존 출력 | 아무것도 덮어쓰지 않음 | 후보/충돌 경로를 보고하고 실패 |
| 입력 위반 | 해상도·encoding·필수 필드 불일치 | 잘못된 기하 출력 미생성 | 세션과 위반 항목을 보고하고 실패 |

</frozen-after-approval>

## Code Map

- `data_slam/260714_frame_data/convert_recording.py` -- 기존 SDK bag→표준 RGB-D 변환 및 z-buffer 정렬.
- `data_slam/260714_frame_data/test_convert_recording_contract.py` -- 변환 안전 계약 단위 테스트.
- `data_slam/0724_chungbuk/make_orb_settings_sdk.py` -- 원본 보정으로 ORB settings 생성.
- `data_slam/0729/0729_*/` -- 수정 금지 원본.
- `data_slam/0729/{converted,settings}/` -- 신규 출력.
- `verification-convert-0729-rgbd-bags.md` -- 변환·원본 보존 검증 기록.

## Tasks & Acceptance

**Execution:**
- [x] `convert_recording.py` -- 모든 `*.db3`를 열거해 정확히 1개만 허용하고, immutable SQLite 연결·source fingerprint와 필수 topic/calibration/TF/association 검증을 추가한다.
- [x] association 계약 -- version/policy/applied/topic/count/pair_index를 source DB에 결속하고 각 pair의 CDR header stamp를 JSON의 original color/depth timestamp와 대조한다.
- [x] `convert_recording.py` -- depth=`Brown Conrady`, color=`Inverse Brown Conrady` 모델과 중복 없는 보정/TF/JSON 키를 강제한다.
- [x] distortion 계약 -- inverse color 모델을 표준 Brown으로 오용하지 않는다. librealsense와 같은 radial-then-tangential projection으로 rectified-grid→raw-color remap을 만들고, depth 3D point는 같은 rectified pinhole K에 직접 투영한다.
- [x] 출력 CameraInfo/ORB 계약 -- rectified color·depth와 일치하게 `plumb_bob`, D=0, 동일 K/P를 기록하고 ORB YAML도 distortion=0을 사용한다.
- [x] z-buffer/수치 계약 -- 반복 target index에서도 결정적으로 최소 depth를 선택하고 depth sentinel·projection overflow·반올림 후 `65535`를 배제한다.
- [x] `convert_recording.py` -- 선택된 모든 pair의 topic ID, CDR layout, encoding, endian, step, 크기와 단조 증가 host timestamp를 출력 생성 전에 검증한다.
- [x] 경로 격리 -- bag/YAML 출력이 원본 recording directory와 같거나 그 내부면 출력 parent 생성 전에 실패한다.
- [x] 출력 트랜잭션 -- 보수적 디스크 preflight 후 임시 sibling bag에 쓰고 Linux `renameat2(RENAME_NOREPLACE)`로만 승격하며 lock inode 소유권, temp cleanup, file/directory fsync와 publish 결과를 명확히 처리한다.
- [x] publish gate -- 생성 metadata.yaml, DB quick-check, topic type/count, 전체 storage/header timestamp, frame/layout/K/D와 depth sample을 final rename 전에 검증한다.
- [x] publish gate 완결 -- metadata와 DB의 `cdr`·단일 내부 상대 DB 경로를 결속하고, 각 topic×timestamp가 정확히 한 개이며 모든 메시지의 storage/header stamp·frame·layout과 모든 CameraInfo K/D/R/P를 검사한다.
- [x] association/source race -- association을 읽기 전에 DB+JSON fingerprint를 잡고 읽기 직후·publish 직전에 불변을 확인한다. message storage/header timestamp와 ID의 stream 순서를 pair에 결속한다.
- [x] 경합 경계 -- lexical output의 symlink component를 resolve 전에 차단하고 YAML rollback은 설치한 inode만 제거하며 source 불변을 link 전후에 확인한다.
- [x] depth/content 경계 -- raw Z16의 nonzero `65535`는 unit 변환 전에 버리지 않고, mm 변환 결과의 `65535`만 출력 sentinel로 배제한다. black RGB나 개별 empty-depth frame은 구조적으로 유효하게 보존하되 bag 전체에는 유효 depth가 있어야 한다.
- [x] 최종 경합/소유권 -- source fingerprint에 lexical symlink와 ctime을 포함하고 publish 직전·직후 불변을 확인한다. temp/final/lock/YAML inode와 output parent identity가 바뀌면 외부 경로를 삭제하지 않고 실패한다.
- [x] strict schema -- source String/Image topic도 `cdr`를 강제하고 provenance flags, 정확한 20 ms 경계, metadata/DB topic 중복, 생성 DB symlink를 거부한다.
- [x] 수치/settings -- `65534.5 mm`의 bankers rounding 경계를 보존하고 YAML intrinsic 정밀도를 유지한다. ORB가 integer FPS를 읽으므로 `(N-1)/span`을 1–240 범위에서 nearest nominal integer로 기록한다.
- [x] `make_orb_settings_sdk.py` -- 같은 DB/model/read-only/association 결속 규칙, 유효 intrinsics/depth unit/baseline과 `(N-1)/(last-first)` FPS를 검증한다.
- [x] settings 트랜잭션 -- rectified D=0 YAML을 other-readable mode의 임시 파일에서 fsync하고 원자적으로 no-overwrite 설치한 뒤 parent를 fsync한다.
- [x] 산출물 -- 기존 이름의 full bag/YAML은 덮어쓰거나 유효 증빙으로 재사용하지 않고, 세 corrected full bag/YAML을 `{session}_rectified` 이름으로 새로 생성·검증한다.

**Acceptance Criteria:**
- Given 세 원본 bag, when 변환하면, then 각 출력 4개 토픽의 메시지 수가 해당 원본 paired count와 같다.
- Given 첫 출력 메시지, when 역직렬화하면, then RGB는 640×480 `bgr8`, depth는 640×480 `16UC1`, CameraInfo K는 세션 color 보정과 일치하며 네 메시지의 stamp/frame이 같다.
- Given aligned depth, when 샘플 통계를 검사하면, then 유효 픽셀이 존재하고 bag의 extrinsic을 사용한 결과다.
- Given inverse Brown color calibration, when rectification map을 검사하면, then 대표 grid point가 `pyrealsense2.rs2_project_point_to_pixel`과 허용오차 이내이며 표준 Brown 순서로 coefficient를 오용하지 않는다.
- Given 변환 전후, when 원본 파일 상태를 비교하면, then 체크섬과 수정시각이 변하지 않는다.
- Given 모호한 DB 또는 기존 출력, when 실행하면, then 기존 데이터 손상 없이 실패한다.
- Given 잘못된 후반 pair·CDR layout·보정값·timestamp, when 실행하면, then 최종 출력 경로를 생성하지 않고 위반 항목을 보고한다.
- Given 디스크 여유가 보수적 예상치보다 작거나 쓰기 중 실패하면, when 실행하면, then 최종 경로에 부분 bag/YAML을 남기지 않는다.
- Given 유효 depth를 mm로 반올림하면, when 출력하면, then `0`은 invalid로 유지되고 `65535`는 유효값으로 기록되지 않는다.

## Spec Change Log

- **Review loop 1 (2026-07-30):** Blind/Edge 리뷰가 복수 DB 우회,
  후반 오류·디스크 오류의 부분 출력, endian/step 무시, 비유한 보정값,
  잘못된 association/topic/timestamp, depth sentinel 반올림, 임의 FPS 및
  settings overwrite 경합을 확인했다. 위 작업·AC·검증을 명시해 “성공처럼
  보이는 손상 bag/YAML” 상태를 배제한다. **KEEP:** read-only SQLite URI,
  현재 세 0729 세션의 bag 자체 보정·host timestamp 사용, 기존 depth→color
  투영식과 z-buffer, 4개 표준 topic/동일 stamp·frame, 세션별 ORB YAML,
  이미 검증된 full 출력과 원본 파일 불변 상태를 보존한다.
- **Review loop 2 (2026-07-30):** 재리뷰가 distortion model 미검증,
  POSIX `rename()`의 빈 디렉터리 교체 경합, 원본 내부 출력 허용,
  immutable SQLite/WAL 경계와 lock/temp cleanup 증빙 누락을 확인했다.
  모델·경로 격리·`renameat2(RENAME_NOREPLACE)`·cleanup·재현 가능한 integration
  검증을 명시해 “다른 보정 모델로 잘못 정렬”, “경합 중 기존 출력 교체”,
  “원본 트리 변경” 상태를 배제한다. **KEEP:** loop 1의 모든 KEEP, 통과한
  9개 helper 계약, actual late-pair/collision/smoke 결과, 세 full bag의 정확한
  count/timestamp/K/depth 계약, Windows checksum·mtime 원본 보존 증빙을 유지한다.
- **Review loop 3 (2026-07-30):** 재리뷰와 로컬 librealsense/ORB source 대조가
  `Inverse Brown Conrady` 계수를 forward Brown projection과 OpenCV
  `undistortPoints`에 그대로 사용한 기하 오류, zero-D CameraInfo와 distorted
  color의 모순, association provenance/header stamp 미결속, 비결정적 duplicate
  z-buffer와 publish gate 증빙 부족을 확인했다. librealsense와 같은
  radial-then-tangential remap의 color rectification + rectified pinhole
  depth projection + D=0 계약, source-bound
  association, deterministic minimum z-buffer와 강화된 publish 검증을 명시해
  “토픽·count는 맞지만 geometry가 틀린 bag” 상태를 배제한다. **KEEP:** loop
  1·2의 read-only DB, exact-one DB, model/finite/layout 검증, source 외부 출력,
  no-replace publish, timestamp/frame/topic 계약, source checksum·mtime 불변
  증빙은 유지한다. **SUPERSEDE:** “기존 depth→color polynomial과 raw inverse
  coefficient ORB YAML, distorted color + D=0 CameraInfo, 기존 full output을
  유효 결과로 유지”는 이번 근거로 폐기한다. 기존 산출물은 수정하지 않되 새
  `_rectified` 산출물의 정확성 증빙으로 사용하지 않는다.
- **Review loop 4 (2026-07-30):** Blind/Edge 재리뷰가 publish gate가 첫
  메시지와 전역 count만 검사해 topic별 누락/중복 및 후반 header/layout 오류를
  놓칠 수 있고, association read→snapshot 및 YAML install 경합, dangling
  symlink와 rollback inode 소유권, raw/output `65535` 의미를 충분히 구분하지
  못함을 확인했다. 모든 출력 메시지·metadata/DB serialization/path 계약,
  association storage/header/순서 결속, read-before-snapshot 제거, inode 기반
  rollback과 lexical symlink 차단, session-level depth 유효성으로 작업·검증을
  보강해 “검증을 통과한 비동기/교체 산출물” 상태를 배제한다. **KEEP:** loop
  1–3의 exact-one immutable DB, 원본 fingerprint, librealsense와 일치하는
  radial-then-tangential rectification, rectified D=0 K/P, deterministic
  minimum z-buffer, source 외부 transactional no-replace publish, corrected
  세 full output의 geometry/count/hash와 원본 checksum·mtime 증빙을 보존한다.
- **Review loop 5 (2026-07-30):** 재리뷰가 마지막 source-check 이후 publish,
  pathname/symlink 및 temp/final inode 경합, source/output topic 중복·serialization,
  provenance flag와 20 ms 경계, YAML 정밀도·FPS 표현의 빈틈을 확인했다.
  source lexical+target fingerprint와 pre/post publish 검증, parent/temp/final
  identity revalidation, exact schema/provenance/numeric guards를 추가한다. ORB
  소스가 `Camera.fps`를 `int`로 읽는 사실에 맞춰 FPS 공식의 결과를 그대로
  부동소수로 쓰지 않고 1–240 범위의 nearest nominal integer로 기록한다고
  명시해 “정확한 공식이라는 문구 때문에 ORB가 읽지 못하는 YAML” 상태를
  배제한다. **KEEP:** loop 1–4의 geometry, 전수 publish gate, source-bound
  association, corrected 산출물과 모든 원본/출력 checksum 증빙을 유지한다.

## Design Notes

출력 color는 bag의 inverse Brown 모델과 librealsense projection 순서를
이용해 rectified pinhole 영상으로 재표본화하고 depth도 같은 rectified K에
투영한다. 따라서 표준 CameraInfo와
ORB YAML은 모두 D=0을 사용한다. 기존 이름의 full 출력은 보존하지만 corrected
결과로 간주하지 않는다. 데이터 변환 성공은 누락된 runtime 자산·빌드 성공을
의미하지 않는다.

## Verification

**Commands:**
- `python -m py_compile <두 스크립트>` -- expected: 성공.
- `convert_recording.py <src> <temp> --max 2` -- expected: rectified 4개 토픽에 각 2개 메시지.
- 합성 edge fixture -- expected: 복수 DB, wrong-topic ID, malformed/late pair,
  endian/step, invalid calibration/TF/timestamp, output 충돌에서 최종 산출물 없이 실패.
- 통합 edge fixture -- expected: unsupported/missing model, source 내부 output,
  final-publish race, temp/lock 생성·정리 실패를 재현하고 final/source 불변을
  명령·exit status·잔존 경로와 함께 기록.
- inverse-model fixture -- expected: rectification map이 `pyrealsense2` projection과
  일치하고 repeated-index minimum z-buffer와 D=0 color/depth/ORB 계약이
  독립 기준과 일치.
- `ros2 bag info data_slam/0729/converted/<session>` -- expected: 4개 표준 토픽과 paired count×4 메시지.
- 읽기 전용 SQLite quick-check 및 첫 메시지 역직렬화 -- expected: `ok`와 위 계약 일치.
- 원본 파일 SHA-256·mtime(ns)와 Windows source 또는 pre-conversion manifest 비교
  결과를 구현 산출물에 기록 -- expected: 모든 원본 파일 불변.

## Suggested Review Order

**변환 흐름**

- 전체 검증·정렬·transactional publish 순서를 먼저 확인한다.
  [`convert_recording.py:1748`](../../data_slam/260714_frame_data/convert_recording.py#L1748)

**입력 무결성**

- ctime·SHA-256·symlink target을 결합해 source 변경을 탐지한다.
  [`convert_recording.py:76`](../../data_slam/260714_frame_data/convert_recording.py#L76)

- association provenance와 모든 pair를 source CDR에 결속한다.
  [`convert_recording.py:553`](../../data_slam/260714_frame_data/convert_recording.py#L553)

- topic ID·storage/header timestamp·stream 순서를 실제 DB와 대조한다.
  [`convert_recording.py:736`](../../data_slam/260714_frame_data/convert_recording.py#L736)

**RGB-D 기하**

- inverse Brown color를 librealsense 순서로 rectified grid에 매핑한다.
  [`convert_recording.py:826`](../../data_slam/260714_frame_data/convert_recording.py#L826)

**출력 원자성**

- parent fd 기반 no-replace rename으로 경로 교체를 차단한다.
  [`convert_recording.py:938`](../../data_slam/260714_frame_data/convert_recording.py#L938)

- metadata·DB의 전체 메시지와 exact child tree를 publish 전에 검사한다.
  [`convert_recording.py:1408`](../../data_slam/260714_frame_data/convert_recording.py#L1408)

- temp·lock inode 소유권을 보존하며 실패 경로를 정리한다.
  [`convert_recording.py:1025`](../../data_slam/260714_frame_data/convert_recording.py#L1025)

**ORB settings**

- integer FPS와 source calibration 계약을 적용한다.
  [`make_orb_settings_sdk.py:284`](../../data_slam/0724_chungbuk/make_orb_settings_sdk.py#L284)

- parent fd에 고정한 YAML no-overwrite 설치를 수행한다.
  [`make_orb_settings_sdk.py:116`](../../data_slam/0724_chungbuk/make_orb_settings_sdk.py#L116)

**검증**

- 합성 계약 19개와 race·수치 경계 fixture를 검토한다.
  [`test_convert_recording_contract.py:202`](../../data_slam/260714_frame_data/test_convert_recording_contract.py#L202)

- 실제 0729 BAG 기반 통합 검증 6개를 검토한다.
  [`test_convert_recording_contract.py:920`](../../data_slam/260714_frame_data/test_convert_recording_contract.py#L920)

- 산출물 checksum·원본 보존·호환성 경계를 확인한다.
  [`verification-convert-0729-rgbd-bags.md:1`](verification-convert-0729-rgbd-bags.md#L1)
