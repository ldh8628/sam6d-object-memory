# 0729 RGB-D 변환 검증 기록

- 검증일: 2026-07-30
- 원본: `data_slam/0729/0729_*`
- Windows 대조 원본: `/media/etri/04D4788ED4788428/ros2_bag/0729_*` (read-only NTFS)
- corrected 변환 출력: `data_slam/0729/converted/0729_*_rectified`
- corrected ORB 설정: `data_slam/0729/settings/orb_0729_*_rectified.yaml`

## 결과

- Python compile: 변환기, settings 생성기, 계약 테스트 모두 성공(exit 0).
- 핵심 계약 단위 테스트: 19/19 성공(exit 0).
- 실데이터 통합 테스트: 6/6 성공(exit 0). corrected full output, settings
  생성 smoke·충돌 불변, 2-frame rectified smoke, 실제 DB header timestamp
  변조, 원본 내부 출력 차단과 생성 metadata/DB의 중복 topic 훼손 거부를
  포함한다.
- 기존 bag/YAML 충돌: 둘 다 exit 1로 실패하고 inode·해시 불변.
- 2-frame smoke: SQLite `quick_check=ok`, 4 topics × 2, 동일 stamp/frame,
  rectified RGB `640x480 bgr8`, aligned depth `640x480 16UC1`, K/D=0 일치,
  depth `65535` 없음.
- inverse Brown rectification의 대표 grid point는
  `pyrealsense2.rs2_project_point_to_pixel`과 `2e-4 px` 이내로 일치하고,
  표준 Brown coefficient 순서와는 다른 결과임을 별도 계약으로 확인했다.
- 세 corrected full bag은 SQLite `quick_check=ok`이고 metadata/DB의 단일
  내부 상대 경로와 `cdr`가 일치한다. 모든 topic×timestamp에 메시지가 정확히
  하나씩 있으며 모든 storage/header timestamp, frame, image layout과 모든
  CameraInfo K/D/R/P를 전수 검사했다. ORB YAML도 rectified 계약에 맞게 D=0이다.
- metadata는 duplicate mapping key, 비정수 count/time, symlink/FIFO/device,
  extra file을 거부한다. publish gate 이후에는 metadata/DB의 inode·size·
  mtime·ctime·link count를 final rename 전후에 다시 비교한다.
- source snapshot은 lexical symlink·target identity·size·mtime·ctime·SHA-256을
  결합하고 해시 전후 descriptor/path 상태도 비교한다. 같은 크기로 재기록한 뒤
  mtime을 복구하거나 symlink를 retarget하는 fixture도 거부했다.
- publish는 parent directory fd에 고정한 `renameat2(RENAME_NOREPLACE)`를
  사용한다. 계약 테스트는 기존 출력 no-replace, lock/temp/YAML inode 교체
  보존과 부모 `fsync` 실패의 명시적 경고를 확인했고, 실제 smoke는 final
  directory 및 두 child inode 계약을 통과했다.
- lock/temp/실패 final 잔여물은 없다.
- Loop 5 Blind/Edge 독립 리뷰의 실제 finding은 patch로 처리했다. numeric
  bool provenance, source DB 재모호화, settings의 depth model/extrinsic 누락,
  YAML 성공 직전 source/parent/final identity, child bag 교체 경로를 추가로
  닫았으며 재검증 결과는 19/19와 6/6이다.

## Review loop 5 재현 명령

아래 명령은 `/home/etri/slam-perception-stack`에서 실행했고 모두 exit 0이다.

```bash
/home/etri/miniconda3/envs/sam6d_ros_humble/bin/python -m py_compile \
  data_slam/260714_frame_data/convert_recording.py \
  data_slam/0724_chungbuk/make_orb_settings_sdk.py \
  data_slam/260714_frame_data/test_convert_recording_contract.py

/home/etri/miniconda3/envs/sam6d_ros_humble/bin/python -m unittest -v \
  data_slam.260714_frame_data.test_convert_recording_contract.ConverterContractTests

RUN_0729_INTEGRATION=1 /home/etri/.local/bin/micromamba run \
  -r /home/etri/miniconda3 -n sam6d_ros_humble python -m unittest -v \
  data_slam.260714_frame_data.test_convert_recording_contract.ActualBagIntegrationTests

git diff --check -- \
  data_slam/260714_frame_data/convert_recording.py \
  data_slam/0724_chungbuk/make_orb_settings_sdk.py

git diff --no-index --check /dev/null \
  data_slam/260714_frame_data/test_convert_recording_contract.py
```

통합 테스트는 `TemporaryDirectory` 안에서 실제 0729 source DB를 읽고 정상
2-frame rectified 출력과 source header timestamp 변조 실패를 재현한다.
full-output 검사는 세 세션의 `quick_check`, metadata/DB path와 `cdr`, 4개
topic count, 모든 topic별 storage/header timestamp, 모든 RGB/depth의
frame·layout, 모든 CameraInfo K/D/R/P, 모든 depth의 `65535` 배제와 bag
전체의 nonzero depth 존재를 검사한다. smoke 산출물의 metadata와 DB topic
table을 각각 중복 topic으로 훼손한 뒤 publish gate가 이를 거부하는 것도
확인했다. ORB YAML 생성기는 source의 `(N-1)/span` FPS가 1–240 범위인지
검증하고 ORB-SLAM3의 integer reader에 맞춰 nearest nominal integer를 쓰며,
intrinsic은 17자리 유효숫자로 직렬화한다.

| Session | Count/topic | Corrected DB bytes | Depth sample nonzero/max |
|---|---:|---:|---|
| `0729_long_circle_inner` | 3391 | 5,218,131,968 | 262051/62593, 245013/62593, 262245/62591 |
| `0729_smaill_circle_inner` | 2001 | 3,079,188,480 | 258434/62589, 255930/62587, 262963/62585 |
| `0729_small_8` | 4365 | 6,716,936,192 | 265956/15445, 251022/21590, 265635/62591 |

모든 depth sample에서 유효 픽셀이 존재하고 `65535`는 없었다. 각 full
bag의 첫 CameraInfo K는 해당 세션 source color calibration과 정확히
일치한다. 세 세션 모두 corrected 첫 RGB/depth payload가 legacy 산출물과
다르므로 legacy bag은 이번 geometry 정확성 증빙으로 사용하지 않는다.

## Corrected 산출물 체크섬

| File | SHA-256 |
|---|---|
| `converted/0729_long_circle_inner_rectified/bag_0.db3` | `d094940272bbc46ec74993660dddd660b359ab806ec4c95db4059f939cdccb59` |
| `converted/0729_smaill_circle_inner_rectified/bag_0.db3` | `b6f8614a65d95241d62c2242a57dcca5765ad444ac69df8bcb99065d31162b6f` |
| `converted/0729_small_8_rectified/bag_0.db3` | `b05454af76efad2ea87dae8adffdb43b4cac1f0b26f1b99de140c492afae38dd` |
| `converted/0729_long_circle_inner_rectified/metadata.yaml` | `e7572000360f23e5296decd94159c4d7a9ed7bcfba370e4c91651c47b7e0068e` |
| `converted/0729_smaill_circle_inner_rectified/metadata.yaml` | `a9ab40f47f31ba7d41b64dd2b2b5be207e681efe6fd4f32cfe8e976acebcd09c` |
| `converted/0729_small_8_rectified/metadata.yaml` | `8904ff341668daa7f5565c516a699242ce302acaebcb3934eee733b11483dfb1` |
| `settings/orb_0729_long_circle_inner_rectified.yaml` | `eadc0b1b07d40a8bb03137315c8eec0135a0b17dcebe296bd6dc67b2be042f9f` |
| `settings/orb_0729_smaill_circle_inner_rectified.yaml` | `33361fb5c3c7bde09bc1e68f4b21621c1d0a462a1ca653ab88a8592dc43fd5ab` |
| `settings/orb_0729_small_8_rectified.yaml` | `d09774a63bcc2ba5b4eb0e3ca9bc450281e17ac514bc692e7eef1b8df247e19e` |

세 YAML은 mode `0644`, FPS `30`, distortion coefficient 0이다.

## 원본 보존

Windows 원본→Linux 원본 방향 `rsync -rnc`는 세 디렉터리 모두 차이 0개였다.
각 디렉터리 5개 파일의 size와 `mtime_ns`도 Windows 원본과 정확히 같았다.
Review loop 5 종료 시 세 원본 디렉터리 15개 파일의 SHA-256을 다시 계산했으며
아래 기록과 모두 일치했다.

| File | SHA-256 | Bytes | mtime (epoch seconds, ns) |
|---|---|---:|---:|
| `0729_long_circle_inner/metadata.yaml` | `a4e7f8cf2e8be24781f384e064f66cf8fc6dcff637b69067199aa382402b24e8` | 24778 | 1785283861.4795278000 |
| `0729_long_circle_inner/recording_20260729_090903.db3` | `f22cda02295256aad7316d19b0169c6aa51e98a256dbb6fccdf9819ccf3407e6` | 5219958784 | 1785283861.3678405000 |
| `0729_long_circle_inner/recording_summary.json` | `92584d9a9df7c82506d6de43dfcc1661c6a251bba6cb16e6c042fb6a61621b6d` | 1710 | 1785283861.6020379000 |
| `0729_long_circle_inner/rgbd_timestamp_associations.json` | `a5a88ed040a858d44d2d7b024419e0e8f6c6d77ce43bdbd6eca423709e3cab09` | 1252717 | 1785283861.4566731000 |
| `0729_long_circle_inner/session_manifest.json` | `e79d782c00a366311b2c004df5c8c868aa281b2f47b069e62f0b861ee1cc1cfb` | 675 | 1785283861.5853982000 |
| `0729_smaill_circle_inner/metadata.yaml` | `c8a354fb5b5a460e1b19773f18e083bb5a50b55bcea901b48fc2c7d11e1e5e72` | 24774 | 1785283062.8321828000 |
| `0729_smaill_circle_inner/recording_20260729_085631.db3` | `af5dc5d9a838eba916ec32f57a169681691390d2eec7bda74cb6d190687807c8` | 3080265728 | 1785283062.7372155000 |
| `0729_smaill_circle_inner/recording_summary.json` | `363e0f8756a295c078a688598cbd41e18098ce16725dec46250df8b0218337e3` | 1702 | 1785283062.8796956000 |
| `0729_smaill_circle_inner/rgbd_timestamp_associations.json` | `cad7355ae4ce65c87ca38c358bc915007350c04b17560ffb09382496b84cc6bb` | 736795 | 1785283062.8164298000 |
| `0729_smaill_circle_inner/session_manifest.json` | `61a849d3f90f08fcab31e945524161ecf2b833d578c316b7b4039136b3fd7a7b` | 669 | 1785283062.8478956000 |
| `0729_small_8/metadata.yaml` | `bdb41eff5c430362fd0b12f9f6f8a0728fc4404aaaf1ed0412376d53ed94eb71` | 24778 | 1785283306.3196509000 |
| `0729_small_8/recording_20260729_085915.db3` | `0b39e7d249ea00641941e2a94a84c17cf95a0255f53a9082544af46be2b1b5b0` | 6719283200 | 1785283306.1715410000 |
| `0729_small_8/recording_summary.json` | `a1444a40f6369c8750a6085d562f5965de2cc565e37c308fac1a5a0f42b8815d` | 1722 | 1785283734.6858959000 |
| `0729_small_8/rgbd_timestamp_associations.json` | `0969dea1d39fc55bba0d23917d834da56e272b5d650bac4f3d640f4e8b2a5bbc` | 1616993 | 1785283306.3037202000 |
| `0729_small_8/session_manifest.json` | `2010dc0a56085c24e567de2172bae718812725fd9fb080dedc57b16bba19908e` | 641 | 1785283734.6542971000 |

## 범위 경계

이 검증은 표준 RGB-D 입력 계약과 변환 무결성을 확인한다. ORB vocabulary,
SAM-6D weights/CAD/templates, workspace build 및 실제 SLAM/SAM-6D 추론 성공은
별도 런타임 검증 대상이다. LiDAR가 없으므로 HDL-Graph-SLAM 입력은 생성하지 않았다.
