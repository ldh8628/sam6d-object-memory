# converting_launch — ros2 bag 변환 툴체인 (정본)

SDK 원본(`data_slam/`)을 알고리즘 입력용 표준 bag(`data_slam_converted/`)으로 만드는
스크립트는 **전부 여기에만** 있다. 다른 폴더의 같은 이름 파일은 이곳을 가리키는 심볼릭 링크다.

## 한 줄 사용법

```bash
python3 converting_launch/convert_all.py \
    --inputs:=data_slam/260807_chungbuk \
    --outputs:=data_slam_converted/260807_chungbuk
```

`--inputs:=` 바로 아래에 `SAM/`·`SLAM/` 이 있으면 **단일 세션**, 없으면 그 하위를 **세션으로 순회**한다.

| 인자 | 뜻 |
|---|---|
| `--inputs:=` | SDK 원본. 날짜 폴더 또는 단일 세션 폴더 |
| `--outputs:=` | 변환본을 쓸 곳 |
| `--session:=a,b` | 날짜 폴더를 줬을 때 일부 세션만 |
| `--stages:=clock,convert,lidar,imu` | 기본은 전부 |
| `--clock-offset-ns:=<ns>` | 두 호스트 시계차를 직접 지정 (`t_common = t_SAM - 이 값`). 실측·표를 모두 제친다 |
| `--stride:=N` | N 프레임마다 하나. 기본 1 = 전부. **희소본은 만들지 않는 것이 원칙**이고, 이 값은 스모크 테스트용이다 |
| `--force` / `--dry-run` | |

## 이 한 파일이 하는 네 가지

| 단계 | 하는 일 | 쓰는 도구 |
|---|---|---|
| 1 `clock` | 두 호스트 시계차 산출 | `calib_offset.py` → manifest → `known_clock_offsets.json` → 0 |
| 2 `convert` | depth 정렬 + 4토픽 표준 bag, **시계차를 SAM 에 굽는다** | `convert_recording.py --offset-ns` |
| 3 `lidar` | Velodyne rosbag2 metadata v9 → v5 + payload 복사 | `downgrade_bag_metadata.py` |
| 4 `imu` | Xsens bag 복사 (이미 v5) | 내부 `copy_bag()` |

### 시계차 산출 우선순위

`offset_ns` 는 **SAM 타임스탬프에서 빼는** 값이다: `t_common = t_SAM - offset_ns`.

| 순위 | 방법 | 조건 | 정확도 |
|---|---|---|---|
| 0 | `--clock-offset-ns:=` 직접 지정 | 사람이 준다 | — |
| 1 | genlock Global Time 실측 (`calib_offset.py`) | NTP probe 사이드카 + `Color_0/image/metadata` | ms 이하 |
| 2 | 레코더 manifest 의 예정 시작시각 차 | `session_manifest.json` 등 | ~0.1 s |
| 3 | `known_clock_offsets.json` (사람이 미리 재서 넣은 표) | 표에 있으면 | 넣은 값에 따름 |
| 4 | `0` (`unmeasured`) | 그 외 | — |

**값이 0 이면 무보정과 같으므로 분기가 없다.** 어떤 방법을 썼는지는 `info.json.clock.method` 에 남는다.

⚠ 1순위는 **NTP probe 사이드카가 있을 때만** 동작한다. probe 없이 프레임 최근접 매칭만 쓰면
실제 오프셋이 6 초든 6 밀리초든 **언제나 ±16.7 ms 안의 그럴듯한 거짓값**이 나온다
(260804 에서 실제 −6.607 s 를 −6.36 ms 로 오판한 적이 있다). 그래서 probe 가 없으면 실패로 돌린다.

## 출력 규격

```
<outputs>/<세션>/
    SAM/    metadata.yaml + SAM_0.db3            ← rosbag2 가 요구하는 2개뿐
    SLAM/   metadata.yaml + SLAM_0.db3
    lidar/  metadata.yaml(v5) + *.db3               (Velodyne 이 있을 때만)
    imu/    metadata.yaml     + *.db3               (Xsens 가 있을 때만)
    info.json                                           ← 메타데이터는 전부 여기 하나
```

변환기가 bag 안에 남기는 `conversion_info.json` 은 `info.json` 으로 흡수한 뒤 지운다.
로그 같은 중간 산출물은 출력 폴더가 아니라 `integration/output/<날짜>/<세션>/logs_convert/` 에 둔다.
**출력 폴더에는 rosbag2 가 요구하는 파일과 `info.json` 하나뿐이다.**

**변환본은 원본을 안 보고도 홀로 선다.** 2026-08-25 이전에는 lidar/imu 의 `.db3` 가 원본을
가리키는 심볼릭이었으나(재인코딩이 0이라 복제를 아꼈다), `data_slam_converted/` 만 들고
다녀도 되게 실데이터 복사로 바꿨다. payload 는 여전히 재인코딩하지 않으므로 바이트가
원본과 같고, 값은 디스크뿐이다 — 30세션 전량이면 lidar+imu 로 **약 45.6 GB** 를 더 쓴다.

## 검사

```bash
python3 converting_launch/verify_layout.py \
    --inputs:=data_slam_converted \
    --against:=converting_launch/baseline_before.json
```

A 구성 / B 토픽 4개 / C rosbag2 v5 / D 자립(심볼릭 0 · payload 크기 일치) / E 시계 적용 /
F 옛 변환본 대조 / G 잔여 bag 을 본다.

## 파일

| 파일 | 역할 |
|---|---|
| `convert_all.py` | **유일한 진입점.** 위 5단계 |
| `convert_recording.py` | SDK(`/device_0/...`, depth 미정렬) → 4토픽 표준 bag. depth 정렬·왜곡보정 내장 |
| `calib_offset.py` | genlock Global Time 으로 두 호스트 시계차 실측. stdout 에 ns 정수 하나 |
| `downgrade_bag_metadata.py` | Jazzy(v9) metadata 를 Humble(v5) 로. `.db3` 는 손대지 않는다 |
| `known_clock_offsets.json` | genlock probe 도 manifest 도 없는 데이터셋의 시계차를 사람이 넣어 두는 표 |
| `verify_layout.py` | 변환본 트리 규격 검사 |
| `baseline_before.json` | 재구성 **전** 변환본의 프레임 수·길이·K (회귀 검증 기준값) |
| `clock_offsets.json` | `calib_offset.py` 결과 캐시 (자동 생성) |

## 알아둘 것

- `convert_recording.py` 는 원본의 `rgbd_timestamp_associations.json` 을 **반드시** 읽는다.
  이 사이드카는 레코더가 촬영 당시 만든 원본의 일부이며, 없으면 변환 자체가 불가능하다.
- `--time-source` 는 `color_global`(카메라 Global Time, 정확) 을 먼저 쓰고, 그 metadata 토픽이
  없는 데이터셋에서는 `assoc` 으로 자동 폴백한다. 쓴 값은 `info.json` 에 남는다.
- Xsens bag 에는 `-wal`/`-shm` 이 딸려 있다. sqlite WAL 이 체크포인트되지 않은 채 끝났으면
  레코드가 `-wal` 에 남아 **`.db3` 만 가져오면 데이터가 빈다.** 다만 실측해 보면 현재 원본
  22개는 **전부 0바이트**(=체크포인트 완료)라 `.db3` 하나로 충분하고, 변환본은 `-wal`/`-shm`
  을 만들지 않는다. `copy_bag()` 은 `-wal` 이 0바이트가 아니면 조용히 잃는 대신 **멈춘다**.
- `convert_recording.py` 는 `rosbag2_py` 가 필요해 conda 환경 `sam6d_ros_humble` 에서 돈다.
  `convert_all.py` 는 numpy/yaml 이 없으면 `sam_yolo` 인터프리터로 자기 자신을 다시 실행한다.
