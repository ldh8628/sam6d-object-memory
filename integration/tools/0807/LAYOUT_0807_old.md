# 0807 충북 데이터셋 — 폴더 구조와 세션 이름

2026-08-10 정리. SLAM 쪽 세션명(주행형태 라벨)을 SAM 쪽에도 적용해 **두 폴더의 이름이 같으면 같은 촬영**이 되도록 맞췄다.

```
data_slam/260807_chungbuk/
├── SLAM/260807/<세션>/          Husky A200 탑재 · Linux · 호스트 cpr-a200-0881
│     recording_<HHMMSS>.db3        RealSense D455F #253822302376 (SDK 포맷, depth 미정렬)
│     cpr-a200-0881_velodyne_husky_*/  Velodyne VLP-16 10 Hz + /a200_0881/platform/odom·joint_states
│     xsens/xsens_*.db3             Xsens MTi 200 Hz
│     rgbd_timestamp_associations.json   color↔depth 페어링 + host epoch ns
│     peer_timestamp_comparison.json     ★ 두 호스트 시계 오프셋 (융합 시 필수)
│     husky/husky_timestamp_sync.json, xsens/xsens_timestamp_sync.json
├── SAM/260807/<세션>/           휴대 노트북 · Windows · 호스트 BOOK-QN90665CQN
│     recording_<HHMMSS>.db3        RealSense D455F #408222300890 (RGB-D 단독, IMU 미기록)
│     rgbd_timestamp_associations.json, recording_summary.json, session_manifest.json
├── SAM_260807_VALIDATION.md     SAM 쪽 전수 검증 보고서
└── figures/                     세션별 추정 궤적
```

두 카메라는 **서로 다른 물리 장비**이며 9핀 케이블로 **하드웨어 동기(master/slave)** 되어 있다.
- SLAM(husky) = `Inter_Cam_Sync_Mode 1 (MASTER)`
- SAM(laptop) = `Inter_Cam_Sync_Mode 3 (FULL SLAVE)`

## 세션 목록

이름 = `recording_20260807_<HHMMSS>__<주행형태>`. HHMMSS는 **SLAM 쪽 세션 생성 시각**이며 실제 촬영 시작보다 약 20 초 이르다.

| 세션 | 촬영 시작(KST) | 길이 | SAM | SLAM | 비고 |
|---|---|---|---|---|---|
| `165039__double_loop_cw` | 16:50:59 | 136 s | ○ | ○ | |
| `170521__outer_loop_inner_loop_mixed` | 17:05:41 | 149 s | ○ | ○ | |
| `170817__rectangular_perimeter_cw_open` | 17:08:37 | 151 s | ○ | ○ | |
| `171116__figure_eight_mixed_turns` | 17:11:36 | 73 s | ○ | ○ | |
| `181715__u_turn_cw` | 18:17:35 | 76 s | ○ | ○ | SAM depth 50프레임 부족 |
| `182155__short_s_curve` | 18:22:15 | 82 s | ○ | ○ | SAM depth 92프레임 부족, 검은 RGB 9장 |
| `182349__inplace_rotation_ccw_1turn` | 18:24:09 | 162 s | ○ | ○ | **SAM 40~140 s 렌즈 차폐 — 사용 주의** |
| `182735__u_turn_cw` | 18:27:55 | 65 s | ○ | ○ | |
| `182943__large_loop_then_small_loop_cw` | 18:30:03 | 125 s | ○ | ○ | |
| `183504__straight_then_circle_ccw` | 18:35:24 | 125 s | ○ | ○ | |
| `183839__irregular_multi_loop_ccw_2turns` | 18:39:00 | 131 s | ○ | ○ | SAM 원래 폴더는 `183840` |
| `184128__irregular_multi_loop_ccw_8turns` | 18:41:48 | 271 s | ○ | ○ | |
| `185223__circle_ccw_8laps` | 18:52:43 | 295 s | ○ | ○ | |
| `185901__multi_loop_figure_eight` | 18:59:21 | 336 s | ○ | ○ | |
| `190624__irregular_multi_loop` | 19:06:44 | 317 s | ○ | ○ | **SAM은 19:10:12에 스트림 중단** (뒤 109 s 없음) |

### SAM 단독 (짝 없음) — 2026-08-25 삭제됨

노트북에서만 시작된 테스트/실패 촬영 4개를 삭제했다(1.92 GiB 회수). 이제 **SAM 15 = SLAM 15**로 폴더 수가 일치한다.

| 삭제한 폴더 | 사유 |
|---|---|
| `recording_20260807_164829` | db3 없음 — 빈 세션 |
| `recording_20260807_180433` | 1.8 s 테스트 샷 |
| `recording_20260807_181902` | 24 s, 정지 후 사람이 렌즈 차폐 |
| `recording_20260807_184931` | 19 s, 벽 근접 정지 |

⚠ `183839` 세션의 SAM 폴더 안 파일명은 `recording_20260807_183840.db3` 그대로다. 두 호스트가 1 초 어긋나게 세션을 만들었기 때문이며, 폴더명만 SLAM 쪽에 맞췄다.

## 두 카메라를 함께 쓸 때 반드시 알아야 할 것

### 1. Depth는 하드웨어 동기, Color는 아니다

genlock은 **depth(스테레오) 스트림만** 트리거한다. 실측 위상 원형표준편차:

| | DEPTH 교차카메라 | COLOR 교차카메라 |
|---|---|---|
| 165039 ~ 182155 (6세션) | **0.80 ~ 1.18 ms** (R 0.98) | **10.8 ~ 13.5 ms** (R 0.04~0.13) |
| 182735 ~ 190624 (8세션) | 0.74 ~ 4.43 ms | 0.74 ~ 4.43 ms (depth와 동일) |

- 완전 비동기의 이론값은 9.62 ms · R→0 이다. 초기 6세션의 **COLOR는 사실상 무작위 위상**이므로, 두 카메라의 RGB를 같은 시점으로 보고 융합하면 최대 ±16.7 ms 어긋난다.
- 18:24:09 세션(`182349`)의 t≈60 s부터 SAM 카메라의 color가 depth에 묶이면서, 이후 세션은 **RGB까지 교차 동기**된다. `182735` 이후 8세션이 RGB 융합에 유리하다.
- 프레임카운터 증가량은 11개 세션에서 **±1 이내로 완전 lockstep**(최대 10,087프레임 구간). 예외는 `181715`(−50), `182155`(−92), `182349`(−27), `190624`(SAM 조기 중단).

### 2. 두 호스트의 시계는 다르다 — 타임스탬프를 그대로 비교하면 안 된다

각 bag의 타임스탬프는 **자기 호스트의 epoch**다. 오프셋(SAM − SLAM)은 하루 동안 **+7.25 ms → +63.93 ms**로 단조 증가한다(≈6.9 ppm).

세션마다 `SLAM/260807/<세션>/peer_timestamp_comparison.json` 의
`clock_mapping.remote_minus_local_clock_ms` 값을 SAM 타임스탬프에서 **빼야** 공통 시간축이 된다.

```
t_common = t_SAM - remote_minus_local_clock_ms
```

genlock이 보장하는 것은 **두 카메라 노출 시점이 물리적으로 같다**는 것이지, 기록된 숫자가 같다는 뜻이 아니다.

### 3. 한 카메라 안에서 color와 depth는 최대 ±16.7 ms 어긋난다

두 카메라 모두 color가 depth보다 **963.4 ppm** 빠르다(SAM은 18:24 이후). 약 35 초마다 한 프레임 주기만큼 밀렸다가 인접 프레임으로 재매칭되는 톱니가 반복된다. 6D pose처럼 정밀 정합이 필요하면 `rgbd_timestamp_associations.json`의 프레임별 Δt를 함께 봐야 한다.

### 4. depth는 color에 정렬되어 있지 않다

color↔depth extrinsic translation = (−0.059, 0.0002, 0.0003) m. SDK 포맷이므로 `convert_recording.py` 정렬 경로를 거칠 것.

## 다음 액션 제안

- **A. 표준 bag 변환** — 유효 세션을 `convert_recording.py`로 depth 정렬 + epoch 스탬프 표준 bag으로 만들고, 그 시점에 §2의 시계 보정을 함께 적용한다.
- **B. LiDAR 준-GT 생성** — hdl_graph_slam으로 15세션 Velodyne 기준궤적을 만든다. 0724와 달리 Velodyne이 이미 10 Hz full sweep이라 병합이 필요 없다.
- **C. 두 카메라 extrinsic 캘리브** — 260804에서 미보정 때문에 객체맵이 흩어진 전례가 있다. genlock으로 시간은 맞았으니 이제 공간 정합만 남았다.
- **D. RGB 융합 세션 선별** — RGB 교차 동기가 필요한 작업은 `182735` 이후 8세션으로 한정한다.

**질문**: 다음은 어느 것부터 진행할까요? 그리고 이 데이터셋의 주 용도가 **LiDAR 기준 SLAM 정확도 평가**인지, **두 카메라 융합 객체맵**인지 알려주시면 §1의 세션 선별 기준을 그에 맞춰 좁혀 드리겠습니다.
