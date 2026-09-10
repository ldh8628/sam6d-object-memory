# D455f IMU 및 객체 기반 카메라 포즈 보정 실험

2026-09-09. 기본 split 지도/localization 경로와 분리된 실험이다. 현재 두 카메라를 움직일 수 없으므로 IMU 수신·설정·초기화 실패 상태는 검사할 수 있지만 회전 중 정확도 개선은 아직 실측할 수 없다.

## D455f RGBD+IMU 실행

`run_orb_slam_imu.py`는 기존 C++ `IMU_RGBD` 경로를 사용한다. `--backend cuda`는 GPU 실행기의 **ORB descriptor 생성만 CUDA로 실행하는 혼합 경로**다. IMU 적분, 추적, 최적화는 CPU에서 실행된다. 전체 ORB-SLAM3/IMU가 GPU로 옮겨진 것으로 해석하면 안 된다.

SLAM 노트북에서 실행한다. 다른 카메라 실험을 종료한 후 다음 환경을 준비한다.

```bash
cd /home/jucpark/sam6d_object_memory
source /home/jucpark/anaconda3/etc/profile.d/conda.sh
conda activate realsense
source orbslam_ws/install_jazzy/setup.bash
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/integration/fastdds_input.xml"
```

장치별 factory extrinsic을 읽는다. 이 명령은 영상·IMU 스트림을 시작하지 않는다. 출력 파일이 이미 있으면 덮어쓰지 않는다.

```bash
python integration/run_orb_slam_imu.py export \
  --serial 253822302376 \
  --output output/readiness/d455f_imu_calibration_20260909.json
```

이 장치의 factory export는 `output/readiness/d455f_imu_calibration_20260909.json`에 이미 저장되어 있다. 현재 realsense 환경은 Python SDK ABI가 맞지 않아 설치된 native `rs-enumerate-devices -c`로 자동 대체하여 읽는다. 출력 행렬은 native 도구의 약 6자리 출력 정밀도를 따른다.

`T_imu_color`는 color optical 좌표를 IMU gyro 좌표로 변환한다. 서로 다른 두 카메라의 rig extrinsic과는 별개다. accel/gyro 축이 일치하지 않으면 생성기는 중단한다. 노이즈 기본값은 [공식 ORB-SLAM3 D435i 예제](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/RGB-D-Inertial/RealSense_D435i.yaml)에서 가져온 시작값이며, **현재 D455f에서 추정한 노이즈 값이 아니다**. JSON의 `noise`를 Allan variance 등으로 측정한 값으로 교체할 수 있다.

현재 장치의 intrinsics로 만든 **새 지도 생성용 RGBD settings**를 `--settings`로 지정한다. 예전 RGBD-only atlas를 불러오는 settings는 거부한다. IMU 모드는 별도 지도를 만들어 운동으로 초기화해야 한다.

```bash
python integration/run_orb_slam_imu.py run \
  --settings output/readiness/imu_mapping_base_20260909.yaml \
  --calibration output/readiness/d455f_imu_calibration_20260909.json \
  --output output/imu_cuda_trial_01 \
  --backend cuda --seconds 60 --start-camera --sync-mode 1 --domain-id 73
```

`--start-camera`가 없으면 기존 `/camera/slam_camera/rgbd`, `/camera/slam_camera/imu`를 구독한다. 시작 옵션은 기존 공통 카메라 설정에 `enable_gyro:=true enable_accel:=true unite_imu_method:=2`를 추가한다. 출력 폴더는 매 실험 다른 이름을 사용한다. `--prepare-only`는 설정 및 실제 실행 명령만 저장한다. CPU 비교는 새 출력 폴더와 `--backend cpu`를 사용한다. 동일 입력 비교는 영상과 IMU가 함께 포함된 bag을 `--bag /absolute/path`로 넣으며 `--start-camera`와 동시에 사용할 수 없다.

IMU가 이미 발행 중일 때 별도 수신 검사를 할 수 있다.

```bash
python integration/run_orb_slam_imu.py sanity \
  --seconds 15 --output output/imu_sanity_trial_01
```

`imu_sanity.json`에 빈도·최대 간격·시간 역행·gyro 평균/표준편차·가속도 크기를 저장한다. 원시 표본은 `imu_samples.npy`에 남긴다. 이것은 수신 검사이며, 관성 초기화 성공이나 보정 정확도를 뜻하지 않는다. 초기화 성공의 직접 증거가 없으면 `UNKNOWN`으로 해석해야 한다.

ORB 코어는 가속 변화가 부족하면 `not enough acceleration`, 움직임이 부족하면 `Not enough motion for initializing. Reseting...`을 출력한다. 이를 우회하여 정지 실험을 성공으로 만들지 않는다. 회전 실험 전 카메라를 고정 프레임에 장착하고 충분한 병진 가속·여러 축 회전으로 초기화한 뒤, 같은 RGBD+IMU bag에서 CPU/GPU 및 RGBD/IMU를 비교한다. IMU만으로 절대 yaw나 장기 위치 오차가 자동 제거되지는 않는다.

[RealSense ROS IMU 합성 방식](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)은 gyro timestamp에 가속도를 보간한다. D455의 출력은 가속도와 각속도이며 절대 자세 센서로 가정하지 않는다.

## 등록 객체를 이용한 보정 후보

현재 ObjectMemory는 SLAM 카메라 포즈로 객체의 지도 좌표를 만들고, 그 좌표를 카메라 영상에 투영한다. 새 `object_pose_correction.py`는 이 반대 방향의 **진단용 후보**를 계산한다. SLAM 내부 상태나 지도는 변경하지 않는다.

- 저장된 객체 중심 `p_map`과 독립적인 원시 SAM-6D 관측 `p_camera`를 대응시킨다.
- `p_map ≈ R p_camera + t`를 최소 3개의 비공선 객체로 풀고, 최대 512개 삼중점 후보에서 과반수 합의를 선택한 후 불확실성 가중 SVD로 다시 적합한다.
- 지도 ID, 객체 ID 중복, 50ms 이상 시간차, 비유한 값, 공선 배치, 과반수 부재를 거부한다. 기본 보정 한도는 0.5m/20°이며 조절할 수 있다.
- 저장된 anchor를 현재 SLAM pose로 투영한 결과를 다시 관측값으로 넣으면 순환 계산이다. 반드시 `source: raw_sam6d`인 **독립적인 실제 관측**을 사용한다. 같은 클래스의 여러 객체는 고유 인스턴스 ID로 대응해야 한다.

거리만 사용하면 잔차는 `||p_map - t|| - measured_range`다. 이 식에는 회전 `R`이 없어서 **거리만으로 회전은 보정할 수 없다**. 단일 객체는 구면, 두 객체는 일반적으로 원상의 위치 모호성이 있다. 위치도 충분히 분산된 객체 또는 기존 pose prior가 필요하다. 이 프로토타입은 RGBD의 중심 **방향과 거리**를 함께 사용한다. 한 객체의 대칭 회전 문제를 피하려고 객체 회전 자체는 적합에 사용하지 않는다.

합성 예제 실행:

```bash
python integration/object_pose_correction.py \
  output/readiness/object_pose_correction_demo_input.json \
  --output output/readiness/object_pose_correction_demo_result.json
python -m unittest discover -s integration -p test_imu_object_experiments.py -v
```

입력의 `T_map_camera`는 기존 SLAM pose이고, `observations` 각 항목에는 `id`, `map_xyz_m`, `camera_xyz_m`, `stamp_ns`, `sigma_m`가 들어간다. 전체 `stamp_ns`, `map_id`, `observation_map_id`, `source`도 필요하다. 단위는 m/ns다. `sigma_m` 기본값 0.03m는 시험 가중치이며 실측 불확실성으로 교체해야 한다.

합성 검증은 5개 대응 중 큰 오관측 1개를 배제하고 주입한 8° 회전 및 `[0.10,-0.03,0.06]m` 이동을 복원했다. 정확한 합성 좌표에서 잔차가 수치 정밀도로 줄어든 것이며 실제 카메라 정확도 개선값이 아니다. 실사용 전에는 고정 객체 검증, 지도/관측 상관오차, 대칭·동일 클래스 오대응, 시각적으로 잘못된 과반수 합의, 시간 정합을 실제 데이터로 평가해야 한다. 이 단계에서는 후보를 별도 파일로 저장하여 기존 pose와 비교한다.

## IMU 회전 계산 자체를 CUDA로 실행하는 별도 경로

`run_imu_rotation_gpu.py`는 ORB descriptor GPU 경로와 별개로 **gyro bias 제거, IMU→color 축 변환, 회전 quaternion 사전 적분, 선택적인 시각 자세 보정을 실제 PyTorch CUDA tensor로 계산**한다. CPU와 CUDA 결과 및 처리시간을 같은 원시 IMU 표본으로 비교할 수 있다. 저장된 IMU의 오프라인 재생 도구이며, ORB의 C++ 관성 BA를 대체하거나 현재 실시간 pose를 덮어쓰지 않는다.

torch CUDA가 설치된 환경에서 실행한다. SAM 노트북은 `sam6d` 환경을 사용한다. SLAM 노트북의 torch 환경은 GPU 실험 보고서의 설치 확인 결과를 따른다. 먼저 `sanity`에서 생성된 `imu_samples.npy`와 해당 카메라 calibration JSON을 같은 노트북으로 복사한다.

```bash
python integration/run_imu_rotation_gpu.py \
  --samples output/readiness/imu_live_20260909/samples/imu_samples.npy \
  --calibration output/readiness/d455f_imu_calibration_20260909.json \
  --device cuda --compare --output output/imu_rotation_cuda_trial_01
```

출력 `result.json`은 실제 GPU 이름, CPU/CUDA 실행시간, 수치 차이, CPU/CUDA 속도비를 기록한다. 시간에는 host→device 복사가 포함되고 최초 warmup은 제외한다. `cpu_rotation.npz`, `cuda_rotation.npz`에 timestamp와 `gyro_xyzw`를 저장한다. 짧은 200Hz 데이터는 GPU 호출 비용 때문에 CPU보다 느릴 수 있다.

독립적인 시각 자세와 함께 비교하려면 다음을 추가한다.

```bash
  --visual-tum /absolute/path/to/tracking_ok_single_map.tum \
  --map-id trial_map_01 --visual-weight 0.1 --visual-gate-deg 30
```

TUM은 같은 host timestamp의 `T_map_color`여야 하고, 유효한 tracking pose만 포함해야 한다. 모든 시각 timestamp는 입력 IMU 구간 안에 있어야 한다. map reset 전후 궤적을 연결하지 않는다. 시각 포즈 간 IMU 회전 증분으로 자세를 예측한 후 10% 시각 보정을 적용한다. 기본 30° 이상 시각 불일치는 업데이트를 거부한다. 출력의 `fused_xyzw`, `visual_innovation_deg`, `visual_update_accepted`로 후보와 거부 여부를 확인할 수 있다. 시각 보정은 해당 프레임 및 과거 결과만 사용하는 순차 필터이며, 저장된 IMU를 구간 적분하는 오프라인 계산이다.

`--bias WX WY WZ`는 **gyro 좌표계 rad/s** 단위다. 기본 0은 미보정 가정이다. 정지한 구간의 gyro 평균을 bias 시작값으로 사용할 수 있지만, 실제로 회전하는 구간의 평균을 빼면 정상 회전까지 제거한다. 이 도구는 가속도에서 중력을 추정하거나 위치를 이중 적분하지 않는다. 따라서 gyro drift/절대 yaw 문제 또는 회전 중 병진 오차를 해결했다고 주장하지 않는다. extrinsic의 회전은 사용하지만 병진 lever arm을 이용한 가속도 보정은 이 회전 전용 실험에 포함하지 않는다.

CPU 회전 수학 검증:

```bash
python -m unittest discover -s integration -p test_imu_rotation_gpu.py -v
```

90° 회전, 카메라/IMU 축 변환, 알려진 gyro bias 제거, 큰 시각 포즈 점프 거부를 검증한다. torch 없는 환경에서는 이 테스트를 건너뛰므로 실제 검증은 torch 환경에서 실행한다.


## 2026-09-09 실제 실행 결과

SLAM 노트북의 D455F(253822302376, BMI085, firmware5.17.3.10)에서 factory extrinsic export, IMU15초 수신, fullRGBD+IMU+CUDA descriptor 실행, torchCUDA gyro 적분과 합성 시각 보정을 실제로 실행했다. 카메라는 모두 정상 종료했다.

| 검증 | 결과 |
|---|---|
| IMU15초 수신 | PASS,2983표본,약200.215Hz,최대간격6.029ms,시간역행0 |
| 실제 IMU CPU/CUDA 적분 | PASS,최대수치차2.22e-16; CPU1.943ms/CUDA0.933ms,배치2.08배 |
| 합성 시각 보정 CPU/CUDA | PASS,최대수치차7.63e-14; CPU24.532ms/CUDA78.939ms,CUDA가약3.22배느림 |
| fullORB RGBD+IMU+CUDA20초 | 631프레임입력/소비,입력오류0,CUDA실행확인;0pose/NOT_TRACKING |
| fullORB 초기화 | 정지상태에서 `not enough acceleration`629회. 관성초기화·회전정확도검증미완료 |
| 객체포즈합성보정 | 5개중오관측1개배제,주입한8° 및[0.10,-0.03,0.06]m복원 |

합성 시각 보정의 첫 실행은 영각도 부근 `acos` 수치오차로 CPU/CUDA 오차 비교에 실패했다. quaternion의 `atan2` 각도로 계산을 수정한 뒤 재실행하여 통과했다. 동일한 테스트에서 수치오차를 숨기기 위해 허용값을 늘리지는 않았다.

현재 준비된 파일로 바로 재실행할 명령은 위 fullORB 예제의 실제 settings와 아래 명령이다. torch 환경은 SLAM 노트북의 `/home/jucpark/anaconda3/envs/sam6d`이다.

```bash
cd /home/jucpark/sam6d_object_memory
source /home/jucpark/anaconda3/etc/profile.d/conda.sh
conda activate sam6d
python integration/run_imu_rotation_gpu.py \
  --samples output/readiness/imu_live_20260909/samples/imu_samples.npy \
  --calibration output/readiness/d455f_imu_calibration_20260909.json \
  --device cuda --compare --output output/imu_rotation_user_trial_01

python integration/run_imu_rotation_gpu.py \
  --samples output/readiness/imu_rotation_synthetic/imu_samples.npy \
  --calibration output/readiness/imu_rotation_synthetic/calibration.json \
  --visual-tum output/readiness/imu_rotation_synthetic/visual.tum \
  --map-id synthetic_constant_turn --device cuda --compare \
  --output output/imu_visual_user_trial_01
```

객체 보정 예제는 SAM 노트북에서 실행한다.

```bash
cd /home/etri/sam6d_object_memory
python3 integration/object_pose_correction.py \
  output/readiness/object_pose_correction_demo_input.json \
  --output output/readiness/object_pose_correction_user_result.json
```

이동 IMU 기록도 검색했다. SAM 노트북의94개metadata중9개는 외장Xsens IMU뿐이며 D455 IMU 기록은 없었다. SLAM 노트북의5개metadata에는IMU가없었다. `sam6d_ws/data/longcircle2`는72.203초 RGBD+Xsens 기록을 포함하지만 카메라↔Xsens extrinsic이 없고, 기존 결과문서도 이 항목을 미확보로 기록한다. 현재D455의 내부IMU factory extrinsic을 Xsens에 잘못 적용해 성공을 만들지 않았다. 이동한D455 RGBD+IMU 기록 또는 실제 카메라 이동 후에 full관성초기화를 검증해야 한다.


### 실제 저장 관측의 객체 보정 실행

```bash
python3 integration/object_pose_correction.py \
  output/readiness/object_correction_recorded_20260909/input.json \
  --output output/readiness/object_correction_recorded_20260909/result.json
```

`260904_test_02`의 원시 `pose_source=sam6d` 관측 중 verify accepted, 같은 timestamp에서3개 이상 대응,100ms 이내 기존 SLAM pose 보간 조건을 만족한4프레임을 모두 검사했다. 모두 진단 후보가 산출됐고 첫 시간순 후보의 변화는6.53cm/1.25도, 적합 잔차71.25→13.34mm였다. 최종 지도와 prior가 같은 녹화에서 유래하므로 상관오차가 있다. 이는 실제 저장 관측의 실행 검사이며 독립 위치 정확도 개선 증명이 아니다. 모든 프레임 결과와 재현 입력 생성 스크립트는 `output/readiness/object_correction_recorded_20260909/`에 있다. SLAM/객체 지도는 변경하지 않았다.


## GPU/IMU 실행 경계 추가 검증 (2026-09-09)

IMU live 실행은 현재 CameraInfo K/D를 공통 helper로 반영하고 `calibration_delta.json`을 남긴다. 기존 ORB가 같은 domain에서 실행 중이면 카메라를 열기 전에 거부한다. Startup sanity는3초 전체 coverage, 최초/마지막 지연, 수신·센서 간격과200Hz 기준수신율을 검사한다. 실제 재시험은580샘플,coverage96.37%,수신192.98Hz로PASS였다. IMU stream을 먼저 정지한 뒤 카메라 node를 종료하며 `camera_shutdown.json`에 결과를 보존한다. 정지/어두운 입력의 inertial initialization은 여전히UNKNOWN이며 pose0을 성공으로 보고하지 않는다.

상대Atlas경로 수정의 실제 CUDA localization(827/828pose) 및 IMU 전체실행 명령·실패수정·재시험 근거는 `output/gpu_relative_atlas_20260909/review_fix_report.md`에 있다. 위 객체보정 실측 자료는 그대로 보존했다.
