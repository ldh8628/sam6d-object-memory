# 회전·실시간 처리 조사와 적용 판단

조사일: 2026-09-07~08. 이 문서의 개선 설명은 저장소 코드 분석에 근거한 판단이며, 실제 개선 수치는 `VALIDATION.md`의 비교 시험으로 구분한다.

## 같은 프레임에서 지도 매칭 복구

[ORB-SLAM3 공식 Tracking.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc)의 매핑 경로는 `TrackWithMotionModel()`이 실패하면 `TrackReferenceKeyFrame()`을 호출한다. 이 저장소의 localization 경로에는 그 즉시 재시도가 없었다. 회전 가속·급격한 방향 변화로 일정 속도 예측의 투영 검색이 빗나갈 때, 다음 프레임의 재localization까지 기다리는 대신 지도 키프레임의 BoW 매칭으로 복구를 시도하는 것은 코드에서 도출한 개선 후보이다.

`Tracking.RotationFallback=1`일 때 저장 지도 localization 경로에 이 재시도를 적용했다. 정상 예측이 성공하면 기존 경로를 사용한다. BoW로 얻은 대응점도 pose 최적화와 local map 검사에 통과해야 한다. 정상 프레임마다 전체 지도 검색을 추가하거나, 기하 검증 기준을 낮추어 pose 수만 늘리지 않는다.

현재 프레임에서 `TrackWithMotionModel()`이 `mbVO=true`로 바뀌면 지도 점이 아닌 temporal point 기반 pose가 잠시 정상 상태로 남을 수 있었다. 이전 저장소의 차단 로직은 다음 프레임에서 이를 검사했다. 현재 프레임도 차단하고, 지도 재localization 성공 시 `mbVO`를 해제하도록 수정했다. 또한 `RECENTLY_LOST`에서도 지도 재검색을 수행하고, 복구 직후의 pose 차이를 운동 예측 속도로 사용하지 않는다. 첫 프레임을 지도 활성화만 하고 반환하던 경로도 즉시 localization을 시도하도록 보완했다. 이 변경은 정확한 지도 pose 여부를 보존하기 위한 것이다.

## 재localization 반복 예산과 메모리 수명

[공식 MLPnPsolver 구현](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/MLPnPsolver.cpp)의 반복 조건은 전체 RANSAC 예산과 호출당 예산을 `||`로 연결한다. 이 저장소도 같은 조건이었다. 호출자가 후보별로 5회씩 번갈아 계산하도록 요청해도 한 후보가 더 많은 반복을 수행할 수 있다. 두 예산을 모두 지키도록 `&&`로 수정했다. 전체 후보 검색을 시간 제한으로 잘라내거나 입력 프레임을 생략하는 변경은 아니다.

또한 `Tracking::Relocalization()`이 생성한 후보별 `MLPnPsolver`가 해제되지 않는 경로를 확인했다. 후보 객체를 `unique_ptr`로 소유하고 누락된 소멸자 정의를 추가하여 성공·실패·예외 경로에서 해제한다. 반복되는 추적 복구가 메모리를 계속 점유하지 않도록 하는 수정이다. 후보 간 실행 순서가 달라질 수 있으므로, 정확도 비교와 장시간 재생을 다시 수행한다. `tests/ransac_budget.cpp`는 일치하지 않는 대응점으로 호출당·전체 반복 제한을 확인하고, 알려진 기하에서는 pose 복구가 유지되는지 검사한다.

## 특징점 주변 검색의 메모리 비용

SDK 재생을 포함한 `perf cpu-clock` 프로파일에서 `Frame::GetFeaturesInArea`의 자체 비용이 약 4.7%, `malloc`이 약 4.3%였다. 이 비율은 SDK 재생 스레드까지 포함한 전체 프로세스 표본이며, 추적 시간의 정확한 분해값은 아니다. SDK 파일 읽기에 따른 페이지 할당 비용도 관측되므로 재생 장치 비용을 물리 카메라 비용으로 간주하지 않는다.

Frame·KeyFrame의 주변 검색은 각 격자 셀의 벡터를 복사하고 결과 벡터에 전체 프레임 특징점 수만큼 공간을 예약하고 있었다. 셀은 const reference로 읽고, 결과는 작은 초기 용량 64에서 필요에 따라 계속 확장하도록 수정했다. 64개로 검색 결과를 제한하지 않는다. 검색 범위·레벨 검사·순서는 그대로이며, 영상 밖 검색에서는 불필요한 할당도 피한다. 밀집 셀, 영상 경계, 레벨 범위, 오른쪽 카메라를 포함한 격자 검색을 독립적인 전수 검색과 비교한다.

## 검토했으나 채택하지 않은 수렴 옵션

[g2o의 공식 종료 action](https://github.com/RainerKuemmerle/g2o/blob/master/g2o/core/sparse_optimizer_terminate_action.cpp)을 참고해 pose 갱신량에 따른 내부 반복 조기 종료도 시험했다. 알려진 기하와 이상치 시험에서는 기존 해를 유지했지만 재생 지연 개선이 일관되지 않아 최종 코드에서 제외했다. 최종 pose 최적화의 반복 횟수와 이상치 판정 기준은 그대로 유지한다.

## 회전 블러와 관성 정보

[ORB-SLAM3 논문](https://arxiv.org/abs/2007.11898)은 visual-inertial 최적화를 포함한다. 그러나 이 저장소의 localization-only 분기에는 관성 모드 미지원 TODO가 남아 있다. 기존 visual RGB-D Atlas를 사용하면서 IMU 센서 옵션만 켜는 것은 검증된 해결책이 아니다. IMU-카메라 외부 보정, 시각 정렬, 관성 지도와 초기화 절차까지 별도의 구현·검증이 필요하므로 이번 실행기의 기본 모드에 적용하지 않았다.

[RealSense 성능 조정 문서](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/)를 참고해 노출과 프레임률을 분리해서 다룬다. 실행기는 auto-exposure priority를 꺼 자동 노출에 따른 FPS 감소를 막으며, 수동 color exposure 옵션을 제공한다. 짧은 노출은 조명·게인과 함께 확인해야 하므로 실제 장치 없이 특정 노출값을 최적값으로 고정하지 않는다. 프레임 속도를 유지한다고 영상 블러나 충분한 특징점까지 보장되지는 않는다.

## 프레임 손실을 숨기지 않는 입력 경로

[RealSense frame management 공식 문서](https://dev.realsenseai.com/docs/frame-management/)는 SDK 버퍼 수가 지연과 프레임 손실의 절충임을 설명하며, 동기화 품질도 무조건 보장하지 않는다. 따라서 단순히 `wait_for_frames()`를 호출하거나 ROS queue 크기를 늘리는 것으로 전 프레임 처리를 입증할 수 없다.

RGB·depth 원시 callback의 프레임을 별도 FIFO에 보관하고 수신 스레드에서 직접 짝짓고 정렬·복사한다. SDK syncer가 수신 전 데이터를 버리거나 재사용하는 경로를 제거했다. ORB 추적과 독립적인 FIFO를 사용하며, RGB와 depth 각 프레임 번호를 검사한다. CPU가 늦어지면 대기 시간이 증가하고, 용량을 넘으면 실패한다. 프레임을 버려 지연이 낮게 보이도록 하지 않는다. SDK·USB 단계의 손실도 번호 불연속으로 검사하지만, 첫 수신 전과 스트림 종료 이후의 물리 센서 출력은 이 검사로 입증할 수 없다.

OpenCV thread 수는 현재 노트북의 회전·SDK 재생 비교 후 기본 3으로 설정했다. 초기 데이터셋 시험에서는 2를 선택했지만 SDK 입력을 함께 측정한 결과를 반영했다. 이것은 코어 간 작업 경쟁과 지연 편차를 줄이려는 설정이며, 항상 빠르다고 가정하지 않는다. `--opencv-threads`와 동일 데이터 재생으로 장비별 측정이 가능하다.

카메라 보정에서는 모델 이름만 보고 계수를 뒤집지 않는다. 저장된 D455F 정보는 inverse Brown-Conrady로 표시되어 있다. [SDK 투영 정의](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)와 설치된 SDK 구현을 확인하고, 실제 SDK 투영과 지도 설정의 OpenCV 투영을 영상 전역의 격자에서 비교한다. 차이가 0.5픽셀을 넘으면 실행을 거부한다. 기존 지도와 보정값을 조용히 바꾸지 않으며, 카메라 없이도 저장된 보정 수치에 대한 SDK/OpenCV 호환성 회귀 검사를 수행한다.

[ROS Fast DDS 구현 문서](https://github.com/ros2/rmw_fastrtps#change-publication-mode)에 따라 비동기 발행을 지정한다. publish 호출 완료는 DDS 전송 완료나 상대 노트북 수신 완료와 다르다. 따라서 로컬 publish 지연과 별도 ROS 수신 프로세스의 지연을 각각 기록한다. 영상·점군을 ROS로 발행하는 코드는 이 실행기에 없다.

## 검증 데이터와 평가

[TUM RGB-D 데이터셋](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)과 [파일 형식·카메라 보정](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)을 사용한다. 카메라 설정은 [ORB-SLAM3 공식 TUM1.yaml](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/RGB-D/TUM1.yaml)에 따른다. 입력은 OpenCV BGR이므로 실행 설정의 `Camera.RGB`만 0으로 맞춘다.

`fr1/xyz` 지도 생성 후 다른 `fr1/rpy` 영상에 대해 localization한다. fallback on/off 비교는 같은 Atlas·입력·특징점·thread 수로 수행한다. 이후 특징점/thread 예산을 조정한 결과는 별도 설정 비교로 보고한다. `rpy`는 지도 생성에는 사용하지 않았지만 튜닝에도 사용했으므로, 새 환경에 대한 독립적인 일반화 검증으로 주장하지 않는다. 지연은 프레임별로 대기열 진입부터 로컬 publish 호출 완료까지 측정한다. 궤적 평가는 ground truth의 위치 선형 보간과 회전 SLERP, 스케일 보정 없는 SE(3) ATE, 인접 유효 프레임의 회전·이동 RPE를 사용한다. 큰 누락 구간을 가로질러 RPE를 계산하지 않고 가용률을 함께 보고한다.

위 재생 데이터, SDK 소프트웨어 장치, 동일 PC ROS 수신은 실제 카메라와 원격 LAN 시험을 대체하지 않는다. 특히 공개 데이터의 회전 정확도가 개선되어도 사용자의 실제 지도·조명·장착 상태에서 같은 결과를 단정하지 않는다.
