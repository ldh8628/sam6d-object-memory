# sam6d-object-memory

ORB-SLAM3 로 추정한 **현재 위치**와 SAM-6D 가 낸 **객체 6D 포즈**를 하나의 map 에
등록하고 계속 기억하는(object memory) 스택.

```
sam6d_ws/         실시간 SAM-6D (ISM + PEM)     -> /sam6d/detections
orbslam_ws/       ORB-SLAM3 + ROS2 래퍼          -> /orbslam3/pose
objectmemory_ws/  융합·기억 엔진 + 실시간 노드   -> ~/landmarks (map frame)
integration/      오프라인 일괄 실행 (SLAM + SAM + fuse)
```

## 받은 직후 해야 할 것 — ORB 어휘사전 압축 해제

`ORBvoc.txt` 는 139 MB 라 GitHub 100 MB 파일 한도를 넘는다. 41 MB tar.gz 만
들어 있으므로 **clone 직후 한 번 풀어야** ORB-SLAM3 가 기동한다
(없으면 `Vocabulary/ORBvoc.txt` 를 못 찾고 죽는다).

```bash
tar -xzf orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz \
    -C orbslam_ws/src/ORB_SLAM3/Vocabulary/
ls -lh orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt   # 139M 이면 정상
```

푼 `ORBvoc.txt` 는 `.gitignore` 되어 있으니 다시 커밋되지 않는다.

## 빌드 / 실행

```bash
# ORB-SLAM3 (conda env: orb3slam, ROS 2 Jazzy)
cd orbslam_ws && colcon build --packages-select orbslam3_core orbslam3_ros2 \
    --build-base build_jazzy --install-base install_jazzy \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

실시간 3-노드 배선(`/orbslam3/pose` + `/sam6d/detections` -> `~/landmarks`)은
`objectmemory_ws/object_memory/ros/README.md`, 오프라인 일괄 실행은
`integration/README.md`, SAM-6D 쪽 자산·환경은 `sam6d_ws/README.md` 와
`sam6d_ws/SETUP.md` 를 본다.

`object_memory` 는 순수 표준 라이브러리라 별도 환경 없이 검사된다:

```bash
cd objectmemory_ws/object_memory && python3 -m pytest scripts_test -q
```

## BMAD 개발 자료

단일 노트북 작업의 설정·스킬·명세·연구 및 검증 첨부물은 [BMAD 자료 대조 목록](docs/bmad-history/README.md)을 참고한다. 원본별 보존 위치와 파일별 SHA-256을 포함한다.
