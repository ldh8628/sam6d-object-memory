# Story 1+2 (minimal): Offline Object Memory Integration

Date: 2026-07-01

## Goal

3개 파일 입력만으로 `T_map_obj = T_map_cam · T_cam_obj`를 계산해 persistent
`object_id`를 유지하는 오프라인 object_memory. ROS2 라이브 노드 없음, bag
"인벤토리" 반복 없음 — bag는 프레임→타임스탬프 매핑 1회에만 사용.

## Three Inputs (real, no fakes)

| # | 입력 | SAM_occlusion 실제 경로 | 산출 |
| - | --- | --- | --- |
| 1 | SLAM 궤적 | `slam_comparison/output/SAM_occlusion/orbslam3_noimu/CameraTrajectory.txt` (TUM, 795행) | `T_map_cam` |
| 2 | SAM-6D pose | `sam6d_ws/outputs/pem_inputs/SAM_occlusion/frame_*/pem_<Obj>/sam6d_results/detection_pem.json` | `T_cam_obj` (R, t mm→m) |
| 3 | 공통 bag | `data_slam/rgbd_imu_sdk_bag/SAM_occlusion/bag/bag_0.db3` (color 795) | frame_idx → timestamp |

정렬 검증: bag color 795 = SLAM 궤적 795행, ts 범위 동일(`…483.858337` 시작 일치).

## New Code (Story 0 core 재사용)

- `src/core/transforms.py`: `quat_to_rotation`, `make_transform_from_quat` 추가.
- `src/adapters/slam_trajectory.py`: TUM → `[SlamCameraPose]`.
- `src/adapters/bag_frame_index.py`: bag color 토픽 → `frame_idx → timestamp` (bag 유일 사용, read-only).
- `src/adapters/sam6d_pem.py`: PEM 트리 → `{frame_idx: [Sam6DDetection]}` (mm→m).
- `src/pipeline/object_memory_runner.py`: fusion + object_name-키 association + 최소 lifecycle.
- `scripts_test/run_object_memory.py`: CLI 러너.

## Association MVP

object_name 키: 이름당 landmark 1개(INV-002 object_memory가 object_id 소유,
INV-003 detection_id는 identity 아님). Lifecycle: 2회 관측→active,
3회 연속 miss→lost, 재검출→active. INV-008: SLAM pose 없는 프레임은 memory
갱신 차단(rejected).

## Run Result (SAM_occlusion)

- SLAM poses 795, bag timestamps 795, 검출 프레임 53, fused 53/53 (SLAM gap 0), 검출 105.
- persistent objects 3:
  - #1 Bear active obs=51
  - #2 Rabbit active obs=51
  - #3 milk lost obs=3 (occlusion) — `active→lost→active→lost` 재등장 lifecycle 재현.
- 결정: new_tentative 3, short_term_match 102.
- 상세: `reports/object_memory_run_SAM_occlusion.md`.

## Validation

```bash
cd objectmemory_ws/object_memory
python3 -m compileall src scripts_test        # OK
python3 -m pytest scripts_test -q             # 44 passed (실데이터 smoke 3 포함)
python3 scripts_test/run_object_memory.py \
  --slam-traj ../../slam_comparison/output/SAM_occlusion/orbslam3_noimu/CameraTrajectory.txt \
  --pem-dir   ../../sam6d_ws/outputs/pem_inputs/SAM_occlusion \
  --bag       ../../data_slam/rgbd_imu_sdk_bag/SAM_occlusion \
  --output-report reports/object_memory_run_SAM_occlusion.md   # exit 0
```

## 다음 액션 제안

1. **(권장) SAM_loop2(197f)로 long-term 재확인** — 물체가 오래 사라졌다 재등장하는
   케이스에서 object_id가 유지되는지. object_name 키는 그대로.
2. **동일 클래스 다중 인스턴스 대응** — object_name+맵거리 반경 association으로 확장
   (Story 4/5 최소형). 지금은 클래스당 1개 가정.
3. **Bear/Rabbit 근접 위치 원인 점검** — 두 객체 T_map_obj가 거의 동일(≈같은 위치).
   SAM-6D PEM pose 품질 이슈인지, 실제 근접 배치인지 확인 필요.
4. **정지 여기서 검토/커밋** — 코드·리포트 리뷰 후 커밋.

질문: 다음은 (1) 다른 bag 일반화, (2) 다중 인스턴스 association 확장, (3) Bear/Rabbit
위치 품질 점검 중 무엇을 우선할까요?
