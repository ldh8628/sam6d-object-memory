# 다른 PC 데이터 변환 프롬프트

`/mnt/nvme1` 의 `sam_slam.tar.gz` / `data_slam` 을 압축 해제한 PC에서, 아래 블록을 그대로
Claude Code 에 붙여넣는다.

---

```
data_slam(SDK 포맷 RealSense bag)을 sam6d_realtime / ORB-SLAM3 / RTAB-Map 입력으로
변환하는 작업. tar 압축은 이미 풀려 있음.

[변환기] data_slam/260714_frame_data/convert_recording.py — 스택 내 유일한 변환기
  python convert_recording.py <recording_dir> <out_bag> --time-source color_global
  출력 4토픽: /camera/camera/{color/image_raw, aligned_depth_to_color/image_raw,
              color/camera_info, aligned_depth_to_color/camera_info}
  = sam6d_realtime topics 기본값과 동일 → 리맵 불필요

[수정할 것] 아래 3개를 convert_recording.py에 반영
  1. :202 파싱해놓고 버리는 cenc 검증 — bgr8 아니면 중단(rgb8이면 R/B 뒤바뀐 채 통과함)
  2. :205 앞 (cw,ch)==(dw,dh)==(W,H) assert
  3. :186 ci.d=[0.0]*5 → 실제 kc 싣기. 추가로 bag 옆 사이드카 json에
     baseline(t_d2c[0]), stride, offset_ns, frames_written vs len(assoc) 기록
     (지금은 0프레임이어도 DONE 찍고 exit 0)

[확정할 것] 심볼릭 링크/경로가 깨져 있으니 다음을 정하고 문서화
  - 어느 세션을 원본으로 쓸지, 변환 출력 경로, sam6d_realtime 실행 config 경로
  - 끊긴 참조: data_slam/0724_chungbuk (심링크), 그 안의 dump_hdl_graph.py /
    downgrade_bag_metadata.py, 비어있는 260807_rgbd_standard/
  - 주의: integration/pipeline/convert.py:47이 metadata.yaml 존재만 보고 스킵해서
    --stride/--max로 만든 잘린 bag을 조용히 재사용함

[검증] 변환 후 ros2 bag info로 4토픽 확인 → run_split_example.yaml 복사해 bag.path만
  바꾸고 (conda activate sam6d, ROS_DOMAIN_ID=72)
  ros2 launch realtime/launch/sam6d_split.launch.py config:=<복사본>
  detections.jsonl 생성 확인. ORB3/RTAB도 같은 bag으로 돌려 pose 나오는지 확인.

[마무리] 수정된 convert_recording.py를 ldh8628/sam6d-object-memory main에 push
  (sam6d_realtime/data/ 는 .gitignore 대상이라 git add -f 필요)
```

---

## 배경 (프롬프트에는 넣지 않아도 되는 근거)

### 변환이 필요한 이유
원본은 `/device_0/...` 토픽에 페이로드가 `std_msgs/String` 인 SDK 포맷이고 **depth 가 color 에
정렬돼 있지 않다**(color↔depth 병진 −0.059 m). 어떤 소비자도 직접 읽지 못한다.

### 전제조건 (51개 세션 전수조사 결과 50개 통과)
세션 폴더에 다음이 있어야 한다.
- `*.db3` 정확히 1개
- `rgbd_timestamp_associations.json` (associations 비어있지 않을 것)
- SDK 토픽 7종: `Depth_0/camera_info`, `Color_0/camera_info`, `Color_0/tf/ref_0`,
  `option/Depth_Units/value`, `Color_0/image/{data,metadata}`, `Depth_0/image/data`

유일한 실패 세션은 `0807_chungbuk/SAM/260807/recording_20260807_164829` — 녹화가 중단돼
db3·association·metadata 가 전부 없다. 스킵할 것.

### 실측된 데이터 균질성
260714 / 260804 / 0807 표본 전부 color `bgr8` 640×480, depth `mono16` 640×480,
`depth_units=0.001`. 그래서 현 자산에서는 해상도·단위 문제가 안 나지만, **코드가 검사해서가
아니라 데이터가 우연히 균질한 것**이다. 위 [수정할 것] 1·2 가 이 부분을 막는다.

### 알려진 정확도 손실
- **왜곡계수 폐기**: `convert_recording.py:186` 이 `ci.d=[0.0]*5` 를 쓴다. 실제 k1≈−0.055,
  k2≈+0.064. 정렬 계산에는 제대로 쓰면서(`:150-153`) 출력 CameraInfo 에만 0 을 넣는다.
- **`Stereo.b` 미전달**: depth→color 병진 −0.0592 m 를 로그로만 출력. `run_integration.py:947`
  기본값 0.095 가 쓰인다. 레포 내에서도 값이 갈린다(0.09498 vs 0.0592) — 미결 항목.
- **0807 10개 세션 전부 `settings/` 없음** → ORB-SLAM3 는 전부 자동생성 설정 경로로 간다.

### 소비자별 판정
| 소비자 | 판정 |
|---|---|
| RTAB-Map | 안전. 4개 메시지가 같은 `stamp` 객체를 공유해 `approx_sync:=false` 가 구조적으로 성립 |
| ORB-SLAM3 | 항상 실행됨. 단 설정 yaml 이 없으면 왜곡 0 + 베이스라인 0.095 로 정확도가 달라짐 |
| SAM-6D | 안전. 토픽명·`bgr8`·mm 단위가 정확히 일치하고 `msg.d` 는 어차피 무시 |
| hdl_graph_slam | **해당 없음** — PointCloud2 를 먹으므로 Velodyne bag 이 필요하다 |

### 주의
- `--stride N` / `--max M` 은 스모크 테스트용이다. 전체 변환에는 빼야 한다.
- 출력 폴더가 이미 있으면 `out bag exists, aborting` 으로 중단한다.
- 실행 환경: `conda activate sam6d_ros_humble` (또는 `orbslam3`) — `rosbag2_py` + `rclpy` 필요.
