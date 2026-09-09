# Sauce Template ISM/PEM Readiness Evaluation

## 결론

판정: **E. 렌더링/template 구조 문제로 현재 사용 부적합**

colorized sauce `rgb_*.png` 자체는 색상 정보가 정상 반영되어 있고, `rgb/mask/xyz` 파일 수와 해상도도 SAM-6D 입력 구조를 만족한다. 그러나 현재 sauce CAD와 template `xyz_*.npy`가 **meter 단위 값**으로 들어가 있으며, SAM-6D core는 Milk 기준처럼 **mm 단위 입력을 전제로 `/1000` 변환**을 수행한다. 그 결과 sauce reference pointcloud가 실제보다 1000배 작아지고, ISM geometric score 계산에서 projection bbox가 깨져 `compute_iou()`가 tensor가 아닌 `0.0` float를 반환한다. 이후 `run_batch_inference_fast.py`가 `.detach()`를 호출하면서 sauce는 30/30 프레임 모두 ISM 단계에서 실패했다.

따라서 현재 template은 색상 문제는 해결됐지만, **scale/unit 불일치 때문에 ISM -> PEM 입력으로 사용할 수 없다.**

## 실행 방식

실행 방식: **offline shadow evaluator**

기존 ROS2 node 및 publish topic은 변경하지 않았고, `sam6d_master/SAM-6D/run_batch_inference_fast.py`의 ISM/PEM core를 helper script에서 직접 호출했다.

환경:

```bash
cd /home/etri/CLI_environment/sam6d_ws
source /home/etri/miniconda3/etc/profile.d/conda.sh
conda activate sam6d_ros_humble
PYTHONPATH=/home/etri/miniconda3/envs/sam6d_test/lib/python3.11/site-packages:$PYTHONPATH ...
```

비고: `sam6d_ros_humble` 자체에는 `torch`, `hydra`, `gorilla`, `pycocotools`, `trimesh`, `PIL`이 없어서, 활성화된 `sam6d_ros_humble` Python 3.11에서 `sam6d_test` site-packages를 추가해 SAM-6D core dependency를 로드했다.

## 사용 입력

bag:

```text
/home/etri/ros2_bag_recording/output/high_texture_far_close/bag
```

추출 결과:

```text
_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/input_frames
```

추출 프레임: 30개  
토픽:

- RGB: `/camera/camera/color/image_raw`
- Depth: `/camera/camera/aligned_depth_to_color/image_raw`
- max sync slop: 80 ms

프레임 추출 명령:

```bash
python tools/extract_rosbag_rgbd.py \
  --bag /home/etri/ros2_bag_recording/output/high_texture_far_close/bag \
  --output-dir _bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/input_frames \
  --stride 70 \
  --max-frames 30 \
  --max-sync-slop-ms 80
```

## Helper Script

추가 생성:

```text
tools/evaluate_sauce_template_readiness.py
```

역할:

- Milk/sauce template 구조 검사
- 30개 RGB-D 프레임에 대해 동일 조건으로 ISM -> PEM shadow evaluation
- frame metrics CSV, top-k candidate CSV, timing plot, score plot, publish plot 생성
- scale diagnostics CSV 생성은 별도 진단 명령으로 수행

평가 실행 명령:

```bash
PYTHONPATH=/home/etri/miniconda3/envs/sam6d_test/lib/python3.11/site-packages:$PYTHONPATH \
python tools/evaluate_sauce_template_readiness.py \
  --workspace . \
  --rgb-dir _bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/input_frames/rgb \
  --depth-dir _bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/input_frames/depth \
  --output-dir _bmad-output/implementation-artifacts/sauce-template-ism-pem-eval \
  --max-frames 30
```

## Template 구조 검사

| Template | RGB Count | Mask Count | Depth/XYZ Count | Resolution | Empty RGB | Verdict |
| --- | ---: | ---: | ---: | --- | ---: | --- |
| Milk reference | 42 | 42 | 42 xyz | 512x512 | 0 | ok |
| sauce vertex_160000 | 42 | 42 | 42 xyz | 512x512 | 0 | ok |
| sauce vertex_630000 | 42 | 42 | 42 xyz | 512x512 | 0 | ok |

구조만 보면 `rgb_i.png`, `mask_i.png`, `xyz_i.npy`는 모두 존재한다. 별도 `obj_meta.json`이나 depth template 파일은 현재 `run_batch_inference_fast.py` 경로에서는 필수 입력이 아니다.

## Template 품질 요약

| Template | Mean Unique Colors/View | Object Area Ratio Mean | BBox W Mean | BBox H Mean |
| --- | ---: | ---: | ---: | ---: |
| Milk reference | 2,134.4 | 0.046204 | 149.0 | 117.6 |
| sauce vertex_160000 | 8,783.2 | 0.115506 | 123.8 | 298.4 |
| sauce vertex_630000 | 8,390.9 | 0.115367 | 123.6 | 298.4 |

색상/texture 측면에서는 sauce가 단색이 아니며 unique color도 충분하다. 다만 view별 bbox가 Milk 대비 매우 세로로 길고 object area 비율이 약 2.5배 크다.

## 시간 성능 결과

| Template | Mean Time | Median Time | P95 Time | FPS | ISM Time | PEM Time | Verdict |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| Milk reference | 0.211 s | 0.193 s | 0.344 s | 4.738 | frame CSV 참조 | frame CSV 참조 | 정상 실행, bag 내 대상 객체 가능성 낮음 |
| sauce vertex_160000 | 0.106 s | 0.101 s | 0.134 s | 9.443 | ISM error까지 | PEM 미실행 | 실패가 빨라 runtime 비교 불가 |
| sauce vertex_630000 | 0.104 s | 0.098 s | 0.132 s | 9.574 | ISM error까지 | PEM 미실행 | 실패가 빨라 runtime 비교 불가 |

주의: sauce의 mean time은 정상 inference 시간이 아니라 ISM exception 발생까지의 시간이다.

Template loading:

| Template | ISM Template Load | PEM Template Load | CAD Load |
| --- | ---: | ---: | ---: |
| Milk reference | 0.716 s | 4.178 s | 0.016 s |
| sauce vertex_160000 | 4.233 s | 2.977 s | 2.924 s |
| sauce vertex_630000 | 16.421 s | 2.690 s | 11.845 s |

vertex_630000은 vertex_160000 대비 CAD load가 약 4.0배, ISM template load가 약 3.9배 느리다. 정상 scale로 고친 뒤에도 runtime 측면에서는 vertex_160000이 더 실용적일 가능성이 높다.

## Score/Pose 성능 결과

| Template | Publish Rate | Mean ISM Score | Mean PEM Score | T Delta Mean | R Delta Mean | Outliers | Verdict |
| --- | ---: | ---: | ---: | --- | --- | ---: | --- |
| Milk reference | 0.033 | 0.0543 | 0.5151 | n/a | n/a | 0 | 1/30 publish, bag 대상 객체 부재 가능성 높음 |
| sauce vertex_160000 | 0.000 | 0.0000 | n/a | n/a | n/a | 0 | ISM 30/30 실패 |
| sauce vertex_630000 | 0.000 | 0.0000 | n/a | n/a | n/a | 0 | ISM 30/30 실패 |

Milk도 1/30만 publish되어 이 bag은 sauce뿐 아니라 Milk 절대 정확도 평가에도 적합하지 않을 가능성이 높다. 다만 Milk는 core가 정상적으로 30/30 ISM을 통과했고, sauce는 30/30 exception으로 실패했으므로 sauce template 문제는 bag 내 객체 존재 여부와 별개로 확인된다.

## 실패 원인 근거

SAM-6D path:

- `sam6d_master/SAM-6D/run_batch_inference_fast.py`
- `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py`
- `sam6d_master/SAM-6D/Instance_Segmentation_Model/utils/bbox_utils.py`

실패 메시지:

```text
[ISM ERROR] 'float' object has no attribute 'detach'
```

발생 흐름:

1. `detector.py::compute_geometric_score()`가 `compute_iou(xyxy, proposals.boxes)` 호출
2. `bbox_utils.py::compute_iou()`는 intersection width/height 중 하나라도 non-positive이면 `iou = 0.0` float 반환
3. `run_batch_inference_fast.py`는 `geometric_score.detach().cpu().numpy()`를 기대
4. sauce에서는 `geometric_score`가 float라 `.detach()` 실패

scale diagnostics:

| Template | CAD Raw Z Extent | Ref Pointcloud Z After `/1000` | Template XYZ Z Mean Range |
| --- | ---: | ---: | ---: |
| Milk reference | 194.999996 | 0.194997 m | 186.0 |
| sauce vertex_160000 | 0.185650 | 0.000186 m | 0.175659 |
| sauce vertex_630000 | 0.185662 | 0.000186 m | 0.175659 |

Milk CAD/template는 mm 단위 값이고, `/1000` 후 meter scale이 된다. Sauce CAD/template는 이미 meter 단위인데 동일하게 `/1000` 처리되어 1000배 작아진다.

## vertex_160000 vs vertex_630000

현재 상태에서는 둘 다 ISM 단계에서 실패하므로 정확도 비교는 불가능하다.

사용 후보를 고른다면, scale 수정 후 재평가 전제에서 **vertex_160000 우선**이 타당하다.

근거:

- template RGB/mask/xyz 구조는 두 버전 모두 동일하게 42 view
- 색상/area/bbox 분포도 거의 동일
- vertex_630000은 CAD load 11.845 s, ISM template load 16.421 s로 훨씬 무거움
- vertex_630000의 정확도 이점은 현재 검증되지 않음

## 산출물

CSV:

- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/template_structure.csv`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/template_view_quality.csv`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/inference_frame_metrics.csv`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/topk_candidate_scores.csv`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/summary_metrics.csv`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/template_load_times.csv`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/scale_diagnostics.csv`

Images:

- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/template_contact_sheet.png`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/qualitative_contact_sheet.png`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/inference_time_plot.png`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/score_distribution_plot.png`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/publish_timeline_plot.png`
- `_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/selected_frame_overlay_milk_reference_001120.png`

## 다음 수정 방향

1. Sauce CAD/template unit을 SAM-6D expectation에 맞춘다.
   - 현재 sauce CAD extents는 meter 단위다.
   - SAM-6D 현 경로는 CAD mesh sample과 `xyz_*.npy`를 `/1000` 처리하므로, 입력 CAD와 template xyz는 Milk처럼 mm 값이어야 한다.
   - 원본 PLY를 직접 수정하지 말고, SAM-6D용 scaled CAD/template을 별도 생성하는 방식이 안전하다.

2. `render_custom_templates.py` 실행 시 scale 정책을 명시한다.
   - sauce PLY가 meter 단위라면 render 전에 1000배 scale하거나, renderer/SAM-6D loader 양쪽의 단위 처리를 일치시켜야 한다.

3. `compute_iou()`의 float fallback은 별도 안정성 버그다.
   - projection bbox가 겹치지 않을 때도 tensor shape을 유지해 `torch.zeros_like(...)` 형태로 반환해야 downstream `.detach()`가 실패하지 않는다.
   - 다만 이 코드는 production path 변경이므로 이번 작업에서는 수정하지 않았다.

4. scale-normalized sauce template을 만든 뒤 동일 evaluator로 재실행한다.
   - 먼저 vertex_160000으로 검증 권장
   - vertex_160000이 score/publish 안정성을 보이면 vertex_630000은 필요 시만 비교

5. 실제 sauce 객체가 포함된 bag을 추가 수집한다.
   - 이번 bag은 Milk reference도 1/30 publish에 그쳐 절대 정확도/GT 평가는 제한적이다.
   - scale 문제가 해결된 뒤 sauce 포함 RGB-D bag으로 overlay, score stability, pose jitter를 다시 평가해야 한다.
