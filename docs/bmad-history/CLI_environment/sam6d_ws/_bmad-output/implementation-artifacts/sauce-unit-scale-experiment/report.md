# Sauce Unit Scale Experiment Report

## 결론

최종 판정: **B + D + E**

- **B:** `xyz_*.npy`만 mm로 바꾸는 것은 부족하다. ISM은 `xyz_*.npy`가 아니라 CAD mesh sample pointcloud를 geometric projection에 사용하므로, meter CAD가 남아 있으면 동일하게 실패한다.
- **D:** PLY/CAD를 mm 단위로 변환하고 template을 재렌더링하면 ISM/PEM core는 작동한다. 다만 현재 bag은 sauce 객체 포함 여부가 불명확하고 score/publish가 낮아 실제 정확도 평가는 sauce 포함 bag이 필요하다.
- **E:** 실사용 후보는 `vertex_160000` 우선이다. `vertex_630000`도 ISM/PEM core는 작동하지만 load/render 비용이 크고 정확도 이점이 검증되지 않았다.

## 실행 방식

모든 실험은 production ROS2 publish 경로를 건드리지 않고 offline shadow evaluation으로 수행했다.

공통 환경:

```bash
cd /home/etri/CLI_environment/sam6d_ws
source /home/etri/miniconda3/etc/profile.d/conda.sh
conda activate sam6d_ros_humble
PYTHONPATH=/home/etri/miniconda3/envs/sam6d_test/lib/python3.11/site-packages:$PYTHONPATH ...
```

사용 RGB-D:

```text
_bmad-output/implementation-artifacts/sauce-template-ism-pem-eval/input_frames
```

프레임 수: 30개

## 수정/추가 파일

Helper:

- `tools/prepare_sauce_unit_scale_experiment.py`
- `tools/evaluate_sauce_template_readiness.py`에 `--targets-json` 옵션 추가

생성 산출물 root:

```text
_bmad-output/implementation-artifacts/sauce-unit-scale-experiment
```

## 1. 단위 진단 결과

SAM-6D code 근거:

- `sam6d_master/SAM-6D/run_batch_inference_fast.py:346-347`: ISM CAD pointcloud를 `mesh.sample(...)/1000.0`으로 변환
- `sam6d_master/SAM-6D/run_batch_inference_fast.py:641-642`: PEM CAD pointcloud도 `/1000.0`
- `sam6d_master/SAM-6D/run_batch_inference_fast.py:652-653`: PEM template `xyz_*.npy`도 `/1000.0`
- `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:215-222`: ISM geometric projection은 CAD pointcloud 기반
- `sam6d_master/SAM-6D/Instance_Segmentation_Model/utils/bbox_utils.py:214-220`: bbox intersection 실패 시 `0.0` float 반환

| Target | CAD Extent Z | XYZ Extent Z | Unit Guess | SAM-6D Compatible |
| --- | ---: | ---: | --- | --- |
| Milk reference | 194.999996 | 186.0 | mm | yes |
| sauce vertex_160000 original | 0.185650 | 0.175659 | meter | no |
| sauce vertex_630000 original | 0.185662 | 0.175659 | meter | no |
| sauce vertex_160000 PLY-mm | 185.65023 | ~175-185 | mm | yes |
| sauce vertex_630000 PLY-mm | 185.66158 | ~175-185 | mm | yes |

CSV:

- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/unit_diagnostics.csv`

## 2. npy-only 수정 결과

작업:

- 기존 template을 아래로 복사
  - `template/sauce_unit_test/vertex_160000_npy_mm/Sauce_with_normal_vertexcolor/templates`
  - `template/sauce_unit_test/vertex_630000_npy_mm/Sauce_with_normal_vertexcolor/templates`
- 복사본의 `xyz_*.npy`만 1000배 변환
- `rgb_*.png`, `mask_*.png`, 원본 CAD/PLY는 변경하지 않음

| Template | XYZ Scale Before | XYZ Scale After | ISM Success | PEM Success | Verdict |
| --- | --- | --- | ---: | ---: | --- |
| sauce_vertex_160000_npy_mm | meter-like | mm-like | 0/30 | 0/30 | failed |
| sauce_vertex_630000_npy_mm | meter-like | mm-like | 0/30 | 0/30 | failed |

실패 로그:

```text
[ISM ERROR] 'float' object has no attribute 'detach'
```

원인:

`xyz_*.npy`는 PEM template feature 쪽에서 사용된다. 현재 실패는 PEM 전 단계인 ISM geometric projection에서 발생하며, ISM은 CAD mesh에서 샘플링한 pointcloud를 `/1000` 처리해 사용한다. 따라서 `xyz_*.npy`만 mm로 바꿔도 meter CAD가 남아 있으면 동일하게 실패한다.

CSV:

- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/npy_only_xyz_scale.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/npy_only_result.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/npy_only_eval/inference_frame_metrics.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/npy_only_eval/summary_metrics.csv`

## 3. PLY-mm 변환 결과

작업:

- 원본 PLY는 덮어쓰지 않음
- ASCII PLY에서 vertex `x/y/z`만 1000배
- `nx/ny/nz`, `s/t`, `red/green/blue`, face index는 그대로 보존

| Input PLY | Output PLY | BBox Before | BBox After | Color Preserved | Status |
| --- | --- | --- | --- | --- | --- |
| `data/cad/sauce/vertex_160000/Sauce_with_normal_vertexcolor.ply` | `data/cad/sauce_mm/vertex_160000/Sauce_with_normal_vertexcolor_mm.ply` | `[0.073117, 0.063, 0.18565]` | `[73.11724, 62.99992, 185.65023]` | True | ok |
| `data/cad/sauce/vertex_630000/Sauce_with_normal_vertexcolor.ply` | `data/cad/sauce_mm/vertex_630000/Sauce_with_normal_vertexcolor_mm.ply` | `[0.07312, 0.063, 0.185662]` | `[73.1204, 62.99991, 185.66158]` | True | ok |

CSV:

- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/ply_mm_conversion.csv`

## 4. PLY-mm 재렌더링 결과

렌더링 명령 형식:

```bash
PYTHONPATH=/home/etri/miniconda3/envs/sam6d_test/lib/python3.11/site-packages:$PYTHONPATH \
/home/etri/miniconda3/envs/sam6d_test/bin/blenderproc run \
  sam6d_master/SAM-6D/Render/render_custom_templates.py -- \
  --cad_path data/cad/sauce_mm/<variant>/Sauce_with_normal_vertexcolor_mm.ply \
  --output_dir template/sauce_mm/<variant>/Sauce_with_normal_vertexcolor_mm
```

`--colorize`는 사용하지 않았다.

| Template | RGB Count | Mask Count | XYZ Count | RGB Color OK | XYZ Unit OK | Status |
| --- | ---: | ---: | ---: | --- | --- | --- |
| sauce_vertex_160000_ply_mm | 42 | 42 | 42 | yes | yes | ok |
| sauce_vertex_630000_ply_mm | 42 | 42 | 42 | yes | yes | ok |

렌더링 비용:

- `vertex_160000` PLY import: 11.598 s
- `vertex_630000` PLY import: 70.851 s

CSV:

- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/rerender_result.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/ply_mm_inspect/template_structure.csv`

## 5. ISM/PEM 결과 비교

동일한 30개 RGB-D frame으로 비교했다.

| Condition | Template | ISM Success Rate | PEM Success Rate | Publish Rate | Mean Semantic | Mean Appearance | Mean Geometric | Mean Time |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| existing meter template | sauce_vertex_160000 | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | 0.106 s |
| existing meter template | sauce_vertex_630000 | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | 0.104 s |
| npy-only xyz mm | sauce_vertex_160000_npy_mm | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | 0.112 s |
| npy-only xyz mm | sauce_vertex_630000_npy_mm | 0.000 | 0.000 | 0.000 | n/a | n/a | n/a | 0.108 s |
| PLY-mm rerender | sauce_vertex_160000_ply_mm | 1.000 | 0.033 | 0.000 | 0.4011 | 0.5103 | 0.3068 | 0.136 s |
| PLY-mm rerender | sauce_vertex_630000_ply_mm | 1.000 | 0.033 | 0.000 | 0.4068 | 0.5170 | 0.3251 | 0.131 s |

해석:

- PLY-mm 재렌더링 후에는 두 버전 모두 ISM exception이 사라져 **30/30 ISM 정상 실행**.
- 후보 threshold를 통과한 1/30 frame에서 PEM까지 진입.
- publish는 0/30이다. 현재 bag이 sauce 객체를 포함한다는 근거가 없어, 이 결과는 template 구조 실패라기보다 입력 scene/object mismatch 또는 threshold 미충족으로 보는 것이 타당하다.

CSV:

- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/ism_pem_condition_comparison.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/ply_mm_eval/inference_frame_metrics.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/ply_mm_eval/topk_candidate_scores.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/ply_mm_eval/summary_metrics.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/ply_mm_eval/template_load_times.csv`

## 6. 실패 원인 분석

### npy-only 실패

실패 위치:

- ISM `run_ism_single()`
- geometric score 이후 `geometric_score.detach()`

원인:

- `xyz_*.npy`만 mm로 바꿔도 ISM reference pointcloud는 여전히 meter CAD에서 생성된다.
- `run_batch_inference_fast.py`가 CAD sample을 `/1000` 하므로 pointcloud가 1000배 작아진다.
- projection bbox와 proposal bbox intersection이 깨지고, `compute_iou()`가 tensor 대신 float `0.0`을 반환한다.

필요 수정:

- PLY/CAD 좌표부터 mm 단위로 맞추고 template을 재렌더링해야 한다.
- 별도 안정성 개선으로 `compute_iou()`의 fallback을 tensor 반환으로 바꾸는 것도 필요하지만, 이는 production code behavior change라 이번 작업에서는 수정하지 않았다.

### PLY-mm 후 publish 0/30

실패 위치:

- ISM exception은 없음
- 대부분 `no_accepted_ism_candidate`
- 1프레임은 PEM 진입 후 `pose_score_below_min`

원인:

- 현재 bag에 sauce 객체가 포함되어 있다는 근거가 없다.
- Mean semantic/appearance/geometric score가 형성되긴 하지만 publish threshold를 넘지 못했다.

필요 수정:

- sauce 포함 RGB-D bag으로 재평가
- 필요 시 threshold tuning은 별도 실험에서 수행

## 7. 최종 추천

1. **npy만 수정하지 말 것.** ISM 실패가 해결되지 않는다.
2. **SAM-6D용 sauce CAD는 별도 mm PLY로 유지할 것.**
   - 원본 meter PLY는 보존
   - `data/cad/sauce_mm/...`를 SAM-6D 입력 CAD로 사용
3. **template은 PLY-mm 기반으로 재렌더링한 `template/sauce_mm/...`를 사용할 것.**
4. **우선 `vertex_160000` 사용 권장.**
   - `vertex_630000`은 import/render/load 비용이 크다.
   - 현재 score/publish 이점은 확인되지 않았다.
5. **실제 sauce 객체가 들어 있는 bag을 추가 수집한 뒤 재평가할 것.**
   - 현재 결과는 “unit 수정 후 SAM-6D core 작동 가능”까지만 증명한다.
   - 실제 pose 정확도/안정성 판단은 sauce 포함 scene이 필요하다.

## 8. 주요 산출물 경로

- `data/cad/sauce_mm/vertex_160000/Sauce_with_normal_vertexcolor_mm.ply`
- `data/cad/sauce_mm/vertex_630000/Sauce_with_normal_vertexcolor_mm.ply`
- `template/sauce_unit_test/vertex_160000_npy_mm/Sauce_with_normal_vertexcolor/templates`
- `template/sauce_unit_test/vertex_630000_npy_mm/Sauce_with_normal_vertexcolor/templates`
- `template/sauce_mm/vertex_160000/Sauce_with_normal_vertexcolor_mm/templates`
- `template/sauce_mm/vertex_630000/Sauce_with_normal_vertexcolor_mm/templates`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/ism_pem_condition_comparison.csv`
- `_bmad-output/implementation-artifacts/sauce-unit-scale-experiment/report.md`
