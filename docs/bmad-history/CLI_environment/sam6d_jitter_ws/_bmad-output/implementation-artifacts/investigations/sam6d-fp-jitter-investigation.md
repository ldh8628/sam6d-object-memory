# SAM-6D Milk FP/Jitter Investigation

Date: 2026-06-09
Language: Korean

## 현재 구조 분석

- ROS 노드 기본 파라미터는 `det_score_thresh=0.2`, `top_k_for_pem=3`, `max_proposals=50`, `ism_rerank.enabled=False`이다. Milk 설정에서는 `det_score_thresh=0.2`, `top_k_for_pem=5`, `max_proposals=50`, `no_vis=true`를 사용한다. 근거: `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:69`, `src/sam6d_ros/config/no_cli_milk.yaml:16`.
- 빠른 ROS 경로는 `run_ism_single()`로 ISM JSON을 만들고, 같은 JSON을 `run_pem_single()`에 넘긴다. 근거: `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:775`, `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:809`.
- ISM은 FastSAM/SAM proposal 생성 후 DINOv2 semantic score, appearance score, geometric score를 계산하고 `(semantic + appearance + geometric * visible_ratio) / (2 + visible_ratio)`로 final score를 만든다. 근거: `sam6d_master/SAM-6D/run_batch_inference_fast.py:382`, `sam6d_master/SAM-6D/run_batch_inference_fast.py:416`.
- 빠른 경로는 final score 상위 `top_k_for_pem`를 저장할 뿐, object-existence threshold, score margin, mask-depth consistency gate, reprojection/pose validity gate를 적용하지 않는다. 근거: `sam6d_master/SAM-6D/run_batch_inference_fast.py:425`, `sam6d_master/SAM-6D/run_batch_inference_fast.py:546`, `sam6d_master/SAM-6D/run_batch_inference_fast.py:686`.
- FastSAM 설정의 detector confidence는 `0.05`이고, wrapper는 box confidence/class를 버리고 bbox 좌표만 사용한다. 근거: `sam6d_master/SAM-6D/Instance_Segmentation_Model/configs/model/segmentor_model/fast_sam.yaml:5`, `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/fast_sam.py:118`.
- 원본 `detector.py`에는 small detection 제거와 per-object NMS가 있으나, 현재 custom/fast inference 경로는 이 post-processing을 일부 우회한다. 근거: `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:352`, `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py:388`.
- PEM은 ISM score가 `det_score_thresh`보다 큰 후보를 전부 입력으로 받고, RGB-D mask 안의 점을 랜덤 샘플링한다. 근거: `sam6d_master/SAM-6D/run_batch_inference_fast.py:546`, `sam6d_master/SAM-6D/run_batch_inference_fast.py:587`.
- 저장된 출력 26개 프레임 기준 ISM은 매 프레임 5개 후보를 저장했고, ISM score 범위는 약 0.411-0.639였다. 따라서 현재 `det_score_thresh=0.2`는 이 출력에서는 후보 제거 기능을 하지 않는다.

## Root Cause 분석

### 문제 1: Milk가 없는데도 후보가 통과하는 구조적 원인

1. FastSAM은 object proposal generator이지 Milk detector가 아니다. 장면 내 물체처럼 보이는 region을 만든 뒤, ISM matching score로 Milk 여부를 사후 판정한다.
2. 현재 matching confidence threshold는 semantic stage의 `confidence_thresh=0.2` 수준이고, ROS fast path에서는 최종 object-existence threshold가 없다.
3. `det_score_thresh=0.2`는 PEM 입력 필터인데, 현재 관측 score 분포에서는 너무 낮아 false positive 차단 기능이 없다.
4. FastSAM confidence/class 정보가 버려지고, SAM proposal stability threshold도 FastSAM 경로에는 적용되지 않는다.
5. final score는 semantic/appearance/geometric 평균형 점수라서 "가장 비슷한 후보"를 고르는 데 유리하지만 "대상 없음"을 판단하기에는 calibration이 없다.
6. PEM 결과를 발행하기 전 score, reprojection consistency, depth consistency, temporal persistence gate가 없다.

### 문제 2: 정지 상태 pose jitter 원인

1. PEM 관측 point cloud가 매 inference마다 `np.random.choice`로 재샘플링된다. 같은 RGB-D 입력이어도 네트워크 입력 점 집합이 달라진다.
2. FastSAM mask와 bbox가 미세하게 흔들리면 crop, sampled points, RGB resize 입력이 함께 바뀐다.
3. 현재 PEM은 후보별 pose를 그대로 발행하며 temporal filter, hysteresis, tracker가 없다.
4. 저장된 최고 점수 pose 기준 translation 표준편차는 x=0.76mm, y=4.47mm, z=1.04mm였고, rotation은 180도급 해 차이가 반복되어 대칭/좌표계/후보 선택 ambiguity 가능성이 있다.
5. depth noise, mask 경계 noise, 후보 crop jitter가 모델 입력에 직접 들어가며, 후처리에서 reprojection/depth residual로 pose를 안정화하지 않는다.

## 문제 1 해결안

### 낮은 난이도

- `det_score_thresh`를 0.2에서 0.50-0.60 범위로 올리고, ISM final score 기준 `min_ism_score`를 별도 gate로 추가한다.
- `top_k_for_pem`을 검증 중에는 1로 낮추고, score margin 조건을 둔다: `top1_score >= min_score` 및 `top1_score - top2_score >= margin`.
- FastSAM `conf_threshold`를 0.05에서 0.15-0.30으로 sweep한다.
- mask 면적, bbox aspect ratio, depth valid ratio, depth z-range sanity check를 Milk 크기/거리 범위에 맞춰 추가한다.

### 중간 난이도

- ISM JSON에 semantic, appearance, geometric, visible_ratio를 저장하고, 단일 final score가 아니라 component-wise gate를 적용한다.
- pose 발행 전 render-vs-observed silhouette IoU, visible depth residual, projected model bbox IoU를 계산해 reject한다.
- 동일 후보가 N개 연속 프레임 이상 유지될 때만 object present로 승격하는 temporal persistence gate를 둔다.
- `ism_rerank.enabled`는 진단용으로 켜되, 현재 구현은 큰 mask 선호 규칙이므로 FP 억제에는 단독 사용하지 않는다.

### 높은 난이도

- GroundingDINO/YOLO instance detector + SAM/FastSAM mask로 "Milk 존재 여부"를 먼저 검증한 뒤 SAM-6D를 pose estimator로 사용한다.
- FoundPose식 2D-3D correspondence + PnP-RANSAC inlier count/reprojection error gate를 추가한다.
- FoundationPose/MegaPose식 render-and-compare scorer 또는 pose selection network를 붙여 pose hypothesis를 재점수화한다.

## 문제 2 해결안

### 낮은 난이도

- inference 시작 시 `np.random.seed(cfg.rd_seed)`를 설정하거나, point sampling을 deterministic stride/FPS 방식으로 변경한다.
- 발행 pose에 One Euro filter 또는 EMA를 적용한다. translation은 EMA/One Euro, rotation은 quaternion slerp 기반 smoothing을 사용한다.
- confidence가 낮거나 이전 pose 대비 sudden jump가 큰 프레임은 hold-last-good-pose 처리한다.

### 중간 난이도

- 이전 프레임 bbox/mask를 prior로 사용해 crop jitter를 줄이고, detection hysteresis를 둔다.
- projected model mask와 observed mask의 IoU 및 depth residual이 개선되는 방향으로 pose refinement를 수행한다.
- 대칭/180도 flip이 반복되면 object symmetry set을 정의하고, 이전 pose와 symmetry-aware distance가 최소인 해를 선택한다.

### 높은 난이도

- tracking mode를 추가한다: 첫 프레임은 detector+pose estimator, 이후는 이전 pose 기반 render-and-compare refinement/FoundationPose-style tracking.
- multi-frame optimization 또는 factor graph로 일정 구간의 pose를 공동 최적화한다.

## 우선순위

1. Score/size/depth gate 추가 및 threshold sweep.
2. Deterministic sampling 적용.
3. Pose 발행 전 confidence + reprojection/depth residual gate.
4. Temporal smoothing과 hold-last-good-pose.
5. Render-and-compare verification 또는 PnP-RANSAC 기반 검증.
6. 별도 Milk detector 또는 tracking mode 도입.

## 예상 효과

- Score/geometry gate: 명백한 FP 감소, 구현 빠름.
- Deterministic sampling: 정지 입력 반복 실행 jitter 감소.
- Temporal smoothing: 시각적 흔들림 즉시 감소. 단, 빠른 움직임에는 lag가 생길 수 있음.
- Pose verification: FP 후보가 pose로 발행되는 문제를 크게 줄임.
- Tracking mode: 정지/저속 장면 안정성 향상, detector jitter 영향 감소.

## 구현 난이도

- 쉬움: YAML threshold 조정, top_k=1, score gate, deterministic seed.
- 보통: component score 저장, depth/mask consistency, One Euro/EMA filter, hold-last-good-pose.
- 어려움: PnP-RANSAC correspondence verification, render-and-compare scorer, tracking/refinement mode.

## 권장 개발 순서

1. 현재 bag에서 "Milk 없음" 구간을 분리하고 FP/TP score histogram을 만든다.
2. `min_ism_score`, `min_pem_score`, `min_score_margin`, `min_depth_valid_ratio`, bbox/mask size gate를 ROS parameter로 추가한다.
3. `np.random.choice` 기반 PEM sampling을 deterministic sampling으로 바꾼다.
4. 발행 단계에 `PoseGate`를 추가해 confidence와 residual 검증을 통과한 pose만 publish한다.
5. `PoseSmoother`를 추가해 accepted pose만 temporal filter에 넣는다.
6. 그 다음 render-and-compare 또는 PnP-RANSAC 검증을 붙인다.

## 참고 근거

- SAM-6D 논문: ISM은 SAM proposal별 semantic/appearance/geometric object matching score를 사용하고 threshold로 대상 instance를 식별한다.
- FoundPose 논문: DINOv2 image-to-model 2D-3D correspondences를 만들고 EPnP + RANSAC inlier 품질로 pose hypothesis를 선택한다.
- MegaPose/FoundationPose: coarse pose hypotheses를 render-and-compare/refinement/pose selection score로 고른다.
- OpenCV solvePnPRansac: reprojection threshold, confidence, inliers를 이용해 outlier가 섞인 대응점에서 pose를 robust하게 추정한다.
- One Euro filter: 정지/저속에서는 jitter를 줄이고 빠른 움직임에서는 cutoff를 올려 lag를 줄이는 adaptive low-pass filter다.
