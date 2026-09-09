---
stepsCompleted: [1, 2, 3, 4, 5, 6]
inputDocuments: []
workflowType: 'research'
lastStep: 6
research_type: 'technical'
research_topic: 'Static Object 6D Pose Jitter Reduction for SAM-6D'
research_goals: 'SAM-6D RGB-D 6D object pose output에서 정지 객체의 frame-to-frame pose jitter를 실시간 ROS2 조건에서 줄이는 최신 연구 및 적용 알고리즘 조사'
user_name: 'Etri'
date: '2026-06-10'
web_research_enabled: true
source_verification: true
---

# Static Object 6D Pose Jitter Reduction for SAM-6D: Technical Research Report

## Executive Summary

SAM-6D의 현재 구조는 RGB-D 단일 프레임을 입력으로 ISM(instance segmentation)과 PEM(pose estimation)을 수행하는 frame-wise zero-shot 6D pose pipeline이다. 이 구조는 recall과 per-frame accuracy에는 강하지만, 정지 객체를 연속 프레임에서 볼 때 pose가 시간적으로 일관되어야 한다는 제약을 직접 모델링하지 않는다. 따라서 수 mm~수 cm translation 흔들림과 수 degree 이하 rotation fluctuation은 알고리즘 관점에서 자연스러운 실패 모드다.

2023-2026 문헌에서 이 문제를 가장 직접적으로 다룬 연구는 Zorina et al.의 "Temporally Consistent Object 6D Pose Estimation for Robot Control"이다. 이 논문은 정지 객체의 per-frame pose가 일정해야 한다는 문제를 명시하고, factor graph 기반 online smoothing, motion model, measurement uncertainty, outlier rejection, symmetry tracking을 결합한다. FoundationPose, BundleSDF, GoTrack은 temporal tracking 계열에서 중요한 후보지만, SAM-6D에 바로 붙이기에는 구조 변경 또는 추가 추론 비용이 더 크다.

실시간 ROS2 시스템에서 "내일 바로 구현"해야 한다면 SAM-6D 추론 자체는 그대로 두고, PoseArray/TF publish 직전에 per-object SE(3) pose stabilizer node를 추가하는 것이 최적이다. 1차 구현은 One Euro Filter 또는 adaptive EMA를 translation과 rotation(quaternion slerp/log map)에 적용하고, confidence/innovation gate와 stationary detector를 붙인다. 석사 논문 주제로는 SAM-6D의 PEM matching residual, ISM score, depth consistency를 measurement uncertainty로 변환해 factor graph 또는 Lie-group EKF/RTS smoother에 넣는 "uncertainty-aware temporal pose consistency for zero-shot RGB-D 6D pose"가 가장 타당하다.

## Source Basis

- SAM-6D, CVPR 2024: RGB-D, ISM+PEM, zero-shot frame-wise 6D pose.
- FoundationPose, CVPR 2024: novel object 6D pose estimation and tracking, RGB-D, model-based/model-free.
- BundleSDF, CVPR 2023: RGB-D video object-centric tracking, pose graph optimization, neural object field.
- GoTrack, CVPRW 2025: CAD-based 6DoF refinement/tracking, frame-to-frame optical flow reduces jitter and compute.
- Temporally Consistent Object 6D Pose Estimation for Robot Control, RA-L/arXiv 2026: static object inconsistency, factor graph smoothing.
- BOP Challenge 2024/2025: current 6D pose evaluation landscape and SOTA context.
- ROS robot_localization/Nav2 docs: EKF/UKF common practice for smooth real-time pose/state estimation.
- One Euro Filter, CHI 2012 and maintained implementations: low-compute jitter-lag tradeoff for real-time tracking.

## 1. Literature Review Table

| Paper | Year | Venue | Method | Temporal Usage | Additional Compute Cost | Real-time Capability | Improvement Metric | Applicability to SAM-6D |
|---|---:|---|---|---|---|---|---|---|
| Temporally Consistent Object 6D Pose Estimation for Robot Control | 2026 | IEEE RA-L / arXiv | Factor graph smoothing, motion model, uncertainty, outlier rejection | Strong: online temporal optimization | Low-Medium, depends window/object count | Yes for robot control scale | Stability, missing detections, outlier robustness | Very high as post-SAM-6D temporal layer |
| GoTrack: Generic 6DoF Object Pose Refinement and Tracking | 2025 | CVPRW | Flow-based refinement + frame-to-frame flow propagation | Strong: tracking mode | Medium GPU, extra flow/refiner | Intended real-time/efficient | Tracking stability, jitter, occlusion robustness | Medium: stronger if CAD/templates available |
| FoundationPose | 2024 | CVPR | Render-and-compare pose estimation/tracking with neural object representation | Strong: tracking supported | Medium-High GPU | Real-time depends config/GPU | ADD/ADD-S, tracking benchmark | Medium: alternative tracker or reinit module, larger integration |
| BundleSDF | 2023 | CVPR | Object-centric RGB-D tracking + pose graph + neural SDF | Strong: video memory, graph optimization | High, separate neural reconstruction thread | Near real-time, not lightweight | ADD/ADD-S AUC, drift, reconstruction | Low-Medium: too heavy for 5-10% budget |
| SAM-6D | 2024 | CVPR | ISM+PEM zero-shot RGB-D pose, partial-to-partial point matching | None: per-frame | Existing baseline | Depends SAM/FastSAM and PEM | BOP AR, segmentation and pose | Baseline; needs external temporal layer |
| GigaPose | 2024 | CVPR | Fast template-patch correspondence coarse pose | None: single RGB frame | Low-Medium; 35x faster coarse stage vs prior SOTA reported | Yes for coarse pose | BOP AR, robustness to segmentation errors | Low for jitter; useful if replacing coarse estimator |
| GenFlow | 2024 | CVPR | Generalizable recurrent optical-flow pose refinement | Recurrent within frame, not video temporal | High if applied per frame/hypothesis | Often expensive for strict FPS | BOP unseen RGB/RGB-D ranking | Low-Medium; accuracy refiner, not stabilizer |
| FoundPose | 2024 | ECCV | DINOv2 template retrieval + 2D-3D correspondence + featuremetric refinement | None: single RGB frame | Low-Medium coarse; refinement can be heavy | Good for onboarding, per-image | BOP RGB SOTA | Low for jitter; can seed tracking |
| MegaPose | 2022/2023 | CoRL | Render-and-compare novel object pose | None: single frame | Medium-High | Slower with many hypotheses/refinement | BOP/ModelNet/YCB-V accuracy | Low for jitter; frame-wise refiner may fluctuate |
| Shape-Constraint Recurrent Flow | 2023 | CVPR | Recurrent flow with 3D shape constraints | Recurrent refinement, not video temporal | Medium-High | More efficient than unconstrained flow, still NN refiner | Pose accuracy/efficiency | Low-Medium; refines pose but not temporal |
| M3T / RBGT / SRT3D family | 2020-2024 | RA-L/ICRA/library | Model-based contour/region tracking | Strong: tracker state | Low CPU/GPU depending modality | Yes, often 30 Hz class | Tracking success | Medium if CAD and segmentation are reliable |
| MaskUKF | 2021 | Frontiers Robotics AI | Segmentation + depth + serial UKF | Strong: filter state | Low | Real-time | 6D pose and velocity tracking | Medium: older but practical filter design |
| DynamicPose | 2025 | arXiv | 2D tracking + VIO-guided Kalman + adaptive candidates | Strong for fast camera/object | Medium | Claimed real-time | Tracking under fast motion | Low-Medium: overkill for static/slow camera |

## 2. Top-10 Related Papers Summary

1. Temporally Consistent Object 6D Pose Estimation for Robot Control
   - 가장 직접 관련. 정지 객체에서 per-frame CosyPose 결과가 흔들리는 그림과 동기를 제시한다.
   - factor graph로 object motion prior, measurement uncertainty, outlier rejection, symmetry를 처리한다.
   - SAM-6D 후단 post-processing 연구의 가장 좋은 출발점이다.

2. GoTrack
   - frame-to-frame optical flow를 사용해 2D-3D correspondence를 다음 프레임으로 전파한다.
   - 논문에서 compute 절감, occlusion robustness, jitter 감소를 명시한다.
   - SAM-6D를 keyframe detector/reinitializer로 두고 GoTrack류 tracker를 사이 프레임에 쓰는 hybrid 설계가 유망하다.

3. FoundationPose
   - novel object pose estimation과 tracking을 통합한 CVPR 2024 SOTA 계열.
   - RGB-D, CAD 또는 reference images 기반이며, tracking task 자체를 지원한다.
   - SAM-6D 대체 또는 보조 tracker로는 강하지만, 최소 수정/5-10% 비용 조건에는 부담이 있다.

4. BundleSDF
   - RGB-D video에서 unknown object pose tracking과 3D reconstruction을 함께 수행한다.
   - pose graph optimization과 memory frames로 temporal consistency를 확보한다.
   - 좋은 연구 참고지만 SAM-6D 실시간 후처리로 넣기에는 무겁다.

5. SAM-6D
   - 현재 대상 시스템. ISM은 SAM 기반 proposal, PEM은 RGB-D partial-to-partial point matching이다.
   - temporal cue가 없으므로 frame-wise 출력 안정화 layer가 필요하다.

6. GigaPose
   - 빠른 coarse pose estimation과 segmentation error robustness가 장점이다.
   - jitter 논문은 아니지만, raw pose 후보 품질을 높이면 filter가 처리해야 하는 outlier가 줄어든다.

7. GenFlow
   - optical-flow 기반 generalizable pose refinement로 BOP unseen RGB/RGB-D 성능이 강하다.
   - recurrent라는 단어는 video temporal이 아니라 iterative refinement에 가깝다.

8. FoundPose
   - DINOv2 foundation feature로 template retrieval/2D-3D correspondence를 수행한다.
   - featuremetric refinement는 accuracy 향상용이며 temporal stabilization은 없다.

9. MegaPose
   - novel object render-and-compare 계열의 대표 baseline.
   - frame-wise로 돌리면 pose selection/refinement가 프레임마다 다른 local optimum에 가서 jitter가 생길 수 있다.

10. MaskUKF / Kalman-family object pose tracking
   - 최신 SOTA는 아니지만 real-time ROS/robotics 실무에서 구현성이 좋다.
   - SAM-6D 후단에 pose covariance 또는 heuristic measurement noise를 붙일 때 현실적인 기반이다.

## 3. Temporal Consistency 관점: 주요 계열 비교

| Method Family | Temporal Consistency | SAM-6D 적용 판단 |
|---|---|---|
| SAM-6D | 없음. RGB-D 한 프레임마다 ISM+PEM 수행 | 후단 stabilizer 필수 |
| FoundationPose | tracking task 지원, previous pose 기반 refinement 가능 | 성능은 좋지만 구조 변경 큼 |
| BundleSDF | pose graph + memory frame + neural field | 연구 참고용, 실시간 budget 초과 가능성 큼 |
| MegaPose | 없음. render-and-compare single frame | jitter 해결책 아님 |
| GigaPose | 없음. fast single-frame coarse pose | detector/refiner 품질 보강용 |
| FoundPose | 없음. single RGB image template retrieval | keyframe initialization에는 유용 |
| GoTrack | 있음. frame-to-frame flow propagation | Phase 2/3 hybrid tracker로 유망 |
| Factor graph temporal pose | 있음. measurement+motion factor | SAM-6D 후단 논문 주제로 최적 |

## 4. Tracking 기반 접근 vs Frame-wise Pose Estimation

Frame-wise pose estimation은 각 프레임을 독립 문제로 풀기 때문에 detection recall이 높고 실패 후 재초기화가 쉽다. 하지만 정지 객체라는 물리 제약, 저속 카메라 motion의 연속성, 이전 프레임의 correspondence를 쓰지 않으므로 작은 segmentation/depth/noise 차이가 R,t 흔들림으로 바로 나온다.

Tracking 기반 접근은 이전 pose, velocity, correspondence, motion model을 사용한다. 따라서 low-motion/static 환경에서는 훨씬 안정적이고 계산량도 줄일 수 있다. 반면 drift, lost track, occlusion 후 recovery 문제가 있으므로 SAM-6D 같은 frame-wise detector를 keyframe reinitializer로 유지하는 hybrid 구조가 가장 실용적이다.

## 5. Algorithm Comparison

| Technique | 효과 | 계산량 | 장점 | 단점 | 조건 만족 여부 |
|---|---|---:|---|---|---|
| EMA | 정지 jitter 즉시 감소 | O(1), negligible | 구현 가장 쉬움 | lag, velocity 대응 약함 | 매우 적합 |
| One Euro Filter | 저속/정지 jitter 감소 + 움직일 때 lag 완화 | O(1), negligible | AR/interactive tracking에 강함 | 튜닝 필요 | 매우 적합 |
| Kalman Filter | 일정 velocity/constant pose 모델에 좋음 | O(n^3)이나 n 작아 매우 낮음 | covariance/gating 가능 | rotation은 Lie algebra 처리 필요 | 매우 적합 |
| EKF | SE(3), quaternion, nonlinear observation에 적합 | 낮음 | ROS state estimation과 친숙 | Jacobian/튜닝 부담 | 적합 |
| UKF | nonlinear rotation에 안정적 | EKF보다 높지만 낮음 | Jacobian 불필요 | object 수 많으면 부담 증가 | 적합, Phase 2 |
| Pose Graph Optimization | outlier, missing, symmetry 처리 강함 | window에 따라 Low-Medium | 논문 기여도 높음 | solver 통합 필요 | 적합, Phase 2/3 |
| Sliding Window Optimization | local temporal consistency 강함 | window 5-15면 관리 가능 | online 가능 | tuning/latency | 적합, Phase 2 |
| Bundle Adjustment | 정확도/consistency 강함 | 높음 | multi-view 최적화 | 5-10% FPS 조건과 충돌 | 부적합 |
| Factor Graph | measurement+motion+symmetry factor 표현력 높음 | Low-Medium | 논문 주제 최상 | 구현 난이도 중상 | 적합, Phase 3 |
| Differentiable Pose Refinement | per-frame accuracy 향상 | 높음 | raw pose quality 개선 | jitter 직접 해결 아님 | 제한적 |
| Object-centric Tracking | stability와 compute 모두 개선 가능 | Medium | real deployment에 좋음 | lost/reinit 설계 필요 | Phase 2/3 적합 |
| Neural Temporal Fusion | temporal learning 가능 | High | 논문 novelty 가능 | dataset/training/FPS 부담 | 현재 조건 부적합 |
| Temporal Transformer | 장기 temporal context | High | offline/video에 강함 | latency와 training 부담 | 부적합 |
| State Space Model Pose Tracking | 효율적 sequence model 가능성 | Medium-High | 연구 novelty | 6D object pose 적용은 아직 초기 | Phase 3 연구용 |

## 6. ROS2 Real-time Pose Stabilization Practice

ROS 생태계에서 가장 널리 쓰이는 real-time pose smoothing/fusion 패턴은 EKF/UKF 기반 state estimation이다. `robot_localization`은 `PoseWithCovarianceStamped`, `TwistWithCovarianceStamped`, odometry, IMU 등을 받아 EKF/UKF로 smooth state를 만든다. 다만 이는 주로 robot/base pose용이며, object pose stream에는 custom stabilizer node가 더 단순하다.

Object pose에는 다음 조합이 실무적으로 가장 흔하다.

- per-object track ID 유지
- confidence gate와 jump rejection
- EMA/One Euro/Kalman smoothing
- quaternion slerp 또는 SO(3) log/exp 기반 rotation smoothing
- TF publish는 filtered pose, debugging topic은 raw pose 병행
- lost track timeout 후 raw pose로 reinitialize

## 7. SAM-6D Applicability Analysis

SAM-6D 구조를 크게 바꾸지 않는 최적 지점은 ROS2 wrapper 또는 inference output publisher 직전이다. PEM 내부나 ISM scoring을 바꾸면 재학습/검증 비용이 커지고 recall 변경 위험이 있다. 반면 후단 stabilizer는 detection recall을 건드리지 않고, PoseArray/TF만 안정화할 수 있다.

권장 state representation:

- Translation: camera/object frame의 3D vector
- Rotation: quaternion 저장, update는 SO(3) log map 또는 slerp
- Optional velocity: v_t, omega_t
- Measurement covariance: 초기에는 score/depth residual 기반 heuristic, Phase 3에서 PEM correspondence residual로 학습/추정

권장 gating:

- raw confidence가 낮으면 update weight 감소
- translation innovation이 object diameter 대비 과도하면 outlier로 보류
- rotation innovation이 symmetry-equivalent이면 symmetry-aware distance 사용
- stationary detector가 true이면 cutoff/min process noise를 낮춤

## 8. Methods That Satisfy Required Conditions

필수 조건을 모두 만족하는 후보:

1. One Euro Filter on SE(3)
   - Recall 감소 없음, FP 목적 아님, pose accuracy 평균 유지, FPS 영향 거의 없음, SAM-6D 구조 변경 없음.

2. Adaptive EMA + Deadband + Confidence Gate
   - 가장 빠른 구현. 정지 객체 jitter에는 효과가 크지만 움직임 반응은 One Euro보다 약하다.

3. Constant-velocity Kalman/EKF on SE(3)
   - measurement covariance와 innovation gate가 가능해 real robot control에 더 적합하다.

4. Small-window Factor Graph
   - compute를 window 5-10으로 제한하면 가능. 논문 기여도와 robustness가 좋다.

5. SAM-6D keyframe + lightweight object-centric tracking
   - recall은 SAM-6D keyframe에서 유지하고, intermediate frames는 tracking으로 안정화한다. 구현은 중간 이상.

조건에서 제외 또는 후순위:

- Full Bundle Adjustment: compute/latency 위험.
- BundleSDF 전체 통합: 구조 변경과 GPU cost 큼.
- Temporal Transformer/Neural Fusion: training data와 latency 부담.
- Differentiable refiner 추가: accuracy는 오를 수 있으나 FPS 악화와 jitter 직접성 부족.

## 9. Static Industrial Robotics and AR/VR Practice

산업용 로봇에서는 CAD 기반 RGB-D/structured-light pose estimation, PPF/ICP/refinement, confidence gating, temporal filtering, multi-view 또는 fixed camera averaging이 많이 쓰인다. 정지 물체 pick-and-place에서는 pose를 한 번만 쓰거나 여러 프레임 median/mean consensus를 쓰는 경우도 많다. continuous servoing에서는 EKF/UKF 또는 tracker state를 사용한다.

AR/VR에서는 visual-inertial tracking, model/marker tracking, exponential smoothing, One Euro Filter, Kalman/complementary/predictive filters가 흔하다. 사용자는 저속 motion에서 jitter에 민감하고 고속 motion에서 lag에 민감하므로, speed-adaptive filter가 실용적이다.

## 10. Current SOTA Interpretation

문제별 SOTA를 분리해야 한다.

- 정지 객체 temporal consistency 직접 대응: Temporally Consistent Object 6D Pose Estimation for Robot Control.
- novel object RGB-D pose tracking: FoundationPose.
- RGB-D unknown object object-centric tracking/reconstruction: BundleSDF.
- efficient generic tracking/refinement with explicit jitter reduction: GoTrack.
- BOP single-frame unseen pose accuracy: GigaPose+GenFlow, FoundPose+FeatRef, FoundationPose/SAM-6D 계열.

SAM-6D jitter 문제에는 single-frame BOP SOTA보다 temporal consistency SOTA가 더 중요하다.

## 11. Implementation Difficulty Ranking

1. Adaptive EMA on translation + quaternion slerp: 0.5-1 day.
2. One Euro Filter on translation + SO(3): 1-2 days.
3. Kalman filter with constant pose/velocity: 2-4 days.
4. EKF/UKF with SE(3) state and gating: 1-2 weeks.
5. Small-window factor graph with GTSAM/Ceres: 2-4 weeks.
6. SAM-6D keyframe + GoTrack/FoundationPose-style tracker: 1-2 months.
7. BundleSDF-style object-centric neural tracking: 2-4 months.
8. Neural temporal fusion/transformer/SSM: 3-6+ months plus dataset.

## 12. Expected Pose Jitter Reduction Ranking

1. Small-window factor graph with uncertainty/outlier/symmetry handling.
2. Object-centric frame-to-frame tracking with SAM-6D reinitialization.
3. EKF/UKF with measurement covariance and innovation gate.
4. One Euro Filter on SE(3).
5. Adaptive EMA/deadband.
6. Per-frame differentiable refinement.
7. Full BundleSDF/BA, only if latency budget is relaxed.
8. Neural temporal transformer/SSM, high potential but not immediate.

## 13. Recommended Development Roadmap

### Phase 1: Immediate Low-Risk Method

Goal: SAM-6D recall/accuracy pipeline은 그대로 유지하고 published pose만 안정화.

- Add `sam6d_pose_stabilizer` ROS2 node.
- Subscribe raw `PoseArray` and object IDs/scores if available.
- Publish `PoseArrayFiltered` and filtered TF.
- Keep raw topics for debug.
- Implement One Euro Filter:
  - translation: per-axis One Euro.
  - rotation: SO(3) log vector or quaternion slerp with adaptive alpha.
  - stationary mode: measured velocity below threshold for N frames이면 cutoff 감소.
- Add innovation gate:
  - large jump is not discarded immediately; hold previous filtered pose for 1-3 frames unless repeated.
  - this preserves recall and avoids FP filtering objective.
- Metrics:
  - static translation std in mm.
  - static rotation geodesic std in degree.
  - response lag under slow camera motion.
  - raw vs filtered BOP-style ADD/ADD-S if ground truth exists.

### Phase 2: Medium-Scale Structure Improvement

Goal: filter에서 tracker/smoother로 확장.

- Implement constant-velocity EKF/UKF on SE(3).
- Estimate measurement covariance from:
  - SAM-6D PEM matching score/residual if accessible.
  - depth consistency between rendered/projected model and observed depth.
  - mask IoU/proposal score.
- Add small sliding window optimizer:
  - variables: object poses over last W frames.
  - factors: SAM-6D measurement, constant-pose/constant-velocity prior, camera motion prior if available, depth reprojection residual.
  - W=5-10 to keep FPS impact small.
- Optional: use SAM-6D every frame at first, then keyframe only if tracker confidence is high.

### Phase 3: Thesis-Level Novel Method

Proposed thesis topic:

"Uncertainty-Aware Temporal Pose Consistency for Zero-Shot RGB-D 6D Object Pose Estimation"

Core contribution:

- SAM-6D PEM correspondence residual and ISM geometry score를 measurement uncertainty로 변환.
- Symmetry-aware SE(3) factor graph or Lie-EKF.
- Static/low-motion detector that adapts process noise.
- Recall-preserving fallback: raw SAM-6D detections remain available; temporal layer only stabilizes pose state.
- Evaluation protocol focused on jitter, not just ADD/ADD-S:
  - static pose variance.
  - short-term Allan variance style drift.
  - control-loop stability metric.
  - latency and FPS overhead.

## 14. Final Recommendations

### A. 가장 빠르게 구현 가능

One Euro Filter + confidence/innovation gate + stationary mode. SAM-6D 코드는 거의 건드리지 않고 ROS2 후단 node로 구현한다.

### B. 석사 논문 수준 신규성 확보 가능

SAM-6D-specific uncertainty-aware factor graph smoothing. 핵심은 PEM/ISM 내부 신뢰도를 temporal optimizer의 measurement covariance로 해석하는 것이다.

### C. 실제 성능 향상 가능성 최대

Phase 1 filter로 즉시 안정화한 뒤, Phase 2에서 EKF/UKF 또는 sliding-window factor graph를 붙인다. 이후 필요하면 GoTrack/FoundationPose류 frame-to-frame tracker를 SAM-6D keyframe reinitializer와 결합한다.

### D. SAM-6D 코드 변경 최소

ROS2 output-level stabilizer. 입력은 raw PoseArray/TF, 출력은 filtered PoseArray/TF. SAM-6D inference path, ISM, PEM, score threshold는 그대로 둔다.

## 15. If Implementing Tomorrow

내일 바로 구현한다면 다음 하나를 적용한다.

**SE(3) One Euro Filter post-processing node**

- translation: per-axis One Euro.
- rotation: quaternion log map 또는 slerp 기반 adaptive low-pass.
- object ID별 filter state 유지.
- confidence가 낮거나 innovation이 큰 pose는 update weight를 낮춘다.
- static detector가 true이면 min cutoff를 낮춰 jitter를 더 줄인다.
- raw output과 filtered output을 모두 publish한다.

이 방법은 recall을 줄이지 않고, FP 제거 목적도 아니며, SAM-6D 구조를 바꾸지 않고, FPS 영향이 거의 없다.

## 16. If Writing a Master's Thesis

석사 논문이라면 다음을 연구한다.

**SAM-6D Measurement-Uncertainty-Aware Temporal Factor Graph**

- SAM-6D의 ISM score, PEM matching confidence, depth residual, mask geometry consistency를 uncertainty로 모델링.
- SE(3) motion prior와 static-object prior를 adaptive하게 선택.
- symmetry-aware rotation distance를 factor에 반영.
- online fixed-lag smoothing으로 실시간성 유지.
- jitter reduction, control stability, latency, BOP AR 유지 여부를 함께 평가.

이 방향은 단순 filter보다 신규성이 있고, FoundationPose/BundleSDF처럼 구조 전체를 바꾸지 않으면서 SAM-6D의 약점인 temporal consistency를 직접 보완한다.

## References

- SAM-6D: https://openaccess.thecvf.com/content/CVPR2024/papers/Lin_SAM-6D_Segment_Anything_Model_Meets_Zero-Shot_6D_Object_Pose_Estimation_CVPR_2024_paper.pdf
- FoundationPose: https://arxiv.org/html/2312.08344
- BundleSDF: https://bundlesdf.github.io/
- BundleSDF CVPR paper: https://openaccess.thecvf.com/content/CVPR2023/papers/Wen_BundleSDF_Neural_6-DoF_Tracking_and_3D_Reconstruction_of_Unknown_Objects_CVPR_2023_paper.pdf
- Temporally Consistent Object 6D Pose Estimation for Robot Control: https://arxiv.org/html/2605.02708v1
- Project page for TemporalPose: https://data.ciirc.cvut.cz/public/projects/2024TemporalPose/
- GoTrack: https://arxiv.org/html/2506.07155v1
- GigaPose: https://arxiv.org/html/2311.14155
- FoundPose: https://arxiv.org/html/2311.18809v2
- GenFlow: https://arxiv.org/abs/2403.11510
- Shape-Constraint Recurrent Flow: https://arxiv.org/abs/2306.13266
- BOP Challenge: https://bop.felk.cvut.cz/challenges/
- ROS robot_localization docs: https://docs.ros.org/en/melodic/api/robot_localization/html/index.html
- ROS state estimation nodes: https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
- Nav2 robot_localization smoothing guide: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html
- One Euro Filter: https://gery.casiez.net/1euro/
- One Euro Filter paper: https://direction.bordeaux.inria.fr/~roussel/publications/2012-CHI-one-euro-filter.pdf
