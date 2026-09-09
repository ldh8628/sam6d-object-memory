# SAM-6D Pose Estimation — 근본 원인 분석 (False Positive & Pose Jitter)

> 분석 일자: 2026-06-08 · 분석 범위: 실제 라이브 실행 경로 (ROS 노드) · **코드 미수정, 분석 전용**
> 모든 근거는 실제 파일의 `파일:라인`으로 제시. 추측 배제.

---

## 현재 파이프라인 구조

### 실제 실행 진입점 (Live 경로)

라이브 시스템의 진입점은 BOP 벤치마크용 `run_inference_custom.py`가 **아니라** ROS 노드이며, 백엔드는 `run_batch_inference_fast.py`다. (조사 시 가장 흔한 함정: 두 경로의 시드/필터링 동작이 다름.)

```
[RGB topic] + [Depth topic]
   └─ ApproximateTimeSynchronizer  (src/sam6d_ros/sam6d_ros/sam6d_inference_node.py:305-310)
        └─ _synced_image_cb        (node:439-450)   ── 프레임마다 워커 스레드 1개
             └─ _run_inference_from_images (node:452)
                  └─ _run_inference (node:763)
                       ├─ import run_batch_inference_fast as rb   (node:764)
                       ├─ rb.run_ism_single(...)  (node:789)  ── 분할/검출/스코어 → detection_ism.json
                       └─ rb.run_pem_single(...)  (node:810)  ── pose 추정 → poses[]
                  └─ _publish_poses (node:898) / _publish_pose_transforms (node:921)  ── 무조건 publish
```

| 단계 | 위치 (파일:라인) | 핵심 함수/클래스 |
|---|---|---|
| 입력 동기화 | `sam6d_inference_node.py:305-310` | `ApproximateTimeSynchronizer` |
| 프레임 콜백 | `sam6d_inference_node.py:439-450` | `_synced_image_cb` |
| 모델 로드(1회) | `sam6d_inference_node.py:329-359` | `_load_models` |
| ISM 분할/검출 | `run_batch_inference_fast.py:358` | `run_ism_single` |
| ISM 스코어 | `Instance_Segmentation_Model/model/detector.py:260-322` | `compute_semantic_score` / `compute_appearance_score` / `compute_geometric_score` |
| ISM→PEM 전달 | `run_batch_inference_fast.py:426` | `detection_ism.json` (파일 IPC) |
| PEM 데이터 준비 | `run_batch_inference_fast.py:539` | `_get_pem_test_data` |
| PEM 추론 | `run_batch_inference_fast.py:637` | `run_pem_single` → `Net.forward` |
| Feature 추출 | `Pose_Estimation_Model/model/pose_estimation_model.py:24` | `ViTEncoder` |
| Coarse matching | `Pose_Estimation_Model/model/coarse_point_matching.py:38-81` | `CoarsePointMatching` → `compute_coarse_Rt` |
| Fine matching | `Pose_Estimation_Model/model/fine_point_matching.py:39-86` | `FinePointMatching` → `compute_fine_Rt` |
| pose score | `Pose_Estimation_Model/utils/model_utils.py:277-285` | (R,t, pose_score) |
| 결과 publish | `sam6d_inference_node.py:898 / 921` | `_publish_poses` |

### 적용 중인 임계값/설정 (실제 값)

| 파라미터 | 값 | 출처 |
|---|---|---|
| `det_score_thresh` | **0.2** | `params.yaml:48`, `run_batch_inference_fast.py:763` |
| `confidence_thresh` (semantic) | **0.2** | `Instance_Segmentation_Model/configs/model/ISM_fastsam.yaml:25` |
| `stability_score_thresh` | 0.97 | `params.yaml`, FastSAM 마스크 품질용 (검출 신뢰도 게이트 아님) |
| `top_k_for_pem` | 3 (`params.yaml`) / 5 (`no_cli_milk.yaml`) | config |
| `n_sample_observed_point` | 2048 | `Pose_Estimation_Model/config/base.yaml:62` |
| `rd_seed` | 1 (**라이브 경로에서 미적용**) | `base.yaml:102` |
| coarse `dis_thres` | 0.15 | `model_utils.py` / `base.yaml:40` |
| `ism_rerank.enabled` | **False** (rerank 비활성, `min_score:0.45`는 死코드) | `sam6d_inference_node.py:85-89` |

---

## 문제 A 원인 분석 — 객체 부재 시 False Positive

### 핵심 결론
파이프라인 전체에 **"객체 없음(no-object) 판정 로직이 존재하지 않는다."** 유일한 필터 두 개가 모두 약한 단일 임계값(0.2)이며, 그마저도 **최종 신뢰도가 아닌 중간 스코어**에만 걸려 있다. 검출이 비면 출력이 없지만, 검출이 하나라도 0.2를 넘으면 pose가 무조건 발행된다.

### 원인 후보 (우선순위 순)

**[P1] 발행 직전 최종 신뢰도 게이트(pose score floor) 부재 — 가장 직접적 원인**
- pose score는 계산되지만(`model_utils.py:281-283`) 어떤 임계값과도 비교되지 않는다.
- 최종 점수는 단순히 `pose_scores = pred_pose_score * ISM_score`로 만들어 **모든 검출에 무조건 기록**된다.
  ```python
  # run_batch_inference_fast.py:680-689
  pose_scores = (out["pred_pose_score"] * out["score"]).detach().cpu().numpy()
  ...
  for idx, det in enumerate(detections):
      detections[idx]["score"] = float(pose_scores[idx])   # floor 없음
  ```
- 노드의 `_publish_poses`(`sam6d_inference_node.py:898`)는 `poses`를 **조건 없이** 순회 발행. score 검사 없음.

**[P2] ISM 게이트가 semantic(DINOv2 CLS 코사인) 단일 모달 0.2뿐 — 너무 낮음**
- ISM에서 후보를 거르는 유일 지점:
  ```python
  # Instance_Segmentation_Model/model/detector.py:286-288
  idx_selected_proposals = torch.arange(...)[score_per_proposal > self.matching_config.confidence_thresh]
  ```
- `confidence_thresh = 0.2`(`ISM_fastsam.yaml:25`)는 **코사인 유사도 0.2**라는 매우 낮은 기준. CAD 객체가 1종뿐이라 `torch.max`/`best_template`이 항상 "가장 비슷한 템플릿"을 골라, 배경 영역도 약한 유사도(>0.2)면 통과.
- appearance/geometric 점수는 `final_score`를 **재가중만** 할 뿐 후보를 탈락시키지 못함:
  ```python
  # run_batch_inference_fast.py:416-419 (combined score, 하지만 임계 비교 없음)
  final_score = (semantic_score + appe_scores + geometric_score*visible_ratio) / (1 + 1 + visible_ratio)
  ```

**[P3] PEM 입구 필터도 동일한 0.2 (ISM score 재사용)**
```python
# run_batch_inference_fast.py:544-548
dets = [d for d in dets_ if d["score"] > det_score_thresh]   # det_score_thresh = 0.2
if not dets:
    return None, None, None, None, None
```
- 비면 None 반환(검출 0개일 때만 무출력). 0.2 초과 1개라도 있으면 그대로 pose 추정 진행.

**[P4] FastSAM은 항상 영역 제안을 생성**
- `fast_sam.py:39`에서 `conf=0.25`로 하드코딩(config의 0.05 무시). FastSAM은 장면에 대해 항상 다수의 마스크를 만들고, 그 마스크들이 단일 CAD와의 약한 유사도 게이트만 통과하면 검출로 살아남음. "대상 없음"을 표현할 출력 자체가 없음.

### 근거 코드 요약
- 게이트 부재: `run_batch_inference_fast.py:680-689`, `sam6d_inference_node.py:898`
- 약한 단일 게이트: `detector.py:286-288`, `run_batch_inference_fast.py:544-548`
- pose score 계산(하지만 미사용): `model_utils.py:281-283`

---

## 문제 B 원인 분석 — 고정 카메라/정지 장면에서 프레임마다 pose 변동

### 핵심 결론
라이브 경로(`run_batch_inference_fast.py`)는 **원본 SAM-6D가 수행하던 시드 설정을 전부 누락**했고(`test_bop.py:200-201`, `run_inference_custom.py:264-265`에는 존재), 동시에 **추론 경로 내부에 프레임마다 실행되는 무시드 랜덤 연산이 2개** 존재한다. 또한 **temporal filtering/스무딩/트래킹이 전혀 없어** 변동이 그대로 출력된다.

### 원인 후보 (우선순위 순)

**[P1] 관측 포인트클라우드 무시드 랜덤 서브샘플링 — 프레임마다 실행**
```python
# run_batch_inference_fast.py:587-594
n = cfg.n_sample_observed_point          # 2048
idx = (np.random.choice(len(choose), n) if len(choose) <= n
       else np.random.choice(len(choose), n, replace=False))
choose = choose[idx]; cloud = cloud[idx]
```
- 동일 depth라도 매 프레임 **다른 2048점 부분집합**이 PEM 입력으로 들어감 → 다른 R,t.
- `np.random`은 라이브 경로 어디에서도 시드되지 않음(아래 P3).

**[P2] Coarse matching의 RANSAC식 무시드 포즈 가설 샘플링 — 프레임마다 실행**
```python
# Pose_Estimation_Model/utils/model_utils.py:221
idx = torch.searchsorted(cumsum_weights, torch.rand(B, n_proposal1*3, device=device))
```
- `compute_coarse_Rt`는 `torch.rand`로 **수천 개(n_proposal1≈6000) 무작위 대응 3쌍**을 뽑아 Procrustes로 풀고 best 1개를 선택(`model_utils.py:219-247`)하는 확률적 추정기.
- `torch`는 라이브 경로에서 한 번도 `manual_seed`되지 않아 RNG 상태가 프레임마다 진행 → 동일 입력에도 다른 `init_R/init_t` → fine 단계까지 전파.

**[P3] 시드/결정성 설정 전면 부재 (라이브 경로)**
- `run_batch_inference_fast.py` 전체: `manual_seed`, `np.random.seed`, `use_deterministic_algorithms`, `cudnn.deterministic` **0건** (grep 확인).
- 원본 진입점에는 존재하나 라이브가 미사용: `test_bop.py:200-201`, `Pose_Estimation_Model/run_inference_custom.py:264-265` (`random.seed`/`torch.manual_seed`).
- 참고: `pointnet2_modules.py:217`의 `torch.manual_seed(1)`은 `if __name__=="__main__"` 셀프테스트 내부 — **임포트 실행 시 미실행**(死코드). FPS 자체는 입력 순서에 의존 → P1의 무작위 부분집합 영향으로 간접 변동.

**[P4] Temporal filtering / smoothing / tracking 전무**
- 노드에 EMA/Kalman/평균화/프레임 간 pose 연관 **부재**(`sam6d_inference_node.py` 전반). 각 프레임은 독립 처리되어 raw pose가 그대로 발행됨 → P1·P2의 변동이 출력에 1:1 노출.

**[P5] GPU/cuDNN 부동소수점 비결정성 — 부차적**
- `cudnn.deterministic` 미설정. 점수 차가 0.2 게이트·argmax 근처에서 작을 때 선택이 뒤집혀 변동 가중(2차 요인).

**[P6] depth noise / mesh.sample — 영향 제한적**
- `mesh.sample(2048)`은 시작 시 1회만(`run_batch_inference_fast.py:267-268, 482-483`) → **프레임 간 변동 아님**(프로세스 재시작 시에만 다름).
- 실제 센서 depth noise가 있으면 P1과 결합해 변동을 키우나, P1·P2가 제거돼도 잔존하는 물리적 요인이며 1차 원인은 아님.

### 근거 코드 요약
- 무시드 랜덤(프레임당): `run_batch_inference_fast.py:587-594`, `model_utils.py:221`
- 시드 부재: 라이브 파일 grep 0건 / 원본 `test_bop.py:200-201`
- 필터 부재: `sam6d_inference_node.py` 전반(EMA/tracking 없음)

---

## 추가로 필요한 Debug 데이터

원인을 정량 확정하려면(코드 수정 없이도 가능한 로깅 기준) 다음을 수집 권장:

### CSV — 프레임 단위 안정성 측정
- `frame_idx, n_proposals, n_pass_semantic(>0.2), n_pass_det(>0.2)` — 객체 부재 프레임에서도 검출이 살아남는지 (문제 A).
- `frame_idx, best_semantic, best_appearance, best_geometric, visible_ratio, ISM_final_score, pred_pose_score, final_score` — "객체 있음 vs 없음" 프레임의 점수 분포 분리도 측정 → 적정 floor 임계값 도출.
- `frame_idx, pred_t_x/y/z, pred_R_quat(x,y,z,w), pose_score` — 정지 장면 N프레임 연속 기록 → translation/rotation 표준편차로 jitter 정량화 (문제 B).

### JSON — 원인 격리 실험 로그
- 동일 입력 1장을 N회 반복 추론한 결과(`detection_ism.json` + pose). 입력이 100% 동일한데 결과가 흔들리면 P1/P2(랜덤성) 확정, 안 흔들리면 depth noise 쪽.
- `np.random.choice`로 뽑힌 `idx` 집합과 coarse `torch.rand` 가설 수를 프레임별로 덤프 → 어느 랜덤 소스가 지배적인지 분리.

### 시각화 결과
- 객체 부재 장면에 대한 ISM 마스크 + semantic score 오버레이 (이미 `dump_ism_mask_candidates.py`, `check_semantic_score.py` 존재 → 활용 가능).
- 정지 장면 N프레임의 pose 축(axis) 오버레이 누적 이미지 → 흔들림 폭 육안 확인.
- pose_score vs (객체 있음/없음) 히스토그램 → floor 결정 근거.

---

## 최종 결론 — 가장 가능성 높은 원인 TOP 5

| 순위 | 문제 | 원인 | 근거 (파일:라인) |
|---|---|---|---|
| **1** | A | **발행 직전 최종 신뢰도(pose score) floor 부재** — pose_score 계산만 하고 어떤 임계와도 비교 없이 무조건 발행 | `run_batch_inference_fast.py:680-689`, `sam6d_inference_node.py:898`, `model_utils.py:281-283` |
| **2** | B | **관측 포인트클라우드 무시드 `np.random.choice` 서브샘플링** (프레임마다 다른 2048점) | `run_batch_inference_fast.py:587-594` |
| **3** | B | **Coarse matching의 무시드 `torch.rand` RANSAC 포즈 가설 샘플링** + 라이브 경로 시드 전면 부재 | `model_utils.py:221`, `run_batch_inference_fast.py`(seed 0건), cf. `test_bop.py:200-201` |
| **4** | A | **유일한 게이트가 semantic 단일 모달 코사인 0.2로 너무 낮음** — 단일 CAD라 항상 best 매칭, 배경도 통과 | `detector.py:286-288`, `run_batch_inference_fast.py:544-548` |
| **5** | B | **Temporal filtering/스무딩/트래킹 전무** — 1·2번 변동이 출력에 그대로 노출 | `sam6d_inference_node.py` 전반 (EMA/Kalman/tracking 부재 확인) |

### 한 줄 요약
- **문제 A**: 시스템에 "객체 없음" 개념과 최종 신뢰도 floor가 없다. 검출이 비지만 않으면 항상 pose를 낸다.
- **문제 B**: 라이브 경로가 원본의 시드 설정을 누락했고, 추론 내부에 프레임마다 실행되는 무시드 랜덤(포인트 샘플링 + RANSAC 포즈 가설)이 살아있으며, 이를 완충할 temporal filter가 없다.

> ⚠️ 본 문서는 분석 전용이며 코드를 수정하지 않았다. 수정 방향(예: pose_score floor 추가, `np.random.seed`/`torch.manual_seed` 적용, EMA/tracking 도입)은 다음 단계에서 별도 결정.
