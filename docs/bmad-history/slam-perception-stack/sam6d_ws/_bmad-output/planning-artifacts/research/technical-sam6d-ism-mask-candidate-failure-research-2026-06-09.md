---
stepsCompleted: [1]
inputDocuments: ['temp.md', 'sam6d_debug.csv', 'frame_results.csv', 'visibility_labels.csv']
workflowType: 'research'
research_type: 'technical'
research_topic: 'SAM-6D ISM Mask Candidate Failure Analysis (FP/FN)'
research_goals: 'pose-gate 튜닝 중단, ISM mask proposal/scoring/render-feature 품질로 FP/FN 근본 재분석'
user_name: 'Ldh9501'
date: '2026-06-09'
web_research_enabled: true
source_verification: true
---

# Technical Research — SAM-6D ISM Mask Candidate Failure Analysis

> **범위**: 이 보고서는 SAM-6D의 ISM(Instance Segmentation Model) **후보 생성·랭킹 단계**를 코드 기준으로 재분석한다. workspace z gate / depth agreement gate는 "pose plausibility 후처리"로만 별도 취급하며, 본 연구의 결론 근거에서 제외한다.
> **방법론**: 코드 주장은 `file:line`, 실험 주장은 `outputs/validation/**` 로그, 논문 주장은 검증된 URL에 각각 앵커링. 코드 미수정·git 미작업.
> **검증 상태**: ★ = 본 세션에서 직접 명령으로 확인한 사실, ☆ = 서브에이전트 코드리딩 기반(미직접확인). 주요 ★ 항목은 §8 부록에 명령 근거.

---

## 1. 결론 요약

### 1.1 1차 원인 판단 — pose 후처리인가, ISM 후보 생성/랭킹인가?

**1차 원인은 ISM 후보 생성/랭킹이다. pose 후처리(workspace z gate)는 증상 억제일 뿐이다.** 근거:

- **ISM 단계에는 결합 점수(final_score)에 대한 절대 임계값이 없다.** ISM은 (1) semantic score > `confidence_thresh=0.2` 게이트와 (2) `nms_thresh=0.25` NMS만 적용하고, **결합 점수 final_score에는 재임계가 없이 NMS 통과분을 전부 출력**한다 (`run_inference_custom.py:201-210`, `model/detector.py:286-288, 384-390`). 즉 ISM은 구조적으로 "항상 무언가를 출력"하는 forced-argmax/forced-output 설계다. 이것이 no-object 프레임 FP의 구조적 뿌리다.
- 후속 게이트(`det_score_thresh=0.2` @ `run_batch_inference_fast.py:570`, `ism_score_min=0.3`/`pose_score_min=0.3` post-pose @ `sam6d_inference_node.py:898-899`, workspace z_max=1500 post-pose @ `:1019-1026`)는 전부 **ISM이 잘못 살린 후보를 사후에 제거**하는 구조다. temp.md의 실측이 이를 입증: workspace-only가 FP −83을 만들지만 이는 `pose_z` 타당성 필터일 뿐 ISM 후보 품질을 바꾸지 않으며, depth agreement는 recall을 0.822→0.500으로 파괴한다.
- 따라서 "publish 직전 제거"가 아니라 "처음부터 올바른 후보를 남기고 잘못된 후보를 낮은 점수로 보내는 것"이 본 연구 방향이며, 이는 코드 구조상 **ISM의 semantic/appearance/geometry 점수 산정과 임계 설계**에서만 해결 가능하다.

### 1.2 RGB/Texture 정보가 정상 사용되는지 — 잠정 판단

**RGB/texture 파이프라인은 end-to-end로 정상 동작한다. "render가 silhouette/constant color라서 appearance가 죽었다"는 가설은 기각된다.** 근거(★ 직접확인):

- 런타임 CAD = `data/cad/Milk_color_uv_vertexcolor_195mm.ply`, PLY 헤더에 `property uchar red/green/blue/alpha` 존재 → **vertex color 보유**.
- 런타임 템플릿 = `Milk_scaled_195mm/templates/rgb_*.png`, 42장, **각 113–137 KB의 512×512 RGB PNG** → 균일색이면 불가능한 파일 크기. 실제 색/텍스처가 들어 있다. (서브에이전트가 1.7KB라 한 것은 `mask_*.png`를 RGB로 오인한 것 — 정정함.)
- DINOv2 입력 정규화는 ImageNet `mean/std`로 **3채널 RGB 보존**, grayscale 변환 없음 (`model/dinov2.py:115-120`; `run_inference_custom.py:131-139`에서 `convert("RGB")` 후 mask 곱).

**단, 이것이 "appearance score가 충분히 변별력 있다"를 의미하지 않는다.** milk carton은 본질적으로 흰색/저텍스처 평면 객체이고, appearance score는 patch-cosine의 query-patch 평균(`model/loss.py:52-62`)이라 흰 배경/흰 벽과의 변별력이 약하다. 즉 **문제는 "RGB가 안 들어간다"가 아니라 "흰 물체라 RGB가 들어가도 구별이 안 된다 + 점수 임계가 미보정"이다.** (단 1개 caveat: §4.5의 CSV `ism_score` 컬럼 의미 검증 필요.)

### 1.3 바로 다음에 해야 할 실험 3개

이미 존재하는 계측 자산(`sam6d_debug.csv`, per-candidate semantic/appearance/geometry/ism score 1522행 + GT 라벨) 덕분에 **상위 2개는 코드 수정 0으로 즉시 가능**하다.

1. **[P0, 코드수정 0] ISM score 분포 분석** — `outputs/validation/SLAM_with_milk_nomilk/_run/debug/sam6d_debug.csv`(per-candidate sem/appe/geo/ism)를 `visibility_labels.csv`(frame→milk_visible 0/1)와 조인하여 TP/FP/TN/FN별 4개 score 분포를 그린다. "어떤 score 항목이 FP를 올리고 FN을 낮추는가"를 pose가 아닌 **ISM 단계 score**로 판정. (§7-실험3)
2. **[P0, 코드수정 0] Negative-frame calibration 가능성 점검** — no-object 프레임의 top-1 candidate의 semantic/appearance score 분포와, object-present 프레임의 top-1 분포를 비교. 단일 임계 대신 top1–top2 margin, semantic-vs-appearance 일치도로 분리 가능한지 확인. (§7-실험4)
3. **[P0→P1] Render-RGB & 실제 crop 나란히 덤프** — `dump_ism_mask_candidates.py`를 확장(또는 신규 분석 스크립트)하여 no-object/object-present 각 프레임의 top-K 후보 crop과 매칭된 best template(rgb_N.png)을 나란히 저장, "흰 배경 latch" 패턴을 시각 확인. (§7-실험1·2)

---

## 2. SAM-6D ISM 파이프라인 코드 분석

### 2.1 Call Graph (forward pass, 순서대로)

진입점 `run_inference_custom.py:226 run_inference()` 기준 (런타임 노드는 `run_batch_inference_fast.run_ism_single()` 경유로 동일 모델 호출 — `sam6d_inference_node.py:411-416, 1341-1363`):

```
run_inference() [run_inference_custom.py]
 ├─(1) segmentor_model.generate_masks(rgb)              # mask proposal 생성
 │       ├─ SAM:     model/sam.py:103 CustomSamAutomaticMaskGenerator.generate_masks
 │       │            └─ _generate_masks → generate_crop_boxes → _process_crop → batched_nms(crop_nms_thresh)
 │       └─ FastSAM: model/fast_sam.py:113 FastSAM.generate_masks  ← ★런타임 기본(params.yaml:44 = fastsam)
 │                    └─ YOLO predict → masks/boxes (conf=0.05, iou=0.9, max_det=200)
 ├─(2) Detections(detections)                            # model/utils.py:70 래핑
 ├─(3) descriptor_model.forward(rgb, detections)         # model/dinov2.py:228 CustomDINOv2.forward
 │       └─ CropResizePad → ViT(is_training=True) → (cls_features, patch_features), ImageNet norm
 ├─(4) compute_semantic_score(query_desc)                # model/detector.py:260
 │       ├─ matching_config.metric → loss.py:27 PairwiseSimilarity (cosine, clamp[0,1])
 │       ├─ aggregation "avg_5": topk(k=5).mean   (detector.py:273-277)  ← N_obj 템플릿 중 top5 평균
 │       ├─ max over objects → score_per_proposal          (detector.py:282-284)
 │       └─ ★GATE: keep score_per_proposal > confidence_thresh(0.2) (detector.py:286-288)
 ├─(5) detections.filter(idx_selected)                   # 게이트 탈락분 영구 제거 (utils.py:173)
 ├─(6) compute_appearance_score(...)                     # detector.py:298
 │       └─ loss.py:52 MaskedPatch_MatrixSimilarity.compute_straight (patch dot, max over ref, mean over query)
 ├─(7) project_template_to_image(...)                    # detector.py:209 (best template pose로 PC 투영)
 ├─(8) compute_geometric_score(...)                      # detector.py:310
 │       ├─ loss.py:64 compute_visible_ratio(thred=0.5)
 │       └─ utils.py:197 compute_iou(projected_bbox, proposal_bbox)
 ├─(9) final_score = (sem + appe + geo*vr)/(1+1+vr)      # ★run_inference_custom.py:201 / detector.py:384
 ├─(10) apply_nms_per_object_id(nms_thresh=0.25)         # detector.py:388-390  ← 결합점수 재임계 없음
 └─(11) save .npz + detection_ism.json                   # utils.py:138-158 (RLE seg, bbox, score)
```

### 2.2 입력/출력

- **입력**: RGB(+depth는 geometry score 투영에만), 사전 렌더 템플릿 42뷰(`rgb_N.png`/`mask_N.png`/`xyz_N.npy`), 사전 추출 ref descriptors(`descriptors`, `appe_descriptors`).
- **출력**: `detection_ism.json` — `[{scene_id, image_id, category_id, bbox[xywh], score(final_score), time, segmentation{size,counts(RLE)}}]`. ★ no-object/전부탈락 시 빈 리스트. PEM은 이 중 `score>det_score_thresh(0.2)`만, top_k=3까지 사용 (`run_batch_inference_fast.py:434, 568-570`).

### 2.3 Score 계산 흐름 (pseudo-code)

```python
# (A) Semantic — DINOv2 cls-token cosine, object/template 집계
sim[p, o, t] = clamp(cosine(q_cls[p], ref_cls[o, t]), 0, 1)          # loss.py:27-44
sem_oa[p, o] = mean(topk(sim[p, o, :], k=5))                          # "avg_5"  detector.py:273-277
sem[p]       = max_o sem_oa[p, o];  obj[p] = argmax_o                 # detector.py:282-284
KEEP p iff sem[p] > 0.2                                               # ★유일한 진짜 게이트  :286-288

# (B) Appearance — DINOv2 patch-token, masked patch matching
S[p] = matmul(q_patch[p], ref_patch[obj,tmpl].T)                     # loss.py:52-62
appe[p] = sum_j( max_k S[p][j,k] ) / (#nonzero query patch + 1e-6)   # query-patch 평균, clamp[0,1]

# (C) Geometry — 투영 bbox IoU × visible_ratio
vr[p]  = #(patch match > 0.5) / #(any match)                         # loss.py:64-76 (thred=0.5)
geo[p] = IoU(proj_bbox[p], proposal_bbox[p])                         # utils.py:197-222

# (D) 결합 — 가중치 하드코딩 (sem:1, appe:1, geo:vr)
final[p] = (sem[p] + appe[p] + geo[p]*vr[p]) / (2 + vr[p])           # ★:201/:384  재임계 없음
```

### 2.4 mask filtering / NMS 흐름

| 단계 | 위치 | 동작 | 임계(런타임값) |
|---|---|---|---|
| segmentor 내부 (FastSAM) | fast_sam.yaml | YOLO conf/iou/max_det | conf 0.05 / iou 0.9 / max_det 200, **stability_score_thresh=0.97**(params.yaml:45) |
| segmentor 내부 (SAM) | sam.py:134-144 | crop 간 batched_nms | box_nms 0.7, pred_iou 0.88, stability 0.85(config 기본) |
| 후보 면적 사전필터 | run_batch_inference_fast.py:386-390 | 면적 top-k만 DINOv2로 | `max_proposals=50` (params.yaml:50) |
| 소형 후보 제거 | detector.py:352-354 | remove_very_small_detections | min_box 0.05, min_mask 3e-4 |
| **semantic 게이트** | detector.py:286-288 | `sem>0.2`만 통과 | **confidence_thresh=0.2** |
| 최종 NMS | detector.py:388-390 | object_id별 NMS | nms_thresh 0.25 |
| (결합점수 임계) | — | **없음** | — |

> **주의**: 런타임은 FastSAM(`segmentor_model=fastsam`)을 쓰며 `stability_score_thresh=0.97`로 매우 높다. 이는 흔들리는/경계 모호한 마스크를 강하게 제거 → 흰 milk carton의 약한 경계 마스크가 통째로 탈락(FN)할 수 있는 지점. SAM용 `ISM_sam.yaml` 값(0.85 등)과 혼동 금지.

---

## 3. FP/FN Failure Mode 분석

| Failure | 관측 패턴 | 가능한 원인(코드/근거) | 확인 방법 | 우선순위 |
|---|---|---|---|---|
| **FP-1 forced output** | no-object 프레임에서 후보가 publish | 결합 final_score에 절대임계 없음. semantic 0.2만 넘기면 NMS 후 전량 출력 (`detector.py:384-390`) | sam6d_debug.csv에서 GT=0 프레임 top-1 ism_score 분포 확인 (실험4) | **P0** |
| **FP-2 흰배경 latch** | 원거리 흰 벽/사무실 물체에 고점수 | 흰 patch가 흰 milk 템플릿 patch와 cosine 높음 → appearance/semantic 모두 permissive. avg_5가 1~2개 유사 템플릿만으로 0.2 통과 (`detector.py:273-277`) | TP/FP의 appearance·semantic 분포 비교 (실험3) | **P0** |
| **FP-3 geometry 무력화** | 잘못 배치된 bbox도 생존 | vr=0이면 geo*vr=0, 분모도 2.0로 → geometry가 점수에 기여 못함. geometry는 게이트가 아니라 가중항일 뿐 (`detector.py:384`) | FP 후보의 geo·vr 값이 0 근처인지 확인 (실험3) | **P1** |
| **FN-1 semantic 절벽** | 실제 milk이 후보에서 탈락 | `sem>0.2` 단일 하드게이트. 시점 미커버/도메인갭/조명으로 sem<0.2면 즉시 제거, 점진적 degrade 없음 (`detector.py:286-288`) | GT=1인데 후보 0개인 프레임의 semantic 분포 (실험3) | **P0** |
| **FN-2 stability 과제거** | milk 마스크 자체가 생성 안 됨 | FastSAM `stability_score_thresh=0.97` 과도 → 흰/반사 경계의 불안정 마스크 제거 (`run_batch_inference_fast.py:169`, params.yaml:45) | stability 0.90/0.85로 낮춰 후보수 변화 비교 (실험5 보조) | **P1** |
| **FN-3 avg_5 희석** | 한두 템플릿만 맞아도 평균이 0.2 미만 | top-5 평균이라 좋은 템플릿이 1~2개뿐이면 희석 (예 (0.3+0.05×4)/5=0.1) (`detector.py:273-277`) | aggregation을 avg_1/max로 바꿔 recall 변화 측정 | **P1** |
| **FN-4 소형/원거리 milk** | 멀리 있는 milk 후보가 사전 제거 | `min_box 0.05`, `min_mask 3e-4`, `max_proposals=50` 면적컷 (`detector.py:352`, run_batch:386) | 면적 필터 완화 후 recall 변화 | **P2** |
| **공통 미보정** | 단일 임계로 TP/FP 분리 불가 | cosine 점수가 calibration 안 됨(CNOS 상속). 절대값 의미 불명확 | TP/FP score margin 분리도 측정 (실험3·4) | **P0** |

> temp.md 실측 연결: FP 다수가 "원거리 흰 배경"이라는 관측은 FP-2(흰 latch) + FP-1(forced output)의 조합이며, workspace z_max=1500이 FP −83을 만든 것은 이 후보들의 **pose_z가 멀다**는 사후 신호를 쓴 것일 뿐 — ISM 단계에서 이미 흰 배경에 고점수를 준 것이 근본이다.

---

## 4. RGB/Texture 사용 여부 분석

### 4.1 렌더링 asset 경로 (★직접확인)

- 렌더러: **BlenderProc** `bproc.renderer.render()` → RGBA color 반환 → BGR→RGB 후 `rgb_N.png` 저장 (`Render/render_custom_templates.py:119, 125-127`). depth/mask only 아님.
- 런타임 CAD: `data/cad/Milk_color_uv_vertexcolor_195mm.ply` (PLY 헤더 `property uchar red/green/blue/alpha` → vertex color 有).
- 런타임 템플릿: `sam6d_master/SAM-6D/Data/custom/Milk_scaled_195mm/templates/` — `rgb_0..41.png`(각 113–137KB, 512×512 RGB), `mask_0..41.png`(1.4–1.9KB), `xyz_0..41.npy`.

### 4.2 PLY/CAD texture 로딩 여부

- `trimesh.load(path)` 기본 로딩, visual 보존 (`utils/trimesh_utils.py:6-12`). PLY가 vertex color를 가지므로 BlenderProc 렌더에 색이 반영됨. **단 `--colorize` 플래그(기본 False)가 켜지면 균일 gray로 덮어씀** — 런타임 milk 템플릿은 색이 있으므로 colorize 미적용으로 렌더된 것으로 판단(파일크기 근거).

### 4.3 render output sanity check 방법

- ★완료: `ls -l rgb_*.png` 크기 113–137KB → 균일색 아님. (PIL 미설치로 픽셀 std 미측정 — §7 실험1에서 conda env 내 측정 권장.)
- 추가 권장: env 내 `python -c "from PIL import Image,...; std per channel"`로 전경 픽셀 채널 std 확인. std가 큰 값이면 텍스처 확정.

### 4.4 feature extractor 입력 확인

- `model/dinov2.py:115-120` `T.Normalize(mean=(0.485,0.456,0.406), std=(0.229,0.224,0.225))` — ImageNet, 3채널 보존.
- `run_inference_custom.py:131-139`: 템플릿 `convert("RGB")` 후 `image*mask` (배경 0). grayscale 변환 없음. → **RGB 정상 진입**.

### 4.5 문제가 있을 경우 예상 증상 + 미해결 caveat

- 만약 render가 constant color였다면: appearance score가 모든 후보에 비슷한 값 → TP/FP 분리 불가. 그러나 §4.1~4.4로 **이 시나리오는 기각**.
- **남은 caveat (검증 필요)**: `sam6d_debug.csv`의 `ism_score` 컬럼이 detector의 `final=(sem+appe+geo*vr)/(2+vr)`와 일치하는지, 아니면 노드가 별도로 `avg(sem,appe,geo)`로 재계산하는지 1행 산술 검증 필요. 불일치 시 §7 분석의 점수 해석에 영향. → **실험3의 사전 sanity check 항목으로 포함**.
- 실질적 결론: RGB는 정상 사용되나 **흰색 저텍스처 객체에서 patch-cosine appearance의 변별력이 낮고 점수가 미보정**이라는 것이 진짜 약점. "RGB 버그"가 아니라 "흰 물체 + 미보정 점수" 문제.

### 4.6 RGB/Texture 사용 여부 검증 체크리스트

- [x] 렌더러가 color 출력 요청하는가 — **예** (`render_custom_templates.py:119,125`)
- [x] CAD에 색 정보 있는가 — **예** (PLY vertex color)
- [x] 템플릿 PNG가 RGB이고 균일색 아닌가 — **예** (113–137KB)
- [x] DINOv2 입력이 RGB 3채널 + ImageNet norm인가 — **예** (`dinov2.py:115`)
- [x] 어디선가 grayscale 변환되는가 — **아니오**
- [ ] (실험1) 전경 픽셀 채널 std 측정으로 텍스처 풍부도 정량화 — 미완
- [ ] (실험3 사전) CSV `ism_score`가 detector 공식과 일치하는지 — 미검증

---

## 5. 관련 논문/기법 조사

> 모든 항목은 "pose 후처리"가 아니라 **mask candidate 생성/랭킹** 단계 개선 기준 평가. TP=실제 milk 회수(recall), TN=no-object/원거리 흰 distractor 거부.

| 기법/논문 | 핵심 아이디어 | TP 개선 | TN 개선 | SAM-6D 적용 난이도 | 추천 | 근거 |
|---|---|---:|---:|---:|---|---|
| **Temperature scaling** (Guo 2017) | cosine 점수 1-param 재보정 → 임계가 present/absent 의미 갖게 | – | M–H | **L** | **강추** | arxiv 1706.04599 |
| **Open-set learned threshold / "none" class** | forced-argmax 대신 보정된 reject 옵션 | L | **H** | L–M | 추천 | arxiv 2307.04047 |
| **Depth/metric-size candidate gate (RGB-D)** | 후보 median depth/물리크기가 CAD 스케일과 불일치 시 거부(후보 단계, pose 아님) | M | **H** | **L–M** | **강추** | SAD arxiv 2305.14207 |
| **SAM 3 (PCS + presence head)** | concept/exemplar 프롬프트 + 전역 presence 토큰으로 부재 거부 | **H** | **H** | M | **강추** | arxiv 2511.16719 |
| **Grounded-SAM (GDINO→SAM)** | text-box detect→segment; box 없으면 후보 0 | M | **H** | M | 추천 | arxiv 2401.14159 |
| **YOLO-World** | 실시간 open-vocab; 빈 프레임 자연 reject | M | **H** | M | 추천(실시간) | arxiv 2401.17270 |
| **T-Rex2 / T-Rex-Omni** | text+visual 프롬프트, **negative visual prompt** | **H** | M–H | M | 추천 | arxiv 2403.14610 / 2511.08997 |
| **Matcher** | DINOv2 correspondence로 SAM 프롬프트(매칭을 mask 前에) | M–H | M | M | 검토 | arxiv 2305.13310 |
| **PerSAM/PerSAM-F** | 1-shot SAM 개인화, pos/neg point prior | M | M | L–M | 검토 | arxiv 2305.03048 |
| **NIDS-Net weight adapter** | few-shot adapter로 DINOv2 임베딩 sharpen | M–H | M | M | 추천(저텍스처) | arxiv 2405.17859 |
| **FoundPose (intermediate DINOv2)** | 마지막 CLS 대신 중간층 patch → 저텍스처 변별↑ | M | M | M | 검토(feature swap) | arxiv 2311.18809 |
| **SAM 2 masklet propagation** | 메모리 기반 mask 추적/전파 | M | L | M | 검토(temporal) | arxiv 2408.00714 |
| **Temporal voting / track-before-detect** | N프레임 지속해야 채택, 누락 프레임 보간 | M–H | M–H | M | 추천 | arxiv 2009.11050 |
| **CNOS (baseline)** | DINOv2 CLS vs 템플릿, 고정임계 | – | L | (현 baseline) | — (문제의 출처) | arxiv 2307.11067 |

---

## 6. 적용 가능한 개선안 우선순위

### P0 — 코드 수정 전 분석/계측 (즉시, 위험 0)
1. **ISM score 분포 분석** (실험3): 기존 `sam6d_debug.csv`(per-candidate sem/appe/geo/ism, 1522행) × `visibility_labels.csv` 조인 → TP/FP/TN/FN별 분포. **어느 항목이 FP를 올리고 FN을 낮추는지 ISM 단계에서 확정**. 코드 0.
2. **Negative-frame calibration 가능성** (실험4): no-object top-1 vs present top-1 score, top1–top2 margin, sem–appe 일치도. 단일/복합 임계 분리력 평가. 코드 0.
3. **CSV `ism_score` 의미 sanity check** (§4.5 caveat): 1행 산술 검증.
4. **Render-RGB std 측정** (실험1): env 내 픽셀 std로 텍스처 풍부도 정량화. 코드 0(분석 스크립트만).

### P1 — 1일 Quick-Dev (저위험, 후보 단계 직접개입)
1. **Calibration + 명시적 no-object 임계**: ISM 결합 점수에 temperature scaling + 검증셋(부재 프레임 포함) 기반 reject 임계 도입 → forced-output 구조 해소(FP-1 직격). 분리 함수로 추가, pose 게이트와 무관.
2. **Depth/metric-size candidate gate**: 각 후보 mask의 median depth + back-projected 물리크기를 CAD 195mm 스케일과 비교, 불일치 거부. **후보 단계**에서 원거리 흰 distractor 제거(FP-2/FP-3). RGB-D 이미 보유. workspace z gate와 구분: 이것은 pose 추정 前 candidate mask에 작동.
3. **stability_score_thresh / aggregation 실험**: 0.97→0.90, avg_5→avg_3/max 스윕으로 FN-2/FN-3 영향 측정(분석→설정변경).

### P2 — 2~3일 연구/구현
1. **Detect-then-segment 프론트엔드**: FastSAM unconditioned 대신 Grounded-SAM 또는 YOLO-World/T-Rex2로 "milk carton" text/exemplar 프롬프트 → 빈 프레임 자연 reject(TN↑) + 후보 집중(TP↑).
2. **Temporal multi-frame voting**: ROS2 bag 연속성 활용. N프레임 지속 후보만 채택(transient 흰벽 latch 제거=TN), 누락 프레임 mask 전파(FN 회수=TP).
3. **Feature 업그레이드**: NIDS-Net weight adapter(milk crop 소수 few-shot) 또는 FoundPose 중간층 patch feature로 저텍스처 변별력↑.

### P3 — 리스크가 큰 장기 과제
1. **SAM 3 PCS + presence head** 도입: presence head가 부재 거부 전용이고 exemplar(pos/neg) 프롬프트로 TP/TN 동시 개선. 단 모델 통합·의존성·라이선스/가용성 리스크, ROS2 실시간성 검증 필요.
2. **3DGS PLY → 고품질 PBR 템플릿 재생성** 및 multi-view onboarding 보강(템플릿 커버리지/도메인갭 축소). asset 재생산 비용·검증 부담.

---

## 7. 다음 BMAD 단계 제안 & 실험 설계

### 7.1 실험 설계 (요청 5종)

**실험1 — Render RGB sanity check** (P0/P1)
- 절차: conda env 내 PIL로 `rgb_*.png` 전경(mask>0) 픽셀의 채널별 mean/std 산출. constant/grayscale/alpha-only 판정. real crop vs render crop을 나란히 PNG 저장.
- 산출물: `render_rgb_stats.csv`(template_id, fg_px, mean_rgb, std_rgb), `side_by_side/`.
- 합격선: std가 유의(예 >15/255)면 텍스처 정상. (★현재 파일크기 근거로 정상 가능성 높음.)

**실험2 — ISM candidate dump** (P0→P1)
- 절차: no-object N프레임 + object-present N프레임에서 top-K(예 5) 후보의 mask area/bbox/sem/appe/geo/ism, best template id를 덤프. `dump_ism_mask_candidates.py` 확장(점수 주입) 또는 detector 후크.
- 산출물: 후보별 crop+overlay PNG, `candidates.csv`(frame, gt, cand_idx, area, bbox, sem, appe, geo, ism, best_tmpl, selected).
- 비교: 최종 선택 후보 vs 탈락 후보.

**실험3 — Score distribution analysis** (P0, 코드 0)
- 절차: `sam6d_debug.csv` × `visibility_labels.csv` 조인 → 각 프레임의 후보를 GT로 TP/FP/TN/FN 라벨. similarity/appearance/geometric/ism_score 각각 4-class violin/hist. **pose_score 제외, ISM 단계 점수만.**
- 핵심 질문: (a) FP를 올리는 항목? (b) FN을 낮추는 항목? (c) 어떤 항목이 가장 분리력 있나(AUC)?
- 사전 sanity: ism_score == (sem+appe+geo*vr)/(2+vr) 검증.

**실험4 — Negative frame calibration** (P0, 코드 0)
- 절차: GT=0 프레임 top-1 후보 score 분포 vs GT=1 top-1. 단일 임계 분리력 + (top1−top2) margin, mask area 안정성, sem-vs-appe 일치도 결합 분리력 평가.
- 산출물: ROC/PR, 권장 reject 규칙 후보(복합).

**실험5 — Multi-view/3DGS asset quality validation** (P1/P2)
- 절차: PLY vertex color 보존 확인(★완료: 보존됨). 렌더가 milk 색/문양 보존하는지 시각 확인(실험1 연계). onboarding 각도 수(42뷰)/편향이 커버리지 악화시키는지 — present 프레임 best_tmpl 분포로 사용 뷰 편중 확인. 정상 mesh vs 3DGS PLY 동일 scene 비교(자산 2종 있으면).

### 7.2 PRD로 넘길 항목
- "ISM Candidate Calibration & Absence Rejection" PRD: P1-1(calibration+no-object 임계) + P1-2(depth/metric-size candidate gate)를 하나의 epic으로. 성공지표: 315-frame 셋에서 **R≥0.90 유지하며 FP 추가 감소**(workspace gate 대비 ISM 단계 분리), pose 게이트와 독립 측정.

### 7.3 Quick-Dev story 후보
- QD-1: `sam6d_debug.csv` 기반 ISM score 분포 분석 노트북/스크립트(실험3·4) — 코드 0, 산출 리포트.
- QD-2: depth/metric-size candidate gate를 ISM 출력 직후(PEM 前) 후보 필터로 삽입(workspace post-pose gate와 명확히 분리된 seam: `run_batch_inference_fast.py:570` 직전).
- QD-3: ISM 결합 score reject 임계 + temperature 파라미터를 config화(`confidence` 블록 확장).

### 7.4 QA 검증 기준
- 모든 변경은 **315-frame shadow 셋에서 ISM 단계 confusion**으로 평가(pose 게이트 off). recall 절벽(<0.90) 금지.
- depth/metric-size gate는 **pose 추정 前 candidate에 작동함을 코드 위치로 증명**(workspace z gate와 혼동 금지).
- hold-out 분리 검증(temp.md의 z_max=1500 과적합 경고 반영): 임계는 train/val 분리 후 확정.

### 7.5 hard mode 금지/허용 조건
- **금지**: 단일 score 임계 튜닝만으로 결론(임계 sweep은 분석 보조로만). workspace z gate 재튜닝으로 회귀. GT 시간매핑 미확정 상태의 확정 결론.
- **허용(hard 전환 가능)**: 실험3·4로 ISM 단계 분리력이 입증되고, calibration/candidate-gate가 hold-out에서 R≥0.90·FP↓를 동시 달성할 때만 hard 게이트 적용.

---

## 8. 부록 — ★직접 확인 명령 근거 (재현용)

- final_score 공식: `grep -n "final_score" run_inference_custom.py` → `:201 (semantic_score + appe_scores + geometric_score*visible_ratio) / (1 + 1 + visible_ratio)`; 동일 `detector.py:384`.
- 계측 자산: `wc -l outputs/validation/SLAM_with_milk_nomilk/_run/debug/sam6d_debug.csv` → 1523행(헤더+1522). 컬럼 6–9 = similarity/appearance/geometric/ism_score. `visibility_labels.csv` = frame_id,milk_visible(0/1).
- 템플릿 RGB: `ls -l .../templates/rgb_*.png` → 42장 각 113,780–137,601 B (균일색 불가). `mask_*.png` = 1.4–1.9KB(서브에이전트 오인 원인).
- CAD vertex color: `head -c 600 data/cad/Milk_color_uv_vertexcolor_195mm.ply` → `property uchar red/green/blue/alpha`, vertex 16302 / face 17494.
- 런타임 경로: `output/sam6d_runtime_params.yaml:4 cad_path=data/cad/Milk_color_uv_vertexcolor_195mm.ply`, `:6 template_dir=.../Milk_scaled_195mm/templates`.

### 미검증/주의 사항
- `sam6d_debug.csv`의 `ism_score` 컬럼이 detector 공식과 동일한지 1행 산술 미검증 (§4.5).
- 전경 픽셀 채널 std 미측정(PIL 미설치) — 텍스처 풍부도는 파일크기 정황근거 (실험1로 확정 필요).
- detector.py 라인 인용 일부는 서브에이전트 코드리딩 기반(☆); final_score·계측·asset 핵심은 ★직접확인.
- 웹 논문 중 depth-aware-SAM 일부(arxiv 2602.xxxx)는 인덱스상 2026 표기로 ID 미검증 — 핵심 권고(depth/size gate, calibration, SAM3 presence, temporal voting)는 검증된 출처 기반.
