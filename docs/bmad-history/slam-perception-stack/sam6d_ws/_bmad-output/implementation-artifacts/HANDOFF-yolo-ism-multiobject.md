# HANDOFF — YOLO-World multi-object ISM 인식 (세션 이어가기용)

작성: 2026-06-18 · 브랜치 `feat/yolo-ism-multiobject` · 새 세션은 이 파일을 먼저 읽고 이어가면 됨.

## 목표 / 범위
`yolo_ism.py`(원본 무수정, import 재사용)를 기반으로 **config 기반 멀티객체 ISM 인식**
실행 파일 `yolo_ism_object_n.py`를 구현. YOLO-World prompt·템플릿·캐시를
`configs/yolo_ism_objects.yaml`에서 로드. **PEM/6D pose/geometric 제외** — ISM(semantic cls
+ masked-appe)까지만. 객체 12개(milk, choco/Febreze/Mugcup/Sauce/Sikhye 각 high/low, saffron).

## 산출물 / 핵심 파일
- `yolo_ism_object_n.py` — 실행 파일. 프레임당 **단일 멀티클래스 YOLO predict → b.cls argmax 라우팅 → 객체별 semantic+masked-appe 게이트**. high/low는 prompt 공유(템플릿만 다름).
- `configs/yolo_ism_objects.yaml` — 객체별 prompt/template_dir/cache. defaults 운영점(conf0.02·sim0.35·appe_gate0.55·use_mask).
- `tools/run_object_n_batch.sh` — 다중 bag 배치 러너.
- `_bmad-output/implementation-artifacts/spec-yolo-ism-multiobject.md` — spec(status done).
- 결과: `outputs/yolo_ism_object_n/<bag>/` (객체별 `<name>_results.csv` + overlay + `combined/` + `multiobject_summary.csv`). 로그: `outputs/_logs/`.
- 템플릿 캐시: `outputs/yolo_ism_object_n/template_features/<name>_{cls,appe}.pt` (template_dir 바꾸면 같은 name 캐시가 stale → 캐시 삭제 후 재실행 필요).

## 커밋 (브랜치 feat/yolo-ism-multiobject)
- `4595fe7` feat: 구현(코드+config+spec)
- `1c42349` fix: milk 템플릿 교체 + choco/saffron prompt 1차
- `ed15c8e` fix: 색-속성 프롬프트(cross-talk 해소) ← **현재 HEAD**

## 현재 프롬프트 set (ed15c8e)
| 객체 | prompt | 근거 |
|---|---|---|
| milk | `milk carton` | 템플릿=**Milk_scaled_195mm**(template/milk는 sem~0.22로 틀림, 195mm는 ~0.52) |
| choco_high/low | `carton` | 갈색 납작 과자박스(초코하임). ※일부 bag서 취약(아래) |
| Febreze_high/low | `febreze spray bottle` | 브랜드명 고유 |
| Mugcup_high/low | `mug cup` | 보라색 컵 |
| saffron | `white jug` | 실물=크림색 손잡이 통(soap 아님) |
| Sauce_high/low | `brown bottle` | 어두운 갈색 병 |
| Sikhye_high/low | `yellow can` | 노란/금색 음료 캔 |

## 핵심 발견 (재도출 불필요)
1. **milk=0 원인=잘못된 템플릿**: `template/milk/templates`(sem~0.22) vs `Milk_scaled_195mm/templates`(sem~0.52, 검증됨). 동일 crop 비교로 확정.
2. **choco/saffron 미검출 원인=YOLO prompt 부적합**(객체 외형과 텍스트 불일치). "chocolate hazelnut spread jar"→0건.
3. **멀티클래스 cross-talk**: 단일 predict에서 box는 argmax로 한 prompt에만 배정 → 닮은 객체(캔/병)가 서로 box를 뺏음 → 엉뚱한 ISM서 기각되어 **양쪽 손실**. Sauce는 단일prompt 67프레임 중 59프레임이 다른 class에 뺏김.
4. **해법=색-속성 프롬프트**: YOLO-World는 CLIP기반이라 색/속성 이해. 모양-only(`drink can`/`sauce bottle`)는 충돌, **색-특화(`yellow can`/`brown bottle`)는 CLIP 유사도가 자기 객체서만 peak → 도둑질↓**. 긴 구문은 오히려 희석(예 "brown sauce bottle"→Sauce 1건). 짧은 색+모양이 최적.
5. **약한 low 변종(choco_low 9, Febreze_low 70)**: high와 같은 prompt·box인데도 약함 → **prompt 아니라 저텍스처 템플릿 품질 문제**(choco_low=below-sim, Febreze_low=below-appe 101). prompt 재설계로 해결 불가. 레버=appe_gate(주의: 객체별 score 인하는 일반화 위해 금지) 또는 더 좋은 템플릿. high/low는 동일 객체의 2개 스캔이라 사실상 중복.

## 다중 bag 일반화 결과 (검출률 %, ed15c8e config)
| 객체 | t_around | t_around_goback | diag1 | diag2 | t_goback | high_tex_around |
|---|--:|--:|--:|--:|--:|--:|
| saffron `white jug` | 89 | 13 | 73 | 70 | 53 | 96 |
| Sauce `brown bottle` | 40 | 7 | 55 | 42 | 24 | 60 |
| Sikhye `yellow can` | 51 | 12 | 59 | 44 | 46 | 8* |
| milk | 54 | 10 | 54 | 44 | 21 | 69 |
| Mugcup | 48 | 12 | 61 | 59 | 32 | 72 |
| choco `carton` | 48 | **0** | 45 | 43 | **2** | 34 |
→ 색 프롬프트 **일반화 양호**. 낮은 칸 대부분 장면/가시성 탓(`t_around_goback`은 전 객체 9~17%=카메라가 테이블 거의 안 봄; high_tex엔 Sikhye 거의 없음).
**예외=choco만 일부 bag서 취약**(0%,2%) — shape-only라 milk "milk carton"과 경쟁+변별력 부족.

## 다음 액션 (중단 지점)
**진행 중이었던 것**: choco를 **color+shape**(`brown carton`/`brown box` 등)로 재설계해 멀티클래스 스윕(milk 충돌 동시 측정) 예정이었음.
1. choco prompt 색-속성 스윕: 후보 `["carton","brown carton","brown box","chocolate box","brown package"]`, 취약 bag(`two_table_goback`,`two_table_around_goback`,`two_table_diagonal2`) 프레임에서 멀티클래스 ISM-pass + milk 보존 측정. 최적 적용 후 전 bag 재실행.
2. (사용자 지시) 결과 미흡 시 **color+모양+(무언가: 브랜드텍스트/크기 등)**로 재설계 — 단 긴 구문은 희석되니 주의. 재실행해 ros2 bag 결과 제시.
3. (선택) low 변종은 prompt 무관 → appe_gate/템플릿 관점에서 별도 검토.

## 재현/실행 방법
```bash
PY=/home/ldh9501/miniconda3/envs/sam_yolo/bin/python
# 단일 bag
$PY yolo_ism_object_n.py --config configs/yolo_ism_objects.yaml \
  --bag data/ros2_bag/two_table_around \
  --frames-dir outputs/yolo_test/two_table_around/frames
# 다중 bag 배치
bash tools/run_object_n_batch.sh
# 템플릿 캐시 stale 시: rm -rf outputs/yolo_ism_object_n/template_features
```
- env: conda `sam_yolo`. 멀티클래스 스윕 시 YOLOWorld 인스턴스 재사용 금지(CLIP text encoder device 충돌) → prompt마다 새 인스턴스.
- 임시 파일은 /tmp에 만들고 삭제, predict는 save=False(파일 미생성).
