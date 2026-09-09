# 02. 대용량 자산 아카이브 — 목록·해제 위치·검증

GitHub에 올릴 수 없는 파일(100 MB 초과 / 비공개 CAD / 재다운로드 번거로운 가중치)을
**4개 아카이브(합 3.6 GB)** 로 분리했다. 아카이브 이름은 `slam-stack-assets/` 디렉터리에 있다.

## 0. 받기 + 한 줄 복원

아카이브는 이 레포의 **GitHub Release `assets-v1`** 로 배포된다(private 레포이므로 접근권자만).

```bash
gh release download assets-v1 --repo ldh8628/slam-perception-stack --dir ~/slam-stack-assets

cd <레포 루트>
bash scripts/restore_assets.sh ~/slam-stack-assets
```

USB/외장하드로 받았다면 그 경로를 인자로 주면 된다.

아카이브는 **레포 루트 기준 상대경로**로 만들어져 있다. 즉 레포 루트에서 풀면
모든 파일이 원래 있어야 할 자리에 그대로 들어간다. **수동으로 파일을 옮길 필요가 없다.**

수동으로 하고 싶다면:

```bash
cd <레포 루트>
sha256sum -c <아카이브 디렉터리>/SHA256SUMS          # 무결성 확인
tar -I zstd -xf <아카이브 디렉터리>/assets-01-orbslam-vocab.tar.zst -C .
cat <아카이브 디렉터리>/assets-02-sam6d-weights.tar.zst.part-* | tar -I zstd -x -C .
tar -I zstd -xf <아카이브 디렉터리>/assets-03-sam6d-cad.tar.zst -C .
tar -I zstd -xf <아카이브 디렉터리>/assets-04-sam6d-templates.tar.zst -C .
```

> `zstd` 필요: `sudo apt install zstd` 또는 `conda install -c conda-forge zstd`
> 2 GB 초과 아카이브는 GitHub Release 업로드 한도(2 GB/파일) 때문에 **1900 MB 단위로 분할**돼 있다.
> `.part-aa`, `.part-ab` … 를 **순서대로 `cat`** 해야 한다(위 명령의 glob이 알아서 정렬한다).

---

## 1. 아카이브 목록

| 아카이브 | 압축 후 | 원본 | 없으면 무엇이 안 되나 |
|---|---:|---:|---|
| `assets-01-orbslam-vocab.tar.zst` | 80 MB | 180 MB | ORB-SLAM3가 **아예 기동 못 함** |
| `assets-02-sam6d-weights.tar.zst.part-{aa,ab}` | 2.7 GB | 3.0 GB | SAM-6D ISM/PEM 전부 |
| `assets-03-sam6d-cad.tar.zst` | 622 MB | 2.4 GB | SAM-6D 검출·자세추정(템플릿 재생성도 불가) |
| `assets-04-sam6d-templates.tar.zst` | 202 MB | 1.5 GB | ISM 템플릿 매칭 (단, CAD가 있으면 재생성 가능) |
| **합계** | **3.6 GB** | 7.1 GB | |

`SHA256SUMS` 파일이 같은 디렉터리에 있다.

---

## 2. 아카이브별 상세

### assets-01-orbslam-vocab — ORB-SLAM3 어휘사전

| 해제 경로 | 크기 |
|---|---:|
| `orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt` | 145 MB |
| `orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz` | 41 MB |

- `no_cli_rgbd.yaml` 의 `orbslam3.vocabulary_path` 기본값이 정확히 이 경로다.
- 대안: ORB-SLAM3 공식 저장소에서 동일 파일을 받아도 된다
  (`https://github.com/UZ-SLAMLab/ORB_SLAM3` → `Vocabulary/ORBvoc.txt.tar.gz`).

### assets-02-sam6d-weights — 모델 가중치·체크포인트 (12개 파일)

| 해제 경로 | 크기 | 용도 | 공개 재다운로드 |
|---|---:|---|---|
| `sam6d_ws/sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth` | 1.3 GB | PEM 본체 | gdown `1joW9IvwsaRJYxoUmGo68dBVg-HcFNyI7` |
| `sam6d_ws/sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/mae_pretrain_vit_base.pth` | 328 MB | PEM 백본 | `dl.fbaipublicfiles.com/mae/pretrain/mae_pretrain_vit_base.pth` |
| `sam6d_ws/sam6d_master/SAM-6D/checkpoints/mae_pretrain_vit_base.pth` | 328 MB | 위와 **동일 파일**(양쪽 필요) | 동일 |
| `sam6d_ws/sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/dinov2/dinov2_vits14_pretrain.pth` | 85 MB | ISM 특징 (**ViT-S/14**) | `dl.fbaipublicfiles.com/dinov2/dinov2_vits14/dinov2_vits14_pretrain.pth` |
| `.../Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-x.pt` | 139 MB | 원본 SAM-6D ISM 경로 | gdown `1m1sjY4ihXBU1fZXdQ-Xdj-mDltW-2Rqv` |
| `.../Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-s.pt` | 23 MB | 〃 | ultralytics assets |
| `sam6d_ws/weights/clip/ViT-B-32.pt` | 338 MB | YOLO-World 텍스트 인코더 | openaipublic CLIP |
| `sam6d_ws/mobile_sam.pt` | 39 MB | **운영 ISM의 분할기** | MobileSAM repo |
| `sam6d_ws/sam_b.pt` | 358 MB | ultralytics SAM-b | ultralytics assets |
| `sam6d_ws/yolov8m-worldv2.pt` | 55 MB | **운영 ISM의 제안기** | ultralytics assets |
| `sam6d_ws/yolov8s-worldv2.pt` | 25 MB | fallback | ultralytics assets |
| `sam6d_ws/yoloe-11s-seg.pt` | 27 MB | YOLOE(localization 개선 실험용) | ultralytics assets |

> **DINOv2 주의**: 공식 `download_dinov2.py`는 vitl 등 다른 크기를 받는다.
> 이 파이프라인은 `yolo_ism.py`의 `DEFAULT_DINOV2_CKPT`(**vits14**)를 쓴다. 크기를 바꾸면 점수 체계가 전부 어긋난다.

### assets-03-sam6d-cad — CAD 모델 (**비공개, 재다운로드 불가**)

해제 위치: `sam6d_ws/data/cad/`

| 객체 디렉터리 | 비고 |
|---|---|
| `Bear` | 곰인형 |
| `Rabbit` | 토끼인형 |
| `Dinosaur` | 공룡인형 (템플릿 hue +9 보정 적용 대상) |
| `choco_hazelnut_color_high` | 초코하임 |
| `Mugcup_color_high` | 머그컵 |
| `saffron` | 샤프란 |
| `Sauce_high` | 저당소스 |
| `Febreze_high` | 페브리즈 |
| `Sikhye_high` | 식혜 |
| `Rack` | 랙 |
| `milk` | 우유(초기 단일객체 실험) |
| `fail` | 실패한 스캔 보관 |
| `20260616.zip` | 스캔 원본 백업 |

> ⚠ **단위**: SAM-6D/BOP는 CAD를 **mm** 로 가정하고 내부에서 /1000 한다.
> 이 아카이브의 `.ply`는 이미 ×1000(mm) 변환본이며, `*.ply.orig_meter` 가 meter 원본이다.
> 새 CAD를 meter 단위로 넣으면 **에러 없이 검출 0건**이 된다 →
> `sam6d_ws/tools/e2e_pipeline/rescale_cad_to_mm.py` 로 변환할 것.

### assets-04-sam6d-templates — 렌더 템플릿 (재생성 가능하지만 오래 걸림)

해제 위치: `sam6d_ws/template/` + `sam6d_ws/sam6d_master/SAM-6D/Data/custom/`

객체 12종 × 42뷰(rgb / mask / xyz):
`Bear, Rabbit, Dinosaur, choco_hazelnut_color_high, Mugcup_color_high, saffron,
Sauce_high, Febreze_high, Sikhye_high, Rack, milk, Milk_scaled_195mm`

- 제외: `template/fail/`(695 MB, 실패 렌더), `template/_logs/`
- CAD만 있으면 재생성 가능: `bash sam6d_ws/tools/render_all_templates.sh`
- **ISM 특징 캐시**(`sam6d_ws/outputs/yolo_ism_object_n/template_features/*.pt`)는 포함하지
  않았다. 첫 실행 시 자동 생성된다(강제 재생성 `--rebuild-features`).

---

## 3. 포함하지 **않은** 대용량 항목과 그 이유

| 항목 | 크기 | 왜 뺐나 |
|---|---:|---|
| `data_slam/` (0724_chungbuk, 260714_frame_data, rgbd_bag) | 319 GB | 압축이 안 되는 원시 bag. 외장하드/rsync 직접 복사가 유일하게 현실적 |
| `sam6d_ws/data/ros2_bag/` | 44 GB | 〃 |
| `rtabmap_ws/data/` | 15 GB | 〃 |
| `*/output`, `*/outputs` (SLAM 궤적·맵·ISM/PEM 결과) | 40 GB+ | 재실행으로 재생성되는 산출물 |
| `sam6d_ws/_ism_research_2026_07/` | 2.3 GB | 종료된 연구 중간산출물(결론은 `_bmad_output_for_slam/` md에 있음) |
| `orbslam_ws/src/orbslam3_core/` | 1.6 GB | UZ-SLAMLab/ORB_SLAM3 **원본 클론**(commit `4452a3c`). 빌드에 안 쓰임 — 필요하면 git clone |
| `sam6d_ws/weights_transfer/` | 1.6 GB | 이전 이관용 분할 아카이브. 이번 `assets-02`가 대체 |
| colcon `build/ install/ log/` | — | 새 PC에서 재생성 |

→ 데이터셋 이관 방법은 `docs/03_DATASETS.md`.
