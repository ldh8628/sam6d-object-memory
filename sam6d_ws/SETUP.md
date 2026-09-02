# sam6d_realtime — 새 기계에 올리기

이 번들은 **코드 · 가중치 · 템플릿 · 입력 bag 까지 다 들어 있다.**
새 기계에서 따로 만들어야 하는 건 **파이썬 환경 하나뿐**이다.

전제: **NVIDIA GPU + 드라이버**. 모든 run yaml 이 `device: cuda:0` 이고 PEM 은 CUDA 확장을 쓴다.
CPU 로는 못 돌린다.

---

## 0. 풀기

```bash
tar -xpzf sam6d_realtime_bundle_260820.tar.gz -C /원하는/경로
cd /원하는/경로/sam6d_realtime
```

`-p` 를 빼지 말 것. `ism_hsv.py` 가 템플릿 렌더 42장의 **`크기 + mtime`** 으로 HSV 캐시가
아직 유효한지 판단한다(`ism_hsv.py:87`). mtime 이 어긋나면 게이트가 조용히 fail-open 되어
**판정이 워크스테이션과 달라진다.** tar 는 mtime 을 정확히 보존하므로 `-p` 로 풀면 그대로다.
그래도 의심스러우면 `python tools/build_hsv_template_cache.py` 로 다시 구우면 된다.

풀어서 나오는 것(총 ~15 GB):

| 경로 | 무엇 |
|---|---|
| `realtime/` | 노드 본체(단일/분리 2가지), launch 2종, run yaml 5종 |
| `yolo_ism*.py`, `ism_hsv.py`, `ism_training_free_gate.py` | ISM 판정 본체 (오프라인 배치와 **같은 파일**) |
| `configs/yolo_ism_objects.yaml` | 운영 config (활성 객체 9개) |
| `sam6d_master/SAM-6D/` | DINOv2 + PEM 코드·체크포인트 (1.7 GB) |
| `template/` | 활성 9객체의 렌더 42장 |
| `outputs/.../template_features/` | ISM 템플릿 특징 + HSV 기준값 |
| `assets/` | CLIP · PEM 템플릿 · CAD 8192점 |
| `data/longcircle2/` | 검증용 입력 bag (72.203초, RGB/depth 각 2,166장, 3.5 GB) |

---

## 1. 베이스 환경 (conda)

`requirements.txt` 만으로는 안 된다. ROS 2 Jazzy 와 CUDA 빌드 torch 가 PyPI 에 없다.

```bash
mamba env create -f environment.yml       # conda 로 해도 되지만 훨씬 느리다
conda activate sam6d
```

## 2. pip 계층

```bash
conda activate sam6d
pip install --no-deps ultralytics==8.4.64 ultralytics-thop==2.1.6 segment-anything==1.0
pip install -r requirements.txt
```

`ultralytics` 3종을 먼저 `--no-deps` 로 까는 이유: 그냥 깔면 pip 이 conda 가 넣은
torch/opencv 를 PyPI 휠로 덮어써서 **CUDA 없는 torch** 가 되고 numpy2 ABI 도 깨진다.

## 3. 확인

```bash
python -c "import rclpy, cv_bridge, message_filters, torch; \
           from vision_msgs.msg import Detection3DArray; print(torch.cuda.is_available())"
```
`True` 가 나와야 한다.

## 4. pointnet2 CUDA 확장 빌드 (PEM 필수)

```bash
conda activate sam6d                       # ★ 반드시 activate 상태에서
cd sam6d_master/SAM-6D/Pose_Estimation_Model/model/pointnet2
export TORCH_CUDA_ARCH_LIST="8.6;8.9;9.0;12.0+PTX" MAX_JOBS=16 CUDA_HOME=$CONDA_PREFIX
pip install --no-build-isolation .
cd -
```

확인:
```bash
python -c "from pointnet2 import _ext; import torch; \
           print(_ext.furthest_point_sampling(torch.rand(1,100,3,device='cuda').contiguous(),16).shape)"
```

**함정 두 개.**
- env 의 pip 바이너리를 직접 부르면(`~/miniconda3/envs/sam6d/bin/pip`) `CONDA_PREFIX` 가
  base 로 잡혀 `fatal error: torch/all.h: No such file or directory` 가 난다. 거기서 `-I` 를
  억지로 넣으면 이번엔 nvcc 가 gcc14 libstdc++ 헤더를 파싱하다
  `error: type name is not allowed (__is_array)` 로 2차 실패한다. **activate 가 정답이다.**
- `TORCH_CUDA_ARCH_LIST` 는 노트북 GPU 에 맞춰 줄여도 된다
  (8.6=RTX30 / 8.9=RTX40·Ada / 9.0=Hopper / 12.0=Blackwell). 여러 개 구우면 빌드가 길어질 뿐이다.

빌드를 건너뛰어도 죽지는 않는다. `pointnet2_utils.py:26` 이 import 에 실패하면 순수 PyTorch
fallback 으로 넘어간다 — **다만 PEM 이 크게 느려진다.**

---

## 5. 실행

```bash
conda activate sam6d
export ROS_DOMAIN_ID=72          # ORB-SLAM3 실시간이 71 을 쓰므로 분리
cd /원하는/경로/sam6d_realtime
```

**bag 재생 (권장 — 수신/추론 프로세스 분리판)**
```bash
ros2 launch realtime/launch/sam6d_split.launch.py config:=realtime/run_split_example.yaml
```

**bag 재생 (단일 프로세스판)**
```bash
ros2 launch realtime/launch/sam6d_realtime.launch.py config:=realtime/run_bag_example.yaml
```
단일 프로세스판은 고속 영상 구독과 추론이 같은 인터프리터에 있어 GIL 때문에 초 단위 정체가
생긴다(최악 9.6 s). 검증용으로만 쓸 것.

**실제 카메라 (RealSense D455f)** — 터미널 2개
```bash
# A: 드라이버
./run/realsense.sh
# B: SAM-6D
./run/sam6d.sh
# ./run/sam6d.sh --view --record
# ./run/sam6d.sh --record-full-depth
```

`run/realsense.sh`가 `realsense` conda 환경(ROS 2 Jazzy)을 활성화하고
`ROS_DOMAIN_ID=72`, RGBD 결합 발행·depth 정렬·동기화·640x480x30 프로파일을 적용한다.
기록은 `output/live_YYYYMMDD_HHMMSS/`에 저장되며 `python tools/serve_pem_explorer.py`
실행 후 localhost Explorer에서 RGB/pose 타임라인과 선택 Depth를 다시 볼 수 있다.

`bag.path` 는 **번들 루트(`sam6d_realtime/`) 기준 상대경로**로 해석된다. 현재 기본값은
재부팅 후에도 독립적으로 사용할 수 있는 로컬 사본 `data/longcircle2` 이다.

---

## 6. 제대로 올라갔는지 보는 법

기동 로그에 이 줄이 찍혀야 한다. 값이 다르면 config 나 캐시가 어긋난 것이다.

```
[ism] relative_assignment=True colour_tiebreak=0.6 cross_object_nms=True hsv_gate=True appe_blocks=[2, 9]@0.605
```

속도 대조 (ROS 를 안 거치는 순수 처리 성능):
```bash
python tools/bench_offline.py --synthetic --n 200 --out bench_laptop.json
python tools/bench_compare.py bench_workstation.json bench_laptop.json
```
워크스테이션 기준선은 전체 **81 ms/프레임(14 Hz)**, GPU 214 W. 노트북에서 **1.5~2.5배 느린 건
정상**이다(발열로 클럭이 떨어지므로 `gpu_start`/`gpu_end` 도 같이 볼 것).

판정 대조는 `data/longcircle2`의 같은 프레임과 같은 난수 시드를 사용해야 한다. 실시간 재생은
처리 프레임이 실행마다 달라질 수 있으므로 설정 비교에는 `temp/verify_eval.py`를 사용한다.

**속도는 달라도 되지만 판정은 같아야 한다.** 다르면 HSV 캐시(mtime) 나 config 를 의심할 것.

---

## 7. 알아 둘 것

- `configs/yolo_ism_objects.yaml` 의 `cad_ply: data/cad/...` 경로는 **번들에 없다.** 일부러 뺐다.
  PEM 이 CAD 에서 쓰는 건 8192점뿐이라 `assets/model_points/<객체>.npy` 로 대체했고,
  `realtime/sam6d_core.py:145` 가 `trimesh.load_mesh` 를 그쪽으로 바꿔 끼운다. 무해하다.
- 활성 객체는 **9개**다(`Rabbit` 은 `enabled: false`). Rabbit 을 켜려면 템플릿과 ISM 특징
  캐시를 따로 가져와야 한다 — 번들에 없다.
- 영상 토픽 QoS 는 `reliable` 이어야 한다. `best_effort` 로 두면 같은 30 Hz 입력에서
  **3.7 Hz 밖에 안 들어온다**(900 KB Image 의 UDP 조각 하나만 잃어도 샘플이 통째로 버려진다).
  카메라가 best_effort 로 발행하면 run yaml 의 `qos_reliability` 를 맞춰 줘야 한다.
- 인터넷은 필요 없다. CLIP 텍스트 인코더만 ultralytics 가 기본적으로 내려받는데
  `realtime/sam6d_core.py:77` 이 `assets/` 를 보도록 고정해 뒀다.
