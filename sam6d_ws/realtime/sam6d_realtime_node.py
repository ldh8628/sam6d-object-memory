#!/usr/bin/env python3
"""sam6d_realtime_node.py — SAM-6D(ISM + PEM) 실시간 ROS 2 노드.

    conda activate sam6d          # ROS 2 Jazzy
    python realtime/sam6d_realtime_node.py --config realtime/run_bag_example.yaml

들어오는 RGB + aligned depth 한 쌍마다 물체를 찾고(ISM) 6D 포즈를 구해(PEM)
`/sam6d/detections`(vision_msgs/Detection3DArray)로 이름·점수와 함께 내보낸다.

**모든 프레임을 처리하지 않는다.** 짝이 맞은 (RGB, depth) 는 `_pending` 한 칸에 계속
덮어써지고, 일꾼 타이머가 한 장을 끝낼 때마다 **그 시점에 들어와 있는 가장 최신 한 쌍**을
집어 간다. 그 사이 들어온 프레임은 건너뛴다.
버린 장수와 처리 Hz는 `/sam6d/status` 로 계속 내보내므로 눈으로 확인할 수 있다.

ISM 판정은 오프라인 배치(tools/build_pem_inputs.py)와 **같은 함수**를 쓴다:
`yolo_ism_object_n.recognize_frame_auto`. 따라서 config 를 켜는 것만으로

    multi_label            한 박스가 여러 프롬프트를 유지 (YOLO NMS 래핑)
    relative assignment    박스마다 주인 객체를 먼저 뽑고 절대 게이트는 그 뒤에
    colour tie-break       형태로 구분 안 되는 짝(인형 3종)은 HSV 색으로 승자 결정
    cross-object NMS       같은 박스를 두 라벨이 주장하면 rank_appe 로 하나만

이 그대로 따라온다. 이전 실시간 노드(sam6d_ros/sam6d_multiobject_node.py)는 객체별
`recognize()` 를 돌아 이 넷이 전부 빠져 있었다.

CAD 는 필요 없다. PEM 이 쓰는 것은 CAD 에서 뽑은 8192 점뿐이라
`assets/model_points/<객체>.npy` 로 대체한다(원본 PLY 919 MB 를 1 MB 로).
PEM 템플릿 특징도 `assets/pem_templates/<객체>.pt` 로 미리 구워 두었다.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
PEM_DIR = os.path.join(REPO, "sam6d_master", "SAM-6D", "Pose_Estimation_Model")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from verify_config import build_appe_cfg, describe            # noqa: E402
from slam_pose_memory import (canonical_object_points,
                              canonical_object_rotation)       # noqa: E402

import rclpy                                                    # noqa: E402
from cv_bridge import CvBridge                                  # noqa: E402
from geometry_msgs.msg import Pose                              # noqa: E402
from message_filters import ApproximateTimeSynchronizer, Subscriber  # noqa: E402
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,  # noqa: E402
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor               # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.parameter import Parameter                           # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from scipy.spatial.transform import Rotation                    # noqa: E402
from sensor_msgs.msg import CameraInfo, Image                   # noqa: E402
from std_msgs.msg import String                                 # noqa: E402
from vision_msgs.msg import (Detection3D, Detection3DArray,     # noqa: E402
                             ObjectHypothesisWithPose)

import yolo_ism as yi                                           # noqa: E402
import yolo_ism_object_n as o_n                                 # noqa: E402


def _sync():
    """GPU 작업이 실제로 끝날 때까지 기다린다 — 단계별 시간을 정직하게 재기 위해."""
    if torch.cuda.is_available():
        torch.cuda.synchronize()


_CLK = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100


def _thread_cpu():
    """이 프로세스의 스레드별 누적 CPU 시간 [s]. 키는 'tid:이름'.

    정체 구간에서 워커가 자고 있는데 프로세스 전체 CPU 는 늘어난다 — 그때 **어느 스레드가**
    CPU 를 쓰는지 이름까지 봐야 원인을 짚을 수 있다. /proc 만 읽으므로 비용이 거의 없다.
    """
    out = {}
    try:
        for t in os.listdir("/proc/self/task"):
            try:
                with open(f"/proc/self/task/{t}/stat") as f:
                    p = f.read().rsplit(") ", 1)[1].split()
                with open(f"/proc/self/task/{t}/comm") as f:
                    nm = f.read().strip()
                out[f"{t}:{nm}"] = (int(p[11]) + int(p[12])) / _CLK
            except Exception:
                pass
    except Exception:
        pass
    return out


def _abs(path: str, base: str = REPO) -> str:
    path = os.path.expanduser(str(path))
    return path if os.path.isabs(path) else os.path.join(base, path)


def _deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in (over or {}).items():
        out[k] = _deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


DEFAULT_CFG = {
    "topics": {
        "rgb": "/camera/camera/color/image_raw",
        "depth": "/camera/camera/aligned_depth_to_color/image_raw",
        "caminfo": "/camera/camera/color/camera_info",
    },
    "ism": {
        "config": "configs/yolo_ism_objects.yaml",
        "objects": [],                 # 비우면 config 의 활성 객체 전부
    },
    "runtime": {
        "device": "cuda:0",
        "use_sim_time": True,          # bag 재생이면 true, 실제 카메라면 false
        "sync_slop": 0.02,
        "sync_queue": 2,
        "det_score_thresh": 0.2,       # PEM 인스턴스 필터 (ISM 게이트와 무관)
        "status_period_s": 1.0,
        # 일꾼 타이머 주기. 짧을수록 처리 시작이 빨라지지만, 무거운 콜백이 도는 동안
        # executor 가 "준비된 타이머"를 반복해서 깨우며 헛도는 비용이 커진다.
        "worker_period_s": 0.005,
        # 파이썬 GC 를 적재 후 얼려 둔다. 모델·템플릿으로 만들어진 거대한 객체 그래프를
        # 세대 2 수집이 훑으면 GIL 을 쥔 채 수 초가 걸릴 수 있다(그동안 워커는 잠든다).
        "gc_freeze": False,
        # 진단: 다른 스레드들이 무엇을 하고 있는지 프로세스 **안에서** 표본추출한다
        # (py-spy 는 ptrace 권한이 필요해 못 쓴다). diagnostics 와 같이 켜야 동작.
        "sample_stacks": False,
        "executor_threads": 4,          # 진단용. 1 이면 다른 콜백이 처리 중 끼어들지 못한다
        "count_input_frames": True,    # 진단용 입력 카운터(끄면 30 Hz 역직렬화 1회분을 아낀다)
        # 한 프레임의 객체를 PEM 에 한 번에 태울지. false 면 객체를 하나씩 태운다
        # (예전 동작). 두 방식을 같은 조건에서 비교할 수 있도록 남겨 둔 스위치다.
        "pem_batch": True,
        # PEM 초기 포즈의 '승자 선택'을 겉모습으로 다시 세운다 (기본 OFF = 예전과 동일).
        # 현행은 후보 300 개를 관측점↔모델표면 **거리만으로** 채점해서, 앞뒤가 뒤집힌
        # 후보와 동점이 되면 난수가 승자를 정한다(프레임마다 포즈가 뒤집히는 원인).
        # 켜면 기하 상위 topk 개를 PEM 이 이미 계산한 특징(dense_fm↔dense_fo)의
        # 점 단위 코사인으로 재정렬한다. 추가 자산·재학습·렌더링 없음.
        #   측정(0807/185223, 기준자세 163건): 뒤집힘(>45°) 15%→9%, 15° 이내 52%→60%,
        #   뒤집힌 것의 48~52% 회복, 멀쩡한 것 파손 1~2%, PEM 시간 +8% 이내.
        # 값: null(끄기) 또는 {topk: 50, w_geo: 0.5, stride: 4}
        "appe_rerank": None,
        # 텍스처 검증(Stage V)은 **기본 ON** 이다. 기하 점수로 정렬한 상위 후보를 '입력
        # 이미지의 텍스처'와 '후보 포즈가 주장하는 모델 표면의 텍스처'로 맞대어 뒤집힘
        # 여부를 판정하고, 텍스처가 지지하는 후보로 승자를 교정한다. 판정(PASS/
        # CORRECTED/AMBIGUOUS)과 신뢰도는 검출 행에 verify 로 남는다.
        # 여기에 키를 두지 않는 것이 중요하다 — 키가 없어야 verify_config 의 기본값이
        # 적용된다. 끄려면 config 에 `verify: null` 또는 `verify: {enabled: false}`.
    },
    "output": {
        "dir": "output/rt_run",
        "overlay_every": 0,            # N 프레임마다 오버레이 PNG (0 = 저장 안 함)
        "write_jsonl": True,
        # 진단 모드: 프레임마다 (a) 시작/완료 시각을 bag 시각으로, (b) PEM 에 실제로
        # 들어간 입력의 품질(마스크 넓이·유효 depth 비율·거리·점 수), (c) 마스크 라벨
        # PNG 를 남긴다. 사후에 "왜 포즈가 흔들리나"를 재현·검증하기 위한 것이며
        # 판정에는 아무 영향이 없다. 기본 OFF (마스크 PNG 저장 비용 ~1 ms/프레임).
        "diagnostics": False,
    },
}


class Sam6DRealtimeNode(Node):
    def __init__(self, cfg: dict):
        super().__init__(
            "sam6d_realtime",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL,
                                           bool(cfg["runtime"]["use_sim_time"]))],
        )
        self.cfg = cfg
        self.device = cfg["runtime"]["device"] if torch.cuda.is_available() else "cpu"
        self.det_thresh = float(cfg["runtime"]["det_score_thresh"])
        self.pem_batch = bool(cfg["runtime"].get("pem_batch", True))
        self.out_dir = _abs(cfg["output"]["dir"])
        os.makedirs(self.out_dir, exist_ok=True)
        self.overlay_every = int(cfg["output"]["overlay_every"])
        if self.overlay_every:
            os.makedirs(os.path.join(self.out_dir, "overlay"), exist_ok=True)
        self.diag = bool(cfg["output"].get("diagnostics", False))
        if self.diag:
            os.makedirs(os.path.join(self.out_dir, "masks"), exist_ok=True)
        self.bridge = CvBridge()
        self.K = None
        self.busy = False
        self.n_in = 0            # 들어온 RGB 장수
        self.n_proc = 0          # 실제 처리한 장수
        self.n_drop = 0          # 처리 중이라 버린 쌍
        self.n_det = 0
        self.t_start = time.monotonic()
        self.last = {}
        self._newest_stamp = None
        self._pending = None     # 처리 대기 중인 '가장 최신' (RGB, depth) 쌍

        t0 = time.time()
        self._load_ism()
        self._load_pem()
        self.get_logger().info(f"[load] 전체 {time.time() - t0:.1f}s")

        # ---- 산출물 ----
        self._f_det = self._f_frm = None
        if cfg["output"]["write_jsonl"]:
            self._f_det = open(os.path.join(self.out_dir, "detections.jsonl"), "w", encoding="utf-8")
            self._f_frm = open(os.path.join(self.out_dir, "frames.jsonl"), "w", encoding="utf-8")
        self._write_meta()

        # ---- ROS I/O ----
        # 짝짓기 큐는 얕으면 안 된다. 깊이 1 이면 RGB·depth 각각 '가장 최신 한 장'만 남는데,
        # 무거운 처리가 도는 동안 둘이 서로 다른 프레임으로 덮어써지기 쉽다. 스탬프가 33 ms
        # 어긋난 짝은 slop 에 걸려 버려지고, 그래서 우연히 같은 프레임이 남았을 때만 처리된다
        # (실측: 처리 시간 152 ms 인데 처리량이 1 Hz — 나머지는 놀고 있었다).
        # 최신 프레임만 처리한다는 정책은 큐가 아니라 아래 _pending(항상 덮어쓰기)이 담당한다.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        par = ReentrantCallbackGroup()      # 무거운 콜백이 도는 동안에도 상태/최신스탬프는 살아 있어야 한다
        t = cfg["topics"]
        self.pub_det = self.create_publisher(Detection3DArray, "/sam6d/detections", 10)
        self.pub_status = self.create_publisher(String, "/sam6d/status", 10)
        self.create_subscription(CameraInfo, t["caminfo"], self._on_caminfo, qos, callback_group=par)
        # 감시용 구독은 큐를 깊게 둔다. 깊이 1 이면 처리 중 도착한 프레임이 미들웨어에서
        # 덮어써져 "몇 장이 들어왔는지" 자체를 셀 수 없다(=건너뛴 장수를 모른다).
        # 이 콜백은 스탬프만 적으므로 깊어도 비용이 없다.
        # 이 구독은 '몇 장이 들어왔나'를 세기 위한 진단용이다. 대신 30 Hz 영상을 한 번 더
        # 역직렬화하므로(프레임당 900 KB) 무거운 처리와 CPU/GIL 을 다툰다. 성능이 아쉬우면 끈다.
        if bool(self.cfg["runtime"].get("count_input_frames", True)):
            self.create_subscription(
                Image, t["rgb"], self._on_rgb_seen,
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=60),
                callback_group=par)
        self.sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, t["rgb"], qos_profile=qos),
             Subscriber(self, Image, t["depth"], qos_profile=qos)],
            queue_size=int(cfg["runtime"]["sync_queue"]),
            slop=float(cfg["runtime"]["sync_slop"]))
        # 짝이 맞으면 '가장 최신 한 쌍'으로 덮어써 두기만 한다(가볍다). 실제 처리는 아래
        # 일꾼 타이머가 한다 — 그래야 무거운 처리 중에도 짝짓기가 계속 돌아 최신 프레임이
        # 갱신되고, 처리가 끝나는 순간 '그 시점의 최신 프레임'을 집어 갈 수 있다.
        self.sync.registerCallback(self._on_pair)
        self.create_timer(float(cfg["runtime"].get("worker_period_s", 0.005)), self._work,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(float(cfg["runtime"]["status_period_s"]), self._publish_status,
                          callback_group=par)
        if self.diag and bool(cfg["runtime"].get("sample_stacks", False)):
            self._start_sampler()
        if bool(cfg["runtime"].get("gc_freeze", False)):
            import gc
            gc.collect()
            gc.freeze()                 # 적재된 객체들을 영구 세대로 옮겨 수집 대상에서 뺀다
            gc.disable()                # 참조 카운팅은 그대로 동작한다(순환 참조만 안 치운다)
            self.get_logger().info("[gc] 적재 후 freeze + disable")
        self._publish_status()          # 준비 완료 신호 (런치의 ready 게이트가 이걸 기다린다)
        self.get_logger().info("[ready] RGB + aligned depth 대기 중 ...")

    def _start_sampler(self):
        """50 ms 마다 모든 스레드의 스택 꼭대기를 세어 둔다. 정체 구간에 CPU 를 쓰는
        스레드가 무엇인지 **코드 위치**로 알아내기 위한 것이다(py-spy 대용)."""
        import collections, sys as _s, threading
        self._stk = collections.Counter()
        self._stk_slow = collections.Counter()
        self._worker_tid = None
        self._stop_sampler = False

        def loop():
            while not self._stop_sampler:
                try:
                    slow = self.busy and (time.perf_counter() - getattr(self, "_t_busy", 0)) > 0.7
                    for tid, fr in _s._current_frames().items():
                        if tid == self._worker_tid:
                            continue
                        key = (f"{os.path.basename(fr.f_code.co_filename)}:"
                               f"{fr.f_lineno}:{fr.f_code.co_name}")
                        self._stk[key] += 1
                        if slow:
                            self._stk_slow[key] += 1
                except Exception:
                    pass
                time.sleep(0.05)

        threading.Thread(target=loop, daemon=True).start()
        self.get_logger().info("[diag] 스레드 스택 표본추출 시작 (50 ms)")

    # ------------------------------------------------------------------ 적재
    def _load_ism(self):
        cfg_path = _abs(self.cfg["ism"]["config"])
        defaults, objs = o_n.load_config(cfg_path)
        want = list(self.cfg["ism"]["objects"] or [])
        if want:
            objs = [o for o in objs if o["name"] in want]
            missing = sorted(set(want) - {o["name"] for o in objs})
            if missing:
                raise SystemExit(f"[config] 요청한 객체가 config 에 없다: {missing}")
        self.get_logger().info(f"[load] DINOv2 + 템플릿 특징 ({len(objs)} 객체) ...")
        self.model = yi.build_dinov2(
            defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, self.device)
        self.objs = o_n.prepare_objects(objs, self.model, self.device, rebuild=False)

        # HSV 게이트는 캐시가 없으면 조용히 fail-open 한다(점수 1.0 = 절대 거절 안 함).
        # 이식하면서 템플릿 mtime 이 바뀌면 그대로 당하므로, 여기서 시끄럽게 죽인다.
        stale = [o["name"] for o in self.objs
                 if o.get("hsv_gate_enabled") and o.get("_hsv_proto") is None]
        if stale:
            raise SystemExit(
                "[hsv] HSV 게이트가 켜져 있는데 기준값 캐시가 무효다: " + ", ".join(stale) +
                "\n      (캐시는 템플릿 파일의 크기·mtime 지문을 검사한다. 복사할 때 mtime 이"
                "\n       바뀌면 stale 이 된다 — tar 나 rsync -a 로 옮길 것.)"
                "\n      복구: python tools/build_hsv_template_cache.py --force")

        self.segmentor = yi.build_segmentor(
            o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), self.device)
        self.pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
        self.unique_prompts, self.groups = o_n.build_prompt_groups(self.objs)
        # 색 타이브레이크가 "형태로 구분 안 되는 짝"을 찾는 데 쓰는 템플릿 유사도 행렬.
        # 프레임마다 다시 계산하면 낭비라 한 번만 만든다(오프라인 배치도 동일).
        self.tsim = o_n.template_similarity(self.objs)
        self.min_score = min(float(o.get("score_threshold", 0.02)) for o in self.objs)
        self.imgsz = int(defaults.get("imgsz", 640))
        self.multi_label = o_n._multi_label_on(self.objs[0])

        # YOLO-World 의 CLIP 텍스트 인코더(354 MB)는 기본적으로 `<현재 폴더>/weights/clip` 에
        # **인터넷에서 내려받는다**. 오프라인 노트북에서는 거기서 죽으므로, 같이 들고 온
        # assets/clip/ViT-B-32.pt 를 보도록 고정한다(ultralytics 가 이름으로 import 해 가므로
        # utils 쪽이 아니라 text_model 모듈의 상수를 바꿔야 한다).
        import ultralytics.nn.text_model as _tm
        _tm.WEIGHTS_DIR = Path(os.path.join(REPO, "assets"))

        from ultralytics import YOLOWorld
        weights = o_n._abspath(defaults.get("weights", "yolov8m-worldv2.pt"))
        self.yolo = YOLOWorld(weights)
        self.yolo.set_classes(self.unique_prompts)
        self._warmup()
        self.get_logger().info(
            f"[load] YOLO-World {os.path.basename(weights)} imgsz={self.imgsz} "
            f"multi_label={self.multi_label} 프롬프트 {len(self.unique_prompts)}개")
        self.get_logger().info(
            f"[ism] relative_assignment={bool(self.objs[0].get('relative_assignment_enabled'))} "
            f"colour_tiebreak={self.objs[0].get('color_tiebreak_template_sim')} "
            f"cross_object_nms={bool(self.objs[0].get('cross_object_nms_enabled'))} "
            f"hsv_gate={bool(self.objs[0].get('hsv_gate_enabled'))} "
            f"appe_blocks={o_n._blocks_of(self.objs[0])}@{o_n._appe_gate_of(self.objs[0])}")

    def _warmup(self, size=(480, 640)):
        """가짜 프레임 한 장을 사슬에 통과시킨다.

        첫 호출에는 CUDA 커널 컴파일과 cuDNN 자동튜닝이 몰려서, 예열하지 않으면
        **첫 프레임만** YOLO 5.3s + ISM 4.4s 로 튄다(실측). 여기서 미리 태워 두면
        첫 실물 프레임부터 정상 속도로 들어간다. 덤으로, 템플릿 특징·마스크 경로가
        깨져 있으면 실물 프레임이 아니라 기동 시점에 드러난다.
        """
        t0 = time.time()
        try:
            bgr = np.full((size[0], size[1], 3), 127, np.uint8)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            with o_n.multi_label_nms(self.multi_label):
                self.yolo.predict(bgr, conf=self.min_score, imgsz=self.imgsz,
                                  verbose=False, device=self.device)
            pb = {i: [] for i in range(len(self.unique_prompts))}
            pb[0] = [([size[1] // 4, size[0] // 4, size[1] // 2, size[0] // 2], 0.5)]
            o_n.recognize_frame_auto(self.groups, pb, bgr, rgb, yi.normalize_rgb(rgb),
                                     self.model, self.device, self.segmentor,
                                     self.pool, self.tsim)
            self.get_logger().info(f"[load] 예열(YOLO+DINOv2+MobileSAM) {time.time() - t0:.1f}s")
        except Exception as e:      # 예열 실패가 노드를 죽일 이유는 없다
            self.get_logger().warn(f"[load] 예열 실패({type(e).__name__}: {e}) — 첫 프레임이 느릴 수 있다")

    def _load_pem(self):
        cwd = os.getcwd()
        os.chdir(PEM_DIR)                      # MAE 체크포인트 경로가 PEM 기준 상대경로다
        for sub in ("provider", "utils", "model", os.path.join("model", "pointnet2")):
            sys.path.append(os.path.join(PEM_DIR, sub))
        sys.path.insert(0, PEM_DIR)
        import gorilla                                            # noqa: E402
        import importlib
        import run_inference_custom as ric                        # noqa: E402
        self.ric = ric
        self.pcfg = gorilla.Config.fromfile(os.path.join(PEM_DIR, "config", "base.yaml"))
        self.pcfg.model_name = "pose_estimation_model"
        # 외형 재정렬 옵션을 PEM 모델 config 에 주입한다(base.yaml 은 건드리지 않는다).
        ar, ver = build_appe_cfg(self.cfg["runtime"])
        self.verify = ver or {}
        if ar:
            self.pcfg.model.appe_rerank = ar
        self.get_logger().info(describe(ar, ver))
        MODEL = importlib.import_module(self.pcfg.model_name)
        self.pem = MODEL.Net(self.pcfg.model).to(self.device).eval()
        gorilla.solver.load_checkpoint(
            model=self.pem, filename=os.path.join(PEM_DIR, "checkpoints", "sam-6d-pem-base.pth"))

        # CAD 대체: trimesh.load_mesh 를 '객체 이름 -> 미리 뽑아둔 8192점' 조회로 바꾼다.
        # (오프라인 run_pem_batch 도 같은 스텁을 쓴다. 다른 점은 키가 파일 경로가 아니라
        #  객체 이름이라는 것뿐 — 그래야 다른 노트북으로 옮겨도 캐시가 유효하다.)
        pts_dir = os.path.join(REPO, "assets", "model_points")
        tem_dir = os.path.join(REPO, "assets", "pem_templates")
        self._pts, self._tem, self._extent = {}, {}, {}

        class _Stub:
            def __init__(s, pts): s._pts = pts
            def sample(s, n):
                if n == len(s._pts):
                    return s._pts
                idx = np.random.choice(len(s._pts), n, replace=(n > len(s._pts)))
                return s._pts[idx]

        for o in self.objs:
            name = o["name"]
            p_pts, p_tem = os.path.join(pts_dir, f"{name}.npy"), os.path.join(tem_dir, f"{name}.pt")
            for p in (p_pts, p_tem):
                if not os.path.isfile(p):
                    raise SystemExit(f"[pem] {name}: 자산이 없다 — {p}")
            pts = np.load(p_pts).astype(np.float32)
            self._pts[name] = _Stub(pts)
            canonical_pts = canonical_object_points(name, pts)
            self._extent[name] = np.ptp(canonical_pts, axis=0) / 1000.0  # BoundingBox3D 용 [m]
            blob = torch.load(p_tem, map_location=self.device)
            tc = blob.get("tc")          # 점별 색 (tools/add_template_colors.py 로 추가)
            if tc is None and self.verify.get("enabled") and self.verify.get("w_col"):
                self.get_logger().warn(
                    f"[pem] {name}: 템플릿에 색(tc)이 없다 — 원색 채널 없이 검증한다. "
                    f"python tools/add_template_colors.py --objects {name}")
            self._tem[name] = (blob["tp"].to(self.device), blob["tf"].to(self.device),
                               None if tc is None else tc.to(self.device).float())
        self.ric.trimesh.load_mesh = lambda key, *a, **k: self._pts[key]
        os.chdir(cwd)
        self.get_logger().info(f"[load] PEM 템플릿·모델점 {len(self._tem)} 객체 (CAD 불필요)")

    def _write_meta(self):
        meta = {
            "started_wall": time.time(),
            "repo": REPO,
            "device": self.device,
            "objects": [o["name"] for o in self.objs],
            "prompts": self.unique_prompts,
            "ism": {
                "imgsz": self.imgsz,
                "multi_label": bool(self.multi_label),
                "relative_assignment": bool(self.objs[0].get("relative_assignment_enabled")),
                "color_tiebreak_template_sim": self.objs[0].get("color_tiebreak_template_sim"),
                "cross_object_nms": bool(self.objs[0].get("cross_object_nms_enabled")),
                "hsv_gate": bool(self.objs[0].get("hsv_gate_enabled")),
                "hsv_threshold": self.objs[0].get("hsv_gate_threshold"),
                "appe_blocks": o_n._blocks_of(self.objs[0]),
                "appe_gate": o_n._appe_gate_of(self.objs[0]),
            },
            "config": self.cfg,
        }
        with open(os.path.join(self.out_dir, "run_meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=1, ensure_ascii=False)

    # ------------------------------------------------------------------ 콜백
    def _on_caminfo(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f"[caminfo] fx={self.K[0, 0]:.1f} 수신")

    def _on_rgb_seen(self, msg):
        """가장 최신 프레임이 언제 것인지만 기록한다(뒤처짐 측정용, 처리 안 함)."""
        self.n_in += 1
        self._newest_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_pair(self, rgb_msg, depth_msg):
        """짝이 맞은 쌍을 '대기 중 최신'으로 덮어쓰기만 한다 (여기서 처리하지 않는다)."""
        if self._pending is not None:
            self.n_drop += 1              # 처리되기 전에 더 새 프레임이 왔다 = 건너뜀
        self._pending = (rgb_msg, depth_msg)

    def _work(self):
        """★ 항상 '지금 대기 중인 가장 최신 프레임'을 처리한다."""
        if self.busy or self.K is None or self._pending is None or not rclpy.ok():
            return
        rgb_msg, depth_msg = self._pending
        self._pending = None
        self.busy = True
        self._t_busy = time.perf_counter()
        if getattr(self, "_stk", None) is not None and self._worker_tid is None:
            import threading
            self._worker_tid = threading.get_ident()
        try:
            self._process(rgb_msg, depth_msg)
        except Exception as e:            # 한 프레임의 실패가 노드를 죽이면 안 된다
            self.get_logger().error(f"프레임 처리 실패: {type(e).__name__}: {e}")
        finally:
            self.busy = False

    # ------------------------------------------------------------------ 본체
    def _process(self, rgb_msg, depth_msg):
        _sync()
        t0 = time.perf_counter()
        t_start_ros = self.get_clock().now().nanoseconds     # bag 시각 (use_sim_time)
        # 진단: 벽시계와 **GPU 실행시간**을 따로 잰다. 둘이 크게 다르면 그 구간은
        # 계산이 아니라 대기(할당기·GIL·다른 프로세스)다. 어느 단계 탓인지 가르는 핵심.
        ev = None
        if self.diag and torch.cuda.is_available():
            ev = [torch.cuda.Event(enable_timing=True) for _ in range(4)]
            ev[0].record()
            self._m0 = torch.cuda.memory_stats()
        # 스레드 CPU 시간. 벽시계와 같으면 이 스레드가 **CPU 를 쓰느라** 오래 걸린 것이고,
        # 훨씬 작으면 **기다린** 것이다(GPU 대기·GIL·락). CUDA 이벤트로는 이 둘을 못 가른다.
        c = [time.thread_time()] if self.diag else None
        if self.diag:
            import resource
            self._ru0 = resource.getrusage(resource.RUSAGE_SELF)
            self._th0 = _thread_cpu()      # 스레드별 CPU 시간 (누가 태우는지 이름까지)
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = bgr.shape[:2]

        # --- 1) YOLO-World 제안 (multi_label: 한 박스가 여러 프롬프트를 유지) ---
        with o_n.multi_label_nms(self.multi_label):
            res = self.yolo.predict(bgr, conf=self.min_score, imgsz=self.imgsz,
                                    verbose=False, device=self.device)
        _sync()                       # CUDA 는 비동기다. 동기화 없이 재면 앞 단계가 과소,
        t_yolo = time.perf_counter()
        if c is not None:
            c.append(time.thread_time())
        if ev is not None:
            ev[1].record()                # 뒤 단계가 과대 계상된다(실제로 그렇게 속았다).
        prompt_boxes = {i: [] for i in range(len(self.unique_prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                ci = int(b.cls[j]) if b.cls is not None else 0
                if ci in prompt_boxes:
                    prompt_boxes[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
        for k in prompt_boxes:
            prompt_boxes[k].sort(key=lambda t: t[1], reverse=True)
        # 상대 할당은 '박스마다 전 객체와 비교'라 ISM 시간이 박스 수에 비례한다.
        # 프레임 시간이 튈 때 원인을 짚을 수 있도록 그 수를 남긴다.
        n_boxes = sum(len(v) for v in prompt_boxes.values())

        # --- 2) ISM: 오프라인 배치와 같은 프레임 단위 판정 ---
        #     relative assignment + 색 타이브레이크 + cross-object NMS 가 전부 이 안에 있다.
        norm_full = yi.normalize_rgb(rgb)
        results = o_n.recognize_frame_auto(self.groups, prompt_boxes, bgr, rgb, norm_full,
                                           self.model, self.device, self.segmentor,
                                           self.pool, self.tsim)
        _sync()
        t_ism = time.perf_counter()
        if c is not None:
            c.append(time.thread_time())
        if ev is not None:
            ev[2].record()

        # --- 3) PEM: 수락된 객체마다 6D 포즈 ---
        # 프레임에서 '객체와 무관한 부분'(깊이 스케일·전체 포인트클라우드)은 여기서
        # 한 번만 만든다. 예전에는 프레임을 PNG 로 써 두고 객체마다 get_test_data 가
        # 그걸 다시 읽어 포인트클라우드를 새로 만들었다 — 객체 수만큼 반복되던 일이다.
        # 수락된 객체가 없으면 아예 만들지 않는다.
        self._t_frame = self._t_prep = self._t_fwd = 0.0
        hits = [(n, r) for n, r in sorted(results.items())
                if r.get("accepted") and r.get("mask") is not None]
        frame = None
        if hits:
            _t = time.perf_counter()
            frame = self.ric.prepare_frame(rgb, depth, self.K)   # rgb 는 RGB 순서
            self._t_frame = time.perf_counter() - _t

        stamp_ns = rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec
        arr = Detection3DArray()
        arr.header = rgb_msg.header
        drawn, rows = [], []
        overlay = rgb.copy() if (self.overlay_every and
                                 (self.n_proc + 1) % self.overlay_every == 0) else None

        for name, r, R, t_mm, score, mpts, dg, vf in self._pem_poses(frame, hits, depth):
            arr.detections.append(self._to_msg(rgb_msg.header, name, R, t_mm, score, vf))
            rows.append({
                "stamp_ns": stamp_ns, "frame_seq": self.n_proc, "object": name,
                "score": round(float(score), 5),
                "R": [[round(float(v), 6) for v in row] for row in R],
                "t_mm": [round(float(v), 4) for v in t_mm],
                "bbox": [int(v) for v in r["box"]],
                **({"pem": dg} if self.diag else {}),
                **({"verify": vf} if vf else {}),
                "rank_geo": (vf or {}).get("rank_geo"),
                "mask_iou": (((vf or {}).get("fine") or {}).get("mask_iou")),
                "texture_score": (((vf or {}).get("fine") or {}).get("texture_score")),
                "cluster_occupancy": (vf or {}).get("cluster_occupancy"),
                "pose_source": "sam6d", "map_id": None,
                "anchor_state": "disabled", "rejection_reason": None,
                "ism": {k: (round(float(r[k]), 5) if isinstance(r.get(k), (int, float)) else r.get(k))
                        for k in ("best_yolo", "best_sem", "masked_appe", "rank_appe",
                                  "hsv_score", "decision") if k in r},
            })
            if overlay is not None:
                col = self._color(name)
                overlay = self.ric.draw_detections(overlay, R[None], t_mm[None],
                                                   mpts * 1000.0, self.K[None], color=col)
                drawn.append((name, float(score), col))
        _sync()
        t_pem = time.perf_counter()
        if c is not None:
            c.append(time.thread_time())
        if ev is not None:
            ev[3].record()
        if self.diag and hits:
            lab = np.zeros((h, w), np.uint8)
            for i, (name, r) in enumerate(hits):
                lab[r["mask"].astype(bool)] = i + 1
            cv2.imwrite(os.path.join(self.out_dir, "masks", f"{stamp_ns}.png"), lab)

        self.pub_det.publish(arr)
        self.n_proc += 1
        self.n_det += len(arr.detections)
        lag = None
        if self._newest_stamp is not None:
            lag = self._newest_stamp - (stamp_ns * 1e-9)     # 최신 프레임보다 얼마나 뒤처졌나 [s]
        self.last = {
            "stamp_ns": stamp_ns, "n_accept": len(arr.detections), "n_boxes": n_boxes,
            "lag_s": round(lag, 3) if lag is not None else None,
            "ms": {"yolo": round(1e3 * (t_yolo - t0)), "ism": round(1e3 * (t_ism - t_yolo)),
                   "pem": round(1e3 * (t_pem - t_ism)), "total": round(1e3 * (t_pem - t0)),
                   "pem_frame": round(1e3 * self._t_frame), "pem_prep": round(1e3 * self._t_prep),
                   "pem_fwd": round(1e3 * self._t_fwd)},
        }
        if self._f_det:
            for row in rows:
                self._f_det.write(json.dumps(row, ensure_ascii=False) + "\n")
            self._f_det.flush()
        if self.diag:
            import resource, os as _os
            ru = resource.getrusage(resource.RUSAGE_SELF)
            self.last["sched"] = {
                # 프로세스 전체 CPU 시간. 워커는 자고 있는데 이 값이 벽시계만큼 늘면
                # **다른 스레드가 GIL 을 쥐고 돌고 있는 것**이고, 이 값도 0 이면
                # 프로세스 전체가 커널/미들웨어를 기다린 것이다. 둘을 가르는 핵심.
                "proc_cpu_ms": round(1e3 * ((ru.ru_utime + ru.ru_stime)
                                            - (self._ru0.ru_utime + self._ru0.ru_stime))),
                # 비자발적 문맥교환 = 커널이 이 프로세스를 강제로 재웠다(=CPU 경쟁)
                "nivcsw": ru.ru_nivcsw - self._ru0.ru_nivcsw,
                "nvcsw": ru.ru_nvcsw - self._ru0.ru_nvcsw,
                "majflt": ru.ru_majflt - self._ru0.ru_majflt,
                "load1": round(_os.getloadavg()[0], 1)}
            th1 = _thread_cpu()
            d = {k: round(1e3 * (v - self._th0.get(k, v))) for k, v in th1.items()}
            self.last["threads"] = {k: v for k, v in
                                    sorted(d.items(), key=lambda x: -x[1])[:6] if v > 0}
        if c is not None and len(c) == 4:
            self.last["cpu_ms"] = {"yolo": round(1e3 * (c[1] - c[0])),
                                   "ism": round(1e3 * (c[2] - c[1])),
                                   "pem": round(1e3 * (c[3] - c[2]))}
        if ev is not None:
            torch.cuda.synchronize()
            m1 = torch.cuda.memory_stats()
            self.last["gpu_ms"] = {"yolo": round(ev[0].elapsed_time(ev[1])),
                                   "ism": round(ev[1].elapsed_time(ev[2])),
                                   "pem": round(ev[2].elapsed_time(ev[3]))}
            g = lambda k: int(m1.get(k, 0)) - int(self._m0.get(k, 0))
            self.last["alloc"] = {"retries": g("num_alloc_retries"),
                                  "dev_alloc": g("num_device_alloc"),
                                  "dev_free": g("num_device_free"),
                                  "sync": g("num_sync_all_streams"),
                                  "reserved_mb": round(m1["reserved_bytes.all.current"] / 1e6)}
        if self._f_frm:
            self._f_frm.write(json.dumps(
                {**self.last, "frame_seq": self.n_proc - 1,
                 "objects": [d["object"] for d in rows],
                 # 이 결과가 '언제부터 화면에 뜰 수 있는가' — bag 시각 기준.
                 "t_start_ns": t_start_ros,
                 "t_done_ns": self.get_clock().now().nanoseconds,
                 "frames_in": self.n_in,
                 "skipped_total": max(0, self.n_in - self.n_proc)}, ensure_ascii=False) + "\n")
            self._f_frm.flush()
        if overlay is not None and drawn:
            self._save_overlay(overlay, drawn)
        self.get_logger().info(
            f"#{self.n_proc - 1} {len(arr.detections)}개 "
            f"({', '.join(d['object'] for d in rows) or '-'})  "
            f"{self.last['ms']['total']}ms [yolo {self.last['ms']['yolo']} / "
            f"ism {self.last['ms']['ism']} / pem {self.last['ms']['pem']}]  "
            f"drop {self.n_drop}")

    def _pem_poses(self, frame, hits, depth=None):
        """수락된 객체들의 6D 포즈.

        pem_batch 면 한 프레임의 객체를 PEM 에 **한 번에** 태운다. 객체마다 따로
        태우면 GPU 는 노는데 커널 호출만 객체당 2850 개씩 쌓인다(실측: B=1 28 ms,
        B=4 는 객체당 17 ms). 배치는 그 고정비를 여러 객체가 나눠 쓰는 것이다.
        pem_batch: false 면 예전처럼 하나씩 태운다 — 비교용으로 남겨 둔 경로다.
        """
        rows = []
        if not hits:
            return rows
        step = len(hits) if self.pem_batch else 1
        for i in range(0, len(hits), step):
            chunk = hits[i:i + step]
            try:
                rows += self._pem_chunk(frame, chunk, depth)
            except Exception as e:
                self.get_logger().warn(
                    f"PEM {[n for n, _ in chunk]} 건너뜀: {type(e).__name__}: {e}")
        return rows

    def _pem_chunk(self, frame, chunk, depth=None):
        # 마스크를 그대로 넘긴다. 예전에는 RLE 로 인코딩해 JSON 으로 쓰고 PEM 이 다시
        # 디코딩했다 — 왕복이 무손실이라 결과는 같고 시간만 들던 구간이다.
        _t = time.perf_counter()
        dets = [{"score": 1.0, "mask": r["mask"].astype(bool), "cad": name}
                for name, r in chunk]            # ← CAD 경로 자리에 객체 이름(스텁이 조회)
        inp, mpts_list, used = self.ric.get_instance_data(
            frame, None, dets, self.det_thresh, self.pcfg.test_dataset)
        _sync(); self._t_prep += time.perf_counter() - _t; _t = time.perf_counter()
        names = [d["cad"] for d in used]         # 마스크가 너무 작아 빠진 객체는 여기 없다
        with torch.inference_mode():
            inp["dense_po"] = torch.cat([self._tem[n][0] for n in names], 0)
            inp["dense_fo"] = torch.cat([self._tem[n][1] for n in names], 0)
            if all(self._tem[n][2] is not None for n in names):
                inp["dense_co"] = torch.cat([self._tem[n][2] for n in names], 0)
            inp["obj_names"] = names                  # 대칭 선언 조회용
            out = self.pem(inp)
        _sync(); self._t_fwd += time.perf_counter() - _t

        coarse = out["score"].detach().cpu().numpy()
        pose_s = (out["pred_pose_score"].detach().cpu().numpy()
                  if "pred_pose_score" in out else None)
        ps = coarse * pose_s if pose_s is not None else coarse
        Rs = out["pred_R"].detach().cpu().numpy()
        ts = out["pred_t"].detach().cpu().numpy() * 1000.0
        vfs = out.get("verify") or [None] * len(names)
        by_name = dict(chunk)
        rows = []
        for j, name in enumerate(names):
            r = by_name[name]
            vf = dict(vfs[j] or {})
            if self.verify.get("enabled") and not vf.get("accepted", False):
                self.get_logger().warn(
                    f"[reject] {name}: {vf.get('rejection_reason', 'candidate_verification_failed')} "
                    f"rank_geo={vf.get('rank_geo')} mask={((vf.get('fine') or {}).get('mask_iou'))} "
                    f"texture={((vf.get('fine') or {}).get('texture_score'))} "
                    f"occupancy={vf.get('cluster_occupancy')}")
                continue
            dg = None
            if self.diag:
                # PEM 이 실제로 본 입력의 품질. 흔들림을 마스크 탓/거리 탓/점수 탓으로
                # 나눠 보려면 포즈만으로는 부족하고 이 값들이 있어야 한다.
                mask = r["mask"].astype(bool)
                area = int(mask.sum())
                valid = int((mask & (depth > 0)).sum()) if depth is not None else -1
                dz = depth[mask & (depth > 0)] if depth is not None else np.zeros(0)
                dg = {"mask_px": area, "depth_valid_px": valid,
                      "depth_valid_frac": round(valid / max(area, 1), 4),
                      "z_med_mm": round(float(np.median(dz)), 1) if dz.size else None,
                      "n_pts_in": int(inp["pts"].shape[1]),   # PEM 이 실제로 쓴 관측점 수(고정 2048)
                      "coarse": round(float(coarse[j]), 5),
                      "pose_score": round(float(pose_s[j]), 5) if pose_s is not None else None,
                      "batch": len(names)}
            rows.append((name, r, Rs[j], ts[j], float(ps[j]), mpts_list[j], dg, vf))
        return rows

    def _to_msg(self, header, name, R, t_mm, score, verify=None) -> Detection3D:
        d = Detection3D()
        d.header = header
        d.id = name
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(t_mm[0] / 1000.0),
                                                    float(t_mm[1] / 1000.0),
                                                    float(t_mm[2] / 1000.0))
        q = Rotation.from_matrix(canonical_object_rotation(name, R)).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = name
        hyp.hypothesis.score = float(score)
        hyp.pose.pose = p
        d.results = [hyp]
        # 검증 신뢰도는 results[0] 을 건드리지 않고 뒤에 덧붙인다 — 하류(ObjectMemory)는
        # results[0] 만 읽으므로 무해하고, score 의 의미가 바뀌지 않아 과거 실행과 비교된다.
        if verify and self.verify.get("publish"):
            v = ObjectHypothesisWithPose()
            v.hypothesis.class_id = "_verify"
            v.hypothesis.score = float(verify.get("conf", 0.0))
            v.pose.pose = p
            d.results.append(v)
        d.bbox.center = p
        ex = self._extent[name]
        d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = (float(ex[0]), float(ex[1]), float(ex[2]))
        return d

    # ------------------------------------------------------------------ 상태·출력
    def _publish_status(self):
        el = max(1e-6, time.monotonic() - self.t_start)
        payload = {
            "ready": True,
            "objects": [o["name"] for o in self.objs],
            "frames_in": self.n_in,
            "frames_processed": self.n_proc,
            # 건너뛴 장수 = 들어온 것 - 처리한 것. 대부분은 미들웨어(큐 깊이 1)가 버리므로
            # 콜백까지 온 뒤 busy 라서 버린 n_drop 만 세면 0 으로 보인다.
            "frames_skipped": max(0, self.n_in - self.n_proc),
            "frames_dropped_busy": self.n_drop,
            "detections_total": self.n_det,
            "hz_in": round(self.n_in / el, 2),
            "hz_processed": round(self.n_proc / el, 2),
            "camera_info": self.K is not None,
            "last": self.last,
        }
        self.pub_status.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    _COLORS = [(0, 0, 255), (0, 200, 0), (255, 0, 0), (0, 200, 255),
               (255, 0, 255), (255, 200, 0), (128, 0, 255), (0, 128, 255)]

    def _color(self, name):
        return self._COLORS[sorted(o["name"] for o in self.objs).index(name) % len(self._COLORS)]

    def _save_overlay(self, rgb_img, drawn):
        out = rgb_img.copy()
        y = 16
        for name, sc, col in drawn:
            cv2.rectangle(out, (6, y - 10), (20, y), col, -1)
            cv2.putText(out, f"{name} {sc:.2f}", (24, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)
            y += 18
        cv2.imwrite(os.path.join(self.out_dir, "overlay", f"frame_{self.n_proc:06d}.png"),
                    cv2.cvtColor(out, cv2.COLOR_RGB2BGR))

    def close(self):
        el = max(1e-6, time.monotonic() - self.t_start)
        summary = {"frames_in": self.n_in, "frames_processed": self.n_proc,
                   "frames_skipped": max(0, self.n_in - self.n_proc),
                   "frames_dropped_busy": self.n_drop, "detections": self.n_det,
                   "elapsed_s": round(el, 1), "hz_processed": round(self.n_proc / el, 2)}
        self.get_logger().info(f"[summary] {summary}")
        if getattr(self, "_stk", None) is not None:
            self._stop_sampler = True
            with open(os.path.join(self.out_dir, "stacks.json"), "w", encoding="utf-8") as f:
                json.dump({"all": self._stk.most_common(40),
                           "during_slow": self._stk_slow.most_common(40)}, f, indent=1)
        path = os.path.join(self.out_dir, "run_meta.json")
        try:
            with open(path, encoding="utf-8") as f:
                meta = json.load(f)
            meta["summary"] = summary
            with open(path, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=1, ensure_ascii=False)
        except OSError:
            pass
        for f in (self._f_det, self._f_frm):
            if f:
                f.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True, help="실행 설정 YAML")
    a, ros_args = ap.parse_known_args()
    with open(_abs(a.config), encoding="utf-8") as f:
        cfg = _deep_merge(DEFAULT_CFG, yaml.safe_load(f) or {})

    rclpy.init(args=ros_args)
    node = Sam6DRealtimeNode(cfg)
    ex = MultiThreadedExecutor(num_threads=int(cfg["runtime"].get("executor_threads", 4)))
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
