#!/usr/bin/env python3
"""2단계 — SAM 카메라 영상에서 물체를 찾고 6D 포즈를 추정한다 (SAM-6D).

    python integration/pipeline/sam6d.py 185223__circle_ccw_8laps

SAM-6D 자체는 SLAM 결과와 무관하므로 SLAM 3종보다 먼저 돌려도 되고 나중에 돌려도 된다.

    ISM (sam_yolo)          YOLO 후보 -> DINOv2/MobileSAM 으로 물체 판별 + 마스크
    PEM (sam6d_ros_humble)  물체마다 6D 포즈
    overlay                 프레임마다 3D 박스를 그린 이미지 (동영상 재료)
    lifecycle               프레임별 검출을 지속 객체로 묶어 '사라짐/이동' 을 갱신

산출물: <출력>/sam/objects.json           — 프레임별 (물체, 포즈, 점수) 목록
        <출력>/sam/lifecycle_<백엔드>.json — 지속 객체 + 프레임별 상태 타임라인
중간물: <출력>/_stage/  — PEM 입력·오버레이 이미지. **fuse_video.py 가 마지막에 지운다.**
        영상까지 만든 뒤에는 쓸모가 없고 세션당 500 MB 를 넘기 때문이다.

lifecycle 단계는 SLAM 궤적과 리그 extrinsic(converting_launch/rigs/<날짜>.urdf)이
있어야 돌아간다(객체를 map 프레임에
세워야 '같은 자리'를 말할 수 있다). 없으면 건너뛰고 이유를 남긴다 — SAM-6D 만 먼저
돌리는 기존 사용법을 깨지 않기 위해서다. objects.json 하나만 읽으므로 _stage 가
지워진 뒤에도 언제든 다시 돌릴 수 있다(--only-lifecycle).
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common as C  # noqa: E402

sys.path.insert(0, str(C.INTEGRATION_DIR / "src"))
import object_lifecycle as OL  # noqa: E402

RUN_INTEGRATION = C.INTEGRATION_DIR / "run_integration.py"
BACKENDS = ("orb3", "rtabmap", "hdl")


def run_lifecycle(data: Path, out: Path, backend: str, gate: float,
                  legacy_confidence: bool = False) -> Path | None:
    """objects.json 의 프레임별 검출을 지속 객체로 묶는다 (사라짐/이동 갱신).

    T_map_obj = T_map_camSLAM(t) · X · T_camSAM_obj 로 map 프레임에 세운 뒤,
    object_lifecycle 이 연관 -> 융합(이동) -> 존재확률 감쇠(사라짐) 를 프레임마다 돌린다.
    X 는 검출이 아니라 **포즈 스트림**에 곱한다 — 그래야 존재 필터의 P_D 가 실제로
    그 검출을 만든 카메라의 시야를 본다.
    """
    dets_json = out / "sam" / "objects.json"
    traj = out / backend / "trajectory.txt"
    info_json = data / "info.json"
    X, x_src = C.load_rig_extrinsic(data)

    if backend == "hdl":
        C.log("lifecycle 건너뜀: hdl 궤적은 LiDAR 프레임이다 — 카메라 프레임으로 옮기는 "
              "hand-eye 는 fuse_video.py 가 한다. orb3/rtabmap 을 써라.")
        return None
    for label, path in (("검출(objects.json)", dets_json), ("SLAM 궤적", traj),
                        ("info.json", info_json)):
        if not path.is_file():
            C.log(f"lifecycle 건너뜀: {label} 이 없다 ({path})")
            return None
    if X is None:
        C.log(f"lifecycle 건너뜀: 리그 extrinsic 을 못 찾았다 — {x_src}")
        return None

    with open(info_json, encoding="utf-8") as f:
        color_topic = (json.load(f).get("SAM", {}).get("color_topic")
                       or "/camera/camera/color/image_raw")
    poses = OL.load_slam_trajectory(str(traj), source_slam_id=backend)
    poses = [OL.SlamCameraPose(stamp=p.stamp,
                               T_map_cam=OL.compose_transform(p.T_map_cam, X),
                               source_slam_id=p.source_slam_id)
             for p in poses]
    timestamps = OL.load_frame_timestamps(str(data / "SAM"), color_topic)
    detections = OL.load_detections(str(dets_json), timestamps=timestamps)
    cam_K, img_size = OL.load_camera(str(info_json), which="SAM")
    n_det = sum(len(v) for v in detections.values())
    C.log(f"lifecycle({backend}): SLAM 포즈 {len(poses)} · SAM 프레임 {len(timestamps)} · "
          f"검출 {n_det}건/{len(detections)}프레임 · extrinsic {x_src}")
    if cam_K is None:
        C.log("  경고: info.json 에 SAM 카메라 K 가 없다 — P_D 가 원뿔 근사로 떨어진다")

    result = OL.run_object_lifecycle(
        poses, detections, timestamps,
        assoc_trans_gate_m=gate, assoc_rot_gate_deg=-1.0,
        cam_K=cam_K, img_size=img_size,
        quality_weighting=not legacy_confidence)

    payload = OL.result_to_dict(result)
    payload["inputs"] = {"detections": str(dets_json), "trajectory": str(traj),
                         "extrinsic": x_src, "backend": backend}
    payload["params"] = {"assoc_trans_gate_m": gate, "assoc_rot_gate_deg": -1.0,
                         "quality_weighting": not legacy_confidence,
                         "pd_base": OL.PD_BASE, "r_lost": OL.R_LOST,
                         "r_retire": OL.R_RETIRE,
                         "min_obs_longterm": OL.MIN_OBS_LONGTERM}
    dst = out / "sam" / f"lifecycle_{backend}.json"
    dst.write_text(json.dumps(payload, indent=1), encoding="utf-8")

    live = [o for o in payload["objects"] if o["status"] != "deleted"]
    C.log(f"lifecycle({backend}): 인스턴스 {len(payload['objects'])}개 중 {len(live)}개 생존")
    for o in live:
        C.log(f"    - #{o['object_id']:<3} {o['object_name']:<20} [{o['status']:<10}] "
              f"obs={o['observations']:<4} r={o['confidence']:.2f}  "
              f"xyz=({o['xyz'][0]:7.3f},{o['xyz'][1]:7.3f},{o['xyz'][2]:7.3f})")
    gone = [t for t in payload["transitions"]
            if t["reason"] in ("missed_threshold", "long_term_memory",
                               "low_evidence", "redetected")]
    C.log(f"lifecycle({backend}): 사라짐/재등장 이벤트 {len(gone)}건 → {dst}")
    return dst


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--stride", type=int, default=10,
                    help="SAM 카메라 N 프레임마다 하나 (기본 10 = 3 Hz)")
    ap.add_argument("--objects", nargs="+", default=[],
                    help="이 이름들만 (기본: configs/yolo_ism_objects.yaml 전부)")
    ap.add_argument("--timeout", type=float, default=7200)
    ap.add_argument("--lifecycle-backend", choices=BACKENDS, default="orb3",
                    help="lifecycle 이 쓸 SLAM 궤적 (기본 orb3)")
    ap.add_argument("--assoc-gate", type=float, default=OL.ASSOC_TRANS_GATE_M,
                    help="같은 인스턴스로 볼 map 공간 반경 [m]")
    ap.add_argument("--legacy-confidence", action="store_true",
                    help="품질 가중 대신 예전 히트카운터 존재 갱신을 쓴다")
    ap.add_argument("--no-lifecycle", action="store_true",
                    help="SAM-6D 만 돌리고 lifecycle 단계를 건너뛴다")
    ap.add_argument("--only-lifecycle", action="store_true",
                    help="SAM-6D 를 다시 돌리지 않고 기존 objects.json 으로 lifecycle 만")
    a = ap.parse_args()

    sess = C.resolve_session(a.session)
    data, out = C.data_dir(sess), C.out_dir(sess)

    if a.only_lifecycle:
        if run_lifecycle(data, out, a.lifecycle_backend, a.assoc_gate,
                         a.legacy_confidence) is None:
            return 1
        return 0

    if not (data / "SAM" / "metadata.yaml").is_file():
        C.die(f"{data}/sam 이 없다 — 먼저 convert.py 를 돌려라")
    stage = out / "_stage"
    stage.mkdir(parents=True, exist_ok=True)

    obj = (" --sam-objects " + " ".join(a.objects)) if a.objects else ""
    rc = C.run(f"python3 {RUN_INTEGRATION} {data} --stage sam --out {stage} "
               f"--sam-stride {a.stride} --max-frames 100000 --sam-timeout {a.timeout}{obj}",
               log_path=out / "logs" / "sam6d.log", label="SAM-6D (ISM + PEM + overlay)")

    src = stage / "sam" / "sam_objects.json"
    if rc != 0 or not src.is_file():
        C.die(f"SAM-6D 실패 — 로그: {out / 'logs' / 'sam6d.log'}")

    dst = out / "sam" / "objects.json"
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)
    (stage / "_probe_bag.py").unlink(missing_ok=True)

    s = json.loads(dst.read_text())
    C.log(f"검출 {s.get('bundles', '?')}건 · 프레임 {s.get('frames_with_detection', '?')}개 "
          f"· 물체 {sorted(s.get('objects', {}))}")

    life = None
    if not a.no_lifecycle:
        life = run_lifecycle(data, out, a.lifecycle_backend, a.assoc_gate,
                             a.legacy_confidence)
    C.log(f"완료 — {dst}" + (f" · {life}" if life else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
