#!/usr/bin/env python3
"""sam6d_infer.py — 추론 프로세스. **ROS 를 전혀 쓰지 않는다.**

공유메모리에서 '가장 최신 프레임 하나'만 집어 ISM+PEM 을 돌리고, 결과를 다시 공유메모리로
돌려준다. 산출물(jsonl·마스크)도 여기서 쓴다.

이 프로세스에 ROS 가 없다는 것이 요점이다. 고속 영상 구독과 같은 인터프리터에 있으면
GIL 때문에 추론이 초 단위로 굶는다(NOTES 19~21절). ROS 밖 실측 상한은 ISM 107 ms · PEM 156 ms.

    conda activate sam6d
    python realtime/sam6d_infer.py --config <run yaml>
"""
from __future__ import annotations

import argparse
import json
import mmap
import os
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shm_channel import FrameReader, JsonWriter          # noqa: E402
import verify_config as VC                             # noqa: E402
from sam6d_core import Sam6DCore, REPO                   # noqa: E402
from pem_explorer_record import ExplorerRecorder, provenance_entry  # noqa: E402


def _explorer_bag_path(cfg):
    explorer = cfg.get("output", {}).get("pem_explorer", {}) or {}
    bag_path = Path(explorer.get("source_bag") or cfg.get("bag", {}).get("path", ""))
    if not bag_path.is_absolute():
        bag_path = Path(REPO) / bag_path
    return bag_path.resolve()


def _explorer_provenance(config_path, cfg, core):
    bag_path = _explorer_bag_path(cfg)
    pem_dir = Path(REPO) / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
    return {
        "config": provenance_entry(config_path),
        "bag": provenance_entry(bag_path),
        "checkpoint": provenance_entry(pem_dir / "checkpoints" / "sam-6d-pem-base.pth"),
        "templates": {name: provenance_entry(Path(REPO) / "assets" / "pem_templates" /
                                               f"{name}.pt") for name in core._tem},
        "cad": {name: provenance_entry(Path(REPO) / "assets" / "model_points" /
                                         f"{name}.npy") for name in core._pts},
    }


def _public_frame_diagnostics(diagnostics):
    """Strip raw recorder-only ndarrays from ordinary frames.jsonl."""
    out = dict(diagnostics)
    out["pem_candidates"] = []
    for attempt in diagnostics.get("pem_candidates", []):
        out["pem_candidates"].append({k: v for k, v in attempt.items()
                                      if not k.startswith("explorer_") and k != "decision"
                                      and k != "crop_bbox_yxyx"})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config, encoding="utf-8")) or {}
    rt, out = cfg.get("runtime", {}), cfg.get("output", {})
    odir = out.get("dir", "output/rt_split")
    odir = odir if os.path.isabs(odir) else os.path.join(REPO, odir)
    os.makedirs(odir, exist_ok=True)
    live_recording = bool((out.get("live_recording") or {}).get("enabled", False))
    diag = bool(out.get("diagnostics", False) or live_recording)
    if diag:
        os.makedirs(os.path.join(odir, "masks"), exist_ok=True)

    explorer_cfg = dict(out.get("pem_explorer") or {})
    if explorer_cfg.get("enabled"):
        diagnostic_cfg = rt.get("pem_diagnostic") or {}
        if not (diagnostic_cfg.get("enabled") and
                (diagnostic_cfg.get("explorer_v2") or {}).get("enabled")):
            raise SystemExit(
                "[explorer] output.pem_explorer.enabled requires "
                "runtime.pem_diagnostic.enabled and explorer_v2.enabled")
        output_profile = explorer_cfg.get("capture_profile")
        diagnostic_profile = (diagnostic_cfg.get("explorer_v2") or {}).get(
            "capture_profile")
        if output_profile != diagnostic_profile:
            raise SystemExit(
                "[explorer] output and diagnostic capture_profile must match exactly")
        declared_bag = cfg.get("bag", {}).get("path")
        if declared_bag:
            declared = Path(declared_bag)
            if not declared.is_absolute():
                declared = Path(REPO) / declared
            if declared.resolve() != _explorer_bag_path(cfg):
                raise SystemExit("[explorer] source_bag and bag.path must identify the same bag")
        sys.path.insert(0, str(Path(REPO) / "tools"))
        from validate_rgbd_dataset import validate_rgbd_dataset
        validation = validate_rgbd_dataset(
            _explorer_bag_path(cfg), require_manifest=True)
        explorer_cfg["expected_last_stamp_ns"] = validation.last_stamp_ns
        # The split path deliberately drops stale frames. Reaching the tail therefore
        # means reaching a small final-frame window, not necessarily decoding the exact
        # last RGB message. Derive that window from the attested dataset instead of a
        # wall-clock guess, while keeping it narrow enough to reject interrupted bags.
        mean_period_ns = (max(0, validation.last_stamp_ns - validation.first_stamp_ns) //
                          max(1, validation.pair_count - 1))
        explorer_cfg["completion_tolerance_ns"] = max(
            int(float(rt.get("sync_slop", 0.02)) * 1e9), 5 * mean_period_ns)
        explorer_cfg["completion_tolerance_policy"] = (
            "max(sync_slop,5_attested_mean_frame_periods)")

    core = Sam6DCore(cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml"),
                     cfg.get("ism", {}).get("objects", []),
                     rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2),
                     appe_rerank=rt.get("appe_rerank"),
                     verify=rt.get("verify", VC.UNSET),
                     pem_diagnostic=rt.get("pem_diagnostic"))
    anchor_cfg = cfg.get("anchor", {})
    slam_cfg = cfg.get("slam", {})
    if anchor_cfg.get("enabled", False):
        anchor_cfg = dict(anchor_cfg)
        anchor_cfg.setdefault("symmetry_axes", core.verify.get("symmetry_axes", {}))
        anchor_cfg.setdefault("sym_step_deg", core.verify.get("sym_step_deg", 10))
        core.configure_anchors(slam_cfg.get("map_id", "default"), anchor_cfg)

    recorder = None
    if explorer_cfg.get("enabled"):
        recorder = ExplorerRecorder(
            odir, _explorer_provenance(a.config, cfg, core), explorer_cfg)

    fifo_path = os.path.join(odir, ".sam6d_results.fifo")
    ack_path = os.path.join(odir, ".sam6d_result_ack")
    for _ in range(600):
        if os.path.exists(fifo_path) and os.path.exists(ack_path):
            break
        time.sleep(0.1)
    else:
        raise SystemExit("[infer] receiver result FIFO를 찾지 못했다")
    result_fd = os.open(fifo_path, os.O_WRONLY)
    ack_fd = os.open(ack_path, os.O_RDONLY)
    ack = mmap.mmap(ack_fd, 8, access=mmap.ACCESS_READ)

    # 모델과 결과 수신기가 모두 준비된 뒤에야 재생을 시작한다.
    ready = os.path.join(odir, "READY")
    Path(ready).write_text(str(time.time()))
    print(f"[infer] 준비 완료 → {ready}", flush=True)

    fr, jw = None, JsonWriter()
    for _ in range(600):                     # 수신 프로세스가 먼저 뜰 때까지 기다린다
        try:
            fr = FrameReader(); break
        except FileNotFoundError:
            time.sleep(0.1)
    if fr is None:
        raise SystemExit("[infer] 프레임 공유메모리를 찾지 못했다")

    f_det = open(os.path.join(odir, "detections.jsonl"), "w", encoding="utf-8")
    f_frm = open(os.path.join(odir, "frames.jsonl"), "w", encoding="utf-8")
    meta = {"started_wall": time.time(), "mode": "split(receiver+infer)",
            "objects": [o["name"] for o in core.objs], "config": cfg}
    json.dump(meta, open(os.path.join(odir, "run_meta.json"), "w"), indent=1, ensure_ascii=False)

    n_proc = n_det = 0
    t_start = time.monotonic()
    idle_since = time.time()
    got_any = False          # 첫 프레임이 오기 전에는 종료 타이머를 걸지 않는다
    completed = False
    try:
        while True:
            got = fr.read_new()
            if got is None:
                # 터미널을 따로 띄우면 사람이 재생/카메라를 켜기까지 시간이 걸린다. 그동안
                # 종료해 버리면 안 되므로 **한 장이라도 받은 뒤에만** 유휴 종료를 건다.
                # idle_exit_s 가 0 이면 스스로 끝나지 않는다(카메라 운용).
                _ie = float(rt.get("idle_exit_s", 20))
                if got_any and _ie > 0 and time.time() - idle_since > _ie:
                    print("[infer] 입력이 끊겨 종료한다", flush=True)
                    break
                time.sleep(0.002)
                continue
            idle_since = time.time()
            got_any = True
            rgb, depth, K, stamp_ns, recv_wall = got
            source_seq = fr.last_source_seq
            slam_context = fr.last_slam_context
            if slam_context is not None and not slam_context.get("map_id"):
                slam_context["map_id"] = str(slam_cfg.get("map_id", "default"))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            t_a = time.time()
            rows, ms, n_boxes, lab = core.process(
                bgr, depth, K, want_mask=(diag or recorder is not None),
                slam_context=slam_context, lossless_mask=live_recording)
            t_b = time.time()
            n_proc += 1; n_det += len(rows)
            if recorder is not None:
                recorder.record_frame(
                    stamp_ns, fr.last_depth_stamp_ns, n_proc - 1, K, depth.shape,
                    core.last_frame_diag.get("pem_candidates", []), lab)
            # bag 시각 환산: 이 프레임이 수신된 벽시계 시각을 기준점으로 삼는다(rate 1.0)
            t_start_ns = int(stamp_ns + (t_a - recv_wall) * 1e9)
            t_done_ns = int(stamp_ns + (t_b - recv_wall) * 1e9)
            result = {"stamp_ns": stamp_ns, "depth_stamp_ns": fr.last_depth_stamp_ns,
                      "source_seq": source_seq, "frame_seq": n_proc - 1,
                      "t_done_ns": t_done_ns, "n": len(rows),
                      "map_id": str((slam_context or {}).get(
                          "map_id", slam_cfg.get("map_id", "default"))),
                      "map_anchors": ({} if core.anchor_manager is None else {
                          name: pose.round(8).tolist()
                          for name, pose in core.anchor_manager.anchors.items()}),
                      "dets": [{"object": r["object"], "score": r["score"],
                                "R": r["R"], "t_mm": r["t_mm"],
                                "pose_source": r.get("pose_source", "sam6d"),
                                "map_id": r.get("map_id"),
                                "anchor_state": r.get("anchor_state"),
                                "rejection_reason": r.get("rejection_reason")}
                               for r in rows],
                      "ms": ms, "n_proc": n_proc, "n_det": n_det}
            payload = json.dumps(result, ensure_ascii=False).encode() + b"\n"
            while payload:
                payload = payload[os.write(result_fd, payload):]
            jw.write(result)
            deadline = time.monotonic() + 5.0
            while struct.unpack_from("<q", ack, 0)[0] < n_proc - 1:
                if time.monotonic() >= deadline:
                    raise TimeoutError("receiver did not publish the inference result")
                time.sleep(0.001)
            for r in rows:
                f_det.write(json.dumps({**r, "stamp_ns": stamp_ns,
                                        "depth_stamp_ns": fr.last_depth_stamp_ns,
                                        "source_seq": source_seq,
                                        "frame_seq": n_proc - 1}, ensure_ascii=False) + "\n")
            f_frm.write(json.dumps({"stamp_ns": stamp_ns, "frame_seq": n_proc - 1,
                                    "depth_stamp_ns": fr.last_depth_stamp_ns,
                                    "source_seq": source_seq,
                                    "n_accept": len(rows), "n_boxes": n_boxes, "ms": ms,
                                    "t_start_ns": t_start_ns, "t_done_ns": t_done_ns,
                                    "objects": [r["object"] for r in rows],
                                    "diagnostics": _public_frame_diagnostics(
                                        core.last_frame_diag)},
                                   ensure_ascii=False) + "\n")
            f_det.flush(); f_frm.flush()
            # ExplorerRecorder owns its one label PNG per attempted frame. Preserve
            # the legacy diagnostics contract outside Explorer without double-writing.
            if recorder is None and lab is not None:
                if not cv2.imwrite(os.path.join(odir, "masks", f"{stamp_ns}.png"), lab):
                    raise OSError(f"failed to write mask for {stamp_ns}")
            print(f"#{n_proc-1} {len(rows)}개 ({', '.join(r['object'] for r in rows) or '-'})  "
                  f"{ms['total']}ms [yolo {ms['yolo']} / ism {ms['ism']} / pem {ms['pem']}]",
                  flush=True)
        completed = True
    except KeyboardInterrupt:
        # A manually interrupted run is intentionally discoverable as incomplete.
        completed = False
    finally:
        el = max(1e-6, time.monotonic() - t_start)
        summary = {"frames_processed": n_proc, "detections": n_det,
                   "elapsed_s": round(el, 1), "hz_processed": round(n_proc / el, 2)}
        print(f"[summary] {summary}", flush=True)
        meta["summary"] = summary
        json.dump(meta, open(os.path.join(odir, "run_meta.json"), "w"),
                  indent=1, ensure_ascii=False)
        f_det.close(); f_frm.close(); fr.close(); jw.close(); os.close(result_fd)
        ack.close(); os.close(ack_fd)
        if recorder is not None:
            recorder.close(completed=completed)
        try:
            os.remove(ready)
        except OSError:
            pass


if __name__ == "__main__":
    main()
