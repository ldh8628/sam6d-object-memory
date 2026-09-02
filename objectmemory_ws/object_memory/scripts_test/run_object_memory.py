#!/usr/bin/env python3
"""Offline object_memory end-to-end runner (3 file inputs).

Inputs:
    --slam-traj   SLAM CameraTrajectory.txt (TUM)          -> T_map_cam
    --pem-dir     SAM-6D PEM output tree for one bag        -> T_cam_obj
    --bag         ros2 bag dir/.db3 shared by SLAM and SAM  -> frame -> timestamp

Output:
    T_map_obj per detection fused into persistent object memory, plus a markdown
    report with per-object landmarks, lifecycle, and decision summary.

No fake data is created; all three inputs are read-only.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from adapters.bag_frame_index import load_frame_timestamps
from adapters.sam6d_pem import load_pem_detections
from adapters.slam_trajectory import load_slam_trajectory
from pipeline.object_memory_runner import run_object_memory


def _fmt_xyz(T):
    return f"({T[0][3]:.3f}, {T[1][3]:.3f}, {T[2][3]:.3f})"


def _png_size(path):
    """(width, height) from a PNG IHDR header, no image library needed."""
    with open(path, "rb") as f:
        head = f.read(24)
    if len(head) < 24 or head[12:16] != b"IHDR":
        return None
    return (int.from_bytes(head[16:20], "big"),
            int.from_bytes(head[20:24], "big"))


def _load_camera(pem_dir):
    """Intrinsics + image size for the FOV-based P_D, from the SAM-6D input
    tree next to the PEM outputs (<bag_root>/input/frame_*/camera.json)."""
    bag_root = os.path.dirname(os.path.dirname(os.path.realpath(pem_dir)))
    for cam_path in sorted(glob.glob(
            os.path.join(bag_root, "input", "frame_*", "camera.json"))):
        try:
            k = json.load(open(cam_path))["cam_K"]
        except (OSError, KeyError, ValueError):
            continue
        cam_K = ((k[0], k[1], k[2]), (k[3], k[4], k[5]), (k[6], k[7], k[8]))
        rgb = os.path.join(os.path.dirname(cam_path), "rgb.png")
        size = _png_size(rgb) if os.path.isfile(rgb) else None
        if size:
            return cam_K, size
    return None, None


def write_report(result, out_path, meta):
    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    store = result.store
    lines = []
    lines.append("# Object Memory Run Report")
    lines.append("")
    lines.append(f"- slam trajectory: `{meta['slam_traj']}`")
    lines.append(f"- pem dir: `{meta['pem_dir']}`")
    lines.append(f"- bag: `{meta['bag']}`")
    lines.append("")
    lines.append("## Formula")
    lines.append("")
    lines.append("`T_map_obj = T_map_cam (SLAM) * T_cam_obj (SAM-6D)`  [translations in metres]")
    lines.append("")
    lines.append("## Run Summary")
    lines.append("")
    lines.append(f"- SLAM poses loaded: {meta['n_poses']}")
    lines.append(f"- bag color timestamps: {meta['n_timestamps']}")
    lines.append(f"- frames with detections: {result.frames_total}")
    lines.append(f"- frames fused (SLAM ok): {result.frames_with_slam}")
    lines.append(f"- frames skipped (no SLAM pose): {result.frames_without_slam}")
    lines.append(f"- total detections: {result.detections_total}")
    lines.append("")
    lines.append("## Persistent Objects (object_memory owns object_id)")
    lines.append("")
    lines.append("| object_id | object_name | status | obs | missed | confidence | T_map_obj xyz (m) |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- |")
    for lm in sorted(store.all_landmarks(), key=lambda x: x.object_id):
        lines.append(
            f"| {lm.object_id} | {lm.object_name} | {lm.status.value} | "
            f"{lm.observation_count} | {lm.missed_count} | {lm.confidence:.2f} | "
            f"{_fmt_xyz(lm.T_map_obj)} |"
        )
    lines.append("")
    lines.append("## Lifecycle Transitions")
    lines.append("")
    log = store.transition_log()
    if log:
        lines.append("| object_id | from -> to | reason | stamp |")
        lines.append("| --- | --- | --- | --- |")
        for object_id, tr in log:
            stamp = f"{tr.stamp:.3f}" if tr.stamp is not None else "-"
            lines.append(
                f"| {object_id} | {tr.from_status.value} -> {tr.to_status.value} "
                f"| {tr.reason} | {stamp} |"
            )
    else:
        lines.append("(no transitions)")
    lines.append("")
    lines.append("## Decision Summary")
    lines.append("")
    counts = {}
    for fr in result.frame_results:
        for d in fr.decisions:
            counts[d.decision_type.value] = counts.get(d.decision_type.value, 0) + 1
    for k in sorted(counts):
        lines.append(f"- {k}: {counts[k]}")
    lines.append("")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Offline object_memory runner")
    parser.add_argument("--slam-traj", required=True)
    parser.add_argument("--pem-dir", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument(
        "--color-topic", default="/camera/camera/color/image_raw"
    )
    parser.add_argument("--source-slam-id", default="orbslam3_noimu")
    parser.add_argument("--time-tolerance", type=float, default=0.05)
    parser.add_argument("--assoc-trans-gate", type=float, default=0.20,
                        help="multi-instance association translation radius (m); "
                             "beyond it a detection spawns a new instance")
    parser.add_argument("--assoc-rot-gate", type=float, default=-1.0,
                        help="association rotation gate (deg); <0 disables it "
                             "(PEM rotation is noisy, so off by default)")
    parser.add_argument("--legacy-confidence", action="store_true",
                        help="use the pre-Story-5 hit-counter existence update "
                             "instead of quality-weighted confidence (default)")
    parser.add_argument("--score-floor", type=float, default=0.0)
    parser.add_argument("--tentative-gate-mult", type=float, default=3.0)
    parser.add_argument("--pd-base", type=float, default=0.6,
                        help="expected detection probability while in view")
    parser.add_argument("--clutter-ratio", type=float, default=0.1,
                        help="false-alarm likelihood ratio of accepted detections")
    parser.add_argument("--r-init", type=float, default=0.7,
                        help="existence r of a freshly seeded tentative")
    parser.add_argument("--r-promote", type=float, default=0.6)
    parser.add_argument("--r-lost", type=float, default=0.3)
    parser.add_argument("--r-retire", type=float, default=0.05)
    parser.add_argument("--tentative-max-age", type=int, default=100,
                        help="frames; deletes tentatives whose claimed position "
                             "never re-enters the view")
    parser.add_argument("--min-obs-longterm", type=int, default=5,
                        help="retire split: obs >= N -> remembered, else deleted")
    parser.add_argument(
        "--output-report", default=None,
        help="default: objectmemory_ws/output/<bag>/object_memory_run.md",
    )
    args = parser.parse_args(argv)

    if args.output_report is None:
        here = os.path.dirname(__file__)
        output_root = os.path.abspath(os.path.join(here, "..", "..", "output"))
        real_pem = os.path.realpath(args.pem_dir)
        bag_root = os.path.dirname(os.path.dirname(real_pem))
        bn = os.path.basename(os.path.normpath(args.bag))
        bag_name = os.path.basename(bag_root) if bn == "bag" else bn
        args.output_report = os.path.join(
            output_root, bag_name, "object_memory_run.md")

    poses = load_slam_trajectory(args.slam_traj, source_slam_id=args.source_slam_id)
    timestamps = load_frame_timestamps(args.bag, color_topic_hint=args.color_topic)
    detections = load_pem_detections(args.pem_dir, timestamps=timestamps)
    cam_K, img_size = _load_camera(args.pem_dir)
    if cam_K is None:
        print("warning: no camera.json found; P_D uses the angular-FOV fallback")

    result = run_object_memory(
        poses, detections, timestamps, time_tolerance=args.time_tolerance,
        assoc_trans_gate_m=args.assoc_trans_gate,
        assoc_rot_gate_deg=args.assoc_rot_gate,
        score_floor=args.score_floor,
        tentative_gate_mult=args.tentative_gate_mult,
        pd_base=args.pd_base,
        clutter_ratio=args.clutter_ratio,
        r_init=args.r_init,
        r_promote=args.r_promote,
        r_lost=args.r_lost,
        r_retire=args.r_retire,
        tentative_max_age=args.tentative_max_age,
        min_obs_longterm=args.min_obs_longterm,
        cam_K=cam_K,
        img_size=img_size,
        quality_weighting=not args.legacy_confidence,
    )

    meta = {
        "slam_traj": args.slam_traj,
        "pem_dir": args.pem_dir,
        "bag": args.bag,
        "n_poses": len(poses),
        "n_timestamps": len(timestamps),
    }
    write_report(result, args.output_report, meta)

    print(f"objects: {len(result.store.all_landmarks())}  "
          f"frames_fused: {result.frames_with_slam}/{result.frames_total}  "
          f"detections: {result.detections_total}")
    for lm in sorted(result.store.all_landmarks(), key=lambda x: x.object_id):
        print(f"  #{lm.object_id} {lm.object_name} [{lm.status.value}] "
              f"obs={lm.observation_count} xyz={_fmt_xyz(lm.T_map_obj)}")
    print(f"report: {args.output_report}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
