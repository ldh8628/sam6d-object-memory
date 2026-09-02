#!/usr/bin/env python3
"""Run ORB-SLAM3 / RTAB-Map / hdl_graph_slam on the 260804 office sessions and
collect everything under output_slam/260804_office/<method>/<session>/.

ORB-SLAM3 and RTAB-Map reuse data_slam/0724_chungbuk/run_slam_0724.py verbatim
(same bag layout, same topics); only the output root is redirected. hdl_graph_slam
gets its own driver here because this dataset differs from 0724 in two ways that
matter:

  * the Velodyne cloud is already a full 360 deg sweep at 10 Hz (0724 carried
    half sweeps at 19.8 Hz and needed merge_velodyne_sweeps.py in front);
  * LiDAR, camera and IMU live in one bag, so playback must be restricted to the
    LiDAR/IMU topics or the RGB-D stream needlessly saturates the DDS layer.

Each method needs its own conda env:
    conda activate orbslam3            --method orbslam3
    conda activate rtabmap             --method rtabmap
    conda activate hdl_graph_slam_humble --method hdl
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트                                   # .../CLI_environment
OUT_ROOT = ROOT / "output_slam" / "260804_office"
STAGING = OUT_ROOT / "_staging"
HDL_WS = ROOT / "hdlgraphslam_ws"

METHOD_DIR = {"orbslam3": "orb3_slam", "rtabmap": "rtabmap", "hdl": "hdl_graph_slam"}
# what run_slam_0724.py calls the per-method subdirectory
SRC_SUBDIR = {"orbslam3": "orbslam3_noimu", "rtabmap": "rtabmap"}

HDL_RECORD_TOPICS = ["/tf", "/tf_static", "/odom", "/clock",
                     "/hdl_graph_slam/debug/keyframe_count",
                     "/hdl_graph_slam/debug/loop_count",
                     "/hdl_graph_slam/debug/floor_constraint_count",
                     "/scan_matching_odometry/debug", "/prefiltering/debug",
                     "/floor_detection/floor_coeffs"]


def load_0724():
    """Import run_slam_0724.py without putting it on sys.path permanently."""
    src = ROOT / "data_slam" / "0724_chungbuk" / "run_slam_0724.py"
    spec = importlib.util.spec_from_file_location("run_slam_0724", src)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------
# hdl_graph_slam
# --------------------------------------------------------------------------
def run_hdl(ds: dict, rate: float, domain: int, timeout: int,
            kf_trans: float, kf_angle: float, far_thresh: float,
            reg_threads: int) -> dict:
    out_dir = OUT_ROOT / METHOD_DIR["hdl"] / ds["name"]
    logs = out_dir / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    rec_bag = out_dir / "hdl_recorded_bag"
    dump_dir = out_dir / "graph_dump"
    for p in (rec_bag, dump_dir):
        if p.exists():
            shutil.rmtree(p, ignore_errors=True)

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["HDL_SOURCE_OPT_ROS"] = "false"          # ROS lives inside the conda env
    setup = f"source {HDL_WS}/install/setup.bash"

    def bash(cmd: str, log: Path):
        return subprocess.Popen(["bash", "-lc", f"{setup} && {cmd}"], env=env,
                                stdout=log.open("w"), stderr=subprocess.STDOUT,
                                preexec_fn=os.setsid)

    n_in = int(ds.get("lidar_msgs", 0))
    print(f"=== hdl: {ds['name']}  scans={n_in} rate={rate} domain={domain}",
          flush=True)

    launch = bash(
        "ros2 launch hdl_graph_slam hdl_graph_slam_501.launch.py use_sim_time:=true "
        # the sweeps in this dataset are already full revolutions, so hdl reads
        # /velodyne_points directly instead of a merged topic
        f"raw_points_topic:={ds.get('lidar', '/velodyne_points')} "
        "points_topic:=/filtered_points "
        "raw_points_qos:=reliable filtered_points_qos:=reliable "
        "enable_floor_detection:=true "
        f"distance_far_thresh:={far_thresh} "
        f"graph_keyframe_delta_trans:={kf_trans} "
        f"graph_keyframe_delta_angle:={kf_angle} "
        f"scan_reg_num_threads:={reg_threads}",
        logs / "launch.log")
    time.sleep(float(os.environ.get("HDL_LAUNCH_WAIT", "10")))

    record = bash(f"ros2 bag record --output {rec_bag} " + " ".join(HDL_RECORD_TOPICS),
                  logs / "record.log")
    time.sleep(3)

    t0 = time.time()
    play = bash(f"ros2 bag play {ds['bag']} --clock --rate {rate} --delay 3 "
                f"--topics {ds.get('lidar', '/velodyne_points')} "
                f"{ds.get('imu', '/xsens/imu/data')}",
                logs / "play.log")
    try:
        play.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(play.pid), signal.SIGINT)
        print("  !! playback timed out")
    play_sec = time.time() - t0
    time.sleep(float(os.environ.get("HDL_POST_WAIT", "25")))

    # dump BEFORE killing the node — the optimised graph only exists in memory
    dump = subprocess.run(
        ["bash", "-lc",
         f"{setup} && python3 "
         f"{ROOT/'data_slam'/'0724_chungbuk'/'dump_hdl_graph.py'} "
         f"--dest {dump_dir} --out {out_dir/'trajectory.txt'} "
         f"--odom-out {out_dir/'odometry.txt'}"],
        env=env, capture_output=True, text=True)
    (logs / "dump.log").write_text(dump.stdout + dump.stderr)
    print("  " + (dump.stdout.strip().splitlines() or ["(no dump output)"])[-1])

    for proc, name in ((record, "record"), (launch, "launch")):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=25)
        except Exception:                                     # noqa: BLE001
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:                                 # noqa: BLE001
                pass
            print(f"  ({name} needed SIGKILL)")

    launch_txt = (logs / "launch.log").read_text(errors="ignore")
    kf = re.findall(r"keyframe_count[^0-9]*(\d+)", launch_txt)
    traj = out_dir / "trajectory.txt"
    n_poses = sum(1 for _ in traj.open()) if traj.exists() else 0
    res = {"name": ds["name"], "method": "hdl_graph_slam", "rate": rate,
           "kf_trans": kf_trans, "kf_angle": kf_angle, "far_thresh": far_thresh,
           "input_scans": n_in, "play_sec": round(play_sec, 1),
           "traj_rows": n_poses, "ok": n_poses > 1,
           "keyframe_count_last": int(kf[-1]) if kf else None,
           "output": str(out_dir)}
    (out_dir / "run_info.json").write_text(json.dumps(res, indent=1))
    print(f"  poses={n_poses}  play={play_sec:.0f}s  ok={res['ok']}", flush=True)
    return res


# --------------------------------------------------------------------------
def collect(method: str, session: str) -> None:
    """Move run_slam_0724's staged output into <method>/<session>/."""
    src = STAGING / session / SRC_SUBDIR[method]
    dst = OUT_ROOT / METHOD_DIR[method] / session
    if not src.is_dir():
        print(f"  !! nothing staged at {src}")
        return
    if dst.exists():
        shutil.rmtree(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(src), str(dst))
    leftover = STAGING / session
    if leftover.is_dir() and not any(leftover.iterdir()):
        leftover.rmdir()
    if STAGING.is_dir() and not any(STAGING.iterdir()):
        STAGING.rmdir()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default=str(HERE / "manifest_260804.json"))
    ap.add_argument("--method", choices=["orbslam3", "rtabmap", "hdl"], required=True)
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--rate", type=float, default=0.5, help="hdl playback rate")
    ap.add_argument("--domain", type=int, default=80, help="hdl ROS_DOMAIN_ID")
    ap.add_argument("--timeout", type=int, default=5400)
    ap.add_argument("--kf-trans", type=float, default=0.5)
    ap.add_argument("--kf-angle", type=float, default=0.25)
    ap.add_argument("--far-thresh", type=float, default=30.0)
    ap.add_argument("--reg-threads", type=int, default=8)
    args = ap.parse_args()

    datasets = json.loads(Path(args.manifest).read_text())
    if args.only:
        datasets = [d for d in datasets if d["name"] in args.only]
    if not datasets:
        print("no sessions selected", file=sys.stderr)
        return 2

    results = []
    if args.method == "hdl":
        for ds in datasets:
            results.append(run_hdl(ds, args.rate, args.domain, args.timeout,
                                   args.kf_trans, args.kf_angle,
                                   args.far_thresh, args.reg_threads))
    else:
        mod = load_0724()
        mod.SC_OUTPUT = STAGING                       # redirect the output root
        mod.CONFIG_DIR = HERE / "configs"
        # Colour, depth and both camera_infos are stamped *identically* here, so
        # ExactTime is the correct matcher — but it only works once the images
        # stop being dropped, hence RELIABLE. Measured on stablization (778
        # frames), odometry updates / of which quality 0 / mispair warnings:
        #   approx 0.05 + best effort (the 0724 default)  279 / 485*  / 146
        #   approx 0.005 + best effort                    165 /  58   /   0
        #   exact        + best effort                    124 / 124   /   0
        #   exact        + reliable                       775 /   1   /   0
        # (*counted on longcircle2, which is where the 0.05 window did the most
        # damage: ApproximateTime is free to pair frame i with frame i+-1 because
        # one 33 ms frame period still fits inside the gate.)
        mod.RTAB_SYNC_ARGS = os.environ.get("RTAB_SYNC_ARGS", "approx_sync:=false")
        mod.RTAB_QOS_ARGS = os.environ.get(
            "RTAB_QOS_ARGS", "qos:=1 qos_image:=1 qos_camera_info:=1")
        for ds in datasets:
            print(f"=== {args.method}: {ds['name']} ===", flush=True)
            r = mod.run_orb(ds) if args.method == "orbslam3" else mod.run_rtab(ds)
            collect(args.method, ds["name"])
            r["output"] = str(OUT_ROOT / METHOD_DIR[args.method] / ds["name"])
            print("    " + json.dumps({k: v for k, v in r.items() if k != "output"}),
                  flush=True)
            results.append(r)

    print(f"\n=== {args.method} summary ===")
    ok_all = True
    for r in results:
        if args.method == "orbslam3":
            ok = r["traj_rows"] > 1 and r.get("kf_rows", 0) > 1
        elif args.method == "rtabmap":
            ok = r["traj_rows"] > 1 and r.get("dense_points", 0) > 0
        else:
            ok = bool(r.get("ok"))
        ok_all &= ok
        print(f"- [{'PASS' if ok else 'FAIL'}] {r['name']}: {json.dumps(r)}")
    summary = OUT_ROOT / f"run_summary_{args.method}.json"
    summary.parent.mkdir(parents=True, exist_ok=True)
    summary.write_text(json.dumps(results, indent=1))
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
