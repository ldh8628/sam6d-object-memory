#!/usr/bin/env python3
"""0807 세션에 RTAB-Map / hdl_graph_slam 을 돌려 integration 출력 트리에 넣는다.

ORB-SLAM3 는 integration/run_integration.py --stage slam 이 이미 담당하므로 여기 없다.
결과는 백엔드마다 따로 나온다:

    integration/output/<pair>/slam_orb3/     (run_integration)
    integration/output/<pair>/slam_rtabmap/  (여기, --method rtabmap)
    integration/output/<pair>/slam_hdl/      (여기, --method hdl)

설정값은 260804 에서 검증된 것을 그대로 쓴다:
  * RTAB 는 **exact sync + RELIABLE QoS** 여야 정상 동작한다(0724 에서 odom 279 -> 775).
    변환된 bag 은 color/depth 스탬프가 완전히 같으므로 approx_sync 는 오히려 해롭다.
  * RTAB 의 `rtabmap_3d_map_poses.txt` 는 **그래프 노드**(~0.9 Hz, 최대 18 s 공백)라
    융합에 그대로 쓰면 검출 대부분이 시간 허용치 밖으로 떨어진다. 그래서
    `/rtabmap/odom` 을 프레임레이트로 같이 녹화한다(densify 단계 입력).
  * hdl 은 노드를 죽이기 전에 dump_graph 를 불러 **최적화된** 그래프를 받아야 한다.
  * 0807 의 Velodyne 은 이미 10 Hz full sweep 이라 0724 처럼 스윕 병합이 필요 없다.

각 백엔드는 자기 conda 환경에서 실행해야 한다:
    conda activate rtabmap               --method rtabmap
    conda activate hdl_graph_slam_humble --method hdl

    python3 run_slam_0807.py --method rtabmap --pair 0807_185223_circle8
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]          # integration/tools/<x>/ -> 레포 루트
RTAB_WS = ROOT / "rtabmap_ws"
HDL_WS = ROOT / "hdlgraphslam_ws"
OUT_ROOT = ROOT / "integration" / "output"
MANIFEST = HERE / "manifest_0807.json"
# 0724_chungbuk 은 tar 로 묶여 심볼릭 링크가 끊길 수 있다. 있으면 그쪽(정본)을,
# 없으면 이 폴더의 복원본을 쓴다.
_DUMP_0724 = HERE.parent / "0724_chungbuk" / "dump_hdl_graph.py"
DUMP_SCRIPT = _DUMP_0724 if _DUMP_0724.is_file() else HERE / "dump_hdl_graph.py"

RTAB_RECORD = ["/rtabmap/odom", "/odom", "/tf", "/tf_static", "/clock"]
HDL_RECORD = ["/tf", "/tf_static", "/odom", "/clock",
              "/hdl_graph_slam/debug/keyframe_count",
              "/hdl_graph_slam/debug/loop_count",
              "/hdl_graph_slam/debug/floor_constraint_count",
              "/scan_matching_odometry/debug", "/prefiltering/debug",
              "/floor_detection/floor_coeffs"]


def spawn(cmd: str, log: Path, ws: Path, env: dict):
    log.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.Popen(["bash", "-lc", f"source {ws}/install/setup.bash && {cmd}"],
                            cwd=str(ws), env=env, stdout=log.open("wb"),
                            stderr=subprocess.STDOUT, preexec_fn=os.setsid)


def stop(proc, label: str, grace: int = 240):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=grace)
    except Exception:                                          # noqa: BLE001
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:                                      # noqa: BLE001
            pass
        print(f"  ({label} needed SIGKILL)", flush=True)


# --------------------------------------------------------------------------
def run_rtabmap(ds: dict, out: Path, rate: float, domain: int, timeout: int) -> dict:
    logs = out / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    db, rec = out / "rtabmap.db", out / "odom_bag"
    for p in (db, rec):
        subprocess.run(["rm", "-rf", str(p)], check=False)

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)

    launch_cmd = (
        "ros2 launch rtabmap_launch rtabmap.launch.py "
        "rtabmap_viz:=false rviz:=false use_sim_time:=true "
        f"frame_id:={ds.get('frame_id', 'camera_color_optical_frame')} "
        f"rgb_topic:={ds['rgb']} depth_topic:={ds['depth']} "
        f"camera_info_topic:={ds['camera_info']} "
        "approx_sync:=false "                     # 스탬프가 동일하므로 exact
        "qos:=1 qos_image:=1 qos_camera_info:=1 " # RELIABLE
        "topic_queue_size:=100 queue_size:=100 "
        f"database_path:={db} args:=\"-d\"")

    print(f"=== rtabmap: {ds['name']}  rate={rate} domain={domain}", flush=True)
    launch = spawn(launch_cmd, logs / "launch.log", RTAB_WS, env)
    time.sleep(8.0)
    record = spawn(f"ros2 bag record --output {rec} " + " ".join(RTAB_RECORD),
                   logs / "record.log", RTAB_WS, env)
    time.sleep(3)

    t0 = time.time()
    play = spawn(f"ros2 bag play {ds['bag']} --clock --rate {rate} --delay 3",
                 logs / "play.log", RTAB_WS, env)
    try:
        play.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(play.pid), signal.SIGINT)
        print("  !! playback timed out", flush=True)
    play_sec = time.time() - t0
    time.sleep(8.0)
    stop(record, "record")
    stop(launch, "launch")

    exp = subprocess.run(
        ["bash", "-lc", f"source {RTAB_WS}/install/setup.bash && "
         f"rtabmap-export --cloud --poses --poses_format 11 --ascii "
         f"--output rtabmap_dense --output_dir {out} {db}"],
        cwd=str(RTAB_WS), env=env, capture_output=True, text=True)
    (logs / "export.log").write_text(exp.stdout + exp.stderr)

    poses = out / "rtabmap_dense_poses.txt"
    n = sum(1 for ln in poses.open() if not ln.startswith("#")) if poses.exists() else 0
    res = {"session": ds["name"], "method": "rtabmap", "rate": rate,
           "play_sec": round(play_sec, 1), "graph_poses": n,
           "odom_bag": rec.is_dir(), "db": db.exists(),
           "ok": bool(n and rec.is_dir()), "output": str(out)}
    print(f"  graph_poses={n}  odom_bag={rec.is_dir()}  play={play_sec:.0f}s", flush=True)
    (out / "run_info.json").write_text(json.dumps(res, indent=1))
    return res


# --------------------------------------------------------------------------
def run_hdl(ds: dict, out: Path, rate: float, domain: int, timeout: int,
            kf_trans: float, kf_angle: float, far_thresh: float,
            reg_threads: int) -> dict:
    logs = out / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    rec, dump = out / "hdl_recorded_bag", out / "graph_dump"
    for p in (rec, dump):
        subprocess.run(["rm", "-rf", str(p)], check=False)

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["HDL_SOURCE_OPT_ROS"] = "false"

    print(f"=== hdl: {ds['name']}  scans={ds.get('lidar_msgs')} rate={rate} "
          f"domain={domain}", flush=True)
    launch = spawn(
        "ros2 launch hdl_graph_slam hdl_graph_slam_501.launch.py use_sim_time:=true "
        f"raw_points_topic:={ds.get('lidar', '/velodyne_points')} "
        "points_topic:=/filtered_points "
        "raw_points_qos:=reliable filtered_points_qos:=reliable "
        "enable_floor_detection:=true "
        f"distance_far_thresh:={far_thresh} "
        f"graph_keyframe_delta_trans:={kf_trans} "
        f"graph_keyframe_delta_angle:={kf_angle} "
        f"scan_reg_num_threads:={reg_threads}",
        logs / "launch.log", HDL_WS, env)
    time.sleep(float(os.environ.get("HDL_LAUNCH_WAIT", "10")))
    record = spawn(f"ros2 bag record --output {rec} " + " ".join(HDL_RECORD),
                   logs / "record.log", HDL_WS, env)
    time.sleep(3)

    t0 = time.time()
    play = spawn(f"ros2 bag play {ds['lidar_bag']} --clock --rate {rate} --delay 3 "
                 f"--topics {ds.get('lidar', '/velodyne_points')}",
                 logs / "play.log", HDL_WS, env)
    try:
        play.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(play.pid), signal.SIGINT)
        print("  !! playback timed out", flush=True)
    play_sec = time.time() - t0
    time.sleep(float(os.environ.get("HDL_POST_WAIT", "25")))

    # 노드를 죽이기 전에 최적화된 그래프를 받아온다
    d = subprocess.run(
        ["bash", "-lc", f"source {HDL_WS}/install/setup.bash && python3 "
         f"{DUMP_SCRIPT} "
         f"--dest {dump} --out {out/'trajectory_velo.txt'} "
         f"--odom-out {out/'odometry_velo.txt'}"],
        env=env, capture_output=True, text=True)
    (logs / "dump.log").write_text(d.stdout + d.stderr)
    print("  " + (d.stdout.strip().splitlines() or ["(no dump output)"])[-1], flush=True)

    stop(record, "record", 25)
    stop(launch, "launch", 25)

    traj = out / "trajectory_velo.txt"
    n = sum(1 for _ in traj.open()) if traj.exists() else 0
    res = {"session": ds["name"], "method": "hdl_graph_slam", "rate": rate,
           "input_scans": ds.get("lidar_msgs"), "play_sec": round(play_sec, 1),
           "traj_rows": n, "ok": n > 1, "frame": "velodyne",
           "note": "LiDAR 프레임이다. 카메라 프레임으로 옮기려면 rig_offset.py 로 "
                   "T_velo_cam 을 추정해야 한다 (캘리브 아님, 궤적 정합).",
           "output": str(out)}
    print(f"  poses={n}  play={play_sec:.0f}s  ok={res['ok']}", flush=True)
    (out / "run_info.json").write_text(json.dumps(res, indent=1))
    return res


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", choices=["rtabmap", "hdl"], required=True)
    ap.add_argument("--pair", required=True, help="integration/output/<pair>")
    ap.add_argument("--session", default="", help="manifest name (기본: 첫 항목)")
    ap.add_argument("--manifest", default=str(MANIFEST))
    ap.add_argument("--rate", type=float, default=0.5)
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--timeout", type=int, default=5400)
    ap.add_argument("--kf-trans", type=float, default=0.5)
    ap.add_argument("--kf-angle", type=float, default=0.25)
    ap.add_argument("--far-thresh", type=float, default=30.0)
    ap.add_argument("--reg-threads", type=int, default=8)
    a = ap.parse_args()

    ds_all = json.loads(Path(a.manifest).read_text())
    ds = next((d for d in ds_all if d["name"] == a.session), None) if a.session else ds_all[0]
    if ds is None:
        print(f"세션을 찾을 수 없다: {a.session}", file=sys.stderr)
        return 2

    out = OUT_ROOT / a.pair / f"slam_{a.method}"
    out.mkdir(parents=True, exist_ok=True)
    domain = a.domain or (91 if a.method == "rtabmap" else 80)

    if a.method == "rtabmap":
        res = run_rtabmap(ds, out, a.rate, domain, a.timeout)
    else:
        res = run_hdl(ds, out, a.rate, domain, a.timeout, a.kf_trans,
                      a.kf_angle, a.far_thresh, a.reg_threads)
    print(json.dumps(res, indent=1, ensure_ascii=False))
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
