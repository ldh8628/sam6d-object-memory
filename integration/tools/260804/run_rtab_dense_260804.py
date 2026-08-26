#!/usr/bin/env python3
"""Re-run RTAB-Map on a 260804 session while recording its per-frame odometry.

Same reasoning as data_slam/0724_chungbuk/run_rtab_dense.py: the exported
`rtabmap_3d_map_poses.txt` is the pose *graph* (69 nodes / 0.96 Hz on
longcircle2), not a per-frame trajectory. `rgbd_odometry` runs at frame rate the
whole time and we simply never saved it, so this run records `/rtabmap/odom` and
exports the cloud from the *same* database — map and dense poses then come from
one run, which is what the Unity export needs.

Two 260804-specific differences from the 0724 script:

  * sync/QoS. Colour, depth and both camera_infos are stamped identically here,
    and the images only stop being dropped with RELIABLE, so this dataset wants
    `approx_sync:=false qos:=1` (measured in RESULTS_260804_office.md: 775
    odometry updates vs 279 with the 0724 defaults).
  * the cloud is exported too (`--cloud`), not just the poses.

    conda activate rtabmap
    python3 run_rtab_dense_260804.py longcircle2
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
OUT_ROOT = ROOT / "output_slam" / "260804_office" / "rtabmap_dense"
MANIFEST = HERE / "manifest_260804.json"

# rtabmap.launch.py puts its nodes in the `rtabmap` namespace, so odometry comes
# out on /rtabmap/odom — recording plain /odom silently captures nothing.
RECORD_TOPICS = ["/rtabmap/odom", "/odom", "/tf", "/tf_static", "/clock"]
LAUNCH_WAIT = 8.0
POST_WAIT = 8.0


def shell(cmd: str) -> list[str]:
    return ["bash", "-lc", f"source {RTAB_WS}/install/setup.bash && " + cmd]


def run(ds: dict, rate: float, domain: int, timeout: int) -> dict:
    name = ds["name"]
    out = OUT_ROOT / name
    logs = out / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    db = out / "rtabmap.db"
    rec = out / "odom_bag"
    for p in (db, rec):
        if p.is_dir():
            subprocess.run(["rm", "-rf", str(p)], check=False)
        elif p.exists():
            p.unlink()

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)

    launch_cmd = (
        "ros2 launch rtabmap_launch rtabmap.launch.py "
        "rtabmap_viz:=false rviz:=false use_sim_time:=true "
        f"frame_id:={ds.get('frame_id', 'camera_color_optical_frame')} "
        f"rgb_topic:={ds['rgb']} depth_topic:={ds['depth']} "
        f"camera_info_topic:={ds['camera_info']} "
        "approx_sync:=false "
        "qos:=1 qos_image:=1 qos_camera_info:=1 "
        "topic_queue_size:=100 queue_size:=100 "
        f"database_path:={db} args:=\"-d\""
    )

    def spawn(cmd: str, log: Path):
        return subprocess.Popen(shell(cmd), cwd=str(RTAB_WS), env=env,
                                stdout=log.open("wb"), stderr=subprocess.STDOUT,
                                preexec_fn=os.setsid)

    print(f"=== {name} ===  rate={rate} domain={domain}", flush=True)
    launch = spawn(launch_cmd, logs / "launch.log")
    time.sleep(LAUNCH_WAIT)
    record = spawn(f"ros2 bag record --output {rec} " + " ".join(RECORD_TOPICS),
                   logs / "record.log")
    time.sleep(3)

    t0 = time.time()
    play = spawn(f"ros2 bag play {ds['bag']} --clock --rate {rate} --delay 3",
                 logs / "play.log")
    try:
        play.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(play.pid), signal.SIGINT)
        print("  !! playback timed out", flush=True)
    play_sec = time.time() - t0
    time.sleep(POST_WAIT)

    for proc, label in ((record, "record"), (launch, "launch")):
        if proc.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=240)
        except Exception:                                     # noqa: BLE001
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:                                 # noqa: BLE001
                pass
            print(f"  ({label} needed SIGKILL)", flush=True)

    exp = subprocess.run(
        shell(f"rtabmap-export --cloud --poses --poses_format 11 --ascii "
              f"--output rtabmap_dense --output_dir {out} {db}"),
        cwd=str(RTAB_WS), env=env, capture_output=True, text=True)
    (logs / "export.log").write_text(exp.stdout + exp.stderr)

    poses = out / "rtabmap_dense_poses.txt"
    n_poses = sum(1 for line in poses.open() if not line.startswith("#")) if poses.exists() else 0
    cloud = out / "rtabmap_dense_cloud.ply"
    res = {"session": name, "rate": rate, "play_sec": round(play_sec, 1),
           "graph_poses": n_poses, "db": db.exists(), "odom_bag": rec.exists(),
           "cloud_mb": round(cloud.stat().st_size / 1e6, 1) if cloud.exists() else 0,
           "ok": bool(n_poses and rec.exists() and cloud.exists())}
    print(f"  graph_poses={n_poses}  odom_bag={rec.exists()}  "
          f"cloud={res['cloud_mb']} MB  play={play_sec:.0f}s", flush=True)
    (out / "run_info.json").write_text(json.dumps(res, indent=1))
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--rate", type=float, default=0.5,
                    help="bag playback rate; 0.5 matches the original run")
    ap.add_argument("--domain", type=int, default=91)
    ap.add_argument("--timeout", type=int, default=5400)
    args = ap.parse_args()

    manifest = {d["name"]: d for d in json.loads(MANIFEST.read_text())}
    results = []
    for name in args.sessions:
        if name not in manifest:
            print(f"{name}: not in manifest, skipped")
            continue
        results.append(run(manifest[name], args.rate, args.domain, args.timeout))
    print(json.dumps(results, indent=1))
    return 0 if results and all(r.get("ok") for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
