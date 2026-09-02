#!/usr/bin/env python3
"""run_integration.py — one entry point for a (SLAM bag, SAM bag) pair.

    python3 integration/run_integration.py <pair>            # pair = integration/data/<pair>

Given ONE input folder that holds both recordings of a session, this runs

    stage 1  SLAM bag -> ORB-SLAM3 RGB-D    (conda env: orbslam3, ROS 2 in-env)
    stage 2  SAM  bag -> SAM-6D ISM + PEM   (conda: sam_yolo -> sam6d_ros_humble)
    stage 3  both     -> one object map in the SLAM camera's map frame
                         T_map_obj = T_map_camSLAM(t) * X_camSLAM_camSAM * T_camSAM_obj

and collects everything under integration/output/<pair>/. Stages 1 and 2 are
separate processes on purpose: the ultralytics/torch versions of the SAM-6D envs
and the ROS/ORB-SLAM3 env cannot coexist in one interpreter. Stage 3 needs no
env at all — objectmemory_ws/object_memory is pure stdlib and is imported as a
library — so this script itself stays plain-stdlib and runs on system python3.

Input layout (bag dir = any directory containing metadata.yaml, found
recursively; classified by its path: "slam" -> SLAM, else "sam" -> SAM):

    integration/data/<pair>/
        SLAM/{metadata.yaml, *.db3}          # 경로에 "slam" 포함 -> SLAM 카메라
        SAM/{metadata.yaml, *.db3}           # 경로에 "sam"  포함 -> SAM 카메라
        lidar/, imu/, calib/, info.json      # converting_launch/convert_all.py 산출
        [slam/settings/orb_*.yaml]        # optional ORB-SLAM3 camera settings
        [sam/calib/X_camSLAM_camSAM.npy]  # optional 4x4 SLAM<-SAM extrinsic

Output layout:

    integration/output/<pair>/
        bag_info.json          probed topics / intrinsics / duration of both bags
        run_summary.json       what ran, how long, where the artefacts are
        slam/  orb_config.yaml orb_settings.yaml orbslam3_launch.log
               CameraTrajectory.txt KeyFrameTrajectory.txt trajectory.txt
               orbslam3_map_points.pcd orbslam3_dense_map.pcd
        sam/   ism.log pem.log viz.log sam_objects.json
               pem_inputs/frame_*/{rgb,depth}.png camera.json
                                  detection_<obj>.json
                                  pem_<obj>/sam6d_results/{detection_pem.json,vis_pem.png}
                                  vis_pem_all.png
        fused/ object_map.json           persistent objects in the map frame
               map_topview.svg           camera path + objects, one picture
               object_memory_report.md   lifecycle / decision report
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

# --------------------------------------------------------------------------
# fixed environment layout (mirrors README.md / tools/run_e2e.sh)
# --------------------------------------------------------------------------
INTEGRATION_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = INTEGRATION_DIR.parent
ORB_WS = PROJECT_ROOT / "orbslam_ws"
SAM_WS = PROJECT_ROOT / "sam6d_ws"
# 2026-08-25 재구성: 변환본은 레포 최상위 data_slam_converted/<날짜>/<세션>/ 에만 있다.
# integration/data/<날짜> 는 그리로 가는 심볼릭 링크라 옛 호출부도 그대로 동작한다.
DATA_DIR = PROJECT_ROOT / "data_slam_converted"
OUTPUT_DIR = INTEGRATION_DIR / "output"

CONDA_SH = Path.home() / "miniconda3" / "etc" / "profile.d" / "conda.sh"
ENV_ORB = "orbslam3"                                     # ROS 2 + ORB-SLAM3
PY_ISM = Path.home() / "miniconda3/envs/sam_yolo/bin/python"          # ISM stage
PY_PEM = Path.home() / "miniconda3/envs/sam6d_ros_humble/bin/python"  # PEM stage

ORB_VOCAB = ORB_WS / "src" / "ORB_SLAM3" / "Vocabulary" / "ORBvoc.txt"
SAM_BAG_ROOT = SAM_WS / "data" / "ros2_bag"          # build_pem_inputs.py --bag root
SAM_PEM_ROOT = SAM_WS / "outputs" / "pem_inputs"     # run_pem_batch.py input root

# object_memory fusion library (pure stdlib -> imported directly, not shelled out)
OBJMEM = PROJECT_ROOT / "objectmemory_ws" / "object_memory"
OBJMEM_SRC = OBJMEM / "src"

# topic name fragments used to pick streams out of a bag
COLOR_HINT = "color/image_raw"
DEPTH_HINT = "aligned_depth_to_color/image_raw"
CAMINFO_HINT = "color/camera_info"

BANNER = "#" * 74


def log(msg: str) -> None:
    print(f"[integration] {msg}", flush=True)


def die(msg: str) -> "None":
    print(f"[integration][ERROR] {msg}", file=sys.stderr, flush=True)
    raise SystemExit(2)


# --------------------------------------------------------------------------
# input discovery
# --------------------------------------------------------------------------
def resolve_pair_dir(arg: str) -> Path:
    """<pair> is either a name under integration/data/ or an explicit path."""
    cand = Path(arg).expanduser()
    if cand.is_dir():
        return cand.resolve()
    cand = DATA_DIR / arg
    if cand.is_dir():
        return cand.resolve()
    die(f"input folder not found: {arg} (looked at ./{arg} and {DATA_DIR / arg})")


def find_bag_dirs(root: Path, max_depth: int = 4) -> list[Path]:
    """Every directory that looks like a rosbag2 dir (has metadata.yaml)."""
    found = []
    root = root.resolve()
    for dirpath, dirnames, filenames in os.walk(root, followlinks=True):
        rel_depth = len(Path(dirpath).relative_to(root).parts)
        if rel_depth > max_depth:
            dirnames[:] = []
            continue
        if "metadata.yaml" in filenames:
            found.append(Path(dirpath))
            dirnames[:] = []          # do not descend into a bag
    return sorted(found)


def classify_bags(pair_dir: Path, bags: list[Path]) -> dict[str, Path]:
    """Split discovered bags into {'slam': dir, 'sam': dir} by path substring.

    "slam" is tested first because it does not contain the substring "sam".
    """
    picked: dict[str, list[Path]] = {"slam": [], "sam": []}
    for b in bags:
        rel = str(b.relative_to(pair_dir)).lower()
        if "slam" in rel:
            picked["slam"].append(b)
        elif "sam" in rel:
            picked["sam"].append(b)
    return picked


# --------------------------------------------------------------------------
# bag probing (only the sam_yolo env has `rosbags`)
# --------------------------------------------------------------------------
PROBE_SRC = r'''
import json, sys
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

out = {}
ts = get_typestore(Stores.ROS2_HUMBLE)
for role, uri in json.loads(sys.argv[1]).items():
    info = {"path": uri, "topics": {}}
    with AnyReader([Path(uri)], default_typestore=ts) as r:
        for c in r.connections:
            e = info["topics"].setdefault(c.topic, {"type": c.msgtype, "count": 0})
            e["count"] += c.msgcount
        info["t_start_ns"] = int(r.start_time)
        info["t_end_ns"] = int(r.end_time)
        info["duration_s"] = round((r.end_time - r.start_time) / 1e9, 3)

        def pick(hint, want_img=True, exclude=""):
            # NOTE: "color/image_raw" is a substring of
            # "aligned_depth_to_color/image_raw" -> the colour picks must
            # explicitly exclude the depth topics.
            for t, m in sorted(info["topics"].items()):
                if exclude and exclude in t:
                    continue
                if hint in t and (("Image" in m["type"]) == want_img):
                    return t
            return ""

        info["color_topic"] = pick("color/image_raw", exclude="depth")
        info["depth_topic"] = pick("aligned_depth_to_color/image_raw")
        info["caminfo_topic"] = pick("color/camera_info", want_img=False,
                                     exclude="depth")

        if info["caminfo_topic"]:
            con = [c for c in r.connections if c.topic == info["caminfo_topic"]]
            for c, _t, raw in r.messages(connections=con):
                m = r.deserialize(raw, c.msgtype)
                info["camera"] = {
                    "width": int(m.width), "height": int(m.height),
                    "K": [float(v) for v in m.k], "D": [float(v) for v in m.d],
                    "distortion_model": str(m.distortion_model),
                }
                break
        if info["color_topic"]:
            con = [c for c in r.connections if c.topic == info["color_topic"]]
            n = 0
            for c, _t, raw in r.messages(connections=con):
                m = r.deserialize(raw, c.msgtype)
                info["color_encoding"] = str(m.encoding)
                n += 1
                break
            info["color_count"] = info["topics"][info["color_topic"]]["count"]
            if info["duration_s"] > 0:
                info["color_hz"] = round(info["color_count"] / info["duration_s"], 2)
    out[role] = info
Path(sys.argv[2]).write_text(json.dumps(out, indent=1))
'''


def probe_bags(bags: dict[str, Path], out_json: Path, work: Path) -> dict:
    if not PY_ISM.is_file():
        die(f"python for bag probing not found: {PY_ISM}")
    probe_py = work / "_probe_bag.py"
    probe_py.write_text(PROBE_SRC, encoding="utf-8")
    spec = json.dumps({k: str(v) for k, v in bags.items()})
    res = subprocess.run([str(PY_ISM), str(probe_py), spec, str(out_json)],
                         capture_output=True, text=True)
    if res.returncode != 0:
        die(f"bag probe failed:\n{res.stdout}\n{res.stderr}")
    return json.loads(out_json.read_text())


# --------------------------------------------------------------------------
# shell helpers
# --------------------------------------------------------------------------
def run_shell(script: str, log_path: Path, cwd: Path, timeout: float,
              title: str) -> dict:
    """Run a bash snippet, tee-ing combined output to log_path."""
    log(f"{title}  -> {log_path}")
    t0 = time.time()
    with log_path.open("wb") as fh:
        fh.write(f"# {title}\n# cwd={cwd}\n{script}\n{'-' * 70}\n".encode())
        fh.flush()
        proc = subprocess.Popen(["bash", "-c", script], cwd=str(cwd),
                                stdout=fh, stderr=subprocess.STDOUT,
                                preexec_fn=os.setsid)
        try:
            rc = proc.wait(timeout=timeout)
            timed_out = False
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)   # lets ORB save its map
            try:
                rc = proc.wait(timeout=90)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                rc = proc.wait()
    dt = time.time() - t0
    status = "TIMEOUT" if timed_out else ("ok" if rc == 0 else f"rc={rc}")
    log(f"{title}  [{status}] {dt:.1f}s")
    return {"returncode": rc, "timed_out": timed_out, "seconds": round(dt, 1),
            "log": str(log_path)}


def tail(path: Path, n: int = 25) -> str:
    try:
        lines = path.read_text(errors="replace").splitlines()
    except OSError:
        return ""
    return "\n".join(lines[-n:])


def relink(link: Path, target: Path) -> None:
    """Point `link` at `target`, refusing to clobber a real directory."""
    if link.is_symlink():
        link.unlink()
    elif link.exists():
        die(f"refusing to replace real path with a symlink: {link}")
    link.parent.mkdir(parents=True, exist_ok=True)
    link.symlink_to(target)


# --------------------------------------------------------------------------
# stage 1 — ORB-SLAM3 on the SLAM bag
# --------------------------------------------------------------------------
ORB_SETTINGS_TMPL = """%YAML:1.0

# auto-generated by integration/run_integration.py from {src}
File.version: "1.0"
Camera.type: "PinHole"

Camera1.fx: {fx:.5f}
Camera1.fy: {fy:.5f}
Camera1.cx: {cx:.5f}
Camera1.cy: {cy:.5f}

Camera1.k1: {k1:.8f}
Camera1.k2: {k2:.8f}
Camera1.p1: {p1:.8f}
Camera1.p2: {p2:.8f}
Camera1.k3: {k3:.8f}

Camera.width: {width}
Camera.height: {height}
Camera.fps: {fps}
Camera.RGB: 0          # rgbd_node always hands ORB-SLAM3 a BGR cv::Mat

Stereo.ThDepth: 40.0
Stereo.b: {baseline:.5f}

RGBD.DepthMapFactor: 1000.0

ORBextractor.nFeatures: 1250
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
"""


def find_session_settings(bag_dir: Path, pair_dir: Path) -> Path | None:
    """An ORB settings yaml shipped with the recording (converted sessions have one)."""
    seen = []
    for base in (bag_dir.parent, bag_dir.parent.parent, pair_dir):
        d = base / "settings"
        if d.is_dir():
            seen += sorted(d.glob("*.yaml"))
    for p in seen:
        if "Camera1.fx" in p.read_text(errors="replace"):
            return p
    return None


def write_orb_settings(info: dict, dest: Path, baseline: float) -> Path:
    cam = info.get("camera")
    if not cam:
        die("SLAM bag has no color/camera_info; pass --orb-settings <yaml> explicitly")
    K, D = cam["K"], list(cam["D"]) + [0.0] * 5
    fps = int(round(info.get("color_hz") or 30)) or 30
    dest.write_text(ORB_SETTINGS_TMPL.format(
        src=info["path"], fx=K[0], fy=K[4], cx=K[2], cy=K[5],
        k1=D[0], k2=D[1], p1=D[2], p2=D[3], k3=D[4],
        width=cam["width"], height=cam["height"], fps=fps, baseline=baseline),
        encoding="utf-8")
    return dest


def run_slam_stage(pair: str, bag_dir: Path, info: dict, out_dir: Path,
                   args) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    color = info.get("color_topic") or COLOR_HINT
    depth = info.get("depth_topic")
    if not depth:
        die(f"SLAM bag has no '{DEPTH_HINT}' topic — ORB-SLAM3 RGB-D needs aligned depth")

    # camera settings: CLI > shipped with the session > generated from camera_info
    if args.orb_settings:
        settings = Path(args.orb_settings).expanduser().resolve()
        origin = "cli"
    else:
        shipped = find_session_settings(bag_dir, DATA_DIR / pair)
        if shipped:
            settings = out_dir / "orb_settings.yaml"
            shutil.copyfile(shipped, settings)
            origin = f"session:{shipped}"
        else:
            settings = write_orb_settings(info, out_dir / "orb_settings.yaml",
                                          args.baseline)
            origin = "generated-from-camera_info"
    log(f"ORB settings [{origin}] {settings}")

    cfg_path = out_dir / "orb_config.yaml"
    cfg_path.write_text(f"""# auto-generated by integration/run_integration.py
dataset: {pair}
bag:
  path: {bag_dir}
  play: true
  clock: true
  rate: {args.rate}
  start_delay: {args.start_delay}
orbslam3:
  vocabulary_path: {ORB_VOCAB}
  settings_path: {settings}
topics:
  rgb: {color}
  depth: {depth}
frames:
  world: map
  camera: camera_color_optical_frame
runtime:
  save_map_points_on_shutdown: true
dense_map:
  enabled: {"true" if args.dense else "false"}
  save_pcd: {"true" if args.dense else "false"}
  pcd_output_path: orbslam3_dense_map.pcd
output:
  dir: {out_dir}
  map_points_path: orbslam3_map_points.pcd
visualization:
  enabled: false
""", encoding="utf-8")

    for stale in ("CameraTrajectory.txt", "KeyFrameTrajectory.txt", "trajectory.txt",
                  "keyframes.txt", "orbslam3_map_points.pcd", "orbslam3_dense_map.pcd"):
        p = out_dir / stale
        if p.is_file():
            p.unlink()

    # NOTE: no `set -u` — the RoboStack activation hooks reference unbound vars.
    script = f"""source {CONDA_SH}
conda activate {ENV_ORB}
source {ORB_WS}/install/setup.bash
ros2 launch orbslam3_ros2 orb_slam.launch.py config:={cfg_path}
"""
    play_s = (info.get("duration_s") or 120.0) / max(args.rate, 1e-3)
    timeout = args.slam_timeout or (play_s + 900.0)
    res = run_shell(script, out_dir / "orbslam3_launch.log", ORB_WS, timeout,
                    f"ORB-SLAM3 RGB-D  ({info.get('color_count', '?')} frames, "
                    f"{info.get('duration_s', '?')}s @ rate {args.rate})")

    traj = out_dir / "CameraTrajectory.txt"
    if traj.is_file():
        shutil.copyfile(traj, out_dir / "trajectory.txt")     # object-memory name
    kf = out_dir / "KeyFrameTrajectory.txt"
    if kf.is_file():
        shutil.copyfile(kf, out_dir / "keyframes.txt")

    n_pose = sum(1 for _ in traj.open()) if traj.is_file() else 0
    res.update({
        "bag": str(bag_dir), "config": str(cfg_path), "settings": str(settings),
        "settings_origin": origin, "rgb_topic": color, "depth_topic": depth,
        "trajectory": str(out_dir / "trajectory.txt") if n_pose else None,
        "poses": n_pose,
        "map_points": str(out_dir / "orbslam3_map_points.pcd")
                      if (out_dir / "orbslam3_map_points.pcd").is_file() else None,
        "dense_map": str(out_dir / "orbslam3_dense_map.pcd")
                     if (out_dir / "orbslam3_dense_map.pcd").is_file() else None,
        "ok": n_pose > 0,
    })
    if not res["ok"]:
        log("ORB-SLAM3 produced no trajectory; last log lines:\n"
            + tail(out_dir / "orbslam3_launch.log"))
    return res


# --------------------------------------------------------------------------
# stage 2 — SAM-6D (ISM in sam_yolo, PEM in sam6d_ros_humble)
# --------------------------------------------------------------------------
def summarize_sam(pem_dir: Path, out_json: Path) -> dict:
    """manifest.csv + per-bundle detection_pem.json -> one flat object list."""
    manifest = pem_dir / "manifest.csv"
    rows, objects = [], {}
    if manifest.is_file():
        with manifest.open(newline="") as f:
            rows = list(csv.DictReader(f))

    dets = []
    for r in rows:
        # the manifest addresses frames through the sam6d_ws alias symlink;
        # resolve() brings them back to their real integration/output path
        fdir = Path(r["frame_dir"]).resolve()
        obj = r["object"]
        frame = fdir.name
        m = re.search(r"frame_(\d+)", frame)
        pem_json = fdir / f"pem_{obj}" / "sam6d_results" / "detection_pem.json"
        entry = {
            "frame": frame,
            "frame_index": int(m.group(1)) if m else None,
            "object": obj,
            "ism_detection": str(fdir / f"detection_{obj}.json"),
            "pem_ok": pem_json.is_file(),
        }
        if pem_json.is_file():
            try:
                best = max(json.loads(pem_json.read_text()),
                           key=lambda d: d.get("score", 0.0))
                entry.update({
                    "score": round(float(best.get("score", 0.0)), 4),
                    "R": best.get("R"),
                    "t_mm": best.get("t"),
                    "bbox": best.get("bbox"),
                    "vis": str(fdir / f"pem_{obj}" / "sam6d_results" / "vis_pem.png"),
                })
            except (ValueError, OSError, KeyError) as e:
                entry["error"] = f"{type(e).__name__}: {e}"
        dets.append(entry)
        o = objects.setdefault(obj, {"frames": 0, "pem_ok": 0, "best_score": 0.0})
        o["frames"] += 1
        o["pem_ok"] += int(entry["pem_ok"])
        o["best_score"] = max(o["best_score"], entry.get("score", 0.0))

    summary = {
        "pem_inputs": str(pem_dir),
        "frames_with_detection": len({d["frame"] for d in dets}),
        "bundles": len(dets),
        "objects": objects,
        "detections": dets,
    }
    out_json.write_text(json.dumps(summary, indent=1), encoding="utf-8")
    return summary


def run_sam_stage(pair: str, bag_dir: Path, info: dict, out_dir: Path,
                  args) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    for hint, key in ((COLOR_HINT, "color_topic"), (DEPTH_HINT, "depth_topic"),
                      (CAMINFO_HINT, "caminfo_topic")):
        if not info.get(key):
            die(f"SAM bag is missing a '{hint}' topic — SAM-6D needs RGB + aligned "
                f"depth + intrinsics from the same bag")

    # The SAM-6D tools address a bag by NAME under sam6d_ws/data/ros2_bag and write
    # to sam6d_ws/outputs/pem_inputs/<name> (both hard-coded). Symlink both ends so
    # the tools stay untouched and the artefacts still live in integration/output.
    link = f"integration_{pair}"
    pem_dir = out_dir / "pem_inputs"
    if pem_dir.is_dir() and not args.keep_sam_outputs:
        # frames selected by a previous run with a different --max-frames would
        # otherwise linger and show up in the vis_pem_all.png glob
        shutil.rmtree(pem_dir)
    pem_dir.mkdir(parents=True, exist_ok=True)
    relink(SAM_BAG_ROOT / link, bag_dir)
    relink(SAM_PEM_ROOT / link, pem_dir)
    log(f"SAM-6D bag alias '{link}' -> {bag_dir}")

    obj_arg = (" --objects " + " ".join(args.sam_objects)) if args.sam_objects else ""
    frame_arg = (f" --stride {args.sam_stride}" if args.sam_stride
                 else "") + f" --max-frames {args.max_frames}"

    res = {"bag": str(bag_dir), "alias": link, "pem_inputs": str(pem_dir)}

    # -- stage A: ISM (YOLO-World + DINOv2 + MobileSAM) -> RLE seg bundles -----
    a = run_shell(
        f"{PY_ISM} -u {SAM_WS}/tools/build_pem_inputs.py "
        f"--bag {link}{frame_arg}{obj_arg}\n",
        out_dir / "ism.log", SAM_WS, args.sam_timeout, "SAM-6D ISM (sam_yolo)")
    res["ism"] = a
    manifest = pem_dir / "manifest.csv"
    if a["returncode"] != 0 or not manifest.is_file():
        res["ok"] = False
        log("ISM stage failed; last log lines:\n" + tail(out_dir / "ism.log"))
        return res
    n_bundles = max(0, sum(1 for _ in manifest.open()) - 1)
    log(f"ISM accepted {n_bundles} (frame,object) bundles")
    if n_bundles == 0:
        res.update({"ok": False, "reason": "ISM accepted no detections"})
        return res

    # -- stage B: PEM (6D pose per bundle) ------------------------------------
    res["pem"] = run_shell(
        f"{PY_PEM} -u {SAM_WS}/tools/run_pem_batch.py --bags {link}\n",
        out_dir / "pem.log", SAM_WS, args.sam_timeout, "SAM-6D PEM (sam6d_ros_humble)")

    # -- stage C: per-frame all-object pose overlay ---------------------------
    res["viz"] = run_shell(
        f"{PY_PEM} -u {SAM_WS}/tools/viz_pem_multiobject.py --bags {link}\n",
        out_dir / "viz.log", SAM_WS, args.sam_timeout, "SAM-6D pose overlay")

    summary = summarize_sam(pem_dir, out_dir / "sam_objects.json")
    res.update({
        "bundles": summary["bundles"],
        "frames_with_detection": summary["frames_with_detection"],
        "objects": summary["objects"],
        "sam_objects": str(out_dir / "sam_objects.json"),
        "ok": summary["bundles"] > 0,
    })
    return res


# --------------------------------------------------------------------------
# stage 3 — fuse: SLAM trajectory x SAM-6D poses -> object map in the SLAM map frame
# --------------------------------------------------------------------------
# SAM camera as mounted on this rig, expressed in the SLAM camera's optical
# frame (X right, Y down, Z forward): 215 mm right, 380 mm down, yawed 90 deg
# to the right. R_X maps SAM-forward onto SLAM-right and shares the down axis.
RIG_R = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
RIG_T = (0.215, 0.380, 0.0)     # right, down, forward [m]


def load_npy_4x4(path: Path) -> tuple:
    """Read a 4x4 float .npy without numpy (the runner stays stdlib-only)."""
    import ast
    import struct
    with path.open("rb") as f:
        if f.read(6) != b"\x93NUMPY":
            die(f"not a .npy file: {path}")
        major, _minor = f.read(2)
        hlen = int.from_bytes(f.read(2 if major == 1 else 4), "little")
        head = ast.literal_eval(f.read(hlen).decode("latin1").strip())
        if tuple(head["shape"]) != (4, 4) or head["fortran_order"]:
            die(f"expected a C-order 4x4 matrix in {path}, got {head}")
        descr = head["descr"]
        fmt = "<16d" if descr.endswith("f8") else "<16f"
        v = struct.unpack(fmt, f.read(struct.calcsize(fmt)))
    return tuple(tuple(v[r * 4 + c] for c in range(4)) for r in range(4))


def find_calib_npy(sam_bag: Path, pair_dir: Path) -> list:
    """4x4 extrinsics shipped with the recording (converted sessions carry one).

    Searched explicitly instead of with `**` because the pair folders normally
    hold symlinks to the real sessions and pathlib does not recurse into those.
    """
    found = []
    for base in (sam_bag.parent, sam_bag.parent.parent, pair_dir):
        d = base / "calib"
        if d.is_dir():
            found += sorted(d.glob("*.npy"))
    seen, uniq = set(), []
    for p in found:
        r = p.resolve()
        if r not in seen:
            seen.add(r)
            uniq.append(p)
    return uniq


def load_pem_camera(pem_dir: Path):
    """cam_K + image size from the PEM input bundles (drives the P_D FOV model)."""
    for cam_path in sorted(pem_dir.glob("frame_*/camera.json")):
        try:
            k = json.loads(cam_path.read_text())["cam_K"]
        except (OSError, KeyError, ValueError):
            continue
        rgb = cam_path.parent / "rgb.png"
        size = None
        if rgb.is_file():
            head = rgb.open("rb").read(24)
            if len(head) >= 24 and head[12:16] == b"IHDR":
                size = (int.from_bytes(head[16:20], "big"),
                        int.from_bytes(head[20:24], "big"))
        if size:
            return ((k[0], k[1], k[2]), (k[3], k[4], k[5]), (k[6], k[7], k[8])), size
    return None, None


def quat_from_R(T) -> list:
    """(qx, qy, qz, qw) from the rotation block of a 4x4 row-major transform."""
    import math
    m = T
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw, qx = 0.25 * s, (m[2][1] - m[1][2]) / s
        qy, qz = (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        qw, qx = (m[2][1] - m[1][2]) / s, 0.25 * s
        qy, qz = (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        qw, qx = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s
        qy, qz = 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        qw, qx = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s
        qy, qz = (m[1][2] + m[2][1]) / s, 0.25 * s
    return [qx, qy, qz, qw]


def write_topview_svg(traj: list, objects: list, path: Path,
                      title: str, margin: int = 46, size: int = 900) -> None:
    """Map-frame top view (x right, z up) of the camera path + fused objects.

    Plain SVG so the runner needs no plotting library. The map frame is the
    first camera pose, i.e. optical axes: x right, y down, z forward, so the
    ground plane is x-z and y is height.
    """
    pts = [(p[0], p[2]) for p in traj]
    pts += [(o["xyz"][0], o["xyz"][2]) for o in objects]
    if len(pts) < 2:
        return
    xs, zs = [p[0] for p in pts], [p[1] for p in pts]
    x0, x1, z0, z1 = min(xs), max(xs), min(zs), max(zs)
    span = max(x1 - x0, z1 - z0, 1e-3)
    scale = (size - 2 * margin) / span

    def px(x, z):
        return (margin + (x - x0) * scale,
                size - margin - (z - z0) * scale)      # +z up on screen

    body = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}" '
            f'viewBox="0 0 {size} {size}" font-family="sans-serif">',
            f'<rect width="{size}" height="{size}" fill="#ffffff"/>',
            f'<text x="{margin}" y="26" font-size="16" fill="#222">{title}</text>',
            f'<text x="{margin}" y="{size - 14}" font-size="12" fill="#666">'
            f'map frame top view: x right, z up (metres), 1 grid = 1 m</text>']
    # 1 m grid
    import math
    gx = math.floor(x0)
    while gx <= x1 + 1:
        x, _ = px(gx, z0)
        body.append(f'<line x1="{x:.1f}" y1="{margin}" x2="{x:.1f}" '
                    f'y2="{size - margin}" stroke="#eee" stroke-width="1"/>')
        gx += 1
    gz = math.floor(z0)
    while gz <= z1 + 1:
        _, y = px(x0, gz)
        body.append(f'<line x1="{margin}" y1="{y:.1f}" x2="{size - margin}" '
                    f'y2="{y:.1f}" stroke="#eee" stroke-width="1"/>')
        gz += 1
    # camera path
    poly = " ".join(f"{px(p[0], p[2])[0]:.1f},{px(p[0], p[2])[1]:.1f}" for p in traj)
    body.append(f'<polyline points="{poly}" fill="none" stroke="#2b6cb0" '
                f'stroke-width="1.6" opacity="0.85"/>')
    if traj:
        sx, sy = px(traj[0][0], traj[0][2])
        body.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="5" fill="#2b6cb0"/>'
                    f'<text x="{sx + 8:.1f}" y="{sy - 6:.1f}" font-size="11" '
                    f'fill="#2b6cb0">start</text>')
    # objects
    palette = {"active": "#c53030", "remembered": "#805ad5",
               "lost": "#dd6b20", "tentative": "#718096"}
    placed = []                      # label anchors, to push overlapping ones apart
    for o in sorted(objects, key=lambda o: o["xyz"][2], reverse=True):
        x, y = px(o["xyz"][0], o["xyz"][2])
        c = palette.get(o["status"], "#718096")
        body.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" fill="{c}" '
                    f'fill-opacity="0.85" stroke="#fff" stroke-width="1.5"/>')
        lx, ly = x + 9, y + 4
        while any(abs(ly - py_) < 13 and abs(lx - px_) < 150 for px_, py_ in placed):
            ly += 13
        placed.append((lx, ly))
        if ly - y > 8:               # leader line when the label had to move
            body.append(f'<line x1="{x:.1f}" y1="{y:.1f}" x2="{lx - 2:.1f}" '
                        f'y2="{ly - 4:.1f}" stroke="{c}" stroke-width="0.8" '
                        f'opacity="0.5"/>')
        body.append(f'<text x="{lx:.1f}" y="{ly:.1f}" font-size="12" fill="#111">'
                    f'{o["object_name"]} #{o["object_id"]}</text>')
    # legend
    ly = 46
    for st, c in palette.items():
        body.append(f'<circle cx="{margin + 6}" cy="{ly}" r="5" fill="{c}"/>'
                    f'<text x="{margin + 18}" y="{ly + 4}" font-size="12" '
                    f'fill="#444">{st}</text>')
        ly += 18
    body.append("</svg>")
    path.write_text("\n".join(body), encoding="utf-8")


def run_fuse_stage(pair: str, sam_bag: Path, info: dict, out_root: Path,
                   args) -> dict:
    """T_map_obj = T_map_camSLAM(t) * X * T_camSAM_obj, fused into object memory.

    Reuses objectmemory_ws/object_memory (adapters + StreamingObjectMemory) as a
    library. X is applied to the POSE stream rather than to the detections so the
    existence filter's P_D sees the frustum of the camera that actually made the
    detection (validated in run_object_memory_extrinsic.py).
    """
    import dataclasses

    out_dir = out_root / getattr(args, "fused_dir", "fused")
    out_dir.mkdir(parents=True, exist_ok=True)
    slam_dir = out_root / getattr(args, "slam_dir", "slam")
    sam_dir = out_root / "sam"
    traj_path = slam_dir / "trajectory.txt"
    pem_dir = sam_dir / "pem_inputs"
    if not traj_path.is_file():
        die(f"no SLAM trajectory at {traj_path} — run --stage slam first")
    if not (pem_dir / "manifest.csv").is_file():
        die(f"no SAM-6D output at {pem_dir} — run --stage sam first")
    if not OBJMEM_SRC.is_dir():
        die(f"object_memory library not found: {OBJMEM_SRC}")

    sys.path.insert(0, str(OBJMEM_SRC))
    from adapters.bag_frame_index import load_frame_timestamps
    from adapters.sam6d_pem import load_pem_detections
    from adapters.slam_trajectory import load_slam_trajectory
    from core.transforms import compose_transform, invert_transform, make_transform
    from pipeline.object_memory_runner import run_object_memory

    # ---- extrinsic X = T_camSLAM_camSAM ---------------------------------
    # auto: the hand-eye calibration shipped with the session if there is one.
    # It matters a lot: on 260804 longcircle2 the calibrated X gives 29 object
    # instances (validated reference) where the hand-measured rig estimate gives
    # 125 — a wrong X smears each object along the camera path, because the same
    # physical object then lands somewhere else for every viewpoint.
    if args.extrinsic == "auto":
        cals = find_calib_npy(sam_bag, DATA_DIR / pair)
        if len(cals) > 1:
            die("several calibration files found; pick one with --extrinsic:\n  "
                + "\n  ".join(str(c) for c in cals))
        if cals:
            args.extrinsic = str(cals[0])
        else:
            args.extrinsic = "rig"
            log("no calib/*.npy next to the SAM bag -> falling back to the "
                "hand-measured rig extrinsic (UNCALIBRATED; if objects smear "
                "along the camera path, this is why)")
    if args.extrinsic == "rig":
        X = make_transform(RIG_R, (args.rig_right, args.rig_down, args.rig_forward))
        x_origin = (f"rig estimate (right {args.rig_right}, down {args.rig_down}, "
                    f"forward {args.rig_forward}, yaw-right 90 deg)")
    elif args.extrinsic == "identity":
        X = make_transform(((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0))
        x_origin = "identity (single camera / SAM bag already in the SLAM camera frame)"
    else:
        xp = Path(args.extrinsic).expanduser()
        if not xp.is_file():
            die(f"--extrinsic must be 'rig', 'identity' or a 4x4 .npy path: {args.extrinsic}")
        X = load_npy_4x4(xp)
        x_origin = f"npy {xp}"
    if args.invert_extrinsic:
        X = invert_transform(X)
        x_origin += " (inverted)"
    log(f"extrinsic T_camSLAM_camSAM: {x_origin}")
    for row in X:
        log("    " + "  ".join(f"{v:8.4f}" for v in row))

    # ---- inputs ---------------------------------------------------------
    poses = load_slam_trajectory(str(traj_path), source_slam_id="orbslam3")
    timestamps = load_frame_timestamps(
        str(sam_bag), color_topic_hint=info.get("color_topic")
        or "/camera/camera/color/image_raw")
    if args.time_offset:
        timestamps = [t + args.time_offset for t in timestamps]
        log(f"applied --time-offset {args.time_offset:+.4f}s to the SAM frame stamps")
    detections = load_pem_detections(str(pem_dir), timestamps=timestamps)
    n_det = sum(len(v) for v in detections.values())
    log(f"{len(poses)} SLAM poses, {len(timestamps)} SAM color stamps, "
        f"{n_det} detections in {len(detections)} frames")
    if len(detections) < 40:
        log("NOTE: sparse detection stream. An object needs >=2 observations to be "
            "promoted, so a thin sample leaves most instances tentative — rerun the "
            "SAM stage with e.g. --sam-stride 10 --max-frames 400 for a real map")

    # SLAM and SAM stamps must overlap or every frame is dropped (INV-008).
    if poses and timestamps:
        overlap = (min(poses[-1].stamp, timestamps[-1]) -
                   max(poses[0].stamp, timestamps[0]))
        if overlap <= 0:
            log(f"WARNING: SLAM [{poses[0].stamp:.3f}, {poses[-1].stamp:.3f}] and SAM "
                f"[{timestamps[0]:.3f}, {timestamps[-1]:.3f}] stamps do not overlap; "
                f"the two laptops' clocks likely differ — pass --time-offset")

    # X onto the pose stream: T_map_camSAM = T_map_camSLAM * X
    poses = [dataclasses.replace(p, T_map_cam=compose_transform(p.T_map_cam, X))
             for p in poses]
    cam_K, img_size = load_pem_camera(pem_dir)

    result = run_object_memory(
        poses, detections, timestamps,
        time_tolerance=args.time_tolerance,
        assoc_trans_gate_m=args.assoc_gate, assoc_rot_gate_deg=-1.0,
        cam_K=cam_K, img_size=img_size, quality_weighting=True,
    )

    # ---- results --------------------------------------------------------
    objects = []
    for lm in sorted(result.store.all_landmarks(), key=lambda x: x.object_id):
        T = lm.T_map_obj
        status = lm.status.value if hasattr(lm.status, "value") else str(lm.status)
        objects.append({
            "object_id": lm.object_id, "object_name": lm.object_name,
            "status": status, "confidence": round(lm.confidence, 4),
            "observations": lm.observation_count, "missed": lm.missed_count,
            "first_seen": lm.first_seen_time, "last_seen": lm.last_seen_time,
            "xyz": [round(T[0][3], 4), round(T[1][3], 4), round(T[2][3], 4)],
            "quat_xyzw": [round(v, 6) for v in quat_from_R(T)],
            "T_map_obj": [[round(v, 6) for v in row] for row in T],
            # 관측을 받아들일 때마다의 융합 포즈. fuse_pose 의 가중치가 1/(n+1) 이라
            # 초반 큰 보정이 1/n 으로 잦아드는데, 최종값만 두면 그게 보이지 않는다.
            "pose_history": [
                {"t": round(s.stamp, 6), "n": s.observation_count,
                 "xyz": [round(v, 4) for v in s.xyz],
                 "quat_xyzw": [round(v, 6) for v in s.quat_xyzw]}
                for s in getattr(lm, "pose_history", [])
            ],
        })
    live = [o for o in objects if o["status"] != "deleted"]

    traj_xyz = [(p.T_map_cam[0][3], p.T_map_cam[1][3], p.T_map_cam[2][3])
                for p in poses]
    write_topview_svg(traj_xyz, live, out_dir / "map_topview.svg",
                      f"{pair} — ORB-SLAM3 path + SAM-6D objects (map frame)")

    obj_map = {
        "pair": pair,
        "frame": "map (ORB-SLAM3 world = first SLAM camera pose)",
        "formula": "T_map_obj = T_map_camSLAM(t) * X_camSLAM_camSAM * T_camSAM_obj",
        "units": "metres / row-major 4x4",
        "inputs": {"slam_trajectory": str(traj_path), "pem_dir": str(pem_dir),
                   "sam_bag": str(sam_bag)},
        "extrinsic": {"origin": x_origin, "X_camSLAM_camSAM": [list(r) for r in X],
                      "applied_to": "poses"},
        "time_offset_s": args.time_offset,
        "stats": {
            "slam_poses": len(poses), "sam_color_stamps": len(timestamps),
            "frames_with_detections": result.frames_total,
            "frames_fused": result.frames_with_slam,
            "frames_dropped_no_slam_pose": result.frames_without_slam,
            "detections": result.detections_total,
            "instances": len(objects), "live_instances": len(live),
        },
        "objects": objects,
    }
    (out_dir / "object_map.json").write_text(json.dumps(obj_map, indent=1),
                                             encoding="utf-8")

    # the object_memory markdown report, generated by its own helper
    report = out_dir / "object_memory_report.md"
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "_rom", str(OBJMEM / "scripts_test" / "run_object_memory.py"))
        rom = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(rom)
        rom.write_report(result, str(report), {
            "slam_traj": str(traj_path), "pem_dir": str(pem_dir),
            "bag": str(sam_bag), "n_poses": len(poses),
            "n_timestamps": len(timestamps)})
    except Exception as e:                       # report is a nicety, not the result
        log(f"markdown report skipped ({type(e).__name__}: {e})")
        report = None

    return {
        "ok": result.frames_with_slam > 0 and bool(objects),
        "extrinsic": x_origin,
        "object_map": str(out_dir / "object_map.json"),
        "topview": str(out_dir / "map_topview.svg"),
        "report": str(report) if report else None,
        **obj_map["stats"],
    }


# --------------------------------------------------------------------------
def main() -> int:
    p = argparse.ArgumentParser(
        description="Run ORB-SLAM3 (SLAM bag) and SAM-6D (SAM bag) from one input folder.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("pair", help="folder name under integration/data/ (or a path)")
    p.add_argument("--stage", choices=["all", "slam", "sam", "fuse"], default="all")
    p.add_argument("--slam-dir", default="slam",
                   help="subdirectory under the output root holding the SLAM "
                        "trajectory (default 'slam'). Use 'slam_orb3' / 'slam_rtabmap' "
                        "/ 'slam_hdl' to keep several SLAM back-ends side by side.")
    p.add_argument("--fused-dir", default="fused",
                   help="subdirectory for the fusion result (default 'fused'). "
                        "Pair it with --slam-dir to fuse each back-end separately.")
    p.add_argument("--probe-only", action="store_true",
                   help="just discover + probe the two bags, run nothing")
    p.add_argument("--out", default="", help="output dir (default integration/output/<pair>)")
    p.add_argument("--slam-bag", default="", help="override SLAM bag dir")
    p.add_argument("--sam-bag", default="", help="override SAM bag dir")
    # ORB-SLAM3
    p.add_argument("--rate", type=float, default=1.0, help="ros2 bag play rate")
    p.add_argument("--start-delay", type=float, default=12.0,
                   help="seconds before bag play starts (vocabulary load)")
    p.add_argument("--orb-settings", default="", help="explicit ORB-SLAM3 settings yaml")
    p.add_argument("--baseline", type=float, default=0.095,
                   help="Stereo.b used only when settings are generated (D455/D435i ~0.095/0.050)")
    p.add_argument("--no-dense", dest="dense", action="store_false",
                   help="skip the dense point-cloud map")
    p.add_argument("--slam-timeout", type=float, default=0.0,
                   help="0 = bag duration/rate + 900 s")
    # SAM-6D
    p.add_argument("--max-frames", type=int, default=30,
                   help="color frames sampled from the SAM bag (evenly spread)")
    p.add_argument("--sam-stride", type=int, default=0,
                   help="use every Nth color frame instead of even spread")
    p.add_argument("--sam-objects", nargs="+", default=[],
                   help="restrict to these object names from configs/yolo_ism_objects.yaml")
    p.add_argument("--sam-timeout", type=float, default=7200.0)
    p.add_argument("--keep-sam-outputs", action="store_true",
                   help="do not wipe sam/pem_inputs/ before the SAM stage")
    # fusion (stage 3) — everything lands in the SLAM camera's map frame
    p.add_argument("--extrinsic", default="auto",
                   help="T_camSLAM_camSAM: 'auto' (calib/*.npy next to the SAM bag, "
                        "else the rig estimate), 'rig' (hand-measured mount), "
                        "'identity' (one camera), or a path to a 4x4 .npy")
    p.add_argument("--invert-extrinsic", action="store_true",
                   help="use X^-1 (flip if the object map mirrors the room)")
    p.add_argument("--rig-right", type=float, default=RIG_T[0],
                   help="rig extrinsic: SAM camera offset to the right [m]")
    p.add_argument("--rig-down", type=float, default=RIG_T[1],
                   help="rig extrinsic: SAM camera offset downward [m]")
    p.add_argument("--rig-forward", type=float, default=RIG_T[2],
                   help="rig extrinsic: SAM camera offset forward [m]")
    p.add_argument("--time-offset", type=float, default=0.0,
                   help="seconds added to the SAM frame stamps (laptop clock skew)")
    p.add_argument("--time-tolerance", type=float, default=0.2,
                   help="max |t_detection - t_pose| to accept a SLAM pose [s]")
    p.add_argument("--assoc-gate", type=float, default=0.20,
                   help="association gate in map space [m]")
    args = p.parse_args()

    pair_dir = resolve_pair_dir(args.pair)
    pair = pair_dir.name
    out_root = Path(args.out).expanduser().resolve() if args.out else OUTPUT_DIR / pair
    out_root.mkdir(parents=True, exist_ok=True)

    print(BANNER)
    print(f"# integration run: {pair}")
    print(f"#   input : {pair_dir}")
    print(f"#   output: {out_root}")
    print(BANNER, flush=True)

    # ---- locate the two bags --------------------------------------------
    bags: dict[str, Path] = {}
    if args.slam_bag:
        bags["slam"] = Path(args.slam_bag).expanduser().resolve()
    if args.sam_bag:
        bags["sam"] = Path(args.sam_bag).expanduser().resolve()
    if len(bags) < 2:
        found = find_bag_dirs(pair_dir)
        if not found:
            die(f"no rosbag2 directory (metadata.yaml) found under {pair_dir}")
        picked = classify_bags(pair_dir, found)
        for role in ("slam", "sam"):
            if role in bags:
                continue
            cands = picked[role]
            if len(cands) != 1:
                die(f"expected exactly 1 {role.upper()} bag under {pair_dir}, found "
                    f"{len(cands)}: {[str(c) for c in cands]}\n"
                    f"  all bags: {[str(c) for c in found]}\n"
                    f"  fix by naming folders slam/ and sam/, or pass --{role}-bag")
            bags[role] = cands[0]
    for role, b in bags.items():
        if not (b / "metadata.yaml").is_file():
            die(f"{role} bag is not a rosbag2 directory: {b}")
        log(f"{role.upper():4s} bag: {b}")

    # ---- probe both bags -------------------------------------------------
    info = probe_bags(bags, out_root / "bag_info.json", out_root)
    for role, i in info.items():
        cam = i.get("camera", {})
        K = cam.get("K", [0] * 9)
        log(f"{role.upper():4s} {i.get('color_count', 0)} color frames, "
            f"{i['duration_s']}s, {i.get('color_encoding', '?')}, "
            f"{cam.get('width', '?')}x{cam.get('height', '?')}, "
            f"fx={K[0]:.1f} fy={K[4]:.1f} cx={K[2]:.1f} cy={K[5]:.1f}")

    # keep the other stage's section when only one stage is re-run
    summary_path = out_root / "run_summary.json"
    summary = {}
    if summary_path.is_file():
        try:
            summary = json.loads(summary_path.read_text())
        except ValueError:
            summary = {}
    summary.update({
        "pair": pair, "input": str(pair_dir), "output": str(out_root),
        "stage": args.stage, "args": vars(args),
        "bags": {k: str(v) for k, v in bags.items()},
        "bag_info": str(out_root / "bag_info.json"),
    })

    # ---- run the stages --------------------------------------------------
    if args.probe_only:
        print(json.dumps(info, indent=1))
        log("probe only — nothing executed")
        return 0
    if args.stage in ("all", "slam"):
        print(f"\n{BANNER}\n# STAGE 1 — ORB-SLAM3 on the SLAM bag\n{BANNER}", flush=True)
        summary["slam"] = run_slam_stage(pair, bags["slam"], info["slam"],
                                         out_root / args.slam_dir, args)
    if args.stage in ("all", "sam"):
        print(f"\n{BANNER}\n# STAGE 2 — SAM-6D (ISM -> PEM) on the SAM bag\n{BANNER}",
              flush=True)
        summary["sam"] = run_sam_stage(pair, bags["sam"], info["sam"],
                                       out_root / "sam", args)
    if args.stage in ("all", "fuse"):
        print(f"\n{BANNER}\n# STAGE 3 — fuse into one object map (SLAM map frame)\n"
              f"{BANNER}", flush=True)
        summary["fuse"] = run_fuse_stage(pair, bags["sam"], info["sam"],
                                         out_root, args)

    summary_path.write_text(json.dumps(summary, indent=1), encoding="utf-8")

    # ---- report ----------------------------------------------------------
    print(f"\n{BANNER}\n# RESULT — {pair}\n{BANNER}")
    s = summary.get("slam")
    if s:
        print(f"SLAM  ORB-SLAM3   {'OK ' if s['ok'] else 'FAIL'}  "
              f"{s['poses']} poses  {s['seconds']}s")
        print(f"        trajectory : {s['trajectory']}")
        print(f"        map points : {s['map_points']}")
        print(f"        dense map  : {s['dense_map']}")
        print(f"        log        : {s['log']}")
    m = summary.get("sam")
    if m:
        print(f"SAM   SAM-6D      {'OK ' if m.get('ok') else 'FAIL'}  "
              f"{m.get('bundles', 0)} detections over "
              f"{m.get('frames_with_detection', 0)} frames")
        for name, o in sorted(m.get("objects", {}).items()):
            print(f"        - {name:18s} {o['pem_ok']}/{o['frames']} poses, "
                  f"best score {o['best_score']:.3f}")
        print(f"        poses json : {m.get('sam_objects')}")
        print(f"        overlays   : {m.get('pem_inputs')}/frame_*/vis_pem_all.png")
    f = summary.get("fuse")
    if f:
        print(f"FUSE  object map  {'OK ' if f.get('ok') else 'FAIL'}  "
              f"{f['live_instances']}/{f['instances']} live instances from "
              f"{f['detections']} detections "
              f"({f['frames_fused']}/{f['frames_with_detections']} frames fused)")
        print(f"        extrinsic  : {f['extrinsic']}")
        obj_map = json.loads(Path(f["object_map"]).read_text())
        for o in obj_map["objects"]:
            if o["status"] == "deleted":
                continue
            x, y, z = o["xyz"]
            print(f"        - #{o['object_id']:<3d} {o['object_name']:<20s} "
                  f"[{o['status']:<10s}] obs={o['observations']:<3d} "
                  f"r={o['confidence']:.2f}  xyz=({x:7.3f},{y:7.3f},{z:7.3f})")
        print(f"        object map : {f['object_map']}")
        print(f"        top view   : {f['topview']}")
        print(f"        report     : {f['report']}")
    print(f"summary: {out_root / 'run_summary.json'}")

    ok = all(summary[k].get("ok") for k in ("slam", "sam", "fuse") if k in summary)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
