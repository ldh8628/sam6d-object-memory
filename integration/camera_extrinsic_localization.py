#!/usr/bin/env python3
"""Build an ORB-SLAM3 map, localize its paired SAM RGB-D camera, and write URDF."""
from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from rig_offset import (fit, pair_poses, proj_SO3, read_tum, residuals,
                        rot_angle_deg, slerp_interp)
from camera_publish import runtime_domain


ROOT = Path(__file__).resolve().parents[1]
ORB_WS = ROOT / "orbslam_ws"
ORB_ENV = "realsense"
ORB_INSTALL = Path(os.environ.get("ORB_INSTALL", str(ORB_WS / "install_jazzy")))
CONDA_SH = Path.home() / "miniconda3/etc/profile.d/conda.sh"
RIG_REFERENCE = ROOT / "integration/camera_extrinsic_reference.json"


SETTINGS = """%YAML:1.0

File.version: "1.0"
Camera.type: "PinHole"
Camera1.fx: {fx:.8f}
Camera1.fy: {fy:.8f}
Camera1.cx: {cx:.8f}
Camera1.cy: {cy:.8f}
Camera1.k1: {k1:.8f}
Camera1.k2: {k2:.8f}
Camera1.p1: {p1:.8f}
Camera1.p2: {p2:.8f}
Camera1.k3: {k3:.8f}
Camera.width: {width}
Camera.height: {height}
Camera.fps: {fps}
Camera.RGB: 0
Stereo.ThDepth: 40.0
Stereo.b: {baseline:.5f}
RGBD.DepthMapFactor: 1000.0
ORBextractor.nFeatures: 2000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
{atlas_key}: "{atlas}"
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


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def ros2_argv(argv: list[str]) -> list[str]:
    """Expand ROS-style ``--key:=value`` arguments for argparse."""
    normalized = []
    for argument in argv:
        if argument.startswith("--") and ":=" in argument:
            key, value = argument.split(":=", 1)
            normalized += [key, value]
        else:
            normalized.append(argument)
    return normalized


def path_arg(value: str) -> Path:
    if not value.strip():
        raise argparse.ArgumentTypeError("값이 비어 있다")
    return Path(value)


def default_output(inputs: Path) -> Path:
    dataset = inputs.parent if inputs.name == "converted" else ROOT / "output" / inputs.name
    return (dataset / "camera_extrinsic").resolve()


def progress_line(stage: str, elapsed: float, expected: float, done: bool = False) -> str:
    percent = 100 if done else min(99, int(elapsed / expected * 100))
    filled = min(10, percent // 10)
    finishing = " · 마무리 중" if not done and elapsed >= expected else ""
    return (f"[{'█' * filled}{'░' * (10 - filled)}] {stage} · 추정 {percent:3d}%"
            f" · 경과 {elapsed:.0f}s{finishing}")


def write_settings(info: dict, role: str, path: Path, atlas_key: str,
                   atlas: str, baseline: float) -> None:
    camera = info[role]["camera"]
    K, D = camera["K"], [*camera["D"], 0, 0, 0, 0, 0]
    path.write_text(SETTINGS.format(
        fx=K[0], fy=K[4], cx=K[2], cy=K[5],
        k1=D[0], k2=D[1], p1=D[2], p2=D[3], k3=D[4],
        width=camera["width"], height=camera["height"],
        fps=max(1, round(info[role].get("color_hz", 30))), baseline=baseline,
        atlas_key=atlas_key, atlas=atlas), encoding="utf-8")


def write_config(info: dict, role: str, path: Path, bag: Path, settings: Path,
                 output: Path, rate: float, start_delay: float,
                 localization: bool, dense_map: bool = False) -> None:
    path.write_text(f"""dataset: {output.parent.name}_{role.lower()}
bag:
  path: {bag}
  play: true
  clock: true
  rate: {rate}
  start_delay: {start_delay}
orbslam3:
  vocabulary_path: {ORB_WS / 'src/ORB_SLAM3/Vocabulary/ORBvoc.txt'}
  settings_path: {settings}
topics:
  rgb: {info[role]['color_topic']}
  depth: {info[role]['depth_topic']}
frames:
  world: map
  camera: {role.lower()}_camera_color_optical_frame
runtime:
  localization_mode: {str(localization).lower()}
  publish_map_points: false
  save_map_points_on_shutdown: {str(not localization).lower()}
dense_map:
  enabled: {str(dense_map).lower()}
  frame_stride: 15
  pixel_stride: 8
  voxel_size: 0.05
  min_depth_m: 0.15
  max_depth_m: 6.0
  max_points: 250000
  include_color: false
  save_pcd: {str(dense_map).lower()}
  pcd_output_path: {output / 'orbslam3_dense_map.pcd'}
  reproject_optimized: true
  reproject_max_points: 2000000
output:
  dir: {output}
  map_points_path: orbslam3_map_points.pcd
visualization:
  enabled: false
""", encoding="utf-8")


def run_launch(config: Path, output: Path, timeout: float, stage: str | None = None,
               expected_s: float | None = None) -> None:
    command = (
        f"source {shlex.quote(str(CONDA_SH))} && conda activate {ORB_ENV} && "
        f"source {shlex.quote(str(ORB_INSTALL / 'setup.bash'))} && "
        "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && "
        "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && "
        f"export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(ROOT / 'integration/fastdds_input.xml'))} && "
        f"ros2 launch orbslam3_ros2 orb_slam.launch.py "
        f"config:={shlex.quote(str(config))}"
    )
    launch_log = output / "orbslam3_launch.log"
    log(f"실행: {output}")
    with launch_log.open("wb") as stream:
        process = subprocess.Popen(["bash", "-lc", command], cwd=ORB_WS,
                                   stdout=stream, stderr=subprocess.STDOUT,
                                   preexec_fn=os.setsid)
        started = time.monotonic()
        progress = stage is not None and expected_s is not None and expected_s > 0
        try:
            if progress:
                deadline = started + timeout
                while process.poll() is None:
                    elapsed = time.monotonic() - started
                    print(f"\r{progress_line(stage, elapsed, expected_s)}", end="", flush=True)
                    if time.monotonic() >= deadline:
                        raise subprocess.TimeoutExpired(command, timeout)
                    try:
                        process.wait(timeout=min(1.0, deadline - time.monotonic()))
                    except subprocess.TimeoutExpired:
                        pass
                rc = process.returncode
            else:
                rc = process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                rc = process.wait(timeout=90)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                rc = process.wait()
            if progress:
                print()
            if isinstance(exc, KeyboardInterrupt):
                raise
            tail = "\n".join(launch_log.read_text(errors="replace").splitlines()[-30:])
            raise RuntimeError(f"ORB-SLAM3 timeout: {launch_log}\n{tail}")
        if progress and rc == 0:
            elapsed = time.monotonic() - started
            print(f"\r{progress_line(stage, elapsed, expected_s, done=True)}")
        elif progress:
            print()
    if rc:
        tail = "\n".join(launch_log.read_text(errors="replace").splitlines()[-30:])
        raise RuntimeError(f"ORB-SLAM3 rc={rc}: {launch_log}\n{tail}")


def trajectory_ok(path: Path, expected_duration: float) -> bool:
    if not path.is_file():
        return False
    try:
        stamps = np.array([float(parts[0]) for line in path.read_text().splitlines()
                           if len(parts := line.split()) == 8])
    except ValueError:
        return False
    if len(stamps) < 30 or not np.isfinite(stamps).all():
        return False
    increasing = np.diff(stamps) > 1e-6
    return (np.count_nonzero(increasing) + 1 >= 0.99 * len(stamps)
            and stamps[-1] - stamps[0] >= 0.9 * expected_duration)


def rig_reference(path: Path, dataset: str, atlas: Path,
                  rejection_reasons: list[str] | None = None) -> dict:
    result = json.loads(path.read_text(encoding="utf-8"))
    X = np.asarray(result.get("X_slam_camera_to_sam_camera"), dtype=float)
    if (X.shape != (4, 4) or not np.isfinite(X).all()
            or not np.allclose(X[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(X[:3, :3].T @ X[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(X[:3, :3]), 1.0, atol=1e-5)):
        raise ValueError(f"invalid rig extrinsic reference: {path}")
    result.update({"dataset": dataset, "map": str(atlas),
                   "calibration_source": "rig reference",
                   "calibration_reference": str(path.resolve())})
    if rejection_reasons:
        result["calibration_attempt_rejection_reasons"] = rejection_reasons
    return result


def stable_gates_match(result: dict, stable_t: float, stable_r: float) -> bool:
    return (result.get("stable_translation_gate_m") == stable_t
            and result.get("stable_rotation_gate_deg") == stable_r)


def run_pair(converted: Path, args: argparse.Namespace, result_dir: Path) -> dict:
    name = converted.parent.name if converted.name == "converted" else converted.name
    info_path = converted / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"변환본이 없습니다: {info_path}")
    info = json.loads(info_path.read_text(encoding="utf-8"))
    pair_out = result_dir
    map_out, sam_out = pair_out / "map_slam", pair_out / "localize_sam"
    map_out.mkdir(parents=True, exist_ok=True)
    sam_out.mkdir(parents=True, exist_ok=True)
    if args.force:
        for stale in (pair_out / "camera_extrinsic.json",
                      pair_out / "camera_extrinsic.urdf",
                      pair_out / "camera_extrinsic_rejected.json"):
            stale.unlink(missing_ok=True)
    atlas = f"map_{name}"
    atlas_file = map_out / f"{atlas}.osa"
    authoritative = pair_out / "camera_extrinsic.json"
    if not authoritative.is_file():
        reference = rig_reference(args.rig_extrinsic, name, atlas_file)
        authoritative.write_text(json.dumps(reference, indent=2), encoding="utf-8")
        write_urdf(pair_out / "camera_extrinsic.urdf", reference)

    map_settings, map_config = map_out / "orb_settings.yaml", map_out / "orb_config.yaml"
    write_settings(info, "SLAM", map_settings, "System.SaveAtlasToFile", atlas,
                   args.baseline)
    write_config(info, "SLAM", map_config, converted / "SLAM", map_settings,
                 map_out, args.rate, args.start_delay, False, dense_map=True)
    map_traj = map_out / "CameraTrajectory.txt"
    if args.force or not (atlas_file.is_file() and
                          trajectory_ok(map_traj, info["SLAM"]["duration_s"])):
        stale_outputs = (atlas_file, map_traj, map_out / "KeyFrameTrajectory.txt",
                         map_out / "orbslam3_map_points.pcd",
                         map_out / "orbslam3_dense_map.pcd")
        if not args.force and any(path.exists() or path.is_symlink()
                                  for path in stale_outputs):
            raise RuntimeError(f"기존 불완전 Atlas를 보존했다: {map_out} (--force 로 재생성)")
        for stale in stale_outputs:
            stale.unlink(missing_ok=True)
        run_launch(map_config, map_out,
                   info["SLAM"]["duration_s"] / args.rate + args.timeout_margin,
                   "map", args.start_delay + info["SLAM"]["duration_s"] / args.rate)
    if not atlas_file.is_file() or not trajectory_ok(map_traj, info["SLAM"]["duration_s"]):
        raise RuntimeError(f"SLAM map/trajectory 생성 실패: {map_out}")
    shutil.copyfile(map_traj, map_out / "trajectory.txt")

    if authoritative.is_file() and not args.force and not args.independent_maps:
        existing = json.loads(authoritative.read_text(encoding="utf-8"))
        if (existing.get("dataset") == name
                and existing.get("calibration_source") == "stable transform cluster"
                and stable_gates_match(
                    existing, args.stable_translation, args.stable_rotation)
                and not quality_issues(
                existing, args.max_baseline, args.max_residual_translation,
                args.max_residual_rotation)):
            existing["map"] = str(atlas_file)
            authoritative.write_text(json.dumps(existing, indent=2), encoding="utf-8")
            write_urdf(pair_out / "camera_extrinsic.urdf", existing)
            log(f"검증된 외부 파라미터 재사용: {authoritative}")
            return existing

    result, issues = None, ["calibration not evaluated"]
    if args.independent_maps:
        try:
            result = run_independent_map(
                converted, args, info, name, pair_out, map_traj, atlas_file)
            issues = quality_issues(result, args.max_baseline,
                                    args.max_residual_translation,
                                    args.max_residual_rotation)
        except RuntimeError as exc:
            issues = [str(exc)]
    else:
        local_atlas = sam_out / atlas_file.name
        if not local_atlas.is_symlink() or local_atlas.resolve() != atlas_file.resolve():
            if (local_atlas.exists() or local_atlas.is_symlink()) and not args.force:
                raise RuntimeError(f"기존 local Atlas를 보존했다: {local_atlas} (--force 로 재생성)")
            local_atlas.unlink(missing_ok=True)
            local_atlas.symlink_to(atlas_file.resolve())
        sam_settings = sam_out / "orb_settings.yaml"
        sam_config = sam_out / "orb_config.yaml"
        write_settings(info, "SAM", sam_settings, "System.LoadAtlasFromFile", atlas,
                       args.baseline)
        write_config(info, "SAM", sam_config, converted / "SAM", sam_settings,
                     sam_out, args.rate, args.start_delay, True)
        sam_traj = sam_out / "CameraTrajectory.txt"
        if (not args.force and not trajectory_ok(sam_traj, info["SAM"]["duration_s"])
                and any(path.exists() for path in
                        (sam_traj, sam_out / "KeyFrameTrajectory.txt"))):
            raise RuntimeError(
                f"기존 불완전 localization을 보존했다: {sam_out} (--force 로 재생성)")
        if args.force or not trajectory_ok(sam_traj, info["SAM"]["duration_s"]):
            sam_traj.unlink(missing_ok=True)
            (sam_out / "KeyFrameTrajectory.txt").unlink(missing_ok=True)
            run_launch(sam_config, sam_out,
                       info["SAM"]["duration_s"] / args.rate + args.timeout_margin,
                       "localize", args.start_delay + info["SAM"]["duration_s"] / args.rate)
        if not trajectory_ok(sam_traj, info["SAM"]["duration_s"]):
            issues = ["SAM localization trajectory missing"]
        else:
            try:
                result = estimate_from_files(
                    map_traj, sam_traj, args.max_gap,
                    args.stable_translation, args.stable_rotation)
                result.update({"dataset": name, "map": str(atlas_file),
                               "slam_trajectory": str(map_traj),
                               "sam_trajectory": str(sam_traj),
                               "calibration_source": "stable transform cluster"})
                issues = quality_issues(result, args.max_baseline,
                                        args.max_residual_translation,
                                        args.max_residual_rotation)
            except RuntimeError as exc:
                issues = [str(exc)]
    if issues:
        log(f"SAM localization 품질 실패: {'; '.join(issues)}")
        result = rig_reference(args.rig_extrinsic, name, atlas_file, issues)
        log(f"rig 기준 외부 파라미터 사용: {args.rig_extrinsic}")
    if not args.independent_maps:
        shutil.copyfile(sam_traj, sam_out / "trajectory.txt")
    (pair_out / "camera_extrinsic_rejected.json").unlink(missing_ok=True)
    authoritative.write_text(
        json.dumps(result, indent=2), encoding="utf-8")
    write_urdf(pair_out / "camera_extrinsic.urdf", result)
    return result


def run_independent_map(converted: Path, args: argparse.Namespace, info: dict,
                        name: str, pair_out: Path, slam_traj: Path,
                        slam_atlas: Path) -> dict:
    sam_out = pair_out / "map_sam"
    sam_out.mkdir(parents=True, exist_ok=True)
    sam_atlas_name = f"map_{name}_sam"
    sam_atlas = sam_out / f"{sam_atlas_name}.osa"
    settings, config = sam_out / "orb_settings.yaml", sam_out / "orb_config.yaml"
    write_settings(info, "SAM", settings, "System.SaveAtlasToFile", sam_atlas_name,
                   args.baseline)
    write_config(info, "SAM", config, converted / "SAM", settings,
                 sam_out, args.rate, args.start_delay, False)
    camera_traj = sam_out / "CameraTrajectory.txt"
    keyframe_traj = sam_out / "KeyFrameTrajectory.txt"
    stale_outputs = (sam_atlas, camera_traj, keyframe_traj,
                     sam_out / "orbslam3_map_points.pcd")
    if args.force or not sam_atlas.is_file():
        if not args.force and any(path.exists() for path in stale_outputs):
            raise RuntimeError(f"기존 불완전 SAM Atlas를 보존했다: {sam_out} (--force 로 재생성)")
        for stale in stale_outputs:
            stale.unlink(missing_ok=True)
        run_launch(config, sam_out,
                   info["SAM"]["duration_s"] / args.rate + args.timeout_margin,
                   "sam map", args.start_delay + info["SAM"]["duration_s"] / args.rate)
    if not sam_atlas.is_file():
        raise RuntimeError(f"SAM map 생성 실패: {sam_out}")

    sam_traj = (camera_traj if trajectory_ok(camera_traj, info["SAM"]["duration_s"])
                else keyframe_traj)
    if not trajectory_ok(sam_traj, info["SAM"]["duration_s"]):
        local_out = pair_out / "localize_sam_map"
        local_out.mkdir(parents=True, exist_ok=True)
        local_atlas = local_out / sam_atlas.name
        if not local_atlas.is_symlink() or local_atlas.resolve() != sam_atlas.resolve():
            if local_atlas.exists() or local_atlas.is_symlink():
                raise RuntimeError(f"기존 local SAM Atlas를 보존했다: {local_atlas}")
            local_atlas.symlink_to(sam_atlas.resolve())
        local_settings = local_out / "orb_settings.yaml"
        local_config = local_out / "orb_config.yaml"
        write_settings(info, "SAM", local_settings, "System.LoadAtlasFromFile",
                       sam_atlas_name, args.baseline)
        write_config(info, "SAM", local_config, converted / "SAM", local_settings,
                     local_out, args.rate, args.start_delay, True)
        sam_traj = local_out / "CameraTrajectory.txt"
        local_keyframes = local_out / "KeyFrameTrajectory.txt"
        if (not args.force and not trajectory_ok(sam_traj, info["SAM"]["duration_s"])
                and (sam_traj.exists() or local_keyframes.exists())):
            raise RuntimeError(
                f"기존 불완전 SAM map localization을 보존했다: {local_out} (--force 로 재생성)")
        if args.force or not trajectory_ok(sam_traj, info["SAM"]["duration_s"]):
            sam_traj.unlink(missing_ok=True)
            local_keyframes.unlink(missing_ok=True)
            run_launch(local_config, local_out,
                       info["SAM"]["duration_s"] / args.rate + args.timeout_margin,
                       "sam map localize",
                       args.start_delay + info["SAM"]["duration_s"] / args.rate)
    if not trajectory_ok(sam_traj, info["SAM"]["duration_s"]):
        raise RuntimeError(f"SAM map localization trajectory 생성 실패: {sam_traj.parent}")
    shutil.copyfile(sam_traj, sam_out / "trajectory.txt")
    result = estimate_independent_files(
        slam_traj, sam_traj, args.max_gap, args.independent_y)
    result.update({"dataset": name, "map": str(slam_atlas),
                   "sam_map": str(sam_atlas),
                   "slam_trajectory": str(slam_traj),
                   "sam_trajectory": str(sam_traj),
                   "calibration_source": "independent maps AX=ZB"})
    return result


def estimate_from_arrays(ts_a: np.ndarray, A: np.ndarray, ts_b: np.ndarray,
                         B: np.ndarray, max_gap: float,
                         stable_t: float = 0.10,
                         stable_r: float = 5.0) -> dict:
    stamps, transforms = [], []
    for stamp, pose_b in zip(ts_b, B):
        pose_a = slerp_interp(ts_a, A, stamp, max_gap=max_gap)
        if pose_a is not None:
            stamps.append(stamp)
            transforms.append(np.linalg.inv(pose_a) @ pose_b)
    if len(transforms) < 30:
        raise RuntimeError(f"동기화된 유효 pose가 너무 적습니다: {len(transforms)}")
    stamps, transforms = np.asarray(stamps), np.asarray(transforms)
    rotations = transforms[:, :3, :3]
    bin_index = np.minimum(
        19, np.searchsorted(np.linspace(stamps[0], stamps[-1] + 1e-9, 21),
                            stamps, side="right") - 1)
    keep, best = None, None
    for seed in np.linspace(0, len(transforms) - 1,
                            min(256, len(transforms)), dtype=int):
        dt = np.linalg.norm(transforms[:, :3, 3] - transforms[seed, :3, 3], axis=1)
        relative = np.einsum("ji,njk->nik", rotations[seed], rotations)
        cosine = np.clip((np.trace(relative, axis1=1, axis2=2) - 1) / 2, -1, 1)
        dr = np.degrees(np.arccos(cosine))
        candidate = (dt <= stable_t) & (dr <= stable_r)
        score = (np.count_nonzero(np.bincount(bin_index[candidate], minlength=20) >= 3),
                 np.count_nonzero(candidate))
        if best is None or score > best:
            keep, best = candidate, score
    for _ in range(8):
        current = transforms[keep]
        center_t = np.median(current[:, :3, 3], axis=0)
        center_R = proj_SO3(current[:, :3, :3].mean(axis=0))
        dt = np.linalg.norm(transforms[:, :3, 3] - center_t, axis=1)
        dr = np.array([rot_angle_deg(center_R.T @ T[:3, :3]) for T in transforms])
        new_keep = (dt <= stable_t) & (dr <= stable_r)
        if new_keep.sum() < 30 or np.array_equal(new_keep, keep):
            break
        keep = new_keep
    stable_bins = int(np.count_nonzero(
        np.bincount(bin_index[keep], minlength=20) >= 3))
    if keep.sum() < max(30, 0.2 * len(transforms)) or stable_bins < 8:
        raise RuntimeError(
            f"시간 전체에 수렴한 R,t가 부족합니다: {keep.sum()}/{len(transforms)} pose, "
            f"stable bins {stable_bins}/20")
    current = transforms[keep]
    X = np.eye(4)
    X[:3, :3] = proj_SO3(current[:, :3, :3].mean(axis=0))
    X[:3, 3] = np.median(current[:, :3, 3], axis=0)
    dt = np.linalg.norm(current[:, :3, 3] - X[:3, 3], axis=1)
    dr = np.array([rot_angle_deg(X[:3, :3].T @ T[:3, :3]) for T in current])
    return {
        "pairs_total": len(transforms), "pairs_used": int(keep.sum()),
        "stable_translation_gate_m": stable_t,
        "stable_rotation_gate_deg": stable_r,
        "stable_time_bins": stable_bins,
        "stable_time_bins_total": 20,
        "stable_duration_s": float(stamps[keep].max() - stamps[keep].min()),
        "X_slam_camera_to_sam_camera": X.tolist(),
        "translation_m": X[:3, 3].tolist(),
        "translation_norm_m": float(np.linalg.norm(X[:3, 3])),
        "residual_translation_median_m": float(np.median(dt)),
        "residual_translation_p90_m": float(np.percentile(dt, 90)),
        "residual_rotation_median_deg": float(np.median(dr)),
        "residual_rotation_p90_deg": float(np.percentile(dr, 90)),
    }


def estimate_from_files(a: Path, b: Path, max_gap: float,
                        stable_t: float = 0.10,
                        stable_r: float = 5.0) -> dict:
    ts_a, A = read_tum(a)
    ts_b, B = read_tum(b)
    return estimate_from_arrays(ts_a, A, ts_b, B, max_gap, stable_t, stable_r)


def estimate_independent_arrays(ts_a: np.ndarray, A: np.ndarray,
                                ts_b: np.ndarray, B: np.ndarray,
                                max_gap: float, fixed_y: float | None = None) -> dict:
    best = fit(ts_a, A, ts_b, B, tau=0.0, stride=2, max_gap=max_gap)
    if best is None:
        raise RuntimeError("독립 맵 궤적 AX=ZB 추정에 실패했습니다")
    if best["cond"] < 0.01 and fixed_y is None:
        raise RuntimeError(
            f"평면 주행으로 optical-frame Y 병진을 관측할 수 없습니다 "
            f"(observability {best['cond']:.6f}); --independent-y 실측값이 필요합니다")
    paired_a, paired_b, stamps = pair_poses(
        ts_a, A, ts_b, B, tau=0.0, stride=2, max_gap=max_gap)
    X, M = best["X"].copy(), best["M"].copy()
    if fixed_y is not None:
        X[1, 3] = fixed_y
        lhs = (paired_a[:, :3, 3]
               + np.einsum("nij,j->ni", paired_a[:, :3, :3], X[:3, 3]))
        rhs_without_t = np.einsum(
            "ij,nj->ni", M[:3, :3], paired_b[:, :3, 3])
        M[:3, 3] = np.median(lhs - rhs_without_t, axis=0)
    _, _, _, dt, dr = residuals(paired_a, paired_b, X, M)
    bin_index = np.minimum(
        19, np.searchsorted(np.linspace(stamps[0], stamps[-1] + 1e-9, 21),
                            stamps, side="right") - 1)
    stable_bins = int(np.count_nonzero(
        np.bincount(bin_index, minlength=20) >= 3))
    if stable_bins < 8:
        raise RuntimeError(f"동기 궤적이 시간 전체에 부족합니다: {stable_bins}/20 bins")
    return {
        "pairs_total": len(paired_a), "pairs_used": len(paired_a),
        "stable_time_bins": stable_bins, "stable_time_bins_total": 20,
        "stable_duration_s": float(stamps[-1] - stamps[0]),
        "time_offset_s": 0.0,
        "observability_ratio": float(best["cond"]),
        "fixed_translation_y_m": fixed_y,
        "X_slam_camera_to_sam_camera": X.tolist(),
        "M_slam_map_from_sam_map": M.tolist(),
        "translation_m": X[:3, 3].tolist(),
        "translation_norm_m": float(np.linalg.norm(X[:3, 3])),
        "residual_translation_median_m": float(np.median(dt)),
        "residual_translation_p90_m": float(np.percentile(dt, 90)),
        "residual_rotation_median_deg": float(np.median(dr)),
        "residual_rotation_p90_deg": float(np.percentile(dr, 90)),
    }


def estimate_independent_files(a: Path, b: Path, max_gap: float,
                               fixed_y: float | None = None) -> dict:
    ts_a, A = read_tum(a)
    ts_b, B = read_tum(b)
    return estimate_independent_arrays(ts_a, A, ts_b, B, max_gap, fixed_y)


def quality_issues(result: dict, max_baseline: float, max_t: float,
                   max_r: float) -> list[str]:
    checks = (("baseline", result["translation_norm_m"], max_baseline, "m"),
              ("translation residual p90", result["residual_translation_p90_m"],
               max_t, "m"),
              ("rotation residual p90", result["residual_rotation_p90_deg"],
               max_r, "deg"))
    return [f"{name} {value:.3f}{unit} > {limit:.3f}{unit}"
            for name, value, limit, unit in checks
            if not math.isfinite(value) or value > limit]


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    pitch = math.asin(float(np.clip(-R[2, 0], -1, 1)))
    if abs(math.cos(pitch)) > 1e-8:
        roll, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return roll, pitch, yaw


def rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def write_urdf(path: Path, result: dict) -> None:
    X = np.asarray(result["X_slam_camera_to_sam_camera"])
    xyz, rpy = X[:3, 3], matrix_to_rpy(X[:3, :3])
    confidence = "high" if (result["residual_translation_p90_m"] <= 0.05 and
                            result["residual_rotation_p90_deg"] <= 3) else "low"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"""<?xml version="1.0"?>
<!-- Estimated from synchronized ORB-SLAM3 trajectories.
     dataset: {result['dataset']}
     source: {result['calibration_source']}
     confidence: {confidence}
     residual p90: {result['residual_translation_p90_m']:.6f} m, {result['residual_rotation_p90_deg']:.6f} deg
     ponytail: planar motion weakly observes vertical offset; measure it directly if needed. -->
<robot name="camera_extrinsic">
  <link name="base_link"/>
  <link name="slam_camera_color_optical_frame"/>
  <link name="sam_camera_color_optical_frame"/>
  <joint name="base_to_slam_cam" type="fixed">
    <parent link="base_link"/>
    <child link="slam_camera_color_optical_frame"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="slam_cam_to_sam_cam" type="fixed">
    <parent link="slam_camera_color_optical_frame"/>
    <child link="sam_camera_color_optical_frame"/>
    <origin xyz="{' '.join(f'{v:.9f}' for v in xyz)}"
            rpy="{' '.join(f'{v:.9f}' for v in rpy)}"/>
  </joint>
</robot>
""", encoding="utf-8")


def self_test() -> None:
    ts_a = np.linspace(0, 5, 101)
    A = []
    X = np.eye(4)
    X[:3, :3] = rpy_to_matrix(0.2, -0.1, 1.1)
    X[:3, 3] = [0.25, -0.04, 0.08]
    for t in ts_a:
        T = np.eye(4)
        T[:3, :3] = rpy_to_matrix(0.02 * math.sin(t), 0.03 * math.cos(t), 0.25 * t)
        T[:3, 3] = [math.sin(t), 0.1 * math.sin(2*t), math.cos(t)]
        A.append(T)
    A = np.asarray(A)
    result = estimate_from_arrays(ts_a, A, ts_a[1:-1], (A @ X)[1:-1], 0.1)
    got = np.asarray(result["X_slam_camera_to_sam_camera"])
    assert np.linalg.norm(got[:3, 3] - X[:3, 3]) < 1e-8
    assert rot_angle_deg(got[:3, :3].T @ X[:3, :3]) < 1e-5
    assert np.max(np.abs(rpy_to_matrix(*matrix_to_rpy(X[:3, :3])) - X[:3, :3])) < 1e-8
    assert quality_issues(result, 1.0, 0.1, 5.0) == []
    corrupted = (A @ X)[1:-1].copy()
    corrupted[30:60, :3, 3] += [0.4, -0.2, 0.1]
    robust = estimate_from_arrays(ts_a, A, ts_a[1:-1], corrupted, 0.1)
    robust_X = np.asarray(robust["X_slam_camera_to_sam_camera"])
    assert robust["pairs_used"] < robust["pairs_total"]
    assert np.linalg.norm(robust_X[:3, 3] - X[:3, 3]) < 1e-8
    bins = np.minimum(19, np.arange(len(A) - 2) * 20 // (len(A) - 2))
    spread = []
    for pose, bin_index in zip(A[1:-1], bins):
        noisy_X = X.copy()
        noisy_X[0, 3] += -0.08 if bin_index < 7 else 0.08 if bin_index >= 13 else 0.0
        spread.append(pose @ noisy_X)
    try:
        estimate_from_arrays(ts_a, A, ts_a[1:-1], np.asarray(spread), 0.1,
                             stable_t=0.05, stable_r=3.0)
    except RuntimeError as exc:
        assert "시간 전체에 수렴한 R,t가 부족합니다" in str(exc)
    else:
        raise AssertionError("overly strict stable-transform gate accepted spread data")
    relaxed = estimate_from_arrays(ts_a, A, ts_a[1:-1], np.asarray(spread), 0.1)
    relaxed_X = np.asarray(relaxed["X_slam_camera_to_sam_camera"])
    assert relaxed["stable_time_bins"] >= 8
    assert np.linalg.norm(relaxed_X[:3, 3] - X[:3, 3]) < 1e-8
    assert rot_angle_deg(relaxed_X[:3, :3].T @ X[:3, :3]) < 1e-5
    assert quality_issues(relaxed, 1.0, 0.1, 5.0) == []
    rotation_spread = []
    for pose, bin_index in zip(A[1:-1], bins):
        noisy_X = X.copy()
        angle = math.radians(-4 if bin_index < 7 else 4 if bin_index >= 13 else 0)
        noisy_X[:3, :3] = X[:3, :3] @ rpy_to_matrix(angle, 0, 0)
        rotation_spread.append(pose @ noisy_X)
    try:
        estimate_from_arrays(ts_a, A, ts_a[1:-1], np.asarray(rotation_spread), 0.1,
                             stable_t=0.10, stable_r=3.0)
    except RuntimeError as exc:
        assert "시간 전체에 수렴한 R,t가 부족합니다" in str(exc)
    else:
        raise AssertionError("overly strict rotation gate accepted spread data")
    rotation_relaxed = estimate_from_arrays(
        ts_a, A, ts_a[1:-1], np.asarray(rotation_spread), 0.1)
    rotation_X = np.asarray(rotation_relaxed["X_slam_camera_to_sam_camera"])
    assert rotation_relaxed["stable_time_bins"] >= 8
    assert rot_angle_deg(rotation_X[:3, :3].T @ X[:3, :3]) < 0.1
    assert len(quality_issues({**result, "translation_norm_m": 10.0},
                              1.0, 0.1, 5.0)) == 1
    M = np.eye(4)
    M[:3, :3] = rpy_to_matrix(-0.3, 0.15, 0.7)
    M[:3, 3] = [1.2, -0.4, 0.8]
    excited = []
    for t in ts_a:
        T = np.eye(4)
        T[:3, :3] = rpy_to_matrix(0.3 * math.sin(t), 0.25 * math.cos(0.7*t),
                                  0.4 * t)
        T[:3, 3] = [math.sin(t), math.cos(0.6*t), 0.2 * math.sin(1.3*t)]
        excited.append(T)
    excited = np.asarray(excited)
    independent = estimate_independent_arrays(
        ts_a, excited, ts_a, np.asarray([np.linalg.inv(M) @ T @ X for T in excited]),
        0.1)
    independent_X = np.asarray(independent["X_slam_camera_to_sam_camera"])
    assert np.linalg.norm(independent_X[:3, 3] - X[:3, 3]) < 1e-5
    assert rot_angle_deg(independent_X[:3, :3].T @ X[:3, :3]) < 1e-4
    planar = excited.copy()
    planar[:, :3, :3] = np.asarray([
        rpy_to_matrix(0.0, 0.0, 0.4 * t) for t in ts_a])
    planar_b = np.asarray([np.linalg.inv(M) @ T @ X for T in planar])
    pinned = estimate_independent_arrays(ts_a, planar, ts_a, planar_b, 0.1, X[1, 3])
    assert abs(pinned["translation_m"][1] - X[1, 3]) < 1e-12
    assert ros2_argv(["--inputs:=a", "--outputs:=b"]) == \
        ["--inputs", "a", "--outputs", "b"]
    try:
        path_arg("")
    except argparse.ArgumentTypeError:
        pass
    else:
        raise AssertionError("empty CLI path accepted")
    assert default_output(Path("/repo/output/sample/converted")) == \
        Path("/repo/output/sample/camera_extrinsic")
    assert "map" in progress_line("map", 5, 10) and "50%" in progress_line("map", 5, 10)
    assert "마무리 중" in progress_line("localize", 11, 10)
    assert "100%" in progress_line("map", 11, 10, done=True)
    assert run_launch.__defaults__ == (None, None)
    reference = rig_reference(RIG_REFERENCE, "test", Path("/tmp/map.osa"))
    assert reference["dataset"] == "test"
    assert reference["calibration_source"] == "rig reference"
    assert stable_gates_match(relaxed, 0.10, 5.0)
    assert not stable_gates_match(relaxed, 0.05, 3.0)
    with tempfile.TemporaryDirectory() as directory:
        trajectory = Path(directory) / "trajectory.txt"
        trajectory.write_text("\n".join(
            f"{i / 10} 0 0 0 0 0 0 1" for i in range(31)))
        assert trajectory_ok(trajectory, 3.0)
        trajectory.write_text("\n".join(
            "0 0 0 0 0 0 0 1" for _ in range(31)))
        assert not trajectory_ok(trajectory, 3.0)


def ensure_build() -> None:
    required = [ORB_INSTALL / "setup.bash",
                ORB_WS / "src/ORB_SLAM3/Vocabulary/ORBvoc.txt"]
    if not all(path.is_file() for path in required):
        raise RuntimeError("ORB-SLAM3 Jazzy 빌드가 없습니다: orbslam_ws/install_jazzy")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--inputs", type=path_arg,
                        help="directory containing info.json, SLAM/, and SAM/")
    source.add_argument("--self-test", action="store_true")
    parser.add_argument("--outputs", type=path_arg,
                        help="result directory (default: output/<dataset>/camera_extrinsic)")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-delay", type=float, default=12.0)
    parser.add_argument("--baseline", type=float, default=0.05,
                        help="D435i stereo baseline [m]")
    parser.add_argument("--max-gap", type=float, default=0.15,
                        help="maximum SLAM trajectory interpolation gap [s]")
    parser.add_argument("--max-baseline", type=float, default=1.0,
                        help="reject larger estimated camera separation [m]")
    parser.add_argument("--max-residual-translation", type=float, default=0.10,
                        help="reject larger calibration translation p90 [m]")
    parser.add_argument("--max-residual-rotation", type=float, default=5.0,
                        help="reject larger calibration rotation p90 [deg]")
    parser.add_argument("--stable-translation", type=float, default=0.10,
                        help="maximum translation spread in the stable R,t cluster [m]")
    parser.add_argument("--stable-rotation", type=float, default=5.0,
                        help="maximum rotation spread in the stable R,t cluster [deg]")
    parser.add_argument("--independent-maps", action="store_true",
                        help="build one map per camera and solve AX=ZB")
    parser.add_argument("--independent-y", type=float,
                        help="fixed SLAM-optical-frame Y translation [m] for planar motion")
    parser.add_argument("--rig-extrinsic", type=path_arg, default=RIG_REFERENCE,
                        help="fixed rig calibration used when trajectory calibration fails")
    parser.add_argument("--timeout-margin", type=float, default=600.0)
    parser.add_argument("--ros-domain-id", type=int,
                        help="기본값: 실행마다 자동 선택")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(ros2_argv(sys.argv[1:]))
    self_test()
    if args.self_test:
        print("self-test: PASS")
        return 0
    thresholds = (args.rate, args.baseline, args.max_gap, args.max_baseline,
                  args.max_residual_translation, args.max_residual_rotation,
                  args.stable_translation, args.stable_rotation)
    if not all(math.isfinite(value) and value > 0 for value in thresholds):
        parser.error("numeric thresholds must be positive")
    if (not math.isfinite(args.start_delay) or args.start_delay < 0
            or not math.isfinite(args.timeout_margin) or args.timeout_margin < 0):
        parser.error("start delay and timeout margin must be finite and non-negative")
    if args.independent_y is not None and not math.isfinite(args.independent_y):
        parser.error("--independent-y must be finite")
    if args.independent_y is not None and not args.independent_maps:
        parser.error("--independent-y requires --independent-maps")
    try:
        args.ros_domain_id = runtime_domain(args.ros_domain_id)
    except ValueError as exc:
        parser.error(str(exc))
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    print(f"[ROS] Jazzy domain {args.ros_domain_id} (localhost)", flush=True)
    inputs = args.inputs.expanduser().resolve()
    missing = ([] if (inputs / "info.json").is_file() else ["info.json"])
    missing += [name for name in ("SLAM", "SAM") if not (inputs / name).is_dir()]
    if missing:
        parser.error(f"--inputs 아래에 필요 항목이 없다: {', '.join(missing)}")
    ensure_build()
    summary_dir = (args.outputs.expanduser().resolve() if args.outputs is not None
                   else default_output(inputs))
    if inputs == summary_dir or inputs in summary_dir.parents or summary_dir in inputs.parents:
        parser.error(f"입력과 출력 경로가 겹친다: {inputs} / {summary_dir}")
    summary = {"datasets": [run_pair(inputs, args, summary_dir)]}
    summary_dir.mkdir(parents=True, exist_ok=True)
    (summary_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    log(f"완료: 1개 URDF ({summary_dir})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
