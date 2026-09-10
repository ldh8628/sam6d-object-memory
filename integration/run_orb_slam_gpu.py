#!/usr/bin/env python3
"""Run/compare ORB-SLAM3 CPU versus hybrid CUDA descriptors, using existing RGBD input.

This is NOT all-GPU SLAM. Detection, orientation, IMU and optimization stay on CPU.
CUDA is explicit and fails on missing/failed backend; no implicit CPU fallback.
Run inside the realsense conda environment. --bag does not acquire a camera.
"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import statistics
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / 'orbslam_ws/src/ORB_SLAM3'
LIBRARY = CORE / 'lib/liborb_cuda_descriptors.so'
SETUP = ROOT / 'orbslam_ws/install_jazzy/setup.bash'
EXECUTABLE = ROOT / 'orbslam_ws/install_jazzy/orbslam3_ros2/lib/orbslam3_ros2/rgbd_node'


def ros_command(*command):
    return ['bash', '-c', 'source "$1" && shift && exec "$@"', 'orb-gpu', str(SETUP), *map(str, command)]


def validate_cuda(library):
    plugin = ctypes.CDLL(str(library))
    plugin.orb_cuda_probe.restype = ctypes.c_char_p
    error = plugin.orb_cuda_probe()
    if error:
        raise RuntimeError(error.decode())
    return plugin  # Retain dlopen handle while probing.


def build_cuda(args):
    compiler = args.nvcc or shutil.which('nvcc')
    if not compiler:
        compiler = '/usr/local/cuda-12.8/bin/nvcc'
    args.cuda_library.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=args.cuda_library.parent) as temp:
        candidate = Path(temp) / args.cuda_library.name
        command = [str(compiler), '-std=c++14', '-O3', '--fmad=false', f'-arch={args.cuda_arch}',
                   '-ccbin', '/usr/bin/g++', '-shared', '-Xcompiler=-fPIC',
                   str(CORE / 'src/OrbCudaDescriptors.cu'), '-o', str(candidate)]
        subprocess.run(command, check=True)
        validate_cuda(candidate)
        candidate.replace(args.cuda_library)
    print(f'CUDA descriptor plugin ready: {args.cuda_library}', flush=True)


def stop(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        process.wait()
        return
    try:
        process.wait(timeout=60)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        raise RuntimeError('ORB process did not shut down cleanly')


def stats(values):
    values = sorted(values)
    if not values:
        return None
    return dict(count=len(values), mean=statistics.mean(values), median=statistics.median(values),
                p95=values[min(len(values)-1, math.ceil(len(values)*.95)-1)], max=max(values))


def ensure_no_orb(env):
    nodes = subprocess.run(ros_command('ros2', 'node', 'list', '--no-daemon'), env=env,
                           capture_output=True, text=True, timeout=20, check=True)
    if any(line.strip().split('/')[-1] == 'orbslam3_rgbd_node' for line in nodes.stdout.splitlines()):
        raise RuntimeError('An ORB node already runs in this ROS domain; stop it before this experiment')


ATLAS_SETTING = r'^System\.LoadAtlasFromFile:[ \t]*([^\n]*)$'


def resolve_atlas(source, text):
    match = re.search(ATLAS_SETTING, text, re.M)
    if not match:
        return None
    parts = shlex.split(match.group(1), comments=True)
    if len(parts) != 1 or not parts[0]:
        raise ValueError('System.LoadAtlasFromFile must name one nonempty Atlas path')
    path = Path(parts[0])
    if path.suffix != '.osa':
        path = Path(str(path) + '.osa')
    if not path.is_absolute():
        path = source.parent / path
    path = path.resolve(strict=True)
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError(f'Atlas must be a nonempty file: {path}')
    return path


def stage_settings(source, output, localization=False):
    text = source.read_text()
    atlas = resolve_atlas(source, text)
    if localization and atlas is None:
        raise ValueError('--localization requires System.LoadAtlasFromFile')
    if atlas is not None:
        # Core unconditionally opens ./<setting>.osa, so keep a basename in its cwd.
        (output / 'input_atlas.osa').symlink_to(atlas)
        text = re.sub(ATLAS_SETTING, 'System.LoadAtlasFromFile: "input_atlas"', text, flags=re.M)
        (output / 'atlas_source.json').write_text(json.dumps(dict(source_settings=str(source), atlas=str(atlas)), indent=2))
    text = re.sub(r'^System\.SaveAtlasToFile:.*$', '', text, flags=re.M)
    (output / 'settings.yaml').write_text(text)


def live_settings(source, camera, destination, localization=False):
    text = source.read_text()
    if (camera['width'], camera['height']) != (640, 480):
        raise RuntimeError('Connected camera does not match requested 640x480 RGB profile')
    distortion = [*camera['D'], 0., 0., 0., 0., 0.]
    values = {'Camera.width': camera['width'], 'Camera.height': camera['height'],
              'Camera1.fx': camera['K'][0], 'Camera1.fy': camera['K'][4],
              'Camera1.cx': camera['K'][2], 'Camera1.cy': camera['K'][5],
              **dict(zip(('Camera1.k1', 'Camera1.k2', 'Camera1.p1', 'Camera1.p2', 'Camera1.k3'), distortion))}
    loading = localization or bool(re.search(r'^System\.LoadAtlasFromFile:', text, re.M))
    deltas = {}
    for key, value in values.items():
        pattern = r'^' + re.escape(key) + r':\s*([-+\d.eE]+)'
        match = re.search(pattern, text, re.M)
        if not match:
            raise RuntimeError(f'Missing numeric calibration setting: {key}')
        original = float(match.group(1))
        deltas[key] = dict(source=original, actual=value, delta=value-original)
        if loading and not math.isclose(original, value, abs_tol=1e-4):
            raise RuntimeError(f'Loaded Atlas camera calibration differs: {key}, settings={original}, camera={value}')
        text = re.sub(pattern, f'{key}: {value:.12g}', text, flags=re.M)
    if loading:
        atlas = resolve_atlas(source, text)
        if atlas is not None:
            text = re.sub(ATLAS_SETTING, lambda _: 'System.LoadAtlasFromFile: ' + json.dumps(str(atlas)), text, flags=re.M)
    destination.write_text(text)
    return deltas


def start_camera(args):
    from camera_publish import camera_arguments
    from two_host_worker import camera_identity
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    ensure_no_orb(os.environ)
    command = ros_command(*camera_arguments(args.serial, 'slam_camera', args.sync_mode))
    (args.output / 'camera_command.json').write_text(json.dumps(command, indent=2))
    with (args.output / 'camera.log').open('w') as log:
        camera = subprocess.Popen(command, env=os.environ, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
    try:
        identity = camera_identity('slam_camera', args.serial, args.sync_mode, 45.)
        (args.output / 'camera_identity.json').write_text(json.dumps(identity, indent=2))
        generated = args.output / 'live_settings.yaml'
        deltas = live_settings(args.settings, identity['camera'], generated, args.localization)
        (args.output / 'calibration_delta.json').write_text(json.dumps(deltas, indent=2))
        args.settings = generated
        return camera
    except BaseException:
        stop(camera)
        raise


def summarize(output, backend):
    rows = [json.loads(line) for line in (output / 'orb_input_health.jsonl').read_text().splitlines()]
    frames = [row for row in rows if row.get('kind') == 'rgbd']
    successful = [row for row in frames if row.get('consumed')]
    measured = successful[30:]  # Same first 30 input frames excluded from CPU/CUDA timing.
    errors = [row for row in rows if row.get('event')]
    trajectory = output / 'CameraTrajectory.txt'
    poses = len([line for line in trajectory.read_text().splitlines() if line and not line.startswith('#')]) if trajectory.exists() else 0
    text = (output / 'node.log').read_text()
    attested = backend == 'cpu' or 'ORB_DESCRIPTOR_BACKEND cuda:' in text
    input_ok = len(measured) >= 10 and not errors and attested
    coverage = poses / len(frames) if frames else 0
    timing = re.search(r'TIMING: frames_tracked=(\d+) track_ms mean=([\d.]+) median=([\d.]+) p95=([\d.]+)', text)
    return dict(backend=backend, gpu_stage='ORB descriptor sampling' if backend == 'cuda' else None,
                cpu_stages='detection, orientation, blur, matching, tracking optimization, mapping, IMU',
                status=('PASS' if coverage >= .9 else 'NOT_TRACKING') if input_ok else 'FAIL',
                cuda_execution_attested=attested if backend == 'cuda' else None,
                received=len(frames), consumed=len(successful), trajectory_poses=poses,
                trajectory_coverage=coverage, minimum_trajectory_coverage=.9,
                input_stamp_sha256=hashlib.sha256(json.dumps([
                    (row['stamp_ns'], row['depth_stamp_ns']) for row in frames]).encode()).hexdigest(),
                errors=errors, consume_ms=stats([row['consume_ms'] for row in measured]),
                queue_ms=stats([row['queue_ms'] for row in measured]),
                track_ms_including_warmup=dict(zip(('frames', 'mean', 'median', 'p95'), map(float, timing.groups()))) if timing else None,
                accuracy_note='No ground truth: trajectory count is tracking coverage, not pose accuracy.')


def run(args, backend, output):
    output.mkdir(parents=True, exist_ok=False)
    stage_settings(args.settings, output, args.localization)
    env = os.environ.copy()
    env.update(ORB_SLAM3_DESCRIPTOR_BACKEND=backend, ORB_SLAM3_CUDA_LIBRARY=str(args.cuda_library),
               ROS_DOMAIN_ID=str(args.domain_id), RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    ensure_no_orb(env)
    params = dict(vocabulary_path=CORE / 'Vocabulary/ORBvoc.txt', settings_path=output / 'settings.yaml',
                  rgbd_topic=args.rgbd_topic, input_health_path=output / 'orb_input_health.jsonl',
                  input_require_publishers='false', enable_viewer='false', publish_map_points='false',
                  dense_map_enabled='false', use_imu=str(args.use_imu).lower(), imu_topic=args.imu_topic,
                  localization_mode=str(args.localization).lower())
    input_topics = [args.rgbd_topic]
    if args.rgb_topic:
        params.pop('rgbd_topic')
        params.update(rgb_topic=args.rgb_topic, depth_topic=args.depth_topic)
        input_topics = [args.rgb_topic, args.depth_topic]
    command = ros_command(EXECUTABLE, '--ros-args')
    for key, value in params.items():
        command += ['-p', f'{key}:={value}']
    node = player = None
    try:
        with (output / 'node.log').open('w') as node_log, (output / 'play.log').open('w') as play_log:
            node = subprocess.Popen(command, cwd=output, env=env, stdout=node_log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            subprocess.run(ros_command('ros2', 'topic', 'echo', '--once', '--qos-durability', 'transient_local',
                           '/orbslam3/ready', 'std_msgs/msg/String'), env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=90, check=True)
            if node.poll() is not None:
                raise RuntimeError(f'ORB exited before input: {node.returncode}')
            if args.bag:
                command = ros_command('ros2', 'bag', 'play', args.bag, '--disable-keyboard-controls',
                        '--rate', args.rate, '--start-offset', args.start_offset,
                        '--playback-duration', args.seconds, '--topics', *input_topics)
                if args.use_imu:
                    command.append(args.imu_topic)
                player = subprocess.Popen(command, cwd=output, env=env, stdout=play_log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + (args.seconds / args.rate if player else args.seconds) + 45
            while (player.poll() is None if player else time.monotonic() < deadline - 45):
                if node.poll() is not None:
                    raise RuntimeError(f'ORB exited during input: {node.returncode}')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Bag playback did not finish within its bounded window')
                time.sleep(.2)
            if player and player.returncode:
                raise RuntimeError(f'Bag playback failed: {player.returncode}')
            drain = subprocess.run(ros_command('ros2', 'service', 'call', '/orbslam3/drain_input',
                        'std_srvs/srv/Trigger', '{}'), env=env, capture_output=True, text=True, timeout=90, check=True)
            (output / 'drain.log').write_text(drain.stdout + drain.stderr)
            if 'success=True' not in drain.stdout:
                raise RuntimeError(f'Input drain failed: {drain.stdout}')
    finally:
        try:
            stop(player)
        finally:
            stop(node)
    if node.returncode not in (0, -signal.SIGINT):
        raise RuntimeError(f'ORB abnormal exit: {node.returncode}; see {output / "node.log"}')
    result = summarize(output, backend)
    result['imu_initialization'] = 'UNKNOWN: tracking does not attest inertial initialization' if args.use_imu else 'disabled'
    (output / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['cpu', 'cuda'], default='cuda')
    parser.add_argument('--settings', type=Path)
    parser.add_argument('--bag', type=Path, help='Existing native RGBD bag; never records a new bag')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--start-offset', type=float, default=15)
    parser.add_argument('--rate', type=float, default=1)
    parser.add_argument('--output', type=Path, default=ROOT / 'output' / time.strftime('orb_gpu_%Y%m%d_%H%M%S'))
    parser.add_argument('--domain-id', type=int, default=174, help='Use an isolated domain for replay')
    parser.add_argument('--rgbd-topic', default='/camera/slam_camera/rgbd')
    parser.add_argument('--rgb-topic', help='Legacy separate-image bag RGB topic (requires --depth-topic)')
    parser.add_argument('--depth-topic', help='Legacy aligned depth topic (requires --rgb-topic)')
    parser.add_argument('--use-imu', action='store_true')
    parser.add_argument('--imu-topic', default='/camera/slam_camera/imu')
    parser.add_argument('--localization', action='store_true')
    parser.add_argument('--start-camera', action='store_true', help='Start one SLAM camera and stop it on exit; no recording')
    parser.add_argument('--serial', help='Exact RealSense serial required with --start-camera')
    parser.add_argument('--sync-mode', type=int, choices=[0, 1, 2, 3], default=1)
    parser.add_argument('--compare', action='store_true', help='CPU then CUDA on the same bag segment')
    parser.add_argument('--cuda-library', type=Path, default=LIBRARY)
    parser.add_argument('--build-cuda', action='store_true', help='Build plugin and exit (core must contain optional hook)')
    parser.add_argument('--nvcc', type=Path)
    parser.add_argument('--cuda-arch', default='sm_120', help='RTX 5090 is sm_120; choose actual GPU architecture')
    args = parser.parse_args()
    args.cuda_library = args.cuda_library.resolve()
    if args.build_cuda:
        build_cuda(args)
        return 0
    if not args.settings or not args.settings.is_file():
        parser.error('--settings must be an existing camera-calibrated ORB settings file')
    if not all(math.isfinite(x) and x > 0 for x in (args.seconds, args.rate)) or not math.isfinite(args.start_offset) or args.start_offset < 0:
        parser.error('seconds/rate must be positive and finite; start-offset must be finite and nonnegative')
    if not 0 <= args.domain_id <= 232:
        parser.error('domain-id must be in [0, 232]')
    if args.compare and not args.bag:
        parser.error('--compare requires --bag for identical input')
    if args.start_camera and (args.bag or args.compare or args.rgb_topic or args.use_imu):
        parser.error('--start-camera requires live native RGBD without --bag/--compare/--use-imu; use IMU launcher for inertial capture')
    if args.start_camera and (not args.serial or not args.serial.isdigit()):
        parser.error('--start-camera requires the exact numeric --serial')
    if args.start_camera and args.rgbd_topic != '/camera/slam_camera/rgbd':
        parser.error('--start-camera publishes /camera/slam_camera/rgbd')
    if bool(args.rgb_topic) != bool(args.depth_topic):
        parser.error('--rgb-topic and --depth-topic must be given together')
    if args.bag and not (args.bag / 'metadata.yaml').is_file():
        parser.error('--bag must contain metadata.yaml')
    args.output = args.output.resolve()
    args.settings = args.settings.resolve()
    if args.start_camera and args.output.exists():
        parser.error('--start-camera requires a new --output directory to preserve earlier camera logs')
    if args.bag:
        args.bag = args.bag.resolve()
    if args.backend == 'cuda' or args.compare:
        validate_cuda(args.cuda_library)
    backends = ('cpu', 'cuda') if args.compare else (args.backend,)
    camera = start_camera(args) if args.start_camera else None
    try:
        results = {backend: run(args, backend, args.output / backend) for backend in backends}
    finally:
        stop(camera)
    if args.compare:
        comparable = results['cpu']['input_stamp_sha256'] == results['cuda']['input_stamp_sha256']
        results['same_frame_count'] = results['cpu']['received'] == results['cuda']['received']
        results['same_input_stamps'] = comparable
        results['tracking_succeeded_both'] = all(results[backend]['status'] == 'PASS' for backend in backends)
        results['consume_median_speedup'] = (results['cpu']['consume_ms']['median'] /
            results['cuda']['consume_ms']['median']) if comparable and all(results[b]['consume_ms'] for b in backends) else None
        results['interpretation'] = 'Above 1 means CUDA faster; below 1 means slower. Full SLAM remains hybrid.'
        (args.output / 'comparison.json').write_text(json.dumps(results, indent=2))
        print(json.dumps(results, indent=2))
    passed = all(results[backend]['status'] == 'PASS' for backend in backends)
    return 0 if passed and (not args.compare or results['same_input_stamps']) else 2


if __name__ == '__main__':
    raise SystemExit(main())
