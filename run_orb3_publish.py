#!/usr/bin/env python3
"""Run a single RGB-D camera against an immutable ORB-SLAM3 Atlas and publish ROS 2 poses.

Python handles configuration, process lifetime and evidence; the native node owns
acquisition and tracking. Use --help. No remote login or remote camera is needed.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parent


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--atlas', required=True, type=Path, help='Previously built, nonempty .osa Atlas (read only)')
    p.add_argument('--settings', required=True, type=Path, help='OpenCV YAML camera settings used to create that Atlas')
    p.add_argument('--serial', default='', help='RealSense serial; mandatory when more than one camera is connected')
    p.add_argument('--vocabulary', type=Path, default=ROOT/'orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    p.add_argument('--output', type=Path, help='New directory for per-frame CSV, trajectory, logs and summary')
    p.add_argument('--localhost-only', action='store_true', help='Limit ROS discovery to this host (for offline tests)')
    p.add_argument('--domain-id', type=int, default=77)
    p.add_argument('--discovery-server', default='', help='Optional Fast DDS discovery server, e.g. 10.77.0.2:11811')
    p.add_argument('--frame-id', default='map')
    p.add_argument('--camera-frame', default='slam_camera_color_optical_frame')
    p.add_argument('--queue-capacity', type=int, default=120, help='Complete RGB-D pairs; FIFO overflow fails the run, never overwrites old frames')
    p.add_argument('--max-backlog-ms', type=float, default=250.0, help='Stop acquisition if pending-frame age exceeds this bound; drain accepted frames and fail the run')
    p.add_argument('--opencv-threads', type=int, default=3, help='Three threads selected for this laptop; benchmark other hardware')
    p.add_argument('--rotation-fallback', choices=('on', 'off'), default='on')
    p.add_argument('--features', type=int, default=1200, help='ORB feature budget; 1200 tested at 640x480/30Hz on this laptop; 0 keeps the mapping setting')
    p.add_argument('--color-exposure', type=float, default=0.0, help='0 preserves the current exposure mode; positive value is SDK color exposure option units')
    p.add_argument('--max-pair-skew-ms', type=float, default=16.7)
    p.add_argument('--max-frames', type=int, default=0, help='Stop after N accepted pairs; 0 runs until Ctrl-C')
    p.add_argument('--realsense-bag', type=Path, help='Camera-free SDK playback for acquisition-path validation')
    p.add_argument('--dataset', type=Path, help='Camera-free replay directory with associations.txt; clearly labeled as replay in results')
    p.add_argument('--replay-rate', type=float, default=1.0, help='Dataset wall-clock rate; 1 = recorded timing, 0 = unpaced stress test')
    p.add_argument('--check', action='store_true', help='Validate files/runtime and print resolved configuration without opening a camera')
    return p


def ensure_environment() -> None:
    """Use the already installed Jazzy stack, including the existing WSL distro."""
    if os.environ.get('ORB3_PUBLISH_ENV') == '1':
        return
    wrapper = ROOT/'orb3_publish/with_env.sh'
    if Path('/opt/orb-live/ros2_jazzy/install/setup.bash').is_file():
        os.execv('/bin/bash', ['bash', str(wrapper), *sys.argv[1:]])
    # This workspace is shared between the user's two WSL distros.
    # No network configuration, package install or camera USB changes are made.
    wsl = Path('/mnt/c/Windows/System32/wsl.exe')
    if wsl.is_file() and str(ROOT).startswith('/mnt/c/'):
        # POSIX signals sent to wsl.exe do not reliably reach the other distro.
        # Keep this parent alive and relay only this invocation's stop request
        # through the shared filesystem; the inner launcher drains its native child.
        control_root=ROOT/'orbslam_ws/output/orb3_publish'
        control_root.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.control-',dir=control_root) as control:
            stop_file=Path(control)/'stop'
            command = [str(wsl), '-d', 'Ubuntu-22.04', '--cd', str(Path.cwd()), '--exec',
                       '/usr/bin/env',f'ORB3_PUBLISH_STOP_FILE={stop_file}', '/bin/bash', str(wrapper), *sys.argv[1:]]
            if Path('/init').exists() and not Path('/proc/sys/fs/binfmt_misc/WSLInterop').exists():
                command = ['/init', str(wsl), *command]
            previous={sig:signal.signal(sig,lambda signum,_frame:stop_file.write_text(str(signum)))
                      for sig in (signal.SIGINT,signal.SIGTERM)}
            try:
                # Keep terminal Ctrl-C in the Python parent's process group.
                process=subprocess.Popen(command,start_new_session=True)
                raise SystemExit(process.wait())
            finally:
                for sig,handler in previous.items():signal.signal(sig,handler)
    raise RuntimeError('Source your ROS Jazzy + orbslam3_ros2 install and set ORB3_PUBLISH_ENV=1; see orb3_publish/README.md')


def load_settings(path: Path) -> dict:
    import yaml
    text = path.read_text()
    if text.startswith('%YAML:'):
        text = '\n'.join(text.splitlines()[1:])
    try:
        cfg = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError(f'Invalid camera YAML: {exc}') from exc
    if not isinstance(cfg, dict) or str(cfg.get('File.version')) != '1.0' or cfg.get('Camera.type') != 'PinHole':
        raise ValueError('Expected File.version 1.0, Camera.type PinHole OpenCV settings')
    for key in ('Camera1.fx','Camera1.fy','Camera.width','Camera.height','Camera.fps','RGBD.DepthMapFactor','ORBextractor.nFeatures'):
        if not isinstance(cfg.get(key), (int,float)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'Invalid or missing {key}')
    for key in ('Camera1.cx','Camera1.cy'):
        if not isinstance(cfg.get(key), (int,float)) or not math.isfinite(cfg[key]):
            raise ValueError(f'Invalid or missing {key}')
    if any(key in cfg for key in ('Camera.newWidth','Camera.newHeight')):
        raise ValueError('Resized camera settings are unsupported; use the original mapping resolution')
    return cfg


def write_opencv_settings(path: Path, cfg: dict) -> None:
    # OpenCV FileStorage accepts JSON quoted scalars, but not YAML null/bool tags.
    lines = ['%YAML:1.0', '---']
    for key, value in cfg.items():
        if not isinstance(value, (str, int, float)) or isinstance(value, bool):
            raise ValueError(f'Unsupported OpenCV scalar for {key}: {value!r}')
        lines.append(f'{key}: {json.dumps(value, allow_nan=False, ensure_ascii=False)}')
    path.write_text('\n'.join(lines)+'\n')


def validate_args(args) -> None:
    if args.dataset and args.realsense_bag:
        raise ValueError('Choose only one replay source')
    if not 0 <= args.domain_id <= 232:
        raise ValueError('ROS domain-id must be 0..232')
    if not 1 <= args.queue_capacity <= 4096 or args.opencv_threads < 1 or args.max_frames < 0:
        raise ValueError('Invalid FIFO capacity, OpenCV thread count or max-frames')
    if args.features not in (None,0) and args.features < 500:
        raise ValueError('Use at least 500 features; reducing features can damage rotation tracking')
    if not all(math.isfinite(v) and v >= 0 for v in (args.replay_rate,args.color_exposure,args.max_pair_skew_ms,args.max_backlog_ms)) or min(args.max_pair_skew_ms,args.max_backlog_ms) == 0:
        raise ValueError('Invalid replay rate, exposure or pair skew')
    if len(args.serial.encode())>64 or not args.frame_id or not args.camera_frame or len(args.frame_id.encode())>128 or len(args.camera_frame.encode())>128:
        raise ValueError('Invalid serial or frame identifier')
    if args.discovery_server:
        from ipaddress import IPv4Address
        host,sep,port=args.discovery_server.rpartition(':')
        if not sep or not port.isdigit() or not 1<=int(port)<=65535:
            raise ValueError('Discovery server must be IPv4:port')
        IPv4Address(host)


def native_executable() -> Path:
    from ament_index_python.packages import get_package_prefix
    path = Path(get_package_prefix('orbslam3_ros2'))/'lib/orbslam3_ros2/orb3_live_node'
    if not path.is_file():
        raise RuntimeError('Build the native node first: bash wsl2/build_orb.sh')
    return path


def validate_replay_geometry(directory: Path, cfg: dict) -> None:
    import cv2
    with (directory/'associations.txt').open() as stream:
        first=next((line.split() for line in stream if line.strip() and not line.lstrip().startswith('#')),None)
    if first is None or len(first)!=4:
        raise ValueError('Replay has no valid first RGB-D association')
    rgb=cv2.imread(str(directory/first[1]),cv2.IMREAD_COLOR)
    depth=cv2.imread(str(directory/first[3]),cv2.IMREAD_UNCHANGED)
    expected=(int(cfg['Camera.height']),int(cfg['Camera.width']))
    if rgb is None or depth is None or rgb.shape[:2]!=expected or depth.shape!=expected or depth.dtype.name!='uint16':
        raise ValueError('First replay RGB-D images disagree with the map camera resolution or depth format')


def frame_rate_report(path: Path, expected_fps: float) -> dict:
    if not path.is_file(): return {}
    count=0; first_capture=last_capture=first_arrival=last_arrival=0
    long_intervals=late=invalid_run=longest_invalid=0; first_valid=None
    with path.open() as f:
        for row in csv.DictReader(f):
            capture=int(row['capture_ns']);arrival=int(row['enqueue_ns'])
            if count==0: first_capture=capture;first_arrival=arrival
            elif capture-last_capture>1.5e9/expected_fps: long_intervals+=1
            last_capture=capture;last_arrival=arrival;count+=1
            late+=float(row['enqueue_to_publish_ms'])>33
            if row['tracking']=='1':
                if first_valid is None: first_valid=count
                invalid_run=0
            else:
                invalid_run+=1;longest_invalid=max(longest_invalid,invalid_run)
    def hz(first,last):
        return (count-1)*1e9/(last-first) if count>1 and last>first else None
    capture_rate=hz(first_capture,last_capture)
    ingress_rate=hz(first_arrival,last_arrival)
    return dict(observed_frame_count=count,expected_capture_hz=expected_fps,
                observed_capture_hz=capture_rate,observed_ingress_hz=ingress_rate,
                source_rate_verified=count>=30 and capture_rate is not None and .95*expected_fps<=capture_rate<=1.05*expected_fps,
                ingress_rate_verified=count>=30 and ingress_rate is not None and .95*expected_fps<=ingress_rate<=1.05*expected_fps,
                capture_intervals_over_1_5_period=long_intervals,all_frames_over_33ms=late,
                first_valid_sequence=first_valid,longest_invalid_run_frames=longest_invalid)


def run(args) -> int:
    validate_args(args)
    atlas, settings, vocabulary = (p.expanduser().resolve(strict=True) for p in (args.atlas,args.settings,args.vocabulary))
    if atlas.suffix != '.osa' or atlas.stat().st_size == 0:
        raise ValueError('Atlas must be a nonempty .osa file; contents are validated by ORB-SLAM3 before opening the camera')
    if vocabulary.stat().st_size == 0:
        raise ValueError('Empty vocabulary')
    cfg=load_settings(settings)
    if args.dataset:
        args.dataset=args.dataset.expanduser().resolve(strict=True)
        if not (args.dataset/'associations.txt').is_file():
            raise ValueError('Replay requires associations.txt; use orb3_publish/prepare_tum.py')
        validate_replay_geometry(args.dataset,cfg)
    if args.realsense_bag: args.realsense_bag=args.realsense_bag.expanduser().resolve(strict=True)
    binary=native_executable()
    map_id=digest(atlas)
    info=dict(atlas=str(atlas),map_id=map_id,settings=str(settings),settings_sha256=digest(settings),
              vocabulary=str(vocabulary),vocabulary_sha256=digest(vocabulary),binary=str(binary),binary_sha256=digest(binary),
              orb_core_sha256=digest(ROOT/'orbslam_ws/src/ORB_SLAM3/lib/libORB_SLAM3.so'),
              source='dataset' if args.dataset else ('sdk_replay' if args.realsense_bag else 'camera'),domain_id=args.domain_id,
              discovery_server=args.discovery_server,localhost_only=args.localhost_only,rotation_fallback=args.rotation_fallback,
              all_frames_policy='FIFO; fail on gaps/duplicates/overflow; never invent a pose',
              source_fps=cfg['Camera.fps'],features=args.features or cfg['ORBextractor.nFeatures'],
              opencv_threads=args.opencv_threads,launcher_sha256=digest(Path(__file__)))
    if args.check:
        print(json.dumps(info,indent=2,ensure_ascii=False))
        return 0
    output=(args.output or ROOT/'orbslam_ws/output/orb3_publish'/time.strftime('%Y%m%d-%H%M%S')).expanduser().resolve()
    output.mkdir(parents=True,exist_ok=False)
    cfg.pop('System.SaveAtlasToFile',None)
    cfg['System.LoadAtlasFromFile']=os.path.relpath(atlas.with_suffix(''),output)
    cfg['Tracking.RotationFallback']=int(args.rotation_fallback=='on')
    cfg['Camera.RGB']=0  # Both SDK and cv::imread provide BGR; no axis or unit changes.
    if args.features: cfg['ORBextractor.nFeatures']=args.features
    generated=output/'settings.yaml'
    write_opencv_settings(generated,cfg)
    session=uuid.uuid4().hex
    params=dict(settings=str(generated),vocabulary=str(vocabulary),output=str(output),serial=args.serial,
                map_id=map_id,session_id=session,frame_id=args.frame_id,camera_frame=args.camera_frame,
                queue_capacity=args.queue_capacity,max_backlog_ms=args.max_backlog_ms,opencv_threads=args.opencv_threads,max_frames=args.max_frames,
                dataset=str(args.dataset or ''),realsense_bag=str(args.realsense_bag or ''),replay_rate=args.replay_rate,color_exposure=args.color_exposure,
                max_pair_skew_ms=args.max_pair_skew_ms)
    # Parameter file prevents ROS CLI interpreting numeric serials or punctuation.
    import yaml
    param_file=output/'parameters.yaml'
    param_file.write_text(yaml.safe_dump({'orb3_live_publish':{'ros__parameters':params}},sort_keys=False))
    env=os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain_id),RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               ROS_LOCALHOST_ONLY='1' if args.localhost_only else '0',
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST' if args.localhost_only else 'SUBNET',
               RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS',RMW_FASTRTPS_USE_QOS_FROM_XML='0')
    if args.discovery_server: env['ROS_DISCOVERY_SERVER']=args.discovery_server
    else: env.pop('ROS_DISCOVERY_SERVER',None)
    # The existing local SHM+UDP profile supports poses on LAN. Images stay in-process.
    env.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE',str(ROOT/'wsl2/fastdds_live.xml'))
    command=[str(binary),'--ros-args','--params-file',str(param_file)]
    info.update(output=str(output),session_id=session,parameters=params,command=command)
    (output/'run.json').write_text(json.dumps(info,indent=2,ensure_ascii=False)+'\n')
    print(f"Starting saved-map localization [{info['source']}]. Pose: /orbslam3/pose; per-frame validity: /distributed_slam/state",flush=True)
    print('Output:',output,flush=True)
    child=None
    stop_watcher=None
    watcher_done=threading.Event()
    def request_stop(signum, _frame):
        if child and child.poll() is None:
            child.send_signal(signum)
    previous={sig:signal.signal(sig,request_stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    def watch_dispatcher():
        stop_file=Path(os.environ['ORB3_PUBLISH_STOP_FILE'])
        while not watcher_done.wait(.05):
            try: signum=int(stop_file.read_text())
            except (OSError,ValueError): continue
            if signum in (signal.SIGINT,signal.SIGTERM):
                request_stop(signum,None)
                return
    code=2
    try:
        with (output/'console.log').open('w') as log:
            child=subprocess.Popen(command,cwd=output,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                   text=True,bufsize=1,start_new_session=True)
            if os.environ.get('ORB3_PUBLISH_STOP_FILE'):
                stop_watcher=threading.Thread(target=watch_dispatcher,name='wsl-stop-relay',daemon=True)
                stop_watcher.start()
            for line in child.stdout:
                log.write(line)
                log.flush()
                print(line,end='',flush=True)
            code=child.wait()
    finally:
        watcher_done.set()
        if stop_watcher: stop_watcher.join()
        if child and child.poll() is None:
            child.terminate()
            try: child.wait(timeout=45)
            except subprocess.TimeoutExpired: child.kill(); child.wait()
        for sig,handler in previous.items(): signal.signal(sig,handler)
        unchanged=atlas.is_file() and digest(atlas)==map_id
        native_path=output/'native_summary.json'
        native=json.loads(native_path.read_text()) if native_path.is_file() else dict(
            error='Native process exited before writing summary; see console.log',
            complete_input_processing=False,processed_frames=0,valid_poses=0)
        if code==0 and not native.get('valid_poses',0): code=5
        if not unchanged: code=4
        summary=dict(native,exit_code=code,atlas_unchanged=unchanged,map_id=map_id,
                     features=cfg['ORBextractor.nFeatures'],opencv_threads=args.opencv_threads,
                     rotation_fallback=args.rotation_fallback,
                     source=info['source'],lan_latency_verified=False,
                     hardware_verified=info['source']=='camera' and native.get('complete_input_processing',False),
                     all_frames_valid=bool(native.get('processed_frames')) and native.get('valid_poses')==native.get('processed_frames'))
        # p99 is measured from the SAME frame's ingress through local publication.
        # It does not establish the requested cross-machine reception latency.
        summary.update(frame_rate_report(output/'frames.csv',float(cfg['Camera.fps'])))
        summary['realtime_input_contract_met']=bool(native.get('complete_input_processing') and
                                                   summary.get('source_rate_verified') and summary.get('ingress_rate_verified'))
        summary['local_33ms_met']=bool(code==0 and unchanged and native.get('complete_input_processing') and
                                      native.get('valid_poses',0)>0 and native.get('valid_enqueue_to_publish_p99_ms',float('inf'))<=33)
        summary['full_requirement_verified']=False  # camera+remote reception acceptance is a separate test
        (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
        print('Result:',output/'summary.json',flush=True)
        if not unchanged:
            print('ERROR: source Atlas checksum changed',file=sys.stderr)
    return code


def main() -> int:
    args=parser().parse_args()
    try:
        validate_args(args)
        ensure_environment()
        return run(args)
    except (ValueError,RuntimeError,OSError,ImportError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        return 2

if __name__=='__main__':
    raise SystemExit(main())
