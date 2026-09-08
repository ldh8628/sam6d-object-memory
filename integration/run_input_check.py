#!/usr/bin/env python3
"""Camera-only baseline and incremental native RGB-D/recorder/ORB input checks."""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time

from camera_publish import camera_command, runtime_domain
from input_check import InputCheckSession
from rgbd_capture import recorder_args
from run_object_memory_rosbag import _conda_command, _stop, _request_shutdown


def run(args):
    cameras = dict(value.split('=',1) for value in args.camera)
    if len(cameras) != len(args.camera) or len(set(cameras.values()))!=len(cameras):
        raise ValueError('camera roles and serials must be distinct')
    if any(role not in ('slam_camera','sam_camera') or not serial for role,serial in cameras.items()):
        raise ValueError('use --camera slam_camera=SERIAL or sam_camera=SERIAL')
    if args.orb and 'slam_camera' not in cameras:
        raise ValueError('--orb requires slam_camera')
    output=args.output.resolve()
    if output.exists(): raise ValueError(f'refusing existing output: {output}')
    output.mkdir(parents=True)
    session=InputCheckSession(output,args.ros_domain_id,cameras,seconds=args.seconds,
        record=args.record,capture=output/'capture',baseline=args.baseline,
        consumers={'slam_camera':['orb']} if args.orb else {},
        is_baseline=not args.orb and not args.record,
        settings={'view':args.view,'hardware_sync':False})
    publishers=[]; consumers=[]; logs=[]; recorder=None; error=None
    try:
        for camera,serial in cameras.items():
            log=(output/f'{camera}_driver.log').open('x'); logs.append(log)
            publishers.append((camera,subprocess.Popen(camera_command(serial,camera,args.ros_domain_id),
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)))
        if args.record:
            log=(output/'recorder.log').open('x'); logs.append(log)
            recorder=subprocess.Popen(_conda_command('sam6d',recorder_args(output/'capture',cameras),
                domain_id=args.ros_domain_id),stdout=log,stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,start_new_session=True)
        if args.orb:
            from create_map_urdf import _camera_info, _map_viewer_command
            info=_camera_info('slam_camera',args.ros_domain_id)
            command=_map_viewer_command(argparse.Namespace(baseline=.095, ros_domain_id=args.ros_domain_id),
                output,info,cameras['slam_camera'])
            config=output/'live_map_preview/orb_config.yaml'
            import yaml
            data=yaml.safe_load(config.read_text()); data['runtime']['enable_viewer']=args.view
            config.write_text(yaml.safe_dump(data,sort_keys=False))
            log=(output/'orb.log').open('x'); logs.append(log)
            consumers.append(('orb',subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)))
        processes=publishers+consumers+([('recorder',recorder)] if recorder else [])
        session.wait_ready(processes,60)
        session.arm()
        print(f'10s warmup + {args.seconds}s measurement: {output}',flush=True)
        while not session.done():
            failed=[(n,p.returncode) for n,p in processes if p.poll() is not None]
            if failed: raise RuntimeError(f'input process exited: {failed}')
            time.sleep(.1)
    except KeyboardInterrupt:
        error='interrupted'
    except Exception as exc:
        error=repr(exc)
    finally:
        session.close_window()
        stopped=[_stop(p) for n,p in publishers]
        time.sleep(2)
        try: session.drain_consumers()
        except Exception as exc:
            stopped.append(False); error=str(error or '')+repr(exc)
        stopped.append(_stop(recorder))
        stopped += [_stop(p) for n,p in consumers]
        result=session.finish(drained=all(stopped),error=error)
        for log in logs: log.close()
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera',action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=120)
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--orb',action='store_true')
    parser.add_argument('--view',action='store_true')
    parser.add_argument('--baseline',type=Path)
    parser.add_argument('--ros-domain-id',type=int)
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0: parser.error('--seconds must be positive')
    if args.view and not args.orb: parser.error('--view requires --orb')
    args.ros_domain_id=runtime_domain(args.ros_domain_id)
    signal.signal(signal.SIGTERM,_request_shutdown)
    result=run(args)
    return 0 if result['status']=='PASS' else 2


if __name__=='__main__': raise SystemExit(main())
