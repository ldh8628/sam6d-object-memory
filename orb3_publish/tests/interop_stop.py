#!/usr/bin/env python3
"""Run from the original terminal to verify SIGINT through the WSL dispatcher."""
import argparse,json,os,re,signal,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--atlas',required=True)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--dataset');source.add_argument('--realsense-bag')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
    a=p.parse_args();out=a.output.resolve()
    if out.exists():raise ValueError('Test output must be new')
    out.parent.mkdir(parents=True,exist_ok=True)
    command=[sys.executable,str(ROOT.parent/'run_orb3_publish.py'),'--localhost-only',
             '--atlas',a.atlas,'--settings',str(ROOT/'orb3_publish/config/TUM1.yaml'),
             '--realsense-bag' if a.realsense_bag else '--dataset',a.realsense_bag or a.dataset,'--output',str(out)]
    result={};clean=False
    with out.with_suffix('.log').open('x') as log:
        process=subprocess.Popen(command,cwd=ROOT.parent,stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+90
            while process.poll() is None and time.monotonic()<deadline:
                frames=out/'frames.csv'
                if frames.exists() and frames.stat().st_size>16384:break
                time.sleep(.1)
            if process.poll() is not None:raise RuntimeError('Launcher exited before the signal test')
            started=time.monotonic();process.send_signal(signal.SIGINT)
            while time.monotonic()-started<15:
                try:
                    result=json.loads((out/'summary.json').read_text());break
                except (OSError,json.JSONDecodeError):time.sleep(.05)
            clean=bool(result.get('complete_input_processing') and result.get('atlas_unchanged') and result.get('valid_poses',0)>0)
            if clean:process.wait(timeout=10)
            report=dict(verified=clean,outer_exit_code=process.poll(),shutdown_seconds=time.monotonic()-started,
                        processed_frames=result.get('processed_frames'),valid_poses=result.get('valid_poses'),
                        output=str(out),source='sdk_replay' if a.realsense_bag else 'dataset',remote_lan_verified=False)
            a.report.parent.mkdir(parents=True,exist_ok=True)
            a.report.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
            if not clean:raise RuntimeError('SIGINT did not produce a drained, clean shutdown through WSL')
        finally:
            if not clean:
                # Only this test's native process has this unique parameter path.
                wsl='/mnt/c/Windows/System32/wsl.exe'
                cleanup=[wsl,'-d','Ubuntu-22.04','--exec','pkill','-INT','-f',re.escape(str(out/'parameters.yaml'))]
                if Path('/init').exists() and not Path('/proc/sys/fs/binfmt_misc/WSLInterop').exists():cleanup=['/init',wsl,*cleanup]
                subprocess.run(cleanup,timeout=15,check=False)
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':main()
