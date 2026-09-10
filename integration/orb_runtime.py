"""ORB-only descriptor selection and access to the host's existing desktop."""
from __future__ import annotations

import argparse
import json
import mmap
import os
from pathlib import Path
import re
import subprocess
import sys


def desktop_environment():
    try:
        result = subprocess.run(['systemctl', '--user', 'show-environment'],
            check=True, capture_output=True, text=True, timeout=10)
        values = dict(line.split('=', 1) for line in result.stdout.splitlines()
                      if line.startswith(('DISPLAY=', 'XAUTHORITY=')))
        if not values.get('XAUTHORITY'):
            values['XAUTHORITY'] = str(Path.home() / '.Xauthority')
        display = re.fullmatch(r':(\d+)(?:\.\d+)?', values.get('DISPLAY', ''))
        if not display or not Path('/tmp/.X11-unix/X' + display[1]).exists():
            raise RuntimeError('local X display is unavailable')
        subprocess.run(['xdpyinfo'], env={**os.environ, **values}, check=True,
                       capture_output=True, text=True, timeout=10)
        return values
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        raise RuntimeError('ORB viewer needs an active desktop session on this host; '
                           'log in to the desktop or use --headless: ' + str(exc)) from exc


def orb_environment(workspace, backend='cpu', *, view=False, desktop=False):
    if backend not in ('cpu', 'cuda'):
        raise ValueError('ORB descriptor backend must be cpu or cuda')
    environment = dict(ORB_SLAM3_DESCRIPTOR_BACKEND=backend, ORB_SLAM3_CUDA_LIBRARY='')
    if backend == 'cuda':
        core = Path(workspace) / 'src/ORB_SLAM3/lib'
        try:
            with (core / 'libORB_SLAM3.so').open('rb') as handle, mmap.mmap(
                    handle.fileno(), 0, access=mmap.ACCESS_READ) as binary:
                if binary.find(b'ORB_SLAM3_DESCRIPTOR_BACKEND') < 0:
                    raise RuntimeError('ORB core lacks the descriptor backend hook; rebuild it')
            from run_orb_slam_gpu import validate_cuda
            library = core / 'liborb_cuda_descriptors.so'
            validate_cuda(library)
        except (OSError, ValueError, AttributeError, RuntimeError) as exc:
            raise RuntimeError('CUDA descriptor backend unavailable (no CPU fallback): ' + str(exc)) from exc
        environment['ORB_SLAM3_CUDA_LIBRARY'] = str(library)
    if view and desktop:
        environment.update(desktop_environment())
    print(f'ORB_RUNTIME requested={backend} viewer={view} '
          f'cuda_library={environment["ORB_SLAM3_CUDA_LIBRARY"] or "none"}', file=sys.stderr, flush=True)
    return environment


def require_cuda_execution(log_path, backend):
    if backend == 'cuda' and 'ORB_DESCRIPTOR_BACKEND cuda:' not in Path(log_path).read_text(errors='replace'):
        raise RuntimeError('CUDA execution was not attested by ORB; see ' + str(log_path))


def remote_probe(config, backend, view=False):
    from two_host import execute, remote_command
    command = ['python', 'integration/orb_runtime.py', '--orb-backend', backend,
               *(['--view'] if view else [])]
    return json.loads(execute(remote_command(config, command), timeout=30))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--orb-backend', choices=('cpu', 'cuda'), default='cpu')
    parser.add_argument('--view', action='store_true')
    args = parser.parse_args()
    try:
        print(json.dumps(orb_environment(Path(__file__).resolve().parents[1] / 'orbslam_ws',
            args.orb_backend, view=args.view, desktop=True)))
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, str(exc) + '\n')


if __name__ == '__main__':
    main()
