#!/usr/bin/env python3
"""Record both laptops, build the map/URDF remotely, and retrieve the results."""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

from two_host import ROOT, load_config, local_command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--name', required=True, help='new session directory name')
    parser.add_argument('--config', type=Path, default=ROOT / 'integration/two_host.local.yaml')
    parser.add_argument('--baseline', type=float, default=.095)
    parser.add_argument('--camera-timeout', type=float, default=30.)
    parser.add_argument('--input-check-seconds', type=float, default=0.,
                        help='0: Enter after at least 600 seconds; >=600: stop automatically after warmup and validation')
    parser.add_argument('--input-check-only', action='store_true',
                        help='validate both camera inputs without building a map')
    args = parser.parse_args(argv)
    if (not args.name or Path(args.name).name != args.name or args.name in ('.', '..')
            or '\0' in args.name):
        parser.error('--name must be one dataset directory name')
    for name in ('baseline', 'camera_timeout'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error('--' + name.replace('_', '-') + ' must be finite and positive')
    seconds = args.input_check_seconds
    if not math.isfinite(seconds) or (seconds != 0 and seconds < 600):
        parser.error('--input-check-seconds must be 0 (interactive) or at least 600')
    try:
        config_path = args.config.expanduser().resolve()
        config = load_config(config_path)
        command = local_command(config, [
            'python', 'integration/create_map_urdf.py', '--name=' + args.name,
            '--two-host-config', str(config_path), '--baseline', str(args.baseline),
            '--camera-timeout', str(args.camera_timeout),
            '--input-check-seconds', str(seconds),
            *(['--input-check-only'] if args.input_check_only else []),
        ])
        os.execvp(command[0], command)
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
