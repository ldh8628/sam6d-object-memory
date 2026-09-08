#!/usr/bin/env python3
"""Two-host transport, configuration and preflight; Python standard library only."""
from __future__ import annotations
import argparse
import ctypes
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import platform
import queue
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
TOPICS = {'/orbslam3/pose': 'geometry_msgs/msg/PoseStamped',
          '/orbslam3/map_id': 'std_msgs/msg/String',
          '/orbslam3/tracking_state': 'std_msgs/msg/String',
          '/orbslam3/ready': 'std_msgs/msg/String'}


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def load_config(path):
    """Accept JSON (a YAML subset) or the example's two-level scalar YAML.

    Deliberately reject advanced YAML instead of adding a deployment dependency.
    """
    raw = Path(path).read_text()
    if raw.lstrip().startswith('{'):
        config = json.loads(raw)
    else:
        config, section = {}, None
        for number, line in enumerate(raw.splitlines(), 1):
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            match = re.fullmatch(r'(  )?([a-z_]+):(?:\s+(.*))?', line)
            if not match:
                raise ValueError(f'unsupported config syntax at line {number}; use example or JSON')
            indent, key, value = match.groups()
            if not indent:
                if value is not None or key in config:
                    raise ValueError(f'invalid/duplicate section: {key}')
                config[key], section = {}, key
            else:
                if section is None or value is None or key in config[section]:
                    raise ValueError(f'invalid/duplicate key: {key}')
                if value.startswith('"'):
                    value = json.loads(value)
                elif value.startswith("'") and value.endswith("'"):
                    value = value[1:-1].replace("''", "'")
                elif re.fullmatch(r'-?\d+(\.\d+)?', value):
                    value = float(value) if '.' in value else int(value)
                elif not re.fullmatch(r'[a-zA-Z0-9_@./:+-]+', value):
                    raise ValueError(f'quote scalar at line {number}')
                config[section][key] = value
    if not isinstance(config, dict):
        raise ValueError('configuration must be a mapping')
    required = {'ssh': {'target', 'remote_root'},
                'network': {'local_ip', 'remote_ip', 'ros_domain_id', 'local_ptp_interface',
                            'remote_ptp_interface', 'ptp_max_offset_us'},
                'cameras': {'slam_sync_mode', 'sam_sync_mode'}}
    for group, keys in required.items():
        if not isinstance(config.get(group), dict):
            raise ValueError(f'{group} must be a mapping')
        if keys - config[group].keys():
            raise ValueError(f'missing {group} settings: {sorted(keys - config[group].keys())}')
    if set(config) - {*required, 'environment'}:
        raise ValueError('unknown config section (credentials must use OpenSSH)')
    for group, keys in required.items():
        optional = {'ssh': {'port'}, 'cameras': {'slam_serial', 'sam_serial'}}
        if set(config[group]) - keys - optional.get(group, set()):
            raise ValueError(f'unknown {group} setting')
    port = config['ssh'].setdefault('port', 22)
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('ssh.port must be 1..65535')
    target = config['ssh']['target']
    if not isinstance(target, str) or not re.fullmatch(r'(?:[\w.-]+@)?[\w.-]+', target) or target.startswith('-'):
        raise ValueError('ssh.target must be a host/SSH alias, optionally user@host')
    remote_root = config['ssh']['remote_root']
    if not isinstance(remote_root, str) or not remote_root.startswith('/') or remote_root == '/' or '..' in Path(remote_root).parts:
        raise ValueError('remote_root must be an absolute project directory')
    network = config['network']
    for key in ('local_ip', 'remote_ip'):
        ipaddress.IPv4Address(network[key])
    if network['local_ip'] == network['remote_ip']:
        raise ValueError('local and remote IP must differ')
    domain = network['ros_domain_id']
    if type(domain) is not int or not 0 <= domain <= 231:
        raise ValueError('ros_domain_id must be 0..231; next domain is reserved for camera isolation')
    for key in ('local_ptp_interface', 'remote_ptp_interface'):
        if not re.fullmatch(r'[\w.-]{1,15}', network[key]):
            raise ValueError(f'invalid {key}')
    offset = network['ptp_max_offset_us']
    if isinstance(offset, bool) or not isinstance(offset, (int, float)) or not math.isfinite(offset) or not 0 < offset <= 1000:
        raise ValueError('ptp_max_offset_us must be positive and at most 1000')
    cams = config['cameras']
    for key in ('slam_serial', 'sam_serial'):
        cams.setdefault(key, 'auto')
        if not isinstance(cams[key], str) or not re.fullmatch(r'auto|\d+', cams[key]):
            raise ValueError(f'{key} must be auto or a quoted numeric serial')
    if (type(cams['slam_sync_mode']) is not int or type(cams['sam_sync_mode']) is not int
            or cams['slam_serial'] == cams['sam_serial'] != 'auto'
            or cams['slam_sync_mode'] != 1 or cams['sam_sync_mode'] != 3):
        raise ValueError('distinct cameras and SLAM master 1 / SAM full slave 3 required')
    defaults = dict(local_conda_sh=str(Path.home() / 'miniconda3/etc/profile.d/conda.sh'),
                    remote_conda_sh='/home/' + (target.split('@')[0] if '@' in target else 'etri') + '/miniconda3/etc/profile.d/conda.sh',
                    orb_env='realsense', sam_env='sam6d')
    given = config.get('environment', {})
    if not isinstance(given, dict) or set(given) - defaults.keys():
        raise ValueError('unknown environment setting')
    defaults.update(given)
    if any(not isinstance(v, str) or not v or '\n' in v for v in defaults.values()):
        raise ValueError('environment values must be nonempty single-line strings')
    config['environment'] = defaults
    return config


def cli_domain(args):
    config = load_config(args.two_host_config)
    domain = config['network']['ros_domain_id']
    if args.ros_domain_id is not None and args.ros_domain_id != domain:
        raise ValueError('--ros-domain-id conflicts with two-host config')
    for role in ('slam', 'sam'):
        selected = getattr(args, role + '_serial')
        if selected is not None and selected != config['cameras'][role + '_serial']:
            raise ValueError('camera serial conflicts with two-host config')
    print(f'[ROS] two-host LAN domain {domain}; camera domain {domain + 1}', flush=True)
    return domain


def execute(argv, **kwargs):
    return subprocess.run(argv, check=True, text=True, capture_output=True, **kwargs).stdout.strip()


def camera_serials():
    """Enumerate this host's RealSense devices without starting image streams."""
    from camera_publish import parse_realsense_serials
    serials = sorted(set(parse_realsense_serials(execute(['rs-enumerate-devices'], timeout=10)).values()))
    if any(not re.fullmatch(r'\d+', value) for value in serials):
        raise ValueError('invalid serial in RealSense enumeration')
    return serials


def resolve_cameras(config):
    """Resolve roles by host once; workers verify these exact devices at startup."""
    selected = {}
    for role, command in (('sam', local_command), ('slam', remote_command)):
        serials = json.loads(execute(command(config, ['python', 'integration/two_host.py', 'cameras']), timeout=30))
        if (not isinstance(serials, list) or
                any(not isinstance(value, str) or not re.fullmatch(r'\d+', value) for value in serials)):
            raise ValueError(f'{role} host returned invalid camera enumeration')
        serials = sorted(set(serials))
        requested = config['cameras'].get(role + '_serial', 'auto')
        if requested == 'auto':
            if len(serials) != 1:
                raise ValueError(f'{role} host requires exactly one RealSense camera for auto selection; '
                                 f'found {serials}; select {role}_serial explicitly if multiple are connected')
            requested = serials[0]
        elif requested not in serials:
            raise ValueError(f'{role} camera {requested} is not connected to its host; found {serials}')
        selected[role + '_serial'] = requested
    if selected['slam_serial'] == selected['sam_serial']:
        raise ValueError('SAM and SLAM hosts resolved to the same camera serial')
    config['cameras'].update(selected)
    print(f"[cameras] SAM (local)={selected['sam_serial']}; SLAM (remote)={selected['slam_serial']}", flush=True)
    return config['cameras']


def ssh_command(config, argv):
    return ['ssh', '-p', str(config['ssh'].get('port', 22)), '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=3',
            config['ssh']['target'], shlex.join([str(a) for a in argv])]


def local_command(config, argv, role='local', lan=False, environment=None, source=True):
    remote = role == 'remote'
    root = Path(config['ssh']['remote_root']) if remote else ROOT
    env = config['environment']
    domain = config['network']['ros_domain_id'] + (0 if lan else 1)
    profile = root / 'integration' / ('fastdds_two_host.xml' if lan else 'fastdds_input.xml')
    exports = dict(ROS_DOMAIN_ID=str(domain), RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                   ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                   ROS_STATIC_PEERS=config['network']['local_ip' if remote else 'remote_ip'] if lan else '',
                   FASTRTPS_DEFAULT_PROFILES_FILE=str(profile), FASTDDS_DEFAULT_PROFILES_FILE=str(profile))
    parts = ['set -e',
             'unset PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH ROS_LOCALHOST_ONLY ROS_DISCOVERY_SERVER',
             'source ' + shlex.quote(env['remote_conda_sh' if remote else 'local_conda_sh']),
             'conda activate ' + shlex.quote(environment or env['orb_env'])]
    if source:
        parts.append('source ' + shlex.quote(str(root / 'orbslam_ws/install_jazzy/setup.bash')))
    parts += ['export ' + ' '.join(k + '=' + shlex.quote(v) for k, v in exports.items()),
              'cd ' + shlex.quote(str(root)), 'exec ' + shlex.join([str(a) for a in argv])]
    return ['bash', '-c', '; '.join(parts)]


def remote_command(config, argv, lan=False, **kwargs):
    return ssh_command(config, local_command(config, argv, role='remote', lan=lan, **kwargs))


def tree_hashes(path):
    path = Path(path)
    if path.is_symlink():
        raise ValueError(f'refusing symlink asset: {path}')
    files = [path] if path.is_file() else sorted(path.rglob('*'))
    if not path.exists():
        raise FileNotFoundError(path)
    result = {}
    for file in files:
        if file.is_symlink():
            raise ValueError(f'refusing symlink asset: {file}')
        if file.is_file():
            digest = hashlib.sha256()
            with file.open('rb') as stream:
                for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
                    digest.update(chunk)
            result['.' if path.is_file() else file.relative_to(path).as_posix()] = digest.hexdigest()
    if not result:
        raise ValueError(f'empty asset: {path}')
    return result


def remote_python(config, code, *args):
    return execute(ssh_command(config, ['python3', '-c', code, *map(str, args)]), timeout=120)


def promote(staging, destination):
    """Linux atomic no-replace rename; never clobber a concurrent transfer."""
    libc = ctypes.CDLL(None, use_errno=True)
    result = libc.renameat2(-100, os.fsencode(staging), -100, os.fsencode(destination), 1)
    if result:
        code = ctypes.get_errno()
        raise OSError(code, os.strerror(code), str(destination))


def checked_sync(config, source, destination, to_remote=True):
    """Transfer into a unique staging path, verify SHA-256, then atomic rename.

    Existing identical targets are reusable; different targets are never replaced.
    """
    module = str(Path(config['ssh']['remote_root']) / 'integration/two_host.py')
    remote_manifest = "import runpy,json,sys; m=runpy.run_path(sys.argv[1]); print(json.dumps(m['tree_hashes'](sys.argv[2])))"
    source, destination = str(source), str(destination)
    source_hashes = tree_hashes(source) if to_remote else json.loads(remote_python(config, remote_manifest, module, source))
    if to_remote:
        exists = remote_python(config, 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).exists())', destination) == 'True'
        existing = json.loads(remote_python(config, remote_manifest, module, destination)) if exists else None
    else:
        exists = Path(destination).exists()
        existing = tree_hashes(destination) if exists else None
    if exists:
        if existing != source_hashes:
            raise ValueError(f'refusing to replace different asset: {destination}')
        return source_hashes
    staging = destination + '.staging-' + uuid.uuid4().hex
    if to_remote:
        is_dir = Path(source).is_dir()
        remote_python(config, 'import pathlib,sys; pathlib.Path(sys.argv[1]).parent.mkdir(parents=True,exist_ok=True)', staging)
    else:
        is_dir = remote_python(config, 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).is_dir())', source) == 'True'
        Path(staging).parent.mkdir(parents=True, exist_ok=True)
    remote = config['ssh']['target'] + ':'
    transport = 'ssh -p ' + str(config['ssh'].get('port', 22)) + ' -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=2 -o ServerAliveCountMax=3'
    src = source + ('/' if is_dir else '')
    dst = staging + ('/' if is_dir else '')
    subprocess.run(['rsync', '-a', '--protect-args', '--checksum', '-e', transport, '--',
                    src if to_remote else remote + src, remote + dst if to_remote else dst], check=True)
    actual = json.loads(remote_python(config, remote_manifest, module, staging)) if to_remote else tree_hashes(staging)
    if actual != source_hashes:
        raise RuntimeError(f'checksum mismatch; retained staging for inspection: {staging}')
    if to_remote:
        remote_python(config, "import runpy,sys; runpy.run_path(sys.argv[1])['promote'](sys.argv[2],sys.argv[3])", module, staging, destination)
    else:
        promote(staging, destination)
    return actual


class RemoteSession:
    """A controller-held stdin lease; worker EOF/timeout guarantees child cleanup."""
    def __init__(self, config, argv, local=False, **kwargs):
        self.process = subprocess.Popen((local_command if local else remote_command)(config, argv, **kwargs),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1, start_new_session=True)
        self.lines, self.closed = queue.Queue(), threading.Event()
        self.lock = threading.Lock()
        def read():
            for line in self.process.stdout:
                if line.startswith('TWO_HOST_READY '):
                    self.lines.put(line[len('TWO_HOST_READY '):])
                else:
                    print(line, end='', file=sys.stderr)
        def heartbeat():
            while not self.closed.wait(1):
                try:
                    with self.lock:
                        self.process.stdin.write('PING\n')
                        self.process.stdin.flush()
                except (OSError, ValueError):
                    return
        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()
        threading.Thread(target=heartbeat, daemon=True).start()
    def check(self):
        if self.process.poll() is not None:
            raise RuntimeError(f'worker exited: {self.process.returncode}')
    def wait_ready(self, timeout=60):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check()
            try:
                return json.loads(self.lines.get(timeout=.1))
            except queue.Empty:
                pass
        raise TimeoutError('worker READY timeout')
    def close(self):
        if self.closed.is_set():
            return
        self.closed.set()
        try:
            with self.lock:
                self.process.stdin.write('STOP\n')
                self.process.stdin.flush()
                self.process.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass
        finally:
            try:
                self.process.stdin.close()
            except OSError:
                pass
        try:
            code = self.process.wait(timeout=180)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
            raise RuntimeError('worker cleanup timeout')
        finally:
            self.reader.join(timeout=1)
            if not self.reader.is_alive():
                self.process.stdout.close()
        if code:
            raise RuntimeError(f'worker failed during shutdown: {code}')
    def __enter__(self):
        return self
    def __exit__(self, *_):
        self.close()


def parse_ptp(text, role):
    def value(name):
        values = re.findall(r'^\s*' + re.escape(name) + r'\s+(\S+)\s*$', text, re.M)
        if len(values) != 1:
            raise ValueError(f'PTP field missing/ambiguous: {name}')
        return values[0]
    state = value('portState')
    if state != ('MASTER' if role == 'remote' else 'SLAVE'):
        raise ValueError(f'PTP {role} has wrong portState: {state}')
    offset = float(value('master_offset')) / 1000
    if not math.isfinite(offset):
        raise ValueError('PTP offset is not finite')
    if role == 'local' and value('gmPresent').lower() not in ('true', '1'):
        raise ValueError('PTP grandmaster unavailable')
    return dict(port_state=state, offset_us=offset, grandmaster_id=value('gmIdentity'),
                utc_offset=int(value('currentUtcOffset')),
                ptp_timescale=value('ptpTimescale') == '1')


def ptp_probe(interface, role):
    if not (Path('/sys/class/net') / interface).is_dir():
        raise ValueError(f'PTP interface missing: {interface}')
    if (Path('/sys/class/net') / interface / 'wireless').exists():
        raise ValueError('PTP requires the configured wired interface')
    # pmc defaults its client socket to /var/run, which normal users cannot write.
    with tempfile.TemporaryDirectory(prefix='ptp-', dir='/tmp') as client_dir:
        text = execute(['pmc', '-u', '-b', '0', '-i', client_dir + '/pmc',
                        'GET PORT_PROPERTIES_NP', 'GET TIME_STATUS_NP',
                        'GET TIME_PROPERTIES_DATA_SET'], timeout=5)
    result = parse_ptp(text, role)
    interfaces = re.findall(r'^\s*interface\s+(\S+)\s*$', text, re.M)
    timestamping = re.findall(r'^\s*timestamping\s+(\S+)\s*$', text, re.M)
    if interfaces != [interface] or len(timestamping) != 1:
        raise ValueError('PTP daemon is not bound to the configured interface')
    result['timestamping'] = timestamping[0].lower()
    if result['timestamping'] == 'software':
        # ptp4l software timestamping disciplines CLOCK_REALTIME itself; no PHC conversion.
        result.update(interface=interface, phc=None, system_offset_us=0.,
                      system_clock_source='ptp4l CLOCK_REALTIME',
                      measurement_uncertainty_us=0., checked_unix_s=time.time())
        return result
    if result['timestamping'] not in ('hardware', 'onestep', 'p2p1step'):
        raise ValueError('unsupported PTP timestamping mode')
    devices = list((Path('/sys/class/net') / interface / 'device/ptp').glob('ptp*'))
    if len(devices) != 1:
        raise ValueError(f'one hardware PTP clock required on {interface}')
    fd = os.open('/dev/' + devices[0].name, os.O_RDONLY)
    try:
        samples = []
        for _ in range(5):
            before = time.time_ns()
            phc = time.clock_gettime_ns(((~fd) << 3) | 3)
            after = time.time_ns()
            correction = result['utc_offset'] * 1_000_000_000 if result['ptp_timescale'] else 0
            samples.append((after - before, ((before + after) // 2 - (phc - correction)) / 1000))
        span, result['system_offset_us'] = min(samples)
        result.update(interface=interface, phc=devices[0].name, measurement_uncertainty_us=span / 2000,
                      checked_unix_s=time.time())
    finally:
        os.close(fd)
    return result


def environment_probe():
    types = {}
    for name in sorted(set(TOPICS.values()) | {'realsense2_camera_msgs/msg/RGBD'}):
        definition = execute(['ros2', 'interface', 'show', name], timeout=15)
        types[name] = hashlib.sha256(definition.encode()).hexdigest()
    return dict(host=platform.node(), os=platform.platform(), python=platform.python_version(),
                conda=os.environ.get('CONDA_DEFAULT_ENV'), ros_distro=os.environ.get('ROS_DISTRO'),
                rmw=os.environ.get('RMW_IMPLEMENTATION'), domain=int(os.environ['ROS_DOMAIN_ID']), message_types=types)


def probe(interface=None, role='local'):
    result = dict(commit=execute(['git', '-C', str(ROOT), 'rev-parse', 'HEAD']), environment=environment_probe())
    if execute(['git', '-C', str(ROOT), 'status', '--porcelain']):
        raise ValueError('working tree is dirty')
    if interface:
        result['ptp'] = ptp_probe(interface, role)
    return result


def validate_clock_pair(sample, maximum_us):
    # Include both PHC-to-system conversions in the host-to-host error budget.
    clocks = [sample[role].get('ptp', sample[role]) for role in ('local', 'remote')]
    if any(not p.get('grandmaster_id') for p in clocks) or clocks[0]['grandmaster_id'] != clocks[1]['grandmaster_id']:
        raise ValueError('hosts do not share the same PTP grandmaster')
    total = sum(abs(p['offset_us']) + abs(p['system_offset_us']) + p['measurement_uncertainty_us']
                for p in clocks)
    if not math.isfinite(total) or total > maximum_us:
        raise ValueError(f'combined host clock error bound {total:g} us exceeds {maximum_us:g} us')
    sample['host_clock_error_bound_us'] = total
    return sample


def preflight(config):
    report = {}
    for role in ('local', 'remote'):
        command = ['python', 'integration/two_host.py', 'probe', '--role', role,
                   '--interface', config['network'][role + '_ptp_interface']]
        report[role] = json.loads(execute((local_command if role == 'local' else remote_command)(config, command), timeout=90))
        ptp = report[role]['ptp']
        if abs(ptp['offset_us']) + abs(ptp['system_offset_us']) + ptp['measurement_uncertainty_us'] > config['network']['ptp_max_offset_us']:
            raise ValueError(f'{role} PTP/system offset exceeds limit: {ptp}')
    sam = json.loads(execute(local_command(config, ['python', 'integration/two_host.py', 'probe'],
        environment=config['environment']['sam_env'], source=False), timeout=90))
    report['local']['sam_environment'] = sam['environment']
    if report['local']['commit'] != report['remote']['commit']:
        raise ValueError('host commit mismatch; deploy same release first')
    for key in ('ros_distro', 'rmw', 'domain', 'message_types'):
        if (report['local']['environment'][key] != report['remote']['environment'][key]
                or sam['environment'][key] != report['remote']['environment'][key]):
            raise ValueError(f'host ROS {key} mismatch')
    if report['local']['environment']['ros_distro'] != 'jazzy':
        raise ValueError('Jazzy required')
    report['checked_unix_s'] = time.time()
    return validate_clock_pair(report, config['network']['ptp_max_offset_us'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['probe', 'ptp', 'cameras'])
    parser.add_argument('--interface')
    parser.add_argument('--role', choices=['local', 'remote'], default='local')
    args = parser.parse_args()
    if args.action == 'cameras':
        print(json.dumps(camera_serials()))
    else:
        print(json.dumps(ptp_probe(args.interface, args.role) if args.action == 'ptp' else probe(args.interface, args.role)))

if __name__ == '__main__':
    main()
