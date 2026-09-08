#!/usr/bin/env python3
"""Deploy a pushed release/SHA without overwriting either host's work."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
from two_host import (ROOT, atomic_json, checked_sync, execute, load_config,
                      local_command, remote_command, remote_python, ssh_command, tree_hashes)

VOCABULARY = 'orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt'
SOURCE_DIRS = ('orbslam_ws/src/ORB_SLAM3', 'orbslam_ws/src/orbslam3_ros2')


def git(*args):
    return execute(['git', '-C', str(ROOT), *args])


def resolve_pushed(ref):
    if ref.startswith('-'):
        raise ValueError('ref must be a tag or commit SHA')
    if git('status', '--porcelain'):
        raise ValueError('local working tree is dirty; commit the release before deployment')
    sha = git('rev-parse', '--verify', ref + '^{commit}')
    if git('rev-parse', 'HEAD') != sha:
        raise ValueError('local HEAD must equal requested release so both hosts run the same code')
    origin = git('remote', 'get-url', 'origin')
    if 'github.com' not in origin:
        raise ValueError('origin must be the GitHub monorepo')
    # Server checks object availability, including an older SHA not advertised as a tip.
    git('fetch', '--no-tags', '--no-write-fetch-head', 'origin', sha)
    if not all(c in '0123456789abcdef' for c in ref.lower()) or len(ref) != 40:
        advertised = git('ls-remote', '--tags', 'origin', 'refs/tags/' + ref, 'refs/tags/' + ref + '^{}')
        rows = dict(line.split('\t')[::-1] for line in advertised.splitlines())
        if rows.get('refs/tags/' + ref + '^{}', rows.get('refs/tags/' + ref)) != sha:
            raise ValueError('release tag is missing or differs on GitHub')
    return sha, origin


def source_fingerprint(sha):
    return hashlib.sha256('\n'.join(git('rev-parse', sha + ':' + p) for p in SOURCE_DIRS).encode()).hexdigest()


def deploy(config, ref):
    sha, origin = resolve_pushed(ref)
    root = config['ssh']['remote_root']
    # Probe a path without invoking git in an ancestor repository.
    exists = remote_python(config, 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).exists())', root) == 'True'
    previous = None
    if exists:
        top = execute(ssh_command(config, ['git', '-C', root, 'rev-parse', '--show-toplevel']))
        if top != root.rstrip('/'):
            raise ValueError('remote_root is not the repository root')
        if execute(ssh_command(config, ['git', '-C', root, 'status', '--porcelain'])):
            raise ValueError('remote working tree is dirty; refusing checkout')
        remote_origin = execute(ssh_command(config, ['git', '-C', root, 'remote', 'get-url', 'origin']))
        if remote_origin != origin:
            raise ValueError('remote origin differs from local origin')
        previous_json = remote_python(config,
            "import pathlib,sys; p=pathlib.Path(sys.argv[1])/'deployment.json'; print(p.read_text() if p.exists() else 'null')", root)
        previous = json.loads(previous_json)
    else:
        (ROOT / 'deployment.json').unlink(missing_ok=True)
        execute(ssh_command(config, ['git', 'clone', '--no-checkout', '--', origin, root]), timeout=600)
    (ROOT / 'deployment.json').unlink(missing_ok=True)
    execute(ssh_command(config, ['git', '-C', root, 'fetch', '--tags', 'origin', sha]), timeout=600)
    execute(ssh_command(config, ['git', '-C', root, 'checkout', '--detach', sha]), timeout=60)
    if execute(ssh_command(config, ['git', '-C', root, 'rev-parse', 'HEAD'])) != sha:
        raise RuntimeError('remote checkout SHA mismatch')
    # Invalidate a prior success before any fallible asset/build operation.
    remote_python(config, "import pathlib,sys; p=pathlib.Path(sys.argv[1])/'deployment.json'; p.unlink(missing_ok=True)", root)
    checksums = checked_sync(config, ROOT / VOCABULARY, str(Path(root) / VOCABULARY))
    fingerprint = source_fingerprint(sha)
    installed = remote_python(config, "import pathlib,sys; print((pathlib.Path(sys.argv[1])/'orbslam_ws/install_jazzy/setup.bash').is_file())", root) == 'True'
    built = not installed or not previous or previous.get('orb_source_sha256') != fingerprint
    if built:
        command = ['colcon', '--log-base', 'orbslam_ws/log_jazzy', 'build',
                   '--base-paths', 'orbslam_ws/src', '--build-base', 'orbslam_ws/build_jazzy',
                   '--install-base', 'orbslam_ws/install_jazzy', '--packages-select',
                   'orbslam3_core', 'orbslam3_ros2', '--executor', 'sequential',
                   '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_ORB_SLAM3_EXAMPLES=OFF',
                   '-DCMAKE_POLICY_VERSION_MINIMUM=3.5', '-DBUILD_TESTING=ON']
        subprocess.run(remote_command(config, command, source=False), check=True)
    for script in ('camera_publish.py', 'camera_extrinsic_localization.py'):
        subprocess.run(remote_command(config, ['python', 'integration/' + script, '--self-test']), check=True)
    subprocess.run(remote_command(config, ['python', 'integration/test_two_host.py']), check=True)
    subprocess.run(remote_command(config, ['python', 'integration/verify_orb_runtime.py']), check=True)
    remote = json.loads(execute(remote_command(config, ['python', 'integration/two_host.py', 'probe']), timeout=90))
    local = json.loads(execute(local_command(config, ['python', 'integration/two_host.py', 'probe']), timeout=90))
    for key in ('ros_distro', 'rmw', 'domain', 'message_types'):
        if remote['environment'][key] != local['environment'][key]:
            raise RuntimeError(f'ROS {key} differs between hosts')
    if local['environment']['ros_distro'] != 'jazzy' or local['commit'] != sha or remote['commit'] != sha:
        raise RuntimeError('deployment runtime is not the expected Jazzy release')
    manifest = dict(commit=sha, ref=ref, deployed_unix_s=time.time(),
                    built_unix_s=time.time() if built else previous.get('built_unix_s'),
                    rebuilt=built, orb_source_sha256=fingerprint,
                    asset_checksums={VOCABULARY: checksums}, local=local, remote=remote)
    # Publish identical success manifests only after both environments pass.
    remote_python(config,
        "import runpy,json,sys; m=runpy.run_path(sys.argv[1]); m['atomic_json'](sys.argv[2],json.loads(sys.argv[3]))",
        str(Path(root)/'integration/two_host.py'), str(Path(root)/'deployment.json'), json.dumps(manifest))
    atomic_json(ROOT / 'deployment.json', manifest)
    print(json.dumps(manifest, indent=2))
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--ref', required=True)
    args = parser.parse_args()
    try:
        deploy(load_config(args.config), args.ref)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.exit(1, f'deployment failed: {exc}\n')

if __name__ == '__main__':
    main()
