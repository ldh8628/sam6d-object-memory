#!/usr/bin/env python3
"""Run one bounded Codex task over SSH and retain its results and session ID."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time
import uuid

from two_host import ROOT, atomic_json, load_config, ssh_command

WORKER_INSTRUCTIONS = """You are the SLAM notebook worker controlled by the SAM notebook Codex.
Execute only the task below. Source edits, commits, pushes and deployment decisions
belong to the SAM controller. Do not change tracked source or resume other sessions.
Preserve existing processes unless this task explicitly requests changing them.
Do not output credentials. Never use password prompts; report missing privileges.
Return observations with commands/evidence, blockers, and any changes you made.
Distinguish task completion from unverified hardware acceptance. Do not delegate.

TASK:
"""


def remote_argv(config, seconds, sandbox, model, session=None):
    root = config['ssh']['remote_root']
    lock_dir = str(Path(root) / 'output/remote_codex')
    codex = ['codex', 'exec', '-C', root, '-s', sandbox, '-m', model,
             '-c', 'model_reasoning_effort="high"', '-c', 'approval_policy="never"',
             '--json']
    codex += ['resume', str(uuid.UUID(session)), '-'] if session else ['-']
    command = (shlex.join(['mkdir', '-p', '--', lock_dir]) + ' && ' +
               shlex.join(['flock', '-n', '-E', '75', str(Path(lock_dir) / 'worker.lock'),
                           'timeout', '--signal=TERM', '--kill-after=10s', str(seconds), *codex]))
    # Login shell discovers ~/.local/bin/codex; prompt is stdin, never shell code.
    return ssh_command(config, ['bash', '-lc', command])


def parse_events(path):
    result = dict(session_id=None, completed=False, final='', errors=[])
    messages = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        event = json.loads(line)
        kind = event.get('type')
        if kind == 'thread.started':
            result['session_id'] = str(uuid.UUID(event['thread_id']))
        elif kind == 'turn.completed':
            result['completed'] = True
        elif kind in ('turn.failed', 'error'):
            result['errors'].append(event)
        elif kind == 'item.completed':
            item = event.get('item', {})
            if item.get('type') == 'agent_message':
                messages.append(item.get('text', ''))
    result['final'] = '\n\n'.join(messages)
    return result


def run(config, prompt, seconds=300, sandbox='read-only', model='gpt-6-astra', resume=None):
    if not prompt.strip():
        raise ValueError('prompt must not be empty')
    if not 10 <= seconds <= 3600:
        raise ValueError('timeout must be 10..3600 seconds')
    session = None
    if resume:
        previous = json.loads(Path(resume).read_text())
        if (previous.get('runner') != 'remote_codex_v1' or
                previous.get('target') != config['ssh']['target'] or
                previous.get('remote_root') != config['ssh']['remote_root']):
            raise ValueError('resume report is not from this worker target/root')
        session = str(uuid.UUID(previous['session_id']))
    directory = ROOT / 'output/remote_codex' / uuid.uuid4().hex
    directory.mkdir(parents=True, mode=0o700)
    report = dict(runner='remote_codex_v1', target=config['ssh']['target'],
                  remote_root=config['ssh']['remote_root'], model=model,
                  sandbox=sandbox, started_unix_s=time.time(), status='running',
                  session_id=session, timeout_s=seconds)
    atomic_json(directory / 'request.json', dict(report, prompt=prompt))
    print(f'Remote task artifacts: {directory}', flush=True)
    try:
        with (directory / 'events.jsonl').open('w') as stdout, (directory / 'stderr.log').open('w') as stderr:
            process = subprocess.run(remote_argv(config, seconds, sandbox, model, session),
                                     input=WORKER_INSTRUCTIONS + prompt, text=True,
                                     stdout=stdout, stderr=stderr, timeout=seconds + 30)
        parsed = parse_events(directory / 'events.jsonl')
        report.update(exit_code=process.returncode, session_id=parsed['session_id'] or session,
                      errors=parsed['errors'])
        (directory / 'final.md').write_text(parsed['final'] + '\n')
        if process.returncode or not parsed['completed'] or parsed['errors'] or not parsed['final'] or not report['session_id']:
            raise RuntimeError('remote turn did not complete; inspect events.jsonl and stderr.log')
        report['status'] = 'completed'
    except KeyboardInterrupt:
        report.update(status='interrupted', error='local call interrupted; remote timeout remains in effect')
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
        report.update(status='failed', error=str(exc))
    finally:
        report['finished_unix_s'] = time.time()
        atomic_json(directory / 'report.json', report)
    print(json.dumps(dict(report, artifact_dir=str(directory)), indent=2), flush=True)
    return report, directory


def self_test():
    import tempfile
    from unittest.mock import patch
    cfg = {'ssh': {'target': 'test-host', 'remote_root': "/tmp/project ' $(false)", 'port': 10022}}
    argv = remote_argv(cfg, 30, 'read-only', 'gpt-6-astra')
    shell = shlex.split(argv[-1])
    assert shell[:2] == ['bash', '-lc']
    tokens = shlex.split(shell[2])
    assert tokens[tokens.index('-C') + 1] == cfg['ssh']['remote_root']
    assert '$(false)' not in tokens and tokens[-1] == '-'
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / 'events.jsonl'
        sid = str(uuid.uuid4())
        events = [{'type': 'thread.started', 'thread_id': sid},
                  {'type': 'item.completed', 'item': {'type': 'agent_message', 'text': 'observed'}},
                  {'type': 'turn.completed'}]
        p.write_text('\n'.join(map(json.dumps, events)))
        parsed = parse_events(p)
        assert parsed['session_id'] == sid and parsed['completed'] and parsed['final'] == 'observed'
        p.write_text(json.dumps({'type': 'turn.failed', 'error': {'message': 'failed'}}))
        parsed = parse_events(p)
        assert parsed['errors'] and not parsed['completed']
        for failure in (subprocess.CompletedProcess([], 75), subprocess.TimeoutExpired('ssh', 60)):
            with patch.dict(run.__globals__, ROOT=Path(d)), patch('subprocess.run') as launch:
                if isinstance(failure, Exception):
                    launch.side_effect = failure
                else:
                    launch.return_value = failure
                report, directory = run(cfg, 'read-only test', seconds=30)
                assert report['status'] == 'failed'
                assert json.loads((directory / 'report.json').read_text())['status'] == 'failed'
    print('remote_codex self-test passed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--prompt-file', type=Path)
    parser.add_argument('--resume-report', type=Path)
    parser.add_argument('--timeout', type=int, default=300)
    parser.add_argument('--sandbox', choices=['read-only', 'workspace-write'], default='read-only')
    parser.add_argument('--model', default='gpt-6-astra')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.config is None or args.prompt_file is None:
        parser.error('--config and --prompt-file are required')
    try:
        report, _ = run(load_config(args.config), args.prompt_file.read_text(),
                        args.timeout, args.sandbox, args.model, args.resume_report)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'remote Codex setup failed: {exc}\n')
    if report['status'] != 'completed':
        parser.exit(1, 'remote Codex task failed; see saved artifacts\n')


if __name__ == '__main__':
    main()
