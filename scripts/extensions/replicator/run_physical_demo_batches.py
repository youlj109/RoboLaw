#!/usr/bin/env python3
"""Host-only coordinator: one fresh Kit process per pending task batch.

Reads an existing execution manifest without changing it. Only the execution
driver writes scene results. There is no initialization recovery, episode retry,
runtime-revision override, timeout, or process cleanup in this coordinator.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SOURCE = 'reports/replicator_trajectory_retry10_20261028'
DEFAULT_OUTPUT = 'exports/replicator_154_physical_demos_20261028'
TERMINAL = {'success', 'failed', 'error'}
SOURCE_FIELDS = ('scene_id', 'task_id', 'variant_id', 'result_path', 'result_sha256',
                 'config_path', 'config_sha256', 'trajectory_path', 'trajectory_sha256',
                 'source_scene_path', 'source_scene_sha256')


def repository_path(value):
    path = (ROOT / value).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Path must be inside this checkout: ' + str(path))
    return path


def read_manifest(path, source):
    raw = path.read_bytes()  # A missing manifest is an error; never initialize one.
    manifest = json.loads(raw)
    if manifest.get('kind') != 'physical_execution':
        raise ValueError('Expected an existing physical_execution manifest')
    if repository_path(manifest['source_run']) != source:
        raise ValueError('Manifest source_run differs from --source-run')
    rows = manifest['entries']
    if not rows or len({row['scene_id'] for row in rows}) != len(rows):
        raise ValueError('Manifest has empty or duplicate scene identities')
    for row in rows:
        status = row['execution_status']
        if status == 'running':
            raise ValueError('Refusing to start while a scene is running: ' + row['scene_id'])
        if status not in TERMINAL | {'pending'}:
            raise ValueError('Unknown execution status: ' + str(status))
        if status == 'pending' and (row.get('attempts') != 0 or row.get('executed') is not False):
            raise ValueError('Pending scene already has an attempt or uncertain execution: ' + row['scene_id'])
    return manifest, hashlib.sha256(raw).hexdigest()


def next_batch(manifest, limit=11):
    """Match the driver's pending[:N] selection without crossing a task boundary."""
    if limit < 1:
        raise ValueError('Batch scene limit must be positive')
    pending = [row for row in manifest['entries'] if row['execution_status'] == 'pending']
    if not pending:
        return []
    task = pending[0]['task_id']
    selected = []
    for row in pending:
        if row['task_id'] != task or len(selected) == limit:
            break
        selected.append(row)
    # This is precisely execute_planned_trajectories.py --resume --limit N.
    driver_selected = pending[:len(selected)]
    if driver_selected != selected or any(row['task_id'] != task for row in driver_selected):
        raise ValueError('Driver limit would select a different task batch')
    return selected


def driver_command(source, output, count, physics_threads=8, camera_overrides=None):
    if count < 1 or physics_threads < 1:
        raise ValueError('Scene count and physics threads must be positive')
    command = ['bash', str(ROOT / 'scripts/arena_python.sh'),
            'scripts/extensions/replicator/execute_planned_trajectories.py',
            '--device', 'cpu', '--physics-threads', str(physics_threads),
            '--source-run', str(source.relative_to(ROOT)),
            '--output', str(output.relative_to(ROOT)), '--resume', '--limit', str(count)]
    if camera_overrides is not None:
        command += ['--camera-overrides', str(repository_path(camera_overrides).relative_to(ROOT))]
    return command


def verify_batch(before, after, selected):
    ids = [row['scene_id'] for row in before['entries']]
    if [row['scene_id'] for row in after['entries']] != ids:
        raise ValueError('Manifest scene identities/order changed during batch')
    for key in ('kind', 'source_run', 'source_progress_sha256'):
        if before.get(key) != after.get(key):
            raise ValueError('Manifest source identity changed: ' + key)
    chosen = {row['scene_id'] for row in selected}
    for old, new in zip(before['entries'], after['entries']):
        sid = old['scene_id']
        if sid not in chosen:
            if old != new:
                raise ValueError('Unselected scene changed: ' + sid)
            continue
        if new['execution_status'] not in TERMINAL:
            raise ValueError('Selected scene did not become terminal: ' + sid)
        if new.get('attempts') != 1:
            raise ValueError('Selected scene does not retain exactly one attempt: ' + sid)
        if any(old.get(key) != new.get(key) for key in SOURCE_FIELDS):
            raise ValueError('Selected scene source binding changed: ' + sid)


def run_batches(source, output, *, batch_size=11, physics_threads=8, max_batches=0,
                dry_run=False, camera_overrides=None):
    manifest_path = output / 'manifest.json'
    manifest, digest = read_manifest(manifest_path, source)
    if dry_run:
        # Only inspect; do not create a lock, logs, result, or start a subprocess.
        rows = [dict(row) for row in manifest['entries']]
        batches = []
        while batch := next_batch({'entries': rows}, batch_size):
            batches.append(dict(task_id=batch[0]['task_id'], scenes=[r['scene_id'] for r in batch],
                command=driver_command(source, output, len(batch), physics_threads, camera_overrides)))
            for row in batch:
                row['execution_status'] = 'success'  # In-memory scheduling only.
            if max_batches and len(batches) >= max_batches:
                break
        print(json.dumps(dict(dry_run=True, manifest_sha256=digest, batches=batches), ensure_ascii=False, indent=2))
        return 0
    if not next_batch(manifest, batch_size):
        print('NO_PENDING_SCENES', flush=True)
        return 0
    # Prevent a second coordinator. The execution driver's GPU lock remains its
    # own responsibility; this lock never modifies manifests or scene evidence.
    with (output / '.physical_batch_coordinator.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError('Another physical batch coordinator holds this output lock') from error
        completed = 0
        while True:
            before, digest = read_manifest(manifest_path, source)
            selected = next_batch(before, batch_size)
            if not selected:
                print('ALL_PENDING_BATCHES_FINISHED', flush=True)
                return completed
            command = driver_command(source, output, len(selected), physics_threads, camera_overrides)
            log_dir = output / 'batch_logs'
            log_dir.mkdir(exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            task = re.sub(r'[^a-zA-Z0-9_.-]', '_', selected[0]['task_id'])
            log_path = log_dir / f'{stamp}_{task}.log'
            with log_path.open('x') as log:
                header = dict(task_id=selected[0]['task_id'], scenes=[r['scene_id'] for r in selected],
                              manifest_sha256=digest, command=command)
                log.write('COORDINATOR_BATCH ' + json.dumps(header, ensure_ascii=False) + '\n')
                log.flush()
                # Recheck just before spawning; never let --resume consume an
                # observed running row or quietly change the selected prefix.
                _, latest_digest = read_manifest(manifest_path, source)
                if latest_digest != digest:
                    raise ValueError('Manifest changed before subprocess launch; stop for review')
                print(f'BATCH_START task={task} scenes={len(selected)} log={log_path}', flush=True)
                child = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    code = child.wait()
                except KeyboardInterrupt:
                    print(f'COORDINATOR_INTERRUPTED child_pid={child.pid}; no process was killed. '
                          f'Review the running child and {log_path} before restarting.', file=sys.stderr, flush=True)
                    raise
            if code != 0:
                raise RuntimeError(f'Batch process exited {code}; stop without retry. Log: {log_path}')
            after, _ = read_manifest(manifest_path, source)
            verify_batch(before, after, selected)
            completed += 1
            print(f'BATCH_DONE task={task} scenes={len(selected)} log={log_path}', flush=True)
            if max_batches and completed >= max_batches:
                return completed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', default=DEFAULT_SOURCE)
    parser.add_argument('--output', default=DEFAULT_OUTPUT)
    parser.add_argument('--batch-size', type=int, default=11, help='Maximum scenes in a process; never crosses task boundary')
    parser.add_argument('--physics-threads', type=int, default=8)
    parser.add_argument('--camera-overrides', help='Forward an explicit camera profile; runtime revision approval is never automatic')
    parser.add_argument('--max-batches', type=int, default=0, help='0 runs all remaining task batches')
    parser.add_argument('--dry-run', action='store_true', help='Host-only inspection; creates no files and starts no process')
    args = parser.parse_args()
    if args.batch_size < 1 or args.physics_threads < 1 or args.max_batches < 0:
        parser.error('Batch size/threads must be positive and max-batches nonnegative')
    try:
        run_batches(repository_path(args.source_run), repository_path(args.output),
                    batch_size=args.batch_size, physics_threads=args.physics_threads,
                    max_batches=args.max_batches, dry_run=args.dry_run,
                    camera_overrides=repository_path(args.camera_overrides) if args.camera_overrides else None)
    except (ValueError, RuntimeError, OSError, KeyError) as error:
        print('BATCH_COORDINATOR_STOP: ' + str(error), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
