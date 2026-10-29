#!/usr/bin/env python3
"""Run the fixed failure-retry cohort once, in fresh per-task Kit processes.

Only the execution driver writes results. Completed scenes are never selected;
an interrupted running row requires review instead of an automatic replay.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PLAN = 'configs/extensions/replicator/physical_retry_round1_20261028.json'
DEFAULT_OUTPUT = 'exports/replicator_physical_retry_round1_20261028'
TERMINAL = {'success', 'failed', 'error', 'planning_failed'}


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def local(value):
    path = (ROOT / value).resolve()
    path.relative_to(ROOT)
    return path


def pending_task(manifest):
    rows = manifest['entries']
    version = manifest.get('schema_version', 1)
    if version not in (1, 2) or version == 2 and manifest.get('retry_round') not in (2, 3, 4, 5):
        raise ValueError('Unsupported retry cohort schema/round')
    expected = {2: 48, 3: 29, 4: 6, 5: 5}[manifest['retry_round']] if version == 2 else 84
    if len(rows) != expected or len({row['scene_id'] for row in rows}) != expected:
        raise ValueError(f'Expected exactly the fixed {expected}-scene cohort')
    for row in rows:
        if row['execution_status'] == 'running':
            raise ValueError('An episode is still running or interrupted: '+row['scene_id'])
        if row['execution_status'] not in TERMINAL | {'pending'}:
            raise ValueError('Unknown execution status')
        if row['execution_status'] == 'pending' and (row.get('attempts') != 0 or row.get('executed') is not False):
            raise ValueError('Pending row already consumed or has uncertain execution')
        if row['execution_status'] == 'planning_failed' and (version != 2 or row.get('attempts') != 0
                or row.get('executed') is not False or row.get('planning_status') != 'failed'):
            raise ValueError('Planning failure cannot consume a physical attempt')
    pending = [row for row in rows if row['execution_status'] == 'pending']
    if not pending:
        return []
    return [row for row in pending if row['task_id'] == pending[0]['task_id']]


def command(plan, output, task):
    return ['bash', str(ROOT/'scripts/arena_python.sh'),
            'scripts/extensions/replicator/execute_failure_retry.py',
            '--plan', str(plan.relative_to(ROOT)), '--output', str(output.relative_to(ROOT)),
            '--resume', '--task', task]


def verify(before, after, chosen):
    old = {row['scene_id']:row for row in before['entries']}
    new = {row['scene_id']:row for row in after['entries']}
    if list(old) != list(new):
        raise ValueError('Scene cohort or order changed')
    selected = {row['scene_id'] for row in chosen}
    for sid, row in old.items():
        if sid not in selected:
            if row != new[sid]:
                raise ValueError('Unselected scene changed: '+sid)
            continue
        updated = new[sid]
        if updated['execution_status'] not in {'success','failed','error'} or updated.get('attempts') != 1:
            raise ValueError('Selected scene did not finish its single allowed attempt: '+sid)
        for key in ('task_id', 'variant_id', 'config_path', 'config_sha256', 'trajectory_path',
                    'trajectory_sha256', 'source_scene_path', 'source_scene_sha256',
                    'source_trajectory_path', 'source_trajectory_sha256', 'physical_attempt_number',
                    'candidate_planning_result','previous_retry_result','execution_overrides','retry_schema_version',
                    'planning_status','prior_physical_attempts'):
            if row.get(key) != updated.get(key):
                raise ValueError('Source/attempt identity changed: '+sid+' '+key)


def run(plan, output, max_batches=0, dry_run=False):
    path = output/'manifest.json'
    manifest = read(path)
    plan_hash = digest(plan)
    if manifest.get('retry_plan_sha256') != plan_hash or local(manifest['retry_plan_path']) != plan:
        raise ValueError('Batch manifest belongs to a different frozen plan')
    tasks = []
    draft = json.loads(json.dumps(manifest))
    while chosen := pending_task(draft):
        task = chosen[0]['task_id']
        tasks.append(dict(task_id=task, scenes=[row['scene_id'] for row in chosen],
                          command=command(plan, output, task)))
        for row in chosen:
            row['execution_status'] = 'success'  # In-memory schedule only.
        if max_batches and len(tasks) >= max_batches:
            break
    if dry_run:
        print(json.dumps(dict(dry_run=True, plan_sha256=plan_hash, batches=tasks), ensure_ascii=False, indent=2))
        return
    with (output/'.retry_coordinator.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for batch in tasks:
            if digest(plan) != plan_hash:
                raise ValueError('Plan changed during batch execution')
            before = read(path)
            chosen = pending_task(before)
            if [row['scene_id'] for row in chosen] != batch['scenes']:
                raise ValueError('Pending selection changed before launch')
            logs = output/'batch_logs'; logs.mkdir(exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            log_path = logs/(stamp+'_'+batch['task_id']+'.log')
            with log_path.open('x') as log:
                log.write(json.dumps(dict(**batch, plan_sha256=plan_hash), ensure_ascii=False)+'\n'); log.flush()
                print('RETRY_BATCH_START', batch['task_id'], len(chosen), str(log_path), flush=True)
                child = subprocess.Popen(batch['command'], cwd=ROOT, stdin=subprocess.DEVNULL,
                                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    code = child.wait()
                except KeyboardInterrupt:
                    print(f'Coordinator interrupted; child {child.pid} not stopped. Inspect its log before resuming.', flush=True)
                    raise
            if code:
                raise RuntimeError(f'Execution driver exited {code}; no automatic retry: {log_path}')
            if digest(plan) != plan_hash:
                raise ValueError('Plan changed during execution')
            verify(before, read(path), chosen)
            print('RETRY_BATCH_DONE', batch['task_id'], len(chosen), flush=True)
        print('RETRY_COORDINATOR_FINISHED', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', default=DEFAULT_PLAN)
    parser.add_argument('--output', default=DEFAULT_OUTPUT)
    parser.add_argument('--max-batches', type=int, default=0)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.max_batches < 0:
        parser.error('max-batches must be nonnegative')
    run(local(args.plan), local(args.output), args.max_batches, args.dry_run)


if __name__ == '__main__':
    main()
