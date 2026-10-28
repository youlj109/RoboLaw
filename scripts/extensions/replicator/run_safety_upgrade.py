#!/usr/bin/env python3
"""One entry point for resumable source-geometry risk screening and live status.

Runs CPU workers only. Native execution remains separately scheduled under the
existing GPU lock; this command never invents a baseline for missing tasks.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.safety_upgrade_progress import (
    REPORT, aggregate, atomic_json, stamp)


def python_command(script, *args):
    if ROOT == Path('/workspace'):
        return [str(ROOT/'.venv-arena/bin/python'), script, *map(str, args)]
    return ['bash', 'scripts/arena_python.sh', script, *map(str, args)]


def execute(command, log_path, timeout):
    started = time.monotonic()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open('w') as log:
        try:
            process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     timeout=timeout, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
            result = dict(returncode=process.returncode)
        except subprocess.TimeoutExpired:
            result = dict(returncode=None, error='worker_timeout', timeout_seconds=timeout)
    return dict(command=command, log_path=str(log_path.relative_to(ROOT)),
                wall_seconds=time.monotonic()-started, **result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers', type=int, choices=range(1, 5), default=2)
    parser.add_argument('--task', action='append', help='Full task ID or Replicator short ID; repeatable')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--refresh-only', action='store_true', help='Aggregate saved task evidence once')
    parser.add_argument('--watch', action='store_true', help='Keep refreshing completed task evidence every 3 seconds')
    parser.add_argument('--worker-timeout', type=int, default=3600)
    args = parser.parse_args()
    report_dir = ROOT/REPORT
    report_dir.mkdir(parents=True, exist_ok=True)
    with (report_dir/'supervisor.lock').open('a+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.refresh_only or args.watch:
            while True:
                result = aggregate(stage='monitoring' if args.watch else 'risk_screening')
                if not args.watch:
                    print(json.dumps(result['counts'], ensure_ascii=False))
                    return 0
                time.sleep(3)
        if args.worker_timeout < 1:
            parser.error('--worker-timeout must be positive')
        manifest = json.loads((ROOT/'configs/extensions/replicator/scene_variants/manifest.json').read_text())
        tasks = list(manifest['tasks'])
        selected = {x if x.startswith('kitchen_bench_replicator_') else
                    'kitchen_bench_replicator_'+x for x in (args.task or tasks)}
        if selected-set(tasks):
            parser.error('Unknown tasks: ' + ', '.join(sorted(selected-set(tasks))))
        tasks = [x for x in tasks if x in selected]
        # Each invocation has its own immutable command logs.
        run_id = time.strftime('%Y%m%dT%H%M%S') + f'_{os.getpid()}'
        logs = report_dir/'runs'/run_id
        aggregate(active_work=[dict(stage='baseline_inventory', description='核验真实成功基线和文件哈希')],
                  stage='baseline_inventory')
        inventory = execute(python_command('scripts/extensions/replicator/inventory_scene_risk_baselines.py',
                            '--output', str(REPORT/'baselines.json')), logs/'inventory.log', args.worker_timeout)
        atomic_json(logs/'inventory_command.json', inventory)
        if inventory['returncode'] != 0:
            aggregate(active_work=[], stage='inventory_needs_repair')
            print(json.dumps(inventory, ensure_ascii=False))
            return 2
        initialized = execute(python_command('scripts/extensions/replicator/screen_scene_risk_placements.py',
                              '--initialize-only'), logs/'initialize.log', args.worker_timeout)
        atomic_json(logs/'initialize_command.json', initialized)
        if initialized['returncode'] != 0:
            aggregate(active_work=[], stage='initialization_needs_repair')
            return 2
        completed = []
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            futures = {}
            for task in tasks:
                short = task.removeprefix('kitchen_bench_replicator_')
                command = python_command('scripts/extensions/replicator/screen_scene_risk_placements.py',
                                         '--task', task, '--skip-init')
                if args.resume:
                    command.append('--resume')
                futures[executor.submit(execute, command, logs/(short+'.log'), args.worker_timeout)] = task
            while futures:
                active = [dict(stage='risk_screening', task_id=task,
                          description='源几何摆位筛选' if future.running() else '等待CPU工作槽')
                          for future, task in futures.items()]
                aggregate(active_work=active, stage='risk_screening')
                done, _ = wait(futures, timeout=3, return_when=FIRST_COMPLETED)
                for future in done:
                    task = futures.pop(future)
                    result = dict(task_id=task, **future.result())
                    completed.append(result)
                    atomic_json(logs/(task.removeprefix('kitchen_bench_replicator_')+'_command.json'), result)
                    print(json.dumps(result, ensure_ascii=False), flush=True)
        summary = aggregate(active_work=[], stage='screening_finished_with_pending_validation')
        atomic_json(logs/'summary.json', dict(updated_at=stamp(), commands=completed,
                    counts=summary['counts'], checks=summary['checks'],
                    all_safety_scenes_accepted=summary['checks']['safe_execution_verified']==154))
        print(json.dumps(dict(counts=summary['counts'], checks=summary['checks']), ensure_ascii=False))
        return int(any(x['returncode'] != 0 for x in completed))*2


if __name__ == '__main__':
    raise SystemExit(main())
