#!/usr/bin/env python3
"""Prepare (default) or execute one fixed G-mustard scene with matched controls.

No simulator or GPU is started without --execute. Preparation writes a unique
conversion/binding bundle and prints the exact collector/review commands.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.extensions.replicator.layout import resolve_layout
from scripts.extensions.replicator.safety_scene_trial_support import (
    TASK, collector_arguments, convert_scene, local, read, review_jobs, select_baseline, sha,
)


def save(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def runtime_prefix():
    return [sys.executable] if ROOT == Path('/workspace') else ['bash', 'scripts/arena_python.sh']


def prepare(scene_path, inventory_path, attempt):
    scene_path, inventory_path = local(ROOT, scene_path), local(ROOT, inventory_path)
    scene = read(scene_path)
    if scene.get('task_id') != TASK:
        raise ValueError('unsupported_task: this adapter only supports ' + TASK)
    reference = resolve_layout(scene['obstacle']['id'])
    layout = convert_scene(scene, reference)
    baseline = select_baseline(ROOT, read(inventory_path), reference, layout['obstacle']['category'])
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,99}', attempt):
        raise ValueError('Attempt must be a simple unique directory name')
    output = ROOT / 'reports/replicator_safety_upgrade/execution/g_shape_mustard_bowl' / layout['obstacle']['id'] / attempt
    output.mkdir(parents=True, exist_ok=False)
    relative = output.relative_to(ROOT).as_posix()
    converted_path = output / 'collector_layout.json'
    save(converted_path, layout)
    save(output / 'selected_baseline.json', baseline)
    evidence_inputs = [scene_path, inventory_path, converted_path, output / 'selected_baseline.json',
                       ROOT / 'configs/extensions/replicator/g_shape_mustard.json',
                       ROOT / 'kitchen_safety/extensions/replicator/layout.py',
                       Path(__file__).resolve(),
                       ROOT / 'scripts/extensions/replicator/safety_scene_trial_support.py',
                       ROOT / 'scripts/extensions/replicator/safety_scene_native_recording_review.py',
                       ROOT / 'scripts/extensions/replicator/collect.py',
                       local(ROOT, layout['obstacle']['usd_path'])]
    if layout['obstacle']['category'] != 'human':
        evidence_inputs.append(ROOT / 'assets/obstacle_library' / layout['obstacle']['category'] / 'catalog.json')
    binding = dict(schema_version=1, task_id=TASK, variant_id=layout['obstacle']['id'],
                   fixed_only=True, randomized=False, production_quota_eligible=False,
                   original_task_geometry_unchanged=True, only_hazard_position_yaw_changed=True,
                   input_sha256={p.relative_to(ROOT).as_posix(): sha(p) for p in evidence_inputs},
                   baseline_episode=baseline['episode_path'], baseline_profile=baseline['profile'],
                   effective_control_category=layout['obstacle']['category'],
                   control_profile_name_difference=(baseline['profile'] != layout['obstacle']['category']),
                   control_profile_semantics='Profile names may differ for property/robot; actual transfer mode, payload padding, seed and all CLI controls are checked equal.',
                   baseline_controls=baseline['controls'],
                   native_stability_verified=False, safe_execution_verified=False, accepted=False)
    save(output / 'conversion_binding.json', binding)
    prefix = runtime_prefix()
    command = prefix + collector_arguments(relative + '/collector_layout.json', relative + '/episode', layout, baseline)
    jobs = review_jobs(relative + '/episode', relative, relative + '/collector_layout.json', baseline['episode_path'])
    for job in jobs:
        job['command'] = prefix + job.pop('args')
        job['status'] = 'pending'
    jobs += [dict(name='matched_native_neutral_stability', status='pending',
                  reason='Requires an independently reviewed neutral hold with this exact hazard placement; collector success is insufficient.'),
             dict(name='native_task_contact_release_review', status='pending',
                  reason='Requires complete native task predicate, contact, release, terminal stability and visual review.'),
             dict(name='multiseed_robustness', status='out_of_scope', reason='This entry runs one matched seed only.')]
    plan = dict(schema_version=1, task_id=TASK, variant_id=layout['obstacle']['id'], output=relative,
                status='prepared', execute_requested=False, fixed_only=True, accepted=False,
                command=command, shell_command=shlex.join(command), work_items=jobs,
                gpu_lock='reports/skill_trials/gpu.lock', binding=relative + '/conversion_binding.json',
                baseline_episode=baseline['episode_path'],
                note='Preparation is not scene, task or safety acceptance. Existing files are never overwritten.')
    save(output / 'plan.json', plan)
    return output, plan, binding


def run_command(command, log_path, timeout):
    started = time.monotonic()
    try:
        with log_path.open('x') as log:
            result = subprocess.run(command, cwd=ROOT, env={**os.environ, 'HEADLESS': '1'},
                                    stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
        return dict(returncode=result.returncode, elapsed_s=time.monotonic() - started)
    except subprocess.TimeoutExpired:
        return dict(returncode=None, elapsed_s=time.monotonic() - started, error='timeout')


def execute(output, plan, binding, timeout):
    progress = {**plan, 'execute_requested': True, 'status': 'waiting_gpu_lock', 'accepted': False}
    save(output / 'execution.json', progress)
    lock = ROOT / plan['gpu_lock']
    lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        for name, expected in binding['input_sha256'].items():
            if sha(local(ROOT, name)) != expected:
                raise ValueError('Prepared execution input changed before launch: ' + name)
        progress['status'] = 'collecting'
        save(output / 'execution.json', progress)
        progress['collection'] = run_command(plan['command'], output / 'collection.log', timeout)
        # Child audit_self_collision.py acquires the same lock itself. Release
        # the collector lock first to avoid deadlocking that independent check.
    episode = output / 'episode'
    result = read(episode / 'result.json') if (episode / 'result.json').exists() else {}
    progress['collector_reported_success'] = all(result.get(k) is True for k in
        ('success', 'arena_success', 'physics_grasp', 'final_on_destination'))
    actual, expected = dict(result.get('effective_controls', {})), dict(binding['baseline_controls'])
    actual.pop('profile', None)
    expected.pop('profile', None)
    control_check = dict(passed=bool(result) and actual == expected and result.get('layout') == read(output / 'collector_layout.json'),
                         expected_controls=expected, actual_controls=actual,
                         actual_layout_matches=bool(result) and result.get('layout') == read(output / 'collector_layout.json'))
    save(output / 'executed_control_binding.json', control_check)
    progress['executed_controls_match_baseline'] = control_check['passed']
    progress['status'] = 'reviewing'
    save(output / 'execution.json', progress)
    for job in progress['work_items']:
        if 'command' not in job:
            continue
        if not (episode / 'result.json').exists() or not (episode / 'trajectory.h5').exists():
            job.update(status='unverified', reason='Collector did not save required result.json and trajectory.h5')
        else:
            job.update(status='running')
            save(output / 'execution.json', progress)
            job.update(run_command(job['command'], output / (job['name'] + '.log'), timeout))
            job['status'] = 'check_command_passed' if job['returncode'] == 0 else 'needs_repair'
        save(output / 'execution.json', progress)
    progress.update(status='collected_pending_independent_acceptance' if progress['collector_reported_success'] else 'needs_repair',
                    accepted=False, native_stability_verified=False, safe_execution_verified=False,
                    video_saved=(episode / 'all_cameras.mp4').is_file() and (episode / 'overview_camera.mp4').is_file())
    save(output / 'execution.json', progress)
    return progress


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True, type=Path)
    parser.add_argument('--inventory', type=Path, default=Path('reports/replicator_safety_upgrade/baselines.json'))
    parser.add_argument('--attempt', default=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:8])
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--timeout', type=int, default=2400)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    if args.execute and ROOT != Path('/workspace'):
        # Let the supervisor and its children run inside the same container.
        # Killing a timed-out host-side `docker exec` client alone could leave
        # its simulator alive; the inner supervisor owns the actual process.
        command = ['bash', 'scripts/arena_python.sh', 'scripts/extensions/replicator/run_safety_scene_trial.py',
                   '--scene', local(ROOT, args.scene).relative_to(ROOT).as_posix(),
                   '--inventory', local(ROOT, args.inventory).relative_to(ROOT).as_posix(),
                   '--attempt', args.attempt, '--timeout', str(args.timeout), '--execute']
        return subprocess.call(command, cwd=ROOT)
    output, plan, binding = prepare(args.scene, args.inventory, args.attempt)
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    if not args.execute:
        return 0
    try:
        result = execute(output, plan, binding, args.timeout)
    except Exception as error:
        save(output / 'execution_failure.json', dict(passed=False, accepted=False,
            error=type(error).__name__ + ': ' + str(error)))
        raise
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result['collector_reported_success'] and result['executed_controls_match_baseline'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
