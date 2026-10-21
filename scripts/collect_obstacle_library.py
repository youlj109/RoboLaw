"""Sequentially verify complete grasp/place trajectories for library obstacles.

Run using Arena Python; each collector has a separate Kit process by default::

    bash scripts/arena_python.sh scripts/collect_obstacle_library.py \
        --video --output datasets/kitchen_auto/obstacle_library_v1

This is a fixed-layout feasibility run, not a policy success-rate evaluation.
Failed attempts and their logs are retained. Use --resume to continue a batch.
Randomized batches accept --randomization-config and --episodes-per-id. Their
readable saved configuration must match on resume; the success target may grow.
Legacy fixed-layout results remain readable. Use a new output directory when
changing geometry, output mode, or collector options.
With --persistent-kit, collection reuses one worker within this batch; data
validation, clearance audits and preview export remain CPU subprocesses.
"""
import argparse
from contextlib import ExitStack
from collections import Counter
import csv
from datetime import datetime, timezone
import errno
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.obstacle_library import load_assets
from kitchen_safety.collection_presets import explicit_collector_overrides

STORAGE_ERROR_EXIT_CODE = 74
WORKER_UNAVAILABLE_EXIT_CODE = 75
STORAGE_ERRNOS = (errno.ENOSPC, errno.EDQUOT)
STORAGE_ERROR_TEXT = re.compile(
    r'\b(?:ENOSPC|EDQUOT)\b|\[Errno (?:28|122)\]|\berrno\s*=\s*(?:28|122)\b|'
    r'no space left on device|disk quota exceeded', re.IGNORECASE)
RUNTIME_DEFAULTS = {'device': 'cuda:0', 'physics_threads': None, 'torch_threads': None,
                    'execution_checks': 'online', 'camera_owner': 'environment', 'cpu_rgb_copy': False,
                    'camera_device': None, 'reuse_planner': False, 'persistent_kit': False,
                    'background_rgb': False, 'execution_backend': 'single',
                    'num_envs': 4, 'validation_workers': 4, 'pipeline': False,
                    'episode_length_s': 120.0}


def add_runtime_arguments(parser):
    parser.add_argument('--observer-config', help='Enable read-only safety evaluation (vectorized backend)')
    parser.add_argument('--run-id', help='New safety observer batch identity')
    parser.add_argument('--episode-length-s', type=float, default=120.0,
                        help='Arena task timeout in simulation seconds, including initialization; default 120')
    parser.add_argument('--execution-backend', choices=['single', 'vectorized'], default='single',
                        help='Optional same-ID batched physics plus recorded-state RGB rendering')
    parser.add_argument('--num-envs', type=int, default=4, help='Maximum environments per vectorized physics batch')
    parser.add_argument('--validation-workers', type=int, default=4,
                        help='CPU workers for vectorized physics-state validation')
    parser.add_argument('--pipeline', action='store_true',
                        help='Overlap vectorized physics with persistent RGB/verification; at most two batches in flight')
    parser.add_argument('--device', default=RUNTIME_DEFAULTS['device'], help='Collector simulation device; default cuda:0')
    parser.add_argument('--physics-threads', type=int, default=None,
                        help='Forward PhysX worker count; 0 runs on the main thread')
    parser.add_argument('--torch-threads', type=int, default=None, help='Forward PyTorch CPU worker count')
    parser.add_argument('--execution-checks', choices=['online', 'deferred'], default='online',
                        help='Run the same execution geometry checks online or after recording physics states')
    parser.add_argument('--camera-owner', choices=['environment', 'writer'], default='environment')
    parser.add_argument('--cpu-rgb-copy', action='store_true', help='Forward CPU RGB copy to the collector')
    parser.add_argument('--camera-device', default=None, help='Forward camera output device to the collector')
    parser.add_argument('--reuse-planner', action='store_true', help='Reuse cuRobo when updating dynamic hazard boxes')
    parser.add_argument('--background-rgb', action='store_true',
                        help='Forward bounded background RGB encoding/writes; use --record-rgb without --video')
    parser.add_argument('--persistent-kit', action='store_true',
                        help='Reuse one Kit worker for collection within this batch; CPU verification stays separate')


def runtime_configuration(args):
    """Omit all legacy defaults, so unchanged saved configurations still compare equal."""
    episode_length_s = getattr(args, 'episode_length_s', 120.0)
    if not math.isfinite(episode_length_s) or episode_length_s <= 0:
        raise ValueError('--episode-length-s must be finite and positive')
    runtime = {name: getattr(args, name, default) for name, default in RUNTIME_DEFAULTS.items()
               if getattr(args, name, default) != default}
    if getattr(args, 'execution_backend', 'single') == 'vectorized':
        runtime.update(num_envs=args.num_envs, validation_workers=args.validation_workers)
    return runtime


def runtime_command_args(runtime, *, collector=False):
    command = []
    for name, value in runtime.items():
        if collector and name in ('persistent_kit', 'execution_backend', 'num_envs', 'validation_workers', 'pipeline'):
            continue  # This option selects the batch runner, not a collector CLI argument.
        command.append('--' + name.replace('_', '-'))
        if not isinstance(value, bool):
            command.append(str(value))
    return command


def storage_error_message(error):
    if isinstance(error, OSError) and error.errno in STORAGE_ERRNOS:
        return f'{type(error).__name__}: {error}'
    if isinstance(error, str) and STORAGE_ERROR_TEXT.search(error):
        return error.strip()[:2000]
    return None


def report_storage_error(error):
    try:
        print(f'storage_error: {error}', file=sys.stderr, flush=True)
    except OSError:
        # stderr may itself be a log on the full filesystem. Preserve exit 74.
        pass


def now():
    return datetime.now(timezone.utc).isoformat()


def read_json(path, default=None):
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as error:
        return {'read_error': f'{type(error).__name__}: {error}'}


def write_json(path, value):
    # A normal atomic file replacement keeps progress readable after interruption.
    temporary = path.with_suffix(path.suffix + f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
    temporary.replace(path)


def run_command(command, log_path):
    started = now()
    begin = time.monotonic()
    try:
        with log_path.open('x') as log:
            code = subprocess.run(command, cwd=ROOT, stdout=log,
                                  stderr=subprocess.STDOUT).returncode
        execution = {'exit_code': code, 'started_at': started,
                     'elapsed_seconds': round(time.monotonic() - begin, 3)}
        if code != 0:
            with log_path.open(errors='replace') as log:
                for line in log:
                    evidence = storage_error_message(line)
                    if evidence:
                        execution['storage_error'] = evidence
                        break
        return execution
    except (OSError, subprocess.SubprocessError) as error:
        execution = {'exit_code': None, 'started_at': started,
                     'elapsed_seconds': round(time.monotonic() - begin, 3),
                     'error': f'{type(error).__name__}: {error}'}
        if storage_error_message(error):
            execution['storage_error'] = storage_error_message(error)
        return execution


def failed_phase(result, destination):
    if result.get('success'):
        return None
    if result.get('failure_phase'):
        return result['failure_phase']
    contact_check = result.get('body_grasp_carried_region', {})
    if contact_check.get('passed') is False and contact_check.get('outside_body_contacts'):
        return contact_check['outside_body_contacts'][0]['phase']
    # The last saved phase describes failures after planning and during execution.
    # Planning can fail before it emits a state, so prefer the error's phase name.
    error = result.get('error', '')
    for phase in ('pregrasp', 'approach', 'close', 'lift', 'transfer_front',
                  'transfer', 'above_bowl', 'lower', 'release', 'retreat', 'settle'):
        if f'{phase}:' in error or f'{phase} ' in error:
            return phase
    try:
        import h5py
        with h5py.File(destination / 'trajectory.h5') as trajectory:
            if len(trajectory['phase']):
                return str(trajectory['phase'].asstr()[-1])
    except (ImportError, OSError, KeyError):
        pass
    return 'startup' if not result.get('steps') else 'unknown'


def normalize_failure_kind(kind):
    return {'planning_failure': 'planning_failed', 'physical_failure': 'physical_failed',
            'data_validation_failure': 'data_validation_failed',
            'clearance_audit_failure': 'clearance_audit_failed',
            'startup_failure': 'startup_failed'}.get(kind, kind)


def failure_kind(result, execution_success, data_success, audit_pass, eligible=True):
    """Keep physical, planning, output, and benchmark-eligibility failures distinct."""
    declared = result.get('failure_kind') or result.get('failure_classification')
    if not execution_success:
        if declared:
            return normalize_failure_kind(declared)
        error = str(result.get('error', '')).lower()
        if any(word in error for word in ('sampling_invalid', 'initial intersection', 'initial penetration')):
            return 'sampling_invalid'
        if any(word in error for word in ('planning failed', 'plan failed', 'no collision-free', 'ik failed', 'no valid grasp')):
            return 'planning_failed'
        return 'physical_failed' if result.get('steps') else 'startup_failed'
    # Once physics succeeded, independent output/audit failures take priority
    # over an eligibility note declared by the collector. All reasons remain.
    if not data_success:
        return 'data_validation_failed'
    if not audit_pass:
        return 'clearance_audit_failed'
    if not eligible:
        return 'obstruction_unconfirmed'
    return None


def augmentation_eligibility(obstacle_id, result, *, randomized):
    """Apply the scene eligibility rule without changing raw geometry evidence."""
    # Explicit user instruction, 2026-10-19: steel clamp is accepted as a
    # side-risk obstacle. Only its direct-path obstruction requirement changes;
    # physical success, RGB validation and independent safety audit still apply.
    if randomized and obstacle_id == 'robot_steel_clamp':
        return True, 'user-approved-side-risk'
    return not randomized or result.get('augmentation_eligible') is True, None


def archive_unfinished_check(destination, name, include_execution=False):
    """Retain a check's partial outputs when no execution checkpoint exists."""
    sources = [destination / f'{name}.{suffix}' for suffix in ('log', 'json')]
    if include_execution:
        sources.append(destination / f'{name}_execution.json')
    sources = [path for path in sources if path.exists()]
    if not sources:
        return []
    index = 0
    while any(source.with_name(f'{source.stem}.interrupted_{index:03d}{source.suffix}').exists()
              for source in sources):
        index += 1
    archived = []
    for source in sources:
        target = source.with_name(f'{source.stem}.interrupted_{index:03d}{source.suffix}')
        source.rename(target)
        archived.append(target.name)
    return archived


def verify_attempt(destination, entry, seed, video, execution, *, record_rgb=False, randomized=False,
                   render_root=None, render_execution=None):
    """Reuse completed checks; archive partial output before rerunning a missing one."""
    result = read_json(destination / 'result.json', {})
    execution_success = (execution.get('exit_code') == 0 and result.get('success') is True
                         and not execution.get('shared_batch_error'))
    render_root = Path(render_root) if render_root is not None else None
    image_root = render_root or destination
    render_success = render_root is None or (render_execution or {}).get('status') == 'completed'
    storage_error = execution.get('storage_error')
    storage_phase = 'collection' if storage_error else None
    if not storage_error and (render_execution or {}).get('storage_error'):
        storage_error = render_execution['storage_error']
        storage_phase = 'render'
    checks = {}
    for name, script in [('validation', 'validate_collection.py'),
                         ('audit', 'audit_collection_clearance.py')]:
        checkpoint = destination / f'{name}_execution.json'
        saved = read_json(checkpoint)
        archived = []
        if saved is not None and saved.get('storage_error') and not storage_error:
            archived = archive_unfinished_check(destination, name, include_execution=True)
            saved = None
        if storage_error:
            checks[name] = {'exit_code': None, 'skipped_reason': 'Stopped after storage_error'}
        elif saved is not None:
            checks[name] = saved
        elif (execution_success or (name == 'validation' and (destination / 'safety_observer_binding.json').exists() and result.get('steps', 0) > 0)) and (name != 'validation' or render_success):
            archived += archive_unfinished_check(destination, name)
            command = [sys.executable, str(ROOT / 'scripts' / script), str(destination),
                       '--report', str(destination / f'{name}.json')]
            if name == 'validation':
                if render_root is not None:
                    command.extend(['--render-output', str(render_root)])
                if video:
                    command.append('--require-previews')
                elif record_rgb:
                    command.append('--require-rgb')
            checks[name] = run_command(command, destination / f'{name}.log')
            if archived:
                checks[name]['archived_interrupted_outputs'] = archived
            write_json(checkpoint, checks[name])
        else:
            checks[name] = {'exit_code': None,
                            'skipped_reason': 'Physical execution or required rendering did not complete successfully'}
        if checks[name].get('storage_error'):
            storage_error = checks[name]['storage_error']
            storage_phase = name

    validation = read_json(destination / 'validation.json')
    audit = read_json(destination / 'audit.json', {})
    data_success = (render_success and checks['validation'].get('exit_code') == 0
                    and isinstance(validation, list) and len(validation) == 1
                    and validation[0].get('passed', True) is not False)
    missing_video_files = []
    if video:
        expected = ['images.h5', 'external_camera.mp4', 'external_camera_2.mp4',
                    'wrist_camera.mp4', 'all_cameras.mp4']
        missing_video_files = [name for name in expected if not (image_root / name).is_file()]
        data_success = data_success and not missing_video_files
    elif record_rgb:
        missing_video_files = [name for name in ('images.h5', 'cameras.json')
                               if not (image_root / name).is_file()]
        data_success = data_success and not missing_video_files
    audit_pass = checks['audit'].get('exit_code') == 0 and audit.get('passed') is True
    audited_episodes = audit.get('episodes', [])
    audited = audited_episodes[0] if audited_episodes else {}
    plans = result.get('plans', [])
    raw_eligible = not randomized or result.get('augmentation_eligible') is True
    eligible, eligibility_reason = augmentation_eligibility(entry['id'], result, randomized=randomized)
    spec = read_json(destination / 'episode_spec.json', {})
    record = {
        'obstacle_id': entry['id'], 'variant': entry['category'], 'seed': seed,
        'completed_at': now(), 'execution': execution, 'checks': checks,
        'execution_success': bool(execution_success), 'data_success': bool(data_success),
        'audit_pass': bool(audit_pass),
        'passed': bool(execution_success and data_success and audit_pass and eligible and not storage_error),
        'augmentation_eligible': bool(raw_eligible), 'randomized': randomized,
        'obstruction_probe': result.get('obstruction_probe'),
        'failure_kind': failure_kind(result, execution_success, data_success, audit_pass, eligible),
        'storage_error': storage_error,
        'failure_phase': failed_phase(result, destination),
        'error': result.get('error') or result.get('read_error') or execution.get('error')
                 or (render_execution or {}).get('error'),
        'frames': result.get('steps'),
        'physics_clearance_checks': result.get('physics_clearance_checks'),
        'recorded_minimum_hazard_clearance_m': result.get('minimum_hazard_clearance_m'),
        'audited_minimum_clearance_m': audited.get('minimum_clearance_m'),
        'audit_states_checked': audited.get('states_checked'),
        'audit_failed_samples': audited.get('failed_sample_count'),
        'audit_error': audited.get('error') or audit.get('read_error'),
        'plan_methods': [{key: plan[key] for key in
                         ('phase', 'backend', 'method', 'solver', 'samples', 'planning_seconds')
                         if key in plan} for plan in plans],
        'video_requested': video, 'rgb_requested': bool(video or record_rgb),
        'missing_video_files': missing_video_files,
        'seeds': spec.get('seeds', {'planner': seed}),
        'timings_seconds': result.get('timings_seconds', result.get('timings', {})),
        'sampled_layout': spec.get('layout'), 'appearance': spec.get('appearance'),
        'appearance_applied': (read_json(render_root / 'render_report.json', {}).get('appearance')
                               if render_root is not None else result.get('appearance')),
        'settled_poses': result.get('settled_poses', result.get('actual_settled_poses')),
        'elapsed_seconds': round(sum(item.get('elapsed_seconds', 0)
                                     for item in [execution, *checks.values()]), 3),
    }
    if eligibility_reason is not None:
        record.update(effective_augmentation_eligible=bool(eligible),
                      eligibility_reason=eligibility_reason, direct_path_required=False)
    if render_root is not None:
        record.update(render_output=str(render_root), render_execution=render_execution,
                      acceptance_status='accepted' if record['passed'] else 'rejected',
                      production_quota_eligible=record['passed'])
    if execution.get('shared_batch_error'):
        record['shared_batch_error'] = execution['shared_batch_error']
    if storage_error:
        record.update(failure_kind='storage_error', storage_error=storage_error,
                      failure_phase=storage_phase)
    elif execution.get('worker_unavailable') or (render_execution or {}).get('worker_unavailable'):
        record.update(worker_unavailable=True, failure_kind='worker_unavailable',
                      failure_phase='render' if (render_execution or {}).get('worker_unavailable') else 'collection')
    elif render_root is not None and execution_success and not render_success:
        record['failure_phase'] = 'render'
    if not record['passed']:
        record['failure_reasons'] = []
        if storage_error:
            record['failure_reasons'].append(storage_error)
        if not execution_success:
            record['failure_reasons'].append(record['error'] or
                f"Physical execution failed or its exit status is unknown: {execution.get('exit_code')}")
            if execution.get('shared_batch_error') and execution['shared_batch_error'] != record['error']:
                record['failure_reasons'].append(execution['shared_batch_error'])
        if execution_success and not data_success:
            record['failure_reasons'].append('Data validation failed; see validation.log and missing_video_files')
        if execution_success and not audit_pass:
            record['failure_reasons'].append('Clearance audit failed; see audit.json and audit.log')
        if randomized and not eligible:
            record['failure_reasons'].append('Obstacle did not satisfy augmentation eligibility; see obstruction_probe')
    if (destination / 'safety_observer_binding.json').exists():
        from kitchen_safety.risk_reporting import result_axes
        record['result_axes'] = result_axes(destination, record, render_root)
    write_json(destination / 'verification.json', record)
    return record


def configuration_for(args, collector_overrides, randomization_config):
    """Normal JSON values, deliberately excluding extendable counts and subsets."""
    settings = read_json(ROOT / 'configs/reproduction_presets.json', {}).get('nonhuman10', {})
    configuration = {'seed': args.seed, 'randomization': randomization_config,
            'video': args.video, 'record_rgb': bool(args.record_rgb or args.video),
            'collector_overrides': collector_overrides,
            'collector_defaults': settings.get('collector', {}),
            'collector_by_id': settings.get('collector_by_id', {})}
    runtime = runtime_configuration(args)
    if runtime:
        configuration['runtime'] = runtime
    if getattr(args, 'observer_config', None):
        from kitchen_safety.risk_profile_loader import load_config
        configuration['safety_observer'] = load_config(args.observer_config)
        configuration['run_id'] = args.run_id or str(Path(args.output).resolve())
    return configuration


def check_resume_configuration(previous, configuration, target, modern_mode):
    saved = previous.get('configuration')
    if saved is not None and saved != configuration:
        raise ValueError('Collection configuration differs from manifest.json; use a new output directory')
    if previous and saved is None and modern_mode:
        raise ValueError('This older batch has no augmentation configuration; use a new output directory')
    if target < previous.get('episodes_per_id', 1):
        raise ValueError('On resume --episodes-per-id may only stay the same or increase')


def next_attempt_index(paths, records):
    indices = [int(record['attempt']) for record in records if 'attempt' in record]
    for path in paths:
        match = re.fullmatch(r'attempt_(\d+)(?:_seed_(-?\d+))?', path.name)
        if match is None:
            raise ValueError(f'Cannot determine the existing attempt index: {path}')
        indices.append(int(match.group(1)))
    return max(indices, default=-1) + 1


def refresh_verifications(output, attempt_records):
    """Reflect ordinary completed-episode revalidation during long batch runs."""
    for path, previous in list(attempt_records.items()):
        saved = read_json(output / path / 'verification.json')
        if saved is None:
            continue
        if saved.get('read_error'):
            raise RuntimeError(f'Cannot read episode verification: {path}: {saved["read_error"]}')
        attempt_records[path] = {**previous, **saved, 'path': path}


def asset_summary(identifier, entry, outcomes, target):
    accepted = [item['path'] for item in outcomes if item['passed']]
    return {'obstacle_id': identifier, 'category': entry['category'], 'label': entry.get('label'),
            'complete': len(accepted) >= target, 'passed': len(accepted) >= target,
            'attempts': len(outcomes), 'success_count': len(accepted), 'target_count': target,
            'deficit': max(0, target - len(accepted)),
            'accepted_path': accepted[0] if accepted else None, 'accepted_paths': accepted,
            'accepted_parameter_statistics': parameter_statistics(outcomes),
            'sampled_parameter_statistics': parameter_statistics(outcomes, sampled_only=True),
            'failure_counts': dict(Counter(normalize_failure_kind(item.get('failure_kind') or 'unclassified')
                                          for item in outcomes if not item['passed'])),
            'failure_phase_counts': dict(Counter(item.get('failure_phase') or 'unknown'
                                                for item in outcomes if not item['passed']))}


def numeric_values(value, prefix=''):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from numeric_values(child, f'{prefix}.{key}' if prefix else key)
    elif isinstance(value, (tuple, list)):
        for index, child in enumerate(value):
            yield from numeric_values(child, f'{prefix}[{index}]')
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        yield prefix, value


def parameter_statistics(records, *, sampled_only=False):
    """Accepted samples keep measured values; all-attempt sampling excludes them."""
    values = {}
    for record in records:
        if not sampled_only and not record['passed']:
            continue
        layout = record.get('sampled_layout') or {}
        parameters = {key: value for key, value in layout.items()
                      if key in ('mustard_position', 'bowl_position', 'mustard_yaw_deg', 'bowl_yaw_deg',
                                 'property', 'robot', 'obstacle')}
        parameters['appearance'] = record.get('appearance') or {}
        if not sampled_only:
            parameters['appearance_applied'] = record.get('appearance_applied') or {}
            parameters['settled_poses'] = record.get('settled_poses') or {}
        for key, number in numeric_values(parameters):
            values.setdefault(key, []).append(number)
    return {key: {'count': len(samples), 'min': min(samples), 'max': max(samples),
                  'mean': sum(samples) / len(samples)} for key, samples in sorted(values.items())}


def batch_statistics(attempts, assets, output):
    seconds = sum(record.get('elapsed_seconds', 0) for record in attempts)
    successes = [record for record in attempts if record['passed']]
    timings = Counter()
    for record in attempts:
        for key, value in numeric_values(record.get('timings_seconds') or {}):
            if not key.endswith('frames'):
                timings[key] += value
        for check, label in [('validation', 'validation_s'), ('audit', 'clearance_audit_s')]:
            timings[label] += record.get('checks', {}).get(check, {}).get('elapsed_seconds', 0)
    # Native per-attempt files are hardlinks to their shared physics batch.
    # Count actual file storage once, including interrupted and failed outputs.
    inodes = {}
    for path in output.rglob('*'):
        try:
            if not path.is_file():
                continue
            status = path.stat()
            inodes[(status.st_dev, status.st_ino)] = status.st_size
        except FileNotFoundError:
            # A producer may atomically replace a JSON temporary during a scan.
            continue
    size = sum(inodes.values())
    return {'attempt_count': len(attempts), 'success_count': len(successes),
            'target_count': sum(asset.get('target_count', 1) for asset in assets),
            'deficit': sum(asset.get('deficit', 0) for asset in assets),
            'accepted_frames': sum(record.get('frames') or 0 for record in successes),
            'elapsed_attempt_seconds': seconds,
            'successful_physical_trajectories_per_hour': len(successes) * 3600 / seconds if seconds else None,
            'timings_seconds': dict(timings), 'disk_bytes': size, 'disk_gib': round(size / 2**30, 3),
            'failure_counts': dict(Counter(normalize_failure_kind(record.get('failure_kind') or 'unclassified')
                                          for record in attempts if not record['passed'])),
            'failure_phase_counts': dict(Counter(record.get('failure_phase') or 'unknown'
                                                for record in attempts if not record['passed'])),
            'accepted_parameter_statistics': parameter_statistics(attempts),
            'sampled_parameter_statistics': parameter_statistics(attempts, sampled_only=True)}


def export_previews(output, assets, count):
    if not count:
        return {'requested_per_id': 0, 'passed': True, 'episodes': []}
    episodes = [output / path for asset in assets for path in asset.get('accepted_paths', [])[:count]]
    if not episodes:
        return {'requested_per_id': count, 'passed': False, 'episodes': [], 'error': 'No accepted episodes'}
    if any((path / 'vectorized_assignment.json').exists() for path in episodes):
        exports = []
        for path in episodes:
            index = 0
            while (path / f'preview_export_{index:03d}.log').exists():
                index += 1
            report_path = path / f'preview_export_{index:03d}.json'
            command = [sys.executable, str(ROOT / 'scripts/export_collection_previews.py'),
                       str(path), '--report', str(report_path)]
            if (path / 'vectorized_assignment.json').exists():
                command.extend(['--render-output', str(path / 'rendered')])
            execution = run_command(command, path / f'preview_export_{index:03d}.log')
            exports.append(dict(path=str(path.relative_to(output)), execution=execution,
                                report=read_json(report_path, [])))
            if execution.get('storage_error'):
                break
        storage = next((item['execution']['storage_error'] for item in exports
                        if item['execution'].get('storage_error')), None)
        return dict(requested_per_id=count, episodes=exports, storage_error=storage,
                    passed=len(exports) == len(episodes) and not storage
                    and all(item['execution'].get('exit_code') == 0 for item in exports))
    # New invocation log/report names avoid overwriting an interrupted export log.
    index = 0
    while (output / f'preview_export_{index:03d}.log').exists():
        index += 1
    report_path = output / f'preview_export_{index:03d}.json'
    command = [sys.executable, str(ROOT / 'scripts/export_collection_previews.py'),
               *map(str, episodes), '--report', str(report_path)]
    execution = run_command(command, output / f'preview_export_{index:03d}.log')
    report = read_json(report_path, [])
    return {'requested_per_id': count, 'passed': execution['exit_code'] == 0 and not execution.get('storage_error'),
            'episodes': [str(path.relative_to(output)) for path in episodes],
            'execution': execution, 'report': report, 'storage_error': execution.get('storage_error')}


def _main(argv=None, *, resources):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ids', nargs='+', help='Default: complete library for fixed mode; nonhuman10 for randomization')
    parser.add_argument('--max-attempts', type=int, default=1, help='Maximum total attempts per obstacle, including existing attempts')
    parser.add_argument('--seed', type=int, default=42, help='Master seed; fixed layouts use seed+attempt, randomized layouts use independent ID/attempt streams')
    parser.add_argument('--output', default='datasets/kitchen_auto/obstacle_library_v1')
    parser.add_argument('--video', action='store_true', help='Record and validate the original three cameras')
    parser.add_argument('--record-rgb', action='store_true', help='Record all three RGB streams without inline MP4 encoding')
    parser.add_argument('--randomization-config', help='Episode randomization JSON; omitted preserves fixed layouts')
    parser.add_argument('--episodes-per-id', type=int, default=1, help='Required successful trajectories per obstacle')
    parser.add_argument('--preview-per-id', type=int, default=0, help='Export this many accepted episodes per ID after collecting')
    parser.add_argument('--resume', action='store_true', help='Reuse existing verification records without overwriting attempts')
    add_runtime_arguments(parser)
    parser.add_argument('--placement-strategy', choices=['pose', 'lower-first'], default=argparse.SUPPRESS,
                        help='Forward to new collectors; omit to retain their default pose strategy')
    parser.add_argument('--grasp-tilt-deg', type=float, default=argparse.SUPPRESS,
                        help='Explicit grasp tilt; requires --grasp-yaw-deg')
    parser.add_argument('--grasp-yaw-deg', type=float, default=argparse.SUPPRESS,
                        help='Explicit grasp yaw; requires --grasp-tilt-deg')
    parser.add_argument('--grasp-offset-z-m', type=float, default=argparse.SUPPRESS,
                        help='Forward grasp/TCP height offset; omit to retain the collector default')
    parser.add_argument('--grasp-offset-y-m', type=float, default=argparse.SUPPRESS)
    parser.add_argument('--transport-bottle-height-m', type=float, default=argparse.SUPPRESS)
    release_options = parser.add_mutually_exclusive_group()
    release_options.add_argument('--supported-release', action='store_true', default=argparse.SUPPRESS)
    release_options.add_argument('--no-supported-release', dest='supported_release',
                                 action='store_false', default=argparse.SUPPRESS)
    parser.add_argument('--transport-tilt-cap-deg', type=float, default=argparse.SUPPRESS)
    parser.add_argument('--placement-tcp-height-m', type=float, default=argparse.SUPPRESS,
                        help='Independent placement TCP height; requires --placement-strategy lower-first')
    args = parser.parse_args(argv)
    if args.observer_config and args.execution_backend != 'vectorized':
        parser.error('--observer-config currently requires --execution-backend vectorized')
    if args.num_envs < 1 or args.validation_workers < 1:
        parser.error('--num-envs and --validation-workers must be positive')
    if args.pipeline and args.execution_backend != 'vectorized':
        parser.error('--pipeline requires --execution-backend vectorized')
    if args.execution_backend == 'vectorized':
        if not args.randomization_config or not args.record_rgb or args.video:
            parser.error('--execution-backend vectorized requires --randomization-config and --record-rgb without --video')
        if args.execution_checks != 'deferred':
            parser.error('The vectorized backend uses full deferred physics checks; specify --execution-checks deferred')
        if args.persistent_kit or args.cpu_rgb_copy:
            parser.error('--persistent-kit and --cpu-rgb-copy apply to the single collector backend only')
    if args.background_rgb and (not args.record_rgb or args.video):
        parser.error('--background-rgb requires --record-rgb without --video')
    runtime = runtime_configuration(args)
    collector_overrides = explicit_collector_overrides(args)
    if args.max_attempts < 1 or args.episodes_per_id < 1 or args.preview_per_id < 0:
        parser.error('--max-attempts/--episodes-per-id must be positive; --preview-per-id must be nonnegative')
    if args.max_attempts < args.episodes_per_id:
        parser.error('--max-attempts must be at least --episodes-per-id')
    if args.preview_per_id and not (args.video or args.record_rgb):
        parser.error('--preview-per-id requires --record-rgb or --video')
    if args.preview_per_id > args.episodes_per_id:
        parser.error('--preview-per-id cannot exceed --episodes-per-id')
    randomization_config = None
    if args.randomization_config:
        from kitchen_safety.randomization import SUPPORTED_IDS, load_randomization_config
        randomization_config = load_randomization_config(ROOT / args.randomization_config)
        if not 0 <= args.seed < 2**32:
            parser.error('Randomized --seed must be an integer in [0, 2**32)')
    if ('grasp_tilt_deg' in collector_overrides) != ('grasp_yaw_deg' in collector_overrides):
        parser.error('--grasp-tilt-deg and --grasp-yaw-deg must be specified together')
    if (collector_overrides.get('placement_tcp_height_m') is not None
            and collector_overrides.get('placement_strategy') != 'lower-first'):
        parser.error('--placement-tcp-height-m requires --placement-strategy lower-first')
    catalog = load_assets()
    entries = {entry['id']: entry for entry in catalog}
    random_ids = read_json(ROOT / 'configs/reproduction_presets.json', {}).get('nonhuman10', {}).get('ids', [])
    identifiers = args.ids or (random_ids if randomization_config is not None else list(entries))
    if len(set(identifiers)) != len(identifiers):
        parser.error('--ids contains duplicate identifiers')
    unknown = set(identifiers) - set(entries)
    if unknown:
        parser.error(f'Unknown obstacle identifiers: {sorted(unknown)}')
    if randomization_config is not None and set(identifiers) - set(SUPPORTED_IDS):
        parser.error('Randomization requires a supported nonhuman10 or human7 obstacle ID')
    output = (ROOT / args.output).resolve()
    if output.exists() and any(output.iterdir()) and not args.resume:
        parser.error(f'{output} is not empty; use --resume or a new output directory')
    output.mkdir(parents=True, exist_ok=True)
    previous = read_json(output / 'manifest.json', {}) if args.resume else {}
    if 'read_error' in previous:
        raise RuntimeError(f'Cannot preserve the existing manifest: {previous["read_error"]}')
    modern_mode = bool(args.randomization_config or args.record_rgb or
                       args.episodes_per_id != 1 or args.preview_per_id or runtime)
    configuration = configuration_for(args, collector_overrides, randomization_config)
    check_resume_configuration(previous, configuration, args.episodes_per_id, modern_mode)
    # Preserve manual recheck metadata and records outside an explicitly resumed
    # subset. A same-seed recheck is an independent historical attempt.
    requested = list(dict.fromkeys(previous.get('requested_ids', []) + identifiers))
    manifest = {**previous,
        'scope': ('Randomized physical grasp/place collection; accepted trajectories are not a policy success rate'
                  if randomization_config is not None else
                  'Fixed-layout complete grasp/place verification, not a policy success-rate benchmark'),
        'planner': 'curobo', 'requested_ids': requested, 'seed': args.seed,
        'max_attempts_per_obstacle': args.max_attempts, 'video_requested': args.video,
        'rgb_requested': bool(args.video or args.record_rgb), 'episodes_per_id': args.episodes_per_id,
        'preview_per_id': args.preview_per_id,
        'started_at': previous.get('started_at', now()), 'complete': False, 'all_passed': False,
    }
    manifest.pop('storage_error', None)
    manifest.pop('worker_unavailable', None)
    manifest.pop('stop_reason', None)
    if modern_mode or 'configuration' in previous:
        manifest['configuration'] = configuration
    if args.observer_config:
        manifests = output / 'manifests'
        manifests.mkdir(exist_ok=True)
        write_json(manifests / 'resolved_observer_config.json', configuration['safety_observer'])
    attempt_records = {item.get('path', f'previous_record_{index}'): item
                       for index, item in enumerate(previous.get('attempts', []))}
    asset_records = {item['obstacle_id']: item for item in previous.get('assets', [])}
    for identifier in requested:
        if identifier in entries:
            outcomes = [item for item in attempt_records.values() if item.get('obstacle_id') == identifier]
            asset_records[identifier] = asset_summary(identifier, entries[identifier], outcomes, args.episodes_per_id)

    def publish():
        refresh_verifications(output, attempt_records)
        for identifier in requested:
            if identifier in entries:
                outcomes = [item for item in attempt_records.values() if item.get('obstacle_id') == identifier]
                asset_records[identifier] = {**asset_records.get(identifier, {}),
                    **asset_summary(identifier, entries[identifier], outcomes, args.episodes_per_id)}
        manifest['attempts'] = list(attempt_records.values())
        manifest['assets'] = list(asset_records.values())
        manifest['passed_count'] = sum(asset_records.get(identifier, {}).get('passed', False) for identifier in requested)
        manifest['failed_count'] = len(requested) - manifest['passed_count']
        if manifest['failed_count']:
            manifest['complete'] = False
            manifest['all_passed'] = False
        manifest['updated_at'] = now()
        manifest['statistics'] = batch_statistics(manifest['attempts'], manifest['assets'], output)
        if args.execution_backend == 'vectorized':
            seconds = manifest.get('collection_wall_seconds', 0)
            imported = manifest.get('imported_history', {}).get('success_count', 0)
            historical_reclassified = sum(
                bool(record.get('historical_eligibility_review')) and bool(record.get('passed'))
                for record in manifest['attempts'])
            new_successes = manifest['statistics']['success_count'] - imported - historical_reclassified
            manifest['statistics'].update(collection_wall_seconds=seconds,
                imported_success_count=imported, new_success_count=new_successes,
                historical_reclassified_success_count=historical_reclassified,
                throughput_time_scope=(
                    'New accepted trajectories only, excluding imported_history.success_count '
                    'and accepted historical_eligibility_review records; '
                    'divided by this output directory\'s accumulated invocation wall time, '
                    'including shared physics, rendering, verification and renderer shutdown'),
                successful_physical_trajectories_per_hour=(
                    new_successes * 3600 / seconds if seconds else None))
        if args.observer_config:
            from kitchen_safety.risk_reporting import write_indexes
            manifest['safety_indexes'] = write_indexes(output, manifest['attempts'])
        write_json(output / 'manifest.json', manifest)
        with (output / 'summary.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['obstacle_id', 'category', 'attempts',
                                                        'success_count', 'target_count', 'deficit', 'passed'])
            writer.writeheader()
            for asset in manifest['assets']:
                writer.writerow({key: asset.get(key) for key in writer.fieldnames})

    if args.execution_backend == 'vectorized':
        from kitchen_safety.parallel_collection import collect_vectorized
        return collect_vectorized(args=args, identifiers=identifiers, entries=entries, output=output,
            overrides=collector_overrides, randomization_config=randomization_config,
            manifest=manifest, attempt_records=attempt_records, asset_records=asset_records,
            publish=publish, resources=resources, api=sys.modules[__name__])

    collect = run_command
    if args.persistent_kit:
        from kitchen_safety.collector_worker import CollectorWorker
        collect = resources.enter_context(CollectorWorker(cwd=ROOT)).execute

    for identifier in identifiers:
        entry = entries[identifier]
        # Discover directories, rather than reconstructing only seed+index paths:
        # targeted fixes may deliberately rerun the same seed under a new index.
        existing = sorted(path for path in (output / identifier).glob('attempt_*') if path.is_dir())
        largest_attempt = -1
        for destination in existing:
            match = re.match(r'attempt_(\d+)(?:_seed_(-?\d+))?', destination.name)
            if match is None:
                raise ValueError(f'Cannot determine the existing attempt index: {destination}')
            attempt = int(match.group(1))
            seed = int(match.group(2)) if match.group(2) is not None else args.seed + attempt
            largest_attempt = max(largest_attempt, attempt)
            record = read_json(destination / 'verification.json')
            if record is not None:
                if 'read_error' in record:
                    raise RuntimeError(f'Unreadable original verification: {destination}: {record}')
                if record.get('video_requested') != args.video:
                    raise ValueError(f'--video differs from the saved attempt: {destination}; use a new output directory')
                if record.get('rgb_requested', record.get('video_requested', False)) != bool(args.video or args.record_rgb):
                    raise ValueError(f'RGB output differs from the saved attempt: {destination}; use a new output directory')
                if record.get('storage_error') and record.get('execution_success'):
                    archive_unfinished_check(destination, 'verification')
                    record = verify_attempt(destination, entry, seed, args.video, record['execution'],
                                            record_rgb=args.record_rgb, randomized=randomization_config is not None)
                print(f'RESUME {identifier} attempt={attempt} seed={seed} passed={record["passed"]}', flush=True)
            else:
                execution = read_json(destination / 'execution.json', {
                    'exit_code': None, 'elapsed_seconds': 0,
                    'error': 'Interrupted attempt: original collector exit status is unavailable'})
                record = verify_attempt(destination, entry, seed, args.video, execution,
                                        record_rgb=args.record_rgb, randomized=randomization_config is not None)
            path = str(destination.relative_to(output))
            attempt_records[path] = {**attempt_records.get(path, {}), **record,
                                     'path': path, 'attempt': attempt}
            # Historical collector storage failures retain their attempt slot.
            # A retried check that still cannot write must stop this invocation.
            if record.get('storage_error') and record.get('execution_success'):
                manifest.update(storage_error=record['storage_error'], stop_reason='storage_error')
                publish()
                return STORAGE_ERROR_EXIT_CODE
        outcomes = [item for item in attempt_records.values() if item.get('obstacle_id') == identifier]
        largest_attempt = next_attempt_index(existing, outcomes) - 1
        asset_records[identifier] = asset_summary(identifier, entry, outcomes, args.episodes_per_id)
        publish()
        while True:
            refresh_verifications(output, attempt_records)
            outcomes = [item for item in attempt_records.values() if item.get('obstacle_id') == identifier]
            if sum(item['passed'] for item in outcomes) >= args.episodes_per_id or len(outcomes) >= args.max_attempts:
                break
            attempt = largest_attempt + 1
            seed = args.seed + attempt
            spec = None
            if randomization_config is not None:
                from kitchen_safety.randomization import sample_episode
                spec = sample_episode(identifier, randomization_config, args.seed, attempt)
                seed = int(spec['seeds']['planner'])
            destination = output / identifier / f'attempt_{attempt:04d}_seed_{seed}'
            destination.mkdir(parents=True)
            command = [sys.executable, str(ROOT / 'scripts/collect_kitchen.py'),
                       '--obstacle_id', identifier, '--variant', entry['category'],
                       '--planner', 'curobo', '--seed', str(seed), '--output', str(destination),
                       *runtime_command_args(runtime, collector=True)]
            if spec is not None:
                spec_path = destination / 'episode_spec.json'
                write_json(spec_path, spec)
                command.extend(['--episode-spec', str(spec_path)])
            for name, value in collector_overrides.items():
                option = name.replace('_', '-')
                if isinstance(value, bool):
                    command.append(('--' if value else '--no-') + option)
                else:
                    command.extend(['--' + option, str(value)])
            if args.video:
                command.append('--video')
            elif args.record_rgb:
                command.append('--record-rgb')
            print(f'START {identifier} attempt={attempt} seed={seed}', flush=True)
            execution = collect(command, destination / 'run.log')
            execution['command'] = command
            execution['collector_overrides'] = dict(collector_overrides)
            if runtime:
                execution['runtime'] = dict(runtime)
            write_json(destination / 'execution.json', execution)
            record = verify_attempt(destination, entry, seed, args.video, execution,
                                    record_rgb=args.record_rgb, randomized=randomization_config is not None)
            path = str(destination.relative_to(output))
            summary = dict(record, path=path, attempt=attempt)
            outcomes.append(summary)
            attempt_records[path] = summary
            largest_attempt = attempt
            asset_records[identifier] = asset_summary(identifier, entry, outcomes, args.episodes_per_id)
            if record.get('storage_error'):
                manifest.update(storage_error=record['storage_error'], stop_reason='storage_error')
            elif record.get('worker_unavailable'):
                manifest.update(worker_unavailable=record.get('error') or
                                'Collector worker unavailable; resume to start a new worker',
                                stop_reason='worker_unavailable')
            publish()
            print(f'FINISH {identifier} attempt={attempt} execution={record["execution_success"]} '
                  f'data={record["data_success"]} audit={record["audit_pass"]} '
                  f'eligible={record["augmentation_eligible"]} frames={record["frames"]} '
                  f'phase={record["failure_phase"]} kind={record["failure_kind"]} '
                  f'accepted={sum(item["passed"] for item in outcomes)}/{args.episodes_per_id}', flush=True)
            if record.get('storage_error'):
                return STORAGE_ERROR_EXIT_CODE
            if record.get('worker_unavailable'):
                return WORKER_UNAVAILABLE_EXIT_CODE
        asset_records[identifier] = asset_summary(identifier, entry, outcomes, args.episodes_per_id)
        asset_records[identifier]['attempt_budget_exhausted'] = len(outcomes) >= args.max_attempts
        publish()
    # A terminated attempt budget is a reported deficit, not completed collection.
    manifest['complete'] = all(asset_records.get(identifier, {}).get('success_count', 0) >= args.episodes_per_id
                               for identifier in requested)
    manifest['all_passed'] = manifest['complete']
    manifest['passed_count'] = sum(asset_records.get(identifier, {}).get('passed', False) for identifier in requested)
    manifest['failed_count'] = len(requested) - manifest['passed_count']
    manifest['collection_pass_finished'] = True
    # A subset resume still reports completion for the complete accumulated
    # batch. Verify/export every requested ID before claiming those previews.
    manifest['previews'] = export_previews(output, [asset_records[item] for item in requested], args.preview_per_id)
    manifest['all_passed'] = manifest['all_passed'] and manifest['previews']['passed']
    manifest['complete'] = manifest['complete'] and manifest['previews']['passed']
    if manifest['previews'].get('storage_error'):
        manifest.update(storage_error=manifest['previews']['storage_error'], stop_reason='storage_error')
    publish()
    print(json.dumps({key: manifest[key] for key in
                      ('complete', 'all_passed', 'passed_count', 'failed_count')}, ensure_ascii=False), flush=True)
    return STORAGE_ERROR_EXIT_CODE if manifest.get('storage_error') else (0 if manifest['all_passed'] else 1)


def main(argv=None):
    try:
        with ExitStack() as resources:
            return _main(argv, resources=resources)
    except OSError as error:
        if not storage_error_message(error):
            raise
        # Metadata itself may be unwritable; stderr and existing partial files
        # remain the recovery evidence, and the caller must stop retrying.
        report_storage_error(storage_error_message(error))
        return STORAGE_ERROR_EXIT_CODE


if __name__ == '__main__':
    raise SystemExit(main())
