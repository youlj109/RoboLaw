#!/usr/bin/env python3
"""Replay existing complete safety archives with fresh CPU worker pools.

No Kit, physics simulation, RGB rendering, or source data writes are performed.
The job arguments and timed executor lifetime match collect_kitchen_vectorized.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
import json
import multiprocessing
from pathlib import Path
import time

from kitchen_safety.vectorized_validation import validate_physics_archive


def without_timings(value):
    if isinstance(value, dict):
        return {key: without_timings(item) for key, item in value.items()
                if key != 'timings_seconds'}
    if isinstance(value, list):
        return [without_timings(item) for item in value]
    return value


def differences(left, right, path='$'):
    """Report every non-timing discrepancy, including all diagnostic distances."""
    if isinstance(left, dict) and isinstance(right, dict):
        found = []
        for key in sorted(set(left) | set(right)):
            if key not in left or key not in right:
                found.append({'path': path + '.' + key, 'missing_from':
                              'left' if key not in left else 'right'})
            else:
                found.extend(differences(left[key], right[key], path + '.' + key))
        return found
    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            return [{'path': path, 'left_length': len(left), 'right_length': len(right)}]
        found = []
        for index, (a, b) in enumerate(zip(left, right)):
            found.extend(differences(a, b, '{}[{}]'.format(path, index)))
        return found
    if left != right:
        return [{'path': path, 'left': left, 'right': right}]
    return []


def load_batch(physics_dir):
    original = json.loads((physics_dir / 'physics_validation.json').read_text())
    episodes = sorted(physics_dir.glob('env_*/result.json'))
    if len(episodes) != len(original):
        raise ValueError('Original validation and episode counts differ')
    jobs = []
    for index, path in enumerate(episodes):
        result = json.loads(path.read_text())
        spec = json.loads((path.parent / 'episode_spec.json').read_text())
        if spec != result['episode_spec']:
            raise ValueError('Saved episode spec differs from result: {}'.format(path))
        env_root = '/World/envs/env_{}'.format(result['env_id'])
        origin = result['environment_origin_world_m']
        if (original[index]['env_root'] != env_root or
                original[index]['env_origin'] != origin):
            raise ValueError('Original environment mapping differs: {}'.format(path))
        jobs.append(dict(
            stage_path=str(physics_dir / 'validation_scene.usdc'),
            physics_states_path=str(path.parent / 'physics_states.npz'),
            episode_spec=spec, env_root=env_root, env_origin=origin,
            settled_poses=result.get('settled_poses', {}), state_frame='env_local',
            action_clearance_states_path=str(path.parent / 'action_clearance_states.npz')))
    return jobs, original


def run_benchmark(source, output, workers):
    source, output = source.resolve(), output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    batches = sorted(source.glob('batch_*/physics'))
    if not batches:
        raise ValueError('No saved batch_*/physics directories in {}'.format(source))
    report = dict(status='running', started_at=datetime.now(timezone.utc).isoformat(),
        source=str(source), workers=workers, runs=[], comparisons=[],
        production_quota_eligible=False,
        scope='CPU-only replay of the exact original physics-rate and post-action diagnostic archives',
        timing_scope='Each run includes fresh spawn ProcessPoolExecutor startup, all map jobs, and shutdown/join; excludes input reconstruction and JSON writes',
        limitations=['One measurement per worker count and source batch, in the recorded order',
                     'No concurrent renderer or physical producer; no claim about full-pipeline speedup',
                     'No simulated trajectory is regenerated; original source files remain unchanged'],
        comparison_scope='Exact recursive equality of every original result field except timings_seconds; includes every check count, collider path, and diagnostic distance')
    report_path = output / 'report.json'

    def save():
        report_path.write_text(json.dumps(report, indent=2) + '\n')

    save()
    try:
        for physics_dir in batches:
            jobs, original = load_batch(physics_dir)
            batch_name = physics_dir.parent.name
            (output / (batch_name + '_jobs.json')).write_text(json.dumps(jobs, indent=2) + '\n')
            actual = {}
            for count in workers:
                print('VALIDATION_BENCH_START', batch_name, count, flush=True)
                started = time.perf_counter()
                with ProcessPoolExecutor(max_workers=min(count, len(jobs)),
                        mp_context=multiprocessing.get_context('spawn')) as executor:
                    result = list(executor.map(validate_physics_archive, jobs))
                elapsed = time.perf_counter() - started
                result_path = output / '{}_workers{}.json'.format(batch_name, count)
                result_path.write_text(json.dumps(result, indent=2) + '\n')
                actual[count] = without_timings(result)
                mismatch = differences(without_timings(original), actual[count])
                report['runs'].append(dict(batch=batch_name, workers=count,
                    pool_wall_s=elapsed, jobs=len(jobs), passed=sum(row['passed'] for row in result),
                    checks=[row['checks'] for row in result],
                    valid_calls=[row.get('valid_calls') for row in result],
                    diagnostic_samples=[row.get('action_clearance_diagnostics', {}).get('samples')
                                        for row in result],
                    result_path=str(result_path), original_result_path=str(physics_dir / 'physics_validation.json'),
                    exactly_matches_original_except_timings=not mismatch,
                    differences_from_original=mismatch))
                save()
                print('VALIDATION_BENCH_DONE', batch_name, count, elapsed,
                      'exact_original_match', not mismatch, flush=True)
            reference = workers[0]
            for count in workers[1:]:
                mismatch = differences(actual[reference], actual[count])
                report['comparisons'].append(dict(batch=batch_name, workers=[reference, count],
                    exactly_equal_except_timings=not mismatch, differences=mismatch))
        totals = {str(count): sum(row['pool_wall_s'] for row in report['runs']
                                 if row['workers'] == count) for count in workers}
        report['total_pool_wall_s_by_workers'] = totals
        report['all_results_match_original'] = all(
            row['exactly_matches_original_except_timings'] for row in report['runs'])
        report['all_worker_results_identical'] = all(
            row['exactly_equal_except_timings'] for row in report['comparisons'])
        report['all_checks_passed'] = all(row['passed'] == row['jobs'] for row in report['runs'])
        report['status'] = 'completed'
    except BaseException as error:
        report.update(status='failed', error='{}: {}'.format(type(error).__name__, error))
        raise
    finally:
        report['finished_at'] = datetime.now(timezone.utc).isoformat()
        save()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, nargs='+', default=[2, 4])
    args = parser.parse_args()
    if len(set(args.workers)) != len(args.workers) or any(count < 1 for count in args.workers):
        parser.error('--workers must contain distinct positive integers')
    report = run_benchmark(args.source, args.output, args.workers)
    print(json.dumps({key: report[key] for key in ('status', 'total_pool_wall_s_by_workers',
          'all_results_match_original', 'all_worker_results_identical', 'all_checks_passed')}), flush=True)
    return 0 if (report['all_results_match_original'] and report['all_worker_results_identical']
                 and report['all_checks_passed']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
