"""Run fixed regression, randomized pilot and full collection sequentially.

Run via scripts/arena_python.sh. The default single backend uses one Kit;
the optional vectorized pipeline overlaps one physics Kit and one render Kit.
No fixed regression failure is treated as permission to skip a scene.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.collect_obstacle_library import (STORAGE_ERROR_EXIT_CODE, WORKER_UNAVAILABLE_EXIT_CODE, now, read_json,
                                              report_storage_error, run_command,
                                              storage_error_message, write_json,
                                              add_runtime_arguments, runtime_configuration, runtime_command_args)


def build_stages(args, output):
    presets = read_json(ROOT / 'configs/reproduction_presets.json')
    runtime = runtime_configuration(args)
    regression_runtime = {key: value for key, value in runtime.items()
                          if key not in ('execution_backend', 'num_envs', 'validation_workers', 'pipeline')}
    observer_args = (['--observer-config', args.observer_config, '--run-id', args.run_id or str(output)]
                     if getattr(args, 'observer_config', None) else [])
    base = [sys.executable, str(ROOT / 'scripts/collect_obstacle_library.py'),
            '--ids', *args.ids, '--record-rgb']
    randomized = [*base, *runtime_command_args(runtime), '--seed', str(args.seed), '--max-attempts', str(args.max_attempts),
                  '--randomization-config', str((ROOT / args.randomization_config).resolve()), *observer_args,
                  '--output', str(output / 'episodes')]
    return [
        {'name': 'regression', 'command': [*base, *runtime_command_args(regression_runtime), '--seed', str(presets['seed']),
            '--max-attempts', '1', '--episodes-per-id', '1', '--output', str(output / 'regression')],
         'manifest': str(output / 'regression/manifest.json')},
        {'name': 'pilot', 'command': [*randomized, '--episodes-per-id', str(args.pilot_per_id)],
         'manifest': str(output / 'episodes/manifest.json')},
        {'name': 'production', 'command': [*randomized, '--episodes-per-id', str(args.episodes_per_id),
            '--preview-per-id', str(args.preview_per_id), '--resume'],
         'manifest': str(output / 'episodes/manifest.json')},
    ]


def pilot_attempt_budgets(step, maximum, saved_manifest):
    """Round cumulative budgets up around existing attempts, preserving ID seeds.

    Revisit the interrupted/current round first. Completed IDs are skipped by
    the batch collector, and a previously finished round is a cheap no-op.
    """
    largest = max((asset.get('attempts', 0) for asset in saved_manifest.get('assets', [])), default=0)
    first = min(maximum, max(step, ((largest + step - 1) // step) * step))
    budgets = list(range(first, maximum + 1, step))
    if not budgets or budgets[-1] != maximum:
        budgets.append(maximum)
    return budgets


def _main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ids', nargs='+', help='Default: the restored nonhuman10 preset')
    parser.add_argument('--randomization-config', default='configs/augmentation_nonhuman10.json')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--episodes-per-id', type=int, default=20)
    parser.add_argument('--pilot-per-id', type=int, default=2)
    parser.add_argument('--max-attempts', type=int, default=100)
    parser.add_argument('--preview-per-id', type=int, default=3)
    parser.add_argument('--output')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    add_runtime_arguments(parser)
    args = parser.parse_args(argv)
    presets = read_json(ROOT / 'configs/reproduction_presets.json')
    allowed = presets['nonhuman10']['ids']
    args.ids = args.ids or allowed
    if len(set(args.ids)) != len(args.ids) or not set(args.ids) <= set(allowed):
        parser.error('--ids must contain unique restored nonhuman10 identifiers')
    if not 1 <= args.pilot_per_id <= args.episodes_per_id <= args.max_attempts:
        parser.error('Require 1 <= pilot-per-id <= episodes-per-id <= max-attempts')
    if not 0 <= args.preview_per_id <= args.episodes_per_id:
        parser.error('Require 0 <= preview-per-id <= episodes-per-id')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    output = (ROOT / (args.output or f'datasets/augmentation/nonhuman10_{stamp}')).resolve()
    if args.resume and not args.output:
        parser.error('--resume requires --output identifying the existing batch')
    stages = build_stages(args, output)
    if args.dry_run:
        print('OUTPUT', output)
        for stage in stages:
            command = list(stage['command'])
            if stage['name'] == 'pilot':
                command[command.index('--max-attempts') + 1] = str(args.pilot_per_id)
                print('pilot cumulative budgets:', pilot_attempt_budgets(args.pilot_per_id, args.max_attempts, {}))
            print(stage['name'], shlex.join(command))
        return 0
    from kitchen_safety.randomization import load_randomization_config
    configuration = {'ids': args.ids, 'seed': args.seed,
                     'randomization': load_randomization_config(ROOT / args.randomization_config),
                     'pilot_per_id': args.pilot_per_id}
    runtime = runtime_configuration(args)
    if runtime:
        configuration['runtime'] = runtime
    if output.exists() and any(output.iterdir()) and not args.resume:
        parser.error(f'{output} is not empty; use --resume or another output')
    output.mkdir(parents=True, exist_ok=True)
    status_path = output / 'augmentation.json'
    previous = read_json(status_path, {})
    if previous.get('read_error'):
        parser.error(f'Cannot read existing progress: {previous["read_error"]}')
    if previous and previous.get('configuration') != configuration:
        parser.error('Configuration changed; use a new output directory')
    if args.episodes_per_id < previous.get('episodes_per_id', 1):
        parser.error('The target may only stay the same or increase on resume')
    report = {**previous, 'configuration': configuration,
              'episodes_per_id': args.episodes_per_id, 'max_attempts_per_id': args.max_attempts,
              'started_at': previous.get('started_at', now()), 'complete': False, 'passed': False,
              'scope': 'Physical data augmentation, not policy success-rate evaluation',
              'stages': previous.get('stages', {})}
    report.pop('storage_error', None)
    report.pop('worker_unavailable', None)
    report.pop('stop_reason', None)
    write_json(status_path, report)
    for stage in stages:
        name = stage['name']
        old = report['stages'].get(name, {})
        # Successful fixed regression and pilot remain valid for a resumed batch.
        if name in ('regression', 'pilot') and old.get('passed'):
            print('RESUME completed stage', name, flush=True)
            continue
        manifest_path = Path(stage['manifest'])
        saved_manifest = read_json(manifest_path, {})
        if name == 'pilot' and saved_manifest.get('episodes_per_id', 0) > args.pilot_per_id:
            print('RESUME production already started; retaining prior pilot report', flush=True)
            continue
        budgets = (pilot_attempt_budgets(args.pilot_per_id, args.max_attempts, saved_manifest)
                   if name == 'pilot' else [None])
        for budget in budgets:
            command = list(stage['command'])
            if budget is not None:
                command[command.index('--max-attempts') + 1] = str(budget)
            if manifest_path.exists() and '--resume' not in command:
                command.append('--resume')
            index = 0
            while (output / f'{name}_{index:03d}.log').exists():
                index += 1
            log = output / f'{name}_{index:03d}.log'
            print('START', name, 'cumulative_budget=', budget, 'log=', log, flush=True)
            execution = run_command(command, log)
            manifest = read_json(manifest_path, {})
            storage_error = execution.get('storage_error')
            if execution['exit_code'] == STORAGE_ERROR_EXIT_CODE:
                storage_error = storage_error or manifest.get('storage_error') or 'Batch reported storage_error (exit 74)'
            worker_unavailable = (manifest.get('worker_unavailable') or 'Collector worker unavailable') \
                if execution['exit_code'] == WORKER_UNAVAILABLE_EXIT_CODE else None
            passed = execution['exit_code'] == 0 and manifest.get('all_passed') is True and not storage_error
            stage_result = {'execution': execution, 'command': command,
                            'manifest': str(manifest_path), 'passed': passed,
                            'statistics': manifest.get('statistics'), 'finished_at': now()}
            if budget is not None:
                stage_result['cumulative_attempt_budget_per_id'] = budget
                report.setdefault('pilot_rounds', []).append(stage_result)
            report['stages'][name] = stage_result
            report['updated_at'] = now()
            if storage_error:
                report.update(storage_error=storage_error, stop_reason='storage_error',
                              complete=False, passed=False)
            elif worker_unavailable:
                report.update(worker_unavailable=worker_unavailable, stop_reason='worker_unavailable',
                              complete=False, passed=False)
            write_json(status_path, report)
            if storage_error:
                report_storage_error(f'{name}: {storage_error}')
                return STORAGE_ERROR_EXIT_CODE
            if worker_unavailable:
                print(f'{name}: {worker_unavailable}; stopped until --resume', file=sys.stderr, flush=True)
                return WORKER_UNAVAILABLE_EXIT_CODE
            print('FINISH', name, 'cumulative_budget=', budget, 'passed=', passed, 'statistics=',
                  json.dumps({key: manifest.get('statistics', {}).get(key) for key in
                              ('attempt_count', 'success_count', 'target_count', 'deficit')}, ensure_ascii=False), flush=True)
            if passed:
                break
        if name == 'regression' and not passed:
            report['stop_reason'] = 'Fixed-layout regression failed; random collection was not started'
            write_json(status_path, report)
            return 1
        # A pilot deficit is retained. Production still fills other IDs, without
        # resetting any failed scene's existing per-ID attempt budget.
    manifest = read_json(output / 'episodes/manifest.json', {})
    report['statistics'] = manifest.get('statistics', {})
    report['passed'] = bool(report['stages'].get('regression', {}).get('passed')
                            and report['stages'].get('production', {}).get('passed'))
    report['complete'] = report['passed']
    report['finished_at'] = now()
    if not report['passed']:
        report['stop_reason'] = 'Attempt budgets or output verification left an explicit collection deficit'
    else:
        report.pop('stop_reason', None)
    write_json(status_path, report)
    print(json.dumps({'output': str(output), 'complete': report['complete'],
                      'statistics': report['statistics']}, ensure_ascii=False), flush=True)
    return 0 if report['passed'] else 1


def main(argv=None):
    try:
        return _main(argv)
    except OSError as error:
        if not storage_error_message(error):
            raise
        report_storage_error(storage_error_message(error))
        return STORAGE_ERROR_EXIT_CODE


if __name__ == '__main__':
    raise SystemExit(main())
