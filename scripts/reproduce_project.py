"""Small reproduction entry point; existing collectors perform the actual work.

Entry-point asset check (standard Python, no GPU):
    python3 scripts/reproduce_project.py

Print commands without starting Kit or creating output:
    python3 scripts/reproduce_project.py --task human4 --dry-run
    python3 scripts/reproduce_project.py --task nonhuman10 --dry-run

Run with the installed Arena environment:
    bash scripts/arena_python.sh scripts/reproduce_project.py --task smoke
    bash scripts/arena_python.sh scripts/reproduce_project.py --task base
    bash scripts/arena_python.sh scripts/reproduce_project.py --task human4
    bash scripts/arena_python.sh scripts/reproduce_project.py --task nonhuman10

Only one child is run at a time. Do not launch other Kit jobs on the same GPU.
Each invocation creates a new output directory; historical results are never
used as fresh successes. Every full episode records the original three cameras.
human4 selects four human layouts; every run establishes its own validation result.
nonhuman10 runs the ten property/robot layouts in one sequential collection batch.
"""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
PRESETS = ROOT / 'configs/reproduction_presets.json'
VARIANTS = ('human', 'property', 'robot')


def read_json(path):
    return json.loads(path.read_text())


def asset_check():
    """Check local entry points, not USD dependency composition or GPU support."""
    layout = read_json(ROOT / 'configs/layout.json')
    prefix = 'assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets'
    paths = {
        'configs/layout.json', 'configs/reproduction_presets.json',
        'assets/kitchen/scene.usd',
        'assets/robolab/objects/hope/mustard_bottle.usd',
        'assets/robolab/objects/ycb/bowl.usd',
        prefix + '/robot_library/droid/franka_robotiq_2f_85_flattened.usd',
        prefix + '/object_library/srl_robolab_assets/robots/franka_stand_grey.usda',
        'assets/curobo/droid/franka_robotiq_2f_85_zero_curobo.yml',
        'assets/curobo/droid/urdf/franka_robotiq_2f_85_zero.urdf',
        'assets/curobo/droid/tcp_calibration.json',
        'assets/planning/placement_guides.json',
        layout['human']['asset'], layout['robot']['barrier_asset'],
    }
    for name in ('glass', 'plate', 'phone'):
        paths.add(layout['property'].get(name + '_asset', f'assets/safety/{name}.usda'))
    for category in ('people', 'property', 'robot'):
        catalog = Path('assets/obstacle_library') / category / 'catalog.json'
        paths.add(str(catalog))
        if (ROOT / catalog).is_file():
            paths.update(entry['usd_path'] for entry in read_json(ROOT / catalog)['assets'])
    missing = [path for path in sorted(paths) if not (ROOT / path).is_file()]
    result = {
        'scope': 'Local entry-point files only. Run smoke to load USD dependencies, physics and the three RGB cameras.',
        'checked_files': len(paths), 'missing': missing, 'passed': not missing,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result['passed'] else 1


def build_jobs(args, presets, output):
    """Return ordinary subprocess arguments; no shell interpolation is used."""
    python = sys.executable
    if args.task == 'nonhuman10':
        settings = presets['nonhuman10']
        selected = list(args.ids or settings['ids'])
        command = [python, str(ROOT / 'scripts/collect_obstacle_library.py'),
                   '--ids', *selected, '--max-attempts', '1',
                   '--seed', str(presets['seed']), '--video',
                   '--output', str(output / 'episodes')]
        # The collector resolves shared and per-ID defaults from this same JSON.
        # Passing shared defaults as CLI overrides would hide per-ID settings.
        return [{'id': 'nonhuman10', 'selected_ids': selected,
                 'preset_status': settings['status'], 'command': command}]
    if args.task == 'human4':
        selected = set(args.ids or [item['id'] for item in presets['human4']])
        jobs = []
        for item in presets['human4']:
            if item['id'] not in selected:
                continue
            command = [python, str(ROOT / 'scripts/validate_lower_first.py'),
                       '--video-only', '--ids', item['id'], '--label', item['label'],
                       '--seed', str(presets['seed']), '--output', str(output / 'episodes')]
            for key in ('grasp_tilt_deg', 'grasp_yaw_deg', 'grasp_offset_z_m',
                        'placement_tcp_height_m'):
                if item[key] is not None:
                    command.extend(['--' + key.replace('_', '-'), str(item[key])])
            jobs.append({'id': item['id'], 'command': command,
                         'accepted_reference': item['accepted_reference']})
        return jobs
    variants = args.variants or (['human'] if args.task == 'smoke' else list(VARIANTS))
    jobs = []
    for variant in variants:
        directory = output / variant
        if args.task == 'smoke':
            command = [python, str(ROOT / 'scripts/reproduce_scene_smoke.py'),
                       '--variant', variant, '--steps', str(args.steps),
                       '--output', str(directory)]
        else:
            command = [python, str(ROOT / 'scripts/collect_kitchen.py'),
                       '--variant', variant, '--planner', 'curobo', '--video',
                       '--seed', str(presets['seed']), '--output', str(directory)]
        jobs.append({'id': variant, 'variant': variant, 'directory': str(directory),
                     'command': command})
    return jobs


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--task', choices=['assets', 'smoke', 'base', 'human4', 'nonhuman10'], default='assets',
                        help='Default: assets. smoke holds the pose; base collects the three default safety scenes; human4 and nonhuman10 run their selected layouts, validated per run.')
    parser.add_argument('--ids', nargs='+', help='human4/nonhuman10 only: select one or more obstacle IDs from that task preset')
    parser.add_argument('--variants', nargs='+', choices=VARIANTS,
                        help='smoke/base only; smoke defaults to human, base to all three')
    parser.add_argument('--steps', type=int, default=30, help='smoke only: hold-pose environment steps (default 30)')
    parser.add_argument('--output', help='New output directory, relative to the project or absolute inside the runtime; default: datasets/reproduction/TASK_TIMESTAMP')
    parser.add_argument('--dry-run', action='store_true', help='Print subprocess commands; do not launch or create output')
    args = parser.parse_args()
    presets = read_json(PRESETS)
    if args.ids:
        if args.task not in ('human4', 'nonhuman10'):
            parser.error('--ids is only for human4/nonhuman10')
        allowed_ids = ({item['id'] for item in presets['human4']} if args.task == 'human4'
                       else set(presets['nonhuman10']['ids']))
        if not set(args.ids) <= allowed_ids:
            parser.error(f'Unknown IDs for {args.task}; valid IDs: ' + ', '.join(sorted(allowed_ids)))
        if args.task == 'nonhuman10' and len(args.ids) != len(set(args.ids)):
            parser.error('--ids contains duplicate identifiers')
    if args.variants and args.task not in ('smoke', 'base'):
        parser.error('--variants is only for smoke/base')
    if args.steps < 1:
        parser.error('--steps must be positive')
    if args.task == 'assets':
        return asset_check()
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    output = (ROOT / (args.output or f'datasets/reproduction/{args.task}_{timestamp}')).resolve()
    jobs = build_jobs(args, presets, output)
    if args.dry_run:
        print('Dry run: execution uses the Python running this entry; run it with scripts/arena_python.sh for simulation.')
        print('OUTPUT', output)
        for job in jobs:
            print(shlex.join(job['command']))
        return 0
    if importlib.util.find_spec('isaaclab_arena') is None:
        parser.error('Arena is not available in this Python. Run: bash scripts/arena_python.sh scripts/reproduce_project.py ...')
    from scripts.collect_obstacle_library import now, run_command, verify_attempt, write_json

    # Existing results must not be overwritten or mistaken for a fresh run.
    output.mkdir(parents=True, exist_ok=False)
    scope = (presets['scope'] if args.task == 'human4' else
             presets['nonhuman10']['scope'] if args.task == 'nonhuman10' else
             'Load/physics/three-camera smoke only; no grasp/place success claim.' if args.task == 'smoke' else
             'Three default safety scenes with current code. Historical passes do not establish success under the corrected collision geometry; inspect this run.')
    report = {'task': args.task, 'scope': scope, 'started_at': now(),
              'seed': presets['seed'], 'complete': False, 'passed': False, 'jobs': []}
    report_path = output / 'reproduction.json'
    write_json(report_path, report)
    for job in jobs:
        log_path = output / (job['id'] + '.log')
        print('START', job['id'], 'log=', log_path, flush=True)
        execution = run_command(job['command'], log_path)
        execution['command'] = job['command']
        item = {**job, 'execution': execution, 'passed': False}
        if args.task == 'base':
            directory = Path(job['directory'])
            directory.mkdir(parents=True, exist_ok=True)
            write_json(directory / 'execution.json', execution)
            verification = verify_attempt(directory, {'id': '', 'category': job['variant']},
                                          presets['seed'], True, execution)
            item['passed'] = verification['passed']
            item['verification'] = str(directory / 'verification.json')
        elif args.task == 'smoke':
            result_path = Path(job['directory']) / 'smoke.json'
            item['passed'] = (execution['exit_code'] == 0 and result_path.is_file()
                              and read_json(result_path).get('passed') is True)
        elif args.task == 'nonhuman10':
            manifest_path = output / 'episodes/manifest.json'
            assets = (read_json(manifest_path).get('assets', [])
                      if manifest_path.is_file() else [])
            accepted = {asset['obstacle_id']: asset.get('passed') is True for asset in assets}
            item['asset_results'] = [{'id': identifier,
                                     'passed': accepted.get(identifier, False)}
                                    for identifier in job['selected_ids']]
            item['passed'] = (execution['exit_code'] == 0
                              and all(asset['passed'] for asset in item['asset_results']))
            item['manifest'] = str(manifest_path)
        else:
            manifest_path = output / 'episodes/manifest.json'
            if execution['exit_code'] == 0 and manifest_path.is_file():
                assets = read_json(manifest_path).get('assets', [])
                item['passed'] = any(asset['id'] == job['id'] and asset.get('accepted')
                                     for asset in assets)
            item['manifest'] = str(manifest_path)
        report['jobs'].append(item)
        write_json(report_path, report)
        print('FINISH', job['id'], 'passed=', item['passed'], flush=True)
    report.update(complete=True, finished_at=now(), passed=all(item['passed'] for item in report['jobs']))
    write_json(report_path, report)
    print('REPRODUCTION_FINISH', 'passed=', report['passed'], 'report=', report_path, flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
