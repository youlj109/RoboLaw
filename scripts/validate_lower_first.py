"""Verify lower-first placement in four fixed human scenes, sequentially.

Run with Arena Python, for example::

    bash scripts/arena_python.sh scripts/validate_lower_first.py --label lower_first_v1

By default each label retains its probe and, only after the probe passes all
existing checks, a separate replay with the original three cameras. With
--video-only, run the complete physical episode with those cameras directly
and apply the same existing verification and audit; no probe is fabricated.
Repeating a label reuses saved verification records. Use a new label for a new
physical retry or validation mode.
Grasp pose and optional placement TCP height are recorded per attempt; the
default placement height keeps the collector's existing grasp-offset behavior.
This is a fixed-seed feasibility check, not a success-rate estimate.
"""
import argparse
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.obstacle_library import get_asset
from scripts.collect_obstacle_library import now, read_json, run_command, verify_attempt, write_json

IDS = [
    'human_medium_shirt_raised_wrist',
    'human_broad_workwear_raised_wrist',
    'human_compact_sport_palm_down',
    'human_compact_sport_raised_wrist',
]
GRASP = {'tilt_deg': 45., 'yaw_deg': 0., 'offset_z_m': 0.}


def relative(path):
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ids', nargs='+', choices=IDS, default=IDS)
    parser.add_argument('--label', default='bottle_body',
                        help='Attempt label; reuse to resume, change to retain a separate retry')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output', default='datasets/kitchen_auto/human_lower_first_validation')
    parser.add_argument('--video-only', action='store_true',
                        help='Directly record and verify a complete physical three-camera run, without a separate probe')
    parser.add_argument('--grasp-offset-z-m', type=float, default=GRASP['offset_z_m'])
    parser.add_argument('--grasp-tilt-deg', type=float, default=GRASP['tilt_deg'])
    parser.add_argument('--grasp-yaw-deg', type=float, default=GRASP['yaw_deg'])
    parser.add_argument('--placement-tcp-height-m', type=float, default=None,
                        help='Explicit lower-first placement TCP height; omit to retain collector default')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', args.label):
        parser.error('--label must be a single directory name starting with a letter or digit')
    identifiers = list(dict.fromkeys(args.ids))
    validation_mode = 'video_only' if args.video_only else 'probe_then_video'
    grasp = {'tilt_deg': args.grasp_tilt_deg, 'yaw_deg': args.grasp_yaw_deg,
             'offset_z_m': args.grasp_offset_z_m}
    candidate_parameters = dict(grasp, placement_tcp_height_m=args.placement_tcp_height_m)
    entries = {identifier: get_asset(identifier) for identifier in identifiers}
    output = (ROOT / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / 'manifest.json'
    report = read_json(manifest_path)
    if report is None:
        report = {
            'started_at': now(), 'assets': [],
        }
    elif not isinstance(report, dict) or 'read_error' in report:
        raise RuntimeError(f'Cannot preserve existing manifest {manifest_path}: {report}')
    report['scope'] = (
        'Complete physical lower-first placement at fixed scene layout. Each accepted attempt '
        'has a verified original-three-camera physical run, including the existing data and '
        'clearance audits. Per-trial validation_mode records probe_then_video (a separate '
        'verified probe is also required) or video_only (no separate probe). '
        'This is not a policy success-rate evaluation.')
    assets = {asset['id']: asset for asset in report['assets']}
    run = {
        'started_at': now(), 'complete': False,
        'runner': 'validate_lower_first.py', 'placement_strategy': 'lower-first',
        'validation_mode': validation_mode,
        'ids': identifiers, 'candidates': [args.label], 'label': args.label,
        'seed': args.seed, 'requested_grasp': dict(grasp),
        'placement_tcp_height_m': args.placement_tcp_height_m,
        'selected_asset_count': len(identifiers), 'selected_candidate_count': 1,
        'completed_asset_count': 0, 'accepted_count': 0,
        'skipped_accepted_count': 0, 'execution_count': 0,
        'reused_verification_count': 0, 'errors': [],
    }
    report.setdefault('search_runs', []).append(run)
    report['complete'] = False
    report.pop('completed_at', None)

    def publish():
        report['accepted_count'] = sum(asset.get('accepted') is not None
                                       for asset in report['assets'])
        report['all_passed'] = bool(report['assets']) and report['accepted_count'] == len(report['assets'])
        report['updated_at'] = now()
        write_json(manifest_path, report)

    def trial(entry, video):
        role = 'video' if video else 'probe'
        directory = output / entry['id'] / args.label / role
        saved = read_json(directory / 'verification.json')
        if saved is not None:
            if not isinstance(saved, dict) or 'read_error' in saved or 'passed' not in saved:
                raise RuntimeError(f'Cannot reuse saved verification: {directory}: {saved}')
            if (saved.get('video_requested') != video or saved.get('seed') != args.seed
                    or saved.get('obstacle_id') != entry['id']):
                raise ValueError(f'Saved verification does not match requested asset/seed/video: '
                                 f'{directory}; use a new --label for a different attempt')
            saved_command = (saved.get('execution') or {}).get('command')
            if saved_command:
                for flag, requested in (
                    ('--grasp-tilt-deg', args.grasp_tilt_deg),
                    ('--grasp-yaw-deg', args.grasp_yaw_deg),
                    ('--grasp-offset-z-m', args.grasp_offset_z_m),
                    ('--placement-tcp-height-m', args.placement_tcp_height_m),
                ):
                    actual = (float(saved_command[saved_command.index(flag) + 1])
                              if flag in saved_command else None)
                    if actual != requested:
                        raise ValueError(f'Saved collector argument {flag}={actual} differs from '
                                         f'requested {requested}: {directory}; use a new --label')
            run['reused_verification_count'] += 1
            print('REUSE', entry['id'], args.label, role, 'passed=', saved['passed'], flush=True)
            return dict(saved, directory=relative(directory))
        if directory.exists():
            raise RuntimeError(f'Unfinished attempt has no verification.json: {directory}. '
                               'Recover its original execution/verification or use a new --label; '
                               'existing files will not be overwritten.')
        directory.mkdir(parents=True)
        command = [
            sys.executable, str(ROOT / 'scripts/collect_kitchen.py'),
            '--variant', entry['category'], '--obstacle_id', entry['id'],
            '--planner', 'curobo', '--seed', str(args.seed), '--output', str(directory),
            '--placement-strategy', 'lower-first',
            '--grasp-tilt-deg', str(args.grasp_tilt_deg),
            '--grasp-yaw-deg', str(args.grasp_yaw_deg),
            '--grasp-offset-z-m', str(args.grasp_offset_z_m),
        ]
        if args.placement_tcp_height_m is not None:
            command.extend(['--placement-tcp-height-m', str(args.placement_tcp_height_m)])
        if video:
            command.append('--video')
        print('START', entry['id'], args.label, role, flush=True)
        run['execution_count'] += 1
        publish()
        execution = run_command(command, directory / 'run.log')
        execution.update(command=command, grasp_candidate=args.label,
                         placement_strategy='lower-first', validation_mode=validation_mode,
                         requested_grasp=dict(grasp),
                         placement_tcp_height_m=args.placement_tcp_height_m)
        write_json(directory / 'execution.json', execution)
        verification = verify_attempt(directory, entry, args.seed, video, execution)
        print('FINISH', entry['id'], args.label, role,
              'passed=', verification['passed'], 'phase=', verification['failure_phase'],
              'frames=', verification['frames'], flush=True)
        return dict(verification, directory=relative(directory))

    publish()
    for identifier in identifiers:
        asset = assets.get(identifier)
        if asset is None:
            asset = {'id': identifier, 'trials': [], 'accepted': None}
            report['assets'].append(asset)
            assets[identifier] = asset
        try:
            if asset.get('accepted') is not None:
                run['skipped_accepted_count'] += 1
                print('SKIP_ACCEPTED', identifier, json.dumps(asset['accepted']), flush=True)
            else:
                item = next((item for item in asset['trials'] if item['candidate'] == args.label), None)
                if item is None:
                    item = dict(candidate=args.label, label=args.label,
                                placement_strategy='lower-first', validation_mode=validation_mode,
                                seed=args.seed, **candidate_parameters)
                    asset['trials'].append(item)
                elif item.get('seed', args.seed) != args.seed:
                    raise ValueError(f'Saved label {args.label} for {identifier} has another seed; '
                                     'use a new --label')
                elif item.get('validation_mode', 'probe_then_video') != validation_mode:
                    raise ValueError(f'Saved label {args.label} for {identifier} has another validation mode; '
                                     'use a new --label')
                else:
                    # Older lower-first labels used the fixed GRASP defaults
                    # and no explicit placement TCP height.
                    saved_parameters = {key: item.get(key, GRASP.get(key))
                                        for key in candidate_parameters}
                    if saved_parameters != candidate_parameters:
                        raise ValueError(f'Saved label {args.label} for {identifier} has different '
                                         f'candidate parameters {saved_parameters}; requested '
                                         f'{candidate_parameters}; use a new --label')
                publish()
                if not args.video_only:
                    item['probe'] = trial(entries[identifier], False)
                    publish()
                if args.video_only or item['probe']['passed']:
                    item['video'] = trial(entries[identifier], True)
                    if item['video']['passed']:
                        asset['accepted'] = dict(candidate=args.label,
                                                 directory=item['video']['directory'],
                                                 placement_strategy='lower-first',
                                                 validation_mode=validation_mode,
                                                 seed=args.seed, **candidate_parameters)
                    publish()
            run['completed_asset_count'] += 1
            run['accepted_count'] += int(asset.get('accepted') is not None)
            print('ASSET_FINISH', identifier, json.dumps(asset.get('accepted')), flush=True)
        except Exception as error:
            detail = {'obstacle_id': identifier, 'label': args.label,
                      'error': f'{type(error).__name__}: {error}', 'recorded_at': now()}
            run['errors'].append(detail)
            print('ASSET_ERROR', json.dumps(detail), flush=True)
        publish()

    run['complete'] = run['completed_asset_count'] == len(identifiers)
    run['all_passed'] = run['complete'] and run['accepted_count'] == len(identifiers)
    run['finished_at'] = now()
    report['complete'] = run['complete']
    if run['complete']:
        run['completed_at'] = run['finished_at']
        report['completed_at'] = run['completed_at']
    publish()
    print('VALIDATION_FINISH', 'complete=', run['complete'],
          'accepted=', run['accepted_count'], '/', len(identifiers), flush=True)
    return 2 if run['errors'] else 0 if run['all_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
