"""Try explicit physical grasps, then replay successful ones with original cameras.

Every candidate resets the simulator in a separate worker. This is a bounded
search at seed 42, not a success-rate estimate. Earlier failures are retained.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.collect_obstacle_library import now, read_json, run_command, verify_attempt, write_json
from kitchen_safety.obstacle_library import get_asset

IDS = ['human_medium_shirt_raised_wrist', 'human_compact_sport_palm_down',
       'human_compact_sport_raised_wrist', 'human_broad_workwear_palm_down',
       'human_broad_workwear_raised_wrist']
CANDIDATES = [
    ('control_current', None, None, 0.),
    ('tilt45_yaw0', 45., 0., 0.),
    ('higher_10mm', 35., 0., .010),
    ('higher_20mm', 35., 0., .020),
    ('yaw_minus30', 35., -30., 0.),
    ('yaw_plus30', 35., 30., 0.),
    ('tilt25_yaw0', 25., 0., 0.),
    ('lower_10mm', 35., 0., -.010),
    ('higher_30mm', 35., 0., .030),
    ('higher_40mm', 35., 0., .040),
    ('higher_40mm_tilt45', 45., 0., .040),
    ('higher_50mm_tilt45', 45., 0., .050),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ids', nargs='+', default=IDS)
    parser.add_argument('--candidates', nargs='+', choices=[x[0] for x in CANDIDATES])
    parser.add_argument('--output', default='datasets/kitchen_auto/human_grasp_search')
    args = parser.parse_args()
    output = ROOT / args.output
    output.mkdir(parents=True, exist_ok=True)
    candidate_by_name = {x[0]: x for x in CANDIDATES}
    candidate_names = list(dict.fromkeys(args.candidates or candidate_by_name))
    candidates = [candidate_by_name[name] for name in candidate_names]
    identifiers = list(dict.fromkeys(args.ids))
    entries = {identifier: get_asset(identifier) for identifier in identifiers}
    manifest_path = output / 'manifest.json'
    report = read_json(manifest_path)
    if report is None:
        report = {'scope': 'Physical alternative-grasp search at fixed layout and seed 42; '
                           'video replay plus state and geometry verification required for acceptance',
                  'started_at': now(), 'assets': []}
    elif not isinstance(report, dict) or 'read_error' in report:
        raise RuntimeError(f'Cannot read existing search manifest: {manifest_path}: {report}')
    assets_by_id = {asset['id']: asset for asset in report['assets']}
    search_run = {
        'started_at': now(), 'complete': False,
        'ids': identifiers, 'candidates': candidate_names,
        'selected_asset_count': len(identifiers),
        'selected_candidate_count': len(candidates),
        'completed_asset_count': 0, 'accepted_count': 0,
        'skipped_accepted_count': 0, 'visited_candidate_count': 0,
        'execution_count': 0, 'reused_verification_count': 0,
    }
    report.setdefault('search_runs', []).append(search_run)
    report['complete'] = False
    report.pop('completed_at', None)

    def save_report():
        report['accepted_count'] = sum(asset.get('accepted') is not None
                                       for asset in report['assets'])
        write_json(manifest_path, report)

    save_report()

    def trial(entry, grasp, video):
        name, tilt, yaw, dz = grasp
        directory = output / entry['id'] / name / ('video' if video else 'probe')
        saved = read_json(directory / 'verification.json')
        if saved is not None:
            if not isinstance(saved, dict) or 'read_error' in saved or 'passed' not in saved:
                raise RuntimeError(f'Cannot reuse saved verification: {directory}: {saved}')
            search_run['reused_verification_count'] += 1
            print('REUSE', entry['id'], name, 'video' if video else 'probe',
                  'passed=', saved['passed'], flush=True)
            return dict(saved, directory=str(directory.relative_to(ROOT)))
        if directory.exists():
            raise RuntimeError(f'Unfinished trial directory has no verification.json: {directory}. '
                               'Finish or recover its verification before continuing; '
                               'the existing trial will not be overwritten.')
        directory.mkdir(parents=True)
        command = [sys.executable, str(ROOT / 'scripts/collect_kitchen.py'),
                   '--variant', entry['category'], '--obstacle_id', entry['id'],
                   '--planner', 'curobo', '--seed', '42', '--output', str(directory)]
        if tilt is not None:
            command += ['--grasp-tilt-deg', str(tilt), '--grasp-yaw-deg', str(yaw),
                        '--grasp-offset-z-m', str(dz)]
        if video:
            command.append('--video')
        print('START', entry['id'], name, 'video' if video else 'probe', flush=True)
        search_run['execution_count'] += 1
        save_report()
        execution = run_command(command, directory / 'run.log')
        execution.update(command=command, grasp_candidate=name)
        write_json(directory / 'execution.json', execution)
        verification = verify_attempt(directory, entry, 42, video, execution)
        print('FINISH', entry['id'], name, 'video' if video else 'probe',
              'passed=', verification['passed'], 'phase=', verification['failure_phase'],
              'frames=', verification['frames'], flush=True)
        return dict(verification, directory=str(directory.relative_to(ROOT)))

    try:
        for identifier in identifiers:
            entry = entries[identifier]
            asset = assets_by_id.get(identifier)
            if asset is None:
                asset = {'id': identifier, 'trials': [], 'accepted': None}
                report['assets'].append(asset)
                assets_by_id[identifier] = asset
            if asset.get('accepted') is not None:
                search_run['skipped_accepted_count'] += 1
                print('SKIP_ACCEPTED', identifier, json.dumps(asset['accepted']), flush=True)
            else:
                trials_by_name = {item['candidate']: item for item in asset['trials']}
                for grasp in candidates:
                    name, tilt, yaw, dz = grasp
                    item = trials_by_name.get(name)
                    if item is None:
                        item = {'candidate': name, 'tilt_deg': tilt,
                                'yaw_deg': yaw, 'offset_z_m': dz}
                        asset['trials'].append(item)
                        trials_by_name[name] = item
                    search_run['visited_candidate_count'] += 1
                    item['probe'] = trial(entry, grasp, False)
                    save_report()
                    if item['probe']['passed']:
                        item['video'] = trial(entry, grasp, True)
                        if item['video']['passed']:
                            asset['accepted'] = {'candidate': name,
                                                 'directory': item['video']['directory'],
                                                 'tilt_deg': tilt, 'yaw_deg': yaw,
                                                 'offset_z_m': dz}
                    save_report()
                    if asset.get('accepted') is not None:
                        break
            search_run['completed_asset_count'] += 1
            search_run['accepted_count'] += int(asset.get('accepted') is not None)
            save_report()
            print('ASSET_FINISH', identifier, json.dumps(asset.get('accepted')), flush=True)
    except Exception as exc:
        search_run['error'] = f'{type(exc).__name__}: {exc}'
        search_run['failed_at'] = now()
        save_report()
        raise
    search_run['complete'] = True
    search_run['completed_at'] = now()
    report['complete'] = True
    report['completed_at'] = search_run['completed_at']
    save_report()
    print('SEARCH_COMPLETE', search_run['accepted_count'], '/', len(identifiers), flush=True)
    return 0 if search_run['accepted_count'] == len(identifiers) else 1


if __name__ == '__main__':
    raise SystemExit(main())
