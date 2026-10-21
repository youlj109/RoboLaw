"""Run all current kitchen layouts once, sequentially, without changing parameters.

Run with Arena Python. Each episode uses a separate headless Kit process.
This checks fixed-seed physical feasibility, not success rate or fresh RGB output.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.obstacle_library import load_assets
from scripts.collect_obstacle_library import (
    failed_phase, now, read_json, run_command, verify_attempt, write_json,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = (ROOT / args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    presets = json.loads((ROOT / 'configs/reproduction_presets.json').read_text())
    preset_by_id = {item['id']: item for item in presets['human4']}
    entries = load_assets()
    jobs = [{'id': 'base', 'category': 'base', 'label': '无障碍对照'}]
    for category in ('property', 'robot', 'human'):
        jobs.extend(dict(entry, obstacle_id=entry['id'])
                    for entry in entries if entry['category'] == category)
        jobs.append({'id': 'default_' + category, 'category': category,
                     'label': '默认 ' + category})
    report = {
        'scope': '20 current layouts, one full headless physical attempt each at seed 42; '
                 'current human4 presets, otherwise default automatic orientation and body localz0. '
                 'No parameter tuning, RGB validation, or success-rate estimate.',
        'started_at': now(), 'complete': False, 'seed': 42,
        'requested_count': len(jobs), 'running': None, 'jobs': [],
    }

    def publish():
        report['checked_count'] = len(report['jobs'])
        report['passed_count'] = sum(bool(item['passed']) for item in report['jobs'])
        report['updated_at'] = now()
        write_json(out / 'audit.json', report)

    publish()
    for job in jobs:
        identifier, category = job['id'], job['category']
        directory = out / identifier
        directory.mkdir()
        preset = preset_by_id.get(identifier, {})
        params = {'grasp_offset_z_m': 0.}
        if preset:
            params.update({key: preset[key] for key in
                           ('grasp_tilt_deg', 'grasp_yaw_deg', 'grasp_offset_z_m')})
        params['placement_strategy'] = 'lower-first' if preset else 'pose'
        if preset.get('placement_tcp_height_m') is not None:
            params['placement_tcp_height_m'] = preset['placement_tcp_height_m']
        command = [sys.executable, str(ROOT / 'scripts/collect_kitchen.py'),
                   '--variant', category, '--planner', 'curobo',
                   '--seed', '42', '--jitter', '0', '--output', str(directory)]
        if job.get('obstacle_id'):
            command.extend(['--obstacle_id', job['obstacle_id']])
        for key, value in params.items():
            command.extend(['--' + key.replace('_', '-'), str(value)])
        report['running'] = {'id': identifier, 'started_at': now(), 'directory': str(directory)}
        publish()
        print('START', len(report['jobs']) + 1, '/', len(jobs), identifier, flush=True)
        execution = run_command(command, directory / 'run.log')
        execution['command'] = command
        write_json(directory / 'execution.json', execution)
        result = read_json(directory / 'result.json', {})
        if not result or result.get('read_error'):
            report['infrastructure_error'] = {
                'id': identifier, 'execution': execution,
                'reason': result.get('read_error', 'Collector produced no result.json; see run.log'),
            }
            report['running'] = None
            publish()
            return 1
        if category != 'base':
            verification = verify_attempt(directory, job, 42, False, execution)
        else:
            physical = execution.get('exit_code') == 0 and result.get('success') is True
            data_check = {'exit_code': None, 'skipped_reason': 'Physical execution failed'}
            if physical:
                data_check = run_command([
                    sys.executable, str(ROOT / 'scripts/validate_collection.py'),
                    str(directory), '--report', str(directory / 'validation.json')],
                    directory / 'validation.log')
            data = read_json(directory / 'validation.json')
            data_ok = (data_check.get('exit_code') == 0 and isinstance(data, list) and len(data) == 1)
            verification = {
                'passed': bool(physical and data_ok), 'execution_success': physical,
                'data_success': data_ok, 'audit_pass': None,
                'audit_scope': 'Not applicable: base has no safety obstacle. Collector retains ordinary world/robot execution checks.',
                'checks': {'validation': data_check}, 'frames': result.get('steps'),
                'error': result.get('error'), 'failure_phase': failed_phase(result, directory),
                'video_requested': False,
            }
            write_json(directory / 'verification.json', verification)
        record = {
            'id': identifier, 'label': job['label'], 'category': category,
            'obstacle_id': job.get('obstacle_id'), 'parameters': params,
            'directory': str(directory.relative_to(out)),
            'passed': verification['passed'],
            'physical_success': result.get('success') is True,
            'data_success': verification['data_success'],
            'audit_pass': verification['audit_pass'],
            'frames': result.get('steps'), 'error': verification.get('error'),
            'failure_phase': verification.get('failure_phase'),
            'body_close': result.get('body_grasp_close', {}).get('both_fingers_near_body'),
            'body_lift': result.get('body_grasp_lift', {}).get('both_fingers_on_body'),
            'body_carried_region': result.get('body_grasp_carried_region', {}).get('passed'),
            'minimum_hazard_clearance_m': result.get('minimum_hazard_clearance_m'),
            'elapsed_seconds': execution.get('elapsed_seconds'),
            'command': command,
        }
        report['jobs'].append(record)
        report['running'] = None
        publish()
        print('FINISH', identifier, 'passed=', record['passed'],
              'phase=', record['failure_phase'], 'frames=', record['frames'], flush=True)
    report['complete'] = True
    report['finished_at'] = now()
    publish()
    print('AUDIT_FINISHED', report['passed_count'], '/', len(jobs), out / 'audit.json', flush=True)
    # A completed audit can contain failed tasks. Its JSON, not this exit code,
    # determines which tasks passed. Nonzero remains an infrastructure failure.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
