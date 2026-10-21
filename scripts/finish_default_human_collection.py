"""Resume only the default MPFB scene until 20 recorded safe successes exist.

Includes the three historical successes. Benchmark eligibility remains separate;
no original verification is changed. Run inside the existing Arena environment.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from summarize_human_collection import ROOT, SCENE, summarize


def process_is_running(pid, output_name):
    try:
        command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
        state = Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[0]
    except FileNotFoundError:
        return False
    return state != 'Z' and 'collect_obstacle_library.py' in command and output_name in command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'datasets/augmentation/human_default20_n4_20261019')
    parser.add_argument('--historical', type=Path, default=ROOT / 'datasets/augmentation/human7_n4_20261019/episodes')
    parser.add_argument('--observer-config', help='New read-only observer batch; historical data excluded')
    parser.add_argument('--run-id')
    parser.add_argument('--wait-for-pid', type=int)
    parser.add_argument('--target', type=int, default=20)
    parser.add_argument('--max-attempts', type=int, default=100)
    args = parser.parse_args()
    output, historical = args.output.resolve(), args.historical.resolve()
    if args.observer_config:
        if output == (ROOT / 'datasets/augmentation/human_default20_n4_20261019').resolve():
            parser.error('Observer collection requires a new --output directory')
        historical = output / '_no_historical_observer_data'
        saved_manifest=output/'episodes/manifest.json'
        if saved_manifest.exists():
            saved=json.loads(saved_manifest.read_text())
            from kitchen_safety.risk_profile_loader import load_config
            previous=saved.get('configuration',{})
            if previous.get('safety_observer')!=load_config(args.observer_config) or previous.get('run_id')!=(args.run_id or str(output)):
                parser.error('Observer config/profile/code/run differs from existing data; use a new output directory')
        if (output / 'successful_episodes.json').exists() and not (output / 'episodes/manifest.json').exists():
            parser.error('Existing output is not an observer batch; use a new directory')
    output.mkdir(parents=True, exist_ok=True)
    previous_count = None

    def progress():
        nonlocal previous_count
        report = summarize(output, historical, args.target, write=True)
        if report['success_count'] != previous_count:
            print(json.dumps({k: report[k] for k in ('success_count', 'target_count', 'deficit',
                'historical_success_count', 'new_success_count', 'benchmark_passed_count')}, ensure_ascii=False), flush=True)
            previous_count = report['success_count']
        return report

    if args.wait_for_pid:
        while process_is_running(args.wait_for_pid, output.name):
            progress()
            time.sleep(15)
    while True:
        report = progress()
        if report['complete']:
            print('DEFAULT_HUMAN_COLLECTION_COMPLETE', flush=True)
            return 0
        raw = output / 'episodes'
        manifest_path = raw / 'manifest.json'
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        if manifest.get('stop_reason') in ('storage_error', 'worker_unavailable'):
            print(json.dumps({'stopped': manifest['stop_reason'], 'details': manifest.get(manifest['stop_reason'])}), flush=True)
            return 74 if manifest['stop_reason'] == 'storage_error' else 75
        attempts = len(list((raw / SCENE).glob('attempt_*')))
        if attempts >= args.max_attempts:
            print('ATTEMPT_BUDGET_EXHAUSTED', flush=True)
            return 1
        # Strict benchmark target stays unchanged. The separate completion
        # selection drives additional attempts without relabelling eligibility.
        raw_target = int(manifest.get('episodes_per_id', args.target - report['historical_success_count']))
        budget = min(args.max_attempts, max(raw_target, attempts + report['deficit']))
        command = [sys.executable, str(ROOT / 'scripts/collect_obstacle_library.py'),
            '--ids', SCENE, '--randomization-config', 'configs/augmentation_human7.json',
            '--seed', '43', '--episodes-per-id', str(raw_target), '--max-attempts', str(budget),
            '--record-rgb', '--preview-per-id', '0',
            '--placement-strategy', 'lower-first', '--grasp-tilt-deg', '35', '--grasp-yaw-deg', '0',
            '--grasp-offset-z-m', '0', '--grasp-offset-y-m', '-0.01',
            '--transport-bottle-height-m', '1.11', '--supported-release', '--transport-tilt-cap-deg', '25',
            '--execution-backend', 'vectorized', '--pipeline', '--num-envs', '4',
            '--validation-workers', '4', '--device', 'cpu', '--physics-threads', '0',
            '--torch-threads', '1', '--camera-device', 'cuda:0', '--background-rgb',
            '--execution-checks', 'deferred', '--episode-length-s', '180', '--output', str(raw)]
        if args.observer_config:
            command.extend(['--observer-config', args.observer_config, '--run-id', args.run_id or str(output)])
        if manifest_path.exists():
            command.append('--resume')
        index = 0
        while (output / f'continuation_{index:03d}.log').exists():
            index += 1
        log_path = output / f'continuation_{index:03d}.log'
        print(f'RESUME attempts={attempts} cumulative_budget={budget} log={log_path}', flush=True)
        with log_path.open('x') as log:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            try:
                while child.poll() is None:
                    progress()
                    time.sleep(15)
            except BaseException:
                child.send_signal(signal.SIGINT)
                child.wait()
                raise
        if child.returncode not in (0, 1):
            progress()
            print(f'COLLECTOR_EXIT {child.returncode}', flush=True)
            return child.returncode


if __name__ == '__main__':
    raise SystemExit(main())
