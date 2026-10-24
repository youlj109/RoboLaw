"""Collect fresh short context probes for declared articulation operations.

Uses measured holds, real physics and offline gDist. No task execution, risk
obstacle placement or production quota acceptance is inferred from this probe.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.task_contract import digest
from kitchen_safety.task_phase_probe_policy import validate_contexts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--asset-cache', type=Path, required=True)
    parser.add_argument('--tasks', nargs='+')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--timeout-s', type=float, default=600)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.asset_cache = args.asset_cache.resolve()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        raise ValueError('Positive stage timeout required')
    catalog = json.loads((ROOT / 'configs/risk_task_semantics.json').read_text())['tasks']
    tasks = args.tasks or sorted(catalog)
    if len(set(tasks)) != len(tasks) or any(t not in catalog for t in tasks):
        raise ValueError('Each requested task must have a unique declared operation scope')
    args.output.mkdir(parents=True, exist_ok=False)
    report_path = args.output / 'suite_probe_report.json'
    report = dict(complete=False, all_passed=False, task_ids=tasks, tasks={},
                  purpose='conditional operation-surface routing only', full_task_trajectory=False,
                  counts_toward_200_quota=False, production_batch=False)

    def save():
        tmp = report_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        tmp.replace(report_path)

    save()
    for task in tasks:
        contract_path = ROOT / 'configs/risk_task_suite' / (task + '.json')
        contract = json.loads(contract_path.read_text())
        operations = [o['operation_id'] for o in contract['operations'] if 'articulation_id' in o]
        contexts = [dict(phase='settle', operation_id=None)]
        for op in operations:
            contexts += [dict(phase=phase, operation_id=oid) for phase, oid in
                         [('operate', op), ('operate', None), ('settle', op), ('operate', op), ('settle', None)]]
        config = dict(schema_version='robolaw-phase-scope-probe-1', task_sha256=digest(contract), contexts=contexts)
        validate_contexts(contract, config)
        assert catalog[task]['task_contract_sha256'] == digest(contract)
        config_path = args.output / (task + '.probe-policy.json')
        config_path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
        run = args.output / task
        entry = dict(passed=False, complete=False, stages={}, task_sha256=digest(contract), config_sha256=digest(config))
        report['tasks'][task] = entry
        save()
        for stage in ('capture', 'audit', 'evaluate', 'verify'):
            if stage == 'verify':
                command = [sys.executable, 'scripts/verify_operation_scope_probe.py', '--episode', str(run / 'capture/native_000/raw_episode'),
                           '--risk', str(run / 'risk/native_000'), '--output', str(run / 'operation_scope_verification.json')]
            else:
                command = [sys.executable, 'scripts/risk_capture_workflow.py', stage, '--output', str(run)]
                if stage == 'capture':
                    command += ['--task-contract', str(contract_path), '--asset-cache', str(args.asset_cache),
                                '--policy-factory', 'kitchen_safety.task_phase_probe_policy:create',
                                '--policy-config', str(config_path), '--max-actions', str(len(contexts)), '--seed', str(args.seed)]
            log_path = args.output / (task + '.' + stage + '.log')
            started = time.perf_counter()
            timeout = False
            with log_path.open('x') as log:
                process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                entry['running_stage'] = stage
                entry['child_pid'] = process.pid
                save()
                try:
                    process.wait(timeout=args.timeout_s)
                except subprocess.TimeoutExpired:
                    timeout = True
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            entry.pop('running_stage', None)
            entry.pop('child_pid', None)
            entry['stages'][stage] = dict(exit_code=process.returncode, timeout=timeout, wall_s=time.perf_counter() - started)
            save()
            print(task, stage, json.dumps(entry['stages'][stage]), flush=True)
            if process.returncode or timeout:
                break
        entry['complete'] = True
        entry['passed'] = len(entry['stages']) == 4 and all(x['exit_code'] == 0 and not x['timeout'] for x in entry['stages'].values())
        save()
    report['complete'] = True
    report['all_passed'] = all(x['passed'] for x in report['tasks'].values())
    save()
    print(json.dumps(dict(complete=True, all_passed=report['all_passed'], tasks=len(tasks))), flush=True)
    return 0 if report['all_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
