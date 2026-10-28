"""Task-bound eleven-variant study; raw capture and offline risk are separate.

Run with the project's Python inside its existing runtime container. New study
outputs never replace old episodes. Configurations must already be prepared.
"""
import argparse
import concurrent.futures
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import traceback

ROOT = Path('/workspace')
sys.path.insert(0, str(ROOT))
from kitchen_safety.task_variant_bundle import prepare
from kitchen_safety.task_contract import digest

def read(p):
    return json.loads(Path(p).read_text())

def write(p, value):
    p = Path(p)
    tmp = p.with_suffix(p.suffix + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(p)

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()

def compact(run):
    p = run / 'risk/native_000/safety_decisions.jsonl'
    if p.stat().st_size < 100_000_000:
        return None
    original = sha(p)
    target = p.with_suffix('.jsonl.gz')
    tmp = target.with_suffix('.gz.tmp')
    assert not target.exists()
    with p.open('rb') as src, gzip.open(tmp, 'wb', compresslevel=1) as dst:
        shutil.copyfileobj(src, dst, 1 << 20)
    h = hashlib.sha256()
    with gzip.open(tmp, 'rb') as src:
        for block in iter(lambda: src.read(1 << 20), b''):
            h.update(block)
    assert h.hexdigest() == original
    tmp.replace(target)
    receipt = dict(archive=str(target), sha256_uncompressed=original,
                   bytes_uncompressed=p.stat().st_size, bytes_compressed=target.stat().st_size,
                   source_raw_unchanged=True)
    write(p.parent / 'lossless_compression.json', receipt)
    p.unlink()
    return receipt

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--mode', choices=['probe', 'motion'], required=True)
    parser.add_argument('--variants', nargs='+')
    parser.add_argument('--workers', type=int, default=2, choices=[1, 2])
    args = parser.parse_args()
    plan = read(args.config / 'study.json')
    selected = args.variants or list(plan['variants'])
    assert len(set(selected)) == len(selected) and set(selected) <= set(plan['variants'])
    base = args.study.resolve()
    base.mkdir(parents=True, exist_ok=False)
    (base / 'logs').mkdir()
    task = ROOT / 'configs/risk_task_suite' / (plan['task_id'] + '.json')
    contract = read(task)
    assert digest(contract) == plan['task_sha256']
    report = dict(task_id=plan['task_id'], mode=args.mode, pid=os.getpid(), complete=False,
                  required=selected, started_wall_ns=time.time_ns(), variants={},
                  study_scope='One explicit layout/seed per variant; no 200-episode or random-range acceptance')
    lock = threading.RLock()
    def save():
        with lock:
            write(base / 'study_report.json', report)
    save()

    def worker(vid):
        entry = dict(status='preparing', stages={}, config_revision=plan['variants'][vid]['revision'])
        with lock:
            report['variants'][vid] = entry
            save()
        try:
            bundle = base / (vid + '.bundle')
            prepare(ROOT, ROOT / 'configs/risk_task_suite/suite.json',
                    ROOT / 'configs/obstacle_library.json', ROOT / 'configs/risk_human_assets.json',
                    plan['task_id'], vid, args.config / plan['variants'][vid]['placement'], bundle)
            run = base / vid
            entry['run'] = str(run)
            config = base / (vid + '.policy.json')
            if args.mode == 'probe':
                factory = 'kitchen_safety.task_hold_probe_policy:create'
                write(config, dict(version='robolaw-measured-hold-probe-1', task_sha256=digest(contract), actions=3))
                limit = 3
            else:
                factory = 'kitchen_safety.task_review_policy:create'
                write(config, dict(version='robolaw-task-review-policy-1', task_sha256=digest(contract),
                                   joint_speed_rad_s=.45, joint_acceleration_rad_s2=.8,
                                   purpose='One measured G-banana obstacle task attempt; safety only evaluated offline'))
                limit = 2400
            for stage in ['capture', 'audit', 'evaluate', 'verify']:
                if stage == 'verify':
                    cmd = [sys.executable, str(ROOT / 'scripts/verify_prepared_variant.py'),
                           '--bundle', str(bundle), '--run', str(run), '--output', str(base / (vid + '.verification.json'))]
                else:
                    cmd = [sys.executable, str(ROOT / 'scripts/risk_capture_workflow.py'), stage, '--output', str(run)]
                    if stage == 'capture':
                        cmd += ['--task-contract', str(bundle / 'task_contract.json'), '--obstacle-binding', str(bundle / 'obstacle_binding.json'),
                                '--policy-factory', factory, '--policy-config', str(config), '--asset-cache', plan['asset_cache'],
                                '--max-actions', str(limit), '--seed', str(plan['seed'])]
                # Two bounded independent pipelines, using prior verified runtime capacity.
                if stage in ['capture', 'evaluate']:
                    while int(subprocess.check_output(['nvidia-smi', '--query-gpu=memory.free', '--format=csv,noheader,nounits'], text=True).strip()) < 12000:
                        with lock:
                            entry['resource_wait'] = 'Waiting for 12000 MiB free GPU memory'
                            save()
                        time.sleep(5)
                log = base / 'logs' / (vid + '.' + stage + '.log')
                tick = time.perf_counter()
                with log.open('x') as stream:
                    child = subprocess.Popen(cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
                    with lock:
                        entry.pop('resource_wait', None)
                        entry.update(status='running', current_stage=stage, child_pid=child.pid)
                        save()
                    code = child.wait()
                with lock:
                    entry.pop('current_stage', None)
                    entry.pop('child_pid', None)
                    entry['stages'][stage] = dict(exit_code=code, wall_s=time.perf_counter()-tick, log=str(log))
                    save()
                if code:
                    raise RuntimeError(stage + ' failed; preserve original output/log')
            raw = read(run / 'capture/native_000/raw_episode/raw_capture_summary.json')
            evaluation = read(run / 'risk/native_000/evaluation_summary.json')
            placement = raw['placement_validation']
            layout = placement.get('passed') is True and not placement.get('fallback_used')
            with lock:
                entry.update(status='validated' if layout and raw['execution_error'] is None else 'layout_or_execution_rejected',
                             placement_validation=placement, task_success=raw['task_success'],
                             execution_error=raw['execution_error'], stop_reason=raw['stop_reason'],
                             physics_steps=raw['physics_steps'], frames=evaluation['frames'],
                             raw_sha256=raw['raw_writer']['sha256'], risk_level_counts=evaluation['risk_level_counts'],
                             task_success_separate_from_risk=True,
                             full_task_success_validated=raw['task_success'] is True,
                             all_risk_evidence_complete=False)
                save()
            archive = compact(run)
            with lock:
                entry['risk_archive'] = archive
                save()
        except BaseException as exc:
            with lock:
                entry.update(status='failed', error=repr(exc), traceback=traceback.format_exc())
                save()
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(worker, v) for v in selected]
        for future in concurrent.futures.as_completed(futures):
            future.result()
    report.update(complete=True, ended_wall_ns=time.time_ns(),
                  valid_layout_and_pipeline_count=sum(x['status']=='validated' for x in report['variants'].values()))
    save()
    print(json.dumps(report, ensure_ascii=False), flush=True)

if __name__ == '__main__':
    main()
