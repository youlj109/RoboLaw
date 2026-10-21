"""Measure explicit episode specs across independent persistent workers.

    bash scripts/arena_python.sh scripts/benchmark_collection_workers.py \
        --workers 2 --output reports/collection_speed_20261018/two_workers

These are performance measurements, never production quota. The default
repeats the input list per worker; --distribution shard assigns each input
once by index % workers. Timing includes app startup, cleanup and shutdown.
App initialization is serialized by default; collection remains concurrent.
Use --concurrent-startup to measure fully concurrent initialization instead.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import re
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.collector_worker import CollectorWorker
from scripts.collect_kitchen_sequence import prepare_entries, write_json


def now():
    return datetime.now(timezone.utc).isoformat()


def redirected_entries(data, directory, indices=None):
    """Replace only --output; geometry, seeds and rendering options are retained."""
    if not isinstance(data, list) or not data:
        raise ValueError('Input must be a nonempty episode list')
    entries = []
    for index, item in enumerate(data):
        if indices is not None and index not in indices:
            continue
        if not isinstance(item, dict) or not isinstance(item.get('argv'), list):
            raise ValueError('Each episode needs an argv list')
        argv = list(item['argv'])
        identifier = str(item.get('id', index))
        label = re.sub(r'[^A-Za-z0-9_-]+', '_', identifier).strip('_') or 'episode'
        destination = directory / f'{index:03d}_{label}'
        replacements = 0
        for position, value in enumerate(argv):
            if value == '--output':
                if position + 1 == len(argv):
                    raise ValueError('--output needs a value')
                argv[position + 1] = str(destination)
                replacements += 1
            elif value.startswith('--output='):
                argv[position] = '--output=' + str(destination)
                replacements += 1
        if replacements != 1:
            raise ValueError('Each episode must have exactly one --output')
        prepared = prepare_entries([{'id': identifier, 'argv': argv}])[0]
        prepared.update(source=item.get('source'), source_episode_index=index)
        entries.append(prepared)
    return entries


def benchmark(input_path, output, *, workers=2, worker_factory=None, request_timeout_s=3600.,
              distribution='repeat', concurrent_startup=False):
    input_path, output = Path(input_path).resolve(), Path(output).resolve()
    if workers < 1:
        raise ValueError('--workers must be positive')
    if distribution not in ('repeat', 'shard'):
        raise ValueError('distribution must be repeat or shard')
    data = json.loads(input_path.read_text())
    groups = [redirected_entries(data, output / f'worker_{index:02d}',
                                None if distribution == 'repeat' else set(range(index, len(data), workers)))
              for index in range(workers)]
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'input.json', data)
    factory = worker_factory or CollectorWorker
    startup_lock = None if concurrent_startup else threading.Lock()
    started = time.perf_counter()
    report = {'started_at': now(), 'input': str(input_path), 'workers_requested': workers,
              'distribution': distribution,
              'concurrent_startup': bool(concurrent_startup),
              'episodes_per_worker': len(data) if distribution == 'repeat' else None,
              'episodes_requested_per_worker': [len(entries) for entries in groups],
              'requested_total': sum(map(len, groups)),
              'production_quota_eligible': False, 'data_validation_run': False,
              'semantics': 'Explicit existing episode specs for throughput measurement, not new production samples or policy success-rate evaluation',
              'timing_semantics': 'Total wall time includes startup waits, worker startup, concurrent collection, cleanup and app shutdown',
              'status': 'running', 'workers': []}
    write_json(output / 'benchmark.json', report)

    def run_worker(index):
        directory = output / f'worker_{index:02d}'
        directory.mkdir()
        entries = groups[index]
        write_json(directory / 'input.json', entries)
        worker_started = time.perf_counter()
        record = {'worker_index': index, 'started_at': now(), 'episodes': [], 'status': 'running'}
        client = None
        try:
            client = factory(cwd=ROOT, request_timeout_s=request_timeout_s,
                             kit_log_path=directory / 'kit.log', startup_lock=startup_lock)
            with client:
                for entry in entries:
                    destination = Path(entry['output'])
                    destination.mkdir()
                    command = [sys.executable, str(ROOT / 'scripts/collect_kitchen.py'), *entry['argv']]
                    execution = client.execute(command, destination / 'run.log')
                    write_json(destination / 'execution.json', dict(execution, command=command))
                    result = {}
                    if (destination / 'result.json').exists():
                        try:
                            result = json.loads((destination / 'result.json').read_text())
                        except (OSError, ValueError) as error:
                            result = {'read_error': repr(error)}
                    physical = result.get('success') is True
                    episode = {'id': entry['id'], 'output': str(destination), 'source': entry['source'],
                               'source_episode_index': entry['source_episode_index'],
                               'execution': execution, 'physical_success': physical,
                               'successful_execution': execution.get('exit_code') == 0 and physical,
                               'steps': result.get('steps'), 'failure_kind': result.get('failure_kind'),
                               'error': result.get('error') or result.get('read_error') or execution.get('error'),
                               'collector_timings_seconds': result.get('timings_seconds'),
                               'production_quota_eligible': False}
                    record['episodes'].append(episode)
                    record['elapsed_seconds'] = time.perf_counter() - worker_started
                    write_json(directory / 'worker_report.json', record)
                    print(f'BENCHMARK worker={index} id={entry["id"]} '
                          f'physical_success={physical} seconds={execution["elapsed_seconds"]:.3f}', flush=True)
                    if execution.get('worker_unavailable') or execution.get('storage_error'):
                        record['stopped_reason'] = execution.get('error') or execution.get('storage_error')
                        break
        except Exception:
            record['error'] = traceback.format_exc()
        finally:
            record['shutdown'] = getattr(client, 'shutdown', None)
            # execute() already saved request logs. Retain remaining native
            # shutdown output that arrived after the final protocol reply.
            pending = getattr(client, '_lines', None)
            with (directory / 'shutdown.log').open('w') as log:
                if pending is not None:
                    while True:
                        try:
                            kind, value = pending.get_nowait()
                        except queue.Empty:
                            break
                        if value is not None:
                            log.write(value if kind == 'line' else f'{kind}: {value}\n')
            record['elapsed_seconds'] = time.perf_counter() - worker_started
            record['completed_at'] = now()
            record['status'] = ('completed' if len(record['episodes']) == len(entries)
                                and not record.get('error') else 'stopped')
            write_json(directory / 'worker_report.json', record)
        return record

    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(run_worker, index): index for index in range(workers)}
        for future in as_completed(futures):
            try:
                result = future.result()
            except Exception:
                result = {'worker_index': futures[future], 'status': 'stopped', 'episodes': [],
                          'error': traceback.format_exc()}
            report['workers'].append(result)
            write_json(output / 'benchmark.json', report)
    report['workers'].sort(key=lambda row: row['worker_index'])
    elapsed = time.perf_counter() - started
    episodes = [episode for worker in report['workers'] for episode in worker['episodes']]
    successes = sum(episode['successful_execution'] for episode in episodes)
    report.update(status='completed' if len(episodes) == report['requested_total'] else 'stopped',
                  completed_at=now(), total_wall_seconds=elapsed, attempted=len(episodes),
                  physical_successes=sum(episode['physical_success'] for episode in episodes),
                  successful_collectors=successes,
                  successful_trajectories_per_hour=successes * 3600 / elapsed if elapsed else 0.,
                  passed=successes == report['requested_total'] and all(
                      worker['status'] == 'completed' for worker in report['workers']))
    write_json(output / 'benchmark.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', type=Path,
                        default=ROOT / 'reports/collection_speed_20261018/gpu_cameras3_input.json')
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--distribution', choices=('repeat', 'shard'), default='repeat')
    parser.add_argument('--concurrent-startup', action='store_true',
                        help='Launch Kit apps concurrently; default serializes only app initialization')
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing directories are not reused')
    parser.add_argument('--request-timeout-s', type=float, default=3600.)
    args = parser.parse_args()
    try:
        report = benchmark(args.input, args.output, workers=args.workers,
                           request_timeout_s=args.request_timeout_s, distribution=args.distribution,
                           concurrent_startup=args.concurrent_startup)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({key: report[key] for key in (
        'total_wall_seconds', 'attempted', 'physical_successes', 'successful_collectors',
        'successful_trajectories_per_hour', 'production_quota_eligible', 'passed')}, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
