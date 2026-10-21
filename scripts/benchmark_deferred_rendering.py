#!/usr/bin/env python3
"""Measure complete diagnostic physics -> offline RGB -> validation pipelines.

Run with host Python. Repeated episode specs are intentional performance inputs;
these results never count towards production collection quotas.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.render_worker import RenderWorker


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def run_owned_command(command, log_path, *, cwd=ROOT):
    """Wait through host interruption so docker exec cannot orphan our Kit.

    Each command is synchronous in its role. A host exception stops subsequent
    launches, but a process already launched is allowed to finish naturally.
    No process belonging to another experiment is signalled.
    """
    started = time.perf_counter()
    pending_error = None
    with Path(log_path).open('w') as log:
        process = subprocess.Popen(command, cwd=cwd, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        while True:
            try:
                exit_code = process.wait()
                break
            except BaseException as error:
                if pending_error is None:
                    pending_error = error
                # Deliberately do not terminate the docker-exec client: that
                # alone does not guarantee its container process has stopped.
    if pending_error is not None:
        raise pending_error
    return {'exit_code': exit_code, 'elapsed_seconds': time.perf_counter() - started,
            'pid': process.pid, 'process_reaped': True}


def partition_specs(specs, batch_size):
    """Preserve order; native environments in one batch need the same topology."""
    batches, current = [], []
    for index, spec in enumerate(specs):
        if current and (len(current) == batch_size or
                        current[0][1]['obstacle_id'] != spec['obstacle_id']):
            batches.append(current)
            current = []
        current.append((index, spec))
    if current:
        batches.append(current)
    return batches


def run_benchmark(specs_path, output, *, batch_size=2, pipeline=False,
                  validation_workers=2, queue_capacity=1, grouped_camera=False,
                  persistent_renderer=False, refresh_render_products=False,
                  root=ROOT, runner=run_owned_command,
                  clock=time.perf_counter, worker_factory=RenderWorker):
    root, output = Path(root).resolve(), Path(output).resolve()
    specs = read_json(specs_path)
    if not isinstance(specs, list) or not specs or any(
            not isinstance(s, dict) or not isinstance(s.get('obstacle_id'), str) for s in specs):
        raise ValueError('episode-specs must be a nonempty JSON list of episode specifications')
    if batch_size < 1 or validation_workers < 1 or queue_capacity < 1:
        raise ValueError('batch-size, validation-workers and queue-capacity must be positive')
    output.relative_to(root)  # Docker exposes the project as /workspace.
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'episode_specs.json', specs)
    batches = partition_specs(specs, batch_size)
    report = {
        'status': 'running', 'pipeline': bool(pipeline), 'batch_size': batch_size,
        'validation_workers': validation_workers, 'requested_episodes': len(specs),
        'queue_capacity': queue_capacity,
        'started_at': datetime.now(timezone.utc).isoformat(),
        'production_quota_eligible': False,
        'experiment_scope': 'Repeated inputs allowed; diagnostic two-stage collection, not production quota.',
        'timing_scope': 'Host start through all owned subprocess exits, including pipeline fill/drain, rendering and full validation.',
        'resource_scope': 'At most one physics Kit and one render Kit; consumer validation is serial within the render role.',
        'role_timing_scope': 'Command-call start/finish relative to benchmark scheduling start; includes launch and process exit, excludes queue wait. Physics timing is once per batch, not once per environment.',
        'configuration': {'physics_device': 'cpu', 'physics_threads': 0,
                          'record_render_state': True, 'physics_record_rgb': False,
                          'defer_clearance_diagnostics': True, 'render_device': 'cpu',
                          'camera_device': 'cuda:0', 'background_rgb': True,
                          'grouped_camera': bool(grouped_camera),
                          'persistent_renderer': bool(persistent_renderer),
                          'refresh_render_products': bool(refresh_render_products),
                          'queue_capacity_episodes': queue_capacity if pipeline else 0},
        'batches': [], 'episodes': [
            {'index': i, 'obstacle_id': s['obstacle_id'], 'status': 'not_started',
             'physics_success': False, 'render_complete': False,
             'validation_passed': False, 'qualified': False} for i, s in enumerate(specs)],
        'host_errors': [],
    }
    lock = threading.RLock()
    stop, producer_done, consumer_done = threading.Event(), threading.Event(), threading.Event()
    jobs = queue.Queue(maxsize=queue_capacity)
    started = clock()
    worker = (worker_factory(cwd=root, directory=output / 'renderer_worker',
              grouped_camera=grouped_camera, refresh_render_products=refresh_render_products)
              if persistent_renderer else None)
    if worker is not None:
        report['role_timing_scope'] += (' Persistent render_request_timing covers one request including lazy '
            'startup if first; actual shared Kit lifecycle/shutdown are recorded separately.')

    def relative(path):
        return str(Path(path).relative_to(root))

    def save():
        with lock:
            write_json(output / 'summary.json', report)

    def fail_host(role, error):
        stop.set()
        with lock:
            report['host_errors'].append({'role': role, 'error': f'{type(error).__name__}: {error}',
                                         'traceback': traceback.format_exc()})
            save()

    def run_role(command, log_path, target, timing_key, operation=None):
        timing = {'start_relative_s': clock() - started, 'finish_relative_s': None,
                  'wall_s': None}
        with lock:
            target[timing_key] = timing
        try:
            return operation() if operation is not None else runner(command, log_path, cwd=root)
        finally:
            finished = clock() - started
            with lock:
                timing.update(finish_relative_s=finished,
                              wall_s=finished - timing['start_relative_s'])

    def close_renderer():
        if worker is None:
            return
        timing = {'start_relative_s': clock() - started}
        try:
            lifecycle = worker.close()
            if lifecycle.get('started') and lifecycle.get('exit_code') != 0:
                raise RuntimeError(f'Shared renderer exited with code {lifecycle.get("exit_code")}')
        except BaseException as error:
            fail_host('renderer_shutdown', error)
        finally:
            timing.update(finish_relative_s=clock() - started)
            timing['wall_s'] = timing['finish_relative_s'] - timing['start_relative_s']
            with lock:
                report['renderer_worker'] = worker.report()
                report['renderer_shutdown_timing'] = timing
                save()

    def collect_batch(batch_index, entries):
        directory = output / f'batch_{batch_index:03d}'
        directory.mkdir()
        spec_path, physics = directory / 'episode_specs.json', directory / 'physics'
        write_json(spec_path, [spec for _, spec in entries])
        command = ['bash', 'scripts/arena_python.sh', 'scripts/collect_kitchen_vectorized.py',
                   '--device', 'cpu', '--num-envs', str(len(entries)),
                   '--episode-specs', relative(spec_path), '--output', relative(physics),
                   '--record-render-state', '--physics-threads', '0',
                   '--defer-clearance-diagnostics', '--validation-workers', str(validation_workers)]
        batch = {'index': batch_index, 'episode_indices': [index for index, _ in entries],
                 'command': command, 'output': relative(physics), 'status': 'running'}
        with lock:
            report['batches'].append(batch)
            for index, _ in entries:
                report['episodes'][index]['status'] = 'physics_running'
            save()
        execution = run_role(command, directory / 'physics.log', batch, 'physics_timing')
        batch_result = read_json(physics / 'batch.json', {})
        ready = []
        with lock:
            batch.update(execution=execution, status='finished',
                         collector_status=batch_result.get('status'),
                         execution_error=batch_result.get('execution_error'))
            for env_index, (index, _) in enumerate(entries):
                source = physics / f'env_{env_index:03d}'
                result = read_json(source / 'result.json', {})
                success = result.get('success') is True
                row = report['episodes'][index]
                row.update(batch_index=batch_index, env_index=env_index,
                           source=relative(source), physics_success=success,
                           steps=result.get('steps'), status='waiting_for_render' if success else 'physics_failed')
                if success:
                    ready.append(index)
                else:
                    row['failure'] = {'kind': 'physics_failed', 'source_failure_kind': result.get('failure_kind'),
                                      'error': result.get('error') or batch_result.get('execution_error') or
                                      ('Missing result.json' if not result else 'Source success is false')}
            save()
        return ready

    def render_episode(index):
        if stop.is_set():
            return
        row = report['episodes'][index]
        directory = output / f'episode_{index:05d}'
        directory.mkdir()
        rendered, validation_path = directory / 'rendered', directory / 'validation.json'
        command = ['bash', 'scripts/arena_python.sh', 'scripts/render_recorded_episode.py',
                   '--device', 'cpu', '--num_envs', '1', '--source-episode', row['source'],
                   '--output', relative(rendered), '--camera-device', 'cuda:0', '--background-rgb']
        if grouped_camera:
            command.append('--grouped-camera')
        if refresh_render_products:
            command.append('--refresh-render-products')
        with lock:
            row.update(status='render_running', render_output=relative(rendered))
            if worker is None:
                row['render_command'] = command
            else:
                row['render_request_input'] = {'source_episode': row['source'], 'output': relative(rendered)}
            save()
        if worker is None:
            execution = run_role(command, directory / 'render.log', row, 'render_timing')
            execution_ok = execution.get('exit_code') == 0
            with lock:
                row['render_execution'] = execution
        else:
            execution = run_role(None, None, row, 'render_request_timing',
                operation=lambda: worker.execute(root / row['source'], rendered))
            execution_ok = execution.get('status') == 'completed'
            with lock:
                row['render_request'] = execution
                report['renderer_worker'] = worker.report()
        rendered_result = read_json(rendered / 'render_report.json', {})
        complete = (execution_ok and rendered_result.get('completed') is True
                    and rendered_result.get('complete_trajectory') is True
                    and rendered_result.get('partial') is False
                    and rendered_result.get('rendered_frames') == row['steps'])
        with lock:
            row.update(render_complete=complete,
                       rendered_frames=rendered_result.get('rendered_frames'))
            if not complete:
                row.update(status='render_failed', failure={'kind': 'render_failed',
                           'error': execution.get('error') or rendered_result.get('error') or 'Renderer did not complete all source frames'})
                save()
                if execution.get('worker_unavailable'):
                    raise RuntimeError('Shared renderer unavailable; remaining requests are not retried')
                return
            row['status'] = 'validation_running'
            save()
        if stop.is_set():
            return
        command = ['bash', 'scripts/arena_python.sh', 'scripts/validate_collection.py', row['source'],
                   '--render-output', relative(rendered), '--require-rgb', '--report', relative(validation_path)]
        execution = run_role(command, directory / 'validation.log', row, 'validation_timing')
        validation = read_json(validation_path, [])
        passed = (execution.get('exit_code') == 0 and isinstance(validation, list)
                  and len(validation) == 1 and validation[0].get('passed') is True)
        with lock:
            row.update(validation_execution=execution, validation_report=relative(validation_path),
                       validation_passed=passed, qualified=bool(row['physics_success'] and complete and passed),
                       status='qualified' if passed else 'validation_failed')
            if not passed:
                row['failure'] = {'kind': 'validation_failed', 'results': validation}
            save()

    def producer():
        try:
            for batch_index, entries in enumerate(batches):
                if stop.is_set():
                    break
                for index in collect_batch(batch_index, entries):
                    if pipeline:
                        while not stop.is_set():
                            try:
                                jobs.put(index, timeout=.1)
                                break
                            except queue.Full:
                                continue
                    elif not stop.is_set():
                        render_episode(index)
        except BaseException as error:
            fail_host('producer' if pipeline else 'serial', error)
        finally:
            try:
                if not pipeline:
                    close_renderer()
            finally:
                producer_done.set()

    def consumer():
        try:
            while not stop.is_set():
                try:
                    index = jobs.get(timeout=.1)
                except queue.Empty:
                    if producer_done.is_set():
                        break
                    continue
                if stop.is_set():
                    break
                render_episode(index)
        except BaseException as error:
            fail_host('consumer', error)
        finally:
            try:
                close_renderer()
            finally:
                consumer_done.set()

    save()
    threads = []
    try:
        if pipeline:
            threads = [threading.Thread(target=producer, name='physics-producer'),
                       threading.Thread(target=consumer, name='rgb-consumer')]
            for thread in threads:
                thread.start()
            # Waiting on completion Events avoids an interrupted Thread.join
            # incorrectly marking a still-running Python thread as stopped.
            while not (producer_done.is_set() and consumer_done.is_set()):
                producer_done.wait(.1) if not producer_done.is_set() else consumer_done.wait(.1)
        else:
            producer()
    except BaseException as error:
        fail_host('main', error)
    finally:
        # Never leave a worker owning an in-flight docker exec after this report.
        # Repeated host interrupts request no new work but cannot skip draining.
        for thread, done in zip(threads, (producer_done, consumer_done)):
            while thread.ident is not None and not done.is_set():
                try:
                    done.wait(.1)
                except BaseException as error:
                    fail_host('main_waiting_for_children', error)
        # Every role must finish its owned command before any join can itself
        # be interrupted. A repeated interrupt must not skip the other role.
        for thread in threads:
            if thread.ident is not None:
                while True:
                    try:
                        thread.join()
                        break
                    except BaseException as error:
                        fail_host('main_joining_finished_roles', error)
        report['total_wall_s'] = clock() - started
        for row in report['episodes']:
            if row['status'] in ('physics_running', 'render_running', 'validation_running', 'waiting_for_render'):
                row['interrupted_stage'] = row['status']
                row['status'] = 'host_interrupted'
        report['counts'] = {key: sum(row.get(key) is True for row in report['episodes'])
                            for key in ('physics_success', 'render_complete', 'validation_passed', 'qualified')}
        report['failure_counts'] = {kind: sum(row['status'] == kind for row in report['episodes'])
                                   for kind in ('physics_failed', 'render_failed', 'validation_failed',
                                                'host_interrupted', 'not_started')}
        qualified = report['counts']['qualified']
        report['status'] = ('host_error' if report['host_errors'] else
                            'completed' if qualified == len(specs) else 'completed_with_failures')
        report['validated_successes_per_hour'] = (qualified * 3600 / report['total_wall_s']
            if report['total_wall_s'] > 0 and not report['host_errors'] else None)
        report['all_owned_subprocesses_finished'] = True
        report['finished_at'] = datetime.now(timezone.utc).isoformat()
        save()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode-specs', type=Path, required=True)
    parser.add_argument('--batch-size', type=int, default=2)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pipeline', action='store_true')
    parser.add_argument('--validation-workers', type=int, default=2)
    parser.add_argument('--queue-capacity', type=int, default=1,
                        help='Positive number of queued episodes, independent of physics batch size; used with --pipeline')
    parser.add_argument('--grouped-camera', action='store_true',
                        help='Opt-in native three-view grouped render product in the standalone RGB renderer')
    parser.add_argument('--persistent-renderer', action='store_true',
                        help='Reuse one renderer Kit and compatible scenes; each episode retains full validation')
    parser.add_argument('--refresh-render-products', action='store_true',
                        help='Refresh original render products before appearance changes in reused scenes')
    args = parser.parse_args()
    report = run_benchmark(args.episode_specs, args.output, batch_size=args.batch_size,
                           pipeline=args.pipeline, validation_workers=args.validation_workers,
                           queue_capacity=args.queue_capacity, grouped_camera=args.grouped_camera,
                           persistent_renderer=args.persistent_renderer,
                           refresh_render_products=args.refresh_render_products)
    print(json.dumps({key: report[key] for key in ('status', 'counts', 'failure_counts', 'total_wall_s',
                                                 'validated_successes_per_hour')}, indent=2))
    return 0 if report['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
