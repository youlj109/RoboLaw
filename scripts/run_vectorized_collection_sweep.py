"""Host-side sequential full-RGB collection sweep, followed by CPU validation.

Each N repeats one episode specification as a performance experiment. This is
independent planning and physical collection, but repeated layouts never add
production quota. No Kit process overlaps another candidate or its RGB decode.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
import json
import multiprocessing
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def _read(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError) as error:
        return {'read_error': f'{type(error).__name__}: {error}'}


def _now():
    return datetime.now(timezone.utc).isoformat()


def collector_command(specs, output, count, warmup_mode, validation_workers, *,
                      warmup_window_frames=16, warmup_max_frames=1024,
                      fixed_appearance_benchmark=False, device='cuda:0', camera_device=None,
                      physics_threads=None, defer_clearance_diagnostics=False,
                      background_rgb=False, reuse_phase_planner=False, curobo_cuda_graph=False,
                      render_aa_op=None,
                      root=ROOT):
    command = ['bash', 'scripts/arena_python.sh', 'scripts/collect_kitchen_vectorized.py',
            '--device', device, '--num-envs', str(count),
            '--episode-specs', str(Path(specs).relative_to(root)),
            '--output', str(Path(output).relative_to(root)), '--record-rgb',
            '--lighting', 'shared', '--warmup-mode', warmup_mode,
            '--warmup-window-frames', str(warmup_window_frames),
            '--warmup-max-frames', str(warmup_max_frames),
            '--validation-workers', str(validation_workers)]
    if fixed_appearance_benchmark:
        command.append('--fixed-appearance-benchmark')
    if camera_device is not None:
        command.extend(['--camera-device', camera_device])
    if physics_threads is not None:
        command.extend(['--physics-threads', str(physics_threads)])
    if render_aa_op is not None:
        command.extend(['--render-aa-op', str(render_aa_op)])
    if defer_clearance_diagnostics:
        command.append('--defer-clearance-diagnostics')
    for enabled, flag in ((background_rgb, '--background-rgb'),
                          (reuse_phase_planner, '--reuse-phase-planner'),
                          (curobo_cuda_graph, '--curobo-cuda-graph')):
        if enabled:
            command.append(flag)
    return command


def _gpu_sample():
    result = subprocess.run(['nvidia-smi', '--id=0',
        '--query-gpu=memory.used,memory.total,utilization.gpu,utilization.memory',
        '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or f'nvidia-smi exited {result.returncode}')
    values = [int(field.strip()) for field in result.stdout.strip().splitlines()[0].split(',')]
    return dict(zip(('memory_used_mib', 'memory_total_mib', 'gpu_utilization_percent',
                     'memory_utilization_percent'), values))


def run_collector(command, log_path, gpu_path, *, cwd=ROOT):
    """Wait for this owned launcher to close, sampling whole device 0 every 2 s."""
    started = time.perf_counter()
    samples, errors = [], []
    result = {'exit_code': None}
    try:
        with Path(log_path).open('w') as log:
            process = subprocess.Popen(command, cwd=str(cwd), stdout=log, stderr=subprocess.STDOUT)
            try:
                next_sample = started
                while process.poll() is None:
                    now = time.perf_counter()
                    if now >= next_sample:
                        try:
                            samples.append({'elapsed_s': now-started, **_gpu_sample()})
                        except (OSError, ValueError, RuntimeError, IndexError, subprocess.TimeoutExpired) as error:
                            errors.append({'elapsed_s': now-started, 'error': repr(error)})
                        next_sample = now + 2.
                    try:
                        process.wait(timeout=max(.001, next_sample-time.perf_counter()))
                    except subprocess.TimeoutExpired:
                        pass
                result['exit_code'] = process.returncode
            except BaseException:
                # Never continue to another candidate after interruption.
                # Terminate only our host docker-exec launch handle.
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                raise
    except OSError as error:
        result['error'] = repr(error)
    finally:
        result['elapsed_seconds'] = time.perf_counter() - started
        telemetry = {'sample_interval_s': 2., 'physical_gpu_index': 0,
            'scope': 'Whole GPU, including desktop/other processes; sampled peak, not continuous peak or PyTorch-only memory.',
            'samples': samples, 'errors': errors}
        write_json(gpu_path, telemetry)
        result['whole_gpu_peak_sampled_memory_mib'] = max(
            (sample['memory_used_mib'] for sample in samples), default=None)
        result['gpu_telemetry'] = str(gpu_path)
    return result


def _validate_one(path):
    # This import is CPU-only; do not import the collector or SimulationApp.
    from scripts.validate_collection import validate_episode
    started = time.perf_counter()
    try:
        return validate_episode(path, require_rgb=True)
    except Exception as error:
        return {'path': str(path), 'passed': False,
                'error': f'{type(error).__name__}: {error}',
                'elapsed_seconds': time.perf_counter()-started}


def validate_batch(directory, output, *, workers=4):
    started = time.perf_counter()
    directory, output = Path(directory), Path(output)
    batch = _read(directory / 'batch.json')
    count = int(batch['num_envs'])
    paths = [str(directory / f'env_{i:03d}') for i in range(count)]
    with ProcessPoolExecutor(max_workers=min(workers, count),
                             mp_context=multiprocessing.get_context('spawn')) as pool:
        results = list(pool.map(_validate_one, paths))
    report = {'status': 'completed', 'workers_requested': workers,
              'elapsed_seconds': time.perf_counter()-started, 'results': results,
              'scope': 'CPU full JPEG decode, action/state continuity and camera metadata/extrinsics. Physical success remains result.success, checked separately.'}
    write_json(output, report)
    print('CPU_RGB_VALIDATION', sum(row['passed'] for row in results), '/', count, flush=True)
    return 0 if all(row['passed'] for row in results) else 1


def run_validation(directory, report_path, log_path, *, workers=4, cwd=ROOT):
    command = ['bash', 'scripts/arena_python.sh', 'scripts/run_vectorized_collection_sweep.py',
               '--cpu-validate', str(Path(directory).relative_to(cwd)),
               '--validation-report', str(Path(report_path).relative_to(cwd)),
               '--validation-workers', str(workers)]
    started = time.perf_counter()
    result = {'command': command, 'exit_code': None}
    try:
        with Path(log_path).open('w') as log:
            process = subprocess.run(command, cwd=str(cwd), stdout=log, stderr=subprocess.STDOUT)
            result['exit_code'] = process.returncode
    except OSError as error:
        result['error'] = repr(error)
    result['elapsed_seconds'] = time.perf_counter()-started
    return result


def classify_run_failure(batch, execution, log_path, candidate_errors=()):
    """Use actual report/log evidence; never infer OOM from a capacity estimate."""
    oom = ('out of memory', 'cuda_error_out_of_memory', 'vk_error_out_of_device_memory',
           'vk_error_out_of_host_memory')
    evidence = None
    for text in (str(execution.get('error', '')), str(batch.get('error', '')),
                 str(batch.get('execution_error', '')),
                 str(batch.get('appearance_warmup', {}).get('error', '')),
                 *(str(error) for error in candidate_errors)):
        if any(token in text.lower() for token in oom):
            evidence = text[-3000:]
            break
    if evidence is None and Path(log_path).exists():
        with Path(log_path).open(errors='replace') as stream:
            for line in stream:
                if any(token in line.lower() for token in oom):
                    evidence = line.strip()[-3000:]
    if evidence is not None:
        return {'kind': 'out_of_memory', 'evidence': evidence}
    warmup = batch.get('appearance_warmup', {})
    if warmup.get('mode') == 'fixed_preparation':
        if warmup.get('preparation_passed') is not True:
            return {'kind': 'appearance_preparation_failed',
                    'evidence': warmup.get('preparation_error') or str(warmup)}
    elif warmup.get('passed') is False:
        return {'kind': 'appearance_warmup_failed', 'evidence': warmup.get('error') or str(warmup)}
    return {'kind': 'collector_failed', 'evidence': batch.get('execution_error') or batch.get('error') or batch.get('read_error')
            or execution.get('error') or f"Collector exit code {execution.get('exit_code')}"}


def run_sweep(episode_spec, output, counts, *, warmup_mode='single_frame', validation_workers=4,
              warmup_window_frames=16, warmup_max_frames=1024,
              fixed_appearance_benchmark=False, device='cuda:0', camera_device=None,
              physics_threads=None, defer_clearance_diagnostics=False,
              background_rgb=False, reuse_phase_planner=False, curobo_cuda_graph=False,
              render_aa_op=None,
              root=ROOT, collector_runner=run_collector, validation_runner=run_validation,
              clock=time.perf_counter):
    root, output, episode_spec = Path(root).resolve(), Path(output).resolve(), Path(episode_spec).resolve()
    output.relative_to(root)  # The existing container mounts this project at /workspace.
    if not counts or any(count < 1 for count in counts) or validation_workers < 1:
        raise ValueError('At least one positive count and a positive validation worker count are required')
    if warmup_mode not in ('single_frame', 'temporal_mean', 'fixed_preparation'):
        raise ValueError('Unknown existing collector warm-up mode')
    if warmup_window_frames < 1 or warmup_max_frames < 1:
        raise ValueError('Warm-up frame counts must be positive')
    spec = json.loads(episode_spec.read_text())
    if not isinstance(spec, dict) or 'layout' not in spec or 'obstacle_id' not in spec:
        raise ValueError('episode-spec must be a single existing episode specification object')
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'started_at': _now(), 'counts': list(counts), 'runs': [],
        'source_episode_spec': str(episode_spec), 'warmup_mode': warmup_mode,
        'warmup_window_frames': warmup_window_frames, 'warmup_max_frames': warmup_max_frames,
        'fixed_appearance_benchmark': bool(fixed_appearance_benchmark),
        'appearance_mode': 'authored_fixed_benchmark' if fixed_appearance_benchmark else 'randomized',
        'validation_workers': validation_workers, 'record_rgb': True,
        'device': device, 'camera_device': camera_device, 'physics_threads': physics_threads,
        'defer_clearance_diagnostics': bool(defer_clearance_diagnostics),
        'background_rgb': bool(background_rgb), 'reuse_phase_planner': bool(reuse_phase_planner),
        'curobo_cuda_graph': bool(curobo_cuda_graph),
        'render_aa_op': render_aa_op,
        'lighting': 'shared', 'production_quota_eligible': False,
        'semantics': 'Each candidate independently plans and physically executes N copies of one supplied layout. Repeats are performance experiments, not new augmentation quota or a policy success-rate estimate.',
        'timing_scope': 'launch_to_shutdown_s includes Kit startup, collection and app/launcher shutdown. validation_wall_s measures the subsequent Docker CPU validation process including worker startup. total_wall_s measures both stages plus report inspection. Success/hour uses total_wall_s.'}
    if fixed_appearance_benchmark:
        report['appearance_scope'] = (
            'Authored USD materials and lights stay fixed. Requested appearance randomization is not applied; '
            '128 pure-render warm-up frames do not certify appearance convergence. Rates compare full physical '
            'collection plus three-camera RGB under fixed appearance only, and cannot fill randomized production quota. '
            'All existing physical, safety, RGB decode, alignment and camera-pose validation remain required.')
    write_json(output / 'summary.json', report)
    duplicates = {}
    overall_started = clock()
    try:
        for count in counts:
            duplicates[count] = duplicates.get(count, 0) + 1
            suffix = '' if duplicates[count] == 1 else f'_repeat{duplicates[count]:02d}'
            directory = output / f'envs_{count:02d}{suffix}'
            directory.mkdir()
            write_json(directory / 'specs.json', [spec for _ in range(count)])
            target = directory / 'collection'
            command = collector_command(directory / 'specs.json', target, count, warmup_mode,
                                         validation_workers, root=root,
                                         warmup_window_frames=warmup_window_frames,
                                         warmup_max_frames=warmup_max_frames,
                                         fixed_appearance_benchmark=fixed_appearance_benchmark,
                                         device=device, camera_device=camera_device,
                                         physics_threads=physics_threads,
                                         defer_clearance_diagnostics=defer_clearance_diagnostics,
                                         background_rgb=background_rgb, reuse_phase_planner=reuse_phase_planner,
                                         curobo_cuda_graph=curobo_cuda_graph, render_aa_op=render_aa_op)
            row = {'num_envs': count, 'output': str(target.relative_to(root)), 'command': command,
                   'status': 'running', 'started_at': _now(), 'production_quota_eligible': False,
                   'appearance_mode': report['appearance_mode'],
                   'validated_successes': 0, 'validated_successes_per_hour': None}
            report['runs'].append(row)
            write_json(output / 'summary.json', report)
            print('COLLECTION_SWEEP_START', count, flush=True)
            started = clock()
            execution = collector_runner(command, directory / 'collector.log', directory / 'gpu_samples.json', cwd=root)
            row['execution'] = execution
            row['launch_to_shutdown_s'] = execution['elapsed_seconds']
            row['whole_gpu_peak_sampled_memory_mib'] = execution.get('whole_gpu_peak_sampled_memory_mib')
            batch = _read(target / 'batch.json')
            row['collector_status'] = batch.get('status')
            row['collector_timings_seconds'] = batch.get('timings_seconds')
            row['collector_total_before_close_s'] = batch.get('timings_seconds', {}).get('total_before_close_s')
            row['validation_wall_s'] = 0.
            raw_results = [_read(target / f'env_{index:03d}' / 'result.json') for index in range(count)]
            candidate_errors = [result.get('error') or result.get('read_error') for result in raw_results
                                if result.get('success') is not True]
            row['collector_successes'] = sum(result.get('success') is True for result in raw_results)
            row['episodes'] = [{'env_id': index, 'result_success': result.get('success') is True,
                'validation_passed': False, 'validated_success': False, 'steps': result.get('steps'),
                'failure_kind': result.get('failure_kind'),
                'error': result.get('error') or result.get('read_error')}
                for index, result in enumerate(raw_results)]
            if execution.get('exit_code') != 0 or batch.get('status') != 'completed':
                row.update(status='failed', failure=classify_run_failure(
                    batch, execution, directory / 'collector.log', candidate_errors))
            else:
                validation = validation_runner(target, directory / 'data_validation.json',
                                               directory / 'validation.log', workers=validation_workers, cwd=root)
                row['validation_execution'] = validation
                row['validation_wall_s'] = validation['elapsed_seconds']
                checks = _read(directory / 'data_validation.json')
                checked = {Path(item.get('path', '')).name: item for item in checks.get('results', [])}
                episodes = []
                for index in range(count):
                    env_name = f'env_{index:03d}'
                    result = raw_results[index]
                    audit = checked.get(env_name, {})
                    successful = result.get('success') is True and audit.get('passed') is True
                    episodes.append({'env_id': index, 'result_success': result.get('success') is True,
                        'validation_passed': audit.get('passed') is True, 'validated_success': successful,
                        'steps': result.get('steps'), 'failure_kind': result.get('failure_kind'),
                        'error': result.get('error') or result.get('read_error') or audit.get('error')})
                row.update(episodes=episodes, collector_successes=sum(e['result_success'] for e in episodes),
                           validated_successes=sum(e['validated_success'] for e in episodes))
                # Exit 1 is the normal result for individual rejected datasets;
                # a complete validator report still supports the other episodes.
                if checks.get('status') == 'completed' and len(checked) == count and validation.get('exit_code') in (0, 1):
                    row['status'] = 'completed'
                else:
                    row.update(status='failed', failure={'kind': 'cpu_validation_failed',
                        'evidence': checks.get('read_error') or validation.get('error')
                        or f"CPU validation exited {validation.get('exit_code')} without a complete report"})
            # The collector can catch an execution exception, flush partial
            # records and mark the report lifecycle completed. That does not
            # turn OOM or another backend exception into a valid throughput run.
            # Keep every observed physical/data success count even on failure.
            if batch.get('execution_error') or any(candidate_errors) or row['status'] == 'failed':
                failure = classify_run_failure(batch, execution, directory / 'collector.log', candidate_errors)
                if batch.get('execution_error') or failure['kind'] == 'out_of_memory':
                    row.update(status='failed', failure=failure, validated_successes_per_hour=None)
            row.update(total_wall_s=clock()-started, completed_at=_now())
            if row['status'] == 'completed' and row['total_wall_s'] > 0:
                row['validated_successes_per_hour'] = row['validated_successes']*3600/row['total_wall_s']
                row['validated_success_fraction_of_attempts'] = row['validated_successes']/count
            write_json(directory / 'run.json', row)
            write_json(output / 'summary.json', report)
            print('COLLECTION_SWEEP_END', count, row['status'], row['validated_successes'], flush=True)
        report['status'] = ('completed' if all(r['status'] == 'completed' for r in report['runs'])
                            else 'completed_with_failures')
    except BaseException as error:
        report.update(status='interrupted' if isinstance(error, KeyboardInterrupt) else 'error', error=repr(error))
        raise
    finally:
        report.update(total_sweep_wall_s=clock()-overall_started, completed_at=_now())
        measured = [r for r in report['runs'] if r.get('validated_successes_per_hour') is not None]
        best = max(measured, key=lambda r: r['validated_successes_per_hour']) if measured else None
        report['best_measured_num_envs'] = best['num_envs'] if best else None
        report['best_measured_validated_successes_per_hour'] = best['validated_successes_per_hour'] if best else None
        write_json(output / 'summary.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--counts', type=int, nargs='+')
    parser.add_argument('--episode-spec', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--warmup-mode', choices=['single_frame', 'temporal_mean', 'fixed_preparation'], default='single_frame',
                        help='fixed_preparation keeps randomized appearance and records diagnostics after exactly 128 pure renders')
    parser.add_argument('--warmup-window-frames', type=int, default=16)
    parser.add_argument('--warmup-max-frames', type=int, default=1024)
    parser.add_argument('--fixed-appearance-benchmark', action='store_true',
                        help='Use authored USD appearance for a performance experiment; never randomized production quota')
    parser.add_argument('--validation-workers', type=int, default=4)
    parser.add_argument('--device', default='cuda:0', help='Physics/state device passed to the collector')
    parser.add_argument('--camera-device', help='Optional camera tensor device override, e.g. cuda:0')
    parser.add_argument('--physics-threads', type=int, help='Optional PhysX thread count; 0 is supported')
    parser.add_argument('--defer-clearance-diagnostics', action='store_true',
                        help='Pass the existing deferred clearance-diagnostic mode to the collector')
    parser.add_argument('--cpu-validate', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--background-rgb', action='store_true')
    parser.add_argument('--reuse-phase-planner', action='store_true')
    parser.add_argument('--curobo-cuda-graph', action='store_true')
    parser.add_argument('--render-aa-op', type=int, choices=range(5), default=None)
    parser.add_argument('--validation-report', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.validation_workers < 1:
        parser.error('validation-workers must be positive')
    if args.cpu_validate:
        if not args.validation_report:
            parser.error('CPU validation requires an output report')
        return validate_batch(args.cpu_validate, args.validation_report, workers=args.validation_workers)
    if args.counts is None or args.episode_spec is None or args.output is None:
        parser.error('--counts, --episode-spec and --output are required')
    result = run_sweep(args.episode_spec, args.output, args.counts,
                       warmup_mode=args.warmup_mode, validation_workers=args.validation_workers,
                       warmup_window_frames=args.warmup_window_frames,
                       warmup_max_frames=args.warmup_max_frames,
                       fixed_appearance_benchmark=args.fixed_appearance_benchmark,
                       device=args.device, camera_device=args.camera_device,
                       physics_threads=args.physics_threads,
                       defer_clearance_diagnostics=args.defer_clearance_diagnostics,
                       background_rgb=args.background_rgb, reuse_phase_planner=args.reuse_phase_planner,
                       curobo_cuda_graph=args.curobo_cuda_graph, render_aa_op=args.render_aa_op)
    return 0 if result['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
