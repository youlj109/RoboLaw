"""Run an explicit list of collectors in one SimulationApp, with fresh stages.

    bash scripts/arena_python.sh scripts/collect_kitchen_sequence.py episodes.json \
        --report reports/sequence.json

JSON is a list of {"id": "optional", "argv": [collector arguments...]} objects
or argument arrays. Every entry needs a distinct explicit --output. Existing
episode_spec.json files are allowed; recorded output files are never replaced.
This runner does not validate datasets or resume the production batch. Each
collector retains its own result.json and independent runpy global namespace.
Python output is copied to sequence_run.log; native Kit logging stays on the
parent process streams. App startup settings must agree across entries.

Use --serve to accept {"id": "1", "argv": [...]} JSONL requests on stdin;
replies start with KITCHEN_COLLECTOR_REPLY. EOF closes the shared app.
"""
import argparse
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timezone
import importlib
import inspect
import json
import os
from pathlib import Path
import runpy
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[1]
COLLECTOR = ROOT / 'scripts/collect_kitchen.py'
APP_OPTIONS = ('headless', 'livestream', 'device', 'experience', 'kit_args', 'distributed')
REPLY_PREFIX = 'KITCHEN_COLLECTOR_REPLY '


def now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    if path is None:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def prepare_entries(data, workspace=ROOT):
    if not isinstance(data, list) or not data:
        raise ValueError('Episode JSON must be a nonempty list')
    entries, destinations = [], set()
    for index, item in enumerate(data):
        argv = item.get('argv') if isinstance(item, dict) else item
        if not isinstance(argv, list) or not argv or not all(isinstance(v, str) for v in argv):
            raise ValueError(f'Episode {index}: argv must be a nonempty list of strings')
        argv = list(argv)
        outputs = []
        for position, token in enumerate(argv):
            if token == '--output':
                if position + 1 == len(argv) or argv[position + 1].startswith('--'):
                    raise ValueError(f'Episode {index}: --output needs a directory')
                outputs.append((position, argv[position + 1], False))
            elif token.startswith('--output='):
                outputs.append((position, token.partition('=')[2], True))
        if len(outputs) != 1 or not outputs[0][1]:
            raise ValueError(f'Episode {index}: exactly one explicit --output is required')
        position, raw, equals = outputs[0]
        output = (Path(workspace) / raw).resolve()
        if output in destinations:
            raise ValueError(f'Episode {index}: duplicate output directory {output}')
        destinations.add(output)
        ensure_unrecorded(output)
        argv[position if equals else position + 1] = f'--output={output}' if equals else str(output)
        entries.append({'id': str(item.get('id', index)) if isinstance(item, dict) else str(index),
                        'argv': argv, 'output': str(output)})
    return entries


def ensure_unrecorded(output):
    for name in ('result.json', 'trajectory.h5', 'images.h5', 'sequence_execution.json'):
        path = Path(output) / name
        if path.exists():
            raise FileExistsError(f'Refusing to overwrite recorded episode artifact: {path}')


class _Tee:
    def __init__(self, stream, logfile):
        self.stream, self.logfile = stream, logfile

    def write(self, data):
        self.logfile.write(data)
        return self.stream.write(data)

    def flush(self):
        self.logfile.flush()
        self.stream.flush()

    def __getattr__(self, name):
        return getattr(self.stream, name)


def _cleanup_episode(state, helpers):
    """Restore collector instrumentation and release its environment references."""
    errors = []
    namespace = state.pop('namespace', None)
    env = namespace.get('env') if namespace else None
    if env is not None:
        try:
            # Collector normally closes this itself; ManagerBasedEnv.close is
            # idempotent. This also handles exceptions before its main finally.
            env.close()
        except Exception:
            errors.append(traceback.format_exc())
    cameras = namespace.get('cameras') if namespace else None
    if cameras is not None:
        try:
            cameras.close()
        except Exception:
            errors.append(traceback.format_exc())
    for target, name, existed, original in reversed(state.pop('wrappers', [])):
        try:
            if existed:
                setattr(target, name, original)
            else:
                delattr(target, name)
        except Exception:
            errors.append(traceback.format_exc())
    if 'physics_profiler' in state:
        cls, existed, original = state.pop('physics_profiler')
        if existed:
            cls._step_wall_profiler = original
        elif '_step_wall_profiler' in vars(cls):
            delattr(cls, '_step_wall_profiler')
    if state['launcher'] is not None:
        try:
            helpers.teardown_simulation_app(suppress_exceptions=False, make_new_stage=True)
        except Exception:
            errors.append(traceback.format_exc())
    # Do not clear the globals dictionary in place: destructors may still use
    # those globals. Drop all references before the caller invokes GC.
    return errors


def run_sequence(entries, report_path, *, simulation_helpers=None, collector_path=COLLECTOR,
                 stop_on_error=False, profiler_class=None, physics_manager_class=None,
                 streamed=False, on_episode=None, on_runner_error=None, kit_log_path=None):
    """Run prepared entries. Injectable helpers support CPU-only lifecycle tests."""
    started = time.perf_counter()
    if simulation_helpers is None:
        simulation_helpers = importlib.import_module('isaaclab_arena.utils.isaaclab_utils.simulation_app')
    if profiler_class is None:
        from kitchen_safety.step_profiling import StepWallProfiler
        profiler_class = StepWallProfiler
    helpers = simulation_helpers
    original_context = helpers.SimulationAppContext
    original_wrap = profiler_class.wrap
    state = {'launcher': None, 'startup_seconds': 0., 'startup_failed': False, 'wrappers': []}
    if kit_log_path is not None:
        kit_log_path = Path(kit_log_path).resolve()
        kit_log_path.parent.mkdir(parents=True, exist_ok=True)
    needs_cameras = (None if streamed else any(any(flag in entry['argv'] for flag in
                     ('--record-rgb', '--video')) for entry in entries))
    report = {'started_at': now(), 'completed_at': None, 'status': 'running',
              'requested_episodes': 0 if streamed else len(entries), 'episodes': [], 'app_startups': 0,
              'app_startup_seconds': 0., 'wall_seconds': 0.,
              'kit_log_path': str(kit_log_path) if kit_log_path is not None else None,
              'semantics': 'Collector success only; dataset validation/audit and augmentation quota are unchanged and external'}
    write_json(report_path, report)

    class BorrowedSimulationAppContext:
        def __init__(self, args):
            self.args = args
            self.app_launcher = None

        def __enter__(self):
            frame = inspect.currentframe()
            try:
                state['namespace'] = frame.f_back.f_globals
            finally:
                del frame
            settings = {key: getattr(self.args, key, None) for key in APP_OPTIONS}
            if streamed:
                settings['enable_cameras'] = bool(getattr(self.args, 'enable_cameras', False))
            if state['launcher'] is None:
                launcher_args = argparse.Namespace(**vars(self.args))
                launcher_args.enable_cameras = settings['enable_cameras'] if streamed else needs_cameras
                if kit_log_path is not None:
                    # Apply only to the launch copy. Keep caller settings
                    # unchanged for subsequent episode consistency checks.
                    kit_args = getattr(launcher_args, 'kit_args', '') or ''
                    launcher_args.kit_args = f'{kit_args} --/log/file={kit_log_path}'.strip()
                tick = time.perf_counter()
                try:
                    state['launcher'] = helpers.get_app_launcher(launcher_args)
                except BaseException:
                    state['startup_failed'] = True
                    raise
                finally:
                    state['startup_seconds'] = time.perf_counter() - tick
                state['app_settings'] = settings
                report['app_startups'] = 1
                report['app_settings'] = settings
            elif settings != state['app_settings']:
                raise ValueError('App startup arguments differ from the first episode; use a separate sequence')
            self.app_launcher = state['launcher']
            cls = physics_manager_class
            if cls is None:
                from isaaclab_physx.physics.physx_manager import PhysxManager
                cls = PhysxManager
            state['physics_profiler'] = (cls, '_step_wall_profiler' in vars(cls),
                                        vars(cls).get('_step_wall_profiler'))
            return self

        def __exit__(self, exc_type, exc_value, exc_tb):
            # Never call the original context's os._exit/app.close path here.
            return False

        def is_running(self):
            return self.app_launcher.app.is_running()

        def is_exiting(self):
            return self.app_launcher.app.is_exiting()

    def tracked_wrap(profiler, target, name, category, hook=None):
        dictionary = vars(target)
        state.setdefault('wrappers', []).append((target, name, name in dictionary, dictionary.get(name)))
        return original_wrap(profiler, target, name, category, hook)

    helpers.SimulationAppContext = BorrowedSimulationAppContext
    profiler_class.wrap = tracked_wrap
    argv_before = sys.argv
    interrupted = None
    exhausted = False
    try:
        for index, entry in enumerate(entries):
            if streamed:
                report['requested_episodes'] += 1
            output = Path(entry['output'])
            ensure_unrecorded(output)
            output.mkdir(parents=True, exist_ok=True)
            episode_started = time.perf_counter()
            record = dict(entry, index=index, started_at=now(), exit_code=0, error=None)
            print(f'SEQUENCE_START {index} {entry["id"]} {output}', flush=True)
            with (output / 'sequence_run.log').open('w') as logfile:
                with redirect_stdout(_Tee(sys.stdout, logfile)), redirect_stderr(_Tee(sys.stderr, logfile)):
                    sys.argv = [str(collector_path), *entry['argv']]
                    try:
                        runpy.run_path(str(collector_path), run_name='__main__')
                    except SystemExit as error:
                        record['exit_code'] = error.code if isinstance(error.code, int) else (0 if error.code is None else 1)
                        if record['exit_code']:
                            record['error'] = f'Collector exited with {error.code!r}'
                    except KeyboardInterrupt as error:
                        interrupted = error
                        record.update(exit_code=130, error='KeyboardInterrupt')
                    except Exception:
                        record.update(exit_code=1, error=traceback.format_exc())
                        print(record['error'], file=sys.stderr, flush=True)
                    finally:
                        sys.argv = argv_before
                        record['collector_wall_seconds'] = time.perf_counter() - episode_started
                        tick = time.perf_counter()
                        record['cleanup_errors'] = _cleanup_episode(state, helpers)
                        # _cleanup_episode has now dropped its globals and env
                        # references, including class-level profiler closures.
                        try:
                            if state['launcher'] is not None:
                                helpers.collect_garbage_and_clear_cuda_cache()
                        except Exception:
                            record['cleanup_errors'].append(traceback.format_exc())
                        record['cleanup_seconds'] = time.perf_counter() - tick
            result_path = output / 'result.json'
            result = None
            if result_path.exists():
                try:
                    result = json.loads(result_path.read_text())
                except (OSError, ValueError):
                    record['result_read_error'] = traceback.format_exc()
            record['result'] = ({key: result.get(key) for key in (
                'success', 'steps', 'failure_kind', 'failure_phase', 'error',
                'augmentation_eligible', 'timings_seconds')} if isinstance(result, dict) else None)
            record['success'] = bool(record['exit_code'] == 0 and result and result.get('success')
                                     and not record['cleanup_errors'])
            record['completed_at'] = now()
            record['elapsed_seconds'] = time.perf_counter() - episode_started
            record['worker_stopping'] = bool(interrupted or state['startup_failed'] or record['cleanup_errors']
                                              or (stop_on_error and not record['success']))
            write_json(output / 'sequence_execution.json', record)
            report['episodes'].append(record)
            report['app_startup_seconds'] = state['startup_seconds']
            report['wall_seconds'] = time.perf_counter() - started
            write_json(report_path, report)
            print(f'SEQUENCE_FINISH {index} success={record["success"]} seconds={record["elapsed_seconds"]:.3f}', flush=True)
            if on_episode is not None:
                on_episode(record)
            if record['worker_stopping']:
                break
        else:
            exhausted = True
    except Exception:
        report['runner_error'] = traceback.format_exc()
        if on_runner_error is not None:
            on_runner_error(entry if 'entry' in locals() else None, report['runner_error'])
    finally:
        sys.argv = argv_before
        helpers.SimulationAppContext = original_context
        profiler_class.wrap = original_wrap
        report['completed_at'] = now()
        report['status'] = ('completed' if (exhausted if streamed else
                            len(report['episodes']) == len(entries)) else 'stopped')
        report['successful_collectors'] = sum(item['success'] for item in report['episodes'])
        report['passed'] = (report['status'] == 'completed' and all(item['success'] for item in report['episodes'])
                            and not report.get('runner_error'))
        report['app_startup_seconds'] = state['startup_seconds']
        report['wall_seconds'] = time.perf_counter() - started
        # Persist before app.close(), which some Kit builds can terminate inside.
        write_json(report_path, report)
        if state['launcher'] is not None:
            tick = time.perf_counter()
            try:
                state['launcher'].app.close()
            except Exception:
                report['shutdown_error'] = traceback.format_exc()
                report['passed'] = False
            report['app_shutdown_seconds'] = time.perf_counter() - tick
            report['wall_seconds'] = time.perf_counter() - started
            write_json(report_path, report)
    return report


def serve(input_stream=None, output_stream=None, report_path=None, workspace=ROOT, **run_kwargs):
    """Consume {id, argv} JSONL; emit one prefixed reply after each cleanup.

The first request determines app device and camera enablement. Subsequent
requests must use the same startup settings. EOF closes the shared app.
    """
    input_stream = input_stream or sys.stdin
    output_stream = output_stream or sys.stdout

    def reply(value):
        # Begin with a newline even if a native Kit log omitted its terminator.
        output_stream.write('\n' + REPLY_PREFIX + json.dumps(value) + '\n')
        output_stream.flush()

    def requests():
        for line in input_stream:
            if not line.strip():
                continue
            request = None
            begin = time.perf_counter()
            try:
                request = json.loads(line)
                if not isinstance(request, dict) or 'id' not in request:
                    raise ValueError('Worker request must be an object containing id and argv')
                yield prepare_entries([request], workspace)[0]
            except (OSError, ValueError, TypeError):
                reply({'id': request.get('id') if isinstance(request, dict) else None,
                       'exit_code': None, 'error': traceback.format_exc(), 'result_path': None,
                       'elapsed_seconds': time.perf_counter() - begin, 'worker_pid': os.getpid()})

    def completed(record):
        errors = record.get('cleanup_errors') or []
        reply({'id': record['id'], 'exit_code': record['exit_code'] if not errors else 1,
               'result_path': str(Path(record['output']) / 'result.json'),
               'elapsed_seconds': record['elapsed_seconds'], 'worker_pid': os.getpid(),
               'success': record['success'], 'error': '\n'.join(errors) if errors else record.get('error'),
               'cleanup_seconds': record['cleanup_seconds'], 'worker_stopping': record['worker_stopping']})

    def failed(entry, error):
        # Reply before potentially slow Kit shutdown, including disk errors
        # while preserving result/execution reports after a collector finishes.
        reply({'id': entry['id'] if entry else None, 'exit_code': None, 'error': error,
               'result_path': str(Path(entry['output']) / 'result.json') if entry else None,
               'worker_pid': os.getpid(), 'worker_stopping': True})

    return run_sequence(requests(), report_path, streamed=True, on_episode=completed,
                        on_runner_error=failed, **run_kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('episodes_json', type=Path, nargs='?')
    parser.add_argument('--serve', action='store_true', help='Accept collector argv requests as JSONL on stdin until EOF')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--kit-log', type=Path, help='Native Kit log file for this shared app')
    parser.add_argument('--stop-on-error', action='store_true', help='Stop after an unsuccessful collector; default continues')
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT))
    if args.serve:
        if args.episodes_json is not None:
            parser.error('--serve cannot be combined with an episode list')
        report = serve(report_path=args.report, stop_on_error=args.stop_on_error, kit_log_path=args.kit_log)
        return 0 if report['passed'] else 1
    if args.episodes_json is None or args.report is None:
        parser.error('List mode requires episodes_json and --report')
    try:
        entries = prepare_entries(json.loads(args.episodes_json.read_text()))
    except (OSError, ValueError, FileExistsError) as error:
        parser.error(str(error))
    report = run_sequence(entries, args.report, stop_on_error=args.stop_on_error, kit_log_path=args.kit_log)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
