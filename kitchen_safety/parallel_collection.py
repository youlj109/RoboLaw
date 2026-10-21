"""Resumable production scheduling of native physics and recorded-state RGB.

No simulator imports: the batch CLI supplies ordinary subprocess/JSON helpers.
Physics runs in homogeneous batches; one renderer consumes successful episodes
serially. Final acceptance remains the existing independent batch verification.
"""
import os
from pathlib import Path
import sys
import threading
import time


class VectorizedBackend:
    def __init__(self, output, args, overrides, api, *, worker_factory=None):
        self.output, self.args, self.overrides, self.api = Path(output), args, overrides, api
        self.root = api.ROOT
        self.runtime = api.runtime_configuration(args)
        self.work = self.output / '_vectorized'
        self.work.mkdir(exist_ok=True)
        self.worker_factory = worker_factory
        self.worker = None
        self.worker_dir = None
        self.closed = False
        self.shared_batch_error = None

    def _new_directory(self, prefix):
        index = 0
        while (self.work / f'{prefix}_{index:04d}').exists():
            index += 1
        return self.work / f'{prefix}_{index:04d}'

    def collect(self, jobs):
        """Checkpoint assignment before launch; retain shared artifacts on failure."""
        directory = self._new_directory('batch')
        directory.mkdir()
        specs = [self.api.read_json(job['destination'] / 'episode_spec.json') for job in jobs]
        if len({spec['obstacle_id'] for spec in specs}) != 1:
            raise ValueError('A native physics batch must contain exactly one obstacle ID')
        mapping = [dict(env_index=index, attempt=job['attempt'], obstacle_id=job['id'],
                        attempt_path=str(job['destination'].relative_to(self.output)))
                   for index, job in enumerate(jobs)]
        for name, value in [('episode_specs', specs), ('job_mapping', mapping),
                            ('collector_overrides', self.overrides)]:
            self.api.write_json(directory / (name + '.json'), value)
        command = [sys.executable, str(self.root / 'scripts/collect_kitchen_vectorized.py'),
                   '--episode-specs', str(directory / 'episode_specs.json'),
                   '--output', str(directory / 'physics'), '--num-envs', str(len(jobs)),
                   '--device', self.args.device, '--record-render-state',
                   '--defer-clearance-diagnostics', '--validation-workers', str(self.args.validation_workers),
                   '--collector-overrides', str(directory / 'collector_overrides.json'),
                   '--job-mapping', str(directory / 'job_mapping.json'),
                   '--collection-purpose', 'production', '--collection-source', str(self.output)]
        if getattr(self.args, 'observer_config', None):
            command.extend(['--observer-config', str(self.args.observer_config),
                            '--run-id', self.args.run_id or str(self.output)])
        for option in ('physics_threads', 'torch_threads'):
            value = getattr(self.args, option)
            if value is not None:
                command.extend(['--' + option.replace('_', '-'), str(value)])
        if 'episode_length_s' in self.runtime:
            command.extend(['--episode-length-s', str(self.runtime['episode_length_s'])])
        for index, job in enumerate(jobs):
            assignment = dict(batch_directory=str(directory), env_index=index,
                physics_source=str(directory / 'physics' / f'env_{index:03d}'),
                collection_purpose='production', collection_source=str(self.output))
            self.api.write_json(job['destination'] / 'vectorized_assignment.json', assignment)
        execution = self.api.run_command(command, directory / 'run.log')
        report_path = directory / 'physics' / 'batch.json'
        report = self.api.read_json(report_path, {})
        # Individual controller failures are expected sampling outcomes. These
        # explicit shared failures instead mean the process/backend is unusable;
        # retrying every remaining sample would only consume the attempt budget.
        shared_error = (report.get('execution_error') or report.get('camera_close_error')
                        or (report.get('error') if report.get('status') == 'error' else None))
        if execution.get('exit_code') != 0:
            shared_error = shared_error or execution.get('error') or (
                f'Native physics process did not exit successfully: {execution.get("exit_code")}')
        if shared_error:
            self.shared_batch_error = str(shared_error)
            execution.update(shared_batch_error=self.shared_batch_error,
                worker_unavailable=True, error=execution.get('error') or self.shared_batch_error)
            storage = self.api.storage_error_message(self.shared_batch_error)
            if storage:
                execution['storage_error'] = storage
        execution.update(command=command, runtime=self.runtime, collector_overrides=self.overrides,
                         execution_scope='shared_native_physics_batch', environment_count=len(jobs),
                         shared_batch_report=str(report_path))
        self.api.write_json(directory / 'execution.json', execution)
        return [self.recover(job['destination']) for job in jobs]

    def recover(self, destination):
        """Materialize completed physics without rerunning an interrupted attempt."""
        destination = Path(destination)
        saved = self.api.read_json(destination / 'execution.json')
        assignment = self.api.read_json(destination / 'vectorized_assignment.json', {})
        if saved is not None:
            return saved
        shared = self.api.read_json(Path(assignment.get('batch_directory', destination)) / 'execution.json')
        # No exit checkpoint means the original process outcome is unknown. A
        # later new attempt gets a new index; partial data is never overwritten.
        if shared is None:
            execution = dict(exit_code=None, elapsed_seconds=0,
                error='Interrupted native batch: original collector exit status is unavailable')
        else:
            source = Path(assignment['physics_source'])
            if source.is_dir():
                for path in source.iterdir():
                    if not path.is_file():
                        continue
                    target = destination / path.name
                    if path.name == 'episode_spec.json':
                        if self.api.read_json(path) != self.api.read_json(target):
                            raise ValueError(f'Native source specification differs from attempt: {destination}')
                    elif target.exists():
                        if not os.path.samefile(path, target):
                            raise ValueError(f'Cannot overwrite existing attempt artifact: {target}')
                    else:
                        os.link(path, target)
            execution = dict(shared)
            execution.update(elapsed_seconds=0,
                shared_batch_wall_seconds=shared.get('elapsed_seconds', 0),
                execution_scope='environment_in_shared_native_physics_batch')
        execution.update(execution_backend='vectorized', **assignment)
        self.api.write_json(destination / 'execution.json', execution)
        return execution

    def _renderer(self):
        if self.worker is None:
            if self.worker_factory is None:
                from kitchen_safety.render_worker import RenderWorker
                self.worker_factory = RenderWorker
            self.worker_dir = self._new_directory('renderer')
            command = [sys.executable, str(self.root / 'scripts/render_recorded_batch.py'),
                '--serve', '--request-dir', str(self.worker_dir),
                '--report', str(self.worker_dir / 'report.json'),
                '--device', 'cpu', '--num_envs', '1',
                '--camera-device', self.args.camera_device or 'cuda:0',
                '--refresh-render-products', '--collection-purpose', 'production',
                '--collection-source', str(self.output)]
            if self.args.background_rgb:
                command.append('--background-rgb')
            self.worker = self.worker_factory(cwd=self.root, directory=self.worker_dir,
                command=command, refresh_render_products=True,
                collection_purpose='production', collection_source=str(self.output))
        return self.worker

    def render(self, destination, execution):
        destination = Path(destination)
        checkpoint = destination / 'render_execution.json'
        previous = self.api.read_json(checkpoint)
        if previous is not None and not (previous.get('worker_unavailable') or previous.get('storage_error')):
            return previous
        result = self.api.read_json(destination / 'result.json', {})
        observer_partial = ((destination / 'safety_observer_binding.json').exists()
                            and result.get('steps', 0) > 0 and (destination / 'render_state_metadata.json').exists())
        if (execution.get('exit_code') != 0 or execution.get('shared_batch_error')
                or (result.get('success') is not True and not observer_partial)):
            return dict(status='skipped', skipped_reason='Physical execution did not pass', request_wall_s=0)
        target = destination / 'rendered'
        # Retry only an interrupted/unavailable renderer, keeping every old file.
        # A completed render/validation failure remains a failed sampling attempt.
        if target.exists() or checkpoint.exists():
            index = 0
            while (destination / f'rendered.interrupted_{index:03d}').exists() or (
                    destination / f'render_execution.interrupted_{index:03d}.json').exists():
                index += 1
            if target.exists():
                target.rename(destination / f'rendered.interrupted_{index:03d}')
            if checkpoint.exists():
                checkpoint.rename(destination / f'render_execution.interrupted_{index:03d}.json')
            # These checks refer to the prior partial RGB output, not the physics.
            self.api.archive_unfinished_check(destination, 'validation', include_execution=True)
        request = self._renderer().execute(destination, target)
        request.update(render_output=str(target), renderer_directory=str(self.worker_dir))
        # RenderWorker replies are flattened; traceback/cleanup_error can carry
        # ENOSPC even when the top-level error reports only a cleanup failure.
        storage = self.api.storage_error_message(str(request))
        if storage:
            request['storage_error'] = storage
        self.api.write_json(checkpoint, request)
        return request

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.worker is not None:
            lifecycle = self.worker.close()
            self.api.write_json(self.worker_dir / 'lifecycle.json', lifecycle)
            if lifecycle.get('started') and (not lifecycle.get('process_reaped') or lifecycle.get('exit_code') != 0):
                return lifecycle.get('error') or 'Persistent renderer did not exit successfully'


class PhysicsProducer:
    """One owned physical subprocess at a time, with explicit completion signal.

    The main thread renders and commits records. Its interruption never abandons
    the worker or infers child completion from Thread.is_alive()/join alone.
    """
    def __init__(self, backend, jobs):
        self.jobs, self.error = jobs, None
        self.finished = threading.Event()

        def run():
            try:
                backend.collect(jobs)
            except BaseException as error:
                self.error = error
            finally:
                self.finished.set()

        self.thread = threading.Thread(target=run, name='native-physics-producer')
        self.thread.start()

    def wait(self):
        self.finished.wait()
        self.thread.join()
        if self.error is not None:
            raise self.error

    def drain(self):
        pending = None
        while not self.finished.is_set():
            try:
                self.finished.wait(.1)
            except BaseException as error:
                pending = pending or error
        while True:
            try:
                self.thread.join()
                break
            except BaseException as error:
                pending = pending or error
        if pending is not None:
            raise pending


def collect_vectorized(*, args, identifiers, entries, output, overrides, randomization_config,
                       manifest, attempt_records, asset_records, publish, resources, api):
    """Run the existing attempt/quota protocol with a different execution backend."""
    from kitchen_safety.randomization import sample_episode
    backend = VectorizedBackend(output, args, overrides, api)
    resources.callback(backend.close)
    started = time.monotonic()
    sessions = manifest.setdefault('execution_sessions', [])
    session = dict(started_at=api.now(), execution_backend='vectorized', elapsed_seconds=0,
                   scheduling=('bounded_physics_and_rgb_pipeline' if args.pipeline
                               else 'physics_batch_then_render_and_verify_serially'),
                   maximum_in_flight_attempts=2 * args.num_envs if args.pipeline else args.num_envs,
                   scope='Invocation wall time including shared physics, renderer, audits, validation and shutdown')
    sessions.append(session)

    def save():
        session['elapsed_seconds'] = time.monotonic() - started
        manifest['collection_wall_seconds'] = sum(item['elapsed_seconds'] for item in sessions)
        publish()

    def accept(destination, entry, attempt, seed):
        execution = backend.recover(destination)
        render = backend.render(destination, execution)
        record = api.verify_attempt(destination, entry, seed, False, execution,
            record_rgb=True, randomized=True, render_root=destination / 'rendered', render_execution=render)
        relative = str(destination.relative_to(output))
        attempt_records[relative] = dict(record, path=relative, attempt=attempt)
        print(f'VECTORIZED FINISH {entry["id"]} attempt={attempt} passed={record["passed"]} '
              f'kind={record.get("failure_kind")}', flush=True)
        if record.get('storage_error'):
            manifest.update(storage_error=record['storage_error'], stop_reason='storage_error')
            return api.STORAGE_ERROR_EXIT_CODE
        if record.get('worker_unavailable'):
            manifest.update(worker_unavailable=record.get('error') or 'Renderer worker unavailable',
                            stop_reason='worker_unavailable')
            return api.WORKER_UNAVAILABLE_EXIT_CODE
        return None

    producer = None
    try:
        for identifier in identifiers:
            entry = entries[identifier]
            existing = sorted(path for path in (output / identifier).glob('attempt_*') if path.is_dir())
            for destination in existing:
                attempt = api.next_attempt_index([destination], []) - 1
                spec = api.read_json(destination / 'episode_spec.json', {})
                seed = int(spec.get('seeds', {}).get('planner', destination.name.rsplit('_seed_', 1)[-1]))
                record = api.read_json(destination / 'verification.json')
                if record is not None and record.get('read_error'):
                    raise ValueError(f'Unreadable verification: {destination}')
                if record is None or ((record.get('storage_error') or record.get('worker_unavailable'))
                                      and record.get('execution_success')):
                    if record is not None:
                        api.archive_unfinished_check(destination, 'verification')
                    stop = accept(destination, entry, attempt, seed)
                    save()
                    if stop:
                        current = attempt_records[str(destination.relative_to(output))]
                        historical_shared_failure = current.get('execution', {}).get('shared_batch_error')
                        if current.get('execution_success') or (
                                stop != api.STORAGE_ERROR_EXIT_CODE and not historical_shared_failure):
                            return stop
                        # Historical collector storage failures consumed an
                        # attempt; successful physics with retryable RGB does not.
                        manifest.pop('storage_error', None)
                        manifest.pop('worker_unavailable', None)
                        manifest.pop('stop_reason', None)
                else:
                    relative = str(destination.relative_to(output))
                    attempt_records[relative] = dict(record, path=relative, attempt=attempt)
            outcomes = [r for r in attempt_records.values() if r.get('obstacle_id') == identifier]
            next_attempt = api.next_attempt_index(existing, outcomes)
            pending_jobs = []
            while True:
                api.refresh_verifications(output, attempt_records)
                outcomes = [r for r in attempt_records.values() if r.get('obstacle_id') == identifier]
                remaining = args.episodes_per_id - sum(r['passed'] for r in outcomes)
                budget = args.max_attempts - len(outcomes)
                if producer is not None and producer.finished.is_set():
                    producer.wait()
                    pending_jobs.extend(producer.jobs)
                    producer = None
                in_flight = len(pending_jobs) + (len(producer.jobs) if producer else 0)
                if (remaining <= 0 or budget <= 0) and in_flight == 0:
                    break
                capacity = (2 * args.num_envs if args.pipeline else args.num_envs) - in_flight
                # Every pending/completed-but-uncommitted attempt occupies one
                # quota slot. Thus even universal in-flight success cannot make
                # this invocation collect beyond the requested target.
                desired = min(args.num_envs, remaining - in_flight, budget - in_flight)
                count = desired if desired <= capacity else 0
                if (producer is None and not backend.shared_batch_error and count > 0
                        and (args.pipeline or not pending_jobs)):
                    jobs = []
                    for attempt in range(next_attempt, next_attempt + count):
                        spec = sample_episode(identifier, randomization_config, args.seed, attempt)
                        seed = int(spec['seeds']['planner'])
                        destination = output / identifier / f'attempt_{attempt:04d}_seed_{seed}'
                        destination.mkdir(parents=True)
                        api.write_json(destination / 'episode_spec.json', spec)
                        jobs.append(dict(destination=destination, id=identifier, attempt=attempt, seed=seed))
                    next_attempt += len(jobs)
                    if args.pipeline:
                        producer = PhysicsProducer(backend, jobs)
                    else:
                        backend.collect(jobs)
                        pending_jobs.extend(jobs)
                if pending_jobs:
                    job = pending_jobs[0]
                    stop = accept(job['destination'], entry, job['attempt'], job['seed'])
                    save()
                    pending_jobs.pop(0)  # Release quota only after record publication.
                    if stop:
                        # All results of this already-completed shared failure
                        # are retained and classified in this invocation. This
                        # does not run RGB/CPU checks for an unusable execution.
                        execution = api.read_json(job['destination'] / 'execution.json', {})
                        if execution.get('shared_batch_error'):
                            for sibling in list(pending_jobs):
                                sibling_execution = backend.recover(sibling['destination'])
                                if sibling_execution.get('batch_directory') != execution.get('batch_directory'):
                                    continue
                                accept(sibling['destination'], entry, sibling['attempt'], sibling['seed'])
                                save()
                                pending_jobs.remove(sibling)
                        return stop
                elif producer is not None:
                    producer.wait()
                    pending_jobs.extend(producer.jobs)
                    producer = None
                else:
                    raise RuntimeError('Vectorized scheduler has no pending work but cannot finish')
            asset_records[identifier] = api.asset_summary(identifier, entry, outcomes, args.episodes_per_id)
            asset_records[identifier]['attempt_budget_exhausted'] = len(outcomes) >= args.max_attempts
            save()
        close_error = backend.close()
        if close_error:
            manifest.update(worker_unavailable=close_error, stop_reason='worker_unavailable')
            return api.WORKER_UNAVAILABLE_EXIT_CODE
        manifest['collection_pass_finished'] = True
        manifest['complete'] = all(asset_records[item].get('success_count', 0) >= args.episodes_per_id
                                   for item in manifest['requested_ids'])
        manifest['previews'] = api.export_previews(output, list(asset_records.values()), args.preview_per_id)
        manifest['complete'] = manifest['complete'] and manifest['previews']['passed']
        manifest['all_passed'] = manifest['complete']
        if manifest['previews'].get('storage_error'):
            manifest.update(storage_error=manifest['previews']['storage_error'], stop_reason='storage_error')
            return api.STORAGE_ERROR_EXIT_CODE
        return 0 if manifest['complete'] else 1
    finally:
        try:
            if producer is not None:
                producer.drain()
        finally:
            close_error = backend.close()
            if close_error:
                manifest.update(worker_unavailable=close_error, stop_reason='worker_unavailable',
                                complete=False, all_passed=False)
            session['finished_at'] = api.now()
            save()
