"""Production scheduling tests with fake child processes; no Kit or USD."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import collect_obstacle_library as batch
from kitchen_safety.parallel_collection import VectorizedBackend


class FakeRuntime:
    def __init__(self, *, ineligible=(), failed=(), unavailable_once=False, infrastructure_failure=None):
        self.ineligible, self.failed = ineligible, failed
        self.unavailable_once = unavailable_once
        self.infrastructure_failure = infrastructure_failure
        self.physics_calls, self.render_calls, self.checks, self.workers, self.samples = [], [], [], [], []

    def sample(self, identifier, config, seed, attempt):
        self.samples.append((identifier, seed, attempt))
        return dict(obstacle_id=identifier, attempt=attempt,
                    seeds=dict(geometry=seed + attempt, appearance=100 + seed + attempt,
                               planner=1000 + seed + attempt), layout={'offset': attempt},
                    appearance={'light': attempt})

    def run(self, command, log):
        log.write_text('fake child log\n')
        script = Path(command[1]).name
        if script == 'collect_kitchen_vectorized.py':
            self.physics_calls.append(command)
            output = Path(command[command.index('--output') + 1])
            output.mkdir()
            specs = json.loads(Path(command[command.index('--episode-specs') + 1]).read_text())
            failure = self.infrastructure_failure if len(self.physics_calls) == 1 else None
            shared_report = {'status': 'completed'}
            if failure == 'execution_error':
                shared_report['execution_error'] = 'RuntimeError: CUDA out of memory in shared env.step'
            elif failure == 'startup':
                shared_report.update(status='error', error='RuntimeError: simulator startup failed')
            batch.write_json(output / 'batch.json', shared_report)
            if failure == 'startup':
                return dict(exit_code=17, elapsed_seconds=12)
            for index, spec in enumerate(specs):
                env = output / f'env_{index:03d}'
                env.mkdir()
                batch.write_json(env / 'episode_spec.json', spec)
                attempt = spec['attempt']
                batch.write_json(env / 'result.json', dict(success=attempt not in self.failed,
                    steps=3, augmentation_eligible=attempt not in self.ineligible,
                    production_quota_eligible=False, collection_purpose='production',
                    failure_kind='planning_failed' if attempt in self.failed else None))
                (env / 'trajectory.h5').write_bytes(b'physical trajectory')
            return dict(exit_code=17 if failure == 'abnormal_exit' else 0, elapsed_seconds=12)
        self.checks.append((script, command))
        report = Path(command[command.index('--report') + 1])
        batch.write_json(report, [{'passed': True}] if script == 'validate_collection.py'
                         else {'passed': True})
        return dict(exit_code=0, elapsed_seconds=.01)

    def worker(self, **kwargs):
        owner = self

        class Worker:
            def __init__(self):
                self.closed = False
                self.directory = Path(kwargs['directory'])
                self.directory.mkdir()

            def execute(self, source, output):
                owner.render_calls.append((source, output))
                output.mkdir()
                if owner.unavailable_once:
                    owner.unavailable_once = False
                    (output / 'partial.txt').write_text('preserved')
                    return dict(status='worker_unavailable', worker_unavailable=True,
                                error='fake unexpected renderer exit', request_wall_s=.01)
                (output / 'images.h5').write_bytes(b'encoded RGB')
                batch.write_json(output / 'cameras.json', {})
                batch.write_json(output / 'render_report.json',
                    {'appearance': batch.read_json(source / 'episode_spec.json')['appearance']})
                return dict(status='completed', request_wall_s=.01)

            def close(self):
                self.closed = True
                return dict(started=True, process_reaped=True, exit_code=0)

        instance = Worker()
        self.workers.append(instance)
        return instance

    def install(self, stack):
        module = types.ModuleType('kitchen_safety.randomization')
        module.SUPPORTED_IDS = ('property_vase', 'property_tall_glass')
        module.load_randomization_config = lambda path: {'version': 1}
        module.sample_episode = self.sample
        worker_module = types.ModuleType('kitchen_safety.render_worker')
        worker_module.RenderWorker = self.worker
        stack.enter_context(patch.dict(sys.modules, {'kitchen_safety.randomization': module,
                                                    'kitchen_safety.render_worker': worker_module}))
        stack.enter_context(patch.object(batch, 'run_command', side_effect=self.run))
        stack.enter_context(patch.object(batch, 'load_assets', return_value=[
            {'id': 'property_vase', 'category': 'property'},
            {'id': 'property_tall_glass', 'category': 'property'}]))


class ParallelCollectionTests(unittest.TestCase):
    def flags(self, output, target=2, budget=4):
        return ['--output', str(output), '--ids', 'property_vase', '--record-rgb',
                '--randomization-config', 'unused.json', '--execution-backend', 'vectorized',
                '--execution-checks', 'deferred', '--device', 'cpu', '--physics-threads', '0',
                '--torch-threads', '1', '--camera-device', 'cuda:0', '--background-rgb',
                '--num-envs', '2', '--episodes-per-id', str(target), '--max-attempts', str(budget)]

    def test_explicit_episode_timeout_reaches_native_collector_and_resume_configuration(self):
        from contextlib import ExitStack
        fake = FakeRuntime()
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            output = Path(directory)
            flags = self.flags(output) + ['--episode-length-s', '180']
            self.assertEqual(batch.main(flags), 0)
            report = batch.read_json(output / 'manifest.json')
            self.assertEqual(report['configuration']['runtime']['episode_length_s'], 180.0)
            command = fake.physics_calls[0]
            self.assertEqual(command.count('--episode-length-s'), 1)
            self.assertEqual(float(command[command.index('--episode-length-s')+1]), 180.)
            self.assertNotIn('--max-steps', command)  # Existing native default remains 2400.
            self.assertEqual(batch.main(flags + ['--resume']), 0)
            with self.assertRaisesRegex(ValueError, 'configuration differs'):
                batch.main(self.flags(output) + ['--resume'])
            self.assertEqual(len(fake.physics_calls), 1)

    def test_imported_acceptances_keep_counts_but_do_not_inflate_new_throughput(self):
        from contextlib import ExitStack
        fake = FakeRuntime()
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            output = Path(directory)
            self.assertEqual(batch.main(self.flags(output, target=2)), 0)
            imported = batch.read_json(output / 'manifest.json')
            imported['imported_history'] = {'success_count': 1, 'source_manifest': 'old/manifest.json'}
            reviewed_path = output / imported['attempts'][1]['path'] / 'verification.json'
            reviewed = batch.read_json(reviewed_path)
            reviewed['historical_eligibility_review'] = {'reason': 'user-approved-side-risk'}
            batch.write_json(reviewed_path, reviewed)
            imported['execution_sessions'] = []
            imported['collection_wall_seconds'] = 0
            batch.write_json(output / 'manifest.json', imported)

            self.assertEqual(batch.main(self.flags(output, target=3) + ['--resume']), 0)
            statistics = batch.read_json(output / 'manifest.json')['statistics']
            self.assertEqual(statistics['success_count'], 3)
            self.assertEqual(statistics['accepted_frames'], 9)
            self.assertEqual(statistics['imported_success_count'], 1)
            self.assertEqual(statistics['historical_reclassified_success_count'], 1)
            self.assertEqual(statistics['new_success_count'], 1)
            self.assertAlmostEqual(statistics['successful_physical_trajectories_per_hour'],
                                   3600 / statistics['collection_wall_seconds'])
            self.assertIn('excluding imported_history.success_count', statistics['throughput_time_scope'])

    def test_success_uses_original_attempts_hardlinks_and_full_acceptance(self):
        from contextlib import ExitStack
        fake = FakeRuntime(ineligible=(0,), failed=(1,))
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            output = Path(directory)
            self.assertEqual(batch.main(self.flags(output)), 0)
            report = batch.read_json(output / 'manifest.json')
            self.assertEqual([r['passed'] for r in report['attempts']], [False, False, True, True])
            self.assertEqual([r['failure_kind'] for r in report['attempts'][:2]],
                             ['obstruction_unconfirmed', 'planning_failed'])
            self.assertEqual(len(fake.physics_calls), 2)
            self.assertEqual(len(fake.render_calls), 3)  # Physical failures never render.
            self.assertTrue(fake.workers[0].closed)
            self.assertEqual(len(fake.workers), 1)
            for record in report['attempts']:
                destination = output / record['path']
                source = Path(record['execution']['physics_source'])
                self.assertEqual((source / 'trajectory.h5').stat().st_ino,
                                 (destination / 'trajectory.h5').stat().st_ino)
                self.assertEqual(batch.read_json(destination / 'result.json')['production_quota_eligible'], False)
                self.assertEqual(record['production_quota_eligible'], record['passed'])
                if record['execution_success']:
                    self.assertEqual(record['checks']['audit']['exit_code'], 0)
                    self.assertEqual(record['checks']['validation']['exit_code'], 0)
            validation = [command for name, command in fake.checks if name == 'validate_collection.py']
            self.assertTrue(all('--render-output' in command and '--require-rgb' in command for command in validation))
            for command in fake.physics_calls:
                self.assertNotIn('--record-rgb', command)
                self.assertNotIn('--episode-length-s', command)  # Legacy 120 s is unchanged.
                for flag in ('--record-render-state', '--job-mapping', '--collector-overrides', '--collection-purpose'):
                    self.assertIn(flag, command)
            self.assertEqual(report['statistics']['collection_wall_seconds'],
                             report['execution_sessions'][0]['elapsed_seconds'])
            self.assertNotEqual(report['statistics']['collection_wall_seconds'], 48)
            self.assertEqual(batch.main(self.flags(output) + ['--resume']), 0)
            self.assertEqual(len(fake.physics_calls), 2)
            self.assertEqual(len(fake.render_calls), 3)
            with self.assertRaisesRegex(ValueError, 'configuration differs'):
                flags = self.flags(output)
                flags[flags.index('--num-envs') + 1] = '3'
                batch.main(flags + ['--resume'])

    def test_renderer_interruption_resumes_physics_and_preserves_partial_output(self):
        from contextlib import ExitStack
        fake = FakeRuntime(unavailable_once=True)
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            output = Path(directory)
            self.assertEqual(batch.main(self.flags(output)), batch.WORKER_UNAVAILABLE_EXIT_CODE)
            first = sorted((output / 'property_vase').glob('attempt_*'))[0]
            original = (first / 'execution.json').read_bytes()
            self.assertEqual(batch.main(self.flags(output) + ['--resume']), 0)
            self.assertEqual(len(fake.physics_calls), 1)
            self.assertEqual(len(fake.samples), 2)
            self.assertEqual((first / 'execution.json').read_bytes(), original)
            self.assertTrue((first / 'rendered.interrupted_000/partial.txt').is_file())
            report = batch.read_json(output / 'manifest.json')
            self.assertEqual(report['statistics']['success_count'], 2)
            self.assertEqual(report['statistics']['attempt_count'], 2)
            self.assertTrue(all(worker.closed for worker in fake.workers))

    def test_subset_resume_keeps_indices_seeds_and_does_not_overproduce_target(self):
        from contextlib import ExitStack
        fake = FakeRuntime()
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            output = Path(directory)
            self.assertEqual(batch.main(self.flags(output, target=1)), 0)
            original = batch.read_json(output / 'manifest.json')['attempts'][0]
            self.assertEqual(batch.main(self.flags(output, target=3) + ['--resume']), 0)
            report = batch.read_json(output / 'manifest.json')
            self.assertEqual(report['statistics']['success_count'], 3)
            self.assertEqual(report['attempts'][0]['seeds'], original['seeds'])
            self.assertEqual([r['attempt'] for r in report['attempts']], [0, 1, 2])
            self.assertEqual([c[c.index('--num-envs') + 1] for c in fake.physics_calls], ['1', '2'])
            other = self.flags(output, target=3)
            other[other.index('--ids') + 1] = 'property_tall_glass'
            self.assertEqual(batch.main(other + ['--resume']), 0)
            report = batch.read_json(output / 'manifest.json')
            self.assertEqual(report['statistics']['success_count'], 6)
            self.assertEqual(len(report['requested_ids']), 2)
            self.assertEqual([a for identifier, _, a in fake.samples if identifier == 'property_tall_glass'], [0, 1, 2])

    def test_budget_shortfall_is_not_complete(self):
        from contextlib import ExitStack
        fake = FakeRuntime(ineligible=(0, 1, 2))
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            self.assertEqual(batch.main(self.flags(directory, target=2, budget=3)), 1)
            report = batch.read_json(Path(directory) / 'manifest.json')
            self.assertFalse(report['complete'])
            self.assertEqual(report['statistics']['attempt_count'], 3)
            self.assertEqual(report['statistics']['deficit'], 2)

    def test_pipeline_overlaps_but_never_reserves_more_than_quota(self):
        from contextlib import ExitStack
        fake = FakeRuntime()
        second_physics_started = threading.Event()
        original_run, original_worker = fake.run, fake.worker

        def run(command, log):
            if Path(command[1]).name == 'collect_kitchen_vectorized.py' and fake.physics_calls:
                second_physics_started.set()
            return original_run(command, log)

        def worker(**kwargs):
            instance = original_worker(**kwargs)
            execute = instance.execute

            def render(source, output):
                if not fake.render_calls:
                    self.assertTrue(second_physics_started.wait(2), 'Producer must overlap first rendering')
                return execute(source, output)

            instance.execute = render
            return instance

        fake.run, fake.worker = run, worker
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            self.assertEqual(batch.main(self.flags(directory, target=5, budget=10) + ['--pipeline']), 0)
            report = batch.read_json(Path(directory) / 'manifest.json')
            self.assertEqual(report['statistics']['success_count'], 5)
            self.assertEqual(len(fake.samples), 5)
            self.assertEqual([r['attempt'] for r in report['attempts']], list(range(5)))
            self.assertEqual([c[c.index('--num-envs') + 1] for c in fake.physics_calls], ['2', '2', '1'])
            self.assertEqual(report['execution_sessions'][0]['scheduling'], 'bounded_physics_and_rgb_pipeline')
            self.assertEqual(report['execution_sessions'][0]['maximum_in_flight_attempts'], 4)

    def test_pipeline_error_drains_owned_producer_and_resumes_completed_physics(self):
        from contextlib import ExitStack
        fake = FakeRuntime(unavailable_once=True)
        second_started, allow_finish, second_finished = (threading.Event() for _ in range(3))
        original_run, original_worker = fake.run, fake.worker

        def run(command, log):
            second = Path(command[1]).name == 'collect_kitchen_vectorized.py' and bool(fake.physics_calls)
            if second:
                second_started.set()
                if not allow_finish.wait(2):
                    raise AssertionError('Consumer did not run concurrently')
            result = original_run(command, log)
            if second:
                second_finished.set()
            return result

        def worker(**kwargs):
            instance = original_worker(**kwargs)
            execute = instance.execute

            def render(source, output):
                if not fake.render_calls:
                    self.assertTrue(second_started.wait(2))
                    allow_finish.set()
                return execute(source, output)

            instance.execute = render
            return instance

        fake.run, fake.worker = run, worker
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            flags = self.flags(directory, target=4, budget=8) + ['--pipeline']
            self.assertEqual(batch.main(flags), batch.WORKER_UNAVAILABLE_EXIT_CODE)
            self.assertTrue(second_finished.is_set())
            self.assertEqual(len(fake.samples), 4)
            self.assertTrue(all(worker.closed for worker in fake.workers))
            self.assertEqual(batch.main(flags + ['--resume']), 0)
            self.assertEqual(len(fake.physics_calls), 2)
            self.assertEqual(len(fake.samples), 4)
            report = batch.read_json(Path(directory) / 'manifest.json')
            self.assertEqual(report['statistics']['success_count'], 4)

    def test_shared_physics_failure_stops_without_spending_retry_budget_and_can_resume(self):
        from contextlib import ExitStack
        for failure in ('startup', 'abnormal_exit', 'execution_error'):
            for pipeline in (False, True):
                with self.subTest(failure=failure, pipeline=pipeline), \
                        tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
                    fake = FakeRuntime(infrastructure_failure=failure)
                    fake.install(stack)
                    flags = self.flags(directory, target=4, budget=100)
                    if pipeline:
                        flags.append('--pipeline')
                    self.assertEqual(batch.main(flags), batch.WORKER_UNAVAILABLE_EXIT_CODE)
                    self.assertEqual(len(fake.physics_calls), 1)
                    self.assertEqual(len(fake.samples), 2)
                    self.assertEqual(fake.render_calls, [])
                    report = batch.read_json(Path(directory) / 'manifest.json')
                    self.assertEqual(len(report['attempts']), 2)
                    self.assertEqual(report['stop_reason'], 'worker_unavailable')
                    self.assertFalse(report['complete'])
                    for record in report['attempts']:
                        self.assertFalse(record['execution_success'])
                        self.assertFalse(record['passed'])
                        self.assertTrue(record['shared_batch_error'])
                        self.assertEqual(record['execution']['exit_code'], 0 if failure == 'execution_error' else 17)
                        self.assertEqual(record['failure_kind'], 'worker_unavailable')
                        self.assertEqual(record['failure_phase'], 'collection')
                        if failure != 'startup':
                            result = batch.read_json(Path(directory) / record['path'] / 'result.json')
                            self.assertTrue(result['success'])  # Raw physical evidence is never rewritten.
                    original = [(Path(directory) / r['path'] / 'execution.json').read_bytes()
                                for r in report['attempts']]
                    self.assertEqual(batch.main(flags + ['--resume']), 0)
                    report = batch.read_json(Path(directory) / 'manifest.json')
                    self.assertEqual(report['statistics']['success_count'], 4)
                    self.assertEqual(report['statistics']['attempt_count'], 6)
                    self.assertEqual(len(fake.physics_calls), 3)
                    self.assertEqual(original, [(Path(directory) / r['path'] / 'execution.json').read_bytes()
                                                for r in report['attempts'][:2]])

    def test_all_individual_physics_failures_are_not_shared_infrastructure_errors(self):
        from contextlib import ExitStack
        fake = FakeRuntime(failed=(0, 1))
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            self.assertEqual(batch.main(self.flags(directory) + ['--pipeline']), 0)
            report = batch.read_json(Path(directory) / 'manifest.json')
            sizes = [int(command[command.index('--num-envs') + 1]) for command in fake.physics_calls]
            self.assertEqual(sum(sizes), 4)
            self.assertTrue(all(1 <= size <= 2 for size in sizes))
            self.assertEqual(len(fake.samples), 4)
            self.assertEqual([r['failure_kind'] for r in report['attempts'][:2]],
                             ['planning_failed', 'planning_failed'])
            self.assertTrue(all(not r.get('shared_batch_error') for r in report['attempts']))

    def test_flattened_renderer_storage_trace_stops_collection(self):
        from contextlib import ExitStack
        fake = FakeRuntime()
        original_worker = fake.worker

        def worker(**kwargs):
            instance = original_worker(**kwargs)
            execute = instance.execute

            def render(source, output):
                result = execute(source, output)
                return dict(result, status='failed', error='Renderer cleanup failed',
                            cleanup_error='OSError: [Errno 28] No space left on device')

            instance.execute = render
            return instance

        fake.worker = worker
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            fake.install(stack)
            self.assertEqual(batch.main(self.flags(directory)), batch.STORAGE_ERROR_EXIT_CODE)
            report = batch.read_json(Path(directory) / 'manifest.json')
            self.assertEqual(report['stop_reason'], 'storage_error')
            self.assertEqual(report['attempts'][0]['failure_phase'], 'render')
            self.assertEqual(len(fake.physics_calls), 1)


if __name__ == '__main__':
    unittest.main()
