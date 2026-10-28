import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from scripts.benchmark_deferred_rendering import main, run_benchmark, run_owned_command, write_json


class FakeRunner:
    def __init__(self, test, *, physics_fail=(), render_fail=(), validation_fail=(), grouped_camera=False):
        self.test = test
        self.physics_fail, self.render_fail, self.validation_fail = (
            set(physics_fail), set(render_fail), set(validation_fail))
        self.calls = []
        self.active = {'physics': 0, 'render': 0, 'validation': 0}
        self.max_active = dict(self.active)
        self.lock = threading.Lock()
        self.grouped_camera = grouped_camera

    def __call__(self, command, log_path, *, cwd):
        program = Path(command[2]).name
        role = {'collect_kitchen_vectorized.py': 'physics',
                'render_recorded_episode.py': 'render', 'validate_collection.py': 'validation'}[program]
        log_path.write_text('fake execution\n')
        value = lambda flag: command[command.index(flag) + 1]
        with self.lock:
            self.active[role] += 1
            self.max_active[role] = max(self.max_active[role], self.active[role])
        try:
            if role == 'physics':
                self.test.assertNotIn('--grouped-camera', command)
                self.test.assertNotIn('--record-rgb', command)
                self.test.assertIn('--record-render-state', command)
                self.test.assertIn('--defer-clearance-diagnostics', command)
                self.test.assertEqual(value('--physics-threads'), '0')
                self.test.assertEqual(value('--validation-workers'), '2')
                self.test.assertEqual(value('--device'), 'cpu')
                output = cwd / value('--output')
                self.test.assertFalse(output.exists())
                output.mkdir()
                specs = json.loads((cwd / value('--episode-specs')).read_text())
                self.test.assertEqual(int(value('--num-envs')), len(specs))
                self.test.assertEqual(len({spec['obstacle_id'] for spec in specs}), 1)
                for env, spec in enumerate(specs):
                    index = spec['test_index']
                    self.calls.append((role, index))
                    directory = output / f'env_{env:03d}'
                    directory.mkdir()
                    write_json(directory / 'episode_spec.json', spec)
                    write_json(directory / 'result.json', {'success': index not in self.physics_fail,
                                                           'steps': 3, 'failure_kind': 'physical_failed'})
                write_json(output / 'batch.json', {'status': 'completed'})
            else:
                if role == 'render':
                    self.test.assertEqual('--grouped-camera' in command, self.grouped_camera)
                    self.test.assertNotIn('--num-envs', command)
                    self.test.assertEqual(value('--num_envs'), '1')
                    self.test.assertEqual(value('--camera-device'), 'cuda:0')
                    self.test.assertIn('--background-rgb', command)
                    source = cwd / value('--source-episode')
                else:
                    self.test.assertNotIn('--grouped-camera', command)
                    source = cwd / command[3]
                    self.test.assertIn('--require-rgb', command)
                index = json.loads((source / 'episode_spec.json').read_text())['test_index']
                self.calls.append((role, index))
                if role == 'render':
                    output = cwd / value('--output')
                    output.mkdir()
                    ok = index not in self.render_fail
                    write_json(output / 'render_report.json', {
                        'completed': ok, 'complete_trajectory': ok,
                        'partial': not ok, 'rendered_frames': 3 if ok else 1})
                else:
                    ok = index not in self.validation_fail
                    write_json(cwd / value('--report'), [{'passed': ok, 'path': str(source)}])
            return {'exit_code': 0, 'elapsed_seconds': .1}
        finally:
            with self.lock:
                self.active[role] -= 1


class DeferredRenderingTests(unittest.TestCase):
    def make_input(self, root, count=4):
        path = root / 'input.json'
        specs = [{'obstacle_id': 'vase' if index < 2 else 'steel', 'test_index': index,
                  'layout': {'unchanged': index}, 'seeds': {'planner': index}}
                 for index in range(count)]
        write_json(path, specs)
        return path

    def test_serial_and_pipeline_preserve_all_specs_and_unique_batch_coverage(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self.make_input(root)
            results = []
            for pipeline in (False, True):
                runner = FakeRunner(self)
                report = run_benchmark(source, root / str(pipeline), pipeline=pipeline,
                                       root=root, runner=runner)
                results.append(report)
                self.assertEqual(report['counts']['qualified'], 4)
                self.assertEqual(report['status'], 'completed')
                self.assertTrue(report['all_owned_subprocesses_finished'])
                self.assertFalse(report['production_quota_eligible'])
                self.assertEqual(report['queue_capacity'], 1)
                self.assertEqual(report['configuration']['queue_capacity_episodes'], 1 if pipeline else 0)
                self.assertFalse(report['configuration']['grouped_camera'])
                self.assertEqual(report['validated_successes_per_hour'], 4 * 3600 / report['total_wall_s'])
                for role in ('physics', 'render', 'validation'):
                    self.assertEqual(sorted(i for kind, i in runner.calls if kind == role), list(range(4)))
                    self.assertEqual(runner.max_active[role], 1)
                if not pipeline:
                    self.assertEqual(runner.calls, [('physics', 0), ('physics', 1),
                        ('render', 0), ('validation', 0), ('render', 1), ('validation', 1),
                        ('physics', 2), ('physics', 3), ('render', 2), ('validation', 2),
                        ('render', 3), ('validation', 3)])
            self.assertEqual([b['episode_indices'] for b in results[0]['batches']],
                             [b['episode_indices'] for b in results[1]['batches']])

    def test_physics_render_and_validation_failures_remain_distinct(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            runner = FakeRunner(self, physics_fail=[0], render_fail=[1], validation_fail=[2])
            report = run_benchmark(self.make_input(root), root / 'run', pipeline=True, root=root, runner=runner)
            self.assertEqual(report['counts'], {'physics_success': 3, 'render_complete': 2,
                                                'validation_passed': 1, 'qualified': 1})
            self.assertEqual(report['failure_counts']['physics_failed'], 1)
            self.assertEqual(report['failure_counts']['render_failed'], 1)
            self.assertEqual(report['failure_counts']['validation_failed'], 1)
            self.assertNotIn(('render', 0), runner.calls)
            self.assertNotIn(('validation', 1), runner.calls)
            self.assertTrue((root / 'run/batch_000/physics/env_000/result.json').exists())
            self.assertTrue((root / 'run/episode_00001/rendered/render_report.json').exists())

    def test_host_error_waits_for_inflight_consumer_and_stops_new_launches(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self.make_input(root)
            fake = FakeRunner(self)
            render_started, producer_failed, release_render = [threading.Event() for _ in range(3)]
            finished = threading.Event()
            result = []

            def runner(command, log_path, *, cwd):
                if command[2].endswith('collect_kitchen_vectorized.py') and 'batch_001' in str(log_path):
                    self.assertTrue(render_started.wait(3))
                    producer_failed.set()
                    raise OSError('fake launch failed')
                if command[2].endswith('render_recorded_episode.py'):
                    render_started.set()
                    self.assertTrue(release_render.wait(3))
                return fake(command, log_path, cwd=cwd)

            def run():
                result.append(run_benchmark(source, root / 'run', pipeline=True, root=root, runner=runner))
                finished.set()

            thread = threading.Thread(target=run)
            thread.start()
            try:
                self.assertTrue(producer_failed.wait(3))
                self.assertFalse(finished.is_set())
            finally:
                release_render.set()
                thread.join(5)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result[0]['status'], 'host_error')
            self.assertIsNone(result[0]['validated_successes_per_hour'])
            self.assertEqual(result[0]['counts']['qualified'], 0)
            self.assertEqual([index for role, index in fake.calls if role == 'render'], [0])
            self.assertFalse(any(role == 'validation' for role, _ in fake.calls))
            failed_timing = result[0]['batches'][1]['physics_timing']
            self.assertIsNotNone(failed_timing['finish_relative_s'])
            self.assertGreaterEqual(failed_timing['finish_relative_s'], failed_timing['start_relative_s'])

    def test_interrupted_docker_wait_reaps_process_before_reraising(self):
        with tempfile.TemporaryDirectory() as temp:
            child = Mock(pid=123, wait=Mock(side_effect=[KeyboardInterrupt(), 0]))
            with patch('scripts.benchmark_deferred_rendering.subprocess.Popen', return_value=child) as popen:
                with self.assertRaises(KeyboardInterrupt):
                    run_owned_command(['fake-command'], Path(temp) / 'log', cwd=Path(temp))
            self.assertEqual(child.wait.call_count, 2)
            child.terminate.assert_not_called()
            child.kill.assert_not_called()
            self.assertTrue(popen.call_args.kwargs['start_new_session'])

    def test_repeated_interrupts_during_join_cannot_skip_other_role_cleanup(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            runner = FakeRunner(self)
            original_join = threading.Thread.join
            calls = []
            remaining_interrupts = [2]

            def interrupted_join(thread, *args, **kwargs):
                calls.append(thread.name)
                # No role may still own a child when thread joining begins.
                self.assertEqual(runner.active, {'physics': 0, 'render': 0, 'validation': 0})
                self.assertEqual(sorted(i for role, i in runner.calls if role == 'validation'), list(range(4)))
                if remaining_interrupts[0]:
                    remaining_interrupts[0] -= 1
                    raise KeyboardInterrupt()
                return original_join(thread, *args, **kwargs)

            with patch.object(threading.Thread, 'join', interrupted_join):
                report = run_benchmark(self.make_input(root), root / 'run', pipeline=True,
                                       root=root, runner=runner)
            self.assertEqual(calls, ['physics-producer'] * 3 + ['rgb-consumer'])
            self.assertEqual(len(report['host_errors']), 2)
            self.assertEqual(report['counts']['qualified'], 4)
            self.assertTrue(report['all_owned_subprocesses_finished'])

    def test_fixed_episode_queue_capacity_is_independent_of_physics_batch_size(self):
        import queue
        original_queue = queue.Queue
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / 'input.json'
            write_json(source, [{'obstacle_id': 'vase', 'test_index': i} for i in range(12)])
            queue_sizes = []

            def make_queue(*args, **kwargs):
                queue_sizes.append(kwargs['maxsize'])
                return original_queue(*args, **kwargs)

            for size in (2, 4):
                runner = FakeRunner(self, grouped_camera=True)
                with patch('scripts.benchmark_deferred_rendering.queue.Queue', side_effect=make_queue):
                    report = run_benchmark(source, root / f'run_{size}', batch_size=size,
                        pipeline=True, queue_capacity=6, grouped_camera=True, root=root, runner=runner)
                self.assertEqual(report['configuration']['queue_capacity_episodes'], 6)
                self.assertTrue(report['configuration']['grouped_camera'])
                self.assertEqual(report['counts']['qualified'], 12)
                self.assertEqual([len(batch['episode_indices']) for batch in report['batches']], [size] * (12 // size))
                self.assertEqual(sorted(i for role, i in runner.calls if role == 'render'), list(range(12)))
            self.assertEqual(queue_sizes, [6, 6])

    def test_role_intervals_use_one_benchmark_clock_and_include_validation(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            now, fake = [100.], FakeRunner(self)

            def runner(command, log_path, *, cwd):
                result = fake(command, log_path, cwd=cwd)
                now[0] += {'collect_kitchen_vectorized.py': 2., 'render_recorded_episode.py': 3.,
                           'validate_collection.py': 1.}[Path(command[2]).name]
                return result

            report = run_benchmark(self.make_input(root, count=2), root / 'run', root=root,
                                   runner=runner, clock=lambda: now[0])
            self.assertEqual(report['batches'][0]['physics_timing'],
                             {'start_relative_s': 0., 'finish_relative_s': 2., 'wall_s': 2.})
            for row, start in zip(report['episodes'], (2., 6.)):
                self.assertEqual(row['render_timing'],
                                 {'start_relative_s': start, 'finish_relative_s': start + 3., 'wall_s': 3.})
                self.assertEqual(row['validation_timing'],
                                 {'start_relative_s': start + 3., 'finish_relative_s': start + 4., 'wall_s': 1.})
            self.assertEqual(report['total_wall_s'], 10.)
            self.assertEqual(report['validated_successes_per_hour'], 720.)

    def test_cli_defaults_overrides_and_nonpositive_queue_rejection(self):
        base = ['benchmark', '--episode-specs', 'input.json', '--output', 'new_output']
        for extra, capacity, grouped in (([], 1, False), (['--queue-capacity', '6', '--grouped-camera'], 6, True)):
            with patch('sys.argv', base + extra), patch('scripts.benchmark_deferred_rendering.run_benchmark',
                    return_value=dict(status='completed', counts={}, failure_counts={}, total_wall_s=1.,
                                      validated_successes_per_hour=0.)) as run, patch('builtins.print'):
                self.assertEqual(main(), 0)
                self.assertEqual(run.call_args.kwargs['queue_capacity'], capacity)
                self.assertEqual(run.call_args.kwargs['grouped_camera'], grouped)
                self.assertFalse(run.call_args.kwargs['persistent_renderer'])
                self.assertFalse(run.call_args.kwargs['refresh_render_products'])
        with patch('sys.argv', base + ['--persistent-renderer', '--refresh-render-products']), patch(
                'scripts.benchmark_deferred_rendering.run_benchmark',
                return_value=dict(status='completed', counts={}, failure_counts={}, total_wall_s=1.,
                                  validated_successes_per_hour=0.)) as run, patch('builtins.print'):
            main()
            self.assertTrue(run.call_args.kwargs['persistent_renderer'])
            self.assertTrue(run.call_args.kwargs['refresh_render_products'])
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self.make_input(root)
            for capacity in (0, -1):
                with self.assertRaisesRegex(ValueError, 'queue-capacity must be positive'):
                    run_benchmark(source, root / f'bad_{capacity}', queue_capacity=capacity, root=root)
                self.assertFalse((root / f'bad_{capacity}').exists())

    def test_persistent_requests_keep_validation_and_include_shared_shutdown(self):
        with tempfile.TemporaryDirectory() as temp:
            root, instances, now = Path(temp), [], [100.]
            fake = FakeRunner(self, render_fail=[1])
            class Worker:
                def __init__(self, **kwargs):
                    self.options, self.closed, self.requests = kwargs, False, []
                    instances.append(self)
                def execute(self, source, output):
                    self.requests.append(str(source))
                    command = ['bash', 'scripts/arena_python.sh', 'scripts/render_recorded_episode.py',
                        '--source-episode', str(source.relative_to(root)), '--output', str(output.relative_to(root)),
                        '--num_envs', '1', '--camera-device', 'cuda:0', '--background-rgb']
                    fake(command, output.parent / 'fake.log', cwd=root)
                    now[0] += 3.
                    ok = json.loads((output / 'render_report.json').read_text())['completed']
                    return {'status': 'completed' if ok else 'failed', 'request_wall_s': 3.}
                def close(self):
                    self.closed = True
                    now[0] += 5.
                    return self.report()
                def report(self):
                    return {'started': bool(self.requests), 'process_reaped': self.closed,
                            'exit_code': 0 if self.closed else None}
            report = run_benchmark(self.make_input(root, count=2), root / 'run', root=root,
                persistent_renderer=True, refresh_render_products=True, runner=fake,
                worker_factory=Worker, clock=lambda: now[0])
            self.assertEqual(len(instances), 1)
            self.assertTrue(instances[0].options['refresh_render_products'])
            self.assertEqual(len(instances[0].requests), 2)
            self.assertTrue(instances[0].closed)
            self.assertEqual(report['counts']['qualified'], 1)
            self.assertEqual(report['failure_counts']['render_failed'], 1)
            self.assertEqual([i for role, i in fake.calls if role == 'validation'], [0])
            self.assertEqual(report['total_wall_s'], 11.)
            self.assertEqual(report['renderer_shutdown_timing']['wall_s'], 5.)
            self.assertTrue(report['renderer_worker']['process_reaped'])
            for row in report['episodes']:
                self.assertNotIn('render_execution', row)
                self.assertNotIn('render_command', row)
                self.assertNotIn('exit_code', row['render_request'])
                self.assertIn('render_request_timing', row)

    def test_unavailable_persistent_worker_stops_requests_and_closes_before_return(self):
        with tempfile.TemporaryDirectory() as temp:
            root, instances = Path(temp), []
            fake = FakeRunner(self)
            class Worker:
                def __init__(self, **kwargs):
                    self.requests, self.closed = 0, False
                    instances.append(self)
                def execute(self, source, output):
                    self.requests += 1
                    return {'status': 'worker_unavailable', 'worker_unavailable': True,
                            'error': 'scene cleanup failed'}
                def close(self):
                    self.closed = True
                    return self.report()
                def report(self):
                    return {'started': True, 'exit_code': 1, 'process_reaped': self.closed}
            report = run_benchmark(self.make_input(root), root / 'run', root=root, pipeline=True,
                persistent_renderer=True, runner=fake, worker_factory=Worker)
            self.assertEqual(instances[0].requests, 1)
            self.assertTrue(instances[0].closed)
            self.assertEqual(report['status'], 'host_error')
            self.assertEqual(report['counts']['qualified'], 0)
            self.assertTrue(report['renderer_worker']['process_reaped'])
            self.assertFalse(any(role == 'validation' for role, _ in fake.calls))


if __name__ == '__main__':
    unittest.main()
