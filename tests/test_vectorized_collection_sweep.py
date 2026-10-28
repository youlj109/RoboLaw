import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.run_vectorized_collection_sweep import classify_run_failure, main, run_sweep, write_json


class SweepTests(unittest.TestCase):
    def test_warmup_failure_keeps_evidence_and_continues_smaller_count_in_input_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            spec = {'obstacle_id': 'test_asset', 'layout': {'mustard_position': [1., 2., 3.]},
                    'seeds': {'planner': 91}, 'appearance': {'lights': {}}}
            source = root / 'source.json'
            write_json(source, spec)
            now, order = [0.], []

            def launch(command, log_path, gpu_path, *, cwd):
                n = int(command[command.index('--num-envs')+1]);order.append(('collect', n))
                self.assertNotIn('--headless', command)
                self.assertNotIn('--fixed-appearance-benchmark', command)
                self.assertNotIn('--camera-device', command)
                self.assertNotIn('--physics-threads', command)
                self.assertNotIn('--defer-clearance-diagnostics', command)
                for flag in ('--background-rgb', '--reuse-phase-planner', '--curobo-cuda-graph'):
                    self.assertNotIn(flag, command)
                for flag, expected in [('--device', 'cuda:0'), ('--lighting', 'shared'),
                                       ('--warmup-mode', 'single_frame'), ('--validation-workers', '4'),
                                       ('--warmup-window-frames', '16'), ('--warmup-max-frames', '1024')]:
                    self.assertEqual(command[command.index(flag)+1], expected)
                self.assertIn('--record-rgb', command)
                specs = json.loads((cwd / command[command.index('--episode-specs')+1]).read_text())
                self.assertEqual(specs, [spec] * n)
                target = cwd / command[command.index('--output')+1]
                self.assertFalse(target.exists())
                target.mkdir()
                log_path.write_text('appearance failed' if n == 8 else 'completed')
                write_json(gpu_path, {'samples': [{'memory_used_mib': 321}]})
                if n == 8:
                    write_json(target / 'batch.json', dict(status='error', num_envs=n,
                        appearance_warmup={'passed': False, 'error': 'Appearance did not converge'}))
                else:
                    write_json(target / 'batch.json', dict(status='completed', num_envs=n))
                    (target / 'env_000').mkdir()
                    write_json(target / 'env_000/result.json', dict(success=True, steps=586))
                now[0] += 4.
                return dict(exit_code=1 if n == 8 else 0, elapsed_seconds=4.,
                            whole_gpu_peak_sampled_memory_mib=321)

            def validate(directory, report_path, log_path, *, workers, cwd):
                order.append(('validate', 1));self.assertEqual(workers, 4)
                write_json(report_path, dict(status='completed', results=[
                    {'path': str(directory / 'env_000'), 'passed': True}]))
                now[0] += 2.
                return dict(exit_code=0, elapsed_seconds=2.)

            result = run_sweep(source, root / 'sweep', [8, 1], root=root,
                collector_runner=launch, validation_runner=validate, clock=lambda: now[0])
            self.assertEqual(order, [('collect', 8), ('collect', 1), ('validate', 1)])
            failed, success = result['runs']
            self.assertEqual(failed['failure']['kind'], 'appearance_warmup_failed')
            self.assertIsNone(failed['validated_successes_per_hour'])
            self.assertEqual(success['total_wall_s'], 6.)
            self.assertEqual(success['launch_to_shutdown_s'], 4.)
            self.assertEqual(success['validation_wall_s'], 2.)
            self.assertEqual(success['validated_successes_per_hour'], 600.)
            self.assertEqual(result['status'], 'completed_with_failures')
            self.assertEqual(result['best_measured_num_envs'], 1)
            self.assertEqual(json.loads(source.read_text()), spec)
            self.assertFalse(result['production_quota_eligible'])
            self.assertFalse(result['fixed_appearance_benchmark'])
            self.assertEqual(result['device'], 'cuda:0')
            self.assertIsNone(result['camera_device'])
            self.assertIsNone(result['physics_threads'])
            self.assertFalse(result['defer_clearance_diagnostics'])
            self.assertEqual(result['appearance_mode'], 'randomized')
            self.assertTrue((root / 'sweep/envs_08/collector.log').is_file())

    def test_success_requires_physical_result_and_rgb_validation_both_true(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);source = root / 'spec.json';now = [0.]
            write_json(source, {'obstacle_id': 'x', 'layout': {}})

            def launch(command, log_path, gpu_path, *, cwd):
                self.assertEqual(command.count('--fixed-appearance-benchmark'), 1)
                self.assertEqual(command.count('--defer-clearance-diagnostics'), 1)
                for flag in ('--background-rgb', '--reuse-phase-planner', '--curobo-cuda-graph'):
                    self.assertEqual(command.count(flag), 1)
                for flag, expected in [('--warmup-mode', 'temporal_mean'),
                                       ('--warmup-window-frames', '32'), ('--warmup-max-frames', '768'),
                                       ('--device', 'cpu'), ('--camera-device', 'cuda:0'),
                                       ('--physics-threads', '0')]:
                    self.assertEqual(command[command.index(flag)+1], expected)
                target = cwd / command[command.index('--output')+1];target.mkdir()
                write_json(target / 'batch.json', dict(status='completed', num_envs=4))
                for i, success in enumerate([True, True, False, True]):
                    directory = target / f'env_{i:03d}';directory.mkdir()
                    write_json(directory / 'result.json', dict(success=success, steps=600))
                now[0] += 4.
                return dict(exit_code=0, elapsed_seconds=4.)

            def validate(directory, report_path, log_path, *, workers, cwd):
                write_json(report_path, dict(status='completed', results=[
                    {'path': str(directory / f'env_{i:03d}'), 'passed': ok}
                    for i, ok in enumerate([True, False, True, True])]))
                now[0] += 2.
                return dict(exit_code=1, elapsed_seconds=2.)

            result = run_sweep(source, root / 'sweep', [4], root=root,
                collector_runner=launch, validation_runner=validate, clock=lambda: now[0],
                warmup_mode='temporal_mean', warmup_window_frames=32, warmup_max_frames=768,
                fixed_appearance_benchmark=True, device='cpu', camera_device='cuda:0',
                physics_threads=0, defer_clearance_diagnostics=True,
                background_rgb=True, reuse_phase_planner=True, curobo_cuda_graph=True)
            row = result['runs'][0]
            self.assertEqual(row['collector_successes'], 3)
            self.assertEqual(row['validated_successes'], 2)
            self.assertEqual(row['validated_successes_per_hour'], 1200.)
            self.assertEqual(row['validated_success_fraction_of_attempts'], .5)
            self.assertEqual(row['status'], 'completed')
            self.assertEqual(result['warmup_window_frames'], 32)
            self.assertEqual(result['warmup_max_frames'], 768)
            self.assertEqual(result['appearance_mode'], 'authored_fixed_benchmark')
            self.assertEqual(row['appearance_mode'], 'authored_fixed_benchmark')
            self.assertIn('cannot fill randomized production quota', result['appearance_scope'])
            self.assertFalse(result['production_quota_eligible'])
            self.assertEqual(result['device'], 'cpu')
            self.assertEqual(result['camera_device'], 'cuda:0')
            self.assertEqual(result['physics_threads'], 0)
            self.assertTrue(result['defer_clearance_diagnostics'])
            saved = json.loads((root / 'sweep/summary.json').read_text())
            for key in ('device', 'camera_device', 'physics_threads', 'defer_clearance_diagnostics',
                        'background_rgb', 'reuse_phase_planner', 'curobo_cuda_graph'):
                self.assertEqual(saved[key], result[key])

    def test_cli_runtime_defaults_and_cpu_with_gpu_camera_override(self):
        base = ['sweep', '--counts', '6', '--episode-spec', 'spec.json', '--output', 'fresh_sweep']
        for extra, expected in [([], dict(device='cuda:0', camera_device=None,
                                         physics_threads=None, defer_clearance_diagnostics=False)),
                (['--device', 'cpu', '--camera-device', 'cuda:0', '--physics-threads', '0',
                  '--defer-clearance-diagnostics'], dict(device='cpu', camera_device='cuda:0',
                    physics_threads=0, defer_clearance_diagnostics=True))]:
            with self.subTest(extra=extra), patch('sys.argv', base + extra), patch(
                    'scripts.run_vectorized_collection_sweep.run_sweep',
                    return_value={'status': 'completed'}) as runner:
                self.assertEqual(main(), 0)
                for key, value in expected.items():
                    self.assertEqual(runner.call_args.kwargs[key], value)

    def test_oom_is_classified_only_from_actual_failure_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'collector.log'
            log.write_text('RuntimeError: CUDA error: out of memory\n')
            report = classify_run_failure({'status': 'error'}, {'exit_code': 1}, log)
            self.assertEqual(report['kind'], 'out_of_memory')
            self.assertIn('CUDA', report['evidence'])
            log.write_text('High memory utilization reported; process error 1\n')
            report = classify_run_failure({'status': 'error'}, {'exit_code': 1}, log)
            self.assertEqual(report['kind'], 'collector_failed')

    def test_fixed_preparation_is_explicit_and_pixel_diagnostic_is_not_failure(self):
        with patch('sys.argv', ['sweep', '--counts', '2', '--episode-spec', 'spec.json',
                               '--output', 'fresh_sweep', '--warmup-mode', 'fixed_preparation']), patch(
                'scripts.run_vectorized_collection_sweep.run_sweep', return_value={'status': 'completed'}) as runner:
            self.assertEqual(main(), 0)
            self.assertEqual(runner.call_args.kwargs['warmup_mode'], 'fixed_preparation')
            self.assertFalse(runner.call_args.kwargs['fixed_appearance_benchmark'])
        with tempfile.TemporaryDirectory() as temporary:
            log = Path(temporary) / 'collector.log'
            log.write_text('')
            warmup = dict(mode='fixed_preparation', passed=False, preparation_passed=True,
                          error='Appearance did not converge')
            failure = classify_run_failure({'appearance_warmup': warmup}, {'exit_code': 1}, log)
            self.assertEqual(failure['kind'], 'collector_failed')
            warmup.update(preparation_passed=False, preparation_error='Physics advanced')
            failure = classify_run_failure({'appearance_warmup': warmup}, {'exit_code': 1}, log)
            self.assertEqual(failure, {'kind': 'appearance_preparation_failed', 'evidence': 'Physics advanced'})

    def test_completed_report_with_caught_cuda_oom_is_failed_but_keeps_partial_successes(self):
        # Same failure shape as the measured 12-env camera readback OOM:
        # collector catches it, writes batch.status=completed and exits zero.
        actual_error = ('torch.OutOfMemoryError: CUDA out of memory. Tried to allocate 32.00 MiB. '
                        'GPU 0 has a total capacity of 31.36 GiB of which 23.12 MiB is free.')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);source = root / 'spec.json';now = [0.]
            write_json(source, {'obstacle_id': 'x', 'layout': {}})

            def launch(command, log_path, gpu_path, *, cwd):
                target = cwd / command[command.index('--output')+1];target.mkdir()
                write_json(target / 'batch.json', dict(status='completed', num_envs=2,
                                                       execution_error=actual_error))
                for index, success in enumerate([True, False]):
                    directory = target / f'env_{index:03d}';directory.mkdir()
                    write_json(directory / 'result.json', dict(success=success, steps=30))
                log_path.write_text('Finished writing partial records\n')
                now[0] += 4.
                return dict(exit_code=0, elapsed_seconds=4.)

            def validate(directory, report_path, log_path, *, workers, cwd):
                write_json(report_path, dict(status='completed', results=[
                    {'path': str(directory / f'env_{i:03d}'), 'passed': True} for i in range(2)]))
                now[0] += 2.
                return dict(exit_code=0, elapsed_seconds=2.)

            result = run_sweep(source, root / 'sweep', [2], root=root,
                collector_runner=launch, validation_runner=validate, clock=lambda: now[0])
            row = result['runs'][0]
            self.assertEqual(row['collector_status'], 'completed')
            self.assertEqual(row['status'], 'failed')
            self.assertEqual(row['failure']['kind'], 'out_of_memory')
            self.assertEqual(row['collector_successes'], 1)
            self.assertEqual(row['validated_successes'], 1)
            self.assertIsNone(row['validated_successes_per_hour'])
            self.assertEqual(row['total_wall_s'], 6.)
            self.assertIsNone(result['best_measured_num_envs'])
            candidate = classify_run_failure({'status': 'completed'}, {'exit_code': 0},
                root / 'absent.log', [actual_error])
            self.assertEqual(candidate['kind'], 'out_of_memory')


if __name__ == '__main__':
    unittest.main()
