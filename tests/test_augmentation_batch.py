"""CPU tests for resumable collection counts and eligibility (no simulator)."""
import argparse
import errno
import json
from pathlib import Path
import sys
import subprocess
import tempfile
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import collect_obstacle_library as batch
from scripts import run_nonhuman_augmentation as orchestrator


class AugmentationBatchTests(unittest.TestCase):
    def test_runtime_defaults_preserve_original_configuration_and_explicit_defaults(self):
        parser = argparse.ArgumentParser()
        batch.add_runtime_arguments(parser)
        args = parser.parse_args([])
        args.seed, args.video, args.record_rgb = 42, False, True
        original = batch.configuration_for(args, {}, {'objects': {'xy_m': .02}})
        self.assertNotIn('runtime', original)
        self.assertEqual(batch.runtime_command_args(batch.runtime_configuration(args)), [])
        defaults = parser.parse_args(['--device', 'cuda:0', '--execution-checks', 'online',
                                      '--camera-owner', 'environment', '--episode-length-s', '120'])
        defaults.seed, defaults.video, defaults.record_rgb = 42, False, True
        current = batch.configuration_for(defaults, {}, {'objects': {'xy_m': .02}})
        self.assertEqual(current, original)
        batch.check_resume_configuration({'configuration': original, 'episodes_per_id': 2}, current, 20, True)

    def test_batch_forwards_runtime_and_requires_same_settings_on_resume(self):
        entry = {'id': 'property_tall_glass', 'category': 'property'}
        flags = ['--device', 'cpu', '--physics-threads', '0', '--torch-threads', '1',
                 '--execution-checks', 'deferred', '--camera-owner', 'writer', '--cpu-rgb-copy',
                 '--camera-device', 'cuda:0', '--reuse-planner', '--background-rgb',
                 '--episode-length-s', '180']
        expected = {'device': 'cpu', 'physics_threads': 0, 'torch_threads': 1,
                    'execution_checks': 'deferred', 'camera_owner': 'writer', 'cpu_rgb_copy': True,
                    'camera_device': 'cuda:0', 'reuse_planner': True, 'background_rgb': True,
                    'episode_length_s': 180.0}
        commands = []

        def run(command, log):
            commands.append(command)
            return {'exit_code': 0, 'elapsed_seconds': 1}

        def verify(destination, entry, seed, video, execution, **kwargs):
            record = {'obstacle_id': entry['id'], 'passed': True, 'video_requested': False,
                      'rgb_requested': True, 'execution_success': True, 'data_success': True,
                      'audit_pass': True, 'augmentation_eligible': True, 'frames': 3,
                      'failure_phase': None, 'failure_kind': None, 'elapsed_seconds': 1}
            batch.write_json(destination / 'verification.json', record)
            return record

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(batch, 'load_assets', return_value=[entry]), \
                patch.object(batch, 'run_command', side_effect=run), \
                patch.object(batch, 'verify_attempt', side_effect=verify):
            common = ['--output', directory, '--ids', entry['id'], '--record-rgb']
            self.assertEqual(batch.main(common + flags), 0)
            report = batch.read_json(Path(directory) / 'manifest.json')
            self.assertEqual(report['configuration']['runtime'], expected)
            self.assertEqual(report['configuration']['collector_overrides'], {})
            command = commands[0]
            for name, value in expected.items():
                option = '--' + name.replace('_', '-')
                self.assertEqual(command.count(option), 1)
                if not isinstance(value, bool):
                    self.assertEqual(command[command.index(option) + 1], str(value))
            self.assertEqual(batch.main(common + flags + ['--resume']), 0)
            for option, value in [('--torch-threads', '2'), ('--camera-device', 'cpu'),
                                  ('--episode-length-s', '120')]:
                changed = list(flags)
                changed[changed.index(option) + 1] = value
                with self.assertRaisesRegex(ValueError, 'configuration differs'):
                    batch.main(common + changed + ['--resume'])
            with self.assertRaisesRegex(ValueError, 'configuration differs'):
                batch.main(common + [flag for flag in flags if flag != '--reuse-planner'] + ['--resume'])
            with self.assertRaisesRegex(ValueError, 'configuration differs'):
                batch.main(common + ['--resume'])  # Defaults must not silently replace saved runtime settings.
            self.assertEqual(len(commands), 1)

    def test_episode_timeout_rejects_invalid_values_before_launch(self):
        parser = argparse.ArgumentParser()
        batch.add_runtime_arguments(parser)
        for value in ('0', '-1', 'nan', 'inf'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'finite and positive'):
                batch.runtime_configuration(parser.parse_args(['--episode-length-s', value]))

    def test_orchestrator_forwards_runtime_to_all_stages_and_checks_resume(self):
        fake_randomization = types.ModuleType('kitchen_safety.randomization')
        fake_randomization.load_randomization_config = lambda path: {'objects': {'xy_m': .02}}
        flags = ['--device', 'cpu', '--physics-threads', '0', '--torch-threads', '1',
                 '--execution-checks', 'deferred', '--camera-owner', 'writer', '--cpu-rgb-copy',
                 '--camera-device', 'cuda:0', '--reuse-planner', '--persistent-kit']
        commands = []

        def run(command, log):
            commands.append(command)
            directory = Path(command[command.index('--output') + 1])
            directory.mkdir(parents=True, exist_ok=True)
            target = int(command[command.index('--episodes-per-id') + 1])
            batch.write_json(directory / 'manifest.json', {'all_passed': True,
                'episodes_per_id': target, 'statistics': {}})
            return {'exit_code': 0, 'elapsed_seconds': 1}

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(orchestrator, 'run_command', side_effect=run), \
                patch.dict(sys.modules, {'kitchen_safety.randomization': fake_randomization}):
            common = ['--output', directory, '--ids', 'property_tall_glass']
            self.assertEqual(orchestrator.main(common + flags), 0)
            self.assertEqual(len(commands), 3)
            for command in commands:
                self.assertIn('--cpu-rgb-copy', command)
                self.assertIn('--reuse-planner', command)
                self.assertIn('--persistent-kit', command)
                self.assertEqual(command[command.index('--camera-device') + 1], 'cuda:0')
                for option, value in zip(flags[:10:2], flags[1:10:2]):
                    self.assertEqual(command[command.index(option) + 1], value)
            report = batch.read_json(Path(directory) / 'augmentation.json')
            self.assertEqual(report['configuration']['runtime']['physics_threads'], 0)
            self.assertEqual(report['configuration']['runtime']['device'], 'cpu')
            self.assertTrue(report['configuration']['runtime']['persistent_kit'])
            self.assertEqual(orchestrator.main(common + flags + ['--resume']), 0)
            self.assertEqual(len(commands), 4)  # Only production is revisited.
            changed = list(flags)
            changed[changed.index('--camera-owner') + 1] = 'environment'
            with self.assertRaises(SystemExit) as raised:
                orchestrator.main(common + changed + ['--resume'])
            self.assertEqual(raised.exception.code, 2)
            self.assertEqual(len(commands), 4)

    def test_persistent_worker_routes_only_collection_and_closes_on_all_exits(self):
        entry = {'id': 'property_tall_glass', 'category': 'property'}
        for outcome in ('success', 'storage_error', 'worker_unavailable', 'validation_exception'):
            with self.subTest(outcome=outcome), tempfile.TemporaryDirectory() as directory:
                events, workers = [], []

                class Worker:
                    def __init__(self, *, cwd):
                        self.cwd = cwd
                        self.closed = False
                        workers.append(self)

                    def __enter__(self):
                        return self

                    def __exit__(self, *exc):
                        self.closed = True
                        events.append('close')

                    def execute(self, command, log_path):
                        self_name = Path(command[1]).name
                        if self_name != 'collect_kitchen.py' or '--persistent-kit' in command:
                            raise AssertionError(f'Wrong worker command: {command}')
                        events.append('collection')
                        log_path.write_text('preserved collection log\n')
                        destination = Path(command[command.index('--output') + 1])
                        if outcome == 'storage_error':
                            return {'exit_code': None, 'elapsed_seconds': 1,
                                    'storage_error': 'OSError: [Errno 28] No space left on device'}
                        if outcome == 'worker_unavailable' and len(workers) == 1:
                            return {'exit_code': None, 'elapsed_seconds': 1,
                                    'worker_unavailable': True, 'error': 'Kit worker exited unexpectedly'}
                        batch.write_json(destination / 'result.json', {'success': True, 'steps': 3})
                        (destination / 'images.h5').touch()
                        batch.write_json(destination / 'cameras.json', {})
                        return {'exit_code': 0, 'elapsed_seconds': 1, 'worker_pid': 123}

                def cpu_execute(command, log_path):
                    script = Path(command[1]).name
                    self.assertIn(script, ('validate_collection.py', 'audit_collection_clearance.py'))
                    events.append(script)
                    if outcome == 'validation_exception':
                        raise RuntimeError('validation child could not be launched')
                    report = Path(command[command.index('--report') + 1])
                    batch.write_json(report, [{'passed': True}] if script == 'validate_collection.py'
                                     else {'passed': True})
                    return {'exit_code': 0, 'elapsed_seconds': .1}

                module = types.ModuleType('kitchen_safety.collector_worker')
                module.CollectorWorker = Worker
                common = ['--output', directory, '--ids', entry['id'], '--record-rgb',
                          '--episodes-per-id', '2', '--max-attempts',
                          '3' if outcome == 'worker_unavailable' else '2']
                with patch.dict(sys.modules, {'kitchen_safety.collector_worker': module}), \
                        patch.object(batch, 'load_assets', return_value=[entry]), \
                        patch.object(batch, 'run_command', side_effect=cpu_execute):
                    if outcome == 'validation_exception':
                        with self.assertRaisesRegex(RuntimeError, 'validation child'):
                            batch.main(common + ['--persistent-kit'])
                    else:
                        self.assertEqual(batch.main(common + ['--persistent-kit']),
                                         {'storage_error': batch.STORAGE_ERROR_EXIT_CODE,
                                          'worker_unavailable': batch.WORKER_UNAVAILABLE_EXIT_CODE}.get(outcome, 0))
                    self.assertEqual(len(workers), 1)
                    self.assertEqual(workers[0].cwd, ROOT)
                    self.assertTrue(workers[0].closed)
                    self.assertEqual(events[-1], 'close')
                    self.assertEqual(events.count('collection'), 2 if outcome == 'success' else 1)
                    logs = list(Path(directory).glob('*/attempt_*/run.log'))
                    self.assertEqual(len(logs), events.count('collection'))
                    if outcome == 'success':
                        report = batch.read_json(Path(directory) / 'manifest.json')
                        self.assertTrue(report['configuration']['runtime']['persistent_kit'])
                        self.assertEqual(events.count('validate_collection.py'), 2)
                        self.assertEqual(events.count('audit_collection_clearance.py'), 2)
                        self.assertEqual(batch.main(common + ['--persistent-kit', '--resume']), 0)
                        self.assertTrue(workers[-1].closed)
                        self.assertEqual(events.count('collection'), 2)
                        with self.assertRaisesRegex(ValueError, 'configuration differs'):
                            batch.main(common + ['--resume'])
                    elif outcome == 'storage_error':
                        self.assertEqual(events, ['collection', 'close'])
                    elif outcome == 'worker_unavailable':
                        self.assertEqual(events, ['collection', 'close'])
                        report = batch.read_json(Path(directory) / 'manifest.json')
                        self.assertFalse(report['complete'])
                        self.assertEqual(report['stop_reason'], 'worker_unavailable')
                        self.assertEqual(report['worker_unavailable'], 'Kit worker exited unexpectedly')
                        self.assertEqual(len(report['attempts']), 1)
                        first = Path(directory) / report['attempts'][0]['path']
                        preserved = (first / 'execution.json').read_bytes()
                        self.assertEqual(batch.main(common + ['--persistent-kit', '--resume']), 0)
                        self.assertEqual(len(workers), 2)
                        self.assertTrue(workers[-1].closed)
                        report = batch.read_json(Path(directory) / 'manifest.json')
                        self.assertTrue(report['complete'])
                        self.assertNotIn('worker_unavailable', report)
                        self.assertNotIn('stop_reason', report)
                        self.assertEqual(len(report['attempts']), 3)
                        self.assertEqual(report['attempts'][0]['failure_kind'], 'worker_unavailable')
                        self.assertEqual((first / 'execution.json').read_bytes(), preserved)

    def test_actual_storage_errors_are_distinct_from_normal_subprocess_failures(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            for number, symbol in ((errno.ENOSPC, 'ENOSPC'), (errno.EDQUOT, 'EDQUOT')):
                with self.subTest(error=symbol):
                    execution = batch.run_command([sys.executable, '-c',
                        f'import os; raise OSError({number}, os.strerror({number}))'], output / f'{symbol}.log')
                    self.assertNotEqual(execution['exit_code'], 0)
                    self.assertTrue(execution['storage_error'])
            execution = batch.run_command([sys.executable, '-c',
                'raise RuntimeError("planning failed: no collision-free path")'], output / 'ordinary.log')
            self.assertNotEqual(execution['exit_code'], 0)
            self.assertNotIn('storage_error', execution)
            execution = batch.run_command([sys.executable, '-c', 'raise SystemExit(74)'], output / 'exit74.log')
            self.assertNotIn('storage_error', execution)  # Only our batch defines 74 as storage failure.
            with patch.object(batch.subprocess, 'run', side_effect=OSError(errno.EDQUOT, 'Disk quota exceeded')):
                execution = batch.run_command(['unused'], output / 'spawn.log')
            self.assertIn('storage_error', execution)

    def test_storage_failure_stops_batch_and_resume_preserves_physics_and_logs(self):
        """Real CPU child processes fail writes to /dev/full; no Kit is started."""
        entry = {'id': 'property_tall_glass', 'category': 'property'}
        for failing_stage in ('collection', 'validation', 'audit', 'preview'):
            with self.subTest(stage=failing_stage), tempfile.TemporaryDirectory() as directory:
                fixture = Path(directory)
                scripts = fixture / 'scripts'
                scripts.mkdir()
                output = fixture / 'output'
                flag = fixture / 'storage_failure'
                flag.write_text(failing_stage)
                calls = fixture / 'calls'
                prefix = (
                    'import json,pathlib,sys\n'
                    f'flag=pathlib.Path({str(flag)!r})\n'
                    f'calls=pathlib.Path({str(calls)!r})\n'
                    'stage=STAGE\n'
                    'with calls.open("a") as stream: stream.write(stage+"\\n")\n'
                    'def maybe_fail():\n'
                    '    if flag.exists() and flag.read_text()==stage:\n'
                    '        with open("/dev/full", "wb", buffering=0) as full: full.write(b"x")\n')
                (scripts / 'collect_kitchen.py').write_text(prefix.replace('STAGE', repr('collection')) +
                    'out=pathlib.Path(sys.argv[sys.argv.index("--output")+1])\n'
                    '(out/"trajectory.h5").write_bytes(b"original physical trajectory")\n'
                    'maybe_fail()\n'
                    '(out/"result.json").write_text(json.dumps({"success":True,"steps":3}))\n'
                    '(out/"images.h5").write_bytes(b"original RGB")\n'
                    '(out/"cameras.json").write_text("{}")\n')
                for name, filename, report in (
                        ('validation', 'validate_collection.py', [{'passed': True}]),
                        ('audit', 'audit_collection_clearance.py', {'passed': True, 'episodes': [{}]}),
                        ('preview', 'export_collection_previews.py', [{'passed': True}])):
                    (scripts / filename).write_text(prefix.replace('STAGE', repr(name)) +
                        'out=pathlib.Path(sys.argv[sys.argv.index("--report")+1])\n'
                        'out.write_text("partial output retained")\n'
                        'maybe_fail()\n'
                        f'out.write_text(json.dumps({report!r}))\n')
                arguments = ['--output', str(output), '--ids', entry['id'],
                             '--max-attempts', '3', '--record-rgb']
                if failing_stage == 'preview':
                    arguments += ['--preview-per-id', '1']
                with patch.object(batch, 'ROOT', fixture), patch.object(batch, 'load_assets', return_value=[entry]):
                    self.assertEqual(batch.main(arguments), batch.STORAGE_ERROR_EXIT_CODE)
                    stopped = batch.read_json(output / 'manifest.json')
                    self.assertFalse(stopped['complete'])
                    self.assertFalse(stopped['all_passed'])
                    self.assertEqual(stopped['stop_reason'], 'storage_error')
                    self.assertEqual(len(stopped['attempts']), 1)
                    expected_calls = ['collection', 'validation', 'audit', 'preview']
                    self.assertEqual(calls.read_text().splitlines(),
                                     expected_calls[:expected_calls.index(failing_stage) + 1])
                    destination = output / entry['id'] / 'attempt_0000_seed_42'
                    trajectory = (destination / 'trajectory.h5').read_bytes()
                    original_log = (output / 'preview_export_000.log' if failing_stage == 'preview'
                                    else destination / ('run.log' if failing_stage == 'collection'
                                                        else f'{failing_stage}.log'))
                    saved_log = original_log.read_bytes()
                    if failing_stage != 'preview':
                        self.assertEqual(stopped['attempts'][0]['failure_kind'], 'storage_error')
                    flag.unlink()
                    self.assertEqual(batch.main(arguments + ['--resume']), 0)
                finished = batch.read_json(output / 'manifest.json')
                self.assertTrue(finished['complete'])
                self.assertNotIn('storage_error', finished)
                self.assertEqual(finished['statistics']['success_count'], 1)
                self.assertEqual(finished['statistics']['attempt_count'], 2 if failing_stage == 'collection' else 1)
                self.assertEqual((destination / 'trajectory.h5').read_bytes(), trajectory)
                self.assertEqual(calls.read_text().splitlines().count('collection'),
                                 2 if failing_stage == 'collection' else 1)
                if failing_stage in ('validation', 'audit'):
                    self.assertEqual((destination / f'{failing_stage}.interrupted_000.log').read_bytes(), saved_log)
                    self.assertTrue((destination / f'{failing_stage}_execution.interrupted_000.json').exists())
                    self.assertEqual(batch.read_json(destination / 'verification.interrupted_000.json')['failure_kind'],
                                     'storage_error')
                    self.assertIsNone(finished['attempts'][0]['storage_error'])
                else:
                    self.assertEqual(original_log.read_bytes(), saved_log)

    def test_metadata_storage_error_returns_distinct_exit_without_launching_children(self):
        entry = {'id': 'property_tall_glass', 'category': 'property'}
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(batch, 'load_assets', return_value=[entry]), \
                patch.object(batch, 'write_json', side_effect=OSError(errno.ENOSPC, 'No space left on device')), \
                patch.object(batch, 'run_command') as child:
            self.assertEqual(batch.main(['--output', directory, '--ids', entry['id']]),
                             batch.STORAGE_ERROR_EXIT_CODE)
            child.assert_not_called()
            with patch.object(batch.sys.stderr, 'write', side_effect=OSError(errno.ENOSPC, 'No space left on device')):
                self.assertEqual(batch.main(['--output', directory, '--ids', entry['id'], '--resume']),
                                 batch.STORAGE_ERROR_EXIT_CODE)
            child.assert_not_called()

    def test_orchestrator_stops_storage_round_and_resumes_without_repeating_regression(self):
        fake_randomization = types.ModuleType('kitchen_safety.randomization')
        fake_randomization.load_randomization_config = lambda path: {'objects': {'xy_m': .02}}
        commands = []
        fail = [True]

        def run(command, log):
            commands.append(command)
            log.write_text('mock batch\n')
            directory = Path(command[command.index('--output') + 1])
            directory.mkdir(parents=True, exist_ok=True)
            target = int(command[command.index('--episodes-per-id') + 1])
            storage_error = fail[0] and target == 2
            batch.write_json(directory / 'manifest.json', {'all_passed': not storage_error,
                'episodes_per_id': target, 'statistics': {},
                'assets': [{'obstacle_id': 'property_tall_glass', 'attempts': 1}]})
            return {'exit_code': batch.STORAGE_ERROR_EXIT_CODE if storage_error else 0, 'elapsed_seconds': 1}

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(orchestrator, 'run_command', side_effect=run), \
                patch.dict(sys.modules, {'kitchen_safety.randomization': fake_randomization}):
            args = ['--output', directory, '--ids', 'property_tall_glass']
            self.assertEqual(orchestrator.main(args), batch.STORAGE_ERROR_EXIT_CODE)
            self.assertEqual(len(commands), 2)
            report = batch.read_json(Path(directory) / 'augmentation.json')
            self.assertEqual(len(report['pilot_rounds']), 1)
            self.assertEqual(report['stop_reason'], 'storage_error')
            self.assertFalse(report['complete'])
            self.assertNotIn('production', report['stages'])
            fail[0] = False
            self.assertEqual(orchestrator.main(args + ['--resume']), 0)
            self.assertEqual(len(commands), 4)
            self.assertTrue(batch.read_json(Path(directory) / 'augmentation.json')['complete'])
            self.assertNotIn('storage_error', batch.read_json(Path(directory) / 'augmentation.json'))

    def test_orchestrator_stops_unavailable_worker_until_explicit_resume(self):
        fake_randomization = types.ModuleType('kitchen_safety.randomization')
        fake_randomization.load_randomization_config = lambda path: {'objects': {'xy_m': .02}}
        commands, fail = [], [True]

        def run(command, log):
            commands.append(command)
            log.write_text('mock batch\n')
            directory = Path(command[command.index('--output') + 1])
            directory.mkdir(parents=True, exist_ok=True)
            target = int(command[command.index('--episodes-per-id') + 1])
            unavailable = fail[0] and target == 2
            manifest = {'all_passed': not unavailable, 'episodes_per_id': target,
                        'statistics': {}, 'assets': [{'obstacle_id': 'property_tall_glass', 'attempts': 1}]}
            if unavailable:
                manifest['worker_unavailable'] = 'Kit exited during collection'
            batch.write_json(directory / 'manifest.json', manifest)
            return {'exit_code': batch.WORKER_UNAVAILABLE_EXIT_CODE if unavailable else 0,
                    'elapsed_seconds': 1}

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(orchestrator, 'run_command', side_effect=run), \
                patch.dict(sys.modules, {'kitchen_safety.randomization': fake_randomization}):
            args = ['--output', directory, '--ids', 'property_tall_glass', '--persistent-kit']
            self.assertEqual(orchestrator.main(args), batch.WORKER_UNAVAILABLE_EXIT_CODE)
            report = batch.read_json(Path(directory) / 'augmentation.json')
            self.assertEqual(len(commands), 2)
            self.assertEqual(len(report['pilot_rounds']), 1)
            self.assertFalse(report['complete'])
            self.assertEqual(report['worker_unavailable'], 'Kit exited during collection')
            self.assertEqual(report['stop_reason'], 'worker_unavailable')
            self.assertNotIn('production', report['stages'])
            fail[0] = False
            self.assertEqual(orchestrator.main(args + ['--resume']), 0)
            report = batch.read_json(Path(directory) / 'augmentation.json')
            self.assertEqual(len(commands), 4)
            self.assertTrue(report['complete'])
            self.assertNotIn('worker_unavailable', report)
            self.assertNotIn('stop_reason', report)

    def test_resume_configuration_allows_growth_but_not_changed_sampling(self):
        config = {'seed': 42, 'randomization': {'objects': {'xy_m': .02}}, 'record_rgb': True}
        previous = {'configuration': config, 'episodes_per_id': 2}
        batch.check_resume_configuration(previous, config, 20, True)
        with self.assertRaisesRegex(ValueError, 'configuration differs'):
            batch.check_resume_configuration(previous, dict(config, seed=43), 20, True)
        with self.assertRaisesRegex(ValueError, 'only stay the same or increase'):
            batch.check_resume_configuration(previous, config, 1, True)
        with self.assertRaisesRegex(ValueError, 'older batch'):
            batch.check_resume_configuration({'attempts': []}, config, 20, True)
        batch.check_resume_configuration({'attempts': []}, config, 1, False)

    def test_attempt_indices_never_reuse_interrupted_or_manual_runs(self):
        paths = [Path('attempt_0000_seed_42'), Path('attempt_0004_seed_42')]
        self.assertEqual(batch.next_attempt_index(paths, [{'attempt': 8}]), 9)
        self.assertEqual(batch.next_attempt_index(paths, []), 5)
        with self.assertRaises(ValueError):
            batch.next_attempt_index([Path('attempt_broken')], [])

    def test_success_without_obstruction_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            batch.write_json(destination / 'result.json', {
                'success': True, 'steps': 4, 'augmentation_eligible': False,
                'obstruction_probe': {'eligible': False}, 'timings_seconds': {'physics_s': 5}})
            batch.write_json(destination / 'validation.json', [{'passed': True}])
            batch.write_json(destination / 'audit.json', {'passed': True, 'episodes': [{}]})
            for name in ('validation', 'audit'):
                batch.write_json(destination / (name + '_execution.json'), {'exit_code': 0})
            record = batch.verify_attempt(destination, {'id': 'x', 'category': 'robot'},
                                          42, False, {'exit_code': 0}, randomized=True)
            self.assertTrue(record['execution_success'])
            self.assertTrue(record['data_success'])
            self.assertTrue(record['audit_pass'])
            self.assertFalse(record['passed'])
            self.assertEqual(record['failure_kind'], 'obstruction_unconfirmed')

    def test_steel_side_risk_exception_preserves_evidence_and_all_other_checks(self):
        cases = [
            ('robot_steel_clamp', True, True, True, True, None),
            ('property_tall_glass', True, True, True, False, 'obstruction_unconfirmed'),
            ('robot_steel_clamp', False, True, True, False, 'physical_failed'),
            ('robot_steel_clamp', True, False, True, False, 'data_validation_failed'),
            ('robot_steel_clamp', True, True, False, False, 'clearance_audit_failed'),
        ]
        for identifier, physical, data, safety, passed, kind in cases:
            with self.subTest(identifier=identifier, physical=physical, data=data, safety=safety), \
                    tempfile.TemporaryDirectory() as directory:
                destination = Path(directory)
                result = dict(success=physical, steps=4, augmentation_eligible=False,
                              obstruction_probe={'eligible': False, 'reason': 'direct path clear'},
                              failure_kind='obstruction_unconfirmed' if physical else 'physical_failed')
                batch.write_json(destination / 'result.json', result)
                original = (destination / 'result.json').read_bytes()
                batch.write_json(destination / 'validation.json', [{'passed': data}])
                batch.write_json(destination / 'audit.json', {'passed': safety, 'episodes': [{}]})
                for name, succeeded in [('validation', data), ('audit', safety)]:
                    batch.write_json(destination / f'{name}_execution.json',
                                     {'exit_code': 0 if succeeded else 1})
                record = batch.verify_attempt(destination, {'id': identifier, 'category': 'robot'},
                                              42, False, {'exit_code': 0}, randomized=True)
                self.assertEqual(record['passed'], passed)
                self.assertEqual(record['failure_kind'], kind)
                self.assertFalse(record['augmentation_eligible'])
                self.assertEqual(record['obstruction_probe'], result['obstruction_probe'])
                self.assertEqual((destination / 'result.json').read_bytes(), original)
                if identifier == 'robot_steel_clamp':
                    self.assertTrue(record['effective_augmentation_eligible'])
                    self.assertEqual(record['eligibility_reason'], 'user-approved-side-risk')
                    self.assertFalse(record['direct_path_required'])
                else:
                    self.assertNotIn('eligibility_reason', record)
        self.assertEqual(batch.augmentation_eligibility(
            'robot_steel_clamp', {'augmentation_eligible': False}, randomized=False), (True, None))

    def test_rgb_validation_requires_rgb_not_preview_video(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            batch.write_json(destination / 'result.json', {'success': True, 'steps': 2})
            (destination / 'images.h5').touch()
            (destination / 'cameras.json').write_text('{}')
            commands = []

            def run(command, log):
                commands.append(command)
                report = Path(command[command.index('--report') + 1])
                batch.write_json(report, [{'passed': True}] if 'validate_collection.py' in command[1]
                                 else {'passed': True, 'episodes': [{}]})
                return {'exit_code': 0, 'elapsed_seconds': .1}

            with patch.object(batch, 'run_command', run):
                record = batch.verify_attempt(destination, {'id': 'x', 'category': 'robot'},
                                              42, False, {'exit_code': 0}, record_rgb=True)
            self.assertTrue(record['passed'])
            self.assertIn('--require-rgb', commands[0])
            self.assertNotIn('--require-previews', commands[0])
            self.assertFalse((destination / 'all_cameras.mp4').exists())

    def test_data_and_audit_failures_take_priority_over_eligibility(self):
        result = {'success': True, 'steps': 3, 'failure_kind': 'obstruction_unconfirmed'}
        self.assertEqual(batch.failure_kind(result, True, False, False, False), 'data_validation_failed')
        self.assertEqual(batch.failure_kind(result, True, True, False, False), 'clearance_audit_failed')
        self.assertEqual(batch.failure_kind(result, True, True, True, False), 'obstruction_unconfirmed')
        for old, current in [('planning_failure', 'planning_failed'), ('physical_failure', 'physical_failed'),
                             ('startup_failure', 'startup_failed')]:
            self.assertEqual(batch.failure_kind({'failure_kind': old}, False, False, False), current)
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory)
            batch.write_json(destination / 'result.json', dict(result, augmentation_eligible=False))
            batch.write_json(destination / 'validation.json', [{'passed': False}])
            batch.write_json(destination / 'audit.json', {'passed': False, 'episodes': [{}]})
            for name in ('validation', 'audit'):
                batch.write_json(destination / f'{name}_execution.json', {'exit_code': 1})
            record = batch.verify_attempt(destination, {'id': 'x', 'category': 'robot'},
                                          42, False, {'exit_code': 0}, randomized=True)
            self.assertEqual(record['failure_kind'], 'data_validation_failed')
            self.assertEqual(len(record['failure_reasons']), 3)

    def test_resume_after_a_real_verification_process_is_terminated(self):
        entry = {'id': 'property_tall_glass', 'category': 'property'}
        for interrupted in ('validation', 'audit'):
            with self.subTest(check=interrupted), tempfile.TemporaryDirectory() as directory:
                fixture = Path(directory)
                output = fixture / 'output'
                destination = output / entry['id'] / 'attempt_0000_seed_42'
                destination.mkdir(parents=True)
                scripts = fixture / 'scripts'
                scripts.mkdir()
                batch.write_json(destination / 'result.json', {'success': True, 'steps': 3})
                batch.write_json(destination / 'execution.json', {'exit_code': 0, 'elapsed_seconds': 1})
                (destination / 'trajectory.h5').write_bytes(b'physical trajectory remains untouched')
                completed = 'audit' if interrupted == 'validation' else 'validation'
                reports = {'validation': [{'passed': True}], 'audit': {'passed': True, 'episodes': [{}]}}
                batch.write_json(destination / f'{completed}.json', reports[completed])
                batch.write_json(destination / f'{completed}_execution.json', {'exit_code': 0})
                (destination / f'{completed}.log').write_text('completed original check\n')
                (destination / f'{interrupted}.interrupted_000.log').write_text('older interruption\n')
                # Start and terminate an actual CPU subprocess while its report
                # is incomplete. No execution checkpoint or verification exists.
                child_code = ('import pathlib,sys,time; '
                              'print("partial verification output",flush=True); '
                              'pathlib.Path(sys.argv[1]).write_text("partial report"); time.sleep(30)')
                partial_report = destination / f'{interrupted}.json'
                with (destination / f'{interrupted}.log').open('w') as log:
                    child = subprocess.Popen([sys.executable, '-c', child_code, str(partial_report)],
                                             stdout=log, stderr=subprocess.STDOUT)
                    try:
                        deadline = time.monotonic() + 5
                        while not partial_report.exists() and time.monotonic() < deadline:
                            time.sleep(.01)
                        self.assertTrue(partial_report.exists())
                    finally:
                        child.terminate()
                        child.wait(timeout=5)
                self.assertNotEqual(child.returncode, 0)
                script_name = 'validate_collection.py' if interrupted == 'validation' else 'audit_collection_clearance.py'
                (scripts / script_name).write_text(
                    'import json,pathlib,sys\n'
                    'out=pathlib.Path(sys.argv[sys.argv.index("--report")+1])\n'
                    f'out.write_text(json.dumps({reports[interrupted]!r}))\n'
                    'print("finished resumed check")\n')
                # The other checker and collect_kitchen.py deliberately do not
                # exist: invoking either would fail instead of silently passing.
                with patch.object(batch, 'ROOT', fixture), patch.object(batch, 'load_assets', return_value=[entry]):
                    code = batch.main(['--output', str(output), '--ids', entry['id'],
                                       '--max-attempts', '1', '--resume'])
                self.assertEqual(code, 0)
                self.assertTrue(batch.read_json(destination / 'verification.json')['passed'])
                self.assertEqual((destination / f'{interrupted}.interrupted_001.log').read_text(),
                                 'partial verification output\n')
                self.assertEqual((destination / f'{interrupted}.interrupted_001.json').read_text(), 'partial report')
                self.assertEqual((destination / f'{interrupted}.interrupted_000.log').read_text(), 'older interruption\n')
                self.assertEqual((destination / f'{completed}.log').read_text(), 'completed original check\n')
                self.assertEqual((destination / 'trajectory.h5').read_bytes(), b'physical trajectory remains untouched')
                self.assertEqual(len(list((output / entry['id']).glob('attempt_*'))), 1)

    def test_full_batch_resume_counts_successes_not_attempts(self):
        entry = {'id': 'property_tall_glass', 'category': 'property'}
        fake_randomization = types.ModuleType('kitchen_safety.randomization')
        fake_randomization.SUPPORTED_IDS = (entry['id'],)
        fake_randomization.load_randomization_config = lambda path: {'objects': {'xy_m': .02}}
        fake_randomization.sample_episode = lambda identifier, config, seed, attempt: {
            'id': identifier, 'seeds': {'geometry': seed + 100 + attempt,
                                      'appearance': seed + 200 + attempt, 'planner': seed + 300 + attempt},
            'layout': {'mustard_position': [2 + attempt / 1000, -.4, 1.]}, 'appearance': {}}
        commands = []

        def run(command, log):
            commands.append(command)
            log.write_text('mock collector\n')
            return {'exit_code': 0, 'elapsed_seconds': 1}

        def verify(destination, current_entry, seed, video, execution, **kwargs):
            attempt = int(destination.name.split('_')[1])
            accepted = attempt > 0
            record = {'obstacle_id': current_entry['id'], 'passed': accepted,
                      'video_requested': video, 'rgb_requested': True,
                      'execution_success': True, 'data_success': True, 'audit_pass': True,
                      'augmentation_eligible': accepted, 'frames': 3,
                      'failure_phase': None if accepted else 'above_bowl',
                      'failure_kind': None if accepted else 'obstruction_unconfirmed',
                      'checks': {'validation': {'elapsed_seconds': .1}, 'audit': {'elapsed_seconds': .2}},
                      'elapsed_seconds': 1, 'timings_seconds': {'camera': {'render_capture_s': .2, 'frames': 3}}}
            batch.write_json(destination / 'verification.json', record)
            return record

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(batch, 'load_assets', return_value=[entry]), \
                patch.object(batch, 'run_command', side_effect=run), \
                patch.object(batch, 'verify_attempt', side_effect=verify), \
                patch.dict(sys.modules, {'kitchen_safety.randomization': fake_randomization}):
            common = ['--ids', entry['id'], '--output', directory, '--record-rgb',
                      '--randomization-config', 'fake.json', '--seed', '42']
            self.assertEqual(batch.main(common + ['--episodes-per-id', '2', '--max-attempts', '3']), 0)
            first = batch.read_json(Path(directory) / 'manifest.json')
            self.assertEqual(first['assets'][0]['success_count'], 2)
            self.assertEqual(first['assets'][0]['attempts'], 3)
            initial_specs = {path: path.read_bytes() for path in Path(directory).glob('*/attempt_*/episode_spec.json')}
            self.assertEqual(len(commands), 3)
            self.assertTrue(all('--episode-spec' in command and '--record-rgb' in command for command in commands))
            self.assertEqual(batch.main(common + ['--episodes-per-id', '3', '--max-attempts', '4', '--resume']), 0)
            final = batch.read_json(Path(directory) / 'manifest.json')
            self.assertEqual(final['statistics']['success_count'], 3)
            self.assertEqual(final['statistics']['attempt_count'], 4)
            self.assertEqual(len(commands), 4)
            self.assertEqual(final['statistics']['timings_seconds'], {
                'camera.render_capture_s': .8, 'validation_s': .4, 'clearance_audit_s': .8})
            self.assertEqual(final['statistics']['elapsed_attempt_seconds'], 4)
            self.assertEqual(final['statistics']['failure_phase_counts'], {'above_bowl': 1})
            self.assertEqual(final['assets'][0]['failure_phase_counts'], {'above_bowl': 1})
            for path, data in initial_specs.items():
                self.assertEqual(path.read_bytes(), data)
            self.assertEqual(batch.main(common + ['--episodes-per-id', '3', '--max-attempts', '4', '--resume']), 0)
            self.assertEqual(len(commands), 4)

    def test_deficit_is_not_collection_complete(self):
        outcomes = [{'path': 'a', 'passed': True}, {'path': 'b', 'passed': False, 'failure_kind': 'planning_failed'}]
        summary = batch.asset_summary('x', {'category': 'robot'}, outcomes, 20)
        self.assertFalse(summary['complete'])
        self.assertEqual(summary['deficit'], 19)
        self.assertEqual(summary['accepted_paths'], ['a'])

    def test_all_attempt_sampling_distribution_exposes_rejection_bias_without_measured_values(self):
        records = [
            {'path': 'accepted', 'passed': True,
             'sampled_layout': {'mustard_position': [0.0, 0.01, 1.0], 'mustard_yaw_deg': 0.0},
             'appearance': {'intensity_multiplier': 0.8},
             'appearance_applied': {'measured_intensity': 800}, 'settled_poses': {'bottle': [0.001]}},
            {'path': 'rejected', 'passed': False, 'failure_kind': 'planning_failed',
             'sampled_layout': {'mustard_position': [0.02, -0.01, 1.0], 'mustard_yaw_deg': 10.0},
             'appearance': {'intensity_multiplier': 1.3},
             'appearance_applied': {'measured_intensity': 1300}, 'settled_poses': {'bottle': [0.021]}}]
        per_id = batch.asset_summary('x', {'category': 'robot'}, records, 2)
        with tempfile.TemporaryDirectory() as directory:
            whole_batch = batch.batch_statistics(records, [per_id], Path(directory))
        for summary in (per_id, whole_batch):
            accepted = summary['accepted_parameter_statistics']
            sampled = summary['sampled_parameter_statistics']
            self.assertEqual(accepted['mustard_position[0]'], {'count': 1, 'min': 0, 'max': 0, 'mean': 0})
            self.assertEqual(sampled['mustard_position[0]'], {'count': 2, 'min': 0, 'max': .02, 'mean': .01})
            self.assertEqual(sampled['mustard_yaw_deg']['max'], 10)
            self.assertEqual(sampled['appearance.intensity_multiplier']['count'], 2)
            self.assertEqual(sampled['appearance.intensity_multiplier']['max'], 1.3)
            self.assertIn('appearance_applied.measured_intensity', accepted)
            self.assertIn('settled_poses.bottle[0]', accepted)
            self.assertFalse(any(key.startswith(('appearance_applied.', 'settled_poses.')) for key in sampled))

    def test_subset_resume_exports_previews_for_all_accumulated_ids(self):
        entries = [{'id': name, 'category': 'property'} for name in
                   ('property_tall_glass', 'property_wine_glass')]
        exported = []

        def verify(destination, entry, seed, video, execution, **kwargs):
            record = {'obstacle_id': entry['id'], 'passed': True, 'video_requested': False,
                      'rgb_requested': True, 'execution_success': True, 'data_success': True,
                      'audit_pass': True, 'augmentation_eligible': True, 'frames': 3,
                      'failure_phase': None, 'failure_kind': None, 'elapsed_seconds': 1}
            batch.write_json(destination / 'verification.json', record)
            return record

        def previews(output, assets, count):
            exported.append((count, [asset['obstacle_id'] for asset in assets]))
            return {'requested_per_id': count, 'passed': True, 'episodes': []}

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(batch, 'load_assets', return_value=entries), \
                patch.object(batch, 'run_command', return_value={'exit_code': 0, 'elapsed_seconds': 1}), \
                patch.object(batch, 'verify_attempt', side_effect=verify), \
                patch.object(batch, 'export_previews', side_effect=previews):
            common = ['--output', directory, '--record-rgb', '--max-attempts', '1']
            self.assertEqual(batch.main(common + ['--ids', *[entry['id'] for entry in entries]]), 0)
            self.assertEqual(batch.main(common + ['--ids', entries[0]['id'], '--resume', '--preview-per-id', '1']), 0)
            self.assertEqual(exported[-1], (1, [entry['id'] for entry in entries]))

    def test_revalidation_during_batch_refreshes_counts_and_preview_selection(self):
        entries = [{'id': name, 'category': 'property'} for name in
                   ('property_tall_glass', 'property_wine_glass')]
        exported = []
        commands = []
        changed = [False]

        def run(command, log):
            commands.append(command)
            log.write_text('mock collection\n')
            if not changed[0] and command[command.index('--obstacle_id') + 1] == entries[1]['id']:
                # Revalidate A while the batch is executing B. Its historical
                # success must disappear from the live report and preview list.
                first = log.parent.parent.parent / entries[0]['id'] / 'attempt_0000_seed_42/verification.json'
                record = batch.read_json(first)
                record.update(passed=False, data_success=False, failure_kind='data_validation_failed')
                batch.write_json(first, record)
                changed[0] = True
            return {'exit_code': 0, 'elapsed_seconds': 1}

        def verify(destination, entry, seed, video, execution, **kwargs):
            record = {'obstacle_id': entry['id'], 'passed': True, 'video_requested': False,
                      'rgb_requested': True, 'execution_success': True, 'data_success': True,
                      'audit_pass': True, 'augmentation_eligible': True, 'frames': 3,
                      'failure_phase': None, 'failure_kind': None, 'elapsed_seconds': 1}
            batch.write_json(destination / 'verification.json', record)
            return record

        def previews(output, assets, count):
            paths = [path for asset in assets for path in asset['accepted_paths'][:count]]
            exported.append(paths)
            return {'requested_per_id': count, 'passed': True, 'episodes': paths}

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(batch, 'load_assets', return_value=entries), \
                patch.object(batch, 'run_command', side_effect=run), \
                patch.object(batch, 'verify_attempt', side_effect=verify), \
                patch.object(batch, 'export_previews', side_effect=previews):
            common = ['--output', directory, '--record-rgb', '--max-attempts', '2',
                      '--preview-per-id', '1', '--ids', *[entry['id'] for entry in entries]]
            self.assertEqual(batch.main(common), 1)
            report = batch.read_json(Path(directory) / 'manifest.json')
            self.assertEqual(report['statistics']['success_count'], 1)
            self.assertFalse(report['complete'])
            self.assertEqual(exported[-1], ['property_wine_glass/attempt_0000_seed_42'])
            self.assertEqual(batch.main(common + ['--resume']), 0)
            self.assertEqual(len(commands), 3)
            self.assertEqual(exported[-1], ['property_tall_glass/attempt_0001_seed_43',
                                           'property_wine_glass/attempt_0000_seed_42'])

    def test_preview_target_cannot_exceed_success_target(self):
        with self.assertRaises(SystemExit) as raised:
            batch.main(['--record-rgb', '--episodes-per-id', '1', '--preview-per-id', '2'])
        self.assertEqual(raised.exception.code, 2)

    def test_orchestrator_shares_pilot_production_output(self):
        args = argparse.Namespace(ids=['property_tall_glass'], seed=42, max_attempts=100,
            randomization_config='configs/augmentation_nonhuman10.json', pilot_per_id=2,
            episodes_per_id=20, preview_per_id=3)
        stages = orchestrator.build_stages(args, Path('/tmp/augmentation'))
        self.assertEqual(stages[1]['manifest'], stages[2]['manifest'])
        self.assertNotEqual(stages[0]['manifest'], stages[1]['manifest'])
        self.assertIn('--resume', stages[2]['command'])
        self.assertNotIn('--preview-per-id', stages[1]['command'])

    def test_orchestrator_stops_after_failed_fixed_regression(self):
        fake_randomization = types.ModuleType('kitchen_safety.randomization')
        fake_randomization.load_randomization_config = lambda path: {'objects': {'xy_m': .02}}
        commands = []

        def run(command, log):
            commands.append(command)
            directory = Path(command[command.index('--output') + 1])
            directory.mkdir(parents=True)
            batch.write_json(directory / 'manifest.json', {'all_passed': False,
                'statistics': {'success_count': 0, 'target_count': 1, 'deficit': 1}})
            return {'exit_code': 1, 'elapsed_seconds': 1}

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(orchestrator, 'run_command', side_effect=run), \
                patch.dict(sys.modules, {'kitchen_safety.randomization': fake_randomization}):
            code = orchestrator.main(['--output', directory, '--ids', 'property_tall_glass'])
            self.assertEqual(code, 1)
            self.assertEqual(len(commands), 1)
            report = batch.read_json(Path(directory) / 'augmentation.json')
            self.assertFalse(report['complete'])
            self.assertIn('regression failed', report['stop_reason'])
            self.assertFalse((Path(directory) / 'episodes').exists())

    def test_pilot_uses_cumulative_rounds_before_full_production(self):
        fake_randomization = types.ModuleType('kitchen_safety.randomization')
        fake_randomization.load_randomization_config = lambda path: {'objects': {'xy_m': .02}}
        commands = []

        def run(command, log):
            commands.append(command)
            log.write_text('mock batch\n')
            directory = Path(command[command.index('--output') + 1])
            directory.mkdir(parents=True, exist_ok=True)
            target = int(command[command.index('--episodes-per-id') + 1])
            budget = int(command[command.index('--max-attempts') + 1])
            # One difficult ID lacks its quota in round two; the other ID is
            # already done and is represented as unchanged in the next round.
            passed = target != 2 or budget >= 4
            batch.write_json(directory / 'manifest.json', {'all_passed': passed,
                'episodes_per_id': target,
                'assets': [{'obstacle_id': 'property_tall_glass', 'attempts': min(budget, 4)},
                           {'obstacle_id': 'property_wine_glass', 'attempts': min(target, 2)}],
                'statistics': {'success_count': target * 2 if passed else 3,
                               'target_count': target * 2, 'deficit': 0 if passed else 1}})
            return {'exit_code': 0 if passed else 1, 'elapsed_seconds': 1}

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(orchestrator, 'run_command', side_effect=run), \
                patch.dict(sys.modules, {'kitchen_safety.randomization': fake_randomization}):
            self.assertEqual(orchestrator.main(['--output', directory, '--ids',
                'property_tall_glass', 'property_wine_glass']), 0)
            pilot = [command for command in commands
                     if command[command.index('--episodes-per-id') + 1] == '2']
            self.assertEqual([command[command.index('--max-attempts') + 1] for command in pilot], ['2', '4'])
            self.assertNotIn('--resume', pilot[0])
            self.assertIn('--resume', pilot[1])
            self.assertTrue(all('property_tall_glass' in command and 'property_wine_glass' in command
                                for command in pilot))
            self.assertEqual(commands[-1][commands[-1].index('--max-attempts') + 1], '100')
            self.assertEqual(commands[-1][commands[-1].index('--episodes-per-id') + 1], '20')
            report = batch.read_json(Path(directory) / 'augmentation.json')
            self.assertEqual([item['cumulative_attempt_budget_per_id'] for item in report['pilot_rounds']], [2, 4])
            self.assertEqual(orchestrator.pilot_attempt_budgets(3, 10, {'assets': [{'attempts': 5}]}), [6, 9, 10])


if __name__ == '__main__':
    unittest.main()
