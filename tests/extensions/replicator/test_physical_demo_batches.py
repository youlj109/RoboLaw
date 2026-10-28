"""CPU-only coordinator tests; mocked subprocesses never launch Kit or Docker."""
from contextlib import redirect_stdout
import argparse
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.extensions.replicator import run_physical_demo_batches as batch


class PhysicalBatchTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        patcher = patch.object(batch, 'ROOT', self.root)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.source = self.root / 'reports/source'
        self.output = self.root / 'exports/demos'
        self.output.mkdir(parents=True)
        self.manifest_path = self.output / 'manifest.json'
        self.rows = [self.row('done', 'a', 'failed'), self.row('a1', 'a'), self.row('a2', 'a'),
                     self.row('b1', 'b'), self.row('b2', 'b')]
        self.save(self.rows)

    @staticmethod
    def row(sid, task, status='pending'):
        return dict(scene_id=sid, task_id=task, execution_status=status,
                    attempts=0 if status == 'pending' else 1, executed=status != 'pending',
                    config_sha256='frozen-' + sid)

    def save(self, rows):
        self.manifest_path.write_text(json.dumps(dict(kind='physical_execution', source_run='reports/source',
            source_progress_sha256='frozen-source', entries=rows)))

    def launch(self, command, **kwargs):
        self.assertEqual(command[:3], ['bash', str(self.root / 'scripts/arena_python.sh'),
            'scripts/extensions/replicator/execute_planned_trajectories.py'])
        self.assertIn('--resume', command)
        self.assertNotIn('--allow-runtime-revision', command)
        self.assertEqual(command[command.index('--device') + 1], 'cpu')
        self.assertEqual(command[command.index('--physics-threads') + 1], '8')
        self.assertTrue(kwargs['start_new_session'])
        self.assertEqual(kwargs['cwd'], self.root)
        def wait():
            current = json.loads(self.manifest_path.read_text())['entries']
            selected = [r for r in current if r['execution_status'] == 'pending'][:int(command[-1])]
            self.assertEqual(len({r['task_id'] for r in selected}), 1)
            for row in selected:
                row.update(execution_status='failed', attempts=1, executed=True)
            self.save(current)
            return 0
        return SimpleNamespace(wait=wait, pid=12345)

    def test_fresh_child_per_task_only_pending_and_logs_are_separate(self):
        before = self.rows[0].copy()
        with patch.object(batch.subprocess, 'Popen', side_effect=self.launch) as popen:
            self.assertEqual(batch.run_batches(self.source, self.output), 2)
        self.assertEqual(popen.call_count, 2)
        self.assertEqual(len(list((self.output / 'batch_logs').glob('*.log'))), 2)
        self.assertEqual(json.loads(self.manifest_path.read_text())['entries'][0], before)

    def test_running_or_previously_attempted_pending_never_launches(self):
        for status, attempts, executed in [('running', 1, False), ('pending', 1, False), ('pending', 0, True)]:
            with self.subTest(status=status, attempts=attempts, executed=executed):
                self.rows[1].update(execution_status=status, attempts=attempts, executed=executed)
                self.save(self.rows)
                original = self.manifest_path.read_bytes()
                with patch.object(batch.subprocess, 'Popen') as popen, self.assertRaises(ValueError):
                    batch.run_batches(self.source, self.output)
                popen.assert_not_called()
                self.assertEqual(self.manifest_path.read_bytes(), original)

    def test_child_nonzero_stops_without_retry(self):
        with patch.object(batch.subprocess, 'Popen', return_value=SimpleNamespace(wait=lambda: 7)) as popen:
            with self.assertRaisesRegex(RuntimeError, 'exited 7'):
                batch.run_batches(self.source, self.output)
        self.assertEqual(popen.call_count, 1)

    def test_zero_exit_with_pending_or_running_is_not_completion(self):
        for status in ['pending', 'running']:
            with self.subTest(status=status):
                self.save(self.rows)
                def wait():
                    rows = json.loads(self.manifest_path.read_text())['entries']
                    rows[1]['execution_status'] = status
                    self.save(rows)
                    return 0
                with patch.object(batch.subprocess, 'Popen', return_value=SimpleNamespace(wait=wait)) as popen:
                    with self.assertRaises(ValueError):
                        batch.run_batches(self.source, self.output)
                self.assertEqual(popen.call_count, 1)

    def test_unselected_mutation_stops_after_first_child(self):
        def launch(command, **kwargs):
            child = self.launch(command, **kwargs)
            def wait():
                child.wait()
                rows = json.loads(self.manifest_path.read_text())['entries']
                rows[0]['config_sha256'] = 'changed'
                self.save(rows)
                return 0
            return SimpleNamespace(wait=wait)
        with patch.object(batch.subprocess, 'Popen', side_effect=launch) as popen:
            with self.assertRaisesRegex(ValueError, 'Unselected scene changed'):
                batch.run_batches(self.source, self.output)
        self.assertEqual(popen.call_count, 1)

    def test_dry_run_is_read_only_and_reproduces_driver_prefix(self):
        original = self.manifest_path.read_bytes()
        with patch.object(batch.subprocess, 'Popen') as popen, redirect_stdout(io.StringIO()) as output:
            batch.run_batches(self.source, self.output, dry_run=True)
        plans = json.loads(output.getvalue())['batches']
        self.assertEqual([p['scenes'] for p in plans], [['a1', 'a2'], ['b1', 'b2']])
        self.assertEqual(self.manifest_path.read_bytes(), original)
        self.assertEqual(list(self.output.iterdir()), [self.manifest_path])
        popen.assert_not_called()

    def test_missing_manifest_is_never_created(self):
        self.manifest_path.unlink()
        with patch.object(batch.subprocess, 'Popen') as popen, self.assertRaises(FileNotFoundError):
            batch.run_batches(self.source, self.output)
        self.assertFalse(self.manifest_path.exists())
        popen.assert_not_called()

    def test_keyboard_interrupt_does_not_kill_child(self):
        with patch.object(batch.subprocess, 'Popen') as popen:
            popen.return_value.wait.side_effect = KeyboardInterrupt
            popen.return_value.pid = 12345
            with self.assertRaises(KeyboardInterrupt):
                batch.run_batches(self.source, self.output)
            popen.return_value.kill.assert_not_called()
            popen.return_value.terminate.assert_not_called()


class ArenaParserCompatibilityTests(unittest.TestCase):
    """Run in the Arena Python environment; imports/parser only, never Kit.

    Host environments without IsaacLab skip this integration check. Run it with
    scripts/arena_python.sh -m unittest discover -s tests/extensions/replicator
    -p test_physical_demo_batches.py to exercise the actual pinned Arena parser.
    """

    def parse_generated_command(self, extra=()):
        try:
            from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
        except ModuleNotFoundError as error:
            self.skipTest('Requires the Arena Python environment: ' + str(error))
        self.assertTrue(callable(get_isaaclab_arena_cli_parser))
        from scripts.extensions.replicator import execute_planned_trajectories as driver
        command = batch.driver_command(batch.ROOT / batch.DEFAULT_SOURCE,
                                       batch.ROOT / batch.DEFAULT_OUTPUT, 11)
        original_parse = argparse.ArgumentParser.parse_args
        captured = []

        class ParsedOnly(Exception):
            pass

        def parse_and_stop(parser, args=None, namespace=None):
            captured.append(original_parse(parser, args, namespace))
            # Stop immediately after the real parser validates every argument,
            # before source validation, manifest access, or app construction.
            raise ParsedOnly

        with patch('sys.argv', [command[2], *command[3:], *extra]), \
                patch.object(argparse.ArgumentParser, 'parse_args', autospec=True, side_effect=parse_and_stop), \
                patch.object(driver, 'source_entries', side_effect=AssertionError('Parser test must not read results')):
            with self.assertRaises(ParsedOnly):
                driver.main()
        return captured[0]

    def test_generated_batch_command_is_accepted_by_actual_driver_parser(self):
        parsed = self.parse_generated_command()
        self.assertEqual(parsed.device, 'cpu')
        self.assertEqual(parsed.physics_threads, 8)
        self.assertEqual(parsed.limit, 11)
        self.assertTrue(parsed.resume)
        self.assertFalse(parsed.allow_runtime_revision)
        self.assertEqual(str(parsed.source_run), batch.DEFAULT_SOURCE)
        self.assertEqual(str(parsed.output), batch.DEFAULT_OUTPUT)


if __name__ == '__main__':
    unittest.main()
