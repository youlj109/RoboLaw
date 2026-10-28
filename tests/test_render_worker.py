"""Real stdlib child processes exercise the protocol without Kit or CUDA."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from kitchen_safety.render_worker import RenderWorker

ROOT = Path(__file__).resolve().parents[1]
FAKE_SERVER = '''
import json, sys
from pathlib import Path
sys.path.insert(0, sys.argv[2])
from scripts.render_recorded_batch import atomic_json, serve_requests
directory = Path(sys.argv[1])
last = [None]
def execute(item):
    last[0] = Path(item['source_episode']).name
    if last[0] in ('bad', 'fatal'):
        raise ValueError('episode failed')
    output = Path(item['output'])
    output.mkdir()
    atomic_json(output / 'render_report.json', {'completed': True})
    return {'completed': True, 'frames': 3}
def cleanup():
    if last[0] == 'fatal':
        raise RuntimeError('scene cleanup failed')
    (directory / 'recovered').write_text('closed failed scene')
atomic_json(directory / 'ready.json', {'ready': True})
try:
    serve_requests(directory, execute, cleanup)
finally:
    (directory / 'actually_closed').write_text('true')
'''


class RenderWorkerTest(unittest.TestCase):
    def worker(self, root, directory=None):
        script = root / 'fake_server.py'
        script.write_text(FAKE_SERVER)
        directory = directory if directory is not None else root / 'worker'
        return RenderWorker(cwd=root, directory=directory,
            command=[sys.executable, str(script), str(directory), str(ROOT)])

    def test_external_unicode_dataset_request_and_overwrite_protection(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            root, external = base/'workspace', base/'USB 新加卷'
            root.mkdir(); external.mkdir()
            source = external/'source'
            source.mkdir()
            (source/'trajectory.h5').write_bytes(b'unchanged physical data')
            worker = self.worker(root, external/'worker')
            try:
                result = worker.execute(source, source/'rendered')
                self.assertEqual(result['status'], 'completed')
                request = json.loads((worker.directory/'requests/00000.json').read_text())
                self.assertEqual(request['source_episode'], str(source.resolve()))
                self.assertEqual(request['output'], str((source/'rendered').resolve()))
                self.assertEqual(worker._relative(root/'local_output'), 'local_output')
                self.assertEqual((source/'trajectory.h5').read_bytes(), b'unchanged physical data')
                second = worker.execute(source, source/'rendered')
                self.assertIn('FileExistsError', second['error'])
                self.assertFalse((worker.directory/'requests/00001.json').exists())
            finally:
                lifecycle = worker.close()
            self.assertEqual(lifecycle['exit_code'], 0)
            self.assertTrue(lifecycle['process_reaped'])

    def test_default_command_keeps_external_request_and_report_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            root = base/'workspace'
            root.mkdir()
            worker = RenderWorker(cwd=root, directory=base/'USB 新加卷'/'worker')
            child = Mock(pid=123)
            child.stdin = Mock(closed=True)
            child.wait.return_value = 0
            with patch('kitchen_safety.render_worker.subprocess.Popen', return_value=child) as spawn:
                try:
                    worker._start()
                    command = spawn.call_args.args[0]
                    self.assertEqual(command[command.index('--request-dir')+1], str(worker.directory))
                    self.assertEqual(command[command.index('--report')+1], str(worker.directory/'report.json'))
                finally:
                    worker.close()

    def test_lazy_start_ordinary_failure_recovery_and_real_exit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            worker = self.worker(root)
            self.assertFalse(worker.directory.exists())
            for name in ('first', 'bad', 'last'):
                (root / name).mkdir()
            try:
                replies = [worker.execute(root / name, root / ('rgb_' + name))
                           for name in ('first', 'bad', 'last')]
                self.assertEqual([r['status'] for r in replies], ['completed', 'failed', 'completed'])
                self.assertEqual([r['id'] for r in replies], ['00000', '00001', '00002'])
                self.assertTrue(all('exit_code' not in r for r in replies))
                self.assertTrue(all('request_wall_s' in r for r in replies))
                self.assertTrue((worker.directory / 'recovered').exists())
                self.assertIsNone(worker.process.poll())
            finally:
                lifecycle = worker.close()
            self.assertEqual(lifecycle['exit_code'], 0)
            self.assertTrue(lifecycle['process_reaped'])
            self.assertTrue((worker.directory / 'actually_closed').exists())
            self.assertGreaterEqual(lifecycle['lifetime_s'], lifecycle['shutdown_wait_s'])
            self.assertEqual(worker.close(), lifecycle)

    def test_new_rendered_child_is_allowed_without_overwriting_source_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root/'source'
            source.mkdir()
            (source/'trajectory.h5').write_bytes(b'unchanged physical data')
            worker = self.worker(root)
            try:
                result = worker.execute(source, source/'rendered')
                self.assertEqual(result['status'], 'completed')
                self.assertTrue((source/'rendered/render_report.json').is_file())
                self.assertEqual((source/'trajectory.h5').read_bytes(), b'unchanged physical data')
                second = worker.execute(source, source/'rendered')
                self.assertTrue(second['worker_unavailable'])
                self.assertIn('FileExistsError', second['error'])
            finally:
                worker.close()

    def test_cleanup_failure_replies_unavailable_and_is_reaped(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'fatal').mkdir()
            worker = self.worker(root)
            try:
                reply = worker.execute(root / 'fatal', root / 'rgb')
                self.assertEqual(reply['status'], 'worker_unavailable')
                self.assertIn('scene cleanup failed', reply['cleanup_error'])
                another = worker.execute(root / 'fatal', root / 'rgb2')
                self.assertEqual(another['status'], 'worker_unavailable')
                self.assertFalse((worker.directory / 'requests/00001.json').exists())
            finally:
                lifecycle = worker.close()
            self.assertNotEqual(lifecycle['exit_code'], 0)
            self.assertTrue(lifecycle['process_reaped'])

    def test_spawn_failure_does_not_retry_or_claim_process_exit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'source').mkdir()
            worker = RenderWorker(cwd=root, directory=root / 'worker', command=['/missing-renderer'])
            reply = worker.execute(root / 'source', root / 'rgb')
            self.assertEqual(reply['status'], 'worker_unavailable')
            self.assertNotIn('exit_code', reply)
            lifecycle = worker.close()
            self.assertFalse(lifecycle['started'])
            self.assertNotIn('exit_code', lifecycle)
            self.assertIsNone(worker.process)

    def test_repeated_interrupts_wait_for_actual_child_before_reraise(self):
        with tempfile.TemporaryDirectory() as tmp:
            worker = RenderWorker(cwd=tmp, directory=Path(tmp) / 'worker')
            child = Mock(pid=123)
            child.stdin = Mock(closed=False)
            child.stdin.close.side_effect = lambda: setattr(child.stdin, 'closed', True)
            child.wait.side_effect = [KeyboardInterrupt(), KeyboardInterrupt(), 0]
            worker.process = child
            worker.lifecycle['started'] = True
            with self.assertRaises(KeyboardInterrupt):
                worker.close()
            self.assertEqual(child.wait.call_count, 3)
            self.assertTrue(worker.report()['process_reaped'])
            self.assertEqual(worker.report()['exit_code'], 0)
            child.kill.assert_not_called()
            child.terminate.assert_not_called()


if __name__ == '__main__':
    unittest.main()
