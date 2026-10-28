"""Real lightweight subprocesses exercise worker protocol, EOF and failures."""
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from kitchen_safety.collector_worker import CollectorWorker, STARTUP_COMPLETE_MARKER


WORKER = r'''
import json, os, sys, time
for line in sys.stdin:
    request = json.loads(line)
    mode = request['argv'][0] if request['argv'] else 'ok'
    if mode == 'die':
        print('worker dying', flush=True)
        raise SystemExit(7)
    if mode == 'stall':
        time.sleep(10)
    if mode == 'child_pipe':
        import subprocess
        subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(10)'])
        raise SystemExit(9)
    print('native log for ' + request['id'], flush=True)
    if mode == 'storage':
        print('OSError: [Errno 28] No space left on device', flush=True)
    identifier = 'incorrect' if mode == 'wrong_id' else request['id']
    response = {'id': identifier, 'exit_code': 1 if mode in ('fail','storage') else 0,
                'worker_pid': os.getpid(), 'result_path': 'result.json', 'elapsed_seconds': .01}
    print('KITCHEN_COLLECTOR_REPLY ' + json.dumps(response), flush=True)
print('EOF closes app', flush=True)
'''


class CollectorWorkerTests(unittest.TestCase):
    def worker(self, root, **kwargs):
        return CollectorWorker(cwd=root, worker_command=[sys.executable, '-u', '-c', WORKER], **kwargs)

    def command(self, mode='ok'):
        return [sys.executable, 'scripts/collect_kitchen.py', mode]

    def test_lazy_one_process_replies_separated_from_logs_and_eof_shutdown(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.worker(root) as worker:
                self.assertIsNone(worker.process)
                first = worker.execute(self.command(), root / 'first.log')
                failure = worker.execute(self.command('fail'), root / 'failed.log')
                final = worker.execute(self.command(), root / 'final.log')
                self.assertEqual([first['exit_code'], failure['exit_code'], final['exit_code']], [0, 1, 0])
                self.assertEqual(first['worker_pid'], final['worker_pid'])
                self.assertEqual(first['result_path'], 'result.json')
                self.assertIn('started_at', first)
                self.assertGreater(first['elapsed_seconds'], 0)
                self.assertEqual((root / 'first.log').read_text(), 'native log for 1\n')
                self.assertNotIn('KITCHEN_COLLECTOR_REPLY', (root / 'failed.log').read_text())
            self.assertEqual(worker.shutdown['exit_code'], 0)
            self.assertFalse(worker.shutdown['forced'])
            self.assertFalse(worker._reader.is_alive())

    def test_worker_death_and_inherited_stdout_do_not_wait_for_eof(self):
        for mode in ['die', 'child_pipe']:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                begin = time.monotonic()
                with self.worker(directory, request_timeout_s=5., shutdown_timeout_s=.1) as worker:
                    result = worker.execute(self.command(mode), Path(directory) / 'run.log')
                    self.assertIsNone(result['exit_code'])
                    self.assertIn('without a reply', result['error'])
                self.assertLess(time.monotonic() - begin, 2.)
                self.assertFalse(worker._reader.is_alive())

    def test_timeout_bad_reply_and_storage_failure_are_reported(self):
        for mode, expected in [('stall', 'TimeoutError'), ('wrong_id', 'ID does not match')]:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                with self.worker(directory, request_timeout_s=.2, shutdown_timeout_s=.1) as worker:
                    result = worker.execute(self.command(mode), Path(directory) / 'run.log')
                    self.assertIsNone(result['exit_code'])
                    self.assertIn(expected, result['error'])
        with tempfile.TemporaryDirectory() as directory:
            with self.worker(directory) as worker:
                result = worker.execute(self.command('storage'), Path(directory) / 'run.log')
                self.assertEqual(result['exit_code'], 1)
                self.assertIn('No space left', result['storage_error'])

    def test_invalid_command_existing_log_and_unused_context_never_start(self):
        with tempfile.TemporaryDirectory() as directory, patch('subprocess.Popen') as start:
            root = Path(directory)
            with self.worker(root) as worker:
                result = worker.execute(['python', 'unrelated.py'], root / 'invalid.log')
                self.assertIsNone(result['exit_code'])
                (root / 'existing.log').write_text('previous data')
                result = worker.execute(self.command(), root / 'existing.log')
                self.assertIsNone(result['exit_code'])
                self.assertEqual((root / 'existing.log').read_text(), 'previous data')
            start.assert_not_called()
            self.assertEqual(worker.shutdown, {'started': False})

    def test_two_workers_release_startup_lock_at_marker_before_episode_finishes(self):
        class ObservedLock:
            def __init__(self):
                self.lock = threading.Lock()
                self.attempts = 0
                self.second_waiting = threading.Event()
            def acquire(self, **kwargs):
                self.attempts += 1
                if self.attempts == 2:
                    self.second_waiting.set()
                return self.lock.acquire(**kwargs)
            def release(self):
                self.lock.release()
        source = r'''
import json, os, sys, time
from pathlib import Path
root = Path(sys.argv[1])
for line in sys.stdin:
    request = json.loads(line)
    mode = request['argv'][0]
    (root / (mode + '_started')).write_text('started')
    if mode == 'first':
        while not (root / 'allow_marker').exists(): time.sleep(.005)
    print('[isaaclab-arena] AppLauncher initialization complete (0.1s)', flush=True)
    if mode == 'first':
        while not (root / 'finish_first').exists(): time.sleep(.005)
    print('KITCHEN_COLLECTOR_REPLY ' + json.dumps({'id': request['id'], 'exit_code': 0,
          'worker_pid': os.getpid(), 'elapsed_seconds': .1}), flush=True)
'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            lock = ObservedLock()
            command = [sys.executable, '-u', '-c', source, directory]
            with CollectorWorker(cwd=root, worker_command=command, startup_lock=lock,
                                 request_timeout_s=5.) as first, \
                 CollectorWorker(cwd=root, worker_command=command, startup_lock=lock,
                                 request_timeout_s=5.) as second, ThreadPoolExecutor(max_workers=2) as executor:
                one = executor.submit(first.execute, self.command('first'), root / 'one.log')
                try:
                    deadline = time.monotonic() + 2.
                    while not (root / 'first_started').exists() and time.monotonic() < deadline:
                        time.sleep(.005)
                    self.assertTrue((root / 'first_started').exists())
                    two = executor.submit(second.execute, self.command('second'), root / 'two.log')
                    self.assertTrue(lock.second_waiting.wait(2.))
                    self.assertIsNone(second.process)
                    self.assertTrue(lock.lock.locked())
                    (root / 'allow_marker').touch()
                    second_result = two.result(timeout=2.)
                    self.assertEqual(second_result['exit_code'], 0)
                    self.assertGreater(second_result['startup_wait_s'], 0.)
                    self.assertFalse(one.done(), 'Second startup waited for the whole first episode')
                    self.assertIn(STARTUP_COMPLETE_MARKER, (root / 'one.log').read_text())
                    self.assertFalse(lock.lock.locked())
                finally:
                    (root / 'allow_marker').touch()
                    (root / 'finish_first').touch()
                self.assertEqual(one.result(timeout=2.)['exit_code'], 0)

    def test_failed_startup_or_missing_marker_releases_lock_for_next_worker(self):
        for mode in ('die', 'wrong_id', 'stall', 'fail', 'popen_failure'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                lock = threading.Lock()
                command = ([str(root / 'missing_interpreter')] if mode == 'popen_failure'
                           else [sys.executable, '-u', '-c', WORKER])
                with CollectorWorker(cwd=root, worker_command=command, startup_lock=lock,
                                     request_timeout_s=.15, shutdown_timeout_s=.1) as failing:
                    result = failing.execute(self.command(mode), root / 'failed.log')
                    self.assertNotEqual(result['exit_code'], 0)
                    self.assertFalse(lock.locked())
                with self.worker(root, startup_lock=lock) as next_worker:
                    result = next_worker.execute(self.command(), root / 'next.log')
                    self.assertEqual(result['exit_code'], 0)
                    self.assertFalse(lock.locked())


if __name__ == '__main__':
    unittest.main()
