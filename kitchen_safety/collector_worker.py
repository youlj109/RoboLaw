"""Lazy persistent-Kit client with the same execution records as run_command.

Only collector commands belong here. Validation, audits and preview export
remain ordinary CPU subprocesses. One client accepts one request at a time.
"""
from datetime import datetime, timezone
import errno
import json
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[1]
REPLY_PREFIX = 'KITCHEN_COLLECTOR_REPLY '
STARTUP_COMPLETE_MARKER = '[isaaclab-arena] AppLauncher initialization complete'
STORAGE_ERROR_TEXT = re.compile(
    r'\b(?:ENOSPC|EDQUOT)\b|\[Errno (?:28|122)\]|\berrno\s*=\s*(?:28|122)\b|'
    r'no space left on device|disk quota exceeded', re.IGNORECASE)


def _storage_error(value):
    if isinstance(value, OSError) and value.errno in (errno.ENOSPC, errno.EDQUOT):
        return f'{type(value).__name__}: {value}'
    if isinstance(value, str) and STORAGE_ERROR_TEXT.search(value):
        return value.strip()[:2000]
    return None


class CollectorWorker:
    """Use ``with CollectorWorker(cwd=ROOT) as worker: worker.execute(cmd, log)``.

The subprocess starts on the first execute(), not on construction or entry.
Worker death, EOF, malformed replies and bounded timeouts become unsuccessful
execution dictionaries. A broken worker is never silently restarted. close()
sends stdin EOF, then bounds shutdown waits and reaps the owned process group.
An optional shared startup_lock serializes only first app initialization;
the AppLauncher completion marker releases it before episode collection ends.
    """

    def __init__(self, *, cwd=ROOT, worker_command=None, request_timeout_s=3600., shutdown_timeout_s=30.,
                 kit_log_path=None, startup_lock=None):
        self.cwd = Path(cwd)
        self._default_worker_command = worker_command is None
        self.kit_log_path = (self.cwd / kit_log_path).resolve() if kit_log_path is not None else None
        self.worker_command = (list(worker_command) if worker_command is not None else
                               [sys.executable, '-u', str(ROOT / 'scripts/collect_kitchen_sequence.py'), '--serve'])
        if request_timeout_s <= 0 or shutdown_timeout_s < 0:
            raise ValueError('Request timeout must be positive and shutdown timeout nonnegative')
        self.request_timeout_s = float(request_timeout_s)
        self.shutdown_timeout_s = float(shutdown_timeout_s)
        self.process = None
        self._reader = None
        self._lines = queue.Queue()
        self._serial = 0
        self._closed = False
        self._broken = None
        self.shutdown = None
        self.startup_lock = startup_lock
        self._startup_lock_held = False
        self._startup_wait_s = 0.

    def __enter__(self):
        if self._closed:
            raise RuntimeError('Collector worker has already been closed')
        return self

    def __exit__(self, exc_type, exc_value, exc_tb):
        self.close()
        return False

    def _start(self, log_path):
        if self.process is not None:
            return
        if self.startup_lock is not None:
            begin = time.monotonic()
            try:
                self._startup_lock_held = self.startup_lock.acquire(timeout=self.request_timeout_s)
            finally:
                self._startup_wait_s = time.monotonic() - begin
            if not self._startup_lock_held:
                raise TimeoutError('Timed out waiting to start collector worker')
        if self._default_worker_command:
            if self.kit_log_path is None:
                self.kit_log_path = Path(log_path).resolve().parent / 'kit.log'
            self.worker_command += ['--kit-log', str(self.kit_log_path)]
        try:
            self.process = subprocess.Popen(self.worker_command, cwd=self.cwd, stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            text=True, encoding='utf-8', errors='replace', bufsize=1,
                                            start_new_session=True)
        except (OSError, subprocess.SubprocessError) as error:
            self._broken = f'Cannot start collector worker: {error}'
            raise

        def read_lines():
            try:
                for line in self.process.stdout:
                    self._lines.put(('line', line))
            except (OSError, ValueError) as error:
                self._lines.put(('error', str(error)))
            finally:
                self._lines.put(('eof', None))

        self._reader = threading.Thread(target=read_lines, name='collector-worker-output', daemon=True)
        self._reader.start()

    def _release_startup_lock(self):
        if self._startup_lock_held:
            self._startup_lock_held = False
            self.startup_lock.release()

    @staticmethod
    def _collector_arguments(command):
        command = [str(part) for part in command]
        positions = [index for index, part in enumerate(command) if Path(part).name == 'collect_kitchen.py']
        if len(positions) != 1:
            raise ValueError('CollectorWorker.execute expects a collect_kitchen.py command')
        return command[positions[0] + 1:]

    def execute(self, command, log_path):
        """Run one collector and return exit_code/started_at/elapsed_seconds.

Additional worker_pid, result_path and worker_elapsed_seconds identify the
shared process execution; startup_wait_s records lock waiting on first start.
Logs are exclusively created, as in run_command.
        """
        started = datetime.now(timezone.utc).isoformat()
        begin = time.monotonic()
        execution = {'exit_code': None, 'started_at': started}
        self._startup_wait_s = 0.
        storage_evidence = None
        try:
            with Path(log_path).open('x') as log:
                argv = self._collector_arguments(command)
                if self._closed or self._broken:
                    raise RuntimeError(self._broken or 'Collector worker has already been closed')
                self._start(log_path)
                if self.process.poll() is not None:
                    raise RuntimeError(f'Collector worker exited before request: {self.process.returncode}')
                self._serial += 1
                identifier = str(self._serial)
                self.process.stdin.write(json.dumps({'id': identifier, 'argv': argv}) + '\n')
                self.process.stdin.flush()
                deadline = begin + self.request_timeout_s
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f'Collector worker request exceeded {self.request_timeout_s:g} seconds')
                    try:
                        kind, line = self._lines.get(timeout=min(.1, remaining))
                    except queue.Empty:
                        if self.process.poll() is not None:
                            raise RuntimeError(f'Collector worker exited without a reply: {self.process.returncode}')
                        continue
                    if kind != 'line':
                        raise RuntimeError(f'Collector worker output ended without a reply ({kind}: {line})')
                    if STARTUP_COMPLETE_MARKER in line:
                        # Release at app startup, not at the end of the first
                        # episode: subsequent scene collection runs in parallel.
                        self._release_startup_lock()
                    position = line.find(REPLY_PREFIX)
                    if position >= 0:
                        if position:
                            log.write(line[:position])
                        try:
                            response = json.loads(line[position + len(REPLY_PREFIX):])
                        except ValueError as error:
                            raise RuntimeError(f'Malformed collector worker reply: {line.strip()}') from error
                        if not isinstance(response, dict) or str(response.get('id')) != identifier:
                            raise RuntimeError(f'Collector worker reply ID does not match request {identifier}: {response!r}')
                        if response.get('exit_code') is not None and not isinstance(response['exit_code'], int):
                            raise RuntimeError('Collector worker returned an invalid exit_code')
                        execution.update(exit_code=response.get('exit_code'),
                                         worker_pid=response.get('worker_pid', self.process.pid),
                                         result_path=response.get('result_path'),
                                         worker_elapsed_seconds=response.get('elapsed_seconds'))
                        if response.get('error'):
                            execution['error'] = response['error']
                            storage_evidence = storage_evidence or _storage_error(response['error'])
                        if response.get('worker_stopping'):
                            self._broken = 'Collector worker is stopping after this request'
                            execution['worker_unavailable'] = True
                        break
                    log.write(line)
                    log.flush()
                    storage_evidence = storage_evidence or _storage_error(line)
        except (OSError, subprocess.SubprocessError, ValueError, RuntimeError, TimeoutError) as error:
            execution['error'] = f'{type(error).__name__}: {error}'
            storage_evidence = storage_evidence or _storage_error(error)
            if self.process is not None or self._broken:
                self._broken = execution['error']
                execution['worker_unavailable'] = True
                self._kill_group()
        finally:
            # Covers failed startup, missing marker, protocol errors and all
            # returns, so another worker is never blocked by a dead process.
            self._release_startup_lock()
            execution['startup_wait_s'] = round(self._startup_wait_s, 6)
            execution['elapsed_seconds'] = round(time.monotonic() - begin, 3)
        if storage_evidence and execution['exit_code'] != 0:
            execution['storage_error'] = storage_evidence
        return execution

    def _kill_group(self):
        if self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def close(self):
        self._release_startup_lock()
        if self._closed:
            return
        self._closed = True
        if self.process is None:
            self.shutdown = {'started': False}
            return
        begin = time.monotonic()
        forced = False
        try:
            if self.process.stdin:
                try:
                    self.process.stdin.close()  # EOF tells the server to close Kit.
                except BrokenPipeError:
                    pass
            try:
                self.process.wait(timeout=self.shutdown_timeout_s)
            except subprocess.TimeoutExpired:
                forced = True
                self._kill_group()
                self.process.wait(timeout=5.)
        finally:
            # A terminated Kit parent can leave helper children holding stdout.
            # They belong to our private process group, not the batch's group.
            self._kill_group()
            if self._reader is not None:
                self._reader.join(timeout=1.)
            # A detached native helper can outlive the process group while
            # retaining stdout. Closing its TextIOWrapper from this thread
            # would wait on the reader's blocking readline lock indefinitely.
            if self.process.stdout is not None and (self._reader is None or not self._reader.is_alive()):
                self.process.stdout.close()
            self.shutdown = {'started': True, 'exit_code': self.process.poll(), 'forced': forced,
                             'elapsed_seconds': time.monotonic() - begin}
