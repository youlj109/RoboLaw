"""Host client for one persistent recorded-state renderer (stdlib only)."""
import json
from pathlib import Path
import subprocess
import time


def _write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


class RenderWorker:
    """One owner, one outstanding request; stdin EOF also handles host death.

    Replies describe requests, never fictitious subprocess exits. Closing waits
    for the actual owned docker-exec/Kit process instead of killing its client.
    """

    def __init__(self, *, cwd, directory, grouped_camera=False,
                 refresh_render_products=False, collection_purpose='diagnostic',
                 collection_source=None, command=None):
        self.cwd, self.directory = Path(cwd).resolve(), Path(directory).resolve()
        self.command = command
        self.grouped_camera = grouped_camera
        self.refresh_render_products = refresh_render_products
        if collection_purpose not in ('diagnostic', 'production'):
            raise ValueError('collection_purpose must be diagnostic or production')
        self.collection_purpose, self.collection_source = collection_purpose, collection_source
        self.process = self.log = None
        self.index = 0
        self.closed = self.unavailable = False
        self.started = None
        self.lifecycle = {'started': False, 'process_reaped': False,
                          'collection_purpose': collection_purpose, 'collection_source': collection_source,
                          'execution_scope': 'shared_persistent_renderer_process'}

    def _relative(self, path):
        resolved = Path(path).resolve()
        try:
            # Project-local paths remain portable between host and /workspace.
            return str(resolved.relative_to(self.cwd))
        except ValueError:
            # External mounted datasets keep their existing absolute namespace.
            return str(resolved)

    def _start(self):
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / 'requests').mkdir()
        (self.directory / 'replies').mkdir()
        command = self.command or [
            'bash', 'scripts/arena_python.sh', 'scripts/render_recorded_batch.py',
            '--serve', '--request-dir', self._relative(self.directory),
            '--report', self._relative(self.directory / 'report.json'),
            '--device', 'cpu', '--num_envs', '1', '--camera-device', 'cuda:0', '--background-rgb']
        if self.command is None:
            if self.grouped_camera:
                command.append('--grouped-camera')
            if self.refresh_render_products:
                command.append('--refresh-render-products')
            if self.collection_purpose != 'diagnostic':
                command.extend(['--collection-purpose', self.collection_purpose])
            if self.collection_source is not None:
                command.extend(['--collection-source', str(self.collection_source)])
        self.started = time.perf_counter()
        self.lifecycle.update(command=command, log=str(self.directory / 'worker.log'),
                              server_report=str(self.directory / 'report.json'))
        self.log = (self.directory / 'worker.log').open('w')
        try:
            self.process = subprocess.Popen(command, cwd=self.cwd, stdin=subprocess.PIPE,
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            self.lifecycle.update(started=True, pid=self.process.pid)
        except BaseException:
            self.log.close()
            self.log = None
            raise

    def execute(self, source_episode, output):
        tick = time.perf_counter()
        request_id = f'{self.index:05d}'
        self.index += 1
        result = {'id': request_id, 'execution_scope': 'persistent_render_request'}
        try:
            if self.closed or self.unavailable:
                raise RuntimeError('Renderer worker is unavailable; no automatic retry')
            source, destination = Path(source_episode).resolve(), Path(output).resolve()
            if not source.is_dir():
                raise FileNotFoundError(source)
            if destination.exists():
                raise FileExistsError(destination)
            if source == destination or destination in source.parents:
                raise ValueError('Output cannot replace the source directory or an ancestor')
            item = {'id': request_id, 'source_episode': self._relative(source),
                    'output': self._relative(destination)}
            if self.process is None:
                self._start()
            _write_json(self.directory / 'requests' / (request_id + '.json'), item)
            reply = self.directory / 'replies' / (request_id + '.json')
            while True:
                ready = self.directory / 'ready.json'
                if 'startup_to_ready_s' not in self.lifecycle and ready.exists():
                    self.lifecycle['startup_to_ready_s'] = time.perf_counter() - self.started
                    self.lifecycle['server_ready'] = json.loads(ready.read_text())
                if reply.exists():
                    result.update(json.loads(reply.read_text()))
                    if result.get('worker_unavailable'):
                        self.unavailable = True
                    break
                exit_code = self.process.poll()
                if exit_code is not None:
                    self.lifecycle.update(exit_code=exit_code, process_reaped=True)
                    raise RuntimeError(f'Renderer exited before request reply: {exit_code}')
                time.sleep(.05)
        except Exception as error:
            self.unavailable = True
            self.lifecycle['error'] = repr(error)
            result.update(status='worker_unavailable', worker_unavailable=True, error=repr(error))
        finally:
            result['request_wall_s'] = time.perf_counter() - tick
        return result

    def close(self):
        if self.closed:
            return self.report()
        self.closed = True
        tick, pending_error = time.perf_counter(), None
        try:
            if self.process is not None:
                if self.process.stdin is not None:
                    while not self.process.stdin.closed:
                        try:
                            self.process.stdin.close()
                        except BaseException as error:
                            pending_error = pending_error or error
                while True:
                    try:
                        exit_code = self.process.wait()
                        self.lifecycle.update(exit_code=exit_code, process_reaped=True)
                        break
                    except BaseException as error:
                        pending_error = pending_error or error
        finally:
            if self.log is not None:
                self.log.close()
            self.lifecycle['shutdown_wait_s'] = time.perf_counter() - tick
            self.lifecycle['lifetime_s'] = (time.perf_counter() - self.started
                                          if self.started is not None else 0.)
            self.lifecycle['closed'] = True
        if pending_error is not None:
            raise pending_error
        return self.report()

    def report(self):
        return dict(self.lifecycle)
