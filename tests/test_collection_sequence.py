"""Exercise the actual runpy path and cleanup without importing or starting Kit."""
import gc
import io
import errno
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from scripts.collect_kitchen_sequence import prepare_entries, run_sequence, serve, REPLY_PREFIX
from scripts import collect_kitchen_sequence as sequence
from kitchen_safety.step_profiling import StepWallProfiler


COLLECTOR = '''
import argparse
import json
from pathlib import Path
import weakref
import sequence_test_runtime as runtime
from sequence_test_helpers import SimulationAppContext
from kitchen_safety.step_profiling import StepWallProfiler
parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
parser.add_argument('--mode', default='pass')
parser.add_argument('--record-rgb', action='store_true')
parser.add_argument('--device', default='cpu')
args = parser.parse_args()
args.enable_cameras = args.record_rgb
runtime.namespace_preexisting.append(globals().get('local_counter', 0))
local_counter = 1
with SimulationAppContext(args):
    payload = runtime.Payload()
    runtime.payload_refs.append(weakref.ref(payload))
    env = runtime.Env()
    runtime.envs.append(env)
    runtime.wrapper_before.append(hasattr(runtime.Renderer.render, '__wrapped__'))
    profiler = StepWallProfiler()
    runtime.Physics._step_wall_profiler = profiler
    profiler.wrap(runtime.Renderer, 'render', 'rendering')
    runtime.Renderer().render()
    result = {'success': args.mode == 'pass', 'steps': 3,
              'failure_kind': None if args.mode == 'pass' else 'physical_failed'}
    (Path(args.output) / 'result.json').write_text(json.dumps(result))
    if args.mode == 'crash':
        raise RuntimeError('failure inside borrowed context')
    env.close()
if not result['success']:
    raise SystemExit(1)
'''


class SequenceTests(unittest.TestCase):
    def helpers(self):
        helpers = types.ModuleType('sequence_test_helpers')
        runtime = types.ModuleType('sequence_test_runtime')
        helpers.SimulationAppContext = type('OriginalContext', (), {})
        runtime.namespace_preexisting, runtime.payload_refs, runtime.wrapper_before, runtime.envs = [], [], [], []
        runtime.render_calls = 0
        class Payload:
            pass
        class Renderer:
            def render(self):
                runtime.render_calls += 1
        class Env:
            def __init__(self):
                self.closed, self.close_count = False, 0
            def close(self):
                if not self.closed:
                    self.closed = True
                    self.close_count += 1
        class Physics:
            _step_wall_profiler = 'before'
        runtime.Payload, runtime.Renderer, runtime.Env, runtime.Physics = Payload, Renderer, Env, Physics
        helpers.startups, helpers.teardowns, helpers.collections, helpers.closes = [], 0, 0, 0
        def close_app():
            helpers.closes += 1
        helpers.app = types.SimpleNamespace(close=close_app, is_running=lambda: True, is_exiting=lambda: False)
        def startup(args):
            helpers.startups.append(vars(args).copy())
            return types.SimpleNamespace(app=helpers.app)
        def teardown(**kwargs):
            helpers.teardowns += 1
            self.assertEqual(kwargs, {'suppress_exceptions': False, 'make_new_stage': True})
            self.assertFalse(hasattr(Renderer.render, '__wrapped__'))
            self.assertEqual(Physics._step_wall_profiler, 'before')
        def collect():
            helpers.collections += 1
            gc.collect()
        helpers.get_app_launcher = startup
        helpers.teardown_simulation_app = teardown
        helpers.collect_garbage_and_clear_cuda_cache = collect
        return helpers, runtime

    def test_one_app_independent_namespaces_and_failure_cleanup_then_continue(self):
        helpers, runtime = self.helpers()
        original_context, original_wrap = helpers.SimulationAppContext, StepWallProfiler.wrap
        original_argv = sys.argv
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            collector = root / 'collector.py'
            collector.write_text(COLLECTOR)
            entries = prepare_entries([
                {'id': 'first', 'argv': ['--output', 'one']},
                ['--output=two', '--record-rgb', '--mode', 'crash'],
                ['--output', 'three', '--mode', 'fail'],
                ['--output', 'four']], workspace=root)
            with patch.dict(sys.modules, {'sequence_test_helpers': helpers, 'sequence_test_runtime': runtime}):
                report = run_sequence(entries, root / 'report.json', simulation_helpers=helpers,
                                      collector_path=collector, physics_manager_class=runtime.Physics,
                                      kit_log_path=root / 'worker' / 'kit.log')
            self.assertEqual(len(helpers.startups), 1)
            self.assertTrue(helpers.startups[0]['enable_cameras'])
            self.assertEqual(helpers.startups[0]['kit_args'], f'--/log/file={root / "worker" / "kit.log"}')
            self.assertIsNone(report['app_settings']['kit_args'])
            self.assertEqual(helpers.closes, 1)
            self.assertEqual(helpers.teardowns, 4)
            self.assertEqual(helpers.collections, 4)
            self.assertEqual(runtime.namespace_preexisting, [0, 0, 0, 0])
            self.assertEqual(runtime.wrapper_before, [False] * 4)
            self.assertEqual(runtime.render_calls, 4)
            self.assertTrue(all(env.closed and env.close_count == 1 for env in runtime.envs))
            self.assertTrue(all(reference() is None for reference in runtime.payload_refs))
            self.assertEqual([entry['exit_code'] for entry in report['episodes']], [0, 1, 1, 0])
            self.assertEqual(report['successful_collectors'], 2)
            self.assertEqual(report['status'], 'completed')
            self.assertFalse(report['passed'])
            self.assertIn('failure inside borrowed context', report['episodes'][1]['error'])
            for entry in entries:
                output = Path(entry['output'])
                self.assertTrue((output / 'result.json').exists())
                self.assertTrue((output / 'sequence_execution.json').exists())
                self.assertTrue((output / 'sequence_run.log').exists())
            self.assertEqual(json.loads((root / 'report.json').read_text())['app_startups'], 1)
        self.assertIs(helpers.SimulationAppContext, original_context)
        self.assertIs(StepWallProfiler.wrap, original_wrap)
        self.assertIs(sys.argv, original_argv)

    def test_recorded_outputs_and_duplicate_paths_rejected_specs_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / 'episode'
            output.mkdir()
            (output / 'episode_spec.json').write_text('{}')
            self.assertEqual(len(prepare_entries([['--output', 'episode']], root)), 1)
            with self.assertRaisesRegex(ValueError, 'duplicate'):
                prepare_entries([['--output', 'episode'], ['--output=episode']], root)
            for name in ['result.json', 'trajectory.h5', 'images.h5', 'sequence_execution.json']:
                with self.subTest(name=name):
                    (output / name).write_text('previous data')
                    with self.assertRaises(FileExistsError):
                        prepare_entries([['--output', 'episode']], root)
                    self.assertEqual((output / name).read_text(), 'previous data')
                    (output / name).unlink()
            with self.assertRaises(ValueError):
                prepare_entries([['--output', 'one', '--output=two']], root)

    def test_cleanup_failure_stops_next_episode_and_preserves_report(self):
        helpers, runtime = self.helpers()
        helpers.teardown_simulation_app = lambda **kwargs: (_ for _ in ()).throw(RuntimeError('cannot create next stage'))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            collector = root / 'collector.py'
            collector.write_text(COLLECTOR)
            entries = prepare_entries([['--output', 'one'], ['--output', 'two']], root)
            with patch.dict(sys.modules, {'sequence_test_helpers': helpers, 'sequence_test_runtime': runtime}):
                report = run_sequence(entries, root / 'report.json', simulation_helpers=helpers,
                                      collector_path=collector, physics_manager_class=runtime.Physics)
            self.assertEqual(len(report['episodes']), 1)
            self.assertEqual(report['status'], 'stopped')
            self.assertFalse(report['passed'])
            self.assertIn('cannot create next stage', report['episodes'][0]['cleanup_errors'][0])
            self.assertEqual(helpers.closes, 1)
            self.assertFalse((root / 'two').exists())

    def test_jsonl_serve_keeps_app_until_eof_and_checks_camera_startup_consistency(self):
        helpers, runtime = self.helpers()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            collector = root / 'collector.py'
            collector.write_text(COLLECTOR)
            requests = [
                {'id': 'first', 'argv': ['--output', 'one', '--record-rgb']},
                {'id': 'different', 'argv': ['--output', 'two']},
                {'id': 'last', 'argv': ['--output', 'three', '--record-rgb']},
            ]
            source = io.StringIO('not json\n' + '\n'.join(map(json.dumps, requests)) + '\n')
            replies = io.StringIO()
            with patch.dict(sys.modules, {'sequence_test_helpers': helpers, 'sequence_test_runtime': runtime}):
                report = serve(source, replies, workspace=root, simulation_helpers=helpers,
                               collector_path=collector, physics_manager_class=runtime.Physics)
            parsed = [json.loads(line[len(REPLY_PREFIX):]) for line in replies.getvalue().splitlines()
                      if line.startswith(REPLY_PREFIX)]
            self.assertEqual([row['id'] for row in parsed], [None, 'first', 'different', 'last'])
            self.assertEqual([row['exit_code'] for row in parsed], [None, 0, 1, 0])
            self.assertEqual(len(helpers.startups), 1)
            self.assertEqual(helpers.closes, 1)
            self.assertEqual(helpers.teardowns, 3)
            self.assertEqual(report['requested_episodes'], 3)
            self.assertEqual(report['successful_collectors'], 2)
            self.assertIn('startup arguments differ', parsed[2]['error'])

    def test_jsonl_reports_failed_checkpoint_before_app_shutdown(self):
        helpers, runtime = self.helpers()
        replies = io.StringIO()
        normal_close = helpers.app.close
        def close():
            self.assertIn('KITCHEN_COLLECTOR_REPLY', replies.getvalue())
            normal_close()
        helpers.app.close = close
        original_write = sequence.write_json
        def write(path, value):
            if path is not None and Path(path).name == 'sequence_execution.json':
                raise OSError(errno.ENOSPC, 'No space left on device')
            return original_write(path, value)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            collector = root / 'collector.py'
            collector.write_text(COLLECTOR)
            source = io.StringIO(json.dumps({'id': 'first', 'argv': ['--output', 'one']}) + '\n')
            with patch.dict(sys.modules, {'sequence_test_helpers': helpers, 'sequence_test_runtime': runtime}), \
                    patch.object(sequence, 'write_json', side_effect=write):
                report = serve(source, replies, workspace=root, simulation_helpers=helpers,
                               collector_path=collector, physics_manager_class=runtime.Physics)
            response = json.loads(next(line[len(REPLY_PREFIX):] for line in replies.getvalue().splitlines()
                                       if line.startswith(REPLY_PREFIX)))
            self.assertEqual(response['id'], 'first')
            self.assertIsNone(response['exit_code'])
            self.assertTrue(response['worker_stopping'])
            self.assertIn('No space left', response['error'])
            self.assertFalse(report['passed'])
            self.assertEqual(helpers.closes, 1)


if __name__ == '__main__':
    unittest.main()
