"""Execute the real PhysxManager.step AST against CPU-only fake dependencies."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.step_profiling import StepWallProfiler

SOURCE = ROOT / ('third_party/IsaacLab-Arena/submodules/IsaacLab/source/'
                 'isaaclab_physx/isaaclab_physx/physics/physx_manager.py')


class Clock:
    def __init__(self):
        self.ns = 0
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.ns

    def advance(self, seconds):
        self.ns += int(seconds * 1e9)


def make_manager(*, device='cuda:0', profiler=None, clock=None,
                 fail_at=None, failure=None, missing_profiler=False):
    """Compile only the real method, never importing its Kit-dependent module."""
    clock = clock or Clock()
    events = []

    def invoke(name, duration, *args):
        events.append((name, *args))
        clock.advance(duration)
        if name == fail_at:
            raise failure

    physics = SimpleNamespace(
        simulate=lambda dt, start: invoke('simulate', 2, dt, start),
        fetch_results=lambda: invoke('fetch_results', 5))
    base = SimpleNamespace(_sim=SimpleNamespace(cfg=SimpleNamespace(dt=1 / 120)), _device=device)
    namespace = {
        'PhysicsManager': base,
        'omni': SimpleNamespace(physx=SimpleNamespace(get_physx_simulation_interface=lambda: physics)),
        'torch': SimpleNamespace(cuda=SimpleNamespace(set_device=lambda value: invoke('set_device', 1, value))),
    }
    tree = ast.parse(SOURCE.read_text(), filename=str(SOURCE))
    manager = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'PhysxManager')
    method = next(node for node in manager.body if isinstance(node, ast.FunctionDef) and node.name == 'step')
    extracted = ast.Module(body=[method], type_ignores=[])
    exec(compile(extracted, str(SOURCE), 'exec'), namespace)
    attrs = {'step': namespace['step'], '_anim_recorder': None,
             'raise_callback_exception_if_any': classmethod(lambda cls: invoke('callback_check', 1))}
    if not missing_profiler:
        attrs['_step_wall_profiler'] = profiler
    return type('FakePhysxManager', (), attrs), events, clock, base


class PhysxStepProfilingTests(unittest.TestCase):
    def test_active_inactive_and_missing_profiler_preserve_cuda_call_order_and_arguments(self):
        expected = [('simulate', 1 / 120, 0.0), ('fetch_results',), ('set_device', 'cuda:0'), ('callback_check',)]
        for mode in ('missing', 'none', 'inactive', 'active'):
            with self.subTest(mode=mode):
                clock = Clock()
                profiler = StepWallProfiler(clock=clock)
                manager, events, _, _ = make_manager(clock=clock,
                    profiler=profiler if mode in ('inactive', 'active') else None,
                    missing_profiler=mode == 'missing')
                if mode == 'active':
                    profiler.begin('lift')
                self.assertIsNone(manager.step())
                self.assertEqual(events, expected)
                if mode == 'active':
                    profiler.finish(9)
                    hooks = profiler.report()['hooks']
                    self.assertEqual(set(hooks), {'physx_simulate', 'physx_fetch_results', 'physx_set_device'})
                    self.assertTrue(all(item['count'] == 1 for item in hooks.values()))
                else:
                    self.assertEqual(clock.calls, 0)
                    self.assertEqual(profiler.report()['hooks'], {})

    def test_cpu_device_never_calls_cuda_set_device(self):
        for active in (False, True):
            with self.subTest(active=active):
                clock = Clock()
                profiler = StepWallProfiler(clock=clock)
                manager, events, _, _ = make_manager(device='cpu', clock=clock, profiler=profiler)
                if active:
                    profiler.begin('approach')
                manager.step()
                if active:
                    profiler.finish(8)
                self.assertEqual(events, [('simulate', 1 / 120, 0.0), ('fetch_results',), ('callback_check',)])
                self.assertNotIn('physx_set_device', profiler.report()['hooks'])

    def test_simulate_and_fetch_exceptions_propagate_without_later_calls(self):
        for active in (False, True):
            for failure_point in ('simulate', 'fetch_results'):
                with self.subTest(active=active, failure_point=failure_point):
                    clock = Clock()
                    profiler = StepWallProfiler(clock=clock)
                    failure = RuntimeError('original backend exception')
                    manager, events, _, _ = make_manager(clock=clock, profiler=profiler,
                        fail_at=failure_point, failure=failure)
                    if active:
                        profiler.begin('transfer')
                    with self.assertRaises(RuntimeError) as raised:
                        try:
                            manager.step()
                        finally:
                            if active:
                                profiler.finish(clock.ns / 1e9)
                    self.assertIs(raised.exception, failure)
                    expected = [('simulate', 1 / 120, 0.0)]
                    if failure_point == 'fetch_results':
                        expected.append(('fetch_results',))
                    self.assertEqual(events, expected)
                    self.assertEqual(profiler.stack, [])
                    self.assertFalse(profiler.active)
                    if active:
                        self.assertEqual(profiler.report()['hooks']['physx_' + failure_point]['count'], 1)
                    else:
                        self.assertEqual(clock.calls, 0)

    def test_deferred_callback_exception_keeps_original_order_and_identity(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)
        failure = ValueError('deferred callback error')
        manager, events, _, _ = make_manager(clock=clock, profiler=profiler,
            fail_at='callback_check', failure=failure)
        profiler.begin('lower')
        with self.assertRaises(ValueError) as raised:
            try:
                manager.step()
            finally:
                profiler.finish(9)
        self.assertIs(raised.exception, failure)
        self.assertEqual([event[0] for event in events],
                         ['simulate', 'fetch_results', 'set_device', 'callback_check'])
        self.assertEqual(profiler.stack, [])

    def test_nested_parent_total_counts_backend_children_once(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)
        manager, _, _, _ = make_manager(clock=clock, profiler=profiler)
        profiler.begin('lift')
        with profiler.section('physics_backend', hook='simulation_step'):
            clock.advance(1)
            manager.step()
            clock.advance(1)
        profiler.finish(12)
        report = profiler.report()
        self.assertEqual(report['categories_seconds'], {'physics_backend': 11, 'other': 1})
        self.assertEqual(report['hooks']['simulation_step']['inclusive_seconds'], 11)
        self.assertEqual(report['hooks']['simulation_step']['exclusive_seconds'], 3)
        self.assertEqual(report['hooks']['physx_simulate']['exclusive_seconds'], 2)
        self.assertEqual(report['hooks']['physx_fetch_results']['exclusive_seconds'], 5)
        self.assertEqual(report['hooks']['physx_set_device']['exclusive_seconds'], 1)
        self.assertEqual(sum(hook['exclusive_seconds'] for hook in report['hooks'].values()), 11)

    def test_missing_simulation_keeps_early_return_without_backend_or_timer_calls(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)
        manager, events, _, base = make_manager(clock=clock, profiler=profiler)
        base._sim = None
        profiler.begin('reset')
        self.assertIsNone(manager.step())
        profiler.finish(0)
        self.assertEqual(events, [])
        self.assertEqual(clock.calls, 0)


if __name__ == '__main__':
    unittest.main()
