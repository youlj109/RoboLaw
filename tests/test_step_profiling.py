"""Deterministic CPU checks for nested natural wall-time attribution."""
from pathlib import Path
import sys
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.step_profiling import StepWallProfiler


class Clock:
    def __init__(self):
        self.ns = 0
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.ns

    def advance(self, seconds):
        self.ns += int(seconds * 1e9)


class StepWallProfilerTests(unittest.TestCase):
    def test_nested_same_and_different_categories_do_not_double_count(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)
        profiler.begin('lift')
        with profiler.section('physics_backend', hook='simulation_step'):
            clock.advance(2)
            with profiler.section('rendering', hook='render_dispatch'):
                clock.advance(3)
            with profiler.section('physics_backend', hook='physics_child'):
                clock.advance(4)
            clock.advance(1)
        profiler.finish(12)
        report = profiler.report()
        self.assertEqual(report['categories_seconds'],
                         {'physics_backend': 7, 'rendering': 3, 'other': 2})
        self.assertEqual(report['hooks']['simulation_step']['inclusive_seconds'], 10)
        self.assertEqual(report['hooks']['simulation_step']['exclusive_seconds'], 3)
        self.assertEqual(report['hooks']['physics_child']['exclusive_seconds'], 4)
        self.assertEqual(report['hooks']['simulation_step']['count'], 1)
        self.assertAlmostEqual(sum(report['categories_percent'].values()), 100)
        self.assertEqual(report['raw_residual_seconds'], 2)
        self.assertEqual(report['step_count'], 1)

    def test_recursive_same_hook_tracks_inclusive_time_without_category_duplication(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)
        profiler.begin('transfer')
        with profiler.section('rendering', hook='render_chain'):
            clock.advance(2)
            with profiler.section('rendering', hook='render_chain'):
                clock.advance(3)
            clock.advance(1)
        profiler.finish(6)
        report = profiler.report()
        self.assertEqual(report['categories_seconds'], {'rendering': 6, 'other': 0})
        self.assertEqual(report['hooks']['render_chain'], {
            'category': 'rendering', 'count': 2, 'inclusive_seconds': 9, 'exclusive_seconds': 6})

    def test_inactive_bound_method_and_module_wrappers_are_transparent(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)
        sentinel = object()
        calls = []
        failure = RuntimeError('original error identity')

        class Target:
            def call(self, value, *, keyword):
                calls.append((self, value, keyword))
                if keyword == 'raise':
                    raise failure
                return value

        target = Target()
        original = profiler.wrap(target, 'call', 'scene_update')
        module = types.ModuleType('sample_module')
        module.call = lambda value, *, keyword: (calls.append((value, keyword)) or value)
        profiler.wrap(module, 'call', 'rendering')
        self.assertIs(target.call(sentinel, keyword='bound'), sentinel)
        self.assertIs(module.call(sentinel, keyword='module'), sentinel)
        self.assertEqual(calls, [(target, sentinel, 'bound'), (sentinel, 'module')])
        self.assertIs(original.__self__, target)
        with self.assertRaises(RuntimeError) as raised:
            target.call(sentinel, keyword='raise')
        self.assertIs(raised.exception, failure)
        with profiler.section('safety_checks'):
            clock.advance(100)
        self.assertEqual(clock.calls, 0)
        self.assertEqual(profiler.report()['hooks'], {})
        self.assertEqual(profiler.report()['step_count'], 0)

    def test_wrapped_exception_unwinds_nested_timers_and_preserves_error(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)
        failure = ValueError('physical check failed')

        class Target:
            def call(self):
                clock.advance(2)
                with profiler.section('safety_checks', hook='state_readback'):
                    clock.advance(3)
                    raise failure

        target = Target()
        profiler.wrap(target, 'call', 'safety_checks', hook='physics_rate_safety')
        profiler.begin('approach')
        with self.assertRaises(ValueError) as raised:
            try:
                target.call()
            finally:
                profiler.finish(6)
        self.assertIs(raised.exception, failure)
        self.assertFalse(profiler.active)
        self.assertEqual(profiler.stack, [])
        report = profiler.report()
        self.assertEqual(report['categories_seconds'], {'safety_checks': 5, 'other': 1})
        self.assertEqual(report['hooks']['physics_rate_safety']['inclusive_seconds'], 5)
        self.assertEqual(report['hooks']['physics_rate_safety']['exclusive_seconds'], 2)
        self.assertEqual(report['hooks']['state_readback']['count'], 1)

    def test_slots_class_method_wrapper_preserves_instance_binding(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)

        class RenderContext:
            __slots__ = ('calls',)

            def __init__(self):
                self.calls = []

            def render_into_camera(self, camera, *, duration):
                self.calls.append(camera)
                clock.advance(duration)
                return camera

        first, second = RenderContext(), RenderContext()
        camera = object()
        original = profiler.wrap(RenderContext, 'render_into_camera', 'rendering', hook='camera_render_chain')
        self.assertIs(first.render_into_camera(camera, duration=2), camera)
        self.assertEqual(clock.calls, 0)  # Inactive class wrapping remains transparent.
        profiler.begin('transfer')
        self.assertIs(second.render_into_camera(camera, duration=3), camera)
        profiler.finish(4)
        self.assertEqual(first.calls, [camera])
        self.assertEqual(second.calls, [camera])
        self.assertIs(RenderContext.render_into_camera.__wrapped__, original)
        report = profiler.report()
        self.assertEqual(report['categories_seconds'], {'rendering': 3, 'other': 1})
        self.assertEqual(report['hooks']['camera_render_chain']['count'], 1)

    def test_phase_totals_and_unclipped_negative_residual(self):
        clock = Clock()
        profiler = StepWallProfiler(clock=clock)
        module = types.ModuleType('sample_module')
        module.advance = clock.advance
        profiler.wrap(module, 'advance', 'rendering', hook='camera_render_chain')
        for phase, category, duration, total in (
                ('lift', 'safety_checks', 2, 3), ('release', 'rendering', 1, 4),
                ('lift', 'physics_backend', 1, 2)):
            profiler.begin(phase)
            if category == 'rendering':
                module.advance(duration)
            else:
                with profiler.section(category):
                    clock.advance(duration)
            profiler.finish(total)
        report = profiler.report()
        self.assertEqual(report['total_seconds'], 9)
        self.assertEqual(report['step_count'], 3)
        self.assertEqual(report['per_phase']['lift']['step_count'], 2)
        self.assertEqual(report['per_phase']['lift']['total_seconds'], 5)
        self.assertEqual(report['per_phase']['lift']['categories_seconds'],
                         {'safety_checks': 2, 'physics_backend': 1, 'other': 2})
        self.assertEqual(report['per_phase']['release']['categories_seconds'], {'rendering': 1, 'other': 3})
        self.assertEqual(report['per_phase']['release']['categories_percent']['rendering'], 25)
        self.assertEqual(report['hooks']['camera_render_chain']['count'], 1)
        # Retain inconsistent denominator evidence instead of hiding it by
        # clipping the raw residual. No simulated work or device wait is added.
        profiler.begin('short_denominator')
        with profiler.section('rendering'):
            clock.advance(5)
        profiler.finish(4)
        phase = profiler.report()['per_phase']['short_denominator']
        self.assertEqual(phase['raw_residual_seconds'], -1)
        self.assertEqual(phase['categories_seconds']['other'], 0)
        self.assertEqual(phase['categories_percent']['rendering'], 125)


if __name__ == '__main__':
    unittest.main()
