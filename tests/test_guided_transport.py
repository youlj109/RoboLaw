"""Real path-check/retiming and collector flow, with an explicit CPU solver double.

These tests exercise no Kit, GPU, human mesh or physical robot. The boolean
cuRobo query and planning return are controlled; production validate_path,
timing, prefix selection and collector integration execute normally.
"""
from copy import deepcopy
import ast
import contextlib
import io
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.guided_transport import replan_guided_suffix, resolve_guided_transport_phases
from kitchen_safety.augmentation_geometry import probe_obstruction
from kitchen_safety.planning_checks import check_planned_path
from kitchen_safety.vectorized_episode import VectorizedEpisode


class CpuResult:
    def __init__(self, values):
        self.values = np.asarray(values)
    def detach(self):
        return self
    def cpu(self):
        return self
    def numpy(self):
        return self.values


class World:
    attachment = np.eye(4)
    gripper_joint_position = .3
    last_collision = None
    last_clearance = None
    def valid(self, q, ignore=()):
        return True
    def plan(self, start, goal, ignore, **kwargs):
        if kwargs.get('max_iterations') == 1000:
            raise RuntimeError('Original full RRT reached')
        return np.array([start, goal])


class Planner(CuroboPlanner):
    def __init__(self, world):
        self.world = world
        self.dt, self.max_joint_speed, self._last_grip = 1/15, .2, .3
        self.args = SimpleNamespace(to_device=lambda value: value)
        self.motion_gen = SimpleNamespace(ik_solver=SimpleNamespace(check_valid=lambda samples:
            CpuResult(~((samples[:, 0] > .065) & (samples[:, 0] < .075)
                        & (np.abs(samples[:, 1]) < .005)))))
        self._prepare_path_query = Mock(return_value=.015)
        self.bridge_calls = []
        self.bridge_failures = 0
        self.reject_initial_plan = False

    def plan_joint_goal(self, start, goal, ignore=(), grip=0, phase='above_bowl'):
        self.bridge_calls.append((start.copy(), goal.copy(), ignore, grip, phase))
        if self.reject_initial_plan and len(self.bridge_calls) == 1:
            raise RuntimeError('Original full MotionGen failed')
        if self.bridge_failures:
            self.bridge_failures -= 1
            raise RuntimeError('Local bridge solver failure')
        detour_start, detour_goal = start.copy(), goal.copy()
        detour_start[1] = detour_goal[1] = .02
        raw = np.array([start, detour_start, detour_goal, goal])
        return dict(raw_positions=raw, positions=raw.copy(), metadata={'method': 'mock_bridge'})


def fixture():
    world = World();planner = Planner(world)
    geometry = np.zeros((4, 7));geometry[:, 0] = [0., .04, .08, .1]
    path = planner.retime_path_smooth(geometry)
    check = planner.validate_path(path, ('mustard_bottle',), 1, phase='above_bowl')
    return world, planner, geometry, path, check


class GuidedTransportTests(unittest.TestCase):
    def call(self, world, planner, geometry, path, check, **kwargs):
        report = {}
        options = dict(dt=planner.dt, apply_tilt_margin=lambda value: value,
                       guide_metadata={'guide_id': 'cpu_test'}, report=report)
        options.update(kwargs)
        trajectory = replan_guided_suffix(planner, world, path, geometry, check,
            path[-1], ('mustard_bottle',), 1, **options)
        return trajectory, report

    def test_failed_guide_prefix_is_rechecked_then_bridged_to_exact_goal(self):
        world, planner, geometry, path, check = fixture()
        self.assertFalse(check['valid'])
        self.assertEqual(check['fcl_samples_checked'], 0)
        original = path.copy();cap_calls = []
        def cap(trajectory):
            cap_calls.append(deepcopy(trajectory))
            return trajectory
        result, report = self.call(world, planner, geometry, path, check, apply_tilt_margin=cap)
        self.assertTrue(report['success'], report)
        first = report['attempts'][0]
        self.assertTrue(first['prefix_validation']['validation_complete'])
        self.assertTrue(report['goal_validation']['validation_complete'])
        self.assertEqual(report['goal_validation']['input_waypoints'], 1)
        self.assertEqual(first['prefix_last_sample'], check['curobo_failures'][0]['edge'])
        self.assertGreater(first['prefix_samples'], first['prefix_geometry_knots'])
        np.testing.assert_array_equal(planner.bridge_calls[0][0], path[first['prefix_last_sample']])
        np.testing.assert_array_equal(planner.bridge_calls[0][1], path[-1])
        np.testing.assert_array_equal(result['positions'][0], path[0])
        np.testing.assert_array_equal(result['positions'][-1], path[-1])
        np.testing.assert_array_equal(path, original)
        self.assertTrue(first['final_validation']['validation_complete'])
        self.assertTrue(first['final_independent_fcl_valid'])
        self.assertEqual(len(cap_calls), 1)
        self.assertLess(len(result['positions']), 3 * len(path))
        json.dumps(report)

    def test_first_bridge_failure_uses_earlier_original_corner_and_keeps_error(self):
        world, planner, geometry, path, check = fixture();planner.bridge_failures = 1
        result, report = self.call(world, planner, geometry, path, check)
        self.assertIsNotNone(result)
        self.assertEqual(len(report['attempts']), 2)
        self.assertIn('Local bridge solver failure', report['attempts'][0]['error'])
        self.assertLess(report['prefix_cut_indices'][1], report['prefix_cut_indices'][0])
        self.assertTrue(any(np.array_equal(planner.bridge_calls[1][0], q) for q in geometry))

    def test_bridge_budget_is_two_and_original_errors_are_recorded(self):
        world, planner, geometry, path, check = fixture();planner.bridge_failures = 9
        result, report = self.call(world, planner, geometry, path, check)
        self.assertIsNone(result)
        self.assertEqual(len(planner.bridge_calls), 2)
        self.assertEqual(len(report['attempts']), 2)
        self.assertTrue(all('Local bridge solver failure' in a['error'] for a in report['attempts']))

    def test_invalid_goal_and_unchecked_prefix_never_reach_bridge_planning(self):
        for reject_goal in (True, False):
            world, planner, geometry, path, check = fixture()
            if reject_goal:
                world.valid = lambda q, ignore=(): q[0] < .099
            else:
                # The original failed cuRobo query did not check this FCL
                # collision in the prefix; the new prefix check must catch it.
                world.valid = lambda q, ignore=(): q[0] > .02
            result, report = self.call(world, planner, geometry, path, check)
            self.assertIsNone(result)
            self.assertEqual(planner.bridge_calls, [])
            self.assertFalse(report['success'])

    def test_tilt_callback_failure_cannot_admit_a_path(self):
        world, planner, geometry, path, check = fixture()
        def cap(trajectory):
            raise RuntimeError('Original cap25 rejection')
        result, report = self.call(world, planner, geometry, path, check, apply_tilt_margin=cap)
        self.assertIsNone(result)
        self.assertEqual(len(report['attempts']), 2)
        self.assertTrue(all('Original cap25 rejection' in a['error'] for a in report['attempts']))

    def test_final_recheck_rejects_geometry_changed_by_tilt_correction(self):
        world, planner, geometry, path, check = fixture()
        def cap(trajectory):
            trajectory['positions'] = path.copy()
            return trajectory
        result, report = self.call(world, planner, geometry, path, check, apply_tilt_margin=cap)
        self.assertIsNone(result)
        self.assertTrue(all(not a['final_validation']['valid'] for a in report['attempts']))

    def test_tilt_correction_may_not_change_actual_endpoints(self):
        world, planner, geometry, path, check = fixture()
        def cap(trajectory):
            trajectory['positions'][0, 0] += .001
            return trajectory
        result, report = self.call(world, planner, geometry, path, check, apply_tilt_margin=cap)
        self.assertIsNone(result)
        self.assertTrue(all(not a['start_exactly_unchanged'] for a in report['attempts']))

    def test_phase_anchor_requires_both_original_path_and_geometry_prefix(self):
        world, planner, geometry, path, check = fixture()
        def fk(q):
            pose = np.eye(4);pose[:3, 3] = q[:3];return pose
        guide = {'guide_id': 'source_declared', 'phase_boundaries': [{
            'id': 'arrival', 'source_anchor_q': geometry[1].tolist(),
            'guide_internal_index': 0, 'source_state_index': 1,
            'before_phase': 'transfer', 'after_phase': 'above_bowl', 'constraint_phase': 'above_bowl'}]}
        trajectory, _ = self.call(world, planner, geometry, path, check, guide_metadata=guide)
        mapped = resolve_guided_transport_phases(trajectory, SimpleNamespace(fk=fk), np.eye(4))
        self.assertTrue(mapped['applied'], mapped)
        for lost in ('path', 'geometry'):
            altered = deepcopy(trajectory)
            evidence = altered['metadata']['guided_suffix_replanning']
            if lost == 'path':
                evidence['selected_prefix_last_sample'] = 0
            else:
                evidence['retained_source_geometry_indices'] = [0]
            rejected = resolve_guided_transport_phases(altered, SimpleNamespace(fk=fk), np.eye(4))
            self.assertFalse(rejected['applied'])
            self.assertEqual(rejected['reason'], 'source_phase_anchor_not_in_retained_prefix')
            self.assertNotIn('phase_boundaries', altered['metadata']['guide'])
            self.assertTrue(rejected['inactive_source_phase_boundaries'])

    def test_real_vectorized_lower_first_flow_calls_bridge_only_for_supported_human(self):
        for variant, supported in [('human', True), ('human', False), ('robot', True)]:
            with self.subTest(variant=variant, supported=supported):
                self.run_collector(variant, supported)

    def test_real_single_collector_lower_first_flow_uses_same_fallback_and_scope(self):
        for variant, supported in [('human', True), ('human', False), ('robot', True)]:
            with self.subTest(variant=variant, supported=supported):
                self.run_collector(variant, supported, single=True)

    def test_real_collectors_record_source_phases_and_probe_both_actual_segments(self):
        for single in (False, True):
            with self.subTest(single=single):
                self.run_collector('human', True, single=single, with_phases=True)

    def test_real_collectors_stop_invalid_start_before_candidate_search(self):
        for single in (False, True):
            with self.subTest(single=single):
                self.run_collector('human', True, single=single, invalid_start=True)

    def test_real_collectors_skip_only_invalid_goal_then_keep_original_fallback(self):
        for single in (False, True):
            with self.subTest(single=single):
                self.run_collector('human', True, single=single, prepend_invalid_goal=True)

    def test_vectorized_collector_replans_when_an_internal_hint_is_now_in_collision(self):
        self.run_collector('human', True, invalid_internal_hint=True)

    def run_collector(self, variant, supported, single=False, with_phases=False,
                      invalid_start=False, prepend_invalid_goal=False,
                      invalid_internal_hint=False):
        world, planner, geometry, _, _ = fixture();planner.reject_initial_plan = True
        if with_phases:
            geometry = np.zeros((6, 7))
            geometry[:, :2] = [[0., 0.], [0., .02], [.04, .02], [.04, 0.], [.08, 0.], [.1, 0.]]
            def valid(q, ignore=(), obstacle_filter=None):
                collision = 'person' not in ignore and .018 < q[0] < .032 and abs(q[1]) < .005
                world.last_clearance = {'obstacle': 'person', 'valid': not collision,
                                        'distance_m': 0. if collision else .1, 'required_m': .04}
                return not collision
            world.valid = valid
            world._matches = lambda name, key: name == key
            planner.motion_gen.ik_solver.check_valid = lambda samples: CpuResult(
                ~((samples[:, 0] > .058) & (samples[:, 0] < .085) & (np.abs(samples[:, 1]) < .005)))
        if invalid_start:
            world.valid = lambda q, ignore=(): q[0] > .001
        planner.validate_path = Mock(wraps=planner.validate_path)
        original_plan = world.plan
        if invalid_internal_hint:
            world.valid = lambda q, ignore=(): not (.079 < q[0] < .081 and abs(q[1]) < .005)
            def connect(start, goal, ignore, **kwargs):
                if kwargs.get('max_iterations') == 300 and not world.valid(goal, ignore):
                    raise RuntimeError('Guide waypoint is in the newly sampled person')
                return original_plan(start, goal, ignore, **kwargs)
            world.plan = Mock(side_effect=connect)
        else:
            world.plan = Mock(wraps=world.plan)
        def fk(q):
            pose = np.eye(4);pose[:3, 3] = q[:3];return pose
        def ik(pose, guess, **kwargs):
            result = guess.copy();result[:3] = pose[:3, 3];return result
        c = VectorizedEpisode.__new__(VectorizedEpisode)
        c.args = SimpleNamespace(variant=variant, supported_release=supported, seed=7,
            obstacle_id='human_compact_sport_palm_down', grasp_tilt_deg=35.,
            transport_tilt_cap_deg=25., max_steps=0)
        c.world, c.kin, c.motion_planner = world, SimpleNamespace(fk=fk, ik=ik), planner
        world.kin = c.kin
        c.ids, c.q, c.grip, c.step_dt, c.phase = np.arange(7), geometry[0].copy(), 1, planner.dt, 'above_bowl'
        c.robot = SimpleNamespace(joint_names=[str(i) for i in range(7)] + ['finger_joint']*6)
        state = dict(q=np.zeros(13), mustard=np.r_[np.zeros(3), [0., 0., 0., 1.], np.zeros(6)],
                     bowl=np.r_[geometry[-1, :3], [0., 0., 0., 1.], np.zeros(6)], tcp=np.zeros(3))
        c.snapshot = lambda: deepcopy(state)
        c._replace_motion_planner = lambda **kwargs: None
        c._save_placement_progress = lambda value: None
        c.result = {'requested_grasp': {'tcp_heights_world_m': {}}}
        c.settings, c.plan_records, c.episode_spec = {}, [], {} if with_phases else None
        if with_phases:
            c.episode_spec['scope'] = 'synthetic flow test'
        c.obstruction_probes, c.timings = [], {'obstruction_probe_s': 0.}
        c.records = [('lift', np.r_[geometry[0], 1], state, state)]
        candidate = dict(above_q=geometry[-1], lower_q=geometry[-1],
            lower_positions=np.array([geometry[-1], geometry[-1]]), above_pose=fk(geometry[-1]),
            lower_pose=fk(geometry[-1]), metadata={'supported_release': {}}, tilt=35., yaw=30.)
        candidates = [candidate]
        if prepend_invalid_goal:
            rejected = deepcopy(candidate)
            rejected['above_q'][0] = .07
            candidates.insert(0, rejected)
        guides = [{'positions': geometry[1:-1], 'metadata': {'guide_id': 'cpu_test'}}]
        if with_phases:
            guides[0]['metadata']['phase_boundaries'] = [{
                'id': 'arrival', 'source_anchor_q': geometry[3].tolist(),
                'guide_internal_index': 2, 'source_state_index': 3,
                'before_phase': 'transfer', 'after_phase': 'above_bowl', 'constraint_phase': 'above_bowl'}]
        probe = Mock(wraps=probe_obstruction)
        def advance(command, grip, phase):
            before = deepcopy(state)
            state['q'][:7] = command
            pose = fk(command)
            state['tcp'] = pose[:3, 3].copy()
            state['mustard'][:3] = (pose @ world.attachment)[:3, 3]
            c.records.append((phase, np.r_[command, grip].astype(np.float32), before, deepcopy(state)))
        with patch('kitchen_safety.placement_planner.iter_placement_candidates', return_value=iter(candidates)) as iterator, \
             patch('kitchen_safety.placement_planner.get_transport_guides', return_value=guides) as guide_lookup, \
             patch('kitchen_safety.vectorized_episode.probe_obstruction', probe), \
             contextlib.redirect_stdout(io.StringIO()):
            gen = None
            if single:
                # Compile the actual nested function without executing the
                # collector's Kit startup. Its body and imported helper remain
                # production code; only its simulator/planner dependencies vary.
                source = Path(__file__).resolve().parents[1] / 'scripts/collect_kitchen.py'
                node = next(node for node in ast.walk(ast.parse(source.read_text()))
                            if isinstance(node, ast.FunctionDef) and node.name == 'lower_first_transport')
                module = ast.Module(body=[deepcopy(node)], type_ignores=[])
                def step(command, grip):
                    advance(command, grip, namespace['phase'])
                namespace = dict(np=np, Rotation=Rotation, json=json, time=time,
                    q=c.q, motion_planner=planner, snapshot=c.snapshot,
                    kin=c.kin, world=world, robot=c.robot, ids=c.ids,
                    args=c.args, new_motion_planner=lambda: planner,
                    result=c.result, e=SimpleNamespace(step_dt=c.step_dt),
                    grip=c.grip, phase=c.phase, records=c.records,
                    episode_spec=c.episode_spec, settings={}, plan_records=c.plan_records,
                    check_planned_path=check_planned_path, step=step,
                    probe_obstruction=probe, obstruction_probes=c.obstruction_probes, timings=c.timings)
                exec(compile(module, str(source), 'exec'), namespace)
                directory = tempfile.TemporaryDirectory()
                self.addCleanup(directory.cleanup)
                namespace['out'] = Path(directory.name)
                run = lambda: namespace['lower_first_transport'](np.array([.1, 0., -.03]))
            else:
                gen = c._lower_first_transport(np.array([.1, 0., -.03]))
                run = lambda: next(gen)
            if invalid_start:
                with self.assertRaisesRegex(RuntimeError, 'Actual transport start failed'):
                    run()
                placement = c.result['placement']
                self.assertEqual(placement['stop_reason'], 'transport_start_invalid')
                self.assertFalse(placement['transport_start_validation']['valid'])
                self.assertEqual(placement['candidates'], [])
                iterator.assert_not_called()
                guide_lookup.assert_not_called()
                world.plan.assert_not_called()
                self.assertEqual(planner.bridge_calls, [])
                self.assertEqual(planner.validate_path.call_count, 1)
            elif variant == 'human' and supported:
                value = run()
                if not single:
                    self.assertEqual(value.shape, (8,))
                self.assertEqual(c.result['placement']['selected_candidate'], int(prepend_invalid_goal))
                self.assertEqual(c.plan_records[0]['method'], 'guided_prefix_curobo_bridge_body_upright_repair')
                self.assertEqual(c.plan_records[0]['transport_tilt_cap_deg'], 25.)
                self.assertEqual(len(planner.bridge_calls), 2)  # Original query, then local bridge.
                if invalid_internal_hint:
                    attempts = c.result['placement']['candidates'][0]['attempts']
                    bridge = next(a for a in attempts if a.get('method') == 'guided_prefix_curobo_bridge')
                    self.assertIn('newly sampled person', bridge['guide_connection_error'])
                    self.assertTrue(bridge['success'])
                    self.assertFalse(any(call.kwargs.get('max_iterations') == 1000
                                         for call in world.plan.call_args_list))
                if prepend_invalid_goal:
                    rejected, accepted = c.result['placement']['candidates']
                    self.assertEqual(rejected['rejection_reason'], 'transport_goal_invalid')
                    self.assertFalse(rejected['transport_goal_validation']['valid'])
                    self.assertNotIn('lower_validation', rejected)
                    self.assertNotIn('retreat_validation', rejected)
                    self.assertEqual(rejected['attempts'], [])
                    self.assertTrue(accepted['transport_goal_validation']['valid'])
                    self.assertEqual(guide_lookup.call_count, 1)
                    self.assertFalse(any(call.kwargs.get('max_iterations') == 1000
                                         for call in world.plan.call_args_list))
                if with_phases:
                    if not single:
                        while True:
                            advance(value[:7], value[7], c.phase)
                            try:
                                value = next(gen)
                            except StopIteration:
                                break
                    placement = c.result['placement']
                    mapping = placement['transport_phase_mapping']
                    self.assertTrue(mapping['applied'], mapping)
                    boundary = mapping['boundaries'][0]['path_index']
                    final = np.asarray(placement['selected_transport']['positions'])
                    segments = placement['execution_phase_segments']
                    phases = [row[0] for row in c.records[1:]]
                    self.assertEqual(phases[:segments[0]['command_end_index']],
                                     ['transfer'] * segments[0]['command_end_index'])
                    self.assertEqual(phases[segments[1]['command_start_index']:],
                                     ['above_bowl'] * (segments[1]['command_end_index']-segments[1]['command_start_index']+12))
                    np.testing.assert_array_equal(
                        c.records[segments[0]['command_end_index']][1][:7], final[boundary].astype(np.float32))
                    self.assertEqual([p['phase'] for p in c.plan_records], ['transfer', 'above_bowl'])
                    self.assertEqual(probe.call_count, 3)
                    self.assertFalse(c.obstruction_probes[0]['confirmed'])
                    self.assertTrue(c.obstruction_probes[1]['confirmed'])
                    self.assertEqual([p['phase'] for p in c.obstruction_probes],
                                     ['above_bowl', 'transfer', 'above_bowl'])
                    for call, endpoints in zip(probe.call_args_list,
                            [(final[0], final[-1]), (final[0], final[boundary]), (final[boundary], final[-1])]):
                        np.testing.assert_array_equal(call.args[2], endpoints[0])
                        np.testing.assert_array_equal(call.args[3], endpoints[1])
                        self.assertEqual(call.args[4:], (('mustard_bottle',), 1, 'above_bowl', 'person'))
            else:
                with self.assertRaisesRegex(RuntimeError, 'no complete transport'):
                    run()
                self.assertEqual(len(planner.bridge_calls), 1)
            placement = c.result['placement']
            if variant == 'human':
                calls = planner.validate_path.call_args_list
                start_calls = [call for call in calls if np.asarray(call.args[0]).shape == (1, 7)
                               and np.array_equal(call.args[0][0], geometry[0])]
                self.assertEqual(len(start_calls), 1)
                first = calls[0]
                self.assertEqual(np.asarray(first.args[0]).shape, (1, 7))
                self.assertEqual(first.args[1:], (('mustard_bottle',), 1))
                self.assertEqual(first.kwargs, {'phase': 'above_bowl'})
                np.testing.assert_array_equal(first.args[0][0], geometry[0])
                self.assertEqual(placement['transport_start_validation']['transport_object_padding_m'], .015)
                if not invalid_start:
                    self.assertEqual(calls[1].args[0].shape, (1, 7))
                    np.testing.assert_array_equal(calls[1].args[0][0], candidates[0]['above_q'])
                    self.assertEqual(calls[1].args[1:], (('mustard_bottle',), 1))
                    self.assertEqual(calls[1].kwargs, {'phase': 'above_bowl'})
                    self.assertTrue(any(len(call.args[0]) > 2 for call in calls))
            else:
                self.assertNotIn('transport_start_validation', placement)
                self.assertTrue(all('transport_goal_validation' not in trial for trial in placement['candidates']))
                self.assertTrue(all(len(call.args[0]) > 1 for call in planner.validate_path.call_args_list))
            if gen is not None:
                gen.close()


if __name__ == '__main__':
    unittest.main()
