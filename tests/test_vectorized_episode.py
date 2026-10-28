"""CPU integration of per-env control flow, with an explicit ideal plant.

These tests do not claim physical success in Isaac. They exercise the actual
phase/planning generator and before/after recording without importing Kit or
cuRobo, while separate simulator regressions assess physical performance.
"""
from copy import deepcopy
import contextlib
import gc
import io
import unittest
from unittest.mock import patch
import weakref

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.vectorized_episode import VectorizedEpisode


JOINTS = [f'joint_{i}' for i in range(7)] + ['finger_joint'] + [f'finger_{i}' for i in range(5)]
GEOMETRY = {'safe_contact_z_range_m': [-.03, .03],
            'default_tcp_point_bottle_local_m': [0., 0., 0.]}


class Kinematics:
    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        pose[:3, :3] = Rotation.from_rotvec(q[3:6]).as_matrix()
        return pose


class World:
    def __init__(self):
        self.attachment = None
        self.gripper_joint_position = 0.
        self.last_collision = None
        self.last_clearance = None
        self.pose_updates = []
        self.clearance_queries = []

    def edge(self, start, goal, ignore):
        return True

    def valid(self, q, ignore=()):
        return True

    def set_obstacle_pose(self, key, pose):
        self.pose_updates.append((key, np.array(pose).copy()))

    def clearance(self, *args, **kwargs):
        self.clearance_queries.append(deepcopy((args, kwargs)))
        return dict(distance_m=.05, valid=True)


class Contacts:
    def __init__(self, close=True, carried=True):
        self.close, self.carried = close, carried
        self.records = []

    def sample(self, phase, action_index, pose):
        self.records.append(dict(phase=phase, action_index=action_index))

    def summarize(self, band, phase):
        return dict(both_fingers_near_body=self.close, both_fingers_on_body=True)

    def carried_region(self, band):
        return dict(passed=self.carried, outside_body_contacts=[] if self.carried else [
            dict(phase='above_bowl', action_index=1)])


class Planner:
    """A measured-state Cartesian solver; never consumes a saved trajectory."""
    def __init__(self, queries, **kwargs):
        self.queries = queries

    def plan_pose(self, start, target, ignore, grip, phase):
        self.queries.append(dict(start=start.copy(), target=target.copy(), phase=phase))
        goal = start.copy()
        goal[:3] = target[:3, 3]
        goal[3:6] = Rotation.from_matrix(target[:3, :3]).as_rotvec()
        return dict(positions=np.array([start, goal]), metadata={})


def initial_state(x=0.):
    state = dict(q=np.zeros(13), dq=np.zeros(13), tcp=np.array([x, 0., 1.3]),
        tcp_quat_xyzw=np.array([0., 0., 0., 1.]),
        robot_body_pose_w=np.zeros((2, 7)), robot_contact_forces=np.zeros((2, 3)),
        mustard=np.array([x, 0., 1., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.]),
        bowl=np.array([x+.25, .1, 1., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.]),
        arena_success=False, final_on_destination=False)
    state['q'][:3] = state['tcp']
    return state


def make_episode(env_id=0, options=None, contacts=None, layout=None):
    queries = []
    controller = VectorizedEpisode(env_id, layout or {'planning': {}}, options or {},
        World(), Kinematics(), contacts or Contacts(), arm_joint_ids=range(7),
        joint_names=JOINTS, nonfinger_ids=[0], grasp_geometry=GEOMETRY,
        planner_factory=lambda *args, **kwargs: Planner(queries, **kwargs))
    return controller, queries


def ideal_step(controller, state, action):
    """Return a post-action measurement, independent from controller records."""
    result = deepcopy(state)
    result['q'][:7] = action[:7]
    result['q'][7:] = .2 * action[-1]
    pose = controller.kin.fk(result['q'][:7])
    result['tcp'] = pose[:3, 3]
    result['tcp_quat_xyzw'] = Rotation.from_matrix(pose[:3, :3]).as_quat()
    if controller.world.attachment is not None and action[-1]:
        carried = pose @ controller.world.attachment
        result['mustard'][:3] = carried[:3, 3]
        result['mustard'][3:7] = Rotation.from_matrix(carried[:3, :3]).as_quat()
    elif controller.phase == 'release':
        result['mustard'][:3] = result['bowl'][:3] + [0., 0., .1]
        result['arena_success'] = result['final_on_destination'] = True
    return result


class VectorizedEpisodeTests(unittest.TestCase):
    def test_optional_render_states_preserve_preaction_alignment_and_own_their_arrays(self):
        controller, _ = make_episode()
        state = initial_state()
        shapes = dict(kitchen_root_state=(21, 13), kitchen_joint_pos=(48,),
                      kitchen_joint_vel=(48,), kitchen_link_pose=(87, 7),
                      kitchen_link_velocity=(87, 6), robot_body_velocity=(2, 6),
                      hazard_state=(13,))
        state.update({key: np.ones(shape) for key, shape in shapes.items()})
        for step in range(2):
            action = controller.prepare(state)
            following = ideal_step(controller, state, action)
            for key in shapes:
                following[key] += 1
            controller.observe(following)
            for key in shapes:
                state[key].fill(-99)
            state = following
        for key in shapes:
            np.testing.assert_array_equal(controller.records[0][2][key], np.ones(shapes[key]))
            np.testing.assert_array_equal(controller.records[0][3][key], controller.records[1][2][key])
            np.testing.assert_array_equal(controller.records[1][3][key], 3 * np.ones(shapes[key]))

    def test_phase_reuse_keeps_one_context_and_refreshes_measured_carry_state(self):
        controller, queries = make_episode(options={'reuse_phase_planner': True},
            layout={'planning': {'transfer_mode_by_variant': {'base': 'goal'}}})
        constructed, reset_calls = [], []
        class Reusable(Planner):
            def reset_for_new_phase(inner, ignore, grip, phase, hazard_key=None):
                reset_calls.append(dict(ignore=ignore, grip=grip, phase=phase,
                                        attachment=controller.world.attachment.copy(),
                                        measured_grip=controller.world.gripper_joint_position))
                return True
        def factory(*args, **kwargs):
            planner = Reusable(queries, **kwargs)
            constructed.append(planner)
            return planner
        controller.planner_factory = factory
        state = initial_state()
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(400):
                action = controller.prepare(state)
                if action is None:
                    break
                state = ideal_step(controller, state, action)
                controller.observe(state)
        self.assertTrue(controller.result['success'], controller.result)
        self.assertEqual(len(constructed), 1)
        self.assertEqual(len(reset_calls), 1)
        self.assertEqual(reset_calls[0]['phase'], 'above_bowl')
        self.assertEqual(reset_calls[0]['grip'], 1)
        self.assertAlmostEqual(reset_calls[0]['measured_grip'], .2, places=7)
        self.assertEqual(reset_calls[0]['attachment'].shape, (4, 4))
        self.assertEqual(controller.result['transport_planner_context'], 'reused')
        self.assertEqual([event['mode'] for event in controller.result['planner_contexts']['events']], ['fresh', 'reused'])
        self.assertEqual([call['phase'] for call in queries],
                         ['pregrasp', 'approach', 'lift', 'above_bowl', 'lower', 'lower', 'retreat'])

    def test_unsupported_phase_reuse_forces_fresh_with_original_settings_and_graph_opt_in(self):
        controller, _ = make_episode(options={'reuse_phase_planner': True, 'use_cuda_graph': True})
        controller._set_state(initial_state())
        class Context:
            def reset_for_new_phase(self, *args, **kwargs):
                return False
        controller.motion_planner = Context()
        old = weakref.ref(controller.motion_planner)
        calls = []
        def factory(*args, **kwargs):
            self.assertIsNone(old())
            calls.append(kwargs)
            return Context()
        controller.planner_factory = factory
        controller._replace_motion_planner(reason='lower_first_transport', allow_reuse=True, phase='above_bowl')
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0]['use_cuda_graph'])
        self.assertTrue(calls[0]['warmup'])
        event = controller.planner_context_events[-1]
        self.assertEqual(event['mode'], 'fresh')
        self.assertEqual(event['fallback_reason'], 'unsupported_dynamic_hazard_geometry_or_topology')

    def test_planner_replacement_drops_old_context_and_cycles_before_same_factory(self):
        class CollisionCostCycle:
            def __init__(self):
                # Same ownership pattern as cuRobo PrimitiveCollisionCost.
                self.forward = self.discrete_fn
            def discrete_fn(self):
                return None
        class Context:
            def __init__(self):
                self.cost = CollisionCostCycle()
        layout = {'planning': {'max_joint_speed_rad_s': .27,
                    'transport_object_padding_m_by_variant': {'property': .004}},
                  'obstacle': {'category': 'property', 'static': False}}
        controller, _ = make_episode(options={'obstacle_id': 'property_tall_glass', 'seed': 73}, layout=layout)
        state = initial_state()
        state['hazard_pose'] = np.array([2., .1, 1., 0., 0., 0., 1.])
        controller._set_state(state)
        released, calls = [], []
        def factory(world, kin, **kwargs):
            self.assertIsNone(controller.motion_planner)
            for reference in released:
                self.assertIsNone(reference(), 'Old planner/cycle is alive inside the new constructor')
            calls.append((world, kin, kwargs))
            return Context()
        controller.planner_factory = factory
        with patch('kitchen_safety.vectorized_episode.gc.collect', wraps=gc.collect) as collect:
            controller._replace_motion_planner()
            collect.assert_not_called()
            released.extend((weakref.ref(controller.motion_planner), weakref.ref(controller.motion_planner.cost)))
            state['hazard_pose'][0] += .01
            controller._set_state(state)
            controller._replace_motion_planner()
            collect.assert_called_once_with()
        self.assertEqual(len(calls), 2)
        for world, kin, kwargs in calls:
            self.assertIs(world, controller.world)
            self.assertIs(kin, controller.kin)
            self.assertEqual(kwargs, dict(dt=1/15, seed=73, max_joint_speed=.27, transport_object_padding_m=.004))
        np.testing.assert_array_equal(controller.planner_hazard_pose, state['hazard_pose'])

    def test_replacement_factory_failure_still_fails_only_the_current_episode(self):
        controller, _ = make_episode()
        other, _ = make_episode(1)
        class Context:
            pass
        controller.motion_planner = Context()
        previous = weakref.ref(controller.motion_planner)
        def factory(*args, **kwargs):
            self.assertIsNone(previous())
            self.assertIsNone(controller.motion_planner)
            raise RuntimeError('fresh planner allocation failed')
        controller.planner_factory = factory
        state = initial_state()
        for _ in range(31):
            action = controller.prepare(state)
            if action is None:
                break
            controller.observe(state)
        self.assertTrue(controller.done)
        self.assertFalse(controller.result['success'])
        self.assertIn('fresh planner allocation failed', controller.result['error'])
        self.assertEqual(len(controller.records), 30)
        self.assertIsNone(controller.motion_planner)
        self.assertIsNotNone(other.prepare(initial_state(.4)))
        self.assertFalse(other.done)

    def test_two_live_plans_have_isolated_state_and_complete_phase_records(self):
        controllers, queries = zip(*(make_episode(i) for i in range(2)))
        states = [initial_state(0.), initial_state(.4)]
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(400):
                actions = [c.prepare(s) for c, s in zip(controllers, states)]
                if all(a is None for a in actions):
                    break
                # Exactly one outside "physics" boundary supplies all states.
                for i, action in enumerate(actions):
                    if action is not None:
                        states[i] = ideal_step(controllers[i], states[i], action)
                        controllers[i].observe(states[i])
            else:
                self.fail('Controllers did not complete their finite phase sequences')
        expected = ['settle', 'pregrasp', 'approach', 'close', 'lift',
                    'transfer_front', 'transfer', 'above_bowl', 'lower',
                    'release', 'retreat', 'settle']
        for controller, calls in zip(controllers, queries):
            self.assertTrue(controller.done)
            self.assertTrue(controller.result['success'], controller.result)
            phases = [row[0] for i, row in enumerate(controller.records)
                      if i == 0 or row[0] != controller.records[i-1][0]]
            self.assertEqual(phases, expected)
            self.assertEqual(controller.records[0][0], 'settle')
            for previous, following in zip(controller.records, controller.records[1:]):
                for key in previous[3]:
                    np.testing.assert_array_equal(previous[3][key], following[2][key])
            self.assertEqual([call['phase'] for call in calls],
                             ['pregrasp', 'approach', 'lift', 'transfer_front',
                              'transfer', 'above_bowl', 'lower', 'retreat'])
            for call in calls:
                first = next(row for row in controller.records if row[0] == call['phase'])
                np.testing.assert_array_equal(call['start'], first[2]['q'][:7])
        self.assertAlmostEqual(queries[1][0]['target'][0, 3] - queries[0][0]['target'][0, 3], .4)
        self.assertFalse(np.shares_memory(controllers[0].records[0][1], controllers[1].records[0][1]))

    def test_action_measurement_handshake_copies_inputs_and_failure_is_local(self):
        first, _ = make_episode(0)
        second, _ = make_episode(1)
        initial = initial_state()
        action = first.prepare(initial)
        expected = action.copy()
        initial['q'][:] = 42.
        action[:] = 99.
        with self.assertRaisesRegex(RuntimeError, 'observe'):
            first.prepare(initial_state())
        first.observe(initial_state())
        np.testing.assert_array_equal(first.records[0][1], expected)
        self.assertNotEqual(first.records[0][2]['q'][0], 42.)
        first.fail(RuntimeError('single environment failed'))
        self.assertIsNone(first.prepare(initial_state()))
        self.assertIsNotNone(second.prepare(initial_state(.4)))
        self.assertFalse(second.done)
        self.assertEqual(len(second.records), 0)
        with self.assertRaisesRegex(RuntimeError, 'prepared action'):
            first.observe(initial_state())

    def test_close_failure_prevents_attachment_and_lift(self):
        controller, queries = make_episode(contacts=Contacts(close=False))
        state = initial_state()
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(150):
                action = controller.prepare(state)
                if action is None:
                    break
                state = ideal_step(controller, state, action)
                controller.observe(state)
        self.assertTrue(controller.done)
        self.assertFalse(controller.result['success'])
        self.assertEqual(controller.result['failure_phase'], 'close')
        self.assertIsNone(controller.world.attachment)
        self.assertEqual([q['phase'] for q in queries], ['pregrasp', 'approach'])

    def test_final_support_velocity_finger_force_and_contact_conditions_unchanged(self):
        cases = [('valid', None), ('unsupported', 'final_on_destination'),
                 ('moving', 'final_bottle_speed_m_s'), ('closed', 'final_gripper_abs_position_rad'),
                 ('force', 'max_arm_contact_force_n'), ('contact', None)]
        for case, failed_condition in cases:
            with self.subTest(case=case):
                controller, _ = make_episode(contacts=Contacts(carried=case != 'contact'))
                state = initial_state()
                state['mustard'][:3] = state['bowl'][:3] + [0., 0., .1]
                state['arena_success'] = True
                state['final_on_destination'] = case != 'unsupported'
                if case == 'moving': state['mustard'][7] = .01
                if case == 'closed': state['q'][7] = .05
                if case == 'force': state['robot_contact_forces'][0, 0] = 5.
                controller._set_state(state)
                controller.result['lift_height'] = .1
                snapshot = controller.snapshot()
                controller.records = [('settle', np.zeros(8), snapshot, snapshot)]
                controller._finish_physical_checks()
                controller._finish()
                self.assertEqual(controller.result['success'], case == 'valid')
                if failed_condition:
                    self.assertIn(failed_condition, [r['condition'] for r in controller.result['physical_failure_reasons']])
                if case == 'contact':
                    self.assertEqual(controller.result['failure_phase'], 'above_bowl')

    def test_difficult_preset_and_explicit_overrides_are_retained(self):
        layout = {'planning': {}, 'obstacle': {'category': 'property', 'static': False}}
        controller, _ = make_episode(options={'obstacle_id': 'property_wine_glass'}, layout=layout)
        self.assertEqual(controller.args.transport_tilt_cap_deg, 15.)
        self.assertTrue(controller.args.supported_release)
        override, _ = make_episode(options={'obstacle_id': 'property_wine_glass',
            'transport_tilt_cap_deg': 12., 'supported_release': False}, layout=layout)
        self.assertEqual(override.args.transport_tilt_cap_deg, 12.)
        self.assertFalse(override.args.supported_release)

    def test_deferred_diagnostics_copy_same_states_without_deferring_contacts_or_world_sync(self):
        layout = {'planning': {}, 'obstacle': {'category': 'property', 'static': False}}
        options = {'obstacle_id': 'property_tall_glass'}
        inline, _ = make_episode(options=options, layout=layout)
        deferred, _ = make_episode(options={**options, 'defer_clearance_diagnostics': True}, layout=layout)
        for controller in (inline, deferred):
            controller.safety_enabled = True
            for index, phase in enumerate(('close', 'lift', 'release')):
                state = initial_state()
                state['q'] += index * .001
                state['hazard_pose'] = np.array([2.+index*.001, .1, 1., 0., 0., 0., 1.])
                attachment = np.eye(4) if phase == 'lift' else None
                controller.world.attachment = attachment
                controller.phase = phase
                action = np.r_[state['q'][:7], float(phase != 'release')]
                controller._pending = (phase, action, deepcopy(state))
                controller.observe(state)
                # Simulator buffers and planning attachments may be reused next.
                state['q'][:] = 99.
                state['hazard_pose'][:] = 99.
                if attachment is not None:
                    attachment[:] = 99.
        self.assertFalse(deferred.done)
        self.assertEqual(len(inline.world.clearance_queries), 3)
        self.assertEqual(deferred.world.clearance_queries, [])
        self.assertEqual(inline.grasp_contacts.records, deferred.grasp_contacts.records)
        self.assertEqual([r['phase'] for r in deferred.grasp_contacts.records], ['close', 'lift'])
        self.assertEqual(len(deferred.world.pose_updates), 3)
        self.assertEqual(len(deferred.clearance_buffer), 3)
        for index, (saved, (args, kwargs)) in enumerate(zip(
                deferred.clearance_buffer._states, inline.world.clearance_queries)):
            np.testing.assert_array_equal(saved.q[:7], args[0])
            np.testing.assert_array_equal(saved.mustard, kwargs['object_pose'])
            np.testing.assert_array_equal(saved.hazard_pose, deferred.world.pose_updates[index][1])
            self.assertEqual(saved.action_index, index)
            self.assertEqual(kwargs['mode'], 'execution')
            self.assertEqual(kwargs['obstacle_filter'], ('glass',))
        np.testing.assert_array_equal(deferred.clearance_buffer._states[1].attachment, np.eye(4))
        report = dict(completed=True, samples=3, records=inline.clearance_records,
                      minimum_hazard_clearance_m=.05, timings_seconds={'total_s': .1})
        deferred.finalize_clearance_diagnostics(report)
        self.assertEqual(deferred.clearance_records, inline.clearance_records)
        self.assertEqual(deferred.result['minimum_hazard_clearance_m'], .05)
        self.assertTrue(deferred.result['action_clearance_diagnostics']['completed'])
        with self.assertRaisesRegex(ValueError, 'original diagnostic samples'):
            deferred.finalize_clearance_diagnostics({**report, 'records': report['records'][:-1]})
        self.assertEqual(deferred.clearance_records, inline.clearance_records)

    def test_deferred_mode_still_stops_on_live_contact_read_failure(self):
        class FailedContacts(Contacts):
            def sample(self, *args):
                raise RuntimeError('contact data unavailable')
        layout = {'planning': {}, 'obstacle': {'category': 'property', 'static': False}}
        controller, _ = make_episode(options={'obstacle_id': 'property_tall_glass',
            'defer_clearance_diagnostics': True}, contacts=FailedContacts(), layout=layout)
        state = initial_state()
        state['hazard_pose'] = np.array([2., .1, 1., 0., 0., 0., 1.])
        controller.phase, controller.safety_enabled = 'close', True
        controller._pending = ('close', np.r_[state['q'][:7], 1.], deepcopy(state))
        controller.observe(state)
        self.assertTrue(controller.done)
        self.assertFalse(controller.result['success'])
        self.assertIn('contact data unavailable', controller.result['error'])
        self.assertEqual(len(controller.clearance_buffer), 0)
        self.assertEqual(controller.world.pose_updates, [])


if __name__ == '__main__':
    unittest.main()
