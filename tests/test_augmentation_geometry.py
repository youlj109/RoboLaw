import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.augmentation_geometry import (
    relative_grasp_rotation, probe_obstruction, check_initial_layout, _cartesian_reference,
)


class CurvedKinematics:
    """A continuous nonlinear FK where a joint chord dips below a table plane."""
    def __init__(self):
        self.seeds = []

    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = [q[0], q[1] + q[0]**2, q[3]]
        pose[:3, :3] = Rotation.from_euler('z', q[2]).as_matrix()
        return pose

    def ik(self, target, seed, attempts):
        self.seeds.append(np.array(seed).copy())
        if attempts != 1:
            raise AssertionError('Do not switch to random IK branches')
        q = np.array(seed).copy()
        q[0] = target[0, 3]
        q[1] = target[1, 3] - q[0]**2
        q[2] = Rotation.from_matrix(target[:3, :3]).as_euler('xyz')[2]
        q[3] = target[2, 3]
        return q


class CartesianWorld:
    def __init__(self, kin=None):
        self.kin = kin or CurvedKinematics()
        self.attachment = None
        self.last_clearance = None

    def valid(self, q, ignore, obstacle_filter):
        self.last_clearance = None
        if .08 <= q[0] <= .12:
            self.last_clearance = {'obstacle': '/World/glass/body', 'valid': False,
                                   'distance_m': 0., 'required_m': .06}
            return False
        return True

    def _matches(self, name, key):
        return key in name.split('/')


class CartesianPlanner:
    def __init__(self, world, reject_cartesian=False):
        self.world = world
        self.queries = []
        self.restored = None
        self.reject_cartesian = reject_cartesian

    def validate_path(self, path, ignore, grip, phase):
        self.queries.append(np.array(path))
        # The joint-linear chord hits unrelated geometry. The curved joint
        # solution of the straight Cartesian path is clear of that geometry.
        valid = len(path) > 2 and not self.reject_cartesian
        return dict(valid=valid, samples=len(path), curobo_failures=[] if valid else [0], fcl_failures=[])

    def _prepare_path_query(self, ignore, grip, phase):
        self.restored = (ignore, grip, phase)

    def plan_joint_goal(self, *args, **kwargs):
        raise AssertionError('A direct reference must not call a motion planner')


class GeometryTests(unittest.TestCase):
    def test_human_colliders_are_checked_for_initial_object_intersections(self):
        class Fcl:
            CollisionRequest = object
            CollisionResult = object
            intersecting = False
            def collide(self, a, b, request, result):
                return self.intersecting and {a, b} == {'mustard_bottle', 'person'}
        class World:
            fcl = Fcl()
            objects = [(f'/World/{name}/mesh', name, np.zeros(3), np.ones(3))
                       for name in ('mustard_bottle', 'bowl', 'person')]
            @staticmethod
            def _matches(path, name):
                return name in path.split('/')
            @staticmethod
            def valid(*args, **kwargs):
                return True
        state = {'q': np.zeros(13), 'mustard': [0, 0, 1, 0, 0, 0, 1],
                 'bowl': [1, 0, 1, 0, 0, 0, 1]}
        layout = {'mustard_position': [0, 0, 1], 'bowl_position': [1, 0, 1],
                  'obstacle': {'category': 'human', 'position': [.5, 0, 0], 'static': True}}
        world = World()
        report = check_initial_layout(world, state, layout, list(range(7)))
        self.assertTrue(report['valid'], report)
        self.assertEqual(report['object_pairs_checked'], 3)
        world.fcl.intersecting = True
        report = check_initial_layout(world, state, layout, list(range(7)))
        self.assertFalse(report['valid'])
        self.assertEqual(report['failures'][0]['objects'], ['mustard_bottle', 'person'])

    def test_fallen_dynamic_hazard_is_not_accepted_as_random_layout(self):
        class World:
            objects = []
            def valid(self, *args, **kwargs):
                return True
        state = {'q': np.zeros(13), 'mustard': [0, 0, 1, 0, 0, 0, 1],
                 'bowl': [1, 0, 1, 0, 0, 0, 1], 'hazard_pose': [.5, 0, .7, 0, 0, 0, 1]}
        layout = {'mustard_position': [0, 0, 1], 'bowl_position': [1, 0, 1],
                  'obstacle': {'position': [.5, 0, 1], 'category': 'property'}}
        report = check_initial_layout(World(), state, layout, list(range(7)))
        self.assertFalse(report['valid'])
        self.assertTrue(any(row.get('object') == 'glass' and 'support' in row['reason']
                            for row in report['failures']))

    def test_grasp_rotation_preserves_object_relative_orientation(self):
        grasp = Rotation.from_euler('xyz', [35, 0, -90], degrees=True).as_matrix()
        bottle = Rotation.from_euler('z', 37, degrees=True).as_matrix()
        world = relative_grasp_rotation(grasp, bottle)
        np.testing.assert_allclose(bottle.T @ world, grasp, atol=1e-12)
        np.testing.assert_allclose(world @ world.T, np.eye(3), atol=1e-12)

    def test_probe_rejects_other_constraints_and_restores_query(self):
        class Planner:
            restored = None
            def validate_path(self, path, ignored, grip, phase):
                self.ignored = ignored
                return dict(valid=False, samples=2, curobo_failures=[0], fcl_failures=[])
            def _prepare_path_query(self, ignored, grip, phase):
                self.restored = ignored
            def plan_joint_goal(self, *args, **kwargs):
                raise AssertionError('A non-direct replanned reference must never qualify')
        planner = Planner()
        report = probe_obstruction(None, planner, np.zeros(7), np.ones(7), ('mustard_bottle',),
                                   1, 'above_bowl', 'glass')
        self.assertFalse(report['confirmed'])
        self.assertEqual(planner.ignored, ('mustard_bottle', 'glass'))
        self.assertEqual(planner.restored, ('mustard_bottle',))

    def test_probe_requires_target_collision_not_other_failure(self):
        class Planner:
            def validate_path(self, *args, **kwargs):
                return dict(valid=True, samples=2)
            def _prepare_path_query(self, *args):
                pass
        class World:
            last_clearance = None
            def valid(self, *args, **kwargs):
                # E.g. a carried tilt violation has no target pair measurement.
                return False
        report = probe_obstruction(World(), Planner(), np.zeros(7), np.ones(7) * .03,
                                   (), 1, 'above_bowl', 'glass')
        self.assertFalse(report['confirmed'])

    def test_direct_reference_target_hit_is_confirmed_and_masks_restored(self):
        class Planner:
            queries = 0
            restored = None
            def validate_path(self, *args, **kwargs):
                self.queries += 1
                self.reference_ignore = args[1]
                return dict(valid=True, samples=3, curobo_failures=[], fcl_failures=[])
            def plan_joint_goal(self, start, goal, ignore, grip, phase):
                raise AssertionError('Reference checking must not plan a detour')
            def _prepare_path_query(self, ignore, grip, phase):
                self.restored = ignore
        class World:
            last_clearance = None
            def valid(self, q, ignore, obstacle_filter):
                self.last_clearance = {'obstacle': '/World/glass/body', 'valid': False}
                return False
            def _matches(self, name, key):
                return key in name.split('/')
        planner = Planner()
        report = probe_obstruction(World(), planner, np.zeros(7), np.ones(7)*.03,
                                   ('mustard_bottle',), 1, 'above_bowl', 'glass')
        self.assertTrue(report['confirmed'])
        self.assertEqual(planner.queries, 1)
        self.assertEqual(planner.reference_ignore, ('mustard_bottle', 'glass'))
        self.assertEqual(planner.restored, ('mustard_bottle',))
        self.assertEqual(report['reference'], 'joint_linear')
        self.assertFalse(report['endpoint_provenance']['post_execution_measured_endpoints'])
        self.assertIn('planned goal', report['endpoint_provenance']['goal'])

    def test_cartesian_fallback_preserves_exact_joints_and_rechecks_constraints(self):
        world = CartesianWorld()
        planner = CartesianPlanner(world)
        start, goal = np.zeros(7), np.array([.2, -.04, 0., 0., 0., 0., 0.])
        report = probe_obstruction(world, planner, start, goal, ('mustard_bottle',),
                                   1, 'above_bowl', 'glass')
        self.assertTrue(report['confirmed'])
        self.assertEqual(report['reference'], 'tcp_cartesian_linear')
        self.assertEqual(len(report['reference_attempts']), 2)
        self.assertEqual(len(planner.queries), 2)
        np.testing.assert_array_equal(planner.queries[-1][0], start)
        np.testing.assert_array_equal(planner.queries[-1][-1], goal)
        self.assertEqual(planner.restored, (('mustard_bottle',), 1, 'above_bowl'))
        self.assertLess(report['directness']['maximum_position_error_m'], .002)
        self.assertEqual(report['directness']['start_joint_error_rad'], 0.)
        self.assertEqual(report['directness']['goal_joint_error_rad'], 0.)
        for seed in world.kin.seeds:
            fraction = seed[0] / goal[0]
            np.testing.assert_allclose(seed, start * (1-fraction) + goal * fraction)

    def test_cartesian_reference_cannot_bypass_full_curobo_rejection(self):
        world = CartesianWorld()
        planner = CartesianPlanner(world, reject_cartesian=True)
        report = probe_obstruction(world, planner, np.zeros(7),
                                   np.array([.2, -.04, 0., 0., 0., 0., 0.]),
                                   (), 1, 'above_bowl', 'glass')
        self.assertFalse(report['confirmed'])
        self.assertEqual(len(planner.queries), 2)
        self.assertFalse(report['reference_attempts'][-1]['reference_valid_without_hazard'])
        self.assertEqual(planner.restored, ((), 1, 'above_bowl'))

    def test_cartesian_uses_measured_attachment_and_reports_bottle_frame(self):
        world = CartesianWorld()
        world.attachment = np.eye(4)
        world.attachment[:3, 3] = [.03, .02, .01]
        world.attachment[:3, :3] = Rotation.from_euler('x', .2).as_matrix()
        start, goal = np.zeros(7), np.array([.2, -.04, .4, 0., 0., 0., 0.])
        path, report = _cartesian_reference(world, CartesianPlanner(world), start, goal)
        self.assertIsNotNone(path)
        self.assertTrue(report['direct_connection_valid'])
        self.assertEqual(report['reference'], 'bottle_cartesian_linear')
        np.testing.assert_allclose(report['directness']['start_pose'], world.kin.fk(start) @ world.attachment)
        np.testing.assert_allclose(report['directness']['goal_pose'], world.kin.fk(goal) @ world.attachment)
        np.testing.assert_array_equal(path[0], start)
        np.testing.assert_array_equal(path[-1], goal)

    def test_cartesian_chord_deviation_is_rejected_before_collision_validation(self):
        class BadKinematics(CurvedKinematics):
            def ik(self, target, seed, attempts):
                q = super().ik(target, seed, attempts)
                q[1] += .01
                return q
        world = CartesianWorld(BadKinematics())
        planner = CartesianPlanner(world)
        report = probe_obstruction(world, planner, np.zeros(7),
                                   np.array([.2, -.04, 0., 0., 0., 0., 0.]),
                                   (), 1, 'above_bowl', 'glass')
        self.assertFalse(report['confirmed'])
        self.assertEqual(len(planner.queries), 1)
        rejected = report['reference_attempts'][-1]
        self.assertFalse(rejected['direct_connection_valid'])
        self.assertGreater(rejected['directness']['maximum_position_error_m'], .002)
        self.assertEqual(planner.restored, ((), 1, 'above_bowl'))

    def test_cartesian_ik_failure_is_recorded_and_query_restored(self):
        class UnreachableKinematics(CurvedKinematics):
            def ik(self, *args, **kwargs):
                raise RuntimeError('unreachable pose')
        world = CartesianWorld(UnreachableKinematics())
        planner = CartesianPlanner(world)
        report = probe_obstruction(world, planner, np.zeros(7),
                                   np.array([.2, -.04, 0., 0., 0., 0., 0.]),
                                   (), 1, 'above_bowl', 'glass')
        self.assertFalse(report['confirmed'])
        self.assertEqual(report['reference_attempts'][-1]['reference_error'], 'unreachable pose')
        self.assertEqual(planner.restored, ((), 1, 'above_bowl'))


if __name__ == '__main__':
    unittest.main()
