"""CPU tests of fixed-shape planner updates; never import CUDA/Kit."""
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

from kitchen_safety.curobo_backend import CuroboPlanner


def fake_motion_gen():
    box = SimpleNamespace(pose=None, dims=None)
    return SimpleNamespace(
        reset=Mock(), warmup=Mock(), update_locked_joints=Mock(),
        attach_spheres_to_robot=Mock(), detach_spheres_from_robot=Mock(),
        finetune_trajopt_solver=SimpleNamespace(reset_seed=Mock()),
        finetune_js_trajopt_solver=SimpleNamespace(reset_seed=Mock()),
        world_coll_checker=SimpleNamespace(
            world_model=SimpleNamespace(get_obstacle=lambda name: box),
            update_obb_pose=Mock(), update_obb_dims=Mock(), enable_obstacle=Mock()))


class ReusedContextTests(unittest.TestCase):
    def setUp(self):
        self.planner = p = CuroboPlanner.__new__(CuroboPlanner)
        p.context_timings, p.context_calls = {}, {}
        p.use_cuda_graph, p.seed = False, 17
        p._warmed_up = p._context_reused = False
        p._query_signature = p._last_grip = None
        p.T_B_W = p.T_ee_tcp = np.eye(4)
        p.transport_object_padding_m = .01
        p.robot_dict = {'kinematics': {'lock_joints': {}}}
        p.obstacle_names = ['/World/glass/collider', '/World/mustard_bottle/collider']
        p._dynamic_box_names = {p.obstacle_names[0]}
        p.world = SimpleNamespace(
            attachment=np.eye(4), gripper_joint_position=.371,
            objects=[(p.obstacle_names[0], None, np.array([1., 2., 3.]), np.array([2., 4., 6.]))],
            _matches=lambda name, key: '/' + key + '/' in name,
            _margin=lambda name, mode: .06)
        p.args = SimpleNamespace(to_device=lambda values: np.array(values, dtype=np.float32))
        p.motion_gen = fake_motion_gen()
        module_patch = patch.dict(sys.modules, {'curobo.types.math': SimpleNamespace(
            Pose=SimpleNamespace(from_list=lambda pose, args: pose))})
        module_patch.start()
        self.addCleanup(module_patch.stop)

    def test_phase_reset_refreshes_geometry_gripper_attachment_and_all_solver_seeds(self):
        p = self.planner
        self.assertTrue(p.reset_for_new_phase(('mustard_bottle',), 1, 'above_bowl', 'glass'))
        p.motion_gen.reset.assert_called_once_with(reset_seed=True)
        p.motion_gen.finetune_trajopt_solver.reset_seed.assert_called_once_with()
        p.motion_gen.finetune_js_trajopt_solver.reset_seed.assert_called_once_with()
        np.testing.assert_allclose(p.motion_gen.world_coll_checker.update_obb_dims.call_args.args[0], [1.12, 2.12, 3.12])
        locked = p.motion_gen.update_locked_joints.call_args.args[0]
        self.assertEqual(locked['finger_joint'], .371)
        self.assertEqual(locked['left_inner_finger_joint'], -.371)
        spheres = p.motion_gen.attach_spheres_to_robot.call_args.kwargs['sphere_tensor']
        self.assertEqual(spheres.shape, (100, 4))
        self.assertTrue((spheres[:90, 3] > .01).all())
        np.testing.assert_array_equal(spheres[90:, 3], -100.)
        p.motion_gen.world_coll_checker.enable_obstacle.assert_any_call('/World/glass/collider', enable=True)
        p.motion_gen.world_coll_checker.enable_obstacle.assert_any_call('/World/mustard_bottle/collider', enable=False)
        report = p.context_report()
        self.assertTrue(report['phase_reused'])
        self.assertEqual(report['seed'], 17)
        self.assertEqual(report['calls']['phase_reset_s'], 1)
        self.assertEqual(report['calls']['world_update_s'], 1)

    def test_geometry_changes_clear_roadmap_but_identical_queries_keep_it(self):
        p = self.planner
        p.reset_for_new_phase(('mustard_bottle',), 1, 'above_bowl')
        p.motion_gen.reset.reset_mock()
        p._prepare_path_query(('mustard_bottle',), 1, 'above_bowl')
        p.motion_gen.reset.assert_not_called()
        p.world.attachment[0, 3] += 1e-10  # Exact comparison, even caller-mutated arrays.
        p._prepare_path_query(('mustard_bottle',), 1, 'above_bowl')
        p.motion_gen.reset.assert_called_once_with(reset_seed=False)
        p.motion_gen.reset.reset_mock()
        p.world.attachment = None
        p.world.gripper_joint_position = 0.
        p._prepare_path_query(('mustard_bottle', 'bowl'), 0, 'retreat')
        p.motion_gen.detach_spheres_from_robot.assert_called_once_with()
        p.motion_gen.reset.assert_called_once_with(reset_seed=False)
        self.assertEqual(p.context_calls['query_geometry_roadmap_reset'], 2)

    def test_unsupported_hazard_returns_false_and_update_errors_propagate(self):
        p = self.planner
        self.assertFalse(p.reset_for_new_phase(hazard_key='unsupported'))
        p.motion_gen.reset.assert_not_called()
        p.motion_gen.update_locked_joints.assert_not_called()
        p.motion_gen.attach_spheres_to_robot.assert_not_called()
        error = RuntimeError('GPU world update failed')
        p.motion_gen.world_coll_checker.update_obb_dims.side_effect = error
        with self.assertRaises(RuntimeError) as raised:
            p.reset_for_new_phase(hazard_key='glass')
        self.assertIs(raised.exception, error)
        p.motion_gen.reset.assert_not_called()
        self.assertEqual(p.context_calls['phase_reset_s'], 2)

    def test_opt_in_warmup_runs_once_and_constructor_preserves_query_dimensions(self):
        motion = fake_motion_gen()
        loader = Mock(side_effect=lambda *args, **kwargs: kwargs)
        modules = {
            'torch': SimpleNamespace(), 'warp': SimpleNamespace(torch=object()),
            'curobo.geom.sdf.world': SimpleNamespace(CollisionCheckerType=SimpleNamespace(MESH='mesh')),
            'curobo.geom.types': SimpleNamespace(Cuboid=object, Mesh=object, WorldConfig=SimpleNamespace),
            'curobo.types.base': SimpleNamespace(TensorDeviceType=SimpleNamespace),
            'curobo.wrap.reacher.motion_gen': SimpleNamespace(MotionGen=lambda config: motion,
                MotionGenConfig=SimpleNamespace(load_from_robot_config=loader)),
        }
        with patch.dict(sys.modules, modules):
            p = CuroboPlanner(SimpleNamespace(meshes={}, objects=[]), SimpleNamespace(base=np.eye(4)),
                              seed=91, use_cuda_graph=True, warmup=True)
            p.warmup()
        motion.warmup.assert_called_once_with(enable_graph=True, batch=None, warmup_js_trajopt=True)
        motion.reset.assert_called_once_with(reset_seed=True)
        motion.finetune_trajopt_solver.reset_seed.assert_called_once_with()
        motion.finetune_js_trajopt_solver.reset_seed.assert_called_once_with()
        settings = loader.call_args.kwargs
        self.assertTrue(settings['use_cuda_graph'])
        self.assertEqual((settings['num_ik_seeds'], settings['num_trajopt_seeds'], settings['trajopt_tsteps']), (32, 4, 40))
        self.assertEqual((settings['ik_seed'], settings['graph_seed']), (91, 91))
        report = p.context_report()
        self.assertTrue(report['warmup_completed'])
        self.assertEqual(report['calls']['warmup_s'], 1)
        self.assertGreaterEqual(report['timings_seconds']['constructor_s'], 0.)


if __name__ == '__main__':
    unittest.main()
