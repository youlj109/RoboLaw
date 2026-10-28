"""Exercise the real adapter with CPU stand-ins for CUDA-owned results."""
import gc
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import weakref

import numpy as np

from kitchen_safety.curobo_backend import CuroboPlanner


class ArrayTensor:
    def __init__(self, value):
        self.value = np.array(value, copy=True)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value


class Result:
    def __init__(self, path, success=True):
        self.success = np.array([success])
        self.status = None if success else 'IK_FAIL'
        self.valid_query = True
        self.position_error = .001
        self.rotation_error = .002
        self.interpolation_dt = .1
        self.position = ArrayTensor(path)
        # A solver result can own tensors not needed by the selected path.
        self.debug_info = {'optimizer_buffers': ArrayTensor(np.zeros((32, 40, 7)))}

    def get_interpolated_plan(self):
        return SimpleNamespace(position=self.position)


class ResultLifetimeTests(unittest.TestCase):
    def setUp(self):
        self.path = np.array([np.zeros(7), np.full(7, .01), np.full(7, .02)])
        self.results, self.debug_buffers, self.queries = [], [], []
        self.success = True

        def solve(start, goal, config):
            result = Result(self.path, self.success)
            self.results.append(weakref.ref(result))
            self.debug_buffers.append(weakref.ref(result.debug_info['optimizer_buffers']))
            self.queries.append(config)
            return result

        self.planner = CuroboPlanner.__new__(CuroboPlanner)
        self.planner.world = SimpleNamespace(attachment=None)
        self.planner.kin = SimpleNamespace(fk=lambda q: np.eye(4))
        self.planner.dt, self.planner.max_joint_speed = 1/15, .4
        self.planner.transport_object_padding_m = .01
        self.planner.T_B_W = self.planner.T_ee_tcp = np.eye(4)
        self.planner.args = SimpleNamespace(to_device=lambda value: np.array(value, copy=True))
        self.planner.torch = SimpleNamespace(cuda=SimpleNamespace(synchronize=Mock()))
        self.planner._last_grip = .38
        self.planner.last_plan = None
        self.planner.obstacle_names = ['/world/glass/a', '/world/bowl/a']
        self.planner.collision_geometry = {'fcl_and_physx_geometry': 'original'}
        self.planner._sync_gripper_and_attachment = Mock()
        self.planner.motion_gen = SimpleNamespace(
            plan_single=solve, plan_single_js=solve,
            world_coll_checker=SimpleNamespace(enable_obstacle=Mock()))
        modules = {
            'curobo.types.math': SimpleNamespace(Pose=SimpleNamespace(
                from_list=lambda values, args: values)),
            'curobo.types.robot': SimpleNamespace(JointState=SimpleNamespace(
                from_position=lambda values, **kwargs: values)),
            'curobo.wrap.reacher.motion_gen': SimpleNamespace(
                MotionGenPlanConfig=lambda **kwargs: kwargs),
        }
        patcher = patch.dict(sys.modules, modules)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_pose_and_joint_queries_release_result_but_preserve_path_and_fallback(self):
        for method in ('pose', 'joint'):
            with self.subTest(method=method):
                if method == 'pose':
                    output = self.planner.plan_pose(self.path[0], np.eye(4),
                                                   ignore_names=('bowl',), phase='transfer')
                else:
                    output = self.planner.plan_joint_goal(self.path[0], self.path[-1],
                                                         ignore_names=('bowl',), phase='transfer')
                gc.collect()
                self.assertIsNone(self.results[-1](), 'Adapter retained the full GPU result')
                self.assertIsNone(self.debug_buffers[-1](), 'Adapter retained optimizer buffers')
                np.testing.assert_array_equal(output['positions'][0], self.path[0])
                np.testing.assert_array_equal(output['positions'][-1], self.path[-1])
                self.assertEqual(output['metadata']['status'], 'SUCCESS')
                self.assertIs(output['metadata'], self.planner.last_plan)
                self.assertEqual(self.queries[-1], {
                    'max_attempts': 4, 'enable_graph_attempt': 2,
                    'enable_finetune_trajopt': True, 'time_dilation_factor': .8})
                self.planner.motion_gen.world_coll_checker.enable_obstacle.assert_any_call(
                    '/world/glass/a', enable=True)
                self.planner.motion_gen.world_coll_checker.enable_obstacle.assert_any_call(
                    '/world/bowl/a', enable=False)
        self.assertEqual(self.planner.torch.cuda.synchronize.call_count, 2)

    def test_failed_queries_still_raise_diagnostics_without_persistent_result(self):
        self.success = False
        for method in ('pose', 'joint'):
            with self.subTest(method=method):
                with self.assertRaisesRegex(RuntimeError, 'IK_FAIL.*valid_query=True'):
                    if method == 'pose':
                        self.planner.plan_pose(self.path[0], np.eye(4), phase='transfer')
                    else:
                        self.planner.plan_joint_goal(self.path[0], self.path[-1], phase='transfer')
                gc.collect()
                self.assertIsNone(self.results[-1]())
                self.assertIsNone(self.debug_buffers[-1]())
                self.assertIsNone(self.planner.last_plan)

    def test_bad_path_start_still_rejected(self):
        self.path[0] = .1
        with self.assertRaisesRegex(RuntimeError, 'does not start at the actual joint state'):
            self.planner.plan_pose(np.zeros(7), np.eye(4), phase='transfer')
        self.assertIsNone(self.planner.last_plan)

    def test_constructor_disables_result_debug_without_changing_solver_settings(self):
        config_loader = Mock(side_effect=lambda *args, **kwargs: kwargs)
        motion_gen = Mock()
        modules = {
            'torch': SimpleNamespace(),
            'warp': SimpleNamespace(torch=object()),
            'curobo.geom.sdf.world': SimpleNamespace(
                CollisionCheckerType=SimpleNamespace(MESH='mesh')),
            'curobo.geom.types': SimpleNamespace(Cuboid=object, Mesh=object,
                                                WorldConfig=SimpleNamespace),
            'curobo.types.base': SimpleNamespace(TensorDeviceType=SimpleNamespace),
            'curobo.wrap.reacher.motion_gen': SimpleNamespace(
                MotionGen=motion_gen,
                MotionGenConfig=SimpleNamespace(load_from_robot_config=config_loader)),
        }
        with patch.dict(sys.modules, modules):
            CuroboPlanner(SimpleNamespace(meshes={}, objects=[]), SimpleNamespace(base=np.eye(4)),
                          seed=73)
        settings = config_loader.call_args.kwargs
        self.assertIs(settings['store_debug_in_result'], False)
        self.assertEqual(settings['num_ik_seeds'], 32)
        self.assertEqual(settings['num_trajopt_seeds'], 4)
        self.assertEqual(settings['trajopt_tsteps'], 40)
        self.assertEqual(settings['ik_seed'], 73)
        self.assertEqual(settings['graph_seed'], 73)
        self.assertEqual(settings['collision_activation_distance'], .04)
        self.assertEqual(settings['position_threshold'], .002)
        self.assertEqual(settings['rotation_threshold'], .02)
        self.assertIs(settings['self_collision_check'], True)
        self.assertIs(settings['self_collision_opt'], True)
        self.assertIs(settings['use_cuda_graph'], False)


if __name__ == '__main__':
    unittest.main()
