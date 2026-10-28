"""Real CPU USD/FCL geometry with fake cuRobo updates; no Kit or CUDA launch."""
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.obstacle_library import resolve_layout
from kitchen_safety.planning import DroidKinematics
from scripts.audit_collection_clearance import hazard_stage


class DynamicWorldUpdateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        layout = resolve_layout('property_wine_glass')
        stage, _ = hazard_stage(layout, 'property', layout['obstacle'])
        base = np.array(layout['robot_position'], dtype=float)
        base[2] += layout['stand_height']
        cls.world = CollisionWorld(stage, DroidKinematics(base, layout['robot_yaw_deg']))
        cls.initial_pose = np.r_[layout['obstacle']['position'], [0., 0., 0., 1.]]

    def setUp(self):
        self.world.set_obstacle_pose('glass', self.initial_pose)
        self.world.clearances['glass'] = .06
        self.world.attachment = np.eye(4)
        self.planner = CuroboPlanner.__new__(CuroboPlanner)
        self.planner.world = self.world
        self.planner.T_B_W = np.linalg.inv(self.world.kin.base)
        self.planner._dynamic_box_names = {
            name for name, _, _, _ in self.world.objects
            if self.world._matches(name, 'glass') or self.world._matches(name, 'barrier')}
        self.planner._last_grip = .38
        self.planner.args = SimpleNamespace(to_device=lambda values: np.array(values, dtype=np.float32))
        self.boxes = {name: SimpleNamespace(pose=['original'], dims=['original'])
                      for name, _, _, _ in self.world.objects}
        self.checker = SimpleNamespace(world_model=SimpleNamespace(get_obstacle=self.boxes.get),
                                       update_obb_pose=Mock(), update_obb_dims=Mock(),
                                       enable_obstacle=Mock())
        self.planner.motion_gen = SimpleNamespace(world_coll_checker=self.checker, reset=Mock())
        fake_pose = SimpleNamespace(from_list=lambda values, args: np.array(values, dtype=float))
        self.module_patch = patch.dict(sys.modules, {'curobo.types.math': SimpleNamespace(Pose=fake_pose)})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)

    def test_box_formula_retains_world_axes_and_degenerate_extent_padding(self):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', [11, -17, 37], degrees=True).as_matrix()
        transform[:3, 3] = [.4, -.2, .7]
        self.planner.T_B_W = transform
        pose, dims = self.planner._planning_box([1., 2., 3.], [2., 2., 3.5], .06)
        np.testing.assert_allclose(pose[:3], (transform @ [1.5, 2., 3.25, 1.])[:3])
        np.testing.assert_allclose(Rotation.from_quat(np.array(pose)[[4, 5, 6, 3]]).as_matrix(),
                                   transform[:3, :3], atol=1e-15)
        np.testing.assert_allclose(dims, [1.12, .1201, .62])

    def test_real_rotated_wine_updates_all_boxes_like_fresh_geometry(self):
        previous_extents = {name: (hi-lo).copy() for name, _, lo, hi in self.world.objects}
        pose = self.initial_pose.copy()
        pose[:3] += [.003, -.008, .002]
        pose[3:] = Rotation.from_euler('xyz', [7., -5., 13.], degrees=True).as_quat()
        self.world.set_obstacle_pose('glass', pose)
        self.world.clearances['glass'] = .071
        attachment = self.world.attachment.copy()
        self.assertTrue(self.planner.update_dynamic_obstacle('glass'))
        gpu_poses = {call.kwargs['name']: call.kwargs['w_obj_pose']
                     for call in self.checker.update_obb_pose.call_args_list}
        gpu_dims = {call.kwargs['name']: call.args[0]
                    for call in self.checker.update_obb_dims.call_args_list}
        self.assertEqual(set(gpu_poses), self.planner._dynamic_box_names)
        self.assertGreater(len(gpu_poses), 1)
        changed_extents = 0
        for name, _, lo, hi in self.world.objects:
            if name not in self.planner._dynamic_box_names:
                self.assertEqual(self.boxes[name].pose, ['original'])
                continue
            # Independent copy of the original constructor's formula.
            transform = self.planner.T_B_W.copy()
            transform[:3, 3] = transform[:3, :3] @ ((lo+hi)/2) + transform[:3, 3]
            expected_pose = np.r_[transform[:3, 3],
                Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]]]
            expected_dims = np.maximum(hi-lo, 1e-4) + 2*self.world._margin(name, 'planning')
            np.testing.assert_array_equal(gpu_poses[name], expected_pose)
            np.testing.assert_array_equal(gpu_dims[name], expected_dims.astype(np.float32))
            np.testing.assert_array_equal(self.boxes[name].pose, expected_pose)
            np.testing.assert_array_equal(self.boxes[name].dims, expected_dims)
            changed_extents += not np.array_equal(previous_extents[name], hi-lo)
        self.assertGreater(changed_extents, 0, 'This case must detect pose-only updates that omit dimensions')
        self.planner.motion_gen.reset.assert_called_once_with(reset_seed=True)
        self.checker.enable_obstacle.assert_not_called()
        np.testing.assert_array_equal(self.world.attachment, attachment)
        self.assertEqual(self.planner._last_grip, .38)

    def test_unsupported_selection_or_topology_returns_false_before_updates(self):
        original_objects = self.world.objects
        try:
            self.assertFalse(self.planner.update_dynamic_obstacle('kitchen'))
            name, obj, lo, hi = next(entry for entry in original_objects
                                     if entry[0] in self.planner._dynamic_box_names)
            self.world.objects = original_objects + [(name + '/new_piece', obj, lo, hi)]
            self.assertFalse(self.planner.update_dynamic_obstacle('glass'))
            self.world.objects = [entry for entry in original_objects if entry[0] != name]
            self.assertFalse(self.planner.update_dynamic_obstacle('glass'))
            self.world.objects = original_objects
            # A true mesh selection is not in the constructor's supported box set.
            self.planner._dynamic_box_names.remove(name)
            self.assertFalse(self.planner.update_dynamic_obstacle('glass'))
        finally:
            self.world.objects = original_objects
        self.checker.update_obb_pose.assert_not_called()
        self.checker.update_obb_dims.assert_not_called()
        self.planner.motion_gen.reset.assert_not_called()

    def test_update_errors_propagate_without_reporting_success_or_resetting(self):
        error = RuntimeError('update failed')
        self.checker.update_obb_dims.side_effect = error
        with self.assertRaises(RuntimeError) as raised:
            self.planner.update_dynamic_obstacle('glass')
        self.assertIs(raised.exception, error)
        self.planner.motion_gen.reset.assert_not_called()


if __name__ == '__main__':
    unittest.main()
