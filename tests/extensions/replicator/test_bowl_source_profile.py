"""Source-asset regression checks for a proposed sink landing, CPU only."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.bowl_controller_profile import ROOT, read_profile, sink_target, grasp_target
from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_grasp, sink_placement_object_pose, sink_reference_bounds
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world


class BowlSourceTargetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout_path = ROOT/'configs/extensions/replicator/g_shape_bowl_sink_source_layout.json'
        cls.layout = json.loads(cls.layout_path.read_text())
        cls.world, cls.kin, cls.pose = build_world(cls.layout)
        cls.profile = read_profile()

    def test_source_floor_offset_avoids_changing_destination_or_orientation(self):
        pose = self.pose.copy()
        pose[3:] = Rotation.from_euler('xyz', [.4, -.3, .2], degrees=True).as_quat()
        target, info = sink_target(pose, self.world, self.profile)
        low, high = sink_reference_bounds(self.world)
        points = self.world.payload_geometry.vertices@target[:3, :3].T+target[:3, 3]
        self.assertTrue(np.all(points[:, :2].min(0) >= low[:2]))
        self.assertTrue(np.all(points[:, :2].max(0) <= high[:2]))
        self.assertAlmostEqual(points[:, 2].min()-high[2], .003, places=12)
        np.testing.assert_allclose(target[:3, :3], Rotation.from_quat(pose[3:]).as_matrix(), atol=1e-14)
        np.testing.assert_allclose(info['offset_from_source_floor_center_m'], [.05, -.1], atol=1e-14)
        self.assertFalse(info['collision_or_task_success_verified'])

    def test_target_outside_original_basin_is_rejected(self):
        low, high = sink_reference_bounds(self.world)
        with self.assertRaisesRegex(ValueError, 'does not fit'):
            sink_placement_object_pose(self.pose, self.world.payload_geometry, self.world, center_xy=high[:2]+[1., 1.])
        with self.assertRaises(ValueError):
            sink_placement_object_pose(self.pose, self.world.payload_geometry, self.world, center_xy=[float('nan'), 0.])

    def test_source_hash_change_cannot_reuse_previous_candidate(self):
        changed = copy.deepcopy(self.profile)
        changed['source_kitchen_sha256'] = '0'*64
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'profile.json'
            path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, 'changed'):
                read_profile(path, self.layout_path)

    def test_live_pose_grasp_matches_reviewed_correction_and_keeps_contact_region(self):
        for shift in ([0., 0., 0.], [.001, -.002, .003]):
            pose = self.pose.copy()
            pose[:3] += shift
            original, before = bowl_rim_grasp(pose, self.world.payload_geometry, self.world)
            adjusted, after = grasp_target(pose, self.world, self.profile)
            np.testing.assert_allclose(adjusted[:3, :3], Rotation.from_euler('x', 25., degrees=True).as_matrix()@original[:3, :3], atol=1e-14)
            np.testing.assert_allclose(adjusted[:3, 3]-original[:3, 3], [0., 0., .005039543828685633], atol=1e-14)
            self.assertEqual(after['contact_region_local_m'], before['contact_region_local_m'])
            self.assertFalse(after['physical_grasp_verified'])
        first, _ = grasp_target(self.pose, self.world, self.profile)
        moved = self.pose.copy(); moved[:2] += [.001, -.002]
        second, _ = grasp_target(moved, self.world, self.profile)
        np.testing.assert_allclose(second[:2, 3]-first[:2, 3], [.001, -.002], atol=1e-14)

    def test_arbitrary_grasp_or_contact_rule_is_rejected(self):
        for field, value in [('rotation_deg', 30.), ('translation_world_m', [0., 0., 0.])]:
            profile = copy.deepcopy(self.profile)
            profile['grasp_adjustment'][field] = value
            with self.assertRaisesRegex(ValueError, 'Unreviewed'):
                grasp_target(self.pose, self.world, profile)
        profile = copy.deepcopy(self.profile)
        profile['contact_region_rule'] = 'expanded_after_failure'
        with self.assertRaisesRegex(ValueError, 'Unreviewed'):
            grasp_target(self.pose, self.world, profile)

    def test_changed_preparation_cannot_justify_runtime_correction(self):
        changed = copy.deepcopy(self.profile)
        name = next(iter(changed['evidence']))
        changed['evidence'][name] = '0'*64
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'profile.json'
            path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, 'evidence changed'):
                read_profile(path, self.layout_path)


if __name__ == '__main__':
    unittest.main()
