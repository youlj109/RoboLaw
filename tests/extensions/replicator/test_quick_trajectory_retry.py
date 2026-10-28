"""CPU contract checks: retry targets cannot alter the fixed task/layout."""
from copy import deepcopy
from types import SimpleNamespace
import unittest

import numpy as np

from kitchen_safety.extensions.replicator.quick_trajectory_retry import (
    actual_sink_rim, apply_retry_targets, measured_bowl_rim_grasp,
)


def box_geometry():
    vertices = np.array([[x, y, z] for x in (-.02, .02)
                         for y in (-.06, .06) for z in (-.01, .01)])
    triangles = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
                          [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                          [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return SimpleNamespace(vertices=vertices, triangles=triangles,
        low=vertices.min(0), high=vertices.max(0), provenance={'source_sha256':'test'})


class RetryTargetsTests(unittest.TestCase):
    def test_attachment_recomputed_and_original_object_goal_preserved(self):
        initial = np.eye(4); initial[:3, 3] = [1., 2., .9]
        original_grasp = initial.copy(); original_grasp[2, 3] += .035
        attachment = np.linalg.inv(original_grasp) @ initial
        destination = initial.copy(); destination[:3, 3] = [1.3, 1.8, .95]
        lower = destination @ np.linalg.inv(attachment)
        targets = {phase: original_grasp.copy() for phase in
                   ('pregrasp','approach','lift','transfer','lower','retreat')}
        targets['lower'] = lower
        targets['transfer'][2, 3] = 1.2
        geometry = box_geometry()
        plan = dict(payload_name='banana', geometry=geometry,
            initial_payload_pose=initial, attachment=attachment, targets=targets,
            nominal_gripper_joint_rad=.5, source_support='/counter', destination_mask='/bowl')
        before = deepcopy(plan)
        layout = {'task_id':'test', 'objects':{'banana':{'position':[1,2,.9]}}}
        recipe = dict(scene_id='test__v',task_id='test',payload_name='banana',layout_fixed=True,
            provenance={'note':'specific reviewed note'},grasp={'jaw_yaw_local_deg':15.,
                'section_offset_m':.01,'tcp_raise_m':.008},
            pregrasp_offset_world_m=[.02,0,.1],lift_offset_world_xy_m=[0,.03],
            carry_height_delta_m=-.03,retreat_offset_world_m=[0,0,.15])
        world = SimpleNamespace(objects=[])
        updated = apply_retry_targets(plan, layout, world, {}, recipe)
        np.testing.assert_allclose(updated['targets']['approach']@updated['attachment'],initial,atol=1e-12)
        np.testing.assert_allclose(updated['targets']['lower']@updated['attachment'],destination,atol=1e-12)
        np.testing.assert_array_equal(plan['attachment'], before['attachment'])
        np.testing.assert_array_equal(plan['targets']['lower'], before['targets']['lower'])
        self.assertIs(updated['geometry'], geometry)
        self.assertEqual(updated['nominal_gripper_joint_rad'], .5)
        self.assertEqual(updated['source_support'], '/counter')
        self.assertEqual(updated['destination_mask'], '/bowl')
        self.assertEqual(world.objects, [])
        self.assertEqual(layout['objects']['banana']['position'], [1,2,.9])
        self.assertAlmostEqual(updated['targets']['approach'][2,3], .9-.01+.043)

    def test_actual_rim_excludes_faucet_without_removing_world_objects(self):
        rows = [('/World/kitchen/Asset/Sink/RimFront',None,np.zeros(3),np.array([1,1,.92])),
                ('/World/kitchen/Asset/Sink/FaucetSpout',None,np.zeros(3),np.array([1,1,1.27]))]
        world = SimpleNamespace(objects=rows)
        rim, paths = actual_sink_rim(world)
        self.assertEqual(rim, .92)
        self.assertEqual(paths, [rows[0][0]])
        self.assertEqual(len(world.objects), 2)
        with self.assertRaises(ValueError):
            actual_sink_rim(SimpleNamespace(objects=rows[1:]))

    def test_bowl_grasp_measures_narrow_wall_instead_of_diameter(self):
        # A radial wall sector has real inner/outer boundaries at 76/80 mm.
        vertices = np.array([[x,y,z] for x in (.076,.08)
                             for y in (-.01,.01) for z in (0.,.04)])
        geometry = box_geometry(); geometry.vertices = vertices
        geometry.low, geometry.high = vertices.min(0), vertices.max(0)
        center, jaw, info = measured_bowl_rim_grasp(geometry,0.)
        np.testing.assert_allclose(center,[.078,0,.038])
        np.testing.assert_allclose(jaw,[1,0,0])
        self.assertAlmostEqual(info['wall_thickness_m'],.004)
        with self.assertRaises(ValueError):
            measured_bowl_rim_grasp(geometry,180.)


if __name__ == '__main__':
    unittest.main()
