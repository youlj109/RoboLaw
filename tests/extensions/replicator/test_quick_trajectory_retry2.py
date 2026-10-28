"""CPU contracts for reviewed retry2 targets, goal corrections and contact scope."""
from copy import deepcopy
from types import SimpleNamespace
import unittest

import numpy as np

from kitchen_safety.extensions.replicator.quick_trajectory_retry2 import (
    PHASES, apply_retry_targets, checked_pose,
)


def pose(x=0., y=0., z=0.):
    result = np.eye(4)
    result[:3, 3] = [x, y, z]
    return result


def fixture(payload='banana'):
    vertices = np.array([[x, y, z] for x in (-.04, .04)
                         for y in (-.04, .04) for z in (-.02, .02)])
    geometry = SimpleNamespace(vertices=vertices, provenance={'source_sha256': 'source'})
    initial = pose(1., 2., .9)
    grasp = pose(1., 2., .93)
    attachment = np.linalg.inv(grasp) @ initial
    goal = pose(1.3, 2., .75)
    targets = {phase: grasp.copy() for phase in PHASES}
    targets['lower'] = goal @ np.linalg.inv(attachment)
    targets['retreat'] = targets['lower'].copy()
    targets['retreat'][2, 3] += .15
    support = '/World/kitchen/Asset/CornerCounter_0'
    world = SimpleNamespace(objects=[
        (support, None, np.array([.9, 1.9, .8]), np.array([1., 2.1, .878])),
        ('/World/kitchen/Asset/CornerCounter_1', None,
         np.array([1., 1.9, .8]), np.array([1.1, 2.1, .878])),
        ('/World/kitchen/Asset/Sink/RimFront', None,
         np.array([1.1, 1.8, .93]), np.array([1.5, 1.83, .95])),
    ])
    plan = dict(payload_name=payload, geometry=geometry,
                initial_payload_pose=initial, attachment=attachment,
                targets=targets, source_support=support,
                destination_mask='/World/kitchen/Asset/Sink/BasinBottom',
                nominal_gripper_joint_rad=.73 if payload == 'bowl' else .50)
    layout = {'task_id': 'test', 'objects': {payload: {'position': [1., 2., .9]}}}
    recipe = dict(scene_id='test__v', task_id='test', payload_name=payload,
                  layout_fixed=True,
                  target_poses_world={key: value.tolist() for key, value in targets.items()},
                  nominal_attachment=attachment.tolist())
    return plan, layout, world, recipe, goal


def add_goal_correction(recipe, dz):
    recipe['goal_vertical_clearance_m'] = dz
    recipe['goal_correction_evidence'] = {'path': 'audited.json', 'sha256': 'bound-by-runner'}
    for phase in ('lower', 'retreat'):
        recipe['target_poses_world'][phase][2][3] += dz


class Retry2TargetsTests(unittest.TestCase):
    def test_rejects_invalid_transform_matrices(self):
        reflection = np.eye(4); reflection[0, 0] = -1
        scaling = np.eye(4); scaling[1, 1] = 1.01
        bottom = np.eye(4); bottom[3, 0] = .01
        nonfinite = np.eye(4); nonfinite[0, 3] = np.nan
        for value in (np.eye(3), reflection, scaling, bottom, nonfinite):
            with self.subTest(value=value), self.assertRaises(ValueError):
                checked_pose(value)
        original = pose(1, 2, 3)
        checked = checked_pose(original)
        checked[0, 3] = 7
        self.assertEqual(original[0, 3], 1)

    def test_new_grasp_rebinds_initial_and_preserves_exact_final_goal(self):
        plan, layout, world, recipe, goal = fixture()
        before = deepcopy(plan)
        grasp = pose(1.01, 2.005, .94)
        grasp[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        attachment = np.linalg.inv(grasp) @ plan['initial_payload_pose']
        recipe['target_poses_world']['approach'] = grasp.tolist()
        recipe['target_poses_world']['lower'] = (goal @ np.linalg.inv(attachment)).tolist()
        recipe['nominal_attachment'] = attachment.tolist()
        result = apply_retry_targets(plan, layout, world, {}, recipe)
        np.testing.assert_allclose(result['targets']['approach'] @ result['attachment'],
                                   plan['initial_payload_pose'], atol=1e-12)
        np.testing.assert_allclose(result['targets']['lower'] @ result['attachment'], goal, atol=1e-12)
        for phase in PHASES:
            np.testing.assert_array_equal(plan['targets'][phase], before['targets'][phase])
        self.assertIs(result['geometry'], plan['geometry'])
        self.assertEqual(result['nominal_gripper_joint_rad'], plan['nominal_gripper_joint_rad'])
        self.assertEqual(result['destination_mask'], plan['destination_mask'])
        self.assertTrue(result['retry']['final_payload_pose_preserved'])
        self.assertTrue(result['retry']['collision_models_and_masks_unchanged'])

    def test_rejects_attachment_inconsistent_with_initial_payload(self):
        plan, layout, world, recipe, _ = fixture()
        recipe['nominal_attachment'][0][3] += .01
        with self.assertRaisesRegex(ValueError, 'attachment.*inconsistent'):
            apply_retry_targets(plan, layout, world, {}, recipe)

    def test_requires_exact_phase_set_and_current_task_payload(self):
        for edit in ('missing', 'extra', 'task', 'payload', 'layout'):
            plan, layout, world, recipe, _ = fixture()
            if edit == 'missing': recipe['target_poses_world'].pop('retreat')
            if edit == 'extra': recipe['target_poses_world']['teleport'] = np.eye(4).tolist()
            if edit == 'task': recipe['task_id'] = 'other'
            if edit == 'payload': recipe['payload_name'] = 'mustard'
            if edit == 'layout': recipe['layout_fixed'] = False
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                apply_retry_targets(plan, layout, world, {}, recipe)

    def test_bowl_vertical_correction_preserves_xy_orientation_and_prefix(self):
        plan, layout, world, recipe, goal = fixture('bowl')
        add_goal_correction(recipe, .012722)
        result = apply_retry_targets(plan, layout, world, {}, recipe)
        expected = goal.copy(); expected[2, 3] += .012722
        np.testing.assert_allclose(result['targets']['lower'] @ result['attachment'], expected)
        for phase in PHASES[:4]:
            np.testing.assert_array_equal(result['targets'][phase], plan['targets'][phase])
        self.assertFalse(result['retry']['final_payload_pose_preserved'])
        self.assertTrue(result['retry']['original_task_destination_preserved'])
        np.testing.assert_array_equal(result['retry']['original_final_payload_pose'], goal)
        self.assertEqual(result['retry']['goal_vertical_clearance_m'], .012722)

    def test_bowl_correction_requires_positive_bounded_delta_and_evidence(self):
        for dz in (-.001, .030001, float('inf'), float('nan')):
            plan, layout, world, recipe, _ = fixture('bowl')
            add_goal_correction(recipe, dz)
            with self.subTest(dz=dz), self.assertRaises(ValueError):
                apply_retry_targets(plan, layout, world, {}, recipe)
        for evidence in (None, {}, ''):
            plan, layout, world, recipe, _ = fixture('bowl')
            add_goal_correction(recipe, .01)
            recipe['goal_correction_evidence'] = evidence
            with self.subTest(evidence=evidence), self.assertRaises(ValueError):
                apply_retry_targets(plan, layout, world, {}, recipe)

    def test_bowl_correction_cannot_change_xy_or_orientation(self):
        for mode in ('x', 'y', 'rotation'):
            plan, layout, world, recipe, _ = fixture('bowl')
            add_goal_correction(recipe, .01)
            if mode == 'rotation':
                matrix = np.array(recipe['target_poses_world']['lower'])
                matrix[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
                recipe['target_poses_world']['lower'] = matrix.tolist()
            else:
                recipe['target_poses_world']['lower'][0 if mode == 'x' else 1][3] += .01
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'final payload pose'):
                apply_retry_targets(plan, layout, world, {}, recipe)

    def test_bowl_corrected_goal_requires_actual_sink_rim_and_room_below_it(self):
        for mode in ('missing_rim', 'too_high'):
            plan, layout, world, recipe, _ = fixture('bowl')
            add_goal_correction(recipe, .03)
            if mode == 'missing_rim':
                world.objects = world.objects[:2]
            else:
                name, unused, low, high = world.objects[2]
                high[2] = .81
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                apply_retry_targets(plan, layout, world, {}, recipe)

    def test_other_payload_cannot_shift_goal_with_or_without_bowl_exception(self):
        for payload in ('banana', 'mustard'):
            for explicit in (False, True):
                plan, layout, world, recipe, _ = fixture(payload)
                if explicit:
                    add_goal_correction(recipe, .01)
                else:
                    recipe['target_poses_world']['lower'][2][3] += .01
                with self.subTest(payload=payload, explicit=explicit), self.assertRaises(ValueError):
                    apply_retry_targets(plan, layout, world, {}, recipe)

    def test_banana_allows_only_measured_original_adjacent_counter_supports(self):
        plan, layout, world, recipe, _ = fixture()
        recipe['source_contact_supports'] = [row[0] for row in world.objects[:2]]
        recipe['source_contact_evidence'] = {'measured_bottom_gap_m': .002}
        result = apply_retry_targets(plan, layout, world, {}, recipe)
        self.assertEqual(result['source_supports'], recipe['source_contact_supports'])
        self.assertFalse(result['retry']['added_hazard_masks'])
        self.assertFalse(result['retry']['collision_models_and_masks_unchanged'])
        self.assertNotIn('source_supports', plan)

    def test_rejects_nonadjacent_low_penetrating_hazard_or_unbound_support(self):
        for mode in ('gap', 'penetration', 'xy', 'hazard', 'wrong_counter',
                     'unknown', 'missing_original', 'no_evidence', 'too_many', 'wrong_payload'):
            plan, layout, world, recipe, _ = fixture('bowl' if mode == 'wrong_payload' else 'banana')
            a, b = [row[0] for row in world.objects[:2]]
            name, unused, low, high = world.objects[1]
            if mode == 'gap': high[2] = .87
            if mode == 'penetration': high[2] = .89
            if mode == 'xy': low[0], high[0] = 1.2, 1.3
            if mode == 'hazard': name = '/World/hazard/CornerCounter_1'
            if mode == 'wrong_counter': name = '/World/kitchen/Asset/Table'
            world.objects[1] = (name, unused, low, high)
            recipe['source_contact_supports'] = [a, name]
            recipe['source_contact_evidence'] = {'measured': True}
            if mode == 'unknown': recipe['source_contact_supports'][1] = '/World/kitchen/CornerCounter_404'
            if mode == 'missing_original': recipe['source_contact_supports'] = [b]
            if mode == 'no_evidence': recipe.pop('source_contact_evidence')
            if mode == 'too_many': recipe['source_contact_supports'].append(a)
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                apply_retry_targets(plan, layout, world, {}, recipe)


if __name__ == '__main__':
    unittest.main()
