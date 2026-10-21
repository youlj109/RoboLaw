"""Bounded offline evidence tests; no simulator or drive changes."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
from safety_risk.grip_evidence_binding import build_grip_evidence_binding, resolve_grip_evidence_binding, target_finger_contacts
from safety_risk.task_success import evaluate_single_pick
from safety_risk.iso_input import _pt_pairs, _latest_target_contact_evidence, raw_frame


def config(name):
    return yaml.safe_load((ROOT / 'workflows/simbox/core/configs/robots' / (name + '.yaml')).read_text())


def pair(body, target='pick_object_right'):
    return {'bodyA': body, 'bodyB': 'object/' + target}


def raw(binding=None, robot='genie1', target='pick_object_right'):
    meta = {'robot_id': robot, 'target_object_id': target, 'physics_config': {'physics_dt_s': .1}}
    if binding is not None:
        meta['grip_evidence_binding'] = binding
    return {'episode_meta': meta,
            'metadata': {'num_steps': 3, 'physics_step_ids': [0, 1, 2],
                         'task_reference_world': {'phase': 'settled_before_first_action', 'positions': {target: [0, 0, 0]}}},
            'object_state': {'object_pose_gt': {target: {'source': 'PhysXDataCollector.post_physics_world_pose',
                'coordinate_frame': 'world', 'translation_per_step': [[0, 0, .1]] * 3}}},
            'collision_gt': {'collision_pair_gt': [[], [], []], 'contact_coverage_per_frame': [True] * 3}}


class BindingTests(unittest.TestCase):
    def test_four_actual_configs_and_both_arms(self):
        for robot in ('fr3', 'genie1', 'lift2', 'split_aloha'):
            cfg = config(robot)
            for arm in ('left', 'right'):
                if ('fl_filter_paths' if arm == 'left' else 'fr_filter_paths') not in cfg:
                    continue
                b = build_grip_evidence_binding(cfg, robot_id=robot, arm=arm, target_object_id='pick_object_right')
                data = raw(b, robot)
                bound = resolve_grip_evidence_binding(data)
                pairs = [pair(b['finger_groups'][side][0]) for side in ('left', 'right')]
                self.assertTrue(target_finger_contacts(pairs, bound)[0])
                self.assertFalse(target_finger_contacts(pairs[:1], bound)[0])
                data['collision_gt']['collision_pair_gt'] = [pairs] * 3
                self.assertTrue(evaluate_single_pick(data, {'version': 'test', 'task_axis': {'lift_height_m': .05, 'hold_duration_s': .2}})['protocol_verified_task_success'])
                latest = _latest_target_contact_evidence(data['collision_gt'], 2, 'gripper_close', bound)
                self.assertTrue(latest['bilateral_target_finger_contact'])
                protected = _pt_pairs({}, data['collision_gt'], 2, 'gripper_close', bound)
                self.assertTrue(all(p['held_contact_active'] for p in protected if p['actor'] == 'held_object'))
                self.assertTrue(any(p['object_id'] == 'pick_object_right' for p in protected))
                projected = raw_frame(data, 2)['features']
                self.assertTrue(projected['hs']['target_cup_relation_evidence']['bilateral_grip_ever'])
                self.assertTrue(next(p for p in projected['pt']['protected_pairs']
                    if p['actor'] == 'robot' and p['object_id'] == 'pick_object_right')['latest_contact_evidence']['bilateral_target_finger_contact'])

    def test_no_cross_arm_or_other_robot_mix(self):
        cfg = config('lift2')
        b = build_grip_evidence_binding(cfg, robot_id='lift2', arm='left', target_object_id='pick_object_right')
        right = build_grip_evidence_binding(cfg, robot_id='lift2', arm='right', target_object_id='pick_object_right')
        bound = resolve_grip_evidence_binding(raw(b, 'lift2'))
        self.assertFalse(target_finger_contacts([pair(b['finger_groups']['left'][0]), pair(right['finger_groups']['right'][0])], bound)[0])
        self.assertFalse(target_finger_contacts([pair(x[0].replace('robot/lift2/', 'robot/other/')) for x in b['finger_groups'].values()], bound)[0])

    def test_missing_nonfranka_and_invalid_explicit_are_unknown(self):
        data = raw()
        b = resolve_grip_evidence_binding(data)
        self.assertIsNone(target_finger_contacts([], b)[0])
        self.assertIsNone(_latest_target_contact_evidence(data['collision_gt'], 0, None, b)['bilateral_target_finger_contact'])
        self.assertTrue(all(p['held_contact_active'] is None for p in _pt_pairs({}, data['collision_gt'], 0, binding=b) if p['actor'] == 'held_object'))
        self.assertIsNone(evaluate_single_pick(data, {'version': 'test', 'task_axis': {'lift_height_m': .05, 'hold_duration_s': .2}})['protocol_verified_task_success'])
        self.assertEqual(resolve_grip_evidence_binding(raw({}, 'franka'))['status'], 'unknown')

    def test_legacy_franka_without_new_metadata(self):
        data = raw(robot='franka', target='pick_object_left')
        pairs = [pair('robot/franka/fr3/' + leaf, 'pick_object_left') for leaf in ('panda_leftfinger', 'panda_rightfinger')]
        data['collision_gt']['collision_pair_gt'] = [pairs] * 3
        self.assertTrue(target_finger_contacts(pairs, resolve_grip_evidence_binding(data))[0])
        data['episode_meta'].pop('robot_id')
        self.assertEqual(resolve_grip_evidence_binding(data)['status'], 'legacy_franka')
        self.assertTrue(evaluate_single_pick(data, {'version': 'test', 'task_axis': {'lift_height_m': .05, 'hold_duration_s': .2}})['protocol_verified_task_success'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
