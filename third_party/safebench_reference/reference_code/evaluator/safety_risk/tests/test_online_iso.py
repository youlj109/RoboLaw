import unittest
from pathlib import Path
from safety_risk.online_iso import OnlineISOEvaluator, DISTANCE_FIELDS, COLLISION_FIELDS, current_snapshot
from scripts.verify_online_iso import verify


class OnlineTests(unittest.TestCase):
    def fixture(self):
        path = Path(__file__).resolve().parents[1] / 'configs/iso_profile.experimental.json'
        online = OnlineISOEvaluator({'profile_file': str(path), 'mode': 'shadow'})
        data = {'step_ids': [0], 'runtime_physics': {'physics_dt_s': 1/30},
                'joint_position_q_gt': [[0]], 'joint_torque_gt': [[0]],
                'low_level_command_sent': [True],
                'target_object_linear_velocity_gt': [None]}
        data.update({k: [{}] for k in DISTANCE_FIELDS})
        data.update({k: [[]] for k in COLLISION_FIELDS})
        data['contact_coverage_per_frame'] = [True]
        return online, data

    def test_exact_frozen_input_replay(self):
        online, data = self.fixture()
        report = online.evaluate(data)
        self.assertIsNone(report['risk_levels']['HS'])
        self.assertFalse(report['data_complete'])
        self.assertTrue(verify({'metadata': {'num_steps': 1}, 'online_iso': online.export()})['verified'])

    def test_projection_failure_keeps_unknown_frame(self):
        online, data = self.fixture()
        data['contact_force_gt'] = []
        report = online.evaluate(data)
        self.assertTrue(all(x is None for x in report['risk_levels'].values()))
        self.assertIsNotNone(online.records[0]['error'])
        self.assertEqual(report['frame_index'], 0)

    def test_missing_robot_collider_cannot_certify_hs_distance(self):
        _, data = self.fixture()
        data['robot_human_distance_matrix_gt'] = [{'human/hand': {'franka/panda_link6': 0.30}}]
        data['ee_human_distance_gt'] = [{'left→human/hand': 0.30}]
        data['object_human_distance_gt'] = [{'pick_object_left': {'human/hand': 0.30}}]
        data['distance_coverage_status'] = {
            'robot_human_complete': False,
            'robot_visual_rigid_bodies_without_collider': ['/World/task_0/franka/panda/fr3/panda_link1'],
        }
        snapshot = current_snapshot(data, {})
        self.assertIsNone(snapshot['features']['hs']['d_robot_h_min_gt_m'])
        data['distance_coverage_status']['robot_human_complete'] = True
        self.assertEqual(current_snapshot(data, {})['features']['hs']['d_robot_h_min_gt_m'], 0.30)

    def test_record_is_not_mutated_by_next_frame(self):
        online, data = self.fixture()
        online.evaluate(data)
        before = online.export()
        data['collision_pair_gt'][0].append({'bodyA': 'robot/a', 'bodyB': 'human/b'})
        self.assertEqual(before, online.export())

    def test_replay_detects_changed_rule(self):
        online, data = self.fixture()
        online.evaluate(data)
        raw = {'metadata': {'num_steps': 1}, 'online_iso': online.export()}
        raw['online_iso']['records'][0]['report']['rules'][0]['level'] = 'L0'
        self.assertFalse(verify(raw)['verified'])
