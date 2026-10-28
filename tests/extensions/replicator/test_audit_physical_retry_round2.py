"""Small synthetic CPU tests for the independent round-2 evidence audit."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scripts.extensions.replicator import audit_physical_retry_round2 as audit


class AuditHelpersTests(unittest.TestCase):
    def settings(self):
        return dict(initial_settle_seconds=1/30, close_seconds=.1,
                    release_seconds=.2, final_settle_seconds=1/30)

    def arrays(self):
        raw = dict(q=np.zeros((6, 7)), phase=np.asarray(audit.PHASES),
            gripper=np.asarray([0., 0., .78, .78, .78, 0.]),
            dt=np.asarray(1/30), tcp_world=np.repeat(np.eye(4)[None], 6, axis=0))
        retimed = dict(raw, source_edge_start_index=np.arange(6), source_edge_end_index=np.arange(6),
            source_edge_fraction=np.zeros(6), source_vertex_output_index=np.arange(6))
        return raw, retimed

    def test_exact_command_stream_uses_execution_grip_and_override_holds(self):
        raw, _ = self.arrays()
        settings = audit.merged_settings(self.settings(), {'close_seconds': .2, 'release_seconds': .3})
        stream = audit.expected_command_stream({'start_q': [0.]*7, 'planning_gripper_joint_rad': .74}, raw, settings)
        close = [row for row in stream if row['phase'] == 'close']
        release = [row for row in stream if row['phase'] == 'release']
        self.assertEqual(len(close), 6)
        self.assertEqual(len(release), 9)
        self.assertTrue(all(row['target'].dtype == np.float32 and row['target'][-1] == np.float32(.78) for row in close))
        self.assertTrue(all(row['target'][-1] == 0 for row in release))
        self.assertEqual([row['source_trajectory_index'] for row in stream if row['source_trajectory_index'] >= 0], list(range(6)))

    def test_override_whitelist_and_numerical_bounds(self):
        for bad in ({'seed': 4}, {'close_seconds': True}, {'close_seconds': float('nan')},
                    {'release_seconds': float('inf')}, {'release_seconds': 0}, {'close_seconds': 10.1}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                audit.merged_settings(self.settings(), bad)
        original = self.settings()
        merged = audit.merged_settings(original, {'close_seconds': 3.})
        self.assertEqual(original['close_seconds'], .1)
        self.assertEqual(merged['close_seconds'], 3.)

    def test_retiming_preserves_new_grip_not_historical_grip(self):
        raw, retimed = self.arrays()
        audit.audit_retiming(raw, retimed, .78, [0.]*7)
        changed = {key: value.copy() for key, value in retimed.items()}
        changed['gripper'][2] = .74
        with self.assertRaisesRegex(ValueError, 'execution grip'):
            audit.audit_retiming(raw, changed, .78, [0.]*7)

    def test_retiming_rejects_changed_vertex_and_wrong_order(self):
        raw, retimed = self.arrays()
        changed = {key: value.copy() for key, value in retimed.items()}
        changed['q'][2, 0] = .001
        with self.assertRaisesRegex(ValueError, 'joints/vertices'):
            audit.audit_retiming(raw, changed, .78, [0.]*7)
        changed = {key: value.copy() for key, value in retimed.items()}
        changed['source_edge_end_index'][2] = 1
        with self.assertRaisesRegex(ValueError, 'mapping'):
            audit.audit_retiming(raw, changed, .78, [0.]*7)

    def test_raw_float32_is_promoted_before_retiming_arithmetic(self):
        raw, _ = self.arrays()
        raw['q'] = np.zeros((6, 7), dtype=np.float32)
        raw['q'][:, 0] = [.000123456, -.000098765, .000123456, .0002, .00025, .0003]
        starts = np.r_[0, np.repeat(np.arange(5), 2)]
        ends = np.r_[0, np.repeat(np.arange(1, 6), 2)]
        fractions = np.r_[0., np.tile([.5, 1.], 5)]
        source = raw['q'].astype(np.float64)
        retimed = dict(q=source[starts]+fractions[:, None]*(source[ends]-source[starts]),
            phase=raw['phase'][ends], gripper=raw['gripper'][ends], dt=np.array(1/30),
            tcp_world=np.repeat(np.eye(4)[None], 11, axis=0),
            source_edge_start_index=starts, source_edge_end_index=ends,
            source_edge_fraction=fractions, source_vertex_output_index=np.arange(0, 11, 2))
        audit.audit_retiming(raw, retimed, .78, raw['q'][0])

    def test_original_predicate_quaternion_and_strict_velocity_boundary(self):
        metadata = {'destination_location': 'sink', 'actual_spawned_local_bounds': {
            'sink': {'min': [-1, -2, 0], 'max': [1, 2, 3]}}, 'thresholds': {
                'force_threshold': .1, 'velocity_threshold': .003, 'support_cone_half_angle_rad': np.pi/4}}
        half = np.sqrt(.5)
        data = dict(destination_pose=np.array([[10, 20, 0, 0, 0, half, half]]*3),
            object_centroid_world=np.array([[8.1, 21, 0], [8.1, 21, 0], [8.1, 21.1, 0]]),
            destination_contact_force=np.array([[0, 0, .1], [0, 0, .1], [0, 0, -.1]]),
            object_linear_velocity=np.array([[0, 0, 0], [.003, 0, 0], [0, 0, 0]]))
        # Stay just within the rotated x bound to avoid floating-point geometry-edge ambiguity.
        data['object_centroid_world'][:2, 1] -= 1e-8
        result = audit.predicate_components(data, metadata)
        self.assertEqual(result['within_destination'].tolist(), [True, True, False])
        self.assertEqual(result['upward_support'].tolist(), [True, True, False])
        self.assertEqual(result['slow'].tolist(), [True, False, True])

    def test_success_requires_original_settle_lift_place_order(self):
        data = dict(has_original_rest_pose=np.array([False, True, True, True]),
            original_rest_position=np.zeros((4, 3)), original_above_rest=np.array([False, False, True, False]),
            original_object_on_destination=np.array([False, False, False, True]),
            terminal_success=np.array([False, False, False, True]))
        metadata = {'thresholds': {'placement_consecutive_steps': 1}}
        audit.audit_terminal_order(data, metadata)
        data['original_above_rest'][:] = False
        with self.assertRaisesRegex(ValueError, 'settle then lift'):
            audit.audit_terminal_order(data, metadata)

    def test_planning_failed_is_zero_attempt_and_no_episode(self):
        with tempfile.TemporaryDirectory() as folder:
            demos = Path(folder)
            row = dict(scene_id='failed_scene', task_id='task', execution_status='planning_failed',
                planning_status='failed', attempts=0, executed=False, task_success=False,
                candidate_planning_result={'path': 'candidate.json', 'sha256': 'candidate'},
                previous_retry_result={'path': 'old.json', 'sha256': 'old'}, physical_attempt_number=3)
            with patch.object(audit, 'audit_history'), patch.object(audit, 'audit_candidate', return_value={'failure_reason': 'IK_FAILED'}):
                result = audit.audit_planning_failure(row, {}, demos, {})
                self.assertFalse(result['executed'])
                self.assertEqual(result['attempts'], 0)
                self.assertIsNone(result['physical_attempt_number'])
                with self.assertRaisesRegex(ValueError, 'consumed'):
                    audit.audit_planning_failure(dict(row, attempts=1), {}, demos, {})
                (demos/'executions'/'failed_scene').mkdir(parents=True)
                with self.assertRaisesRegex(ValueError, 'unexpected physical episode'):
                    audit.audit_planning_failure(row, {}, demos, {})


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        change = patch.object(audit, 'ROOT', self.root)
        change.start(); self.addCleanup(change.stop)
        self.original = dict(scene_id='scene', task_id='task', variant_id='variant', result_path='old-plan.json',
            result_sha256='old-plan', config_path='layout.json', config_sha256='layout',
            source_scene_path='scene.usda', source_scene_sha256='scene',
            trajectory_path='original.npz', trajectory_sha256=self.write_text('original.npz', 'original'),
            execution_status='failed', task_success=False, executed=True)
        original_result = dict(self.original, kind='physical_execution', status='complete',
            planning_result_path='old-plan.json', planning_result_sha256='old-plan')
        self.original.update(self.write_result('first/result.json', original_result))
        history = [audit.prior_binding(self.original)]
        self.previous = dict(self.original, trajectory_path='round1.npz',
            trajectory_sha256=self.write_text('round1.npz', 'different-timing'), physical_attempt_number=2,
            attempts=1, source_trajectory_path='original.npz', source_trajectory_sha256=self.original['trajectory_sha256'],
            prior_physical_attempts=history, retry_plan_path='round1-plan.json', retry_plan_sha256='round1-plan')
        prior_result = dict(self.previous, kind='physical_failure_retry_execution', status='complete',
            planning_result_path='old-plan.json', planning_result_sha256='old-plan')
        self.previous.update(self.write_result('round1/result.json', prior_result))
        self.row = dict(self.previous, physical_attempt_number=3,
            prior_physical_attempts=history+[audit.prior_binding(self.previous)],
            previous_retry_result={'path': self.previous['execution_result_path'], 'sha256': self.previous['execution_result_sha256']})
        self.context = dict(original={'scene': self.original}, previous={'scene': self.previous}, retakes={},
            previous_manifest={'retry_plan_path': 'round1-plan.json', 'retry_plan_sha256': 'round1-plan'})

    def write_text(self, name, text):
        path = self.root/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(text)
        return audit.digest(path)

    def write_result(self, name, result):
        return {'execution_result_path': name, 'execution_result_sha256': self.write_text(name, json.dumps(result))}

    def test_prior_retry_can_have_its_own_retimed_trajectory(self):
        checked = audit.audit_history(self.row, self.row, self.context)
        self.assertEqual(len(checked), 2)
        self.assertNotEqual(checked[0]['trajectory_sha256'], checked[1]['trajectory_sha256'])

    def test_missing_round1_history_or_tampered_history_rejected(self):
        bad = copy.deepcopy(self.row); bad['prior_physical_attempts'] = bad['prior_physical_attempts'][:1]
        with self.assertRaisesRegex(ValueError, 'history differs'):
            audit.audit_history(bad, self.row, self.context)
        (self.root/'round1/result.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'SHA mismatch'):
            audit.audit_history(self.row, self.row, self.context)


if __name__ == '__main__':
    unittest.main()
