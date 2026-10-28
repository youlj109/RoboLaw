"""Schema-2 neutral reports must reproduce their raw evidence and source clock."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.neutral_task_probe import audit, initial_pose_audit
from scripts.extensions.replicator.review_neutral_evidence import load_neutral, validate_schema2


class NeutralEvidenceTest(unittest.TestCase):
    def fixture(self):
        n = 9
        def poses(shape):
            data = np.zeros(shape); data[..., 6] = 1.; return data
        data = dict(physics_step=np.arange(10, 19), q=np.zeros((n, 1)), dq=np.zeros((n, 1)),
            robot_body_pose_w=poses((n, 1, 7)), task_object_states=poses((n, 2, 13)),
            background=poses((n, 1, 7)), background_velocity=np.zeros((n, 1, 6)),
            static_anchor_transforms=np.tile(np.eye(4), (n, 1, 1, 1)),
            task_reference_transforms=np.empty((n, 0, 4, 4)),
            capture_physics_steps=np.repeat(np.arange(10, 18, 2)[:, None], 2, axis=1))
        layout = dict(robot_position=[0, 0, 0], robot_yaw_deg=0., stand_height=0.,
            objects={name: dict(position=[0, 0, 0], yaw_deg=0.) for name in ['banana', 'bowl']})
        info = dict(schema_version=2, task_id='kitchen_bench_replicator_g_shape_banana_bowl',
            environment_factory='AssignedTaskEnvironment', physics_dt=1., dt=2., decimation=2, frames=4,
            reset_physics_step=10, requested_duration_seconds=8., task_settling_seconds=2., background_settling_seconds=5.,
            task_object_names=['banana', 'bowl'], joint_names=['joint'], robot_body_names=['panda_link0'],
            background_paths=['/hinge'], static_anchor_paths=['/anchor'], task_reference_names=[], task_reference_paths=[],
            callback_errors=[], layout=layout, background_topology=dict(joint_connected_native_paths=['/hinge'],
                unobserved_joint_connected_bodies=[]), expected_source_relations=1,
            source_relation_audit=dict(initial_source_relations_passed=True, checks=[dict(passed=True)]), task_references={},
            static_anchors={'/anchor': dict(source_initial_max_matrix_difference=0., no_rigid_body_ancestor=True,
                animated_source_ancestors=[], animated_runtime_ancestors=[], source_world_transform=np.eye(4).tolist())})
        info['recorded_initial_source_poses'] = initial_pose_audit(data['robot_body_pose_w'][0], info['robot_body_names'],
            data['task_object_states'][0], info['task_object_names'], layout)
        info.update(audit(data, info))
        return info, data

    def test_complete_clock_includes_initial_state_without_extra_duration(self):
        info, data = self.fixture()
        result = validate_schema2(info, data)
        self.assertEqual(result['observed_physics_samples'], 9)
        self.assertEqual(result['observed_duration_seconds'], 8.)
        self.assertTrue(result['passed'])

    def test_claimed_success_cannot_hide_raw_gap_or_substep_background_motion(self):
        for mutation in ['gap', 'motion']:
            info, data = self.fixture()
            if mutation == 'gap': data['physics_step'][3] += 1
            else: data['background'][1, 0, 0] = .00201
            with self.subTest(mutation=mutation), self.assertRaisesRegex(ValueError, 'Recomputed'):
                validate_schema2(info, data)

    def test_omitted_check_and_changed_statistic_fail(self):
        info, data = self.fixture()
        del info['checks']['all_background_stable']
        with self.assertRaisesRegex(ValueError, 'claimed checks'):
            validate_schema2(info, data)
        info, data = self.fixture()
        info['observed_duration_seconds'] += 1
        with self.assertRaisesRegex(ValueError, 'claimed raw statistics'):
            validate_schema2(info, data)

    def test_source_static_transform_and_initial_pose_must_match_raw(self):
        info, data = self.fixture()
        data['static_anchor_transforms'][0, 0, 0, 3] = .001
        with self.assertRaisesRegex(ValueError, 'static/source'):
            validate_schema2(info, data)
        info, data = self.fixture()
        data['task_object_states'][0, 0, 0] = .001
        with self.assertRaisesRegex(ValueError, 'initial source pose'):
            validate_schema2(info, data)

    def test_legacy_neutral_only_allowed_for_historical_review(self):
        with tempfile.TemporaryDirectory() as dirname:
            path = Path(dirname)
            (path/'neutral_probe.json').write_text(json.dumps(dict(passed=True, dt=.5)))
            np.savez(path/'neutral_states.npz', q=np.zeros((4, 1)))
            self.assertEqual(load_neutral(path)[0]['duration_seconds'], 2.)
            with self.assertRaisesRegex(ValueError, 'Final delivery'):
                load_neutral(path, require_full_physics=True)

    def test_schema2_raw_hash_change_rejected_before_source_or_visual_claim(self):
        with tempfile.TemporaryDirectory() as dirname:
            path = Path(dirname)
            (path/'neutral_probe.json').write_text(json.dumps(dict(schema_version=2,
                input_sha256={'neutral_states.npz': 'stale'}, passed=True)))
            np.savez(path/'neutral_states.npz', q=np.zeros((4, 1)))
            with self.assertRaisesRegex(ValueError, 'hash changed'):
                load_neutral(path, require_full_physics=True)


if __name__ == '__main__':
    unittest.main()
