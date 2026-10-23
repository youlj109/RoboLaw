"""New neutral-task audit rejects gaps and keeps banana/sink identities intact."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from scripts.extensions.replicator.neutral_task_probe import (MUSTARD_TASK, ROOT, audit, environment_route,
    initial_pose_audit, mustard_evidence_layout, require_successful_report)


class NeutralTaskProbeTest(unittest.TestCase):
    def fixture(self, names=('banana', 'bowl'), references=('floor', 'corner_counter')):
        # Six seconds at 2 Hz, recorded initial state plus every substep.
        n = 13
        info = dict(physics_dt=.5, dt=1., decimation=2, frames=6, reset_physics_step=10,
            requested_duration_seconds=6., task_settling_seconds=2., background_settling_seconds=5.,
            task_object_names=list(names), joint_names=['joint'], robot_body_names=['robot'], background_paths=['/hinge', '/loose'],
            static_anchor_paths=['/anchor'], task_reference_paths=list(references), callback_errors=[],
            background_topology={'joint_connected_native_paths': ['/hinge'], 'unobserved_joint_connected_bodies': []},
            source_relation_audit={'initial_source_relations_passed': True, 'checks': [{'passed': True}]},
            expected_source_relations=1, recorded_initial_source_poses={'passed': True})
        legal = dict(source_initial_max_matrix_difference=0., no_rigid_body_ancestor=True,
                     animated_source_ancestors=[], animated_runtime_ancestors=[])
        info['static_anchors'] = {'/anchor': copy.deepcopy(legal)}
        info['task_references'] = {name: copy.deepcopy(legal) for name in references}
        obj = np.zeros((n, len(names), 13)); obj[:, :, 6] = 1
        bg = np.zeros((n, 2, 7)); bg[:, :, 6] = 1
        capture = 10 + np.arange(6) * 2
        data = dict(physics_step=10+np.arange(n), q=np.zeros((n, 1)), dq=np.zeros((n, 1)),
            robot_body_pose_w=bg[:, :1].copy(),
            task_object_states=obj, background=bg, background_velocity=np.zeros((n, 2, 6)),
            static_anchor_transforms=np.tile(np.eye(4), (n, 1, 1, 1)),
            task_reference_transforms=np.tile(np.eye(4), (n, len(references), 1, 1)),
            capture_physics_steps=np.column_stack([capture, capture]))
        return data, info

    def test_real_object_names_and_static_sink_reference(self):
        data, info = self.fixture()
        report = audit(data, info)
        self.assertTrue(report['passed'])
        self.assertEqual(set(report['task_objects']), {'banana', 'bowl'})
        data, info = self.fixture(('bowl',), ('floor', 'sink_counter', 'sink'))
        report = audit(data, info)
        self.assertTrue(report['passed'])
        self.assertEqual(set(report['task_objects']), {'bowl'})
        self.assertNotIn('sink', info['task_object_names'])

    def test_missing_substep_or_camera_advance_cannot_pass(self):
        data, info = self.fixture()
        data['physics_step'][4] += 1
        self.assertFalse(audit(data, info)['checks']['full_post_reset_physics_coverage'])
        data, info = self.fixture()
        data['capture_physics_steps'][1, 1] += 1
        self.assertFalse(audit(data, info)['checks']['camera_capture_alignment'])

    def test_joint_body_checked_from_reset_and_free_body_after_five_seconds(self):
        data, info = self.fixture()
        data['background'][1, 1, 0] = .1
        self.assertTrue(audit(data, info)['passed'])
        data['background'][1, 0, 0] = .003
        self.assertFalse(audit(data, info)['checks']['all_background_stable'])

    def test_dynamic_or_moving_sink_cannot_be_declared_static(self):
        data, info = self.fixture(('bowl',), ('sink',))
        info['task_references']['sink']['no_rigid_body_ancestor'] = False
        self.assertFalse(audit(data, info)['checks']['legal_task_references'])
        info['task_references']['sink']['no_rigid_body_ancestor'] = True
        data['task_reference_transforms'][-1, 0, 0, 3] = .001
        self.assertFalse(audit(data, info)['checks']['task_references_stable'])

    def test_unobserved_joint_or_task_drift_is_rejected(self):
        data, info = self.fixture()
        info['background_topology']['unobserved_joint_connected_bodies'] = ['/missing']
        self.assertFalse(audit(data, info)['checks']['background_joint_coverage'])
        data['task_object_states'][-1, 0, 0] = .002
        self.assertFalse(audit(data, info)['checks']['task_objects_stable'])

    def test_malformed_task_record_is_not_relabelled(self):
        data, info = self.fixture()
        data['task_object_states'] = data['task_object_states'][:, :1]
        with self.assertRaisesRegex(ValueError, 'task_object_states'):
            audit(data, info)

    def test_failed_json_raises_before_normal_kit_shutdown(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'neutral_probe.json'
            for report, code in [({'passed': False}, 0), ({'passed': False}, 2), ({}, 0), ({'passed': True}, 2)]:
                path.write_text(json.dumps(report))
                with self.assertRaisesRegex(RuntimeError, 'retained evidence'):
                    require_successful_report(directory, code)
            path.write_text('{"passed": true}')
            require_successful_report(directory, 0)

    def test_mustard_uses_original_factory_and_real_source_identities(self):
        self.assertEqual(environment_route(MUSTARD_TASK), 'KitchenSafetyEnvironment')
        self.assertEqual(environment_route('kitchen_bench_replicator_g_shape_banana_bowl'), 'AssignedTaskEnvironment')
        with self.assertRaises(ValueError):
            environment_route('unknown')
        original = json.loads((ROOT/'configs/extensions/replicator/g_shape_mustard.json').read_text())
        before = copy.deepcopy(original)
        layout = mustard_evidence_layout(original)
        self.assertEqual(original, before)
        self.assertEqual(set(layout['objects']), {'mustard_bottle', 'bowl'})
        self.assertEqual(layout['objects']['mustard_bottle']['position'], original['mustard_position'])
        from scripts.extensions.replicator.audit_task_source_relation import check_layout
        report = check_layout(layout)
        self.assertTrue(report['initial_source_relations_passed'])
        self.assertEqual(len(report['checks']), 7)

    def test_initial_pose_or_partial_source_relation_cannot_be_claimed_valid(self):
        layout = dict(robot_position=[0., 0., 0.], stand_height=.8, robot_yaw_deg=0.,
                      objects={'mustard_bottle': {'position': [1., 2., 3.], 'yaw_deg': 0.}})
        body = np.asarray([[0., 0., .8, 0., 0., 0., 1.]])
        obj = np.asarray([[1., 2., 3., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.]])
        self.assertTrue(initial_pose_audit(body, ['panda_link0'], obj, ['mustard_bottle'], layout)['passed'])
        obj[0, 0] += .001
        self.assertFalse(initial_pose_audit(body, ['panda_link0'], obj, ['mustard_bottle'], layout)['passed'])
        data, info = self.fixture(); info['source_relation_audit']['checks'] = []
        self.assertFalse(audit(data, info)['checks']['original_source_relations'])
        data, info = self.fixture(); info['recorded_initial_source_poses']['passed'] = False
        self.assertFalse(audit(data, info)['checks']['recorded_initial_source_poses'])


if __name__ == '__main__':
    unittest.main()
