import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.banana_transport_height import read_candidate, transport_target


class TransportHeightTests(unittest.TestCase):
    def setUp(self):
        self.candidate = read_candidate()
        self.payload, self.bowl, self.attachment = np.eye(4), np.eye(4), np.eye(4)
        self.payload[:3, 3] = [-2.2, 2.1, 1.06]
        self.bowl[:3, 3] = [-1.8, 2.15, .92]
        self.attachment[:3, 3] = [.013, -.004, -.028]
        self.payload[:3, :3] = Rotation.from_euler('xyz', [.2, -.1, .3]).as_matrix()
        self.attachment[:3, :3] = Rotation.from_euler('z', .13).as_matrix()

    def target(self, **changes):
        args = dict(actual_payload=self.payload, actual_bowl=self.bowl,
            original_close_attachment=self.attachment, candidate=self.candidate,
            source_object_sha256=self.candidate['source_object_sha256'])
        return transport_target(**(args|changes))

    def test_dynamic_source_xy_height_and_orientation_without_attachment_rebinding(self):
        original = [a.copy() for a in (self.payload, self.bowl, self.attachment)]
        target, info = self.target()
        desired = target@self.attachment
        np.testing.assert_allclose(desired[:2, 3], self.bowl[:2, 3], atol=1e-14)
        self.assertAlmostEqual(desired[2, 3], self.payload[2, 3], places=14)
        np.testing.assert_allclose(desired[:3,:3], self.payload[:3,:3], atol=1e-14)
        for a, b in zip(original, (self.payload, self.bowl, self.attachment)):
            np.testing.assert_array_equal(a, b)
        self.assertFalse(info['attachment_rebound'])
        self.assertFalse(info['physical_transport_verified'])
        # A different real lift/target observation produces a different goal.
        self.payload[2, 3] += .023
        self.bowl[0, 3] -= .12
        changed, _ = self.target()
        changed_payload = changed@self.attachment
        np.testing.assert_allclose(changed_payload[:3, 3], [-1.92, 2.15, 1.083], atol=1e-14)

    def test_does_not_raise_to_old_point_two_metres_above_bowl(self):
        target, info = self.target()
        self.assertAlmostEqual((target@self.attachment)[2, 3], 1.06)
        self.assertNotAlmostEqual((target@self.attachment)[2, 3], self.bowl[2, 3]+.2)
        self.assertEqual(info['target_root_z_difference_m'], 0.)

    def test_nonrigid_reflected_nan_or_wrong_shape_observation_rejected(self):
        cases = [np.ones((3,3)), np.full((4,4), np.nan)]
        reflected = np.eye(4); reflected[0,0] = -1.; cases.append(reflected)
        scaled = np.eye(4); scaled[0,0] = 1.001; cases.append(scaled)
        affine = np.eye(4); affine[3,0] = .001; cases.append(affine)
        for key in ('actual_payload', 'actual_bowl', 'original_close_attachment'):
            for bad in cases:
                with self.subTest(key=key, shape=bad.shape), self.assertRaises(ValueError):
                    self.target(**{key: bad})

    def test_source_mismatch_or_changed_rule_rejected(self):
        with self.assertRaises(ValueError):
            self.target(source_object_sha256={'banana':'other','bowl':'other'})
        for key, value in [('additional_height_m', .001), ('source_clearance_m', .001),
                ('attachment_rule', 'rebind_current'), ('baseline_and_variants_share_candidate', False),
                ('full_original_world_and_self_and_source_checks_required', False), ('task_id', 'other')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.target(candidate=self.candidate|{key:value})

    def test_prepared_controller_source_identity_is_bound(self):
        from kitchen_safety.extensions.replicator.banana_transport_height import BASE_PROFILE
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp); base = tmp/'base.json'; candidate = tmp/'candidate.json'
            base.write_bytes(BASE_PROFILE.read_bytes()); candidate.write_text(json.dumps(self.candidate))
            self.assertEqual(read_candidate(candidate, base)['candidate_id'], 'retain_actual_lift_end_height')
            base.write_text(base.read_text()+' ')
            with self.assertRaises(ValueError): read_candidate(candidate, base)
            altered = json.loads(base.read_text()); altered['source_object_sha256']['banana'] = 'different'
            base.write_text(json.dumps(altered))
            bad = self.candidate|{'base_profile_sha256':hashlib.sha256(base.read_bytes()).hexdigest()}
            candidate.write_text(json.dumps(bad))
            with self.assertRaises(ValueError): read_candidate(candidate, base)


class TransportProvenanceTests(unittest.TestCase):
    def fixture(self):
        from kitchen_safety.extensions.replicator.task_collection_evidence import BANANA_TRANSPORT_CANDIDATE, BANANA_TASK
        candidate = read_candidate()
        text = json.dumps(candidate)
        digest = hashlib.sha256(text.encode()).hexdigest()
        binding = dict(profile_path='transport.json', profile_sha256=digest,
            module_path=BANANA_TRANSPORT_CANDIDATE, module_sha256='module')
        metadata = dict(task_id=BANANA_TASK, source_hashes=candidate['source_object_sha256'],
            transport_candidate_binding=binding)
        entries = {'transport.json': dict(kind='transport_candidate_input', initial_sha256=digest, source_text=text),
            BANANA_TRANSPORT_CANDIDATE: dict(kind='execution_source', initial_sha256='module'),
            'scripts/extensions/replicator/task_collect.py': dict(kind='execution_source',
                source_text='provenance.bind_transport_candidate(path)')}
        from kitchen_safety.extensions.replicator.banana_transport_height import BASE_PROFILE
        bases = [dict(initial_sha256=candidate['base_profile_sha256'], source_text=BASE_PROFILE.read_text())]
        return dict(transport_candidate_binding=binding), metadata, entries, bases

    def test_selected_input_is_captured_before_app_without_global_dependency(self):
        from kitchen_safety.extensions.replicator.task_collection_evidence import (
            TaskRunProvenance, ROOT, TASK_CODE, BANANA_TRANSPORT_CANDIDATE)
        from kitchen_safety.extensions.replicator.banana_transport_height import CANDIDATE
        self.assertNotIn(BANANA_TRANSPORT_CANDIDATE, TASK_CODE)
        recorder = TaskRunProvenance(ROOT/'configs/extensions/replicator/g_shape_banana_bowl_source_layout.json')
        self.assertNotIn(BANANA_TRANSPORT_CANDIDATE, recorder.entries)
        binding = recorder.bind_transport_candidate(CANDIDATE)
        self.assertEqual(recorder.entries[binding['profile_path']]['kind'], 'transport_candidate_input')
        self.assertEqual(recorder.entries[binding['module_path']]['kind'], 'execution_source')
        self.assertTrue(recorder.check()['passed'])
        recorder.scene_bound = True
        with self.assertRaises(ValueError): recorder.bind_transport_candidate(CANDIDATE)
        bowl = TaskRunProvenance(ROOT/'configs/extensions/replicator/g_shape_bowl_sink_source_layout.json')
        with self.assertRaises(ValueError): bowl.bind_transport_candidate(CANDIDATE)

    def test_history_is_decided_by_recorded_collector_and_task(self):
        from kitchen_safety.extensions.replicator.task_collection_evidence import audit_transport_candidate_binding, BANANA_TASK, BOWL_TASK
        self.assertEqual(audit_transport_candidate_binding({}, {'task_id':BANANA_TASK}, {}, []), {'required':False})
        code = {'scripts/extensions/replicator/task_collect.py':dict(kind='execution_source',
            source_text='provenance.bind_transport_candidate(path)')}
        self.assertEqual(audit_transport_candidate_binding({}, {'task_id':BOWL_TASK}, code, []), {'required':False})
        with self.assertRaises(ValueError):
            audit_transport_candidate_binding({}, {'task_id':BANANA_TASK}, code, [])

    def test_binding_module_input_rules_and_original_assets_cannot_be_missing_or_changed(self):
        from copy import deepcopy
        from kitchen_safety.extensions.replicator.task_collection_evidence import audit_transport_candidate_binding, BANANA_TRANSPORT_CANDIDATE
        report, metadata, entries, bases = self.fixture()
        self.assertTrue(audit_transport_candidate_binding(report, metadata, entries, bases)['required'])
        for key in ('transport.json', BANANA_TRANSPORT_CANDIDATE):
            altered = deepcopy(entries); del altered[key]
            with self.subTest(missing=key), self.assertRaises(ValueError):
                audit_transport_candidate_binding(report, metadata, altered, bases)
        for field, value in [('additional_height_m', .01), ('source_clearance_m', 0.),
                ('attachment_rule', 'measured_postlift'), ('source_layout_sha256', 'wrong'),
                ('source_kitchen_sha256', 'wrong'), ('source_object_sha256', {}),
                ('base_profile_sha256', 'wrong'), ('baseline_and_variants_share_candidate', False)]:
            altered = deepcopy(entries); candidate = json.loads(altered['transport.json']['source_text'])
            candidate[field] = value; altered['transport.json']['source_text'] = json.dumps(candidate)
            with self.subTest(field=field), self.assertRaises(ValueError):
                audit_transport_candidate_binding(report, metadata, altered, bases)
        for field in ('profile_sha256', 'module_sha256', 'profile_path', 'module_path'):
            changed_meta = deepcopy(metadata); changed_report = deepcopy(report)
            changed_meta['transport_candidate_binding'][field] = 'wrong'
            changed_report['transport_candidate_binding'][field] = 'wrong'
            with self.subTest(field=field), self.assertRaises(ValueError):
                audit_transport_candidate_binding(changed_report, changed_meta, entries, bases)


if __name__ == '__main__':
    unittest.main()
