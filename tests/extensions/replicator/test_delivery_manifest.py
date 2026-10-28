"""Incomplete evidence and stale/automatic visual claims cannot become delivery."""
import json
import hashlib
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import h5py
import numpy as np

from kitchen_safety.extensions.replicator.startup_recorder import audit_startup
from scripts.extensions.replicator.build_delivery_manifest import (Manifest, TASK, build, episode_checks,
    verify_source_geometry_coverage, verify_startup)
from scripts.extensions.replicator.review_episode import sha


class DeliveryManifestTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_empty_task_keeps_missing_gates_and_is_not_complete(self):
        result = build(dict(task_id=TASK, neutral=str(self.root), baselines={}, variants={}, planning_seeds=[42, 43, 44]))
        self.assertFalse(result['passed'])
        self.assertIn('complete_specification', result['failed_or_missing'])
        self.assertIn('neutral_and_mobility', result['failed_or_missing'])

    def test_mutated_reviewed_file_and_failed_json_rejected(self):
        path = self.root / 'evidence.json'
        path.write_text('{"passed": true, "checks": {"real": true}}')
        manifest = Manifest()
        manifest.hashes({str(path): sha(path)})
        digest = sha(path)
        path.write_text('{"passed": true, "checks": {"real": false}}')
        with self.assertRaisesRegex(ValueError, 'Changed'):
            manifest.hashes({str(path): digest})
        with self.assertRaisesRegex(ValueError, 'failed'):
            manifest.document(path, require_pass=True)

    def test_visual_true_without_observations_or_bound_media_rejected(self):
        path = self.root / 'review.json'
        media = self.root / 'video.mp4'
        media.write_bytes(b'fixture')
        payload = dict(passed=True, episode=str(self.root), scope='sampled', sha256={str(media): sha(media)})
        path.write_text(json.dumps(payload))
        with self.assertRaisesRegex(ValueError, 'observations'):
            Manifest().visual(path, self.root, [media])
        payload['observations'] = ['This is a synthetic test review, not a real visual acceptance.']
        path.write_text(json.dumps(payload))
        Manifest().visual(path, self.root, [media])
        with self.assertRaisesRegex(ValueError, 'required images/video'):
            Manifest().visual(path, self.root, [self.root / 'other.mp4'])

    def test_relative_neutral_overview_hash_uses_neutral_directory(self):
        nested = self.root / 'overview' / 'images.h5'
        nested.parent.mkdir()
        nested.write_bytes(b'fixture')
        Manifest().hashes({'overview/images.h5': sha(nested)}, relative_to=self.root)

    def test_missing_independent_report_is_retained_as_failed_check(self):
        manifest = Manifest()
        manifest.check('independent', lambda: manifest.document(self.root / 'missing.json', require_pass=True))
        self.assertFalse(manifest.checks['independent']['passed'])
        self.assertIn('Missing evidence', manifest.checks['independent']['error'])

    def test_old_episode_without_absolute_motion_sidecar_cannot_pass(self):
        manifest = Manifest()
        episode_checks(manifest, 'old', dict(episode=str(self.root), reports=str(self.root)), TASK, self.root)
        gate = manifest.checks['old.absolute_motion_and_provenance']
        self.assertFalse(gate['passed'])
        self.assertIn('Absolute timeline/source provenance failed', gate['error'])

    def test_diagnostic_explanation_is_not_final_visual_evidence(self):
        video = self.root/'explanation.mp4'; video.write_bytes(b'synthetic fixture')
        video.with_suffix('.validation.json').write_text(json.dumps(dict(passed=True,
            diagnostic_only=True, complete_source_physics_bound=True)))
        entry = dict(episode=str(self.root/'variant'), reports=str(self.root), baseline='human',
            geometry_explanation=str(video))
        spec = dict(task_id=TASK, neutral=str(self.root), planning_seeds=[42, 43, 44],
            baselines={'human': dict(episode=str(self.root/'baseline'), reports=str(self.root))}, variants={'human': entry})
        result = build(spec)
        gate = result['checks']['human.source_explanation_and_explicit_visual']
        self.assertFalse(gate['passed'])
        self.assertIn('diagnostic or incomplete', gate['error'])

    def test_source_relation_and_fixed_base_check_lists_preserve_semantics(self):
        path = self.root / 'audit.json'
        for checks in [[{'passed': True}], [{'intersecting_pair_count': 0}]]:
            path.write_text(json.dumps(dict(passed=True, checks=checks)))
            Manifest().document(path, require_pass=True)
        for checks in [[], [{'passed': False}], [{'intersecting_pair_count': 1}], [{}]]:
            path.write_text(json.dumps(dict(passed=True, checks=checks)))
            with self.assertRaises(ValueError):
                Manifest().document(path, require_pass=True)

    def source_geometry_fixture(self, prefix_safe=True, motion_safe=True):
        from scripts.extensions.replicator.variant_geometry import summarize_samples
        def row(i, stream, safe):
            measure = dict(distance_m=.1 if safe else .01, required_m=.02,
                slack_m=.08 if safe else -.01, valid=safe, intersecting=False)
            result = dict(stream=stream, physics_index=i, action_index=i, phase='settle',
                **{mode: dict(measure) for mode in ['source_mesh', 'source_hull', 'conservative_execution']})
            if stream == 'startup': result.update(absolute_physics_step=10+i, startup_stage='warmup')
            return result
        prefix = [row(i, 'startup', prefix_safe) for i in range(2)]
        motion = [row(i, 'motion', motion_safe) for i in range(2)]
        for name in ['startup_physics_states.npz', 'physics_states.npz']:
            np.savez(self.root/name, q=np.zeros((2, 1)), joint_names=['joint'], phase=['settle']*2,
                action_index=np.arange(2), physics_step=np.arange(10, 12), startup_stage=['warmup']*2)
        names = ['startup_physics_states.npz', 'startup_audit.json', 'motion_physics_index.npz',
                 'motion_physics_alignment.json', 'collection_provenance.json']
        for name in names[1:]: (self.root/name).write_text('{}')
        continuity = dict(passed=True)
        mocked = patch('scripts.extensions.replicator.build_delivery_manifest.audit_saved_collection_evidence', return_value=continuity)
        mocked.start(); self.addCleanup(mocked.stop)
        return dict(full_reset_to_end_source_geometry=True, continuous_collection_evidence=continuity,
            startup_input_sha256={name: sha(self.root/name) for name in names}, startup_rows=prefix, rows=motion,
            startup_samples=2, samples=2, total_physics_samples=4, required_m=.02,
            startup_summary=summarize_samples(prefix), summary=summarize_samples(motion),
            whole_episode_summary=summarize_samples(prefix+motion))

    def test_full_source_coverage_and_raw_row_identity(self):
        geometry = self.source_geometry_fixture()
        verify_source_geometry_coverage(Manifest(), geometry, self.root, 'safe')
        geometry['startup_rows'][1]['absolute_physics_step'] += 1
        with self.assertRaisesRegex(ValueError, 'prefix timeline'):
            verify_source_geometry_coverage(Manifest(), geometry, self.root, 'safe')

    def test_old_motion_only_and_missing_source_prefix_cannot_pass(self):
        with self.assertRaisesRegex(ValueError, 'omits reset/startup'):
            verify_source_geometry_coverage(Manifest(), {'full_reset_to_end_source_geometry': False}, self.root, 'safe')
        geometry = self.source_geometry_fixture()
        geometry['startup_rows'].pop()
        with self.assertRaisesRegex(ValueError, 'samples missing'):
            verify_source_geometry_coverage(Manifest(), geometry, self.root, 'safe')

    def test_startup_source_risk_is_unsafe_but_cannot_prove_motion_necessity(self):
        geometry = self.source_geometry_fixture(prefix_safe=False)
        for expected in ['safe', 'risk']:
            with self.subTest(expected=expected), self.assertRaisesRegex(ValueError, 'required interval'):
                verify_source_geometry_coverage(Manifest(), geometry, self.root, expected)

    def test_forged_source_boolean_or_margin_is_rejected(self):
        geometry = self.source_geometry_fixture()
        geometry['rows'][0]['source_mesh']['distance_m'] = .01
        with self.assertRaisesRegex(ValueError, 'margin/arithmetic'):
            verify_source_geometry_coverage(Manifest(), geometry, self.root, 'safe')

    def startup_fixture(self):
        episode, neutral = self.root / 'episode', self.root / 'neutral'
        episode.mkdir(); neutral.mkdir(); (episode / 'overview').mkdir()
        n, decimation, settle, total = 117, 2, 50, 60
        def poses(shape):
            value = np.zeros(shape); value[..., 6] = 1.; return value
        data = dict(physics_step=np.arange(100, 100+n), startup_stage=np.asarray(
            ['reset_initial']+['warmup']*16+['initial_settle']*100),
            warmup_action_index=np.asarray([-1]+np.repeat(np.arange(8), 2).tolist()+[-1]*100),
            action_index=np.asarray([-1]*17+np.repeat(np.arange(50), 2).tolist()), phase=np.asarray(['settle']*n),
            q=np.zeros((n, 13)), dq=np.zeros((n, 13)), mustard=poses((n, 13)), bowl=poses((n, 13)),
            hazard_state=np.zeros((n, 13)), has_hazard_state=np.zeros(n, dtype=bool),
            hazard_pose=np.zeros((n, 7)), has_hazard_pose=np.zeros(n, dtype=bool),
            background_rigid_poses=poses((n, 1, 7)), background_rigid_velocities=np.zeros((n, 1, 6)),
            robot_body_pose_w=poses((n, 1, 7)), static_anchor_transforms=np.empty((n, 0, 4, 4)))
        alias = dict(native_path='native_root', unique_joint_graph_root=['authored_body'], source_mass_kg=1.,
                     native_mass_kg=1., translation_error_m=0., rotation_error_deg=0.)
        topology = dict(joint_connected_native_paths=['native_root'], authored_joint_connected_paths=['authored_body'],
            unobserved_joint_connected_bodies=[], articulation_root_aliases={'authored_body': alias},
            joint_endpoints=[dict(joint='hinge', role='body0', rigid_body='authored_body')],
            static_anchor_paths=[], animated_static_anchors=[])
        meta = dict(reset_physics_step=100, world_ready_physics_step=216, physics_dt_s=.05, action_dt_s=.1,
            joint_names=['j'+str(i) for i in range(13)], robot_body_names=['robot'], background_paths=['native_root'],
            static_anchor_paths=[], background_topology=topology, callback_errors=[], hazard_kind='none', dynamic_hazard=False,
            background_physics={'recipe': 'synthetic fixture only'})
        for name in ['joint_names', 'robot_body_names', 'background_paths', 'static_anchor_paths']:
            data[name] = np.asarray(meta[name], dtype=str)
        np.savez(episode / 'startup_physics_states.npz', **data)
        report = audit_startup(data, meta)
        report.update(metadata=meta, prefix_safety=dict(passed=True, failure=None, samples=n, checks=n, valid_calls=2*n),
            code_snapshots={k: '# synthetic fixture source '+k for k in ['collector', 'startup_recorder', 'deferred_safety']})
        codes = {k: hashlib.sha256(v.encode()).hexdigest() for k, v in report['code_snapshots'].items()}
        report['input_sha256'] = dict(codes, **{'startup_physics_states.npz': sha(episode / 'startup_physics_states.npz')})
        report['checks'].update(prefix_execution_safety=True, code_unchanged_during_startup=True)
        integrity = dict(passed=True, initial_sha256=codes, checked_sha256=codes, checked_physics_step=236)
        result = dict(startup_recording=report, collection_code_integrity=integrity, background_rigid_paths=['native_root'],
            background_physics=meta['background_physics'], effective_controls=dict(initial_settling_actions=settle))
        hold = dict(passed=True, articulation_root_aliases={'authored_body': alias}, joint_connected_bodies=['authored_body'],
            joint_targets=[dict(joint='hinge', role='body0', rigid_body='authored_body', kind='dynamic_rigid_body')],
            background_paths=['native_root'], static_anchor_paths=[], background_physics=meta['background_physics'])
        # Isolate prefix validation from the separate source/USD neutral audit.
        # The latter has its own raw-schema tests and actual-data CPU regression.
        np.savez(neutral / 'neutral_states.npz', q=np.zeros((1, 1)))
        def validated_neutral(directory, require_full_physics=False):
            self.assertEqual(directory, neutral)
            self.assertTrue(require_full_physics)
            return hold, {}
        mocked = patch('scripts.extensions.replicator.build_delivery_manifest.load_neutral', side_effect=validated_neutral)
        mocked.start()
        self.addCleanup(mocked.stop)
        for path, payload in [(episode/'startup_audit.json', report), (episode/'startup_code_integrity.json', integrity),
                (episode/'result.json', result), (neutral/'neutral_probe.json', hold)]:
            path.write_text(json.dumps(payload))
        with h5py.File(episode / 'trajectory.h5', 'w') as f:
            f.attrs['dt'] = .1
            for name in ['joint_names', 'robot_body_names']:
                f.attrs[name] = json.dumps(meta[name])
            f['time'] = np.arange(total)*.1
            for name in ['q', 'dq', 'mustard', 'bowl', 'robot_body_pose_w', 'background_rigid_poses']:
                for group in ['obs', 'next_obs']:
                    f[group+'/'+name] = data[name][:total]
        (episode/'overview/frame_alignment.json').write_text(json.dumps(dict(measurements=[dict(
            action_index=i, physics_step_before=116+2*i, physics_step_after=116+2*i) for i in range(total)])))
        np.savez(episode / 'physics_states.npz', action_index=np.repeat(np.arange(settle, total), decimation))
        return episode, neutral, data, report, result

    def test_complete_startup_fixture_and_original_missing_prefix(self):
        episode, neutral, _, _, _ = self.startup_fixture()
        self.assertTrue(verify_startup(Manifest(), episode, neutral)['passed'])
        (episode/'startup_physics_states.npz').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing evidence'):
            verify_startup(Manifest(), episode, neutral)

    def test_prefix_fcl_true_without_full_checks_rejected(self):
        episode, neutral, _, report, result = self.startup_fixture()
        report['prefix_safety']['checks'] -= 1
        (episode/'startup_audit.json').write_text(json.dumps(report))
        (episode/'result.json').write_text(json.dumps(result))
        with self.assertRaisesRegex(ValueError, 'prefix FCL'):
            verify_startup(Manifest(), episode, neutral)

    def test_unproven_alias_and_wrong_hazard_identity_rejected(self):
        episode, neutral, _, report, result = self.startup_fixture()
        alias = report['metadata']['background_topology']['articulation_root_aliases']['authored_body']
        alias['native_mass_kg'] = 2.
        for path, value in [(episode/'startup_audit.json', report), (episode/'result.json', result)]:
            path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'Unproven'):
            verify_startup(Manifest(), episode, neutral)
        alias['native_mass_kg'] = 1.
        result['obstacle'] = dict(static=False)
        for path, value in [(episode/'startup_audit.json', report), (episode/'result.json', result)]:
            path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'hazard identity'):
            verify_startup(Manifest(), episode, neutral)

    def test_raw_prefix_motion_cannot_hide_behind_positive_report(self):
        episode, neutral, data, report, result = self.startup_fixture()
        data['q'][2, 0] = .01
        np.savez(episode/'startup_physics_states.npz', **data)
        report['input_sha256']['startup_physics_states.npz'] = sha(episode/'startup_physics_states.npz')
        for path, value in [(episode/'startup_audit.json', report), (episode/'result.json', result)]:
            path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'Recomputed startup'):
            verify_startup(Manifest(), episode, neutral)

    def test_startup_hdf5_state_and_physics_handoff_must_match(self):
        episode, neutral, _, _, _ = self.startup_fixture()
        with h5py.File(episode/'trajectory.h5', 'r+') as f:
            f['obs/q'][1, 0] = .00001
        with self.assertRaisesRegex(ValueError, 'actual state mismatch'):
            verify_startup(Manifest(), episode, neutral)
        with h5py.File(episode/'trajectory.h5', 'r+') as f:
            f['obs/q'][1, 0] = 0.
        np.savez(episode/'physics_states.npz', action_index=np.repeat(np.arange(51, 60), 2))
        with self.assertRaisesRegex(ValueError, 'missing or duplicate'):
            verify_startup(Manifest(), episode, neutral)

    def test_snapshot_hash_and_actual_collection_end_check_are_required(self):
        episode, neutral, _, report, result = self.startup_fixture()
        report['code_snapshots']['collector'] += '\n# changed'
        for path, value in [(episode/'startup_audit.json', report), (episode/'result.json', result)]:
            path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError, 'snapshot/initial/final'):
            verify_startup(Manifest(), episode, neutral)


if __name__ == '__main__':
    unittest.main()
