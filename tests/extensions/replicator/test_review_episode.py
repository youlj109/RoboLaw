"""Reject incomplete/misleading physical evidence without running Kit or CUDA."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import h5py
import numpy as np

from scripts.extensions.replicator.review_episode import main, review


class EpisodeReviewTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.episode = self.root / 'episode'
        self.neutral = self.root / 'neutral'
        self.episode.mkdir()
        self.neutral.mkdir()
        self.phases = ['settle'] * 2 + ['close'] * 2 + ['lift'] * 2 + ['above_bowl'] * 2 + ['lower'] * 2 + ['release'] * 2 + ['settle'] * 4
        self.n, self.dt = len(self.phases), .5
        self.profile = dict(recipe='test-installation', source_sha256='source', recipe_sha256='recipe', overlay_sha256='overlay')
        self.result = dict(success=True, arena_success=True, final_on_destination=True,
            background_physics=self.profile, background_rigid_paths=['/hinged', '/loose'],
            physical_failure_reasons=[], effective_controls=dict(initial_settling_seconds=1., initial_settling_actions=2),
            final_destination_contact_force_matrix_w_n=[[[[0, 0, 1]]]])
        self.neutral_info = dict(passed=True, frames=24, dt=.5, background_settling_seconds=1.,
            articulation_root_aliases={}, joint_connected_bodies=['/hinged'], background_paths=['/hinged', '/loose'],
            joint_targets=[dict(kind='dynamic_rigid_body', rigid_body='/hinged')],
            background_tolerances=dict(translation_m=.002, rotation_deg=1.), background_physics=self.profile)
        self.contacts = [dict(action_index=i, phase=phase, fingers={name: dict(normal_forces_n=[.5], points_bottle_local_m=[[0, 0, 0]])
            for name in ['left_inner_finger', 'right_inner_finger']})
            for i, phase in enumerate(self.phases) if phase in ['close', 'lift', 'above_bowl', 'lower']]
        self.obj = np.zeros((self.n + 1, 13))
        self.obj[:, 6] = 1
        self.obj[5:, 2] = .1
        self.bg = np.zeros((self.n + 1, 2, 7))
        self.bg[:, :, 6] = 1
        self.save()

    def save(self):
        (self.episode / 'result.json').write_text(json.dumps(self.result))
        (self.episode / 'grasp_contacts.json').write_text(json.dumps(self.contacts))
        (self.episode / 'source_relation_audit.json').write_text(json.dumps({'passed': True}))
        (self.neutral / 'neutral_probe.json').write_text(json.dumps(self.neutral_info))
        neutral_frames = self.neutral_info['frames']
        neutral_bg = np.zeros((neutral_frames, 2, 7))
        neutral_bg[:, :, 6] = 1
        np.savez(self.neutral / 'neutral_states.npz', q=np.zeros((neutral_frames, 1)),
                 background=neutral_bg, background_velocity=np.zeros((neutral_frames, 2, 6)))
        with h5py.File(self.episode / 'trajectory.h5', 'w') as h5:
            h5.attrs['dt'] = self.dt
            h5.attrs['joint_names'] = json.dumps(['finger_joint'])
            h5.create_dataset('phase', data=self.phases, dtype=h5py.string_dtype())
            for name, values in [('mustard', self.obj), ('bowl', self.obj),
                                 ('q', np.zeros((self.n + 1, 1))), ('background_rigid_poses', self.bg)]:
                h5.create_dataset('obs/' + name, data=values[:-1])
                h5.create_dataset('next_obs/' + name, data=values[1:])

    def run_review(self):
        return review(self.episode, self.neutral, [-.06, .02])

    def test_valid_measured_episode_uses_actual_neutral_duration_and_settling(self):
        result = self.run_review()
        self.assertTrue(result['passed'], result['checks'])
        self.assertEqual(result['neutral_duration_s'], 12.)
        self.assertEqual(result['initial_settling_s'], 1.)
        self.assertIn(str(self.neutral / 'neutral_states.npz'), result['input_sha256'])
        self.assertIn('Pending', result['visual_review'])

    def test_source_success_flags_cannot_replace_bilateral_carry(self):
        self.contacts[3]['fingers']['right_inner_finger']['normal_forces_n'] = [0]
        self.save()
        result = self.run_review()
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['bilateral_body_load_through_recorded_carry'])

    def test_missing_contact_frame_and_wrong_phase_rejected(self):
        self.contacts.pop(3)
        self.save()
        self.assertFalse(self.run_review()['checks']['carried_contact_records_complete'])
        self.contacts[0]['phase'] = 'release'
        self.save()
        with self.assertRaisesRegex(ValueError, 'phase'):
            self.run_review()

    def test_joint_bodies_are_checked_from_reset_but_free_settling_is_allowed(self):
        self.bg[:2, 1, 0] = .01
        self.save()
        self.assertTrue(self.run_review()['passed'])
        self.bg[1, 0, 0] = .003
        self.save()
        result = self.run_review()
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['all_background_stable'])

    def test_short_neutral_or_changed_profile_cannot_certify_new_episode(self):
        self.neutral_info['frames'] = 10
        self.save()
        self.assertFalse(self.run_review()['checks']['neutral_duration_covers_episode'])
        self.neutral_info['background_physics'] = {**self.profile, 'overlay_sha256': 'changed'}
        self.save()
        self.assertFalse(self.run_review()['checks']['same_background_physics_as_neutral'])

    def test_incomplete_background_and_post_release_motion_rejected(self):
        self.neutral_info['joint_connected_bodies'].append('/unobserved')
        self.obj[-1, 0] = .01
        self.save()
        result = self.run_review()
        self.assertFalse(result['checks']['background_paths_complete'])
        self.assertFalse(result['checks']['released_object_still'])

    def test_nonfinite_or_missing_raw_records_fail(self):
        self.obj[-1, 1] = np.nan
        self.save()
        with self.assertRaisesRegex(ValueError, 'Nonfinite'):
            self.run_review()
        (self.episode / 'trajectory.h5').unlink()
        with self.assertRaises(OSError):
            self.run_review()

    def test_original_force_and_velocity_threshold_boundaries(self):
        self.result['final_destination_contact_force_matrix_w_n'] = [[[[0, 0, .0999]]]]
        self.save()
        result = self.run_review()
        self.assertTrue(result['checks']['destination_support_force'])
        self.assertFalse(result['checks']['original_support_force_magnitude'])
        self.result['final_destination_contact_force_matrix_w_n'] = [[[[0, 0, .1]]]]
        self.obj[-1, 7] = .003
        self.save()
        result = self.run_review()
        self.assertTrue(result['checks']['original_support_force_magnitude'])
        self.assertFalse(result['checks']['original_object_speed'])
        self.obj[-1, 7] = .002999
        self.save()
        self.assertTrue(self.run_review()['passed'])

    def test_large_sideways_or_downward_force_is_not_source_support(self):
        for force in [[1, 0, 0], [0, 0, -1], [1, 0, .1]]:
            with self.subTest(force=force):
                self.result['final_destination_contact_force_matrix_w_n'] = [[[force]]]
                self.save()
                result = self.run_review()
                self.assertTrue(result['checks']['original_support_force_magnitude'])
                self.assertFalse(result['checks']['original_upward_support_cone'])

    def test_neutral_mapping_and_native_arrays_are_required(self):
        self.neutral_info['joint_targets'][0]['native_rigid_body'] = '/wrong'
        self.save()
        self.assertFalse(self.run_review()['checks']['joint_target_native_mapping_complete'])
        np.savez(self.neutral / 'neutral_states.npz', q=np.zeros((24, 1)))
        with self.assertRaises(KeyError):
            self.run_review()

    def test_partial_new_physics_cannot_fall_back_to_legacy_success(self):
        (self.episode / 'startup_audit.json').write_text('{"passed": true}')
        with self.assertRaisesRegex(ValueError, 'Incomplete absolute'):
            self.run_review()

    def test_full_background_uses_reset_not_a_warmup_excursion_and_checks_boundaries(self):
        # These mocks isolate reference-time/row alignment from independent
        # provenance and prefix-audit unit suites; no real evidence is approved.
        full = np.zeros((self.n+9, 2, 7)); full[:, :, 6] = 1.
        full[8, 0, 0] = .0015
        full[15:, 0, 0] = -.0015
        self.bg = full[8:].copy()
        self.save()
        self.assertFalse(self.run_review()['checks']['all_background_stable'])
        meta = dict(background_paths=['/hinged', '/loose'], background_physics=self.profile,
            reset_physics_step=0, physics_dt_s=self.dt)
        prefix = dict(metadata=meta, passed=True)
        self.result['startup_recording'] = prefix
        (self.episode/'result.json').write_text(json.dumps(self.result))
        (self.episode/'startup_audit.json').write_text(json.dumps(prefix))
        np.savez(self.episode/'startup_physics_states.npz', physics_step=np.arange(11),
            background_paths=meta['background_paths'], background_rigid_poses=full[:11])
        np.savez(self.episode/'background_physics_states.npz', paths=meta['background_paths'], poses=full[11:])
        np.savez(self.episode/'motion_physics_index.npz', physics_step=np.arange(11, len(full)))
        with patch('kitchen_safety.extensions.replicator.collection_evidence.audit_saved_collection_evidence',
                   return_value=dict(passed=True, input_sha256={})), \
             patch('kitchen_safety.extensions.replicator.startup_recorder.audit_startup', return_value=dict(passed=True)):
            result = self.run_review()
            self.assertTrue(result['passed'], result['checks'])
            self.assertEqual(result['background_reference'], 'reset_initial_full_physics')
            self.assertEqual(result['background_physics_samples'], len(full))
            full[-1, 0, 0] += .00001
            np.savez(self.episode/'background_physics_states.npz', paths=meta['background_paths'], poses=full[11:])
            with self.assertRaisesRegex(ValueError, 'every recorded action boundary'):
                self.run_review()

    def test_cli_missing_evidence_writes_failure_and_preserves_existing_report(self):
        output = self.root / 'review.json'
        bottle = self.root / 'bottle.usd'
        bottle.write_text('test fixture')
        (self.episode / 'grasp_contacts.json').unlink()
        argv = [str(self.episode), '--neutral', str(self.neutral), '--output', str(output), '--bottle-usd', str(bottle)]
        with patch('kitchen_safety.grasp_geometry.bottle_body_geometry', return_value={'safe_contact_z_range_m': [-.06, .02]}):
            self.assertEqual(main(argv), 1)
        content = output.read_bytes()
        self.assertFalse(json.loads(content)['passed'])
        with self.assertRaises(FileExistsError):
            main(argv)
        self.assertEqual(output.read_bytes(), content)


if __name__ == '__main__':
    unittest.main()
