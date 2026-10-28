"""Support-aware placement geometry without Kit or a permissive safety result.

An exact Cartesian test kinematics and planar bowl isolate the target geometry;
separate collision-rejection tests ensure it cannot bypass the supplied world.
These tests do not establish robot reachability or physical task success.
"""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.placement_planner import (iter_placement_candidates,
    get_placement_seed_configs, _TCP_ORIENTATION)


class CartesianKinematics:
    limits = np.tile([-10., 10.], (7, 1))

    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        pose[:3, :3] = Rotation.from_rotvec(q[3:6]).as_matrix()
        return pose

    def ik(self, pose, guess, attempts=1):
        return np.r_[pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_rotvec(), 0.]


class RedundantCartesianKinematics(CartesianKinematics):
    """Seventh coordinate represents separate IK branches with the same pose."""

    def ik(self, pose, guess, attempts=1):
        q = super().ik(pose, guess, attempts)
        q[6] = guess[6]
        return q


def source_meshes(*args):
    bottle = np.array([[x, y, z] for x in (-.02, .02) for y in (-.02, .02)
                       for z in (-.08, .08)])
    bowl = np.array([[[-.15, -.15, 0.], [.15, -.15, 0.], [.15, .15, 0.]],
                     [[-.15, -.15, 0.], [.15, .15, 0.], [-.15, .15, 0.]]])
    return bottle, bowl


def setup(valid=True):
    # A measured 35-degree grasp with nonzero TCP-to-bottle translation.
    attachment = np.eye(4)
    attachment[:3, :3] = (Rotation.from_euler('x', 35., degrees=True).as_matrix()
                          @ _TCP_ORIENTATION).T
    attachment[:3, 3] = [.012, -.004, .009]
    queries = []

    def query(q, ignore):
        queries.append(tuple(ignore))
        return valid

    world = SimpleNamespace(attachment=attachment, clearances={'person': .04},
        last_collision=('arm', 'person'), valid=query,
        edge=lambda a, b, ignore: query(a, ignore))
    return world, CartesianKinematics(), queries


class SupportedPlacementTests(unittest.TestCase):
    def test_physical_reference_is_resolved_at_the_current_bowl(self):
        seeds = get_placement_seed_configs('human_default_mpfb')
        self.assertTrue(seeds)
        self.assertEqual(get_placement_seed_configs('property_vase'), [])
        world, kin, queries = setup()
        bowl = np.array([2.52, -.31, .95, 0., 0., 0., 1.])
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            candidate = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=bowl, seed_configs=seeds,
                orientations=[(35., 30.)], max_candidates=1))
        np.testing.assert_allclose(candidate['predicted_release_bottle_pose'][:2, 3], bowl[:2])
        self.assertFalse(np.allclose(candidate['lower_q'], seeds[0]['q']))
        self.assertEqual(candidate['metadata']['seed_name'], seeds[0]['name'])
        self.assertTrue(queries)

    def test_physical_reference_cannot_bypass_current_collision_rejection(self):
        world, kin, queries = setup(valid=False)
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            candidates = list(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                seed_configs=get_placement_seed_configs('human_default_mpfb'),
                orientations=[(35., 30.)]))
        self.assertEqual(candidates, [])
        self.assertTrue(queries)

    def candidate(self, bowl, **kwargs):
        world, kin, queries = setup()
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            candidate = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], hover_height=1.135, bowl_pose=bowl,
                orientations=[(35., 30.)], max_candidates=1, **kwargs))
        return candidate, world, queries

    def test_supported_target_centers_bottle_and_replaces_nominal_height(self):
        bowl = np.array([2.5, -.3, .95, 0., 0., 0., 1.])
        before = bowl.copy()
        c, world, queries = self.candidate(bowl)
        bottle = c['lower_pose'] @ world.attachment
        np.testing.assert_allclose(bottle[:2, 3], bowl[:2], atol=1e-12)
        self.assertAlmostEqual(bottle[2, 3] - .08 - bowl[2], .003, places=12)
        self.assertNotAlmostEqual(c['lower_pose'][2, 3], 1.105, places=3)
        self.assertAlmostEqual(c['above_pose'][2, 3] - c['lower_pose'][2, 3], .03, places=12)
        np.testing.assert_allclose(c['metadata']['requested_lower_xyz'], c['lower_pose'][:3, 3])
        self.assertEqual(c['metadata']['nominal_lower_xyz'], [2.5, -.3, 1.105])
        self.assertIn(('Mustard', 'Bowl'), queries)
        self.assertIn(('Mustard',), queries)
        np.testing.assert_array_equal(bowl, before)

    def test_measured_bowl_translation_moves_entire_column(self):
        bowl = np.array([2.5, -.3, .95, 0., 0., 0., 1.])
        first, _, _ = self.candidate(bowl)
        shift = np.array([.02, -.01, .012]);bowl[:3] += shift
        second, _, _ = self.candidate(bowl)
        for key in ('lower_pose', 'above_pose'):
            np.testing.assert_allclose(second[key][:3, 3] - first[key][:3, 3], shift, atol=1e-12)

    def test_no_bowl_preserves_fixed_height_and_legacy_orientation(self):
        world, kin, _ = setup()
        with patch('kitchen_safety.placement_geometry.release_lowering_target', side_effect=AssertionError('Unexpected support query')):
            c = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], hover_height=1.135, obstacle_id='human_compact_sport_palm_down'))
        np.testing.assert_allclose(c['lower_pose'][:3, 3], [2.5, -.3, 1.105], atol=1e-12)
        self.assertEqual((c['tilt'], c['yaw']), (45., -30.))
        self.assertNotIn('supported_release', c['metadata'])

    def test_supported_defaults_include_upright_grasp_and_broad_yaw_search(self):
        world, kin, _ = setup();diagnostics = {}
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            c = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                preferred_tilt_deg=35., obstacle_id='human_compact_sport_palm_down', diagnostics=diagnostics))
        self.assertEqual((c['tilt'], c['yaw']), (35., -30.))
        values = diagnostics['orientation_candidates_deg']
        self.assertEqual(set(tilt for tilt, yaw in values), {15., 25., 35., 45., 55.})
        self.assertTrue({90., -90., 120., -120., 150., -150., 180.}.issubset({yaw for tilt, yaw in values}))

    def test_collision_rejections_still_prevent_candidates(self):
        world, kin, queries = setup(valid=False);diagnostics = {}
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            candidates = list(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                orientations=[(35., 30.)], diagnostics=diagnostics))
        self.assertEqual(candidates, [])
        self.assertGreater(diagnostics['rejection_counts']['lower_geometry'], 0)
        self.assertTrue(queries)


    def test_short_column_fallback_keeps_original_collision_checks(self):
        world, kin, _ = setup()
        bowl = np.array([2.5, -.3, .95, 0., 0., 0., 1.])
        lower_z = []

        def valid(q, ignore):
            if 'Bowl' in ignore:
                lower_z[:] = [q[2]]
                return True
            return q[2] <= lower_z[0] + .019

        world.valid = valid
        world.edge = lambda a, b, ignore: max(a[2], b[2]) <= lower_z[0] + .019
        diagnostics = {}
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            c = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=bowl, orientations=[(35., 30.)],
                diagnostics=diagnostics))
        self.assertAlmostEqual(c['above_pose'][2, 3] - c['lower_pose'][2, 3], .018)
        self.assertEqual(c['metadata']['column_height_m'], .018)
        self.assertEqual(diagnostics['rejection_counts']['column_geometry'], 2)

    def test_off_center_uses_entire_projected_footprint(self):
        world, kin, _ = setup()
        bowl = np.array([2.5, -.3, .95, 0., 0., 0., 1.])
        world.valid = lambda q, ignore: abs((kin.fk(q) @ world.attachment)[0, 3] - bowl[0]) >= .019
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            c = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=bowl, orientations=[(35., 30.)]))
        self.assertEqual(c['metadata']['bottle_xy_offset_m'], [.02, 0.])
        self.assertTrue(c['metadata']['full_projected_footprint_support'])
        self.assertEqual(c['metadata']['supported_release']['bottom_vertices_checked'], 8)
        self.assertEqual(c['metadata']['supported_release']['bottom_band_m'], 1.)
        np.testing.assert_allclose(c['predicted_release_bottle_pose'][:2, 3], bowl[:2] + [.02, 0.])

    def test_off_center_footprint_outside_bowl_never_reaches_ik(self):
        world, kin, _ = setup()
        bowl = np.array([2.5, -.3, .95, 0., 0., 0., 1.])
        world.valid = lambda q, ignore: abs((kin.fk(q) @ world.attachment)[0, 3] - bowl[0]) >= .019
        bottle, triangles = source_meshes()
        triangles[:, :, :2] *= .2
        diagnostics = {}
        with patch('kitchen_safety.placement_geometry._meshes', return_value=(bottle, triangles)):
            candidates = list(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=bowl, orientations=[(35., 30.)],
                diagnostics=diagnostics))
        self.assertEqual(candidates, [])
        self.assertGreater(diagnostics['rejection_counts']['support_target'], 0)
        # Only the center has full footprint support; all offset targets are
        # rejected before their inverse kinematics or trajectory can run.
        self.assertEqual(diagnostics['ik_attempts'], 11)

    def test_compact_raised_preference_is_a_live_supported_target(self):
        world, kin, _ = setup();diagnostics = {}
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            c = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                preferred_tilt_deg=35., obstacle_id='human_compact_sport_raised_wrist',
                diagnostics=diagnostics))
        self.assertEqual((c['tilt'], c['yaw']), (45., 60.))
        self.assertEqual(c['metadata']['bottle_xy_offset_m'], [.03, 0.])
        self.assertEqual(diagnostics['ik_attempts'], 1)
        np.testing.assert_allclose(c['predicted_release_bottle_pose'][:2, 3], [2.53, -.3])

    def test_default_mpfb_prioritizes_existing_offset_away_from_hand(self):
        world, kin, _ = setup()
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            candidate = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                preferred_tilt_deg=35., obstacle_id='human_default_mpfb'))
        self.assertEqual(candidate['metadata']['target_preference'], 'default_mpfb_away_from_hand')
        self.assertEqual(candidate['metadata']['bottle_xy_offset_m'], [0., .02])
        self.assertTrue(candidate['metadata']['full_projected_footprint_support'])
        np.testing.assert_allclose(candidate['predicted_release_bottle_pose'][:2, 3], [2.5, -.28])

    def test_default_mpfb_offset_preference_keeps_support_and_center_fallback(self):
        world, kin, _ = setup();diagnostics = {}
        bottle, triangles = source_meshes();triangles[:, :, :2] *= .2
        with patch('kitchen_safety.placement_geometry._meshes', return_value=(bottle, triangles)):
            candidate = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                preferred_tilt_deg=35., obstacle_id='human_default_mpfb', diagnostics=diagnostics))
        self.assertEqual(candidate['metadata']['bottle_xy_offset_m'], [0., 0.])
        self.assertGreaterEqual(diagnostics['rejection_counts']['support_target'], 6)

    def diverse_candidates(self, obstacle_id='human_compact_sport_palm_down', **kwargs):
        world, _, _ = setup()
        kin = RedundantCartesianKinematics()
        diagnostics = {}
        options = dict(bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                       preferred_tilt_deg=35., obstacle_id=obstacle_id,
                       max_candidates=8, max_ik_attempts=528, diagnostics=diagnostics)
        options.update(kwargs)
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            candidates = list(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], **options))
        return candidates, diagnostics

    def test_human_budget_interleaves_targets_and_retains_ik_branches(self):
        candidates, diagnostics = self.diverse_candidates()
        self.assertEqual(len(candidates), 8)
        self.assertGreater(len({c['tilt'] for c in candidates}), 1)
        self.assertGreater(len({c['yaw'] for c in candidates}), 1)
        self.assertTrue(any(any(c['metadata']['bottle_xy_offset_m']) for c in candidates))
        first = candidates[0]
        repeated = [c for c in candidates if c['tilt'] == first['tilt'] and c['yaw'] == first['yaw']
                    and c['metadata']['bottle_xy_offset_m'] == first['metadata']['bottle_xy_offset_m']]
        self.assertGreater(len({c['metadata']['seed_name'] for c in repeated}), 1)
        self.assertEqual(diagnostics['stop_reason'], 'candidate_budget')
        self.assertEqual(diagnostics['candidates'], 8)

    def test_first_compact_positive_yaw_column_keeps_original_seed(self):
        world, _, _ = setup();kin = RedundantCartesianKinematics()
        # Reproduce the actual layout's rejected negative-yaw center without
        # claiming that this synthetic world represents the real human mesh.
        def valid(q, ignore):
            relative = kin.fk(q)[:3, :3] @ _TCP_ORIENTATION.T
            return Rotation.from_matrix(relative).as_euler('xyz', degrees=True)[2] >= 0
        world.valid = valid
        world.edge = lambda a, b, ignore: valid(a, ignore) and valid(b, ignore)
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            c = next(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                preferred_tilt_deg=35., obstacle_id='human_compact_sport_palm_down'))
        self.assertEqual((c['tilt'], c['yaw']), (35., 30.))
        self.assertEqual(c['metadata']['seed_name'], 'compact_yaw30_j1zero')

    def test_matching_supported_guide_branch_is_tried_early(self):
        for obstacle_id, seed_name in (
                ('human_default_mpfb', 'compact_yawminus30_random'),
                ('human_medium_shirt_palm_down', 'compact_yawminus30_random'),
                ('human_broad_workwear_palm_down', 'compact_yaw30_j1positive')):
            with self.subTest(obstacle_id=obstacle_id), patch(
                    'kitchen_safety.placement_planner._placement_guide_matches',
                    return_value=[{'tilt_deg': 25., 'yaw_deg': 30., 'seed_names': [seed_name]}]):
                candidates, _ = self.diverse_candidates(obstacle_id)
            self.assertEqual((candidates[0]['tilt'], candidates[0]['yaw']), (35., 30.))
            guided = next(c for c in candidates if (c['tilt'], c['yaw']) == (25., 30.)
                          and c['metadata']['bottle_xy_offset_m'] == [0., 0.])
            self.assertEqual(guided['metadata']['seed_name'], seed_name)
            self.assertTrue(guided['metadata']['column_fcl_checked'])
            self.assertFalse(guided['metadata']['transport_validated'])

    def test_nonhuman_and_explicit_orientation_keep_branch_first_order(self):
        for options in ({'obstacle_id': 'property_vase'},
                        {'obstacle_id': 'human_compact_sport_palm_down', 'orientations': [(35., 30.)]}):
            with self.subTest(options=options), patch(
                    'kitchen_safety.placement_planner._placement_guide_matches',
                    side_effect=AssertionError('Legacy branch must not read guide preferences')):
                candidates, diagnostics = self.diverse_candidates(**options)
            self.assertEqual(len(candidates), 8)
            self.assertEqual({(c['tilt'], c['yaw'], *c['metadata']['bottle_xy_offset_m'])
                              for c in candidates}, {(35., 30., 0., 0.)})
            self.assertEqual(diagnostics['target_order'],
                             'preferred_pose_then_interleaved_orientation_and_bowl_offset')

    def test_diversity_retains_ik_budget_and_collision_rejections(self):
        candidates, diagnostics = self.diverse_candidates(max_ik_attempts=3)
        self.assertEqual(len(candidates), 3)
        self.assertEqual(diagnostics['ik_attempts'], 3)
        self.assertEqual(diagnostics['stop_reason'], 'ik_attempt_budget')
        world, kin, queries = setup(valid=False);diagnostics = {}
        with patch('kitchen_safety.placement_geometry._meshes', side_effect=source_meshes):
            candidates = list(iter_placement_candidates(world, kin, np.zeros(7),
                [2.5, -.3, 1.105], bowl_pose=[2.5, -.3, .95, 0., 0., 0., 1.],
                obstacle_id='human_default_mpfb', preferred_tilt_deg=35.,
                diagnostics=diagnostics, max_ik_attempts=12))
        self.assertEqual(candidates, [])
        self.assertEqual(diagnostics['ik_attempts'], 12)
        self.assertTrue(queries)
        self.assertGreater(diagnostics['rejection_counts']['lower_geometry'], 0)


if __name__ == '__main__':
    unittest.main()
