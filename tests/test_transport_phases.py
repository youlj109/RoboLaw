"""CPU geometric phase mapping; no collision or physics outcomes select anchors."""
from copy import deepcopy
import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.placement_planner import get_transport_guides
from kitchen_safety.transport_phases import map_guide_phase_boundaries


class CartesianKinematics:
    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        pose[:3, :3] = Rotation.from_rotvec(q[3:6]).as_matrix()
        return pose


def metadata(q):
    return {'guide_id': 'declared_route', 'phase_boundaries': [{
        'id': 'arrival', 'source_anchor_q': list(q), 'guide_internal_index': 3,
        'source_state_index': 4, 'before_phase': 'transfer',
        'after_phase': 'above_bowl', 'constraint_phase': 'above_bowl',
        'source': {'selection_basis': 'original corridor before preplacement bridge'},
    }]}


class TransportPhaseTests(unittest.TestCase):
    def setUp(self):
        self.kin = CartesianKinematics()
        self.attachment = np.eye(4)
        self.attachment[:3, 3] = [.07, -.02, .03]

    def test_retimed_and_orientation_repaired_point_uses_bottle_xyz(self):
        anchor = np.array([.3, .2, 1., 0., 0., 0., .5])
        anchor_xyz = (self.kin.fk(anchor) @ self.attachment)[:3, 3]
        repaired = anchor.copy();repaired[3:6] = [.2, -.1, .4]
        repaired[:3] = anchor_xyz - self.kin.fk(repaired)[:3, :3] @ self.attachment[:3, 3]
        path = np.array([anchor + [.1, 0, 0, 0, 0, 0, 0],
                         repaired + [.004, 0, 0, 0, 0, 0, 0], repaired,
                         repaired + [0, 0, -.008, 0, 0, 0, 0],
                         anchor + [0, 0, -.1, 0, 0, 0, 0]])
        before = path.copy();a = self.attachment.copy();meta = metadata(anchor);old_meta = deepcopy(meta)
        result = map_guide_phase_boundaries(path, self.kin, self.attachment, meta)
        self.assertTrue(result['applied'], result)
        boundary = result['boundaries'][0]
        self.assertEqual(boundary['path_index'], 2)
        self.assertLess(boundary['error_m'], 1e-12)
        self.assertEqual((boundary['before_phase'], boundary['after_phase']), ('transfer', 'above_bowl'))
        self.assertEqual(boundary['constraint_phase'], 'above_bowl')
        np.testing.assert_allclose(boundary['source_anchor']['carried_xyz_m'], anchor_xyz)
        np.testing.assert_array_equal(path, before)
        np.testing.assert_array_equal(self.attachment, a)
        self.assertEqual(meta, old_meta)
        json.dumps(result)

    def test_replaced_suffix_that_loses_anchor_does_not_segment(self):
        anchor = np.zeros(7);path = np.zeros((3, 7));path[:, 0] = [-.1, .0021, .1]
        result = map_guide_phase_boundaries(path, self.kin, self.attachment, metadata(anchor))
        self.assertEqual(result, {'applied': False, 'boundaries': [], 'reason': 'anchor_outside_tolerance'})

    def test_closest_endpoint_cannot_be_replaced_by_nearby_internal_point(self):
        anchor = np.zeros(7);path = np.zeros((3, 7));path[:, 0] = [0., .001, .1]
        result = map_guide_phase_boundaries(path, self.kin, self.attachment, metadata(anchor))
        self.assertEqual(result['reason'], 'anchor_at_path_endpoint')
        self.assertEqual(result['boundaries'], [])

    def test_separate_passes_near_same_xyz_are_ambiguous(self):
        anchor = np.zeros(7);path = np.zeros((5, 7));path[:, 0] = [-.1, 0., .1, .001, .2]
        result = map_guide_phase_boundaries(path, self.kin, self.attachment, metadata(anchor))
        self.assertEqual(result['reason'], 'ambiguous_anchor_windows')
        self.assertFalse(result['applied'])

    def test_contiguous_xyz_ties_use_earliest_index_and_report_ambiguity(self):
        anchor = np.zeros(7);path = np.zeros((4, 7));path[:, 0] = [-.1, 0., 0., .1]
        path[1, 6] = 2.  # Do not choose q-nearest index 2 instead.
        result = map_guide_phase_boundaries(path, self.kin, self.attachment, metadata(anchor))
        boundary = result['boundaries'][0]
        self.assertEqual(boundary['path_index'], 1)
        self.assertTrue(boundary['xyz_tie'])
        self.assertEqual(boundary['nearest_xyz_tie_indices'], [1, 2])

    def test_absent_or_invalid_declaration_falls_back_without_mutation(self):
        path = np.zeros((3, 7));path[:, 0] = [-.1, 0., .1]
        for meta in ({}, {'phase_boundaries': [{}]}, metadata([float('nan')] * 7)):
            before = path.copy()
            result = map_guide_phase_boundaries(path, self.kin, self.attachment, meta)
            self.assertFalse(result['applied'])
            self.assertEqual(result['boundaries'], [])
            np.testing.assert_array_equal(path, before)

    def test_loader_supplies_independent_self_contained_source_anchor(self):
        candidate = {'tilt': 35., 'yaw': 30., 'metadata': {'seed_name': 'compact_yaw30_j1zero'}}
        ident = 'human_compact_sport_palm_down'
        guide = next(g for g in get_transport_guides(ident, candidate)
                     if g['metadata']['guide_id'].endswith('current_bridge'))
        boundary = guide['metadata']['phase_boundaries'][0]
        self.assertEqual((boundary['guide_internal_index'], boundary['source_state_index']), (49, 50))
        np.testing.assert_array_equal(boundary['source_anchor_q'], guide['positions'][49])
        boundary['source_anchor_q'][0] = 123.
        reread = get_transport_guides(ident, candidate)[0]
        self.assertNotEqual(reread['metadata']['phase_boundaries'][0]['source_anchor_q'][0], 123.)

    def test_four_palm_guides_inherit_the_same_original_corridor_boundary(self):
        root = Path(__file__).resolve().parents[1]
        asset = json.loads((root / 'assets/planning/placement_guides.json').read_text())
        canonical_id = 'human_compact_sport_palm_down_supported35_yaw30_current_bridge'
        ids = (canonical_id, 'human_default_mpfb_supported25_yaw30_bridge',
               'human_medium_shirt_palm_down_supported25_yaw30_bridge',
               'human_broad_workwear_palm_down_supported25_yaw30_bridge')
        guides = {g['id']: g for g in asset['guides']}
        with np.load(root / guides[canonical_id]['source']['geometry_npz']) as archive:
            canonical = archive['positions'].copy()
        for ident in ids:
            with self.subTest(guide=ident):
                guide = guides[ident]
                boundary = guide['phase_boundaries'][0]
                self.assertEqual((boundary['source_state_index'], boundary['guide_internal_index']), (50, 49))
                with np.load(root / guide['source']['geometry_npz']) as archive:
                    original = archive['positions'].copy()
                np.testing.assert_array_equal(original[1:51], canonical[1:51])
                np.testing.assert_array_equal(guide['positions'], original[1:-1])
                np.testing.assert_array_equal(guide['positions'][49], canonical[50])
                self.assertEqual(boundary['source']['inclusive_source_segments'],
                                 {'transfer': [0, 50], 'above_bowl': [50, len(original)-1]})
                candidate = {'tilt': guide['match']['tilt_deg'], 'yaw': guide['match']['yaw_deg'],
                             'metadata': {'seed_name': guide['match']['seed_names'][0]}}
                loaded = next(g for g in get_transport_guides(guide['obstacle_id'], candidate)
                              if g['metadata']['guide_id'] == ident)
                np.testing.assert_array_equal(
                    loaded['metadata']['phase_boundaries'][0]['source_anchor_q'], canonical[50])


if __name__ == '__main__':
    unittest.main()
