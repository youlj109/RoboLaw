"""Distance pruning must preserve PT bands and fail closed on incomplete evidence."""
from copy import deepcopy
from types import SimpleNamespace
import itertools
import unittest
from unittest.mock import Mock, patch
import numpy as np

from kitchen_safety.pt_geometry import PropertyDistanceGeometry, ENV_SOURCE, METRIC
from kitchen_safety.pt_pruning import VERSION, NUMERICAL_MARGIN_M, warning_outer_m
from kitchen_safety.risk_profile_loader import load_config
from kitchen_safety.pt_assets import apply_support_distance_rules, VERSION as ASSET_VERSION
from safety_risk.pt_hs_inspired import evaluate_pairs


class PTPruningTests(unittest.TestCase):
    def setUp(self):
        self.profile = deepcopy(load_config('configs/safety_observer_arena.json')['profile'])
        pt = self.profile['pt_hs_inspired']
        pt['objects']['wall'] = pt['objects']['protected_obstacle']
        pt['pairs'] = [dict(actor='held_object', object_id='wall')]
        self.threshold = warning_outer_m(pt['objects']['wall'])
        self.raw = dict(actor='held_object', object_id='wall', distance_m=None,
            distance_source=ENV_SOURCE, distance_metric=METRIC, distance_coverage_complete=True,
            held_contact_active=True, force_n=0., impulse_ns=0.,
            distance_geometry_evidence=dict(actor='held_object', object_id='wall',
                distance_source=ENV_SOURCE, metric=METRIC, coverage_complete=True,
                distance_status='lower_bound', distance_bound=dict(version=VERSION,
                    method='complete_actor_asset_union_aabb', lower_bound_m=.5,
                    numerical_margin_m=NUMERICAL_MARGIN_M)))
        self.binding = dict(pt_asset_inventory=dict(version=ASSET_VERSION,
            objects={'wall': {}}, allowed_support_relations=[]))

    def report(self, raw):
        rules = [dict(rule_id=k, category='PT', level=level, evidence=e,
                      missing=m, complete=not m) for k, level, e, m in evaluate_pairs([raw], self.profile)]
        result = dict(rules=rules, categories={'PT': {}}, risk_levels={})
        apply_support_distance_rules(result, {'pt': {'protected_pairs': [raw]}}, self.binding)
        return result

    def test_far_certificate_matches_exact_and_preserves_other_rules(self):
        exact = deepcopy(self.raw); exact['distance_m'] = .6
        bounded, baseline = self.report(self.raw), self.report(exact)
        self.assertEqual([(r['level'], r['complete']) for r in bounded['rules']],
                         [(r['level'], r['complete']) for r in baseline['rules']])
        self.assertEqual(bounded['rules'][1:], baseline['rules'][1:])
        self.assertIsNone(bounded['rules'][0]['evidence']['distance_m'])
        self.assertEqual(bounded['rules'][0]['evidence']['warning_outer_m'], self.threshold)
        self.assertEqual(bounded['arena_pt_distance_summary']['bounded_pair_count'], 1)
        self.assertIsNone(bounded['arena_pt_distance_summary']['nearest_pair'])

    def test_boundary_and_bad_or_missing_evidence_stay_unknown(self):
        variants = []
        for value in (self.threshold, 0., float('nan'), float('inf'), True):
            raw = deepcopy(self.raw); raw['distance_geometry_evidence']['distance_bound']['lower_bound_m'] = value
            variants.append(raw)
        for field, value in (('held_contact_active', None), ('distance_source', 'wrong'),
                             ('distance_coverage_complete', False)):
            raw = deepcopy(self.raw); raw[field] = value; variants.append(raw)
        for field, value in (('coverage_complete', False), ('distance_bound', None), ('metric', 'wrong')):
            raw = deepcopy(self.raw); raw['distance_geometry_evidence'][field] = value; variants.append(raw)
        for raw in variants:
            with self.subTest(raw=raw):
                rule = self.report(raw)['rules'][0]
                self.assertIsNone(rule['level']); self.assertFalse(rule['complete'])

    def test_changed_warning_boundary_rechecked(self):
        raw = deepcopy(self.raw)
        raw['distance_geometry_evidence']['distance_bound']['lower_bound_m'] = .05
        self.profile['pt_hs_inspired']['objects']['wall']['distance']['warning_band_m'] = .06
        self.assertIsNone(self.report(raw)['rules'][0]['level'])

    def test_inactive_payload_remains_inapplicable(self):
        raw = deepcopy(self.raw); raw['held_contact_active'] = False
        report = self.report(raw)
        self.assertEqual(report['rules'][0]['level'], 'L0')
        self.assertEqual(report['arena_pt_distance_summary']['valid_applicable_pair_count'], 0)

    def test_nearest_pair_requires_proof_against_skipped_distances(self):
        pt = self.profile['pt_hs_inspired']
        pt['objects']['near'] = deepcopy(pt['objects']['wall'])
        pt['pairs'].append(dict(actor='robot', object_id='near'))
        exact = deepcopy(self.raw)
        exact.update(actor='robot', object_id='near', distance_m=.6)
        features = {'pt': {'protected_pairs': [self.raw, exact]}}
        for distance, expected in ((.6, None), (.03, 'near')):
            exact['distance_m'] = distance
            rules = [dict(rule_id=k, category='PT', level=level, evidence=e,
                missing=m, complete=not m) for k, level, e, m in evaluate_pairs(features['pt']['protected_pairs'], self.profile)]
            report = dict(rules=rules, categories={'PT': {}}, risk_levels={})
            apply_support_distance_rules(report, features, self.binding)
            summary = report['arena_pt_distance_summary']
            nearest = summary['nearest_pair']
            self.assertEqual(nearest['object_id'] if nearest else None, expected)
            self.assertEqual(summary['nearest_exact_pair']['object_id'], 'near')

    def test_near_distances_and_threshold_equality_still_trigger(self):
        for d, level in ((0., 'L2'), (.03, 'L1'), (self.threshold, 'L1')):
            raw = deepcopy(self.raw); raw['distance_m'] = d
            self.assertEqual(self.report(raw)['rules'][0]['level'], level)

    def test_motion_recomputes_bounds_and_missing_geometry_cannot_skip(self):
        geo = PropertyDistanceGeometry.__new__(PropertyDistanceGeometry)
        geo.root = '/env'; geo.target = 'bottle'; geo.assets = {'bottle': {}, 'wall': {}}
        geo.errors = []; geo.warning_thresholds = {'bottle': self.threshold, 'wall': self.threshold}
        geo.fcl = SimpleNamespace(Transform=lambda *args: args)
        geo.entries = []
        for group, x in (('robot', 0.), ('bottle', .2), ('wall', 2.)):
            corners = np.asarray(list(itertools.product((0., .1), repeat=3)))
            corners[:, 0] += x
            geo.entries.append(dict(group=group, actor=group, key=group, path=group,
                dynamic=group == 'robot', corners=corners, last_pose=None, object=Mock()))
        with patch('kitchen_safety.pt_geometry.minimum_distance', return_value={'distance_m': .01}) as exact:
            first = geo.sample({'robot': [0., 0., 0., 0., 0., 0., 1.]}, {})
            self.assertEqual(exact.call_count, 1)  # Robot-target is retained even if far.
            self.assertEqual(first['pairs'][1]['distance_status'], 'lower_bound')
            exact.reset_mock()
            moved = geo.sample({'robot': [1.89, 0., 0., 0., 0., 0., 1.]}, {})
            self.assertEqual(exact.call_count, 2)
            self.assertEqual(moved['pairs'][1]['distance_status'], 'exact')
            failed = geo.sample({}, {})
            self.assertFalse(failed['pairs'][1]['coverage_complete'])
            self.assertIsNone(failed['pairs'][1]['distance_bound'])


if __name__ == '__main__':
    unittest.main()
