import unittest
import numpy as np
from kitchen_safety.extensions.doors.evidence import openness, predicate_progress, contact_sample
from kitchen_safety.extensions.doors.evidence import mechanism_quality, contact_causality
import copy
import json
from pathlib import Path


class SourcePredicateTests(unittest.TestCase):
    def test_strict_threshold_and_rest_sequence(self):
        self.assertIsNone(predicate_progress([0., .05, .2], 0., 1.))
        self.assertEqual(predicate_progress([0., .06, .200001], 0., 1.), 2)
        self.assertIsNone(predicate_progress([.3], 0., 1.))

    def test_negative_lower_and_reverse_axis_convention(self):
        self.assertEqual(openness(0., -1., 0.), 0.)
        self.assertEqual(openness(-1., -1., 0.), 1.)
        self.assertIsNone(predicate_progress([0., -.06, -.2], -1., 0.))
        self.assertEqual(predicate_progress([0., -.06, -.21], -1., 0.), 2)
        with self.assertRaises(ValueError):
            openness(0., 1., 1.)


class ContactBufferTests(unittest.TestCase):
    def test_only_active_contacts_and_moving_handle_coordinates(self):
        class View:
            def get_contact_data(self, dt):
                assert dt == 1 / 120
                return (np.array([[900.], [-2.], [800.], [3.]]),
                        np.array([[999., 0, 0], [1., 0, 0], [999., 0, 0], [1., .01, 0]]),
                        np.tile([0., 1., 0.], (4, 1)), np.zeros((4, 1)),
                        np.array([[1], [1]]), np.array([[1], [3]]))
        initial = np.eye(4)
        current = np.eye(4); current[0, 3] = 1.
        sample = contact_sample(View(), 1 / 120, np.asarray, initial, current,
                                [[-.02, -.02, -.02], [.02, .02, .02]], .001)
        self.assertEqual(sample['count'], 2)
        self.assertEqual(sample['per_finger_abs_normal_force_n'], [2., 3.])
        self.assertEqual(sample['normal_force_sum_n'], 1.)
        self.assertEqual(sample['points_on_source_handle_bounds'], [True, True])


class ReviewRegressionTests(unittest.TestCase):
    def setUp(self):
        self.limits = json.loads(Path('configs/extensions/doors/review_gates_lwx.json').read_text())
        self.meta = {'physics_dt': 1/120, 'door_joint_limits_rad': [0., 1.],
                     'mechanism_inventory': [{'path': '/kitchen/fridge_main_group',
                                              'joint_names': ['fridge_door_joint'],
                                              'joint_kinds': ['revolute']}]}
        self.samples = []
        for i, q in enumerate([0., 0., .01, .1, .3, .3]):
            loaded = i in (1, 2, 3, 4)
            self.samples.append({'physics_step': i, 'time_s': i/120,
                'phase': 'released_hold' if i == 5 else ('pull_door' if i in (2,3,4) else 'grasp_handle'),
                'state': {'door_q': q},
                'mechanisms': {'/kitchen/fridge_main_group': {'q': [q], 'dq': [0.]}},
                'contacts': {'normal_force_abs_sum_n': 2. if loaded else 0.,
                             'count': 2 if loaded else 0, 'counts': [[int(loaded)], [int(loaded)]],
                             'points_on_source_handle_bounds': [True, True] if loaded else [],
                             'normal_forces_n': [1.,1.] if loaded else [],
                             'per_finger_abs_normal_force_n': [1.,1.] if loaded else [0.,0.]}})

    def test_preceding_loaded_contact_and_release(self):
        self.assertTrue(contact_causality(self.samples, self.meta, self.limits)['passed'])

    def test_same_substep_is_unresolved_not_accepted(self):
        self.samples[1]['contacts'] = copy.deepcopy(self.samples[0]['contacts'])
        report = contact_causality(self.samples, self.meta, self.limits)
        self.assertFalse(report['passed'])
        self.assertEqual(report['order'], 'unresolved_within_same_physics_step')

    def test_neutral_does_not_discard_startup_motion(self):
        report = mechanism_quality(self.samples, self.meta, self.limits, neutral=True)
        self.assertFalse(report['passed'])
        self.assertEqual(report['joints'][0]['max_drift_from_initial'], .3)

    def test_missing_physics_step_rejects_coverage(self):
        del self.samples[2]
        self.assertFalse(mechanism_quality(self.samples, self.meta, self.limits, neutral=False)['passed'])

    def test_zero_impulse_on_one_finger_is_reported_with_contact_intact(self):
        self.samples[3]['contacts']['per_finger_abs_normal_force_n'] = [0., 1.]
        self.samples[3]['contacts']['normal_force_abs_sum_n'] = 1.
        self.samples[3]['contacts']['normal_forces_n'] = [0., 1.]
        report = contact_causality(self.samples, self.meta, self.limits)
        self.assertTrue(report['passed'])
        self.assertEqual(report['single_finger_zero_normal_load_samples'], [3])

    def test_lost_contact_and_total_unloading_reject(self):
        self.samples[3]['contacts']['counts'] = [[0], [1]]
        self.assertFalse(contact_causality(self.samples, self.meta, self.limits)['passed'])


if __name__ == '__main__':
    unittest.main()
