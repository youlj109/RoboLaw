"""Negative controls for bounded/resumable collection acceptance."""
import copy
import importlib.util
import json
import unittest
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/extensions/doors/run_fixed_sample_lwx.py'
spec = importlib.util.spec_from_file_location('fixed_sample_runner', SOURCE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def report():
    return {'stable_policy_necessity_in_tested_seeds': True, 'fixed_endpoint_rerouting_supported': True,
            'pairs': [{'seed': seed,
                       'off': {'success': True, 'path_metrics': {'joint_l2_length_rad': 1., 'tcp_length_m': .4},
                               'risk_with_added_geometry': {'violations': [{'intersecting': True}]}},
                       'on': {'success': True, 'path_metrics': {'joint_l2_length_rad': 1.1, 'tcp_length_m': .42}}}
                      for seed in [42, 43, 44]]}


class FixedRunTests(unittest.TestCase):
    def test_actual_hits_and_small_reroute_pass(self):
        self.assertTrue(runner.paired_accepted(report()))

    def test_margin_only_seed_cannot_be_promoted(self):
        value = report()
        value['pairs'][1]['off']['risk_with_added_geometry']['violations'] = [{'intersecting': False}]
        self.assertFalse(runner.paired_accepted(value))

    def test_missing_or_duplicate_seeds_are_rejected(self):
        for seeds in [[42, 43], [42, 42, 44], [44, 43, 42]]:
            value = report()
            value['pairs'] = [dict(copy.deepcopy(value['pairs'][0]), seed=seed) for seed in seeds]
            self.assertFalse(runner.paired_accepted(value))

    def test_excessive_joint_or_tcp_path_is_rejected(self):
        for key, value in [('joint_l2_length_rad', 1.601), ('tcp_length_m', .551)]:
            document = report()
            document['pairs'][2]['on']['path_metrics'][key] = value
            self.assertFalse(runner.paired_accepted(document))

    def test_three_identical_failures_require_diagnosis(self):
        self.assertFalse(runner.repeated_failure([{'failure': 'contact'}] * 2))
        self.assertTrue(runner.repeated_failure([{'failure': 'contact'}] * 3))
        self.assertFalse(runner.repeated_failure([{'failure': 'contact'}, {'failure': 'native crash'}, {'failure': 'contact'}]))
        self.assertFalse(runner.repeated_failure([{}] * 3))

    def test_release_policy_is_preserved_and_adaptation_is_rejected(self):
        policy = dict(seed=42, handle_offset_z=.06, release_retreat_m=.01, release_settle_steps=45, pull_steps=1100)
        args = runner.collection_arguments('task', 'human', 'config', policy, 'review', 'output', 'collect')
        self.assertEqual(args[args.index('--release-retreat-m') + 1], '0.01')
        self.assertEqual(args[args.index('--release-settle-steps') + 1], '45')
        self.assertEqual(args[args.index('--pull-steps') + 1], '1100')
        policy['handle_height_candidates_m'] = [0., .06]
        with self.assertRaises(ValueError):
            runner.collection_arguments('task', 'human', 'config', policy, 'review', 'output', 'collect')

    def test_collision_signature_ignores_numerical_attempt_variations(self):
        failures = []
        for action, distance in [(263, .00196), (264, .00190), (262, .00188)]:
            detail = dict(action_index=action, phase='release_handle', failure=dict(robot_part='finger', obstacle='/door/panel', distance_m=distance))
            signature = runner.failure_signature('collect', 'RuntimeError: EXECUTION_COLLISION ' + json.dumps(detail))
            failures.append(dict(failure_signature=signature))
        self.assertTrue(runner.repeated_failure(failures))


if __name__ == '__main__':
    unittest.main()
