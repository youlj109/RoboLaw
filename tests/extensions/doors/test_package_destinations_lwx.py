"""Regression: independently matched baselines may share episode basenames."""
import importlib.util
from pathlib import Path
import unittest
SPEC = importlib.util.spec_from_file_location('package_delivery_lwx', Path(__file__).resolve().parents[3] / 'scripts/extensions/doors/package_delivery_evidence_lwx.py')
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)

class DestinationTests(unittest.TestCase):
    def test_unique_historical_names_preserved(self):
        roots = [Path('datasets/one/C02_human_08'), Path('datasets/two/C02_robot_26')]
        self.assertEqual(module.episode_destinations(roots), {p: p.name for p in roots})

    def test_matched_baselines_with_same_name_stay_distinct(self):
        roots = [Path('datasets/release/C02_base_01'), Path('datasets/settle/C02_base_01')]
        names = module.episode_destinations(roots)
        self.assertEqual(len(set(names.values())), 2)
        self.assertTrue(names[roots[0]].startswith('release__C02_base_01__'))
        self.assertTrue(names[roots[1]].startswith('settle__C02_base_01__'))
        self.assertEqual(names, module.episode_destinations(reversed(roots)))

    def test_identical_parent_names_do_not_merge(self):
        roots = [Path('a/data/base'), Path('b/data/base')]
        self.assertEqual(len(set(module.episode_destinations(roots).values())), 2)

    def test_repeated_identical_episode_copies_once(self):
        p = Path('datasets/one/base')
        self.assertEqual(module.episode_destinations([p, p]), {p: 'base'})

if __name__ == '__main__':
    unittest.main()
