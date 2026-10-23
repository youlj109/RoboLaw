import unittest
from types import SimpleNamespace
from kitchen_safety.extensions.doors.pregrasp import check_pregrasp, POLICY

class PregraspGateTests(unittest.TestCase):
    def run_gate(self, **changes):
        report = dict(valid=True, validation_complete=True, fcl_samples_unchecked=0)
        report.update(changes)
        seen = {}
        def validate_path(positions, **kwargs):
            seen.update(kwargs)
            return report
        result = check_pregrasp(SimpleNamespace(validate_path=validate_path), {'positions': [[0.]*7]})
        self.assertEqual(seen['resolution'], .005)
        self.assertEqual(seen['phase'], 'door_pregrasp')
        self.assertEqual(seen['grip_binary'], 0.)
        self.assertNotIn('accepted', report)
        return result
    def test_complete_safe_path_accepted(self):
        self.assertTrue(self.run_gate()['accepted'])
    def test_unsafe_path_rejected(self):
        self.assertFalse(self.run_gate(valid=False)['accepted'])
    def test_incomplete_independent_check_rejected(self):
        self.assertFalse(self.run_gate(validation_complete=False)['accepted'])
    def test_unchecked_samples_rejected(self):
        self.assertFalse(self.run_gate(fcl_samples_unchecked=1)['accepted'])

if __name__ == '__main__': unittest.main()
