"""Final explanations reject legacy source rows; diagnostics remain explicit."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from scripts.extensions.replicator.render_geometry_explanation import bind_report, require_matched_controls
from scripts.extensions.replicator.review_episode import sha


class GeometryExplanationBindingTest(unittest.TestCase):
    def test_profiles_do_not_hide_changed_actual_controls(self):
        base = dict(variant='base', effective_controls=dict(profile='base', seed=42, collector_options={'height': 1.085}))
        actual = dict(variant='property', effective_controls=dict(profile='property', seed=42, collector_options={'height': 1.085}))
        require_matched_controls(base, actual)
        actual['effective_controls']['collector_options']['height'] = 1.1
        with self.assertRaisesRegex(ValueError, 'Controls differ'):
            require_matched_controls(base, actual)
        actual['effective_controls']['collector_options']['height'] = 1.085
        base['effective_controls']['profile'] = 'human'
        with self.assertRaisesRegex(ValueError, 'profile identities'):
            require_matched_controls(base, actual)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root/'result.json').write_text('{}')
        self.layout = self.root/'layout.json'; self.layout.write_text('{}')
        source = self.root/'source.usda'; source.write_text('synthetic CPU fixture, not real geometry')
        np.savez(self.root/'physics_states.npz', q=np.zeros((1, 1)), phase=['carry'], action_index=[1])
        self.record = dict(episode=self.root)
        self.report = dict(passed=True, expected='safe', episode=str(self.root),
            result_sha256=sha(self.root/'result.json'), physics_sha256=sha(self.root/'physics_states.npz'),
            layout_sha256=sha(self.layout), code_sha256={}, samples=1,
            source=dict(hazard_layers=[], bottle_mesh=dict(source=str(source), sha256=sha(source)),
                        robot_source_mesh=dict(source=str(source), sha256=sha(source))),
            rows=[dict(physics_index=0, action_index=1, phase='carry')], full_reset_to_end_source_geometry=False)
        self.path = self.root/'audit.json'; self.path.write_text(json.dumps(self.report))

    def test_default_delivery_refuses_old_motion_only_source_even_if_passed_true(self):
        with self.assertRaisesRegex(ValueError, 'omits reset/startup'):
            bind_report(self.path, self.record, self.layout, 'safe')
        _, _, groups = bind_report(self.path, self.record, self.layout, 'safe', require_complete=False)
        self.assertEqual(list(groups), [1])

    def test_exact_source_state_and_expectation_binding_cannot_be_bypassed(self):
        with self.assertRaisesRegex(ValueError, 'wrong expectation'):
            bind_report(self.path, self.record, self.layout, 'risk', require_complete=False)
        (self.root/'result.json').write_text('{"changed": true}')
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            bind_report(self.path, self.record, self.layout, 'safe', require_complete=False)

    def test_full_coverage_validator_runs_before_render_and_keeps_sample_mapping(self):
        with patch('scripts.extensions.replicator.build_delivery_manifest.verify_source_geometry_coverage') as verify:
            bind_report(self.path, self.record, self.layout, 'safe')
            self.assertEqual(verify.call_args.args[1:], (self.report, self.root, 'safe'))
        self.report['rows'][0]['action_index'] = 2
        self.path.write_text(json.dumps(self.report))
        with self.assertRaisesRegex(ValueError, 'do not align'):
            bind_report(self.path, self.record, self.layout, 'safe', require_complete=False)


if __name__ == '__main__':
    unittest.main()
