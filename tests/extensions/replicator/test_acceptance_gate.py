"""A clean Kit shutdown must never turn failed or incomplete data into success."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts/extensions/replicator'))
from trial_evidence import acceptance

class AcceptanceGateTest(unittest.TestCase):
    def test_result_and_independent_reports_are_all_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'human' / 'trial'; reports = Path(tmp) / 'reports'
            out.mkdir(parents=True); reports.mkdir()
            (out / 'trajectory.h5').touch()
            statuses = dict(collection=0, export=0, validation=0, audit=0, source_relations=0, fixed_base=0, self_collision=0)
            physical = dict(success=False, arena_success=True, final_on_destination=True,
                            deferred_safety=dict(passed=True), body_grasp_carried_region=dict(passed=True))
            (out / 'result.json').write_text(json.dumps(physical))
            for name in ['videos', 'validation', 'audit']:
                (reports / (name + '.json')).write_text(json.dumps([dict(passed=True)] if name != 'audit' else dict(passed=True)))
            self.assertFalse(acceptance(out, reports, statuses, require_rgb=True)['passed'])
            physical['success'] = True
            (out / 'result.json').write_text(json.dumps(physical))
            self.assertFalse(acceptance(out, reports, statuses, require_rgb=True)['passed'])
            for name in ['source_relation_audit', 'fixed_base_hazard_audit', 'self_collision_audit']:
                (out / (name + '.json')).write_text(json.dumps(dict(passed=True)))
            self.assertTrue(acceptance(out, reports, statuses, require_rgb=True)['passed'])
            physical['overview_camera'] = {'name': 'overview_camera'}
            (out / 'result.json').write_text(json.dumps(physical))
            self.assertFalse(acceptance(out, reports, statuses, require_rgb=True)['passed'])
            statuses['overview'] = 0
            (out / 'overview_validation.json').write_text(json.dumps(dict(passed=True)))
            self.assertTrue(acceptance(out, reports, statuses, require_rgb=True)['passed'])
            (out / 'source_relation_audit.json').write_text(json.dumps(dict(passed=False)))
            self.assertFalse(acceptance(out, reports, statuses, require_rgb=True)['passed'])
            (out / 'source_relation_audit.json').write_text(json.dumps(dict(passed=True)))
            (reports / 'audit.json').unlink()
            self.assertFalse(acceptance(out, reports, statuses, require_rgb=True)['passed'])
            (reports / 'audit.json').write_text(json.dumps(dict(passed=True)))
            del statuses['audit']
            self.assertFalse(acceptance(out, reports, statuses, require_rgb=True)['passed'])

    def test_physical_success_without_causal_avoidance_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'avoidance_trials'/'replicator'/'robot'/'trial';reports=Path(tmp)/'reports'
            out.mkdir(parents=True);reports.mkdir();(out/'trajectory.h5').touch()
            physical=dict(success=True,arena_success=True,final_on_destination=True,deferred_safety=dict(passed=True),body_grasp_carried_region=dict(passed=True))
            (out/'result.json').write_text(json.dumps(physical))
            statuses={key:0 for key in ['collection','source_relations','fixed_base','export','validation','audit','self_collision','background_consistency','avoidance']}
            for name in ['source_relation_audit','fixed_base_hazard_audit','self_collision_audit','background_consistency_audit']:(out/(name+'.json')).write_text(json.dumps(dict(passed=True)))
            for name in ['videos','validation','audit']:(reports/(name+'.json')).write_text(json.dumps(dict(passed=True)))
            self.assertFalse(acceptance(out,reports,statuses,require_rgb=True)['passed'])
            (out/'avoidance_audit.json').write_text(json.dumps(dict(accepted=False)))
            self.assertFalse(acceptance(out,reports,statuses,require_rgb=True)['passed'])
            (out/'avoidance_audit.json').write_text(json.dumps(dict(accepted=True)))
            self.assertTrue(acceptance(out,reports,statuses,require_rgb=True)['passed'])
            (out/'background_consistency_audit.json').write_text(json.dumps(dict(passed=False)))
            self.assertFalse(acceptance(out,reports,statuses,require_rgb=True)['passed'])
            (out/'background_consistency_audit.json').write_text(json.dumps(dict(passed=True)))
            statuses.pop('avoidance')
            self.assertFalse(acceptance(out,reports,statuses,require_rgb=True)['passed'])

if __name__ == '__main__': unittest.main()
