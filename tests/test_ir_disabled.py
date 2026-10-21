"""IR opt-out must skip rules and preserve active grades and export semantics."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from kitchen_safety.risk_profile_loader import load_config, create_evaluator
from kitchen_safety.risk_observer import aggregate_known_results, arena_coverage_report, unknown_report
from kitchen_safety.risk_export import risk_arrays

class IRDisabledTests(unittest.TestCase):
    def test_current_config_disables_ir(self):
        settings=load_config('configs/safety_observer_arena.json')
        self.assertEqual(settings['profile']['rules']['disabled_categories'], ['IR'])
        self.assertEqual(settings['config']['disabled_categories'], ['IR'])

    def test_evaluator_skips_ir_without_changing_active_rules(self):
        profile=load_config('configs/safety_observer_arena.json')['profile']
        snapshot={'features':{}, 'metadata':profile['binding']}
        disabled=create_evaluator(profile).evaluate(snapshot)
        enabled_profile=deepcopy(profile);enabled_profile['rules'].pop('disabled_categories')
        enabled=create_evaluator(enabled_profile).evaluate(snapshot)
        self.assertEqual(disabled['categories']['IR']['status'],'disabled')
        self.assertIsNone(disabled['categories']['IR']['level'])
        self.assertFalse(any(r['category']=='IR' for r in disabled['rules']))
        self.assertEqual(disabled['rules'],[r for r in enabled['rules'] if r['category']!='IR'])
        self.assertTrue(any(r['category']=='IR' for r in enabled['rules']))

    def test_disabled_does_not_reduce_completeness(self):
        report=unknown_report('fixture')
        for c in ('HS','PT','RS'):
            report['categories'][c]=dict(level='L0',data_complete=True,missing=[],status='exact')
        report['categories']['IR']=dict(level=None,data_complete=None,missing=[],status='disabled')
        report['rules']=[dict(rule_id=c+'-TEST',category=c,level='L0',complete=True,missing=[]) for c in ('HS','PT','RS')]
        result=aggregate_known_results(report)
        self.assertTrue(result['data_complete']);self.assertEqual(result['risk_levels']['overall'],'L0')
        self.assertEqual(result['categories']['IR']['status'],'disabled')

    def test_error_still_marks_ir_disabled(self):
        result=arena_coverage_report(unknown_report('error'),{}, {},'error',['IR'])
        self.assertEqual(result['categories']['IR']['status'],'disabled')
        self.assertEqual(result['categories']['IR']['missing'],[])
        self.assertIsNone(result['risk_levels']['overall'])
        self.assertFalse(result['data_complete'])

    def test_invalid_disabled_category_rejected(self):
        profile=load_config('configs/safety_observer_arena.json')['profile']
        for bad in ('IR', ['HS'], ['IR','IR'], [{}]):
            profile['rules']['disabled_categories']=bad
            with self.assertRaises(ValueError):create_evaluator(profile)

    def test_export_enabled_mask(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            (p/'safety_observer_binding.json').write_text(json.dumps(dict(action_dt_s=1/15,physics_dt_s=1/120,decision_stride=4)))
            report=arena_coverage_report(unknown_report('error'),{}, {},'error',['IR'])
            (p/'safety_decisions.jsonl').write_text(json.dumps(dict(decision_physics_step=0,source_physics_step=-1,observation_id=0,report=report))+'\n')
            data=risk_arrays(p,1,0)
            self.assertEqual(data['risk.enabled'].tolist(),[[1,1,1,0]])
            self.assertEqual(data['risk.level'].tolist(),[[-1,-1,-1,-1]])
            self.assertEqual(data['risk.data_complete'].tolist(),[[0,0,0,0]])

if __name__=='__main__':unittest.main()
