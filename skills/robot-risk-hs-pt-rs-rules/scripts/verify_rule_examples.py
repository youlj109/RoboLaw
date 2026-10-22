#!/usr/bin/env python3
"""Check portable examples against frozen rule functions, without simulator/GPU.

No task files are modified. Selected original AST definitions avoid importing
Isaac Sim/numpy feature adapters; the numerical rule bodies are not rewritten.
This is not an end-to-end validation of a new scene or another implementation.
"""
import ast
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
REF = ROOT / 'assets/frozen-implementation'


def load(path, names, namespace):
    tree = ast.parse(path.read_text())
    nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names]
    if {n.name for n in nodes} != set(names):
        raise ValueError('Frozen rule definition missing')
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), namespace)


def main():
    manifest = json.loads((REF / 'SOURCE_MANIFEST.json').read_text())
    for item in manifest['files']:
        if hashlib.sha256((REF / item['path']).read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('Frozen source hash mismatch: ' + item['path'])
    sys.path.insert(0, str(REF))
    from safety_risk.evaluator_factory import create_evaluator
    from safety_risk.pt_hs_inspired import evaluate_pairs
    profile = json.loads((REF / 'profile.reference.json').read_text())
    profile['rules']['disabled_categories'] = ['IR']
    profile['runtime_binding'] = dict(human_inventory=dict(verified_absent=False),
                                    hs_geometry=dict(dynamic_relation_ids=['example-relation']))
    hns = dict(math=math, deepcopy=deepcopy, VERSION='arena-hs-dynamic-ssm-1')
    load(REF / 'arena_adapters/hs_dynamic.py', ['validate_config', 'relation', 'DynamicHSEvaluator'], hns)
    pns = dict(math=math, deepcopy=deepcopy, VERSION='arena-pt-scene-assets-1')
    load(REF / 'arena_adapters/pt_assets.py', ['apply_support_distance_rules'], pns)
    ons = dict(deepcopy=deepcopy, CATEGORIES=('HS', 'PT', 'RS', 'IR'),
               apply_support_distance_rules=pns['apply_support_distance_rules'])
    load(REF / 'arena_adapters/risk_observer.py', ['aggregate_known_results', 'arena_coverage_report'], ons)
    base = create_evaluator(profile)
    dynamic = hns['DynamicHSEvaluator'](base, profile)

    def snapshot():
        return dict(metadata=deepcopy(profile['binding']), features=dict(
            hs=dict(d_robot_h_min_gt_m=1., d_ee_h_min_gt_m=1., d_obj_h_min_gt_m=1.,
                    v_rel_h_gt_mps=0., human_contact_flag_gt=False),
            pt=dict(drop_flag_gt=False, protected_pairs=[]), rs={}))

    def rule(report, rid):
        return next(r for r in report['rules'] if r['rule_id'] == rid)

    def pt_pair(i, omitted=False):
        p = deepcopy(profile)
        actor = i.get('actor', 'robot')
        key = dict(actor=actor, object_id='protected_obstacle')
        p['pt_hs_inspired']['pairs'] = [key]
        if omitted:
            p['pt_hs_inspired']['distance_omissions'] = dict(
                version='arena-pt-distance-omission-1', status='omitted_by_user_policy',
                scope='distance_only_all_phases',
                authorization='Example-only explicitly omitted relation',
                selection_report_sha256='0'*64, relations=[key])
        source = 'pt_environment_gdist_surface_distance_gt'
        raw = dict(**key, distance_m=i.get('D_m'), distance_source=source,
                   distance_metric='gdist_authored_collision_surface_distance',
                   distance_coverage_complete=True, held_contact_active=i.get('held', True),
                   distance_geometry_evidence=dict(distance_source=source,
                       metric='gdist_authored_collision_surface_distance', coverage_complete=True),
                   force_n=i.get('measured_N'), impulse_ns=i.get('measured_Ns'),
                   impulse_semantics=i.get('statistic'))
        return p, raw, evaluate_pairs([raw], p)

    results = []
    for case in json.loads((ROOT / 'assets/conformance-cases.json').read_text())['cases']:
        rid, i, expected = case['rule_id'], case['inputs'], case['expected']
        s = snapshot()
        observed = None
        extra = {}
        if rid in ('HS-DIST-001', 'HS-category'):
            row = dict(relation_id='example-relation', source_group='robot', source_actor='example-link',
                       human_collider='example-human', distance_m=i['D_m'],
                       robot_surface_speed_mps=i.get('source_surface_speed_mps', 0.),
                       human_surface_speed_mps=i.get('human_surface_speed_mps', 0.),
                       human_static=i.get('human_static', True),
                       stop_model=i.get('stop_model', 'robot_coupled'), complete=True)
            s['features']['hs']['dynamic_distance'] = dict(version=hns['VERSION'], pairs=[row])
            out = dynamic.evaluate(s)
            observed = (out['risk_levels']['HS'] if rid == 'HS-category' else rule(out, rid)['level'])
        elif rid == 'HS-COLL-001':
            s['features']['hs']['human_contact_flag_gt'] = i['valid_contact']
            observed = rule(base.evaluate(s), rid)['level']
        elif rid == 'HS-TTC-001':
            s['features']['hs']['TTC_h_min_gt_s'] = i['TTC_s']
            out = rule(dynamic.evaluate(s), rid)
            observed = out['level'];extra['evaluation_status'] = out['evaluation_status']
        elif rid in ('PT-DIST-001', 'PT-FORCE-002', 'PT-IMP-002'):
            p, raw, rows = pt_pair(i, i.get('omitted_by_policy', False))
            selected = next((r for r in rows if r[0] == rid), None)
            if i.get('task_goal_relation_excluded'):
                report = dict(rules=[dict(rule_id=rid, category='PT', level=selected[1],
                    evidence=selected[2], complete=not selected[3], missing=selected[3])],
                    categories=dict(PT={}), risk_levels={})
                inventory = dict(version=pns['VERSION'], objects={'protected_obstacle': {}},
                    excluded_pt_relations=[dict(actor='robot', object_id='protected_obstacle', scope='all_pair_rules_all_phases')],
                    allowed_support_relations=[])
                pns['apply_support_distance_rules'](report, dict(pt=dict(protected_pairs=[raw])),
                                                    dict(pt_asset_inventory=inventory))
                out = report['rules'][0];observed = out['level'];extra['evaluation_status'] = out['evaluation_status']
            else:
                observed = selected[1] if selected else None
            if i.get('omitted_by_policy'):extra['rule_emitted'] = selected is not None
        elif rid in ('PT-DROP-001', 'PT-DMG-001', 'PT-SUP-001'):
            s['features']['pt'].update(drop_flag_gt=i.get('drop'), h_drop_gt_m=i.get('h_m'),
                damage_flag_gt=i.get('damage'), support_margin_gt_m=i.get('support_margin_m'))
            out = ons['aggregate_known_results'](base.evaluate(s))
            observed = rule(out, rid)['level']
        elif rid.startswith('RS-'):
            r = s['features']['rs']
            if rid == 'RS-TORQ-001':r['joint_torque_ratio_gt'] = i['ratio']
            elif rid == 'RS-JOINT-001':r['joint_limit_margin_gt_rad'] = i['margin_rad']
            elif rid == 'RS-FAULT-001':r['motion_after_fault_gt'] = i.get('motion_after_observed_fault')
            else:
                r['self_collision_flag_gt'] = i.get('self_collision', bool(i.get('nonallowed_pairs')))
                s['features']['_existing_self_collision_pairs'] = i.get('pairs', i.get('nonallowed_pairs', []))
            observed = rule(base.evaluate(s), rid)['level']
        elif rid == 'category_aggregation':
            rr = [dict(rule_id='HS-example', category='HS', level=x,
                complete=i.get('complete', [True]*len(i['rule_levels']))[j], missing=[],
                note=i.get('notes', [None]*len(i['rule_levels']))[j]) for j, x in enumerate(i['rule_levels'])]
            out = ons['aggregate_known_results'](dict(rules=rr,
                categories=dict(HS=dict(data_complete=all(r['complete'] for r in rr), missing=[])),
                risk_levels=dict(HS=None)))
            observed = out['risk_levels']['HS'];extra['data_complete'] = out['categories']['HS']['data_complete']
        elif rid == 'HS-applicability':
            out = ons['arena_coverage_report'](base.evaluate(s), s['features'],
                dict(human_inventory=dict(verified_absent=True)), disabled_categories=['IR'])
            observed = out['risk_levels']['HS']
        elif rid == 'IR':
            out = base.evaluate(s);observed = out['risk_levels']['IR']
        else:
            raise ValueError('Unknown example: ' + rid)
        matches = observed == expected['level'] and all(extra[k] == expected[k] for k in extra if k in expected)
        results.append(dict(case_id=case['case_id'], expected=expected['level'], observed=observed, passed=matches))
    failed = [r for r in results if not r['passed']]
    print(json.dumps(dict(cases=len(results), passed=len(results)-len(failed), failed=failed,
        scope='Frozen rule examples and source hashes only; no simulation/GPU/new-scene validation'), ensure_ascii=False, indent=2))
    return bool(failed)


if __name__ == '__main__':
    raise SystemExit(main())
