"""Exercise the production task rule adapter with explicit synthetic evidence.

Portable rule boundaries are separate from geometry/scene validation. No
simulation, GPU queries or original episode rewrites are performed here.
"""
import argparse
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest
from kitchen_safety.risk_observer import aggregate_known_results
from kitchen_safety.pt_assets import apply_support_distance_rules

METRIC='gdist_authored_collision_surface_distance'


def fixture(Rules,profile,*,actor='robot',omitted=False,absent=False,operation=False):
    source='/fixture/payload' if actor=='held_object' else '/fixture/robot/link'
    hs=dict(relation_id='HS:fixture',category='HS',source='/fixture/robot/link',source_role='robot',
        source_colliders=['/fixture/robot/link/collider'],target='/fixture/human/collider',target_colliders=['/fixture/human/collider'],query_distance=True)
    pt=dict(relation_id='PT:fixture',category='PT',source=source,source_role='payload' if actor=='held_object' else 'robot',
        source_colliders=[source+'/collider'],target='/fixture/property',target_colliders=['/fixture/property/collider'],
        query_distance=not omitted,evidence_only=False,reason='explicit synthetic distance-only scope fixture')
    if operation:
        pt['operation_distance_omissions']=[dict(operation_id='fixture_operation',allowed_phases=['operate'],
            target_colliders=pt['target_colliders'],declaration_sha256='fixture_surface_proof')]
    scope=dict(relations=[pt]+([] if absent else [hs]),human_inventory=dict(complete=True,verified_absent=absent))
    scope['scope_sha256']=digest(scope)
    rules=Rules(dict(task_id='rule_contract_fixture',roles=dict(payload_ids=['fixture_payload'])),scope,dict(allowed_pairs=[]),dict(profile=profile,profile_sha256=digest(profile)))
    def geometry(r,d):
        return dict(distance_m=d,metric=METRIC,complete=True,source_collider=r['source_colliders'][0],
            destination_collider=r['target_colliders'][0],effective_target_colliders=r['target_colliders'])
    h=dict(relation_id=hs['relation_id'],source_actor=hs['source'],source_group='robot',human_collider=hs['target'],
        distance_m=1.,robot_surface_speed_mps=0.,human_surface_speed_mps=0.,human_static=True,
        stop_model='robot_coupled',complete=True,missing=[],nearest_geometry=geometry(hs,1.))
    p=dict(relation_id=pt['relation_id'],source=source,target=pt['target'],source_role=pt['source_role'],
        distance_m=1.,distance_coverage_complete=True,distance_geometry_evidence=geometry(pt,1.),
        source_surface_speed_mps=0.,target_surface_speed_mps=0.,force_n=None,impulse_ns=None,
        impulse_semantics='max_body_pair_step_vector_norm_over_four_steps',task_phase='transport',held_contact_active=True)
    features=dict(hs=dict(dynamic_distance=dict(version=profile['hs_dynamic']['version'],pairs=[] if absent else [h]),human_contact_flag_gt=False),
        pt=dict(payloads={'fixture_payload':dict(drop_flag_gt=None,h_drop_gt_m=None)},protected_pairs=[p]),
        rs=dict(joint_limit_margin_gt_rad=None,joint_torque_ratio_gt=None,motion_after_fault_gt=None,
            self_collision_flag_gt=None,self_collision_evidence=dict(coverage_complete=True)),_existing_self_collision_pairs=None)
    return rules,features


def select(report,rid):
    return next((r for r in report['rules'] if r['rule_id']==rid),None)


def portable(Rules,profile,case):
    rid=case['rule_id'];i=case['inputs'];expected=case['expected']
    rules,f=fixture(Rules,profile,actor=i.get('actor','robot'),omitted=i.get('omitted_by_policy',False),absent=rid=='HS-applicability')
    h=f['hs']['dynamic_distance']['pairs'];p=f['pt']['protected_pairs'][0]
    if rid in ('HS-DIST-001','HS-category'):
        h[0].update(distance_m=i['D_m'],robot_surface_speed_mps=i.get('source_surface_speed_mps',0.),
            human_surface_speed_mps=i.get('human_surface_speed_mps',0.),human_static=i.get('human_static',True),
            stop_model=i.get('stop_model','robot_coupled'))
        h[0]['nearest_geometry']['distance_m']=i['D_m'];f['hs']['human_contact_flag_gt']=i.get('contact',False)
    elif rid=='HS-COLL-001':f['hs']['human_contact_flag_gt']=i['valid_contact']
    elif rid=='HS-TTC-001':f['hs']['TTC_h_min_gt_s']=i['TTC_s']
    elif rid in ('PT-DIST-001','PT-FORCE-002','PT-IMP-002'):
        p.update(distance_m=i.get('D_m'),force_n=i.get('measured_N'),impulse_ns=i.get('measured_Ns'),
            held_contact_active=i.get('held',True),impulse_semantics=i.get('statistic'))
        p['distance_geometry_evidence']['distance_m']=p['distance_m']
    elif rid in ('PT-DROP-001','PT-DMG-001','PT-SUP-001'):
        f['pt']['payloads']['fixture_payload'].update(drop_flag_gt=i.get('drop'),h_drop_gt_m=i.get('h_m'))
        f['pt'].update(damage_flag_gt=i.get('damage'),support_margin_gt_m=i.get('support_margin_m'))
    elif rid.startswith('RS-'):
        r=f['rs']
        if rid=='RS-TORQ-001':r['joint_torque_ratio_gt']=i['ratio']
        elif rid=='RS-JOINT-001':r['joint_limit_margin_gt_rad']=i['margin_rad']
        elif rid=='RS-FAULT-001':r['motion_after_fault_gt']=i.get('motion_after_observed_fault')
        else:
            r['self_collision_flag_gt']=i.get('self_collision',bool(i.get('nonallowed_pairs')))
            r['self_collision_evidence']['coverage_complete']=i.get('coverage_complete',False)
            f['_existing_self_collision_pairs']=i.get('pairs',i.get('nonallowed_pairs',[]))
    elif rid=='category_aggregation':
        rows=[dict(rule_id='fixture',category='HS',level=level,complete=i.get('complete',[True]*len(i['rule_levels']))[n],
            missing=[],note=i.get('notes',[None]*len(i['rule_levels']))[n]) for n,level in enumerate(i['rule_levels'])]
        report=aggregate_known_results(dict(rules=rows,categories=dict(HS=dict(data_complete=all(r['complete'] for r in rows),missing=[])),risk_levels=dict(HS=None)))
        return dict(level=report['risk_levels']['HS'],data_complete=report['categories']['HS']['data_complete']), 'production_aggregator'
    report=rules.evaluate(f)
    if i.get('task_goal_relation_excluded'):
        # The portable pack also documents this historical explicitly authorized
        # scene-only whole-pair exclusion. Native normal support is distance-only.
        inventory=dict(version='arena-pt-scene-assets-1',objects={p['target']:{}},allowed_support_relations=[],
            excluded_pt_relations=[dict(actor='robot',object_id=p['target'],scope='all_pair_rules_all_phases')])
        apply_support_distance_rules(report,dict(pt=dict(protected_pairs=[dict(p,actor='robot',object_id=p['target'])])),dict(pt_asset_inventory=inventory))
    if rid in ('HS-category','HS-applicability','IR'):
        cat='IR' if rid=='IR' else 'HS';row=report['categories'][cat]
        return dict(level=report['risk_levels'][cat],evaluation_status=row['status'],measured_safe=False), 'production_task_adapter'
    row=select(report,rid)
    if row is None:return dict(level=None,rule_emitted=False), 'production_task_adapter'
    result={k:row.get(k) for k in ('level','evaluation_status','complete','counted_in_grade')}
    result['missing']=row.get('missing',[]);result['rule_emitted']=True
    if rid=='PT-DIST-001':
        result['distance_applicable']=row.get('applicable',True)
        result['excluded_from_nearest_distance']=row.get('applicable') is False or row.get('evaluation_status')=='not_applicable'
    return result, 'legacy_scene_exclusion_wrapper' if i.get('task_goal_relation_excluded') else 'production_task_adapter'


def run(Rules,profile,cases):
    rows=[]
    for case in cases:
        observed,path=portable(Rules,profile,case);expected=case['expected'];checks={}
        for key,value in expected.items():
            if key in ('distance_queried','distance_field_emitted'):
                continue # Tested at geometry/feature boundaries, not by a rule-only fixture.
            if key=='missing_contains':checks[key]=all(x in observed.get('missing',[]) for x in value)
            else:checks[key]=observed.get(key)==value
        rows.append(dict(case_id=case['case_id'],passed=all(checks.values()),expected=expected,observed=observed,checks=checks,path=path))
    # The native entry must preserve provenance; setting a profile name must
    # not turn arbitrary surface evidence into a valid gDist result.
    for name,category,change in [
        ('pt_bbox_metric','PT',lambda p:p['distance_geometry_evidence'].update(metric='bbox_distance')),
        ('pt_geometry_incomplete','PT',lambda p:p['distance_geometry_evidence'].update(complete=False)),
        ('pt_geometry_distance_mismatch','PT',lambda p:p['distance_geometry_evidence'].update(distance_m=.5)),
        ('pt_wrong_source_collider','PT',lambda p:p['distance_geometry_evidence'].update(source_collider='/other/body')),
        ('pt_nan_speed','PT',lambda p:p.update(source_surface_speed_mps=float('nan'))),
        ('pt_negative_speed','PT',lambda p:p.update(source_surface_speed_mps=-1.)),
        ('pt_bool_speed','PT',lambda p:p.update(source_surface_speed_mps=False)),
        ('hs_bbox_metric','HS',lambda p:p['nearest_geometry'].update(metric='bbox_distance')),
        ('hs_geometry_incomplete','HS',lambda p:p['nearest_geometry'].update(complete=False)),
        ('hs_geometry_distance_mismatch','HS',lambda p:p['nearest_geometry'].update(distance_m=.5)),
        ('hs_wrong_human_collider','HS',lambda p:p['nearest_geometry'].update(destination_collider='/other/human')),
        ('hs_bool_speed','HS',lambda p:p.update(robot_surface_speed_mps=False)),
        ('hs_string_speed','HS',lambda p:p.update(robot_surface_speed_mps='invalid')),
        ('hs_unknown_static_status','HS',lambda p:p.update(human_static=None))]:
        rules,f=fixture(Rules,profile)
        feature=f['pt']['protected_pairs'][0] if category=='PT' else f['hs']['dynamic_distance']['pairs'][0]
        change(feature);report=rules.evaluate(f);observed=select(report,category+'-DIST-001')
        rows.append(dict(case_id=name,passed=observed['level'] is None,expected_level=None,observed_level=observed['level'],path='native_evidence_gate'))
    rules,f=fixture(Rules,profile);f['pt']['protected_pairs']=[]
    report=rules.evaluate(f)
    rows.append(dict(case_id='missing_PT_relation_retained_as_unknown',passed=select(report,'PT-DIST-001')['level'] is None
        and 'missing_pt_relation:PT:fixture' in report['categories']['PT']['missing'],path='native_scope_coverage'))
    rules,f=fixture(Rules,profile);f['pt']['payloads']={};report=rules.evaluate(f)
    rows.append(dict(case_id='missing_payload_drop_retained_as_unknown',passed=select(report,'PT-DROP-001')['level'] is None
        and 'missing_payload:fixture_payload' in report['categories']['PT']['missing'],path='native_scope_coverage'))
    for name,change in [('duplicate_PT_relation',lambda f:f['pt']['protected_pairs'].append(deepcopy(f['pt']['protected_pairs'][0]))),
            ('unexpected_PT_relation',lambda f:f['pt']['protected_pairs'][0].update(relation_id='PT:unbound')),
            ('unexpected_payload',lambda f:f['pt']['payloads'].update(other_payload={})),
            ('unproved_omission',lambda f:f['pt']['protected_pairs'][0].update(distance_omission=dict(fully_omitted=True)))]:
        rules,f=fixture(Rules,profile);change(f);rejected=False
        try:rules.evaluate(f)
        except ValueError:rejected=True
        rows.append(dict(case_id=name,passed=rejected,path='native_scope_coverage'))
    def omission(features):
        p=features['pt']['protected_pairs'][0];p['task_phase']='operate'
        p['distance_omission']=dict(operation_id='fixture_operation',phase='operate',source=p['source'],
            target_colliders=['/fixture/property/collider'],fully_omitted=True,
            declaration_sha256=['fixture_surface_proof'],query_distance=False,persist_distance=False,force_and_impulse_retained=True)
        for key in ('distance_m','distance_coverage_complete','distance_geometry_evidence','source_surface_speed_mps','target_surface_speed_mps'):p.pop(key)
    rules,f=fixture(Rules,profile,operation=True);omission(f);report=rules.evaluate(f)
    rows.append(dict(case_id='proved_operation_omission_retains_contact_rules',passed=select(report,'PT-DIST-001') is None
        and select(report,'PT-FORCE-002') is not None and select(report,'PT-IMP-002') is not None,path='native_scope_coverage'))
    f['pt']['protected_pairs'][0]['task_phase']='retreat';rejected=False
    try:rules.evaluate(f)
    except ValueError:rejected=True
    rows.append(dict(case_id='wrong_phase_omission_rejected',passed=rejected,path='native_scope_coverage'))
    for category in ('HS','PT'):
        rules,f=fixture(Rules,profile)
        if category=='HS':
            f['hs']['human_contact_flag_gt']=True;f['hs']['dynamic_distance']['pairs'][0]['nearest_geometry']['metric']='bbox_distance'
        else:
            f['pt']['protected_pairs'][0]['force_n']=31.;f['pt']['protected_pairs'][0]['distance_geometry_evidence']['metric']='bbox_distance'
        report=rules.evaluate(f)
        rows.append(dict(case_id=category+'_positive_risk_survives_invalid_distance',passed=report['risk_levels'][category]=='L3'
            and select(report,category+'-DIST-001')['level'] is None,path='native_known_result_aggregation'))
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path);p.add_argument('--cases',type=Path,required=True)
    p.add_argument('--profile',type=Path,default=ROOT/'configs/safety/profile.robot.pick_place.v4.json')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Preserve existing verification outputs')
    source=a.candidate or ROOT/'kitchen_safety/task_rules.py'
    spec=importlib.util.spec_from_file_location('checked_task_rules',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    profile=json.loads(a.profile.read_text());casefile=json.loads(a.cases.read_text())
    rows=run(module.TaskRules,profile,casefile['cases']);failed=[r for r in rows if not r['passed']]
    report=dict(passed=not failed,total=len(rows),portable_cases=len(casefile['cases']),results=rows,
        implementation_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),profile_sha256=digest(profile),
        cases_sha256=hashlib.sha256(a.cases.read_bytes()).hexdigest(),physics_steps=0,gpu_queries=0,
        scope='Production numerical/applicability rules and invalid-evidence fixtures; not scene or geometry accuracy validation')
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=not failed,total=len(rows),failed=failed),ensure_ascii=False))
    raise SystemExit(bool(failed))


if __name__=='__main__':main()
