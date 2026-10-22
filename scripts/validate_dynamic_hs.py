"""Explicit dynamic HS regression checks, using labeled fixtures and a frozen old input.

Does not collect data or rewrite any archived profile. Run separately from capture.
"""
import argparse
from copy import deepcopy
import json,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from kitchen_safety.hs_dynamic import relation,features,validate_config,VERSION
from kitchen_safety.risk_profile_loader import create_evaluator
from kitchen_safety.risk_observer import arena_coverage_report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference-episode',type=Path,required=True)
    p.add_argument('--reference-decisions',type=Path,required=True)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    c=json.loads(args.profile.read_text())['hs_dynamic'];validate_config(c)
    checks=[]
    def check(name,ok,detail=None):
        checks.append(dict(name=name,passed=bool(ok),detail=detail))
        if not ok:raise AssertionError(name+': '+str(detail))
    for v,s in [(.05,.02875),(.1,.035),(.2,.055)]:
        r=relation(.2,v,0.,True,c)
        check('static_formula_'+str(v),math.isclose(r['protective_distance_m'],s,abs_tol=1e-12),r)
    r=relation(.2,.2,0,True,c);s=r['protective_distance_m']
    check('protection_equality_L2',relation(s,.2,0,True,c)['level']=='L2')
    check('warning_equality_L1',relation(s+c['warning_band_m'],.2,0,True,c)['level']=='L1')
    check('outside_warning_L0',relation(s+c['warning_band_m']+.001,.2,0,True,c)['level']=='L0')
    check('missing_velocity_unknown',relation(.01,None,0,True,c)['level'] is None)
    check('moving_human_without_bound_unknown',relation(.1,.1,.1,False,c)['level'] is None)
    check('free_payload_unknown',relation(.01,.1,0,True,c,stop_model='uncontrolled_payload')['level'] is None)
    check('supported_payload_zero_stop',relation(.1,0,0,True,c,stop_model='supported_stationary')['stop_s']==0)
    cc=deepcopy(c);cc.update(robot_acceleration_bound_mps2=2.,moving_human_acceleration_bound_mps2=3.)
    a=relation(1.,.2,.3,False,cc)
    vr=.2+2*.05;ts=vr;vh=.3+3*(.05+ts)
    expected=vh*(.05+ts)+vr*.05+vr*vr/2+.025
    check('acceleration_envelopes',math.isclose(a['protective_distance_m'],expected,abs_tol=1e-12))
    # Rigid-surface velocity: COM at x=1, v=(0,2,0), omega=(0,0,1).
    # Vertices x=0,2 => y velocity 1,3: maximum exactly 3 m/s.
    geometry=SimpleNamespace(root='/env',metadata={'motion_body_names':['arm']},entries=[
        {'path':'arm_mesh','actor':'arm','group':'robot','model':SimpleNamespace(vertices=np.array([[0.,0,0],[2.,0,0],[1.,1,0]]))},
        {'path':'person_mesh','actor':None,'group':'human'}])
    motion=dict(body_pose=[[0,0,0,0,0,0,1]],body_com_position_w=[[1,0,0]],
        body_com_linear_velocity_w=[[0,2,0]],body_com_angular_velocity_w=[[0,0,1]])
    measurement={'dynamic_relations':[dict(relation_id='near',source_group='robot',source_actor='arm',
        source_colliders=['arm_mesh'],human_collider='person_mesh',distance_m=.1)]}
    f=features(geometry,motion,measurement,{},c)
    check('surface_speed_uses_COM_and_rotation',f['pairs'][0]['robot_surface_speed_mps']==3.)
    motion['body_com_position_w']=None
    check('missing_COM_no_difference_fallback',features(geometry,motion,measurement,{},c)['level'] is None)
    frozen=json.loads((args.reference_episode/'safety_observer_binding.json').read_text())
    oldprofile=frozen['resolved_profile'];profile=deepcopy(oldprofile);profile['hs_dynamic']=c
    profile['runtime_binding']['hs_geometry']['dynamic_relation_ids']=['near','fast']
    with args.reference_decisions.open() as stream:record=json.loads(next(stream))
    snapshot=deepcopy(record['input']);hs=snapshot['features']['hs']
    def pair(rid,d,v):
        r=relation(d,v,0,True,c);r.update(relation_id=rid,source_group='robot',source_actor=rid,
            human_collider='person_mesh',source_colliders=['mesh']);return r
    # Farther fast link is dangerous, nearest slow link is safe.
    hs['dynamic_distance']=dict(version=VERSION,pairs=[pair('near',.1,0),pair('fast',.2,1.)],
        level='L0',coverage_complete=True,dominant_pair={'distance_m':999})
    hs['human_contact_flag_gt']=False
    evaluator=create_evaluator(profile)
    report=evaluator.evaluate(snapshot)
    def rule(rid):return next(r for r in report['rules'] if r['rule_id']==rid)
    check('dominant_is_margin_not_nearest',rule('HS-DIST-001')['evidence']['dominant_pair']['source_actor']=='fast')
    check('recomputed_not_trusted_pregrade',rule('HS-DIST-001')['level']=='L2')
    check('TTC_diagnostic_has_no_grade',rule('HS-TTC-001')['level'] is None and rule('HS-TTC-001')['complete'])
    baseline=create_evaluator(oldprofile).evaluate(snapshot)
    check('PT_RS_rules_unchanged',[r for r in report['rules'] if r['category'] in ('PT','RS')]==[r for r in baseline['rules'] if r['category'] in ('PT','RS')])
    hs['dynamic_distance']['pairs']=[pair('near',.1,0)]
    report=evaluator.evaluate(snapshot)
    check('missing_pair_coverage_separate',rule('HS-DIST-001')['level']=='L0' and not rule('HS-DIST-001')['complete'])
    hs['dynamic_distance']['pairs']=[];hs['human_contact_flag_gt']=None
    report=evaluator.evaluate(snapshot)
    check('no_known_results_remain_unknown',report['risk_levels']['HS'] is None)
    hs.update(dynamic_scope='fixed',fixed_distance_exempt=True,human_contact_flag_gt=True)
    report=evaluator.evaluate(snapshot)
    check('fixed_distance_exempt_but_contact_L3',rule('HS-DIST-001')['level'] is None and report['risk_levels']['HS']=='L3')
    b=deepcopy(frozen['binding']);b['human_inventory']['verified_absent']=True
    absent=arena_coverage_report(report,snapshot['features'],b,disabled_categories=['IR'])
    check('verified_absence_still_not_applicable',absent['categories']['HS']['status']=='not_applicable')
    out=dict(passed=True,rule_version=VERSION,checks=checks,scope='Formula, synthetic rigid-body telemetry and adapter regression using historical input; not a physical brake calibration or complete task validation')
    args.output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=True,checks=len(checks),output=str(args.output))))

if __name__=='__main__':main()
