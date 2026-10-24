"""RS missing-evidence regression; no physics, GPU query, or raw mutation.

Optionally compare candidate feature extraction against an existing episode's
measured channels and saved mesh results. This is an explicit code comparison,
not a frozen historical replay or a replacement set of risk labels.
"""
import argparse
from collections import deque
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def check(module):
    names=['j1','j2'];passed=[]
    robot=dict(q=[0.,0.],joint_pos_limits=[[-1.,1.],[-2.,2.]],applied_torque=[2.,4.],joint_effort_limits=[10.,10.])
    cases=[('complete',{},1.,.4),('missing_position',{'q':None},None,.4),
        ('missing_torque',{'applied_torque':None},1.,None),
        ('nonfinite_position',{'q':[0.,float('nan')]},None,.4),
        ('nonfinite_torque',{'applied_torque':[2.,float('nan')]},1.,None),
        ('nonfinite_effort',{'joint_effort_limits':[10.,float('inf')]},1.,None),
        ('zero_effort',{'joint_effort_limits':[10.,0.]},1.,None),
        ('inverted_position_bounds',{'joint_pos_limits':[[1.,-1.],[-2.,2.]]},None,.4),
        ('short_position_array',{'q':[0.]},None,.4),
        ('long_torque_array',{'applied_torque':[1.,2.,3.]},1.,None),
        ('measured_joint_violation',{'q':[1.2,0.]},-.2,.4)]
    for name,updates,margin,ratio in cases:
        actual=module.robot_joint_features(dict(robot,**updates),names,names)
        for measured,expected in zip(actual[:2],(margin,ratio)):
            assert measured is None if expected is None else measured is not None and abs(measured-expected)<1e-12,(name,actual)
        assert bool(actual[2])==(margin is None or ratio is None),(name,actual)
        passed.append(name)
    for label,actual_names,controlled in [('duplicate_joint_identity',['j1','j1'],names),
            ('unbound_joint',names,['j3']),('empty_controlled',names,[]),('duplicate_controlled',names,['j1','j1'])]:
        assert module.robot_joint_features(robot,actual_names,controlled)[:2]==(None,None)
        passed.append(label)
    scope=dict(body_paths=['/robot/a','/robot/b','/robot/c'],evaluated_pairs=[['/robot/a','/robot/c'],['/robot/b','/robot/c']],
        allowed_pairs=[['/robot/a','/robot/b']],joint_graph_complete=True)
    rows=[dict(pair=p,complete=True,intersection=False) for p in scope['evaluated_pairs']]
    positive=deepcopy(rows);positive[0].update(intersection=True)
    unknown=deepcopy(rows);unknown[0].update(intersection=None,complete=False)
    nonboolean=deepcopy(rows);nonboolean[0].update(intersection=0)
    scenarios=[('complete_negative',rows,False,True,[]),('empty_result',[],None,False,[]),
        ('missing_pair',rows[:1],None,False,[]),('duplicate_pair',[rows[0],rows[0],rows[1]],None,False,[]),
        ('unknown_intersection',unknown,None,False,[]),('nonboolean_intersection',nonboolean,None,False,[]),
        ('unexpected_pair',rows+[dict(pair=['/other/x','/robot/a'],complete=True,intersection=True)],None,False,[]),
        ('known_positive',positive,True,True,[]),('positive_with_missing',positive[:1],True,False,[]),
        ('raw_positive_with_missing',[],True,False,[dict(pairs=[dict(bodyA='/robot/c',bodyB='/robot/a')])]),
        ('allowed_structural_contact',rows,False,True,[dict(pairs=[dict(bodyA='/robot/b',bodyB='/robot/a')])])]
    for label,values,flag,complete,windows in scenarios:
        actual=module.self_collision_features(values,scope,windows)
        assert actual[0] is flag and actual[2]['coverage_complete'] is complete,(label,actual)
        passed.append(label)
    for label,change in [('missing_graph',dict(joint_graph_complete=False)),
                         ('incomplete_scope',dict(allowed_pairs=[]))]:
        flag,_,evidence=module.self_collision_features(rows,dict(scope,**change),[])
        assert flag is None and evidence['coverage_complete'] is False
        passed.append(label)
    # Exercise the public feature path as well as the independent helpers.
    f=module.TaskFeatures.__new__(module.TaskFeatures)
    f.windows=deque();f.payloads={};f.previous_hs={};f.previous_hs_step=None;f.step=-1;f.dt=1/120
    f.model={'controlled_joint_names':names};f.schema={'robot_joint_names':names}
    f.scope={'relations':[],'bindings':{'robot_root':'/robot'},'human_inventory':{'collision_paths':[]},'coverage':{'scope_resolved':True}}
    f.rs_scope=scope;f.profile={'hs_dynamic':{'version':'fixture'}};f.fault_result={}
    values=f.features({'robot':dict(robot,q=None)},{'relations':{},'self_collision':[]},'hold')
    assert values['rs']['joint_torque_ratio_gt']==.4
    assert values['rs']['joint_limit_margin_gt_rad'] is None
    assert values['rs']['self_collision_flag_gt'] is None
    assert values['_existing_self_collision_pairs'] is None
    passed.append('public_features_missing_channels')
    return passed


def compare(module,episode,risk):
    def read(name):return json.loads((episode/name).read_text())
    summary=read('raw_capture_summary.json');scope=read('relation_scope.json');rs=read('rs_pair_scope.json')
    schema=read('raw_capture_schema.json');h=hashlib.sha256();states=[]
    with (episode/'raw_evidence.jsonl').open('rb') as stream:
        for line in stream:
            h.update(line);row=json.loads(line)
            if row['kind'] in ('observation','terminal_observation'):states.append(row)
    assert h.hexdigest()==summary['raw_writer']['sha256']
    frames=0;first=None
    with (risk/'safety_decisions.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);source=states[frames]
            if first is None:first=deepcopy(row['features'])
            assert row['action_index']==source['action_index'] and row['source_physics_step']==source['source_physics_step']
            old=row['features']['rs'];measured=module.robot_joint_features(source['state']['robot'],schema['robot_joint_names'],scope['bindings']['robot_model']['controlled_joint_names'])
            assert measured[:2]==(old['joint_limit_margin_gt_rad'],old['joint_torque_ratio_gt']),(frames,measured,old)
            assert not measured[2]
            # Compare negative full-geometry checks; raw collision positives
            # are independently tested by fixtures above.
            flag,_,evidence=module.self_collision_features(row['geometry']['self_collision'],rs,[])
            assert evidence['coverage_complete'] is True and old['self_collision_evidence']['coverage_complete'] is True
            if old['self_collision_flag_gt'] is False:assert flag is False
            frames+=1
    assert frames==len(states)
    from kitchen_safety.task_rules import TaskRules
    rules=TaskRules(read('task_contract.json'),scope,rs,read('frozen_rules.json'))
    rule_checks=[]
    for name,ratio,collision,pairs,fault,expected in [
        ('known_torque_unknown_other_rs',1.2,None,None,None,'L2'),
        ('known_normal_torque_unknown_other_rs',.4,None,None,None,'L0'),
        ('all_rs_unknown',None,None,None,None,None),
        ('positive_collision_with_missing_pairs',None,True,rs['evaluated_pairs'][:1],None,'L2'),
        ('observed_fault_motion',None,None,None,True,'L3')]:
        injected=deepcopy(first)
        injected['rs'].update(joint_limit_margin_gt_rad=None,joint_torque_ratio_gt=ratio,
            self_collision_flag_gt=collision,self_collision_evidence={'coverage_complete':False},motion_after_fault_gt=fault)
        injected['_existing_self_collision_pairs']=pairs
        report=rules.evaluate(injected)
        assert report['risk_levels']['RS']==expected,(name,report['categories']['RS'])
        assert report['categories']['RS']['data_complete'] is False
        assert report['categories']['IR']['status']=='disabled'
        rule_checks.append(name)
    return dict(task_id=scope['task_id'],frames=frames,raw_sha256=h.hexdigest(),rule_checks=rule_checks,
        scope='Measured joint/mesh coverage comparison plus synthetic missing-evidence rule cases; no historical regrading')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path);p.add_argument('--episode',type=Path);p.add_argument('--risk',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise ValueError('Preserve existing verification output')
    if bool(args.episode)!=bool(args.risk):raise ValueError('Supply both --episode and --risk')
    source=args.candidate or ROOT/'kitchen_safety/task_features.py'
    spec=importlib.util.spec_from_file_location('reviewed_task_features',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=dict(passed=True,implementation_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),cases=check(module),
        physics_steps=0,gpu_queries=0,production_collection=False)
    if args.episode:result['existing_evidence_comparison']=compare(module,args.episode,args.risk)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':main()
