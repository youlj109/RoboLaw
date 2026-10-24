"""Check native fault chronology through actual feature and rule adapters.

Synthetic causal traces and raw audit fixtures; no simulation or GPU queries.
Optional comparison reuses measured raw q/dq and events without relabeling.
"""
import argparse
from collections import deque
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from kitchen_safety.rs_evidence import FaultEvidence
from kitchen_safety.task_rules import TaskRules
from scripts.check_native_rule_contract import fixture,select

CONFIG=dict(motion_velocity_radps=.01,motion_displacement_rad=.001)


def instance(module):
    obj=module.TaskFeatures.__new__(module.TaskFeatures)
    obj.fault=FaultEvidence(CONFIG);obj.fault_result={};obj.step=-1;obj.current=None
    obj.windows=deque(maxlen=4);obj.dt=1/120;obj._grasp=lambda *a,**k:None
    return obj


def event(before=0,**updates):
    return dict(dict(before_physics_step=before,action_index=before//4,kind='command_write_exception',
        method='set_dof_position_targets',error='synthetic fixture'),**updates)


def state(q=0.,dq=0.):return dict(robot=dict(q=[q] if q is not None else None,dq=[dq]))


def initialize(obj,initial):
    if hasattr(obj,'initialize'):obj.initialize(initial)
    # Compatibility branch for the pre-fix implementation, used only to
    # demonstrate its missing anchor. Never alter the candidate itself.


def advance(obj,row):
    obj.advance(dict(physics_step=obj.step+1,state=row,contacts=[],contact_coverage=True,phase='fixture'))


def cases(module,profile):
    rows=[]
    def check(name,initial,trace,expected,post=None):
        obj=instance(module);initialize(obj,initial)
        for kind,value in trace:
            if kind=='event':obj.fault_event(value)
            else:advance(obj,value)
        actual=obj.fault_result.get('motion_after_fault_gt')
        rules,f=fixture(TaskRules,profile);f['rs']['motion_after_fault_gt']=actual
        level=select(rules.evaluate(f),'RS-FAULT-001')['level']
        target='L3' if expected is True else 'L0' if expected is False else None
        passed=actual is expected and level==target
        if post is not None:passed=passed and obj.fault_result.get('post_fault_samples')==post
        rows.append(dict(case_id=name,passed=passed,expected_motion=expected,observed_motion=actual,
            expected_level=target,observed_level=level,post_fault_samples=obj.fault_result.get('post_fault_samples')))
    check('first_post_command_velocity',state(),[('event',event()),('step',state(dq=.02))],True,1)
    check('single_step_motion_latches',state(),[('event',event()),('step',state(dq=.02)),('step',state())],True,2)
    check('initial_displacement_anchor',state(),[('event',event()),('step',state(q=.002))],True,1)
    check('later_command_first_step',state(),[('step',state()),('event',event(1)),('step',state(dq=.02))],True,1)
    check('no_fault_is_not_healthy',state(),[('step',state(dq=.02))],None)
    check('known_fault_stationary',state(),[('event',event()),('step',state())],False,1)
    check('strict_velocity_boundary',state(),[('event',event()),('step',state(dq=.01))],False,1)
    check('strict_displacement_boundary',state(),[('event',event()),('step',state(q=.001))],False,1)
    check('invalid_initial_then_motion',state(q=None),[('step',state(dq=.02))],True,1)
    check('invalid_sample_not_own_post_fault',state(),[('step',state(q=None))],None,0)
    check('invalid_sample_then_motion',state(),[('step',state(q=None)),('step',state(dq=.02))],True,1)
    check('fault_without_post_measurement',state(),[('event',event())],None)
    check('positive_survives_later_missing',state(),[('event',event()),('step',state(dq=.02)),('step',state(q=None))],True,2)
    for name,value in [('future',event(1)),('past',event(-1)),('bool_step',event(False)),
                       ('unknown_kind',event(kind='unproved_fault')),('unknown_method',event(method='unknown'))]:
        obj=instance(module);initialize(obj,state());rejected=False
        try:obj.fault_event(value)
        except ValueError:rejected=True
        rows.append(dict(case_id='reject_'+name,passed=rejected))
    return rows


def audit_cases(script):
    rows=[]
    with tempfile.TemporaryDirectory(prefix='native-fault-clock-') as folder:
        base=Path(folder)
        for name,change,expected in [('valid',{},True),('future',{'before_physics_step':1},False),
                ('past',{'before_physics_step':-1},False),('wrong_action',{'action_index':1},False),
                ('unknown_kind',{'kind':'other'},False),('unknown_method',{'method':'other'},False)]:
            episode=base/name;episode.mkdir();measured=dict(complete=True)
            records=[dict(kind='initial_state',physics_step=-1,state=measured),
                     dict(kind='observation',action_index=0,source_physics_step=-1,time_s=0.,state=measured),
                     dict(kind='fault_event',payload=event(**change))]
            records += [dict(kind='physics',physics_step=n,action_index=0,start_time_s=n/120,end_time_s=(n+1)/120,
                state=measured,contact_coverage=True,contacts=[],commands=[{}] if n==0 else []) for n in range(4)]
            raw=''.join(json.dumps(x)+'\n' for x in records).encode();(episode/'raw_evidence.jsonl').write_bytes(raw)
            for filename,value in [('raw_capture_summary.json',dict(raw_writer=dict(sha256=hashlib.sha256(raw).hexdigest()))),
                 ('raw_capture_schema.json',dict(physics_hz=120,action_hz=30)),('relation_scope.json',{}),('variation_snapshot.json',{})]:
                (episode/filename).write_text(json.dumps(value))
            output=episode/'audit.json';p=subprocess.run([sys.executable,str(script),'--episode',str(episode),'--output',str(output)],capture_output=True,text=True)
            actual=json.loads(output.read_text()) if output.exists() else {}
            rows.append(dict(case_id='audit_'+name,passed=actual.get('passed') is expected and p.returncode==int(not expected),errors=actual.get('errors'),stderr=p.stderr[-500:]))
    return rows


def compare(module,episode,risk,profile):
    """Read measured fault inputs; never replace saved features or grades."""
    summary=json.loads((episode/'raw_capture_summary.json').read_text())
    saved=json.loads((risk/'evaluation_summary.json').read_text())
    assert saved['complete'] and saved['raw_sha256']==summary['raw_writer']['sha256']
    obj=instance(module);h=hashlib.sha256();rh=hashlib.sha256();frames=steps=events=0;changes=[]
    rules,f=fixture(TaskRules,profile)
    with (episode/'raw_evidence.jsonl').open('rb') as raw,(risk/'safety_decisions.jsonl').open('rb') as decisions:
        for line in raw:
            h.update(line);row=json.loads(line);kind=row['kind']
            if kind=='initial_state':initialize(obj,row['state'])
            elif kind=='physics':obj.advance(row);steps+=1
            elif kind=='fault_event':obj.fault_event(row['payload']);events+=1
            elif kind in ('observation','terminal_observation'):
                line=next(decisions);rh.update(line);old=json.loads(line)
                assert old['action_index']==row['action_index']==frames
                assert old['source_physics_step']==row['source_physics_step']==obj.step
                candidate=obj.fault_result.get('motion_after_fault_gt')
                f['rs']['motion_after_fault_gt']=candidate
                after=select(rules.evaluate(f),'RS-FAULT-001')['level']
                before=select(old['report'],'RS-FAULT-001')['level']
                if candidate is not old['features']['rs'].get('motion_after_fault_gt') or after!=before:
                    changes.append(dict(frame=frames,before=before,after=after))
                frames+=1
            else:raise ValueError('Unknown source packet')
        assert not decisions.read(1)
    assert h.hexdigest()==summary['raw_writer']['sha256'] and frames==saved['frames']
    assert steps==summary['physics_steps']
    return dict(passed=not changes,frames=frames,physics_steps_read=steps,raw_fault_events=events,
        final_observed_fault_active=obj.fault_result.get('observed_fault_active'),changes=changes,
        source_raw_sha256=h.hexdigest(),saved_risk_sha256=rh.hexdigest(),
        scope='Read-only native fault channel comparison and unchanged RS-FAULT rule; not complete feature replay')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,default=ROOT/'kitchen_safety/task_features.py')
    p.add_argument('--profile',type=Path,default=ROOT/'configs/safety/profile.robot.pick_place.v4.json')
    p.add_argument('--audit-script',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--episode',type=Path);p.add_argument('--risk',type=Path)
    a=p.parse_args();assert not a.output.exists()
    if bool(a.episode)!=bool(a.risk):raise ValueError('Supply episode and risk together')
    spec=importlib.util.spec_from_file_location('candidate_features',a.candidate);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rows=cases(module,json.loads(a.profile.read_text()))
    if a.audit_script:rows+=audit_cases(a.audit_script)
    result=dict(passed=all(r['passed'] for r in rows),cases=len(rows),results=rows,
        implementation_sha256=hashlib.sha256(a.candidate.read_bytes()).hexdigest(),
        audit_sha256=hashlib.sha256(a.audit_script.read_bytes()).hexdigest() if a.audit_script else None,
        simulation_steps=0,gpu_queries=0,historical_outputs_modified=False)
    if a.episode:
        result['existing_data_comparison']=compare(module,a.episode,a.risk,json.loads(a.profile.read_text()))
        result['passed']=result['passed'] and result['existing_data_comparison']['passed']
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],cases=len(rows),failures=[r for r in rows if not r['passed']])))
    raise SystemExit(not result['passed'])


if __name__=='__main__':main()
