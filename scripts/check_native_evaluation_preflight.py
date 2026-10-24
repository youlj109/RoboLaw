"""Prove direct evaluation rejects corrupt raw input before backend loading.

CPU fixtures use a backend-load sentinel, not synthetic geometry or grades.
"""
import argparse,hashlib,importlib.util,json,sys,tempfile
from copy import deepcopy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest
from kitchen_safety import geometry_backend


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


class BackendReached(Exception):pass


def run(evaluator,auditor):
    state=dict(complete=True);records=[dict(kind='initial_state',physics_step=-1,state=state),
        dict(kind='observation',action_index=0,source_physics_step=-1,time_s=0.,state=state)]
    records += [dict(kind='physics',physics_step=n,action_index=0,start_time_s=n/120,end_time_s=(n+1)/120,
        state=state,contact_coverage=True,contacts=[],commands=[{'method':'fixture_actual_command'}] if n==0 else []) for n in range(4)]
    records += [dict(kind='terminal_observation',action_index=1,source_physics_step=3,time_s=4/120,state=state)]
    base={'raw_capture_summary.json':dict(evaluation_status='pending_offline',online_geometry_queries=0,physics_steps=4),
        'raw_capture_schema.json':dict(physics_hz=120,action_hz=30,terminal_sample_required=True),
        'task_contract.json':{},'mesh_inventory.json':{},'relation_scope.json':dict(task_sha256=digest({})),
        'rs_pair_scope.json':{},'variation_snapshot.json':{},
        'frozen_rules.json':dict(profile={},profile_sha256=digest({}),implementation_sha256={},mesh_backend=dict(options={}))}
    cases=[('valid',None),('physics_clock',lambda r,m:m['raw_capture_schema.json'].update(physics_hz=60)),
        ('action_clock',lambda r,m:m['raw_capture_schema.json'].update(action_hz=15)),
        ('physics_timestamp',lambda r,m:r[2].update(end_time_s=.1)),
        ('observation_timestamp',lambda r,m:r[1].update(time_s=.1)),
        ('terminal_timestamp',lambda r,m:r[-1].update(time_s=.5)),
        ('state_incomplete',lambda r,m:r[2].update(state={'complete':False})),
        ('contacts_incomplete',lambda r,m:r[2].update(contact_coverage=False)),
        ('no_commands',lambda r,m:[x.update(commands=[]) for x in r if x['kind']=='physics']),
        ('noncausal_observation',lambda r,m:r[1].update(source_physics_step=0)),
        ('missing_terminal',lambda r,m:r.pop()),
        ('execution_aborted',lambda r,m:m['raw_capture_summary.json'].update(execution_error='fixture')),
        ('digest_mismatch','digest')]
    old_backend=geometry_backend.get_backend;old_argv=sys.argv[:];old_module=sys.modules.get('scripts.audit_native_task');rows=[]
    sys.modules['scripts.audit_native_task']=auditor
    try:
        with tempfile.TemporaryDirectory(prefix='native-preflight-') as temp:
            for name,mutate in cases:
                raw=deepcopy(records);meta=deepcopy(base)
                if callable(mutate):mutate(raw,meta)
                blob=''.join(json.dumps(x)+'\n' for x in raw).encode()
                meta['raw_capture_summary.json']['raw_writer']=dict(sha256='invalid' if mutate=='digest' else hashlib.sha256(blob).hexdigest())
                episode=Path(temp)/name;episode.mkdir()
                for filename,value in meta.items():(episode/filename).write_text(json.dumps(value))
                (episode/'raw_evidence.jsonl').write_bytes(blob)
                backend_calls=[]
                def sentinel(*args,**kwargs):backend_calls.append(True);raise BackendReached()
                geometry_backend.get_backend=sentinel
                output=episode/'never-created-risk';sys.argv=['evaluate','--episode',str(episode),'--output',str(output)]
                outcome=None;error=None
                try:evaluator.main()
                except BackendReached:outcome='backend_gate_reached'
                except ValueError as exc:outcome='preflight_rejected' if str(exc).startswith('Raw evidence preflight failed:') else 'other_failure';error=str(exc)
                except Exception as exc:outcome='unexpected_exception';error=repr(exc)
                passed=(outcome=='backend_gate_reached' and len(backend_calls)==1) if name in ('valid','state_incomplete','contacts_incomplete') else (outcome=='preflight_rejected' and not backend_calls)
                rows.append(dict(case_id=name,passed=passed,outcome=outcome,backend_calls=len(backend_calls),risk_output_created=output.exists(),error=error))
                assert not output.exists()
    finally:
        geometry_backend.get_backend=old_backend;sys.argv=old_argv
        if old_module is None:sys.modules.pop('scripts.audit_native_task',None)
        else:sys.modules['scripts.audit_native_task']=old_module
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evaluate',type=Path,default=ROOT/'scripts/evaluate_native_task.py');p.add_argument('--audit',type=Path,default=ROOT/'scripts/audit_native_task.py');p.add_argument('--baseline',type=Path);p.add_argument('--profile',type=Path,default=ROOT/'configs/safety/profile.robot.pick_place.v4.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    auditor=load('candidate_raw_audit',a.audit);rows=run(load('candidate_evaluator',a.evaluate),auditor)
    # Channel gaps must not erase positive/known evidence from other rules.
    from scripts.check_native_rule_contract import fixture,select
    from kitchen_safety.task_rules import TaskRules
    profile=json.loads(a.profile.read_text())
    rules,features=fixture(TaskRules,profile)
    pair=features['hs']['dynamic_distance']['pairs'][0];pair['distance_m']=.001;pair['nearest_geometry']['distance_m']=.001
    features['hs']['human_contact_flag_gt']=None
    report=rules.evaluate(features)
    preserved=select(report,'HS-DIST-001')['level']=='L2' and select(report,'HS-COLL-001')['level'] is None and report['risk_levels']['HS']=='L2'
    result=dict(passed=all(x['passed'] for x in rows) and preserved,cases=len(rows),results=rows,missing_contact_keeps_known_HS_L2=preserved,simulation_steps=0,gpu_queries=0,
        backend_loader_is_sentinel=True,scope='Raw admission gate only; valid fixture stops before loading GPU, not full evaluation',
        evaluate_sha256=hashlib.sha256(a.evaluate.read_bytes()).hexdigest(),audit_sha256=hashlib.sha256(a.audit.read_bytes()).hexdigest())
    if a.baseline:result['baseline_results']=run(load('baseline_evaluator',a.baseline),auditor)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(passed=result['passed'],cases=len(rows),failures=[r for r in rows if not r['passed']])))
    raise SystemExit(not result['passed'])
if __name__=='__main__':main()
