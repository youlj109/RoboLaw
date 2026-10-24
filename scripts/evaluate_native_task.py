"""Replay a native raw episode through shared gDist and frozen HS/PT/RS rules."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest
from kitchen_safety.risk_observer import write_json,finite_json


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();start=time.perf_counter()
    def read(name):return json.loads((args.episode/name).read_text())
    summary=read('raw_capture_summary.json');frozen=read('frozen_rules.json')
    if summary['evaluation_status']!='pending_offline' or summary['online_geometry_queries']!=0:raise ValueError('Not a native raw-only episode')
    if digest(frozen['profile'])!=frozen['profile_sha256']:raise ValueError('Frozen rules differ')
    for name,expected in frozen['implementation_sha256'].items():
        source=ROOT/name
        if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest()!=expected:
            raise ValueError('Frozen implementation differs; explicit migration is required: '+name)
    from kitchen_safety.task_mesh import TaskMeshQueries
    from kitchen_safety.geometry_backend import get_backend
    from kitchen_safety.task_features import TaskFeatures
    from kitchen_safety.task_rules import TaskRules
    contract=read('task_contract.json');inventory=read('mesh_inventory.json');scope=read('relation_scope.json')
    rs=read('rs_pair_scope.json');schema=read('raw_capture_schema.json')
    if scope['task_sha256']!=digest(contract):raise ValueError('Task binding identity differs')
    backend=get_backend('gdist',frozen['mesh_backend']['options'])
    if backend.runtime_info()['build']!=frozen['mesh_backend']['build']:raise ValueError('Frozen gDist build differs')
    args.output.mkdir(parents=True,exist_ok=False)
    receipt=dict(complete=False,evaluation_mode='offline_replay',source_episode=str(args.episode.resolve()),
        full_task_execution_verified=summary.get('full_task_execution_verified',False))
    write_json(args.output/'evaluation_summary.json',receipt)
    try:
        queries=TaskMeshQueries(inventory,scope,args.episode/'meshes',backend)
        features=TaskFeatures(contract,inventory,scope,rs,frozen,schema,queries);rules=TaskRules(contract,scope,rs,frozen)
        counts={k:Counter() for k in ('HS','PT','RS')};timings=Counter();h=hashlib.sha256();steps=observations=0
        initial=None;terminals=0
        with (args.episode/'raw_evidence.jsonl').open('rb') as source,(args.output/'safety_decisions.jsonl').open('x') as out:
            for line in source:
                h.update(line);packet=json.loads(line);kind=packet['kind']
                if terminals:raise ValueError('Raw evidence follows terminal state')
                if kind=='initial_state':
                    if initial is not None or steps:raise ValueError('Repeated initial state')
                    if packet.get('physics_step')!=-1:raise ValueError('Invalid initial source step')
                    initial=packet['state'];features.initialize(initial);continue
                if kind=='fault_event':
                    event=packet['payload']
                    if initial is None or not observations or type(event.get('action_index')) is not int or event['action_index']!=observations-1:
                        raise ValueError('Fault event has no matching active action')
                    features.fault_event(event);continue
                if kind=='physics':
                    t=time.perf_counter();features.advance(packet);steps+=1;timings['temporal_features_s']+=time.perf_counter()-t;continue
                if kind not in ('observation','terminal_observation'):raise ValueError('Unknown raw packet type: '+kind)
                if initial is None or packet['source_physics_step']!=steps-1 or packet['action_index']!=observations or steps!=4*observations:
                    raise ValueError('Noncausal observation/action/physics alignment')
                state=packet['state']
                if state!= (initial if steps==0 else features.current['state']):raise ValueError('Observation does not match measured preceding state')
                t=time.perf_counter();geometry=queries.frame(state['actor_states_world'],rs['evaluated_pairs'],
                    operation_id=packet.get('operation_id'),phase=packet['phase']);timings['geometry_s']+=time.perf_counter()-t
                t=time.perf_counter();values=features.features(state,geometry,packet['phase']);timings['feature_extraction_s']+=time.perf_counter()-t
                t=time.perf_counter();report=rules.evaluate(values);timings['rules_s']+=time.perf_counter()-t
                record=dict(sample_kind=kind,action_index=observations,source_physics_step=steps-1,time_s=packet['time_s'],
                    evaluation_mode='offline_replay',phase=packet['phase'],operation_id=packet.get('operation_id'),
                    features=values,geometry=geometry,report=report)
                out.write(json.dumps(finite_json(record),ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
                for cat in counts:counts[cat][str(report['risk_levels'][cat])]+=1
                observations+=1
                if kind=='terminal_observation':terminals+=1
        if h.hexdigest()!=summary['raw_writer']['sha256']:raise ValueError('Raw evidence digest differs')
        if schema.get('terminal_sample_required') and terminals!=1:raise ValueError('Final action window has no terminal safety sample')
        if steps!=summary['physics_steps'] or steps!=4*(observations-terminals):raise ValueError('Incomplete last action or raw footer mismatch')
        receipt.update(complete=True,raw_sha256=h.hexdigest(),physics_steps=steps,frames=observations,
            terminal_samples=terminals,action_observations=observations-terminals,
            risk_level_counts=counts,timings=dict(timings),scope_hash=scope['scope_sha256'],profile_hash=rules.profile_hash,
            rule_coverage='known-results; damage and authoritative fault feed remain unavailable; moving-object bounds may be unknown',
            geometry_backend=backend.runtime_info())
    except Exception as exc:receipt['error']=repr(exc);raise
    finally:
        receipt['wall_s']=time.perf_counter()-start;write_json(args.output/'evaluation_summary.json',receipt)
    print(json.dumps(receipt,ensure_ascii=False,default=dict))

if __name__=='__main__':main()
