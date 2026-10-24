"""Explicit grasp-feature migration diagnostic using saved raw and mesh results.

No simulator/GPU queries, no replacement historical labels. The frozen baseline
must reproduce its recorded grasp evidence before candidate differences count.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tarfile
import time
from types import SimpleNamespace
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from kitchen_safety.task_rules import TaskRules


def load_module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module);return module


def compare(episode,risk,candidate,output):
    started=time.perf_counter()
    def read(name):return json.loads((episode/name).read_text())
    frozen=read('frozen_rules.json');summary=read('raw_capture_summary.json')
    contract=read('task_contract.json');inventory=read('mesh_inventory.json');scope=read('relation_scope.json')
    rs=read('rs_pair_scope.json');schema=read('raw_capture_schema.json')
    risk_summary=json.loads((risk/'evaluation_summary.json').read_text())
    assert risk_summary['complete'] and risk_summary['raw_sha256']==summary['raw_writer']['sha256']
    # Shared functions and numeric rules must still be the episode's version.
    # task_features alone is the explicitly compared implementation delta.
    dependencies=['task_scope.py','hs_geometry.py','hs_dynamic.py','grasp_evidence.py','rs_evidence.py','pt_assets.py',
                  'task_rules.py','task_contract.py','risk_profile_loader.py','risk_observer.py']
    for name in dependencies:
        rel='kitchen_safety/'+name
        assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==frozen['implementation_sha256'][rel],rel
    for rel,expected in frozen['implementation_sha256'].items():
        if rel.startswith('third_party/safebench_reference/'):
            assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==expected,rel
    archive=episode/frozen['implementation_archive']['path']
    assert hashlib.sha256(archive.read_bytes()).hexdigest()==frozen['implementation_archive']['sha256']
    with tarfile.open(archive) as tar:
        baseline_source=tar.extractfile('kitchen_safety/task_features.py').read()
    assert hashlib.sha256(baseline_source).hexdigest()==frozen['implementation_sha256']['kitchen_safety/task_features.py']
    output.mkdir(parents=True,exist_ok=False);baseline_path=output/'baseline_task_features.py';baseline_path.write_bytes(baseline_source)
    baseline=load_module(baseline_path,'frozen_baseline_features');new=load_module(candidate,'candidate_features')
    models={};entries={}
    for item in inventory['colliders']:
        key=item['fingerprint']
        if key not in models:
            with np.load(episode/'meshes'/(key+'.npz'),allow_pickle=False) as data:
                vertices=np.ascontiguousarray(data['vertices'],dtype=np.float64)
                faces=np.ascontiguousarray(data['faces'],dtype=np.int32)
            assert hashlib.sha256(vertices.tobytes()+faces.tobytes()).hexdigest()==key
            models[key]=SimpleNamespace(vertices=vertices)
        entries[item['path']]={'model':models[key]}
    queries=SimpleNamespace(entries=entries)
    old=baseline.TaskFeatures(contract,inventory,scope,rs,frozen,schema,queries)
    revised=new.TaskFeatures(contract,inventory,scope,rs,frozen,schema,queries)
    rules=TaskRules(contract,scope,rs,frozen)
    raw_hash=hashlib.sha256();risk_hash=hashlib.sha256();samples=steps=0;deltas=[];grade_deltas=[]
    before_counts={k:Counter() for k in ('HS','PT','RS')};after_counts={k:Counter() for k in before_counts}
    held_counts={k:dict(before=Counter(),after=Counter()) for k in contract['roles']['payload_ids']}
    stable_fields=('held','reason','opening_detected','supported','gripper_separated','source_physics_step')
    with (episode/'raw_evidence.jsonl').open('rb') as raw,(risk/'safety_decisions.jsonl').open('rb') as risk_stream:
        for line in raw:
            raw_hash.update(line);packet=json.loads(line)
            kind=packet['kind']
            if kind=='initial_state':continue
            if kind=='fault_event':
                old.fault_event(packet['payload']);revised.fault_event(packet['payload']);continue
            if kind=='physics':old.advance(packet);revised.advance(packet);steps+=1;continue
            assert kind in ('observation','terminal_observation')
            risk_line=risk_stream.readline();assert risk_line
            risk_hash.update(risk_line);saved=json.loads(risk_line)
            assert saved['action_index']==packet['action_index']==samples
            assert saved['source_physics_step']==packet['source_physics_step']==steps-1
            assert saved['phase']==packet['phase'] and saved['geometry']['scope_sha256']==scope['scope_sha256']
            state=packet['state'];geometry=saved['geometry'];phase=packet['phase']
            old._grasp(state,phase,geometry)
            for aid,p in old.payloads.items():
                assert p['grasp']==saved['features']['pt']['payloads'][aid]['grasp'],('baseline_not_reproduced',samples,aid)
            features=revised.features(state,geometry,phase)
            after=rules.evaluate(features)
            for aid,p in revised.payloads.items():
                before=saved['features']['pt']['payloads'][aid]['grasp'];now=p['grasp']
                held_counts[aid]['before'][str(before['held'])]+=1;held_counts[aid]['after'][str(now['held'])]+=1
                if any(before.get(k)!=now.get(k) for k in stable_fields):
                    deltas.append(dict(action_index=samples,payload_id=aid,phase=phase,
                        before={k:before.get(k) for k in stable_fields},after={k:now.get(k) for k in stable_fields}))
            if saved['report']['risk_levels']!=after['risk_levels']:
                grade_deltas.append(dict(action_index=samples,before=saved['report']['risk_levels'],after=after['risk_levels']))
            for cat in before_counts:
                before_counts[cat][str(saved['report']['risk_levels'][cat])]+=1
                after_counts[cat][str(after['risk_levels'][cat])]+=1
            samples+=1
            if samples%100==0:print(json.dumps(dict(samples=samples,grasp_deltas=len(deltas),grade_deltas=len(grade_deltas),wall_s=time.perf_counter()-started)),flush=True)
        assert not risk_stream.read(1)
    assert raw_hash.hexdigest()==summary['raw_writer']['sha256'] and steps==summary['physics_steps']
    assert samples==risk_summary['frames']
    report=dict(passed=True,task_id=contract['task_id'],raw_sha256=raw_hash.hexdigest(),saved_risk_sha256=risk_hash.hexdigest(),
        baseline_feature_sha256=hashlib.sha256(baseline_source).hexdigest(),candidate_feature_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
        source_scope_sha256=scope['scope_sha256'],samples=samples,source_physics_steps=steps,
        newly_simulated_steps=0,gpu_queries=0,baseline_grasp_reproduced=True,held_counts=held_counts,
        grasp_deltas=deltas,grade_deltas=grade_deltas,before_levels=before_counts,after_levels=after_counts,
        wall_s=time.perf_counter()-started,historical_outputs_modified=False,
        mode='explicit_candidate_feature_migration_diagnostic_reusing_frozen_mesh_results',
        limitation='No replacement label dataset; old geometry/backend retained; no new task execution or physical grasp validation')
    (output/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('passed','task_id','samples','wall_s')}|dict(grasp_deltas=len(deltas),grade_deltas=len(grade_deltas))),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','risk','candidate','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();compare(a.episode,a.risk,a.candidate,a.output)


if __name__=='__main__':main()
