"""CPU-only check of exact grip/payload exclusions and recorded spike samples."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
import orjson

ROOT=Path('/workspace');sys.path.insert(0,str(ROOT))
BASE=Path('/media/user/新加卷/RoboLaw-runtime/grip_contact_exclusion_20261027')
from kitchen_safety.task_scope import bind_scope
from kitchen_safety.task_rules import TaskRules,grip_contact_exemption
from kitchen_safety.task_contact_policy import load_policy
from kitchen_safety.task_features import contact_step

def read(p):return json.loads(Path(p).read_text())

def main():
    # The retained index supplies each task's own actual loaded inventory.
    index=read('/media/user/新加卷/RoboLaw-runtime/normal_support_update_20261026/verification.json')
    results=[]
    for entry in index['results']:
        episode=Path(entry['source_episode']);contract=read(ROOT/'configs/risk_task_suite'/(entry['task_id']+'.json'))
        inventory=read(episode/'mesh_inventory.json');old=read(episode/'relation_scope.json')
        bindings=deepcopy(old['bindings']);policy=load_policy(ROOT,contract)
        if policy is not None:bindings['pt_normal_support_exclusions']=policy
        new=bind_scope(contract,inventory,bindings)
        old_current=deepcopy(new)
        spec=importlib.util.spec_from_file_location('old_scope',BASE/'old_task_scope.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        before=module.bind_scope(contract,inventory,bindings)
        previous={r['relation_id']:r for r in before['relations']}
        exempt=[]
        assert new['rs']==before['rs'] and new['human_inventory']==before['human_inventory']
        assert new['omitted']==before['omitted'] and new['coverage']==before['coverage']
        for relation in new['relations']:
            declaration=grip_contact_exemption(relation,new,contract)
            old_r=previous[relation['relation_id']]
            if declaration:
                clone=deepcopy(relation);clone.pop('normal_grip_contact_exemption')
                clone.update(grade_force=True,grade_impulse=True)
                assert clone==old_r
                exempt.append(relation['relation_id'])
            else:assert relation==old_r
        assert len(previous)==len(new['relations'])
        if contract['roles']['payload_ids']:assert exempt
        else:assert not exempt
        results.append(dict(task_id=contract['task_id'],excluded_grip_payload_relations=len(exempt),unrelated_relations_unchanged=True))
    assert len(results)==31 and sum(bool(r['excluded_grip_payload_relations']) for r in results)==20
    ep=Path('/media/user/新加卷/RoboLaw-runtime/banana_grasp_research_20261027/vertical_section/capture/native_000/raw_episode')
    contract=read(ep/'task_contract.json');inventory=read(ep/'mesh_inventory.json');old=read(ep/'relation_scope.json')
    new=bind_scope(contract,inventory,deepcopy(old['bindings']));frozen=read(ep/'frozen_rules.json');rs=read(ep/'rs_pair_scope.json')
    old_rules=TaskRules(contract,old,rs,frozen);new_rules=TaskRules(contract,new,rs,frozen)
    exempt=set(new_rules.grip_contact_exemptions);assert len(exempt)==11
    guards=[]
    for name,mutate in [
        ('non_grip_source',lambda r:r.update(source='/World/envs/env_0/Robot/panda_link6')),
        ('external_target',lambda r:r.update(target='/World/envs/env_0/bowl',contact_target_colliders=['/World/envs/env_0/bowl/shape'])),
        ('missing_authorization',lambda r:r['normal_grip_contact_exemption'].update(authorization='')),
        ('force_flag_conflict',lambda r:r.update(grade_force=True))]:
        relation=deepcopy(next(r for r in new['relations'] if r['relation_id'] in exempt));mutate(relation)
        rejected=False
        try:grip_contact_exemption(relation,new,contract)
        except ValueError:rejected=True
        assert rejected,name;guards.append(dict(case=name,passed=True))
    samples=[]
    decisions=ep.parents[2]/'risk/safety_decisions.jsonl'
    with decisions.open('rb') as source:
        for line in source:
            record=orjson.loads(line);i=record['action_index']
            if i not in (229,230):continue
            features=record['features'];legacy=old_rules.evaluate(features)
            assert legacy['risk_levels']==record['report']['risk_levels']
            updated=deepcopy(features)
            for pair in updated['pt']['protected_pairs']:
                if pair['relation_id'] in exempt:
                    pair.pop('force_n',None);pair.pop('impulse_ns',None)
            report=new_rules.evaluate(updated)
            assert legacy['risk_levels']['PT']=='L3' and report['risk_levels']['PT']=='L0'
            assert legacy['risk_levels']['RS']==report['risk_levels']['RS']
            assert legacy['risk_levels']['HS']==report['risk_levels']['HS']
            assert not any(r.get('relation_id') in exempt and r['rule_id'] in ['PT-FORCE-002','PT-IMP-002'] for r in report['rules'])
            samples.append(dict(action_index=i,time_s=record['time_s'],old_PT='L3',new_PT='L0',HS_RS_unchanged=True))
            if i==230:break
    assert len(samples)==2
    # Raw contact force remains available for bilateral grasp evidence.
    packet=dict(physics_step=0,contact_coverage=True,contacts=[dict(actor0='/robot/finger',actor1='/banana',collider0='/robot/finger/shape',collider1='/banana/shape',points=[dict(impulse_vector_ns=[0,0,4.012268201974572])])])
    measured=contact_step(packet,1/120)['pairs'][0]
    assert abs(measured['force_n']-481.47218423694864)<1e-8 and measured['contacts']==packet['contacts']
    output=dict(passed=True,tasks=31,applied_tasks=20,unaffected_articulation_tasks=11,
        results=results,guards=guards,recorded_spike_samples=samples,raw_grasp_contact_evidence_retained=True,
        historical_labels_modified=False,new_simulation_steps=0,GPU_queries=0,
        validation_scope='31 retained scene inventories plus two exact recorded spike samples; no new collection or full replay')
    (BASE/'verification.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:output[k] for k in ['passed','tasks','applied_tasks','unaffected_articulation_tasks','recorded_spike_samples']}))

if __name__=='__main__':main()
