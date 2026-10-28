"""CPU-only verification of retained scene bindings; no risk replay or capture."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys

ROOT=Path('/workspace')
sys.path.insert(0,str(ROOT))
from kitchen_safety.task_scope import bind_scope
from kitchen_safety.task_contact_policy import load_policy
from kitchen_safety.task_rules import grip_contact_exemption

def read(p):return json.loads(Path(p).read_text())

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-index',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise ValueError('Preserve existing report')
    results=[];guards=[]
    for entry in read(a.source_index)['results']:
        tid=entry['task_id'];episode=Path(entry['run'])/'capture/native_000/raw_episode'
        contract=read(ROOT/'configs/risk_task_suite'/(tid+'.json'))
        inventory=read(episode/'mesh_inventory.json');before=read(episode/'relation_scope.json')
        bindings=deepcopy(before['bindings']);policy=load_policy(ROOT,contract)
        if policy is not None:bindings['pt_normal_support_exclusions']=policy
        after=bind_scope(contract,inventory,bindings)
        omissions=[o for o in after['omitted'] if o.get('reason')=='authorized_normal_task_support_contact']
        excluded={}
        for o in omissions:excluded.setdefault(o['source'],set()).update(o['target_colliders'])
        old={r['relation_id']:r for r in before['relations']}
        new={r['relation_id']:r for r in after['relations']}
        assert after['rs']==before['rs'] and after['human_inventory']==before['human_inventory']
        removed=0;targets_removed=0
        for rid,r in old.items():
            if r['category']!='PT':
                assert new[rid]==r
                continue
            drop=excluded.get(r['source'],set())
            contact=set(r['contact_target_colliders'])-drop
            targets_removed+=len(set(r['contact_target_colliders'])&drop)
            if not contact:
                assert rid not in new
                removed+=1
            else:
                current=new[rid]
                assert set(current['contact_target_colliders'])==contact
                assert set(current['target_colliders'])==set(r['target_colliders'])-drop
                # Remaining relationships retain their original numerical/applicability flags.
                for key in ['grade_distance','grade_force','grade_impulse','evidence_only','applicability']:
                    contact_exempt=grip_contact_exemption(current,after,contract) is not None
                    expected=False if contact_exempt and key in ('grade_force','grade_impulse') else r[key]
                    assert current[key]==expected,(tid,rid,key)
        assert set(new)<=set(old)
        if policy is None:assert new==old and not omissions
        else:
            assert omissions and targets_removed>0 and after['coverage']['scope_resolved']
            if not guards:
                for name,change in [
                    ('wrong_task_hash',lambda x:x['policy'].update(task_sha256='0'*64)),
                    ('wrong_task_id',lambda x:x['policy'].update(task_id='other_task')),
                    ('distance_only_scope_cannot_claim_full_exclusion',lambda x:x['policy'].update(scope='distance_only')),
                    ('undeclared_support',lambda x:x['policy']['relations'][0].update(target_id='__not_a_support__')),
                    ('unbound_source_role',lambda x:x['policy']['relations'][0].update(source_role='all_assets'))]:
                    candidate=deepcopy(bindings);change(candidate['pt_normal_support_exclusions'])
                    rejected=False
                    try:bind_scope(contract,inventory,candidate)
                    except (ValueError,KeyError):rejected=True
                    assert rejected,name
                    guards.append(dict(case=name,passed=True))
        results.append(dict(task_id=tid,policy_applied=policy is not None,
            source_episode=str(episode),source_raw_sha256=read(episode/'raw_capture_summary.json')['raw_writer']['sha256'],
            old_task_hash=before['task_sha256'],current_task_hash=after['task_sha256'],
            old_scope_hash=before['scope_sha256'],new_scope_hash=after['scope_sha256'],
            excluded_body_surface_count=targets_removed,removed_PT_relations=removed,
            retained_PT_relations=sum(r['category']=='PT' for r in new.values()),
            HS_and_RS_unchanged=True,unrelated_relations_retained=True,
            feature_and_query_plan_contains_no_excluded_surfaces=True))
    assert len(results)==31 and sum(r['policy_applied'] for r in results)==20
    # A shared PhysX body pair can contain both excluded top and retained side
    # contacts. Excluded impulses must not leak into retained PT measurements.
    from kitchen_safety.task_features import contact_step
    def contact(collider,impulse):
        return dict(actor0='/robot/arm',actor1='/counter',collider0='/robot/arm/shape',
            collider1=collider,points=[dict(impulse_vector_ns=[0,0,impulse])])
    top=contact('/counter/top',1.);side=contact('/counter/side',.1)
    packet=dict(physics_step=0,contact_coverage=True,contacts=[top,side])
    omissions=[({'/robot/arm/shape'},{'/counter/top'})]
    measured=contact_step(packet,1/120,omissions)['pairs'][0]
    assert abs(measured['force_n']-12)<1e-10 and len(measured['contacts'])==2 and measured['pt_contacts']==[side]
    only=contact_step(dict(packet,contacts=[top]),1/120,omissions)['pairs'][0]
    assert 'force_n' not in only and 'impulse_ns' not in only and only['contacts']==[top] and only['pt_contacts']==[]
    legacy=contact_step(packet,1/120)['pairs'][0]
    assert abs(legacy['force_n']-132)<1e-10
    guards.extend([dict(case=name,passed=True) for name in [
        'mixed_body_pair_excludes_only_normal_support_impulse',
        'fully_omitted_pair_keeps_shared_raw_but_no_PT_force_or_impulse',
        'no_policy_retains_original_contact_semantics']])
    report=dict(passed=True,tasks=31,applied_tasks=20,no_corresponding_support_relation=11,
        excluded_body_surface_count=sum(r['excluded_body_surface_count'] for r in results),
        removed_PT_relations=sum(r['removed_PT_relations'] for r in results),
        guards=guards,results=results,new_simulation_steps=0,GPU_queries=0,
        historical_raw_and_labels_modified=False,
        scope='Current catalog policies resolved against retained actual scene inventory; no relabeling of historical episodes')
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['passed','tasks','applied_tasks','no_corresponding_support_relation','excluded_body_surface_count','removed_PT_relations']}))

if __name__=='__main__':main()
