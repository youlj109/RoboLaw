"""Check exact PT rigid-assembly omissions against measured frozen stages.

No new simulation, geometry query or historical scope mutation.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def check(suite,candidate_dir=None):
    import kitchen_safety
    if candidate_dir:kitchen_safety.__path__.insert(0,str(candidate_dir))
    from kitchen_safety.task_binding import fixed_assembly_distance_groups,derive_bindings
    from kitchen_safety.task_scope import bind_scope
    results=[];fixture=None
    for run in sorted(suite.iterdir()):
        raw=run/'capture/native_000/raw_episode'
        if not raw.is_dir():continue
        def read(name):return json.loads((raw/name).read_text())
        contract=read('task_contract.json');inventory=read('mesh_inventory.json');old=read('relation_scope.json')
        binding=deepcopy(old['bindings'])
        # Prove the legacy result can be rebuilt before introducing new policy.
        assert bind_scope(contract,inventory,binding)==old,('old_scope_not_reproduced',contract['task_id'])
        groups=fixed_assembly_distance_groups(contract,inventory,binding)
        derived=derive_bindings(contract,inventory,binding['asset_roots'],binding['robot_root'],
            human_roots=binding['human_roots'],expected_humans=binding['expected_humans'],
            grip_actor_paths=binding['grip_actor_paths'],fixed_hs_authorization=binding['hs_fixed_omission_authorized'],
            backend_fixed_actor_paths=binding['fixed_graph_evidence']['physx_fixed_base_roots'])
        assert derived['fixed_assembly_distance_groups']==groups
        binding['fixed_assembly_distance_groups']=groups
        new=bind_scope(contract,inventory,binding)
        expected={(left,right) for g in groups for left in g['actor_paths'] for right in g['actor_paths'] if left!=right}
        by_old={r['relation_id']:r for r in old['relations']};by_new={r['relation_id']:r for r in new['relations']}
        assert by_old.keys()==by_new.keys()
        changed=[]
        for rid,before in by_old.items():
            after=by_new[rid]
            if (before['source'],before['target']) in expected and before['category']=='PT':
                assert before['query_distance'] is True
                assert after['query_distance'] is after['persist_distance'] is after['grade_distance'] is False
                assert after['grade_force'] is after['grade_impulse'] is True
                for key in ('source_colliders','target_colliders','contact_target_colliders'):
                    assert before[key]==after[key]
                restored=dict(after);restored.pop('assembly_group_sha256');restored.pop('reason')
                restored.update(query_distance=True,persist_distance=True,grade_distance=True)
                assert restored==before
                changed.append(rid)
            else:assert before==after,(contract['task_id'],rid)
        assert len(changed)==len(expected)
        assert new['rs']==old['rs'] and new['human_inventory']==old['human_inventory']
        assert new['hs_acquisition_omissions']==old['hs_acquisition_omissions']
        results.append(dict(task_id=contract['task_id'],group_count=len(groups),removed_distance_relations=changed,
            other_relations_unchanged=True,hs_rs_unchanged=True,scope_sha256=new['scope_sha256']))
        if groups:
            assert fixture is None,'Update test fixture selection explicitly for new qualified assets'
            fixture=(contract,inventory,deepcopy(old['bindings']),groups)
    assert len(results)==11 and sum(len(x['removed_distance_relations']) for x in results)==2
    assert fixture is not None
    contract,inventory,binding,groups=fixture
    joint_path=groups[0]['source_joints'][0]['path'];tests=[]
    # Synthetic human presence checks the HS routing even though the measured
    # operation suite is baseline. This is a scope fixture, not human geometry.
    human_inv=deepcopy(inventory);human_binding=deepcopy(binding)
    human_inv['colliders'].append(dict(path='/scope_fixture_human/surface',actor=None,dynamic=False,fingerprint='synthetic-scope-only'))
    human_binding.update(human_roots=['/scope_fixture_human'],expected_humans=True,human_inventory_verified=True)
    human_before=bind_scope(contract,human_inv,human_binding)
    human_binding['fixed_assembly_distance_groups']=fixed_assembly_distance_groups(contract,human_inv,human_binding)
    human_after=bind_scope(contract,human_inv,human_binding)
    before_hs=[r for r in human_before['relations'] if r['category']=='HS']
    assert before_hs and before_hs==[r for r in human_after['relations'] if r['category']=='HS']
    assert {a for group in groups for a in group['actor_paths']}<={r['source'] for r in before_hs}
    tests.append('internal_PT_omission_does_not_remove_HS_sources')
    variants=[('revolute',{'type':'PhysicsRevoluteJoint'},{}),('disabled',{'enabled':False},{}),
        ('breakable_force',{}, {'physics:breakForce':'100'}),('breakable_torque',{}, {'physics:breakTorque':'100'}),
        ('missing_force',{}, {'physics:breakForce':None}),('missing_torque',{}, {'physics:breakTorque':None}),
        ('negative_infinity',{}, {'physics:breakForce':'-inf'}),('nonfinite_unknown',{}, {'physics:breakForce':'nan'}),
        ('collisions_enabled',{}, {'physics:collisionEnabled':'True'}),
        ('missing_collision_semantics',{}, {'physics:collisionEnabled':None}),
        ('numeric_not_boolean',{}, {'physics:collisionEnabled':0})]
    for label,fields,attrs in variants:
        changed=deepcopy(inventory);joint=next(j for j in changed['joints'] if j['path']==joint_path)
        joint.update(fields);joint['attributes'].update(attrs)
        assert fixed_assembly_distance_groups(contract,changed,binding)==[],label
        tests.append(label)
    for label,endpoint in [('unbound_endpoint','/not_loaded/body'),('outside_task_asset',binding['robot_root'])]:
        changed=deepcopy(inventory);joint=next(j for j in changed['joints'] if j['path']==joint_path)
        joint['body1']=[endpoint]
        assert fixed_assembly_distance_groups(contract,changed,binding)==[],label
        tests.append(label)
    forged=deepcopy(binding);forged['fixed_assembly_distance_groups']=deepcopy(groups)
    forged['fixed_assembly_distance_groups'][0]['actor_paths'].append(binding['robot_root'])
    try:bind_scope(contract,inventory,forged)
    except ValueError:tests.append('forged_group_rejected')
    else:raise AssertionError('Forged omission accepted')
    stale=deepcopy(binding);stale['fixed_assembly_distance_groups']=groups
    changed=deepcopy(inventory);next(j for j in changed['joints'] if j['path']==joint_path)['enabled']=False
    try:bind_scope(contract,changed,stale)
    except ValueError:tests.append('stale_joint_evidence_rejected')
    else:raise AssertionError('Stale omission accepted')
    source=candidate_dir or ROOT/'kitchen_safety'
    return dict(passed=True,tasks=results,negative_cases=tests,physics_steps=0,gpu_queries=0,
        source_sha256={name:hashlib.sha256((source/name).read_bytes()).hexdigest() for name in ('task_binding.py','task_scope.py')},
        historical_outputs_modified=False,scope='11 frozen articulation inventories and targeted negative bindings; no motion/collision safety claim')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',type=Path,required=True)
    p.add_argument('--candidate-dir',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise ValueError('Preserve existing check outputs')
    report=check(args.suite,args.candidate_dir)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':main()
