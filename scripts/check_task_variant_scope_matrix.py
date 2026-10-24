"""Counterfactual scope fixtures over recorded task and asset inventories.

Never creates raw episodes or physical layouts. Tests role binding and scope
semantics only; copied donor coordinates are NOT poses in a new task scene.
"""
import argparse,hashlib,json,sys,time
from copy import deepcopy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest
from kitchen_safety.task_scope import bind_scope,under
from kitchen_safety.task_binding import derive_bindings,bind_operation_contacts,robot_self_pairs


def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_source(spec):
    episode=Path(spec['episode']);summary=read(episode/'raw_capture_summary.json');inv=read(episode/'mesh_inventory.json');scope=read(episode/'relation_scope.json');contract=read(episode/'task_contract.json')
    assert summary['raw_writer']['sha256']==spec['raw_sha256'] and scope['task_sha256']==digest(contract)
    assert scope['scope_sha256']==summary['identity']['scope_sha256']
    copied=deepcopy(scope);identity=copied.pop('scope_sha256');assert digest(copied)==identity
    assert inv['complete'] and not inv['errors'] and digest(inv)==scope['inventory_sha256']
    return dict(inventory=inv,scope=scope,contract=contract,source_spec=spec,
        source_file_sha256={n:sha(episode/n) for n in ('raw_capture_summary.json','mesh_inventory.json','relation_scope.json','task_contract.json')})


def bind(contract,inventory,saved,roots,humans,semantics):
    original=saved['bindings']
    binding=derive_bindings(contract,inventory,roots,original['robot_root'],human_roots=humans,
        expected_humans=bool(humans),grip_actor_paths=original['grip_actor_paths'],
        fixed_hs_authorization=original['hs_fixed_omission_authorized'],
        backend_fixed_actor_paths=original['fixed_graph_evidence']['physx_fixed_base_roots'])
    binding['hs_human_surface_omissions']=[] # no inherited donor clothing policy
    binding['robot_model']=deepcopy(original['robot_model'])
    bind_operation_contacts(contract,inventory,binding,semantics.get(contract['task_id']))
    scope=bind_scope(contract,inventory,binding)
    rs=robot_self_pairs(inventory,binding['robot_root'],explicit_structural_pairs=binding['robot_model']['structural_pairs'])
    assert scope['coverage']['scope_resolved'],scope['unresolved']
    assert scope['coverage']['manipulation_surface_bindings_complete']
    return scope,rs


def check(task,donor,semantics,baseline,baseline_rs):
    contract=task['contract'];inv=deepcopy(task['inventory']);saved=task['scope'];roots=deepcopy(saved['bindings']['asset_roots']);human_roots=[];added=[]
    variant_id='baseline'
    if donor is not None:
        spec=donor['source_spec'];variant_id=spec['variant_id'];aid=spec['asset_id'];root=donor['scope']['bindings']['asset_roots'][aid]
        assert aid not in roots and not any(under(e['path'],root) for e in inv['colliders'])
        added=[deepcopy(e) for e in donor['inventory']['colliders'] if under(e['path'],root)];assert added
        # These eleven donors are simple independent rigid/static obstacles.
        # Do not silently discard physics links for another kind of donor.
        assert not any(under(j['path'],root) for j in donor['inventory']['joints'])
        roots[aid]=root;inv['colliders']+=added
        if spec['category']=='HS':human_roots=[root]
        inv.pop('inventory_sha256',None)
        inv['counterfactual_fixture']=dict(task_inventory_sha256=digest(task['inventory']),donor_inventory_sha256=digest(donor['inventory']),variant_id=variant_id,not_a_loaded_scene=True)
        inv['inventory_sha256']=digest(inv)
    scope,rs=bind(contract,inv,saved,roots,human_roots,semantics)
    # RS may carry non-robot joint-owner audit metadata, but evaluated physical
    # robot pairs, exclusions and full robot shape list must remain identical.
    for field in ('evaluated_pairs','allowed_pairs','body_paths','joint_graph_complete'):assert rs[field]==baseline_rs[field]
    assert scope['rs']['collider_paths']==baseline['rs']['collider_paths']
    assert scope['bindings']['robot_model']==baseline['bindings']['robot_model']
    assert scope['bindings']['surface_bindings']==baseline['bindings']['surface_bindings']
    assert scope['bindings']['operation_contacts']==baseline['bindings']['operation_contacts']
    old={x['relation_id']:x for x in baseline['relations']};current={x['relation_id']:x for x in scope['relations']}
    assert len(current)==len(scope['relations']) and old.keys()<=current.keys()
    def comparable(row):
        item=deepcopy(row);item.pop('assembly_group_sha256',None);return item
    assert all(comparable(before)==comparable(current[rid]) for rid,before in old.items())
    new=[r for rid,r in current.items() if rid not in old]
    if donor is None:assert not new and scope['human_inventory']['verified_absent']
    elif donor['source_spec']['category']=='HS':
        paths={e['path'] for e in added}
        protected=scope['human_inventory']['collision_paths']
        assert set(protected)==paths and len(protected)==len(paths)
        assert not scope['human_inventory']['intentionally_omitted_collision_paths']
        assert all(r['category']=='HS' and set(r['target_colliders'])<=paths for r in new)
        assert {p for r in new for p in r['target_colliders']}==paths
        sources={r['source'] for r in baseline['relations'] if r['source'] not in scope['bindings']['fixed_robot_actor_paths']}
        assert len(new)==len(sources)*len(paths) and {r['source'] for r in new}==sources
        assert not any(set(r.get('contact_target_colliders',[]))&paths for r in scope['relations'] if r['category']=='PT')
        assert not paths&set(scope['rs']['collider_paths'])
    else:
        paths={e['path'] for e in added};assert new and scope['human_inventory']['verified_absent']
        assert all(r['category']=='PT' and set(r['contact_target_colliders'])==paths for r in new)
        assert all(r['grade_force'] and r['grade_impulse'] for r in new)
        assert {r['source'] for r in new}=={r['source'] for r in baseline['relations']}
        assert all(r['query_distance'] and set(r['target_colliders'])==paths for r in new if r['source'] not in scope['bindings']['fixed_world_actor_paths'])
        assert not paths&set(scope['rs']['collider_paths'])
    return dict(task_id=contract['task_id'],variant_id=variant_id,passed=True,hs_relations=sum(r['category']=='HS' for r in scope['relations']),
        pt_relations=sum(r['category']=='PT' for r in scope['relations']),added_relations=len(new),donor_colliders=len(added),
        rs_pairs=len(rs['evaluated_pairs']),baseline_relations_unchanged=True,operation_surfaces_unchanged=True,
        normal_support_surfaces_unchanged=True,scope_sha256=scope['scope_sha256'],fixture_only=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--sources',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    manifest=read(a.sources);assert manifest['version']=='robolaw-counterfactual-scope-sources-1'
    suite=read(ROOT/'configs/risk_task_suite/suite.json');semantics=read(ROOT/'configs/risk_task_semantics.json')['tasks'];rows=[];failures=[]
    assert {t['task_id'] for t in manifest['tasks']}=={t['task_id'] for t in suite['tasks']} and len(manifest['tasks'])==len(suite['tasks'])
    assert {d['variant_id'] for d in manifest['donors']}=={v['variant_id'] for v in suite['variants'] if v['category']!='baseline'} and len(manifest['donors'])==len(suite['variants'])-1
    donors=[load_source(s) for s in manifest['donors']];tasks=[load_source(s) for s in manifest['tasks']]
    for donor in donors:
        variant=next(v for v in suite['variants'] if v['variant_id']==donor['source_spec']['variant_id']);assert variant['category']==donor['source_spec']['category'] and variant['obstacle_id']==donor['source_spec']['asset_id']
    started=time.perf_counter()
    for task in tasks:
        c=task['contract'];expected=next(t for t in suite['tasks'] if t['task_id']==c['task_id']);assert digest(c)==expected['contract_sha256'] and not task['scope']['bindings']['human_roots']
        baseline,rs=bind(c,task['inventory'],task['scope'],task['scope']['bindings']['asset_roots'],[],semantics)
        for donor in [None]+donors:
            try:rows.append(check(task,donor,semantics,baseline,rs))
            except Exception as exc:
                import traceback
                failures.append(dict(task_id=c['task_id'],variant_id=donor['source_spec']['variant_id'] if donor else 'baseline',error=repr(exc),traceback=traceback.format_exc()))
        print(c['task_id'],len(rows),'checks;',len(failures),'failures',flush=True)
    result=dict(passed=not failures and len(rows)==len(tasks)*(len(donors)+1),combinations=len(rows),failures=failures,results=rows,
        simulation_steps=0,gpu_queries=0,grades_generated=0,physical_layouts_validated=0,wall_s=time.perf_counter()-started,
        source_manifest_sha256=sha(a.sources),source_file_hashes=[dict(source=x['source_spec'],hashes=x['source_file_sha256']) for x in tasks+donors],
        implementation_sha256={n:sha(ROOT/'kitchen_safety'/n) for n in ('task_scope.py','task_binding.py')},
        scope='Explicit synthetic topology compositions from recorded task/asset inventories. No scene reconstruction, donor-pose validity, distance accuracy or motion acceptance is inferred.')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(passed=result['passed'],combinations=len(rows),failures=len(failures))))
    raise SystemExit(not result['passed'])
if __name__=='__main__':main()
