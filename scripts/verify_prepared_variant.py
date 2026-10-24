"""Audit a prepared variant against one actual raw/offline run, read-only."""
import argparse,hashlib,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_variant_bundle import read,file_hash,verify_prepared_binding,digest
from kitchen_safety.task_variant import sample_placement_parameters
from kitchen_safety.task_scope import under


def verify(bundle,run):
    bundle=Path(bundle);run=Path(run);manifest=read(bundle/'variant_bundle.json')
    for name,expected in manifest['files_sha256'].items():
        if Path(name).name!=name or file_hash(bundle/name)!=expected:raise ValueError('Prepared input file identity mismatch')
    contract=read(bundle/'task_contract.json');episode=run/'capture/native_000/raw_episode';risk=run/'risk/native_000'
    assert read(episode/'task_contract.json')==contract and manifest['task_sha256']==digest(contract)
    context=read(episode/'capture_context.json');summary=read(episode/'raw_capture_summary.json');scope=read(episode/'relation_scope.json');inventory=read(episode/'mesh_inventory.json');rs=read(episode/'rs_pair_scope.json')
    assert scope['task_sha256']==digest(contract) and scope['scope_sha256']==summary['identity']['scope_sha256']
    assert summary['online_geometry_queries']==summary['online_risk_evaluations']==0
    schema=read(episode/'raw_capture_schema.json')
    assert all(schema[k]==0 for k in ('risk_queries_online','features_online','grades_online'))
    assert summary['evaluation_status']=='pending_offline'
    variant_asset=None
    if manifest['category']!='baseline':
        binding=read(bundle/'obstacle_binding.json');assert verify_prepared_binding(contract,binding)
        assert context['native_assets']['obstacle']==sample_placement_parameters(binding,context['random_seeds']['placement'])
        aid=binding['asset_id'];variant_asset=scope['bindings']['asset_roots'][aid]
        assert context['native_assets']['source_usd_paths'][aid]==binding['usd_path']
        entries=[e for e in inventory['colliders'] if under(e['path'],variant_asset)];assert entries
        assert all(e['dynamic'] is (not binding['static']) for e in entries)
        if manifest['category']=='HS':
            excluded={x['collider_path'] for x in scope['bindings']['hs_human_surface_omissions']}
            targets={p for r in scope['relations'] if r['category']=='HS' for p in r['target_colliders']}
            assert not scope['human_inventory']['verified_absent']
            assert {e['path'] for e in entries}-excluded<=targets and not targets&excluded
            assert not any(r['source'] in scope['bindings']['fixed_robot_actor_paths'] for r in scope['relations'] if r['category']=='HS')
        else:
            assert {e['path'] for e in entries}<={p for r in scope['relations'] if r['category']=='PT' for p in r['contact_target_colliders']}
        assert not any(under(p,variant_asset) for p in scope['rs']['collider_paths'])
    else:assert context['native_assets']['obstacle'] is None
    evaluation=read(risk/'evaluation_summary.json');assert evaluation['complete'] and evaluation['raw_preflight']['evaluation_admissible']
    assert evaluation['raw_sha256']==summary['raw_writer']['sha256']==file_hash(episode/'raw_evidence.jsonl')
    expected={r['relation_id'] for r in scope['relations'] if r['query_distance']};frames=0;geometry_rows=0
    for line in (risk/'safety_decisions.jsonl').open():
        row=json.loads(line);frames+=1;geometry=row['geometry'];omitted={rid for rid,v in geometry['distance_omissions'].items() if v['fully_omitted']}
        assert set(geometry['relations'])==expected-omitted
        for g in geometry['relations'].values():
            assert g['complete'] and g['metric']=='gdist_authored_collision_surface_distance' and math.isfinite(g['distance_m']) and g['distance_m']>=0;geometry_rows+=1
        assert len(geometry['self_collision'])==len(rs['evaluated_pairs']) and all(x['complete'] for x in geometry['self_collision'])
        assert row['report']['task_id']==manifest['task_id'] and row['report']['scope_hash']==scope['scope_sha256']
        assert row['report']['risk_levels']['IR'] is None and not any(r['category']=='IR' for r in row['report']['rules'])
    assert frames==evaluation['frames']
    return dict(passed=True,task_id=manifest['task_id'],variant_id=manifest['variant_id'],category=manifest['category'],
        raw_sha256=summary['raw_writer']['sha256'],risk_sha256=file_hash(risk/'safety_decisions.jsonl'),
        bundle_sha256=file_hash(bundle/'variant_bundle.json'),physics_steps=summary['physics_steps'],frames=frames,
        hs_relations=sum(x['category']=='HS' for x in scope['relations']),geometry_rows=geometry_rows,rs_pairs_per_frame=len(rs['evaluated_pairs']),
        placement_validation=summary['placement_validation'],online_risk_calls=0,prepared_input_matches_loaded_scene=True,
        full_motion_validation=False,scope='Prepared identities, actual loaded mesh scope and offline coverage; not full trajectory or layout distribution acceptance')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    result=verify(a.bundle,a.run);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
