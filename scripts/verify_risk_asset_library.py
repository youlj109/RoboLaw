"""Verify loaded risk asset identity, raw channels, mesh coverage and rule routing.

Read-only evidence audit. Passing does not certify placement or motion envelope;
those outcomes are reported separately and cannot be replaced by a green exit.
"""
import argparse,hashlib,json,math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest
from kitchen_safety.task_scope import under


def verify(run,binding):
    episode=run/'capture/native_000/raw_episode';risk=run/'risk/native_000'
    def read(name):return json.loads((episode/name).read_text())
    contract=read('task_contract.json');scope=read('relation_scope.json');inventory=read('mesh_inventory.json');summary=read('raw_capture_summary.json');schema=read('raw_capture_schema.json');rs=read('rs_pair_scope.json')
    evaluation=json.loads((risk/'evaluation_summary.json').read_text());audit=json.loads((run/'audit/native_000.json').read_text())
    assert evaluation['complete'] and audit['passed'] and evaluation['raw_sha256']==summary['raw_writer']['sha256']
    assert digest(contract)==binding['task_sha256']
    source=Path(binding['usd_path']);assert hashlib.sha256(source.read_bytes()).hexdigest()==binding['placement_evidence']['asset_usd_sha256']
    root=scope['bindings']['asset_roots'][binding['asset_id']]
    entries=[e for e in inventory['colliders'] if under(e['path'],root)];assert entries
    assert all(e['dynamic'] is (not binding['static']) for e in entries)
    paths={e['path'] for e in entries};dynamic={e['actor'] for e in entries if e['dynamic']}
    relations={r['relation_id']:r for r in scope['relations'] if r['category']=='PT' and paths&set(r['target_colliders'])}
    assert relations and set.union(*(set(r['target_colliders']) for r in relations.values()))==paths
    queried={rid for rid,r in relations.items() if r['query_distance']}
    omitted=set(relations)-queried;assert queried
    assert all(relations[rid]['persist_distance'] and relations[rid]['grade_distance'] for rid in queried)
    assert all(not relations[rid]['persist_distance'] and not relations[rid]['grade_distance'] for rid in omitted)
    assert not any(under(p,root) for p in rs['body_paths'])
    assert scope['human_inventory']['verified_absent'] is True
    h=hashlib.sha256();steps=0;observations=[];last=None
    with (episode/'raw_evidence.jsonl').open('rb') as stream:
        for line in stream:
            h.update(line);row=json.loads(line)
            if row['kind'] in ('initial_state','physics'):
                state=row['state'];assert state['complete']
                assert dynamic<=state['actor_states_world'].keys() and dynamic<=state['actor_motion_world'].keys()
                for actor in dynamic:
                    motion=state['actor_motion_world'][actor]
                    for key in ['com_position_world_m','com_linear_velocity_mps','angular_velocity_radps']:
                        assert len(motion[key])==3 and all(math.isfinite(v) for v in motion[key])
                last=state
            if row['kind']=='physics':assert row['physics_step']==steps;steps+=1
            if row['kind'] in ('observation','terminal_observation'):
                assert row['state']==last;observations.append(row['source_physics_step'])
    assert h.hexdigest()==summary['raw_writer']['sha256']==evaluation['raw_sha256']
    assert steps==summary['physics_steps']==12 and len(observations)==4
    for key in ('risk_queries_online','features_online','grades_online'):assert schema[key]==0
    frames=0;distance_rows=0;rule_rows=0;rs_rows=0;unknown_distances=0;rh=hashlib.sha256()
    with (risk/'safety_decisions.jsonl').open('rb') as stream:
        for line in stream:
            rh.update(line);row=json.loads(line);assert row['source_physics_step']==observations[frames]
            assert row['report']['scope_hash']==scope['scope_sha256'] and row['evaluation_mode']=='offline_replay'
            pairs={p['relation_id']:p for p in row['features']['pt']['protected_pairs']}
            for rid in relations:
                p=pairs[rid];selected=[r for r in row['report']['rules'] if r.get('relation_id')==rid]
                if rid in queried:
                    g=row['geometry']['relations'][rid]
                    assert g['metric']=='gdist_authored_collision_surface_distance' and g['complete'] is True
                    assert math.isfinite(g['distance_m']) and g['distance_m']>=0 and p['distance_m']==g['distance_m']
                    assert {r['rule_id'] for r in selected}=={'PT-DIST-001','PT-FORCE-002','PT-IMP-002'}
                    unknown_distances+=sum(r['rule_id']=='PT-DIST-001' and r['level'] is None for r in selected)
                    distance_rows+=1
                else:
                    assert rid not in row['geometry']['relations'] and 'distance_m' not in p
                    assert {r['rule_id'] for r in selected}=={'PT-FORCE-002','PT-IMP-002'}
                rule_rows+=len(selected)
            assert row['features']['rs']['self_collision_evidence']['coverage_complete'] is True
            assert all(p['complete'] is True for p in row['geometry']['self_collision'])
            assert len(row['geometry']['self_collision'])==len(rs['evaluated_pairs'])
            assert row['report']['categories']['HS']['status']=='not_applicable'
            assert row['report']['categories']['IR']['status']=='disabled'
            rs_rows+=len(rs['evaluated_pairs']);frames+=1
    assert frames==evaluation['frames']==4
    return dict(passed=True,asset_id=binding['asset_id'],category=binding['category'],static=binding['static'],
        task_id=contract['task_id'],colliders=len(paths),dynamic_actors=len(dynamic),pt_relations=len(relations),queried_relations=len(queried),omitted_distance_relations=len(omitted),
        physics_steps=steps,frames=frames,mesh_distance_rows=distance_rows,pt_rule_rows=rule_rows,
        unknown_pt_distance_rows=unknown_distances,rs_self_pair_rows=rs_rows,
        placement_passed=audit['placement_validation'].get('passed'),placement_validation=audit['placement_validation'],
        raw_sha256=h.hexdigest(),risk_sha256=rh.hexdigest(),asset_sha256=binding['placement_evidence']['asset_usd_sha256'],
        full_motion_envelope_verified=False,source_risk_category_is_not_a_grade=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists()
    source=json.loads((a.suite/'library_probe_report.json').read_text());assert source['complete']
    rows=[];failed=[]
    for aid,entry in source['variants'].items():
        if not entry['passed']:failed.append(dict(asset_id=aid,reason='pipeline stage failed'));continue
        try:row=verify(a.suite/aid,json.loads((a.suite/(aid+'.binding.json')).read_text()));rows.append(row)
        except Exception as exc:failed.append(dict(asset_id=aid,reason=repr(exc)))
    result=dict(passed=not failed and len(rows)==len(source['variants']),assets=len(source['variants']),results=rows,failed=failed,
        full_task_variant_matrix_verified=False,scope='Real asset model integration on declared representative tasks; placement and full motion are independent')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    raise SystemExit(not result['passed'])
if __name__=='__main__':main()
