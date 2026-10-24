"""Read-only audit of fixed-assembly omissions in a fresh native probe."""
import argparse
import hashlib
import json
from pathlib import Path


def verify(episode,risk):
    def read(root,name):return json.loads((root/name).read_text())
    raw=read(episode,'raw_capture_summary.json');summary=read(risk,'evaluation_summary.json')
    scope=read(episode,'relation_scope.json');rs=read(episode,'rs_pair_scope.json')
    assert summary['complete'] and summary['evaluation_mode']=='offline_replay'
    assert summary['raw_sha256']==raw['raw_writer']['sha256']
    assert raw['online_geometry_queries']==raw['online_risk_evaluations']==0
    groups=scope['bindings']['fixed_assembly_distance_groups'];assert groups
    omitted={r['relation_id']:r for r in scope['relations'] if r.get('assembly_group_sha256')}
    assert len(omitted)==sum(len(g['actor_paths'])*(len(g['actor_paths'])-1) for g in groups)
    expected_rs={tuple(sorted(p)) for p in rs['evaluated_pairs']}
    contexts=[];h=hashlib.sha256()
    with (episode/'raw_evidence.jsonl').open('rb') as stream:
        for line in stream:
            h.update(line);p=json.loads(line)
            if p['kind'] in ('observation','terminal_observation'):
                contexts.append({k:p.get(k) for k in ('action_index','source_physics_step','phase','operation_id','time_s')})
    assert h.hexdigest()==raw['raw_writer']['sha256']
    frames=[]
    with (risk/'safety_decisions.jsonl').open() as stream:
        for index,line in enumerate(stream):
            row=json.loads(line);assert {k:row.get(k) for k in contexts[index]}==contexts[index]
            geometry=row['geometry'];features=row['features'];report=row['report']
            assert scope['scope_sha256']==geometry['scope_sha256']==report['scope_hash']
            pt={p['relation_id']:p for p in features['pt']['protected_pairs']}
            for rid,relation in omitted.items():
                assert rid not in geometry['relations'] and rid not in geometry.get('distance_omissions',{})
                assert not {'distance_m','distance_coverage_complete','distance_geometry_evidence','source_surface_speed_mps','target_surface_speed_mps'}&pt[rid].keys()
                rules=[r for r in report['rules'] if r.get('relation_id')==rid]
                assert {r['rule_id'] for r in rules}=={'PT-FORCE-002','PT-IMP-002'}
                assert relation['grade_force'] is relation['grade_impulse'] is True
            for rid,result in geometry['relations'].items():
                assert result['complete'] is True and result['distance_m'] is not None
            pairs=geometry['self_collision']
            assert {tuple(sorted(p['pair'])) for p in pairs}==expected_rs and len(pairs)==len(expected_rs)
            assert all(p['complete'] for p in pairs) and features['rs']['self_collision_evidence']['coverage_complete'] is True
            assert report['categories']['IR']['status']=='disabled'
            frames.append(dict(action_index=index,queried_distance_relations=len(geometry['relations']),
                omitted_internal_relations=len(omitted),rs_pairs=len(pairs),risk_levels=report['risk_levels']))
    assert len(frames)==len(contexts)==summary['frames']
    return dict(passed=True,task_id=scope['task_id'],source_raw_sha256=h.hexdigest(),groups=groups,
        omitted_relations=sorted(omitted),frames=frames,full_task_trajectory=False,
        scope='No internal distance queries/fields/rules; force/impulse retained; remaining geometry complete; RS independent')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','risk','output'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise ValueError('Preserve prior verification output')
    result=verify(args.episode,args.risk);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=True,task_id=result['task_id'],frames=len(result['frames']),omitted_relations=len(result['omitted_relations']))))


if __name__=='__main__':main()
