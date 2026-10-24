"""Read-only task/variant specification and source-provenance audit.

This cannot establish runtime placement, geometry or trajectory acceptance.
"""
import argparse,hashlib,json
from collections import Counter
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import compile_task,digest


def main():
    import yaml
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists()
    suite=json.loads((ROOT/'configs/risk_task_suite/suite.json').read_text());library=json.loads((ROOT/'configs/obstacle_library.json').read_text());assets={v['id']:v for v in library['assets']};rows=[];seen=set()
    for item in suite['tasks']:
        path=ROOT/item['path'];task=json.loads(path.read_text());tid=task['task_id'];assert tid==item['task_id'] and tid not in seen;seen.add(tid)
        assert digest(task)==item['contract_sha256'] and task['environment_id']==item['environment_id'] and task['family']==item['family']
        origin=task['provenance'];base=origin['base'] if origin['kind']=='derived_user_requested' else origin
        source=ROOT/base['path'];assert hashlib.sha256(source.read_bytes()).hexdigest()==base['sha256']
        if origin['kind']=='upstream':assert yaml.safe_load(source.read_text())==task['arena_environment_spec']
        else:assert origin['kind']=='derived_user_requested'
        assert compile_task(task['arena_environment_spec'],origin)==task
        for relation in task['normal_distance_relations']:
            assert relation['source_id'] in task['roles']['payload_ids'] and relation['target_id'] in task['asset_declarations']
            assert all(relation[k] is False for k in ('query_distance','persist_distance','grade_distance'))
            assert relation['grade_force'] is True and relation['grade_impulse'] is True
        assert task['pt']['inherited_distance_omissions']==[]
        assert task['raw_requirements']['physics_hz']==120 and task['raw_requirements']['action_hz']==task['raw_requirements']['label_hz']==30
        rows.append(dict(task_id=tid,environment_id=task['environment_id'],family=task['family'],contract_sha256=digest(task),source_kind=origin['kind'],source_sha256=base['sha256']))
    categories=Counter(v['category'] for v in suite['variants']);assert categories==dict(baseline=1,HS=1,PT=6,RS=4)
    variants=[]
    for v in suite['variants']:
        if v['category'] in ('PT','RS'):
            asset=assets[v['obstacle_id']];assert v['usd_path']==asset['usd_path']
            assert v['category']==('PT' if asset['category']=='property' else 'RS')
            path=ROOT/v['usd_path'];assert path.is_file()
            variants.append(dict(variant_id=v['variant_id'],category=v['category'],asset_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),static=asset['static']))
        else:variants.append(dict(variant_id=v['variant_id'],category=v['category']))
    assert len(rows)==suite['task_count']==32 and len({r['environment_id'] for r in rows})==suite['environment_count']==9
    assert Counter(r['family'] for r in rows)==dict(pick_and_place=21,articulated_object=11)
    assert suite['delivery']['production_collection_required'] is False and suite['delivery']['episodes_per_group_required']==0
    assert suite['randomization']['material_scope']=='visual_only' and suite['randomization']['physical_material_randomization'] is False
    assert suite['workflow']['mesh_backend']=='gdist' and suite['workflow']['capture']=='raw-only' and suite['workflow']['evaluation']=='offline_replay'
    assert suite['workflow']['disabled_categories']==['IR']
    result=dict(passed=True,tasks=rows,variants=variants,task_count=len(rows),environment_count=9,declared_combinations=32*12,
        production_quota_required=False,simulation_steps=0,runtime_acceptance_inferred=False,
        scope='Current source/contract/role consistency and declared model identities only; not 384 runtime validations')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ('passed','task_count','environment_count','declared_combinations','runtime_acceptance_inferred')}))
if __name__=='__main__':main()
