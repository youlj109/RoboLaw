"""Check all declared selection combinations and negative input cases, no Sim/GPU."""
import argparse,json,sys,tempfile
from pathlib import Path
from copy import deepcopy
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_variant_bundle import resolve_inputs,make_binding,verify_prepared_binding,prepare,read,digest,file_hash


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    suite=read(ROOT/'configs/risk_task_suite/suite.json');library=read(ROOT/'configs/obstacle_library.json');humans=read(ROOT/'configs/risk_human_assets.json');rows=[];bad=[]
    def template(c):return dict(task_sha256=digest(c),placement_mode='native_relations',placement_evidence={'scope':'synthetic input-validation layout, not accepted physics'},placement_relations=[dict(kind='next_to',reference=c['roles']['robot']['id'],params={'side':'negative_x','distance_m':.3})])
    for task in suite['tasks']:
        categories=[]
        for variant in suite['variants']:
            c,v,asset=resolve_inputs(ROOT,suite,library,humans,task['task_id'],variant['variant_id'])
            placement=None if asset is None else template(c);b=make_binding(c,v,asset,placement)
            if b is None:assert v['category']=='baseline'
            else:
                assert verify_prepared_binding(c,b) and b['category']==v['category'] and b['asset_id']==v['obstacle_id']
                assert b['static']==asset['static'] and 'hs_surface_omissions' not in b
                assert b['task_sha256']==digest(c) and 'placement_parameter_samples' not in b
            categories.append(v['category'])
        assert categories.count('baseline')==1 and categories.count('HS')==1 and categories.count('PT')==6 and categories.count('RS')==4
        rows.append(dict(task_id=task['task_id'],variants=len(categories),family=task['family'],input_checks_passed=True,runtime_acceptance=False))
    c,v,asset=resolve_inputs(ROOT,suite,library,humans,suite['tasks'][0]['task_id'],'property_tall_glass');t=template(c)
    def reject(name,call):
        try:call()
        except (ValueError,FileNotFoundError):bad.append(dict(case=name,passed=True));return
        raise AssertionError('Accepted '+name)
    for name,change in [('wrong_task',{'task_sha256':'other'}),('static_override',{'static':True}),('asset_override',{'asset_id':'other'}),('path_override',{'usd_path':'other'}),('unknown_field',{'extra':1}),('empty_provenance',{'placement_evidence':{}}),('cross_task_reference',{'placement_relations':[dict(kind='on',reference='undeclared',params={})]}),('unanchored',{'placement_relations':[dict(kind='rotate_around_solution',params={'yaw_rad':0})]}),('mixed_pose',{'position_xyz':[0,0,0]}),('unsupported_relation',{'placement_relations':[dict(kind='random_around_solution',params={})]}),('bad_range',{'placement_parameter_ranges':[dict(relation_index=0,parameter='distance_m',uniform=[-1,1])]}),('property_human_omissions',{'hs_surface_omissions':{}})]:
        reject(name,lambda change=change:make_binding(c,v,asset,dict(t,**change)))
    world=dict(task_sha256=digest(c),placement_mode='world_pose',placement_evidence={'scope':'fixture'},position_xyz=[1,2,3],rotation_xyzw=[0,0,0,1]);assert verify_prepared_binding(c,make_binding(c,v,asset,world))
    reject('nonfinite_world',lambda:make_binding(c,v,asset,dict(world,position_xyz=[float('nan'),0,0])))
    reject('bad_quaternion',lambda:make_binding(c,v,asset,dict(world,rotation_xyzw=[0,0,0,0])))
    reject('baseline_with_obstacle',lambda:make_binding(c,{'category':'baseline'},None,t))
    reject('unknown_variant',lambda:resolve_inputs(ROOT,suite,library,humans,c['task_id'],'unknown'))
    duplicate=deepcopy(suite);duplicate['tasks'].append(duplicate['tasks'][0]);reject('duplicate_task',lambda:resolve_inputs(ROOT,duplicate,library,humans,suite['tasks'][0]['task_id'],'baseline'))
    tampered=make_binding(c,v,asset,t);tampered['static']=True;reject('changed_after_preparation',lambda:verify_prepared_binding(c,tampered))
    hc,hv,ha=resolve_inputs(ROOT,suite,library,humans,c['task_id'],'human_static_adult');ht=template(hc)
    omission=dict(source_usd_sha256=ha['source_sha256'],relative_colliders=['FixtureOnly'],authorization='synthetic input check, not anatomical authorization',reason='fixture')
    for name,change in [('human_wrong_revision',{'source_usd_sha256':'other'}),('human_wildcard',{'relative_colliders':['*']}),('human_cross_root',{'relative_colliders':['../other']}),('human_no_authorization',{'authorization':''})]:
        reject(name,lambda change=change:make_binding(hc,hv,ha,dict(ht,hs_surface_omissions=dict(omission,**change))))
    with tempfile.TemporaryDirectory(prefix='variant-bundle-') as directory:
        base=Path(directory);fake=base/'fixture.usda';fake.write_text('# fixture asset identity, no geometry\n');sa=dict(asset,usd_path=str(fake),source_sha256=file_hash(fake));sb=make_binding(c,v,sa,t);assert verify_prepared_binding(c,sb)
        fake.write_text('# modified fixture\n');reject('source_changed',lambda:verify_prepared_binding(c,sb))
        tp=base/'placement.json';tp.write_text(json.dumps(t));out=base/'prepared';m=prepare(ROOT,ROOT/'configs/risk_task_suite/suite.json',ROOT/'configs/obstacle_library.json',ROOT/'configs/risk_human_assets.json',c['task_id'],v['variant_id'],tp,out)
        assert all(file_hash(out/n)==h for n,h in m['files_sha256'].items()) and not m['runtime_scope_verified']
        reject('output_overwrite',lambda:prepare(ROOT,ROOT/'configs/risk_task_suite/suite.json',ROOT/'configs/obstacle_library.json',ROOT/'configs/risk_human_assets.json',c['task_id'],v['variant_id'],tp,out))
    result=dict(passed=True,task_count=len(rows),declared_combinations=sum(x['variants'] for x in rows),tasks=rows,negative_cases=bad,
        world_pose_input_check=True,source_recheck=True,bundle_output_hashes_checked=True,simulation_steps=0,gpu_queries=0,
        scope='384 input selections with synthetic placement declarations, not 384 simulated layouts or risk evaluations')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ('passed','task_count','declared_combinations')}|{'negative_cases':len(bad)}))
if __name__=='__main__':main()
