"""Prepare task-bound risk variants without simulation, queries or risk labels."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from kitchen_safety.task_contract import digest,compile_task
from kitchen_safety.task_variant import sample_placement_parameters

VERSION='robolaw-task-variant-bundle-1'


def read(path):return json.loads(Path(path).read_text())
def file_hash(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def unique(rows,key,value):
    found=[x for x in rows if x.get(key)==value]
    if len(found)!=1:raise ValueError('Expected one declared '+key+': '+str(value))
    return found[0]


def resolve_inputs(root,suite,library,humans,task_id,variant_id):
    root=Path(root);entry=unique(suite['tasks'],'task_id',task_id);path=(root/entry['path']).resolve();contract=read(path)
    if (digest(contract)!=entry['contract_sha256'] or contract['task_id']!=task_id
            or contract['environment_id']!=entry['environment_id'] or contract['family']!=entry['family']
            or compile_task(contract['arena_environment_spec'],contract['provenance'])!=contract):
        raise ValueError('Task catalog, source roles or frozen contract differ')
    variant=unique(suite['variants'],'variant_id',variant_id);category=variant['category']
    if category=='baseline':
        if variant['obstacle_id'] is not None:raise ValueError('Baseline must not declare an obstacle')
        return contract,variant,None
    if category in ('PT','RS'):
        source=unique(library['assets'],'id',variant['obstacle_id'])
        if source['category']!=('property' if category=='PT' else 'robot') or source['usd_path']!=variant['usd_path']:
            raise ValueError('Variant and asset library disagree')
    elif category=='HS':
        source=unique(humans['assets'],'id',variant['obstacle_id'])
        if source.get('category')!='human' or source.get('static') is not True:
            raise ValueError('Static-adult variant requires a declared static human')
    else:raise ValueError('Unsupported risk variant category')
    if type(source['static']) is not bool:raise ValueError('Asset motion type must be explicit')
    usd=(root/source['usd_path']).resolve()
    if not usd.is_file():raise ValueError('Risk asset is not restored locally: '+str(usd))
    asset=dict(asset_id=source['id'],usd_path=str(usd),category=category,static=source['static'],
        source_sha256=file_hash(usd),source_library_record_sha256=digest(source))
    if source.get('sha256') is not None and source['sha256']!=asset['source_sha256']:
        raise ValueError('Risk asset source differs from registered revision')
    return contract,variant,asset


def make_binding(contract,variant,asset,placement):
    if variant['category']=='baseline':
        if placement is not None or asset is not None:raise ValueError('Baseline cannot accept obstacle placement or assets')
        return None
    if not isinstance(placement,dict) or placement.get('task_sha256')!=digest(contract):
        raise ValueError('Placement must bind this exact task contract')
    if asset['asset_id'] in contract['asset_declarations'] or asset['asset_id']==contract['roles']['robot']['id']:
        raise ValueError('Risk asset identity collides with current task')
    allowed={'task_sha256','placement_mode','placement_evidence','placement_relations',
        'placement_parameter_ranges','position_xyz','rotation_xyzw','hs_surface_omissions'}
    if set(placement)-allowed:raise ValueError('Placement cannot override asset identity, physics or unknown fields')
    if not isinstance(placement.get('placement_evidence'),dict) or not placement['placement_evidence']:
        raise ValueError('Placement requires explicit task-specific provenance')
    mode=placement.get('placement_mode')
    if mode=='native_relations':
        if any(k in placement for k in ('position_xyz','rotation_xyzw')):raise ValueError('Native relations cannot include world poses')
        rows=placement.get('placement_relations');declared=set(contract['asset_declarations'])|{contract['roles']['robot']['id']}
        if not isinstance(rows,list) or not rows:raise ValueError('Explicit placement relations required')
        anchored=False
        for row in rows:
            if not isinstance(row,dict) or set(row)-{'kind','reference','params'}:raise ValueError('Invalid placement relation')
            kind=row.get('kind');params=row.get('params',{})
            if not isinstance(params,dict) or 'parent' in params:raise ValueError('Invalid reference parameters')
            if kind in ('on','next_to','face_to'):
                if row.get('reference') not in declared:raise ValueError('Placement reference is not in current task')
                anchored |= kind in ('on','next_to')
            elif kind=='rotate_around_solution':
                if 'reference' in row:raise ValueError('Rotation must not have a reference')
            else:raise ValueError('Unsupported placement relation kind')
            if any(isinstance(x,float) and not math.isfinite(x) for x in params.values()):raise ValueError('Nonfinite placement parameter')
        if not anchored:raise ValueError('Placement has no positional anchor')
        # Validate range declarations using the same sampler; do not persist or
        # apply this diagnostic draw. Actual capture freezes its own seed/draw.
        sample_placement_parameters(placement,0)
    elif mode=='world_pose':
        if any(k in placement for k in ('placement_relations','placement_parameter_ranges')):
            raise ValueError('World pose cannot include relation sampling')
        for key,count in [('position_xyz',3),('rotation_xyzw',4)]:
            values=placement.get(key)
            if not isinstance(values,list) or len(values)!=count or any(type(x) not in (int,float) or not math.isfinite(x) for x in values):
                raise ValueError('Explicit finite '+key+' required')
        if abs(sum(x*x for x in placement['rotation_xyzw'])-1)>1e-5:raise ValueError('Nonunit world quaternion')
    else:raise ValueError('Explicit world_pose or native_relations mode required')
    omissions=placement.get('hs_surface_omissions')
    if omissions is not None:
        if asset['category']!='HS' or not isinstance(omissions,dict):raise ValueError('Human omissions cannot be applied to a property/robot asset')
        paths=omissions.get('relative_colliders')
        if (omissions.get('source_usd_sha256')!=asset['source_sha256'] or not omissions.get('authorization') or not omissions.get('reason')
                or not isinstance(paths,list) or not paths or any(not isinstance(p,str) or p.startswith('/') or
                any(x in p for x in '*?[]') or any(x in p.split('/') for x in ('','..','.')) for p in paths) or len(set(paths))!=len(paths)):
            raise ValueError('Invalid source-bound explicit human omission; loaded inventory still required')
    result=deepcopy(placement);result.update({k:asset[k] for k in ('asset_id','usd_path','category','static')})
    result['placement_evidence'].update(preparer_version=VERSION,task_contract_sha256=digest(contract),
        asset_source_sha256=asset['source_sha256'],source_library_record_sha256=asset['source_library_record_sha256'],
        input_placement_sha256=digest(placement),runtime_placement_acceptance_inferred=False)
    result['prepared_binding_sha256']=digest(result)
    return result


def verify_prepared_binding(contract,binding):
    """Recheck bundle output at capture, before sampling or opening the scene."""
    evidence=binding.get('placement_evidence',{})
    if 'prepared_binding_sha256' not in binding and 'preparer_version' not in evidence:return False
    if evidence.get('preparer_version')!=VERSION or binding.get('task_sha256')!=digest(contract):
        raise ValueError('Prepared variant belongs to another task/version')
    copied=deepcopy(binding);identity=copied.pop('prepared_binding_sha256',None)
    if digest(copied)!=identity:raise ValueError('Prepared variant was modified; prepare a new bundle')
    if file_hash(binding['usd_path'])!=evidence.get('asset_source_sha256'):
        raise ValueError('Prepared risk asset source changed before capture')
    return True


def prepare(root,suite_path,library_path,human_path,task_id,variant_id,placement_path,output):
    output=Path(output).resolve()
    if output.exists():raise ValueError('Bundle output already exists')
    suite=read(suite_path);library=read(library_path);humans=read(human_path)
    contract,variant,asset=resolve_inputs(root,suite,library,humans,task_id,variant_id)
    placement=read(placement_path) if placement_path else None;binding=make_binding(contract,variant,asset,placement)
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value):(output/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    save('task_contract.json',contract);files=['task_contract.json']
    capture=['--task-contract',str(output/'task_contract.json')]
    if binding is not None:
        save('obstacle_binding.json',binding);save('input_placement.json',placement);save('asset_selection.json',asset)
        files+=['obstacle_binding.json','input_placement.json','asset_selection.json'];capture+=['--obstacle-binding',str(output/'obstacle_binding.json')]
    manifest=dict(version=VERSION,task_id=task_id,variant_id=variant_id,category=variant['category'],
        task_sha256=digest(contract),source_catalog_sha256=file_hash(suite_path),
        files_sha256={name:file_hash(output/name) for name in files},capture_arguments=capture,
        remaining_capture_arguments=['--output','--asset-cache','--policy-factory','--policy-config'],
        source_asset_dependencies='Runtime restoration and frozen inventory still required; USD root hash is not dependency closure',
        runtime_placement_verified=False,runtime_scope_verified=False,safety_evaluated=False,
        simulation_steps=0,geometry_queries=0,scope='Input identity and task/asset selection only; no scene acceptance or risk grade')
    save('variant_bundle.json',manifest);return manifest
