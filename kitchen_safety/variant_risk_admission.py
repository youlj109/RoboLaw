"""Explicit additive obstacle admission; preserve the reviewed native ceiling.

Only the single frozen ylj hazard is added. No numerical rule is changed and no
geometry query is performed here. Existing exclusions remain independently bound.
"""
from copy import deepcopy
from pathlib import Path
import hashlib,json
from kitchen_safety.task_contract import digest

VERSION='robolaw-single-variant-risk-admission-1'
POLICY='configs/risk_integration/replicator154_added_obstacles_20261029.json'

def under(path,root):return path==root or path.startswith(root.rstrip('/')+'/')

def declaration(root,contract,entry,layout):
    root=Path(root);policy=json.loads((root/POLICY).read_text())
    if (policy['version']!=VERSION or not policy.get('authorization')
            or entry['variant_id'] not in policy['variants']
            or entry['task_id']!='kitchen_bench_'+contract['task_id']
            or layout['obstacle']['id']!=entry['variant_id']):
        raise ValueError('Added-obstacle risk admission is not authorized for this scene')
    index=root/'configs/extensions/replicator/final_154_scenes_20261028.json'
    if hashlib.sha256(index.read_bytes()).hexdigest()!=policy['scene_index_sha256']:
        raise ValueError('Authorized 154-scene index changed')
    obstacle=layout['obstacle'];asset=root/obstacle['usd_path']
    if (obstacle['usd_path'] not in policy['variant_asset_paths'][entry['variant_id']] or
            policy['asset_sha256_by_path'][obstacle['usd_path']]!=hashlib.sha256(asset.read_bytes()).hexdigest()):
        raise ValueError('Authorized added obstacle USD changed')
    result=dict(version=VERSION,authorization=policy['authorization'],policy_sha256=digest(policy),
        task_id=contract['task_id'],scene_id=entry['scene_id'],variant_id=entry['variant_id'],
        category='HS' if entry['variant_id']=='human_default_mpfb' else 'PT',
        catalog_category=obstacle['category'],asset_id='hazard',usd_path=str(asset),
        usd_sha256=policy['asset_sha256_by_path'][obstacle['usd_path']],source_relative_usd=obstacle['usd_path'],base_assets_may_expand=False,
        scope='exact_single_added_obstacle_only',RS='unchanged robot torque/joint/self/fault rules; external obstacle contacts remain task-bound PT')
    return result

def obstacle_binding(admission,policy):
    result=dict(asset_id='hazard',category=admission['category'],usd_path=admission['usd_path'])
    if result['category']=='HS':
        result['hs_surface_omissions']=dict(source_usd_sha256=admission['usd_sha256'],
            relative_colliders=policy['human_omitted_relative_colliders_by_usd'][admission['source_relative_usd']],
            authorization=policy['authorization']+' Preserve the previously authorized casual-suit HS exclusion on this explicitly mapped ylj posed adult revision.',
            reason='exact_authorized_casual_suit_surface_mapping')
    return result

def bind(inventory,bindings,admission,base_lock):
    if admission['version']!=VERSION or admission['task_id']!=base_lock['task_id']:
        raise ValueError('Variant admission task/ceiling mismatch')
    root=bindings['asset_roots'][admission['asset_id']]
    entries=[e for e in inventory['colliders'] if under(e['path'],root)]
    if not entries:raise ValueError('Added obstacle has no frozen collision surface')
    if under(root,bindings['robot_root']) or any(under(root,r) or under(r,root) for aid,r in bindings['asset_roots'].items() if aid!='hazard'):
        raise ValueError('Added obstacle overlaps another semantic root')
    targets={}
    if admission['category']=='PT':
        for e in entries:
            owner=e.get('actor') or root
            if not under(owner,root):raise ValueError('Added obstacle collider owner escapes its root')
            targets.setdefault(owner,[]).append(e['path'])
        if root in bindings['human_roots']:raise ValueError('PT hazard cannot also bind a human')
    else:
        if bindings['human_roots']!=[root]:raise ValueError('HS admission requires exactly the declared human root')
    omitted={v['collider_path'] for v in bindings['hs_human_surface_omissions']}
    bound=dict(admission,asset_root=root,collider_paths=sorted(e['path'] for e in entries),
        pt_targets={k:sorted(v) for k,v in targets.items()},
        human_collision_paths=sorted(e['path'] for e in entries if e['path'] not in omitted) if admission['category']=='HS' else [],
        base_lock_sha256=digest(base_lock),base_pt_targets=deepcopy(base_lock['pt_targets']))
    bound['binding_sha256']=digest(bound);bindings['variant_risk_admission']=bound
    return bound

def workspace(policy,bound):
    if bound['category']!='PT':return policy
    result=deepcopy(policy)
    # The added obstacle participates within the same arm reach envelope, rather
    # than inheriting a manually selected original support-table footprint.
    for field in ('role_roots','whole_body_role_roots'):
        if bound['asset_root'] not in result[field]:result[field].append(bound['asset_root'])
    result['variant_risk_admission_sha256']=bound['binding_sha256']
    result.pop('policy_sha256',None);result['policy_sha256']=digest(result)
    return result

def add_targets(policy,bindings,targets,env):
    bound=bindings.get('variant_risk_admission')
    if not bound or bound['category']!='PT':return policy
    result=deepcopy(policy)
    if result is None:raise ValueError('Reviewed variant capture requires an existing workspace target policy')
    rows={env+'/'+r['target']:r for r in result['included_targets']}
    for target,paths in bound['pt_targets'].items():
        if set(paths)!={e['path'] for e in targets.get(target,[])}:
            raise ValueError('Added obstacle target topology differs')
        rows[target]=dict(target=target[len(env)+1:],colliders=[p[len(env)+1:] for p in paths],
            risk_colliders=[p[len(env)+1:] for p in paths])
    result['included_targets']=[rows[k] for k in sorted(rows)]
    result['authorization']+='; '+bound['authorization']
    result['selection_basis']['variant_risk_admission_sha256']=bound['binding_sha256']
    return result

def extend_lock(base,bound,env):
    result=deepcopy(base)
    for target,paths in bound['pt_targets'].items():
        relative=target[len(env)+1:]
        if relative in result['pt_targets']:raise ValueError('New hazard is not distinct from native assets')
        result['pt_targets'][relative]=[p[len(env)+1:] for p in paths]
    result['human_collision_paths']=sorted(set(result['human_collision_paths'])|{p[len(env)+1:] for p in bound['human_collision_paths']})
    result['variant_risk_admission']=deepcopy(bound)
    assert {k:v for k,v in result['pt_targets'].items() if k in base['pt_targets']}==base['pt_targets']
    return result

def validate_scope(scope,bound):
    env=scope['bindings']['robot_root'].rsplit('/',1)[0]
    native={env+'/'+k for k in bound['base_pt_targets']}
    actual={r['target'] for r in scope['relations'] if r['category']=='PT'}
    if not actual<=native|set(bound['pt_targets']):raise ValueError('Variant expanded native PT asset scope')
    if not set(bound['pt_targets'])<=actual:raise ValueError('Added PT hazard was omitted from risk relations')
    if bound['category']=='HS':
        actual_humans=set(scope['human_inventory']['collision_paths'])
        if actual_humans!=set(bound['human_collision_paths']) or scope['human_inventory']['verified_absent']:
            raise ValueError('Added human is not fully bound or incorrectly marked absent')
        if not actual_humans<={r['target'] for r in scope['relations'] if r['category']=='HS'}:
            raise ValueError('Added human surface has no active HS relation')
