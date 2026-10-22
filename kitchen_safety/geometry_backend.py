"""Explicit mesh backend identity. No automatic backend or geometric fallback."""
from copy import deepcopy
import importlib

GIT_COMMIT='0300e214f0c9702e83292f35a5d07bdc27ea14ba'
DEFAULT_OPTIONS=dict(target=65536,maxstep=8,small=32768,concurrency=64,seeds=64)

def descriptor(name='gdist'):
    if name not in ('gdist','warp'):raise ValueError('Unknown mesh backend: '+str(name))
    label='gDist CUDA' if name=='gdist' else 'Warp CUDA'
    return dict(id=name,label=label,metric=name+'_authored_collision_surface_distance',
        hs_version='arena-hs-'+name+'-triangle-surface-1',pt_version='arena-pt-'+name+'-triangle-surface-1',
        hs_source=label+' separation of USD collision surfaces (polygon/primitive surface tessellation); world_m; xyzw',
        target_source='link_target_'+name+'_surface_distance_gt',environment_source='pt_environment_'+name+'_surface_distance_gt',
        query_execution=('frame-wide gDist CUDA triangle batches; persistent BVHs, shared HS/PT/RS queue, asynchronous CUDA streams'
                         if name=='gdist' else 'frame-wide Warp CUDA triangle batches; shared HS/PT/RS queue and CUDA graphs'))

def configured_name(config):
    name=config.get('mesh_distance_backend',{}).get('id')
    if name is None:
        metric=config.get('pt_v4_geometry',{}).get('metric','warp_authored_collision_surface_distance')
        name='gdist' if metric=='gdist_authored_collision_surface_distance' else 'warp'
    descriptor(name);return name

def get_backend(name='gdist',options=None):
    descriptor(name)
    module=importlib.import_module('kitchen_safety.'+name+'_mesh_backend')
    if name=='gdist':module.configure(DEFAULT_OPTIONS if options is None else options)
    return module

def migrate_frozen(frozen,name):
    """Derived binding/profile only; source archive remains immutable."""
    x=deepcopy(frozen);s=descriptor(name);config=x['resolved_config'];profile=x['resolved_profile']
    config['mesh_distance_backend']=dict(id=name,version='arena-gdist-runtime-1' if name=='gdist' else 'arena-warp-runtime-1',
        options=deepcopy(DEFAULT_OPTIONS) if name=='gdist' else {},source_commit=GIT_COMMIT if name=='gdist' else None,missing_policy='unknown_no_fallback')
    g=config['pt_v4_geometry'];g.update(version=s['pt_version'],metric=s['metric'],target_source=s['target_source'],environment_source=s['environment_source'])
    config['hs_scopes']['distance_backend']=s['label']+' full triangle-surface queries; conservative AABB pruning only'
    config['rs_evidence']['self_collision']='Independent authored '+s['label']+' surfaces; explicit structural exclusions retained'
    pt=profile['pt_hs_inspired'];pt.update(distance_metric=s['metric'],distance_sources={'robot_target':s['target_source'],'environment':s['environment_source']})
    if 'target_cup_distance_metric' in pt:pt['target_cup_distance_metric']=s['metric']
    if 'target_cup_distance_source' in pt:pt['target_cup_distance_source']=s['target_source']
    if 'target_cup_distance_limitation' in pt:pt['target_cup_distance_limitation']=s['label']+' authored triangle surfaces; not cooked PhysX solids or penetration depths'
    profile.setdefault('arena_semantics',{})['distance_source']=s['metric']
    profile['source_profile_description']=profile.get('description')
    profile['description']='Derived '+s['label']+' backend; source physical thresholds, HS/PT/RS rules and exemptions retained'
    profile['mesh_distance_backend']=deepcopy(config['mesh_distance_backend'])
    profile['hs_geometry']={**profile.get('hs_geometry',{}),'distance_source':s['metric']}
    binding=x['binding']
    binding['geometry_sources']={'HS':s['hs_source'] if binding.get('human_inventory',{}).get('observed_human_paths') else None,'PT':s['metric']}
    for key in ('hs_geometry','rs_authored_geometry'):
        meta=binding.get(key)
        if meta:meta.update(version=s['hs_version'],source=s['hs_source'],query_execution=s['query_execution'],mesh_backend=name)
    meta=binding['pt_geometry'];meta.update(version=s['pt_version'],metric=s['metric'],target_source=s['target_source'],environment_source=s['environment_source'],query_execution=s['query_execution'],mesh_backend=name,
        formula='min_collider_pairs('+s['label']+'_triangle_surface_distance)',query_strategy='Complete authored triangle-surface minima; per-link target distances retained')
    binding['rs_self_geometry'].update(query_execution=s['query_execution'],source='Independent '+s['label']+' authored collision-surface intersection at measured link poses',mesh_backend=name)
    profile['runtime_binding']=deepcopy(binding)
    from kitchen_safety.risk_profile_loader import digest
    x['profile_sha256']=digest(profile)
    effective=get_backend(name,config['mesh_distance_backend'].get('options'))
    x['mesh_backend_runtime']=effective.runtime_info() if hasattr(effective,'runtime_info') else s
    x['geometry_backend_migration']=dict(source_backend=configured_name(frozen['resolved_config']),requested_backend=name,
        source_profile_sha256=frozen['profile_sha256'],effective_profile_sha256=x['profile_sha256'],
        semantics='Explicit derived backend provenance; physical evidence, rules, thresholds and exemptions unchanged')
    return x
