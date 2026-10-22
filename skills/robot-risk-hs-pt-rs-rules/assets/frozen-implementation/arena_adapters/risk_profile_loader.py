"""Explicit SafeBench import and Arena bindings; never import legacy controllers."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / 'third_party/safebench_reference/reference_code/evaluator'
if str(REFERENCE) not in sys.path:
    sys.path.insert(0, str(REFERENCE))
from safety_risk.evaluator_factory import create_evaluator as _create_reference_evaluator
from safety_risk.iso_ssm_candidate import digest, VERSION


def create_evaluator(profile):
    base=_create_reference_evaluator(profile)
    if profile.get('hs_dynamic',{}).get('enabled'):
        from kitchen_safety.hs_dynamic import DynamicHSEvaluator
        return DynamicHSEvaluator(base,profile)
    return base


def load_config(path):
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    config = json.loads(path.read_text())
    if config.get('schema_version') != 'arena-safety-observer-1' or config.get('mode') != 'shadow':
        raise ValueError('Expected arena-safety-observer-1 in shadow mode')
    if config.get('enabled') is not True or any(config.get('action_authority', {}).values()):
        raise ValueError('Observer must be enabled with no action authority')
    from kitchen_safety.pt_assets import validate_policy
    validate_policy(config.get('pt_asset_coverage', {}))
    from kitchen_safety.hs_data_policy import validate as validate_hs_data_policy
    validate_hs_data_policy(config.get('hs_scopes',{}).get('data_omissions'))
    profile_path = ROOT / config['profile_file']
    profile = json.loads(profile_path.read_text())
    if profile.get('rules',{}).get('pt_mode') != 'pt-hs-inspired-v4' or profile.get('pt_hs_inspired',{}).get('version') != 'pt-hs-inspired-v4':
        raise ValueError('This Arena adapter requires a PT v4 profile')
    geometry=config.get('pt_v4_geometry',{})
    from kitchen_safety.geometry_backend import descriptor,configured_name,get_backend
    backend_name=configured_name(config);spec=descriptor(backend_name)
    backend=get_backend(backend_name,config.get('mesh_distance_backend',{}).get('options'))
    if (geometry.get('version') != spec['pt_version'] or geometry.get('metric') != spec['metric']
            or geometry.get('target_source') != spec['target_source'] or geometry.get('environment_source') != spec['environment_source']
            or profile['pt_hs_inspired'].get('distance_metric') != spec['metric']
            or profile['pt_hs_inspired'].get('distance_sources') != {'robot_target':spec['target_source'],'environment':spec['environment_source']}
            or not geometry.get('allowed_grip_body_names')):
        raise ValueError('PT v4 backend identity, profile evidence sources and explicit grip bindings must match')
    import math
    for section in (config['grasp_evidence'],config['rs_evidence']['fault_motion']):
        if not section or any(isinstance(v,bool) or not isinstance(v,(int,float)) or
                              not math.isfinite(v) or v<=0 for v in section.values()):
            raise ValueError('Evidence protocol thresholds must be finite and positive')
    if config['hs_scopes']['fixed_body_names'] != ['panda_link0']:
        raise ValueError('This scene binding identifies panda_link0 as the fixed base and stand')
    if not isinstance(config['hs_scopes'].get('exempt_fixed_distance',False),bool):
        raise ValueError('HS fixed-distance exemption must be an explicit boolean')
    disabled = config.get('disabled_categories', [])
    if not isinstance(disabled, list) or any(c != 'IR' for c in disabled) or len(disabled) != len(set(disabled)):
        raise ValueError('disabled_categories must be [] or [IR]')
    if disabled:
        profile['rules']['disabled_categories'] = list(disabled)
    create_evaluator(profile)  # Validate the complete rule profile, not four thresholds.
    files = sorted((ROOT / 'kitchen_safety').glob('*.py')) + [ROOT/'scripts'/name for name in
        ('collect_kitchen_vectorized.py','collect_obstacle_library.py','export_safety_lerobot_v3.py',
         'render_recorded_episode.py','render_recorded_batch.py','validate_collection.py',
         'audit_collection_clearance.py')]
    files += sorted((REFERENCE/'safety_risk').glob('*.py'))
    files += sorted((ROOT/'native/gdist').glob('*'))+[ROOT/'scripts/build_gdist_backend.py']
    files=[p for p in files if p.is_file()]
    code = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    # These are ordinary configuration values in the existing resume comparison.
    # Editing a catalog pose/USD identity or the base layout must not silently
    # mix different scenes into an existing observer run.
    scene_ids=set(json.loads((ROOT/'configs/reproduction_presets.json').read_text())['nonhuman10']['ids'])
    scene_ids.add('human_default_mpfb')
    assets=[]
    for category in ('people','property','robot'):
        catalog=ROOT/'assets/obstacle_library'/category/'catalog.json'
        if catalog.exists():
            assets.extend(a for a in json.loads(catalog.read_text())['assets'] if a['id'] in scene_ids)
    scene_definition=dict(layout=json.loads((ROOT/'configs/layout.json').read_text()),
                          assets=sorted(assets,key=lambda a:a['id']))
    return dict(config=config, profile=profile, mesh_backend_runtime=backend.runtime_info() if hasattr(backend,'runtime_info') else spec, implementation_files_sha256=code, profile_file_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                scene_definition=scene_definition,adapter_version='arena-risk-observer-17-hs-data-omissions', evaluator_version=VERSION)


def bind_profile(settings, binding):
    profile = deepcopy(settings['profile'])
    profile['binding'] = {key: binding[key] for key in
        ('robot_id', 'operation_mode', 'payload_profile', 'object_class', 'scenario_id')}
    profile['runtime_binding'] = deepcopy(binding)
    profile['rules']['allowed_self_contact_pairs']=deepcopy(binding['rs_self_geometry']['allowed_pairs'])
    objects = profile['pt_hs_inspired']['objects']
    prototype = objects.pop('protected_obstacle')
    for name in binding['protected_environment_ids']:
        objects[name] = deepcopy(prototype)
        if 'pt_asset_inventory' in binding:
            objects[name]['arena_asset_binding'] = deepcopy(binding['pt_asset_inventory']['objects'][name])
    pairs = profile['pt_hs_inspired']['pairs']
    if profile['pt_hs_inspired']['version']=='pt-hs-inspired-v4':
        target=profile['pt_hs_inspired']['target_object_id']
        target_pair=next(p for p in pairs if p['actor']=='robot' and p['object_id']==target)
        target_pair['allowed_grip_link_paths']=list(binding['pt_grip_link_paths'])
        target_pair['grip_declaration_source']='Arena measured collider rigid-owner paths; explicit Robotiq role mapping in pt_v4_geometry'
    else:
        pairs[0]['allowed_grip_links'] = list(binding['gripper_contact_groups'])
    for name in binding['protected_environment_ids']:
        pairs.extend([{'actor': actor, 'object_id': name} for actor in ('robot', 'held_object')])
    if 'pt_asset_inventory' in binding:
        profile['arena_pt_asset_policy'] = deepcopy(binding['pt_asset_inventory'])
        if binding['pt_asset_inventory'].get('distance_omissions'):
            profile['pt_hs_inspired']['distance_omissions']=deepcopy(binding['pt_asset_inventory']['distance_omissions'])
    return profile
