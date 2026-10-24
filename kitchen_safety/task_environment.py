"""Native Arena graph construction for all task contracts.

The caller must supply this attempt's obstacle placement. No Coastal coordinates
or risk exclusion lists are inherited. This builds an environment, not a policy.
"""
from copy import deepcopy
import math
from pathlib import Path
from dataclasses import replace
import hashlib
from contextlib import contextmanager
from urllib.parse import urlparse
from kitchen_safety.task_contract import digest


@contextmanager
def local_object_assets(spec,asset_cache=None):
    """Temporarily resolve matching installed official props, preserving class physics."""
    from isaaclab_arena.assets.registries import AssetRegistry
    root=Path(__file__).resolve().parents[1];changes=[];parameter_changes=[];receipt=[]
    try:
        for obj in [spec.background]+list(spec.objects):
            cls=AssetRegistry().get_asset_by_name(obj.registry_name)
            explicit=obj.params.get('usd_path')
            uri=explicit or vars(cls).get('usd_path')
            if not isinstance(uri,str) or not uri.startswith('https://'):continue
            parsed=urlparse(uri)
            if parsed.hostname not in ('omniverse-content-staging.s3-us-west-2.amazonaws.com',
                                       'omniverse-content-production.s3-us-west-2.amazonaws.com'):continue
            suffix=parsed.path.lstrip('/')
            if '..' in Path(suffix).parts:raise ValueError('Unexpected asset URL path traversal')
            candidates=[root/'assets/nvidia'/suffix]
            if asset_cache:candidates.append(Path(asset_cache)/'official'/parsed.hostname/suffix)
            marker='/srl_robolab_assets/'
            if marker in parsed.path:candidates.insert(0,root/'assets/robolab'/parsed.path.split(marker,1)[1])
            path=next((p for p in candidates if p.is_file()),None)
            if path is None:
                raise FileNotFoundError('Restore the official asset and dependency tree before capture: '+uri)
            dependency_receipt=None
            if asset_cache and path.is_relative_to(Path(asset_cache)/'official'):
                from kitchen_safety.task_asset_cache import verify_root
                dependency_receipt=verify_root(asset_cache,uri)
            if explicit:
                parameter_changes.append((obj,explicit));obj.params['usd_path']=str(path)
            else:
                changes.append((cls,uri));cls.usd_path=str(path)
            receipt.append(dict(registry_name=obj.registry_name,source_uri=uri,local_path=str(path),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),dependencies=dependency_receipt))
        yield receipt
    finally:
        for cls,uri in reversed(changes):cls.usd_path=uri
        for obj,uri in reversed(parameter_changes):obj.params['usd_path']=uri


def build_environment(contract, *, enable_cameras=False, obstacle=None,asset_cache=None,placement_seed=None):
    if obstacle is not None:
        from kitchen_safety.task_variant_bundle import verify_prepared_binding
        verify_prepared_binding(contract,obstacle)
    from kitchen_safety.curobo_backend import ensure_warp_torch_compat
    ensure_warp_torch_compat()
    from isaaclab_arena.environment_spec.arena_env_graph_spec import ArenaEnvGraphSpec
    from isaaclab_arena.assets.background import Background
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    if obstacle is not None:
        from kitchen_safety.task_variant import sample_placement_parameters
        obstacle=sample_placement_parameters(obstacle,placement_seed)
    spec = ArenaEnvGraphSpec.from_dict(deepcopy(contract['arena_environment_spec']))
    # Resolve the already installed exact DROID robot/stand before the native
    # graph constructor opens remote USD dependencies. Other embodiments never
    # receive this mapping. Restore module defaults even if building fails.
    robot_local = None
    if spec.embodiment.registry_name == 'droid_abs_joint_pos':
        from isaaclab_arena.embodiments.droid import droid
        root = Path(__file__).resolve().parents[1]
        prefix = root/'assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets'
        robot_path = prefix/'robot_library/droid/franka_robotiq_2f_85_flattened.usd'
        stand_path = prefix/'object_library/srl_robolab_assets/robots/franka_stand_grey.usda'
        if robot_path.is_file() and stand_path.is_file():
            old_robot, old_stand = droid._DROID_ROBOT_PRIM, droid._DROID_STAND_PRIM
            robot_local = dict(robot=dict(path=str(robot_path),sha256=hashlib.sha256(robot_path.read_bytes()).hexdigest()),
                stand=dict(path=str(stand_path),sha256=hashlib.sha256(stand_path.read_bytes()).hexdigest()))
            try:
                droid._DROID_ROBOT_PRIM=replace(old_robot,robot_usd_path=str(robot_path))
                droid._DROID_STAND_PRIM=replace(old_stand,stand_usd_path=str(stand_path))
                with local_object_assets(spec,asset_cache) as object_local:
                    environment=spec.to_arena_env(enable_cameras=enable_cameras)
            finally:
                droid._DROID_ROBOT_PRIM,droid._DROID_STAND_PRIM=old_robot,old_stand
        else:
            with local_object_assets(spec,asset_cache) as object_local:
                environment=spec.to_arena_env(enable_cameras=enable_cameras)
    else:
        with local_object_assets(spec,asset_cache) as object_local:
            environment=spec.to_arena_env(enable_cameras=enable_cameras)
    semantic_assets = {}
    backgrounds = [a for a in environment.scene.assets.values() if isinstance(a, Background)]
    if len(backgrounds) != 1:
        raise ValueError('Expected exactly the task-declared background')
    for aid, declaration in contract['asset_declarations'].items():
        asset = backgrounds[0] if declaration['kind'] == 'background' else environment.scene.assets.get(aid)
        if asset is None:
            raise ValueError('Native graph asset was not materialized: '+aid)
        semantic_assets[aid] = asset
    original_callback=environment.env_cfg_callback
    def causal_clock(cfg):
        if original_callback is not None:cfg=original_callback(cfg)
        cfg.sim.dt=1/120
        cfg.decimation=4
        return cfg
    environment.env_cfg_callback=causal_clock
    if obstacle is not None:
        required = {'asset_id','usd_path','static','category','placement_evidence'}
        if not required <= obstacle.keys():
            raise ValueError('Risk obstacle requires identity, static flag and placement provenance')
        if (obstacle['asset_id'] in semantic_assets or obstacle['asset_id'] in environment.scene.assets or
            obstacle['asset_id']==contract['roles']['robot']['id']):
            raise ValueError('Obstacle ID conflicts with a task asset')
        if obstacle['category'] not in ('HS','PT','RS') or not obstacle['placement_evidence']:
            raise ValueError('Invalid obstacle category/placement evidence')
        from kitchen_safety.task_variant import obstacle_relations,placement_reference_assets
        placement_assets=placement_reference_assets(contract,semantic_assets,environment.embodiment)
        relations=obstacle_relations(contract,obstacle,placement_assets)
        pose=None
        if relations is None:
            if len(obstacle.get('position_xyz', [])) != 3 or len(obstacle.get('rotation_xyzw', [])) != 4:
                raise ValueError('Obstacle pose dimensions differ')
            if not all(math.isfinite(x) for x in obstacle['position_xyz']+obstacle['rotation_xyzw']):
                raise ValueError('Obstacle pose is nonfinite')
            if abs(sum(x*x for x in obstacle['rotation_xyzw'])-1)>1e-5:
                raise ValueError('Obstacle quaternion is not normalized')
            pose=Pose(tuple(obstacle['position_xyz']),tuple(obstacle['rotation_xyzw']))
        if not isinstance(obstacle['static'], bool):
            raise ValueError('Obstacle static flag must be boolean')
        path = Path(obstacle['usd_path'])
        if not path.is_file():
            raise ValueError('Risk obstacle must be resolved locally before building: '+str(path))
        constructor=Object
        if obstacle['static'] and relations is not None:
            from kitchen_safety.task_variant import static_relation_object
            constructor=static_relation_object
        asset = constructor(name=obstacle['asset_id'], usd_path=str(path),
            object_type=ObjectType.BASE if obstacle['static'] else ObjectType.RIGID,
            initial_pose=pose,relations=relations or [])
        environment.scene.add_asset(asset); semantic_assets[obstacle['asset_id']] = asset
    return environment, dict(task_id=contract['task_id'], task_sha256=digest(contract),
        asset_path_templates={aid: a.get_prim_path() for aid,a in semantic_assets.items()},
        source_usd_paths={aid: getattr(a,'usd_path',None) for aid,a in semantic_assets.items()},
        obstacle=deepcopy(obstacle),local_robot_assets=robot_local,local_object_assets=object_local,
        placement_reference_roles={contract['roles']['robot']['id']:'current task robot embodiment; placement only, not a protected PT asset'})


def resolve_loaded_roots(stage, metadata, env_root):
    resolved = {}
    for aid, template in metadata['asset_path_templates'].items():
        path = template.replace('{ENV_REGEX_NS}', env_root)
        if any(c in path for c in '{}*') or not path.startswith(env_root+'/'):
            raise ValueError('Unresolved/cross-environment asset path: '+path)
        if not stage.GetPrimAtPath(path):
            raise ValueError('Loaded asset path does not exist: '+path)
        resolved[aid] = path
    return resolved
