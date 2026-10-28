"""Independent original-relation environments for four additional mustard tasks.

Runtime factory only: no collector is launched by import/build, and existing
G-mustard environment/collector/shared modules are not patched or changed.
"""
from dataclasses import dataclass
import math
from pathlib import Path
from isaaclab_arena.environments.arena_environment_factory import ArenaEnvironmentCfg,ArenaEnvironmentFactory

ROOT=Path(__file__).resolve().parents[3]


@dataclass
class MustardFamilyCfg(ArenaEnvironmentCfg):
    preparation_json: str=''
    episode_length_s: float=120.


class MustardFamilyEnvironment(ArenaEnvironmentFactory[MustardFamilyCfg]):
    name='replicator_additional_mustard_source'
    _legacy_argparse_cfg_type=MustardFamilyCfg

    def build(self,cfg):
        if not cfg.preparation_json:raise ValueError('An explicit CPU source-preparation report is required')
        if not math.isfinite(cfg.episode_length_s) or cfg.episode_length_s<=0:raise ValueError('Invalid episode length')
        from .mustard_family_preparation import load_preparation
        preparation=load_preparation(cfg.preparation_json)
        layout,original,profile=preparation['layout'],preparation['original_task_spec'],preparation['profile']
        if not profile['physical_profile']['topology_applicable']:
            raise ValueError('The disclosed existing physical recipe is not applicable to this source topology')
        from isaaclab.envs.common import ViewerCfg
        from isaaclab_arena.assets.background import Background
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_reference import ObjectReference
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.relations.relations import IsAnchor,NextTo,On,RotateAroundSolution
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.tasks.pick_and_place_task import PickAndPlaceTask
        from isaaclab_arena.utils.pose import Pose
        from .mustard_family_background import family_mounted_kitchen
        from .local_droid import LocalDroid
        from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q
        kitchen_path,physics=family_mounted_kitchen(ROOT/layout['kitchen_usd'],ROOT/'reports/replicator_mustard_family_assets')
        # Runtime identity only: retain the exact audited recipe/overlay bytes.
        physics=dict(physics,recipe_source_path=(
            'kitchen_safety/extensions/replicator/mustard_family_background.py'
            if physics['recipe']=='additional-mustard-fixed-refrigerator-housing-only-v1'
            else 'kitchen_safety/extensions/replicator/background_physics.py'))
        kitchen=Background(name='kitchen',usd_path=str(kitchen_path),object_min_z=-.2,
            initial_pose=Pose.identity(),reset_nested_physics=True,**original['background']['params'])
        def pose(position,yaw):
            half=math.radians(yaw)/2
            return Pose(tuple(position),(0.,0.,math.sin(half),math.cos(half)))
        objects={name:Object(name=name,usd_path=str(ROOT/item['usd_path']),object_type=ObjectType.RIGID,
            initial_pose=pose(item['position'],item['yaw_deg'])) for name,item in layout['objects'].items()}
        refs={item['id']:ObjectReference(name=item['id'],parent_asset=kitchen,
            prim_path=kitchen.get_prim_path()+'/'+item['prim_path'],object_type=ObjectType.BASE,**item['params'])
            for item in original['object_references']}
        base=list(layout['robot_position']);base[2]+=layout['stand_height']
        initial,_=source_initial_q()
        robot=LocalDroid(enable_cameras=cfg.enable_cameras,stand_height_m=layout['stand_height'],
            initial_pose=pose(base,layout['robot_yaw_deg']),initial_joint_pose=list(initial.values()))
        robot.event_config.randomize_franka_joint_state.params['std']=0.
        entities={'kitchen':kitchen,'droid':robot,**objects,**refs}
        kinds={'is_anchor':IsAnchor,'on':On,'next_to':NextTo,'rotate_around_solution':RotateAroundSolution}
        for relation in original['relations']:
            cls=kinds[relation['kind']]
            args=[entities[relation['reference']]] if 'reference' in relation else []
            entities[relation['subject']].add_relation(cls(*args,**relation['params']))
        task_params=dict(original['task']['subtasks'][0]['params'])
        for name in ('pick_up_object','destination_location','background_scene'):task_params[name]=entities[task_params[name]]
        task=PickAndPlaceTask(**task_params,task_description=original['task']['description'],episode_length_s=cfg.episode_length_s)
        def viewer(env_cfg):
            camera=profile['extra_camera'];env_cfg.viewer=ViewerCfg(eye=tuple(camera['eye']),lookat=tuple(camera['target']))
            return env_cfg
        assets=[kitchen,*objects.values(),*refs.values(),self.asset_registry.get_asset_by_name('light')(),
                self.asset_registry.get_asset_by_name('directional_light')()]
        result=IsaacLabArenaEnvironment(name=layout['task_id']+'_safety',embodiment=robot,
            scene=Scene(assets=assets),task=task,env_cfg_callback=viewer)
        result.resolved_task_layout=layout
        result.source_task_spec=original
        result.source_relation_audit=preparation['evidence']['source_relations']
        result.background_physics_provenance=physics
        result.mustard_family_preparation=preparation
        return result
