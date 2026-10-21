"""Arena external environment factory; no new safety metrics or policy logic."""
import json
import math
import copy
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

from isaaclab_arena.environments.arena_environment_factory import ArenaEnvironmentCfg, ArenaEnvironmentFactory

ROOT = Path(__file__).resolve().parents[1]

@dataclass
class KitchenSafetyCfg(ArenaEnvironmentCfg):
    variant: str = 'human'
    human_model: str = 'mpfb'
    object_jitter_m: float = 0.0
    collection_seed: int = 0
    obstacle_id: str = ''
    episode_spec: Optional[dict] = None
    episode_length_s: float = 120.0

class KitchenSafetyEnvironment(ArenaEnvironmentFactory[KitchenSafetyCfg]):
    name = 'kitchen_safety'
    _legacy_argparse_cfg_type = KitchenSafetyCfg

    def build(self, cfg):
        if not math.isfinite(cfg.episode_length_s) or cfg.episode_length_s <= 0:
            raise ValueError('episode_length_s must be finite and positive')
        from isaaclab.envs.common import ViewerCfg
        from isaaclab_arena.assets.background import Background
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.embodiments.droid import droid
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.tasks.pick_and_place_task import PickAndPlaceTask
        from isaaclab_arena.utils.pose import Pose

        from kitchen_safety.obstacle_library import resolve_layout
        if cfg.episode_spec is not None:
            if cfg.episode_spec['obstacle_id'] != cfg.obstacle_id:
                raise ValueError('Episode specification and environment obstacle IDs differ')
            layout = copy.deepcopy(cfg.episode_spec['layout'])
        else:
            layout = resolve_layout(cfg.obstacle_id, cfg.object_jitter_m, cfg.collection_seed)
        assert cfg.variant in ('base','human','property','robot')
        kitchen = Background(name='kitchen', usd_path=str(ROOT/'assets/kitchen/scene.usd'),
                             object_min_z=-0.2, initial_pose=Pose.identity(), reset_nested_physics=False)
        def obj(name, path, pos, static=False, yaw_deg=0):
            angle = math.radians(yaw_deg)
            return Object(name=name, usd_path=str(ROOT/path),
                          object_type=ObjectType.BASE if static else ObjectType.RIGID,
                          initial_pose=Pose(tuple(pos), (0., 0., math.sin(angle/2), math.cos(angle/2))))
        mustard = obj('mustard_bottle','assets/robolab/objects/hope/mustard_bottle.usd',layout['mustard_position'],
                      yaw_deg=layout.get('mustard_yaw_deg', 0))
        bowl = obj('bowl','assets/robolab/objects/ycb/bowl.usd',layout['bowl_position'],
                   yaw_deg=layout.get('bowl_yaw_deg', 0))
        # Use the original Arena robot/stand, downloaded locally with dependencies.
        prefix=ROOT/'assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets'
        old_robot,old_stand=droid._DROID_ROBOT_PRIM,droid._DROID_STAND_PRIM
        try:
            droid._DROID_ROBOT_PRIM=replace(old_robot,robot_usd_path=str(prefix/'robot_library/droid/franka_robotiq_2f_85_flattened.usd'))
            droid._DROID_STAND_PRIM=replace(old_stand,stand_usd_path=str(prefix/'object_library/srl_robolab_assets/robots/franka_stand_grey.usda'))
            yaw=math.radians(layout['robot_yaw_deg'])
            robot_position=list(layout['robot_position'])
            robot_position[2]+=layout['stand_height']
            # Keep the original DROID sensor mounts and intrinsics for every benchmark variant.
            embodiment=droid.DroidAbsoluteJointPositionEmbodiment(
                enable_cameras=cfg.enable_cameras,stand_height_m=layout['stand_height'],
                stand_footprint_xy_m=(0.4,0.4),
                initial_joint_pose=[math.radians(v) for v in (0,-36,0,-144,0,108,0)]+[0.]*6,
                initial_pose=Pose(tuple(robot_position),(0.,0.,math.sin(yaw/2),math.cos(yaw/2))))
        finally:
            droid._DROID_ROBOT_PRIM,droid._DROID_STAND_PRIM=old_robot,old_stand
        embodiment.event_config.randomize_franka_joint_state.params['std']=0.0
        assets=[kitchen,mustard,bowl,self.asset_registry.get_asset_by_name('light')(),
                self.asset_registry.get_asset_by_name('directional_light')()]
        if cfg.obstacle_id:
            obstacle = layout['obstacle']
            if obstacle['category'] != cfg.variant:
                raise ValueError(f"Obstacle {cfg.obstacle_id} belongs to {obstacle['category']}, not {cfg.variant}")
            name = {'human':'person', 'property':'glass', 'robot':'barrier'}[cfg.variant]
            assets.append(obj(name, obstacle['usd_path'], obstacle['position'],
                              obstacle['static'], obstacle.get('yaw_deg', 0)))
        elif cfg.variant=='human':
            if cfg.human_model not in ('mpfb','nvidia','blockout'):
                raise ValueError(f'Unknown human model: {cfg.human_model}')
            human_path=('assets/safety/person.usda' if cfg.human_model=='blockout' else
                        f'assets/people/{cfg.human_model}/collidable.usda')
            if cfg.human_model == 'mpfb':
                human_path = layout['human'].get('asset', human_path)
            assets.append(obj('person',human_path,layout['human']['position'],True,
                              layout['human'].get('yaw_deg',0)))
        elif cfg.variant=='property':
            for name in ('glass','plate','phone'):
                assets.append(obj(name,layout['property'].get(name+'_asset',f'assets/safety/{name}.usda'),
                                  layout['property'][name+'_position']))
        elif cfg.variant=='robot':
            assets.append(obj('barrier',layout['robot'].get('barrier_asset', 'assets/safety/barrier.usda'),layout['robot']['barrier_position'],True,
                              layout['robot'].get('barrier_yaw_deg', 0)))
        task=PickAndPlaceTask(mustard,bowl,kitchen,task_description=layout['task'],
                              episode_length_s=cfg.episode_length_s)
        def viewer(env_cfg):
            env_cfg.viewer=ViewerCfg(eye=tuple(layout['camera_eye']),lookat=tuple(layout['camera_target']))
            return env_cfg
        return IsaacLabArenaEnvironment(name=self.name,embodiment=embodiment,scene=Scene(assets=assets),
                                        task=task,env_cfg_callback=viewer)
