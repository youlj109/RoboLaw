"""Local assets adapter for the source Arena door graph, with unchanged task semantics."""
from pathlib import Path
from dataclasses import replace
ROOT = Path(__file__).resolve().parents[3]
TASK = ROOT/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench/kitchen_bench_lightwheel_open_fridge.yaml'

def build(variant='base', enable_cameras=True):
    from isaaclab_arena.environment_spec.arena_env_graph_spec import ArenaEnvGraphSpec
    from isaaclab_arena.environment_spec.arena_env_graph_conversion_utils import instantiate_assets_from_spec, _attach_spatial_relations_to_assets, _ensure_scene_lighting
    from isaaclab_arena.environment_spec.arena_env_graph_task_conversion_utils import build_task_from_spec
    from isaaclab_arena.assets.registries import AssetRegistry
    from isaaclab_arena.assets.background import Background
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.embodiments.droid import droid
    from isaaclab_arena.embodiments.embodiment_base import EmbodimentBase
    from isaaclab_arena.embodiments.robot_on_stand_utils import compose_on_stand_usd
    class LocalDroid(droid.DroidAbsoluteJointPositionEmbodiment):
        def __init__(self, enable_cameras=False, stand_height_m=.8):
            EmbodimentBase.__init__(self, enable_cameras=enable_cameras)
            self.gripper=droid.RobotiqGripper()
            self.stand_height_m=stand_height_m
            self.stand_footprint_xy_m=droid._DROID_STAND_PRIM.stand_default_footprint_xy_m
            self.placement_bbox_stand_only=False
            self.scene_config=droid.DroidSceneCfg()
            prefix=ROOT/'assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets'
            self.scene_config.robot.spawn.usd_path=compose_on_stand_usd(
                replace(droid._DROID_ROBOT_PRIM, robot_usd_path=str(prefix/'robot_library/droid/franka_robotiq_2f_85_flattened.usd')),
                replace(droid._DROID_STAND_PRIM, stand_usd_path=str(prefix/'object_library/srl_robolab_assets/robots/franka_stand_grey.usda')),
                stand_height_m=stand_height_m, stand_footprint_xy_m=self.stand_footprint_xy_m,
                output_basename='doors_droid_on_stand')
            self.camera_config=droid.DroidCameraCfg()
            self.observation_config=droid.DroidObservationsCfg()
            self.event_config=droid.DroidEventCfg()
            self.event_config.randomize_franka_joint_state.params['std']=0.
            self.action_config=droid.DroidAbsoluteJointPositionActionsCfg()
            self.reward_config=None
            self.mimic_env=None
            self.add_camera_variations(self.camera_config)
    class LocalRegistry:
        def get_asset_by_name(self,name):
            if name=='droid_abs_joint_pos':return LocalDroid
            if name=='lightwheel_kitchen_one_wall_coastal':
                return lambda **kw: Background(name='kitchen', usd_path=str(ROOT/'assets/kitchen/scene.usd'), object_min_z=-.2, initial_pose=Pose.identity(), **kw)
            return AssetRegistry().get_asset_by_name(name)
    graph=ArenaEnvGraphSpec.from_yaml(TASK)
    assets=instantiate_assets_from_spec(graph,LocalRegistry(),enable_cameras=enable_cameras)
    _ensure_scene_lighting(graph,assets)
    _attach_spatial_relations_to_assets(graph.relations,assets)
    objects=[v for k,v in assets.items() if k!=graph.embodiment.id]
    if variant!='base':
        import json, math
        spec=json.loads((ROOT/'configs/extensions/doors/open_fridge_variants.json').read_text())[variant]
        yaw=math.radians(spec.get('yaw_deg',0))
        objects.append(Object(name=spec['name'], usd_path=str(ROOT/spec['asset']),object_type=ObjectType.BASE if spec['static'] else ObjectType.RIGID, initial_pose=Pose(tuple(spec['position']),(0.,0.,math.sin(yaw/2),math.cos(yaw/2)))))
    task=build_task_from_spec(graph.task,assets)
    task.episode_length_s=120.
    env=IsaacLabArenaEnvironment(name=graph.env_name+'_'+variant,scene=Scene(assets=objects),embodiment=assets[graph.embodiment.id],task=task)
    return env,graph
