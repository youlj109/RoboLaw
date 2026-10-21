"""Local-asset adaptation of the unmodified Arena toaster graph task.

The public graph, relations, task predicate and DROID camera/action configs are
retained. Only asset resolution is local to this module; no registry monkeypatch.
"""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[3]
TASK_ID = 'kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian'
SOURCE = ROOT / 'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench' / (TASK_ID + '.yaml')
KITCHEN = Path.home() / '.cache/lightwheel_sdk/floorplan/robocasa-robocasakitchen-9-8/scene.usd'


def build_environment(enable_cameras=True, variant='base', fixed_pose=None, trial_config=None):
    # cuRobo 0.7 imports Warp's pre-1.16 torch namespace. This compatibility
    # alias restores that namespace without changing any kernel or algorithm.
    import warp as wp
    from types import SimpleNamespace
    if not hasattr(wp, 'torch'):
        wp.torch = SimpleNamespace(device_from_torch=wp.device_from_torch)
    from dataclasses import replace
    from isaaclab_arena.assets.registries import AssetRegistry
    from isaaclab_arena.assets.background import Background
    from isaaclab_arena.embodiments.embodiment_base import EmbodimentBase
    from isaaclab_arena.embodiments.droid import droid
    from isaaclab_arena.embodiments.gripper import RobotiqGripper
    from isaaclab_arena.embodiments.robot_on_stand_utils import compose_on_stand_usd
    from isaaclab_arena.environment_spec.arena_env_graph_spec import ArenaEnvGraphSpec
    from isaaclab_arena.environment_spec.arena_env_graph_conversion_utils import (
        instantiate_assets_from_spec, _ensure_scene_lighting, _attach_spatial_relations_to_assets)
    from isaaclab_arena.environment_spec.arena_env_graph_task_conversion_utils import build_task_from_spec
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType

    class LocalDroid(droid.DroidAbsoluteJointPositionEmbodiment):
        def __init__(self, enable_cameras=False, stand_height_m=.8,
                     placement_bbox_stand_only=True, collision_mode='mesh', **kwargs):
            EmbodimentBase.__init__(self, enable_cameras=enable_cameras,
                                   collision_mode=collision_mode, **kwargs)
            self.gripper = RobotiqGripper()
            self.stand_height_m = stand_height_m
            self.stand_footprint_xy_m = droid._DROID_STAND_PRIM.stand_default_footprint_xy_m
            self.placement_bbox_stand_only = placement_bbox_stand_only
            self.scene_config = droid.DroidSceneCfg()
            prefix = ROOT / 'assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets'
            robot = replace(droid._DROID_ROBOT_PRIM, robot_usd_path=str(prefix / 'robot_library/droid/franka_robotiq_2f_85_flattened.usd'))
            stand = replace(droid._DROID_STAND_PRIM, stand_usd_path=str(prefix / 'object_library/srl_robolab_assets/robots/franka_stand_grey.usda'))
            self.scene_config.robot.spawn.usd_path = compose_on_stand_usd(
                robot, stand, stand_height_m=stand_height_m,
                stand_footprint_xy_m=self.stand_footprint_xy_m,
                output_basename='controls_droid_original_stand')
            self.camera_config = droid.DroidCameraCfg()
            self.observation_config = droid.DroidObservationsCfg()
            self.event_config = droid.DroidEventCfg()
            self.action_config = droid.DroidAbsoluteJointPositionActionsCfg()
            self.add_camera_variations(self.camera_config)
            self.event_config.randomize_franka_joint_state.params['std'] = 0.0

    class LocalRegistry:
        def get_asset_by_name(self, name):
            if name == 'droid_abs_joint_pos':
                return LocalDroid
            if name == 'lightwheel_kitchen_g_shaped_large_scandinavian':
                return lambda **kwargs: Background(name='kitchen', usd_path=str(KITCHEN),
                    object_min_z=-.2, initial_pose=Pose.identity(), reset_nested_physics=False, **kwargs)
            return AssetRegistry().get_asset_by_name(name)

    spec = ArenaEnvGraphSpec.from_yaml(SOURCE)
    assets = instantiate_assets_from_spec(spec, LocalRegistry(), enable_cameras=enable_cameras)
    _ensure_scene_lighting(spec, assets)
    _attach_spatial_relations_to_assets(spec.relations, assets)
    if fixed_pose is not None:
        assets[spec.embodiment.id].set_initial_pose(Pose(tuple(fixed_pose[:3]),tuple(fixed_pose[3:7])))
    if variant != 'base':
        import math
        config = trial_config if trial_config is not None else json.loads((ROOT/'configs/extensions/controls/left_button_trial.json').read_text())
        entry=config['variants'][variant]
        name={'human':'person','property':'glass','robot':'barrier'}[variant]
        path,xyz,yaw,static=entry['asset'],entry['position'],entry['yaw_deg'],variant!='property'
        a=math.radians(yaw)/2
        assets[name]=Object(name=name,usd_path=str(ROOT/path),
            object_type=ObjectType.BASE if static else ObjectType.RIGID,
            initial_pose=Pose(tuple(xyz),(0,0,math.sin(a),math.cos(a))))
    environment = IsaacLabArenaEnvironment(name=spec.env_name,
        scene=Scene(assets=[v for k, v in assets.items() if k != spec.embodiment.id]),
        embodiment=assets[spec.embodiment.id], task=build_task_from_spec(spec.task, assets))
    return environment, assets
