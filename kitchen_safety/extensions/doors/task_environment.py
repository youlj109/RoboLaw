"""Local assets adapter for the source Arena door graph, with unchanged task semantics."""
from pathlib import Path
from dataclasses import replace
ROOT = Path(__file__).resolve().parents[3]
TASK = ROOT/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench/kitchen_bench_lightwheel_open_fridge.yaml'

def build(task_id, variant='base', enable_cameras=True, variant_config=None):
    task_yaml=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench'/(task_id+'.yaml')
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
        def __init__(self, enable_cameras=False, stand_height_m=.8, stand_footprint_xy_m=None, placement_bbox_stand_only=False, collision_mode="mesh"):
            EmbodimentBase.__init__(self, enable_cameras=enable_cameras, collision_mode=collision_mode)
            self.gripper=droid.RobotiqGripper()
            self.stand_height_m=stand_height_m
            self.stand_footprint_xy_m=tuple(stand_footprint_xy_m or droid._DROID_STAND_PRIM.stand_default_footprint_xy_m)
            self.placement_bbox_stand_only=placement_bbox_stand_only
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
    import json
    selected_config=json.loads(Path(variant_config or ROOT/'configs/extensions/doors/c02_c03_initial_lwx.json').read_text())
    class LocalRegistry:
        def get_asset_by_name(self,name):
            if name=='droid_abs_joint_pos':return LocalDroid
            if name in ('lightwheel_kitchen_one_wall_coastal','lightwheel_kitchen_u_shaped_with_island_farmhouse2','lightwheel_kitchen_g_shaped_large_scandinavian'):
                paths={'lightwheel_kitchen_one_wall_coastal':ROOT/'assets/kitchen/scene.usd','lightwheel_kitchen_u_shaped_with_island_farmhouse2':Path.home()/'.cache/lightwheel_sdk/floorplan/robocasa-robocasakitchen-7-10/scene.usd','lightwheel_kitchen_g_shaped_large_scandinavian':Path.home()/'.cache/lightwheel_sdk/floorplan/robocasa-robocasakitchen-9-8/scene.usd'}
                return lambda **kw: Background(name='kitchen', usd_path=str(ROOT/selected_config['_kitchen_asset']) if selected_config.get('_kitchen_asset') else str(paths[name]), object_min_z=-.2, initial_pose=Pose.identity(), **kw)
            return AssetRegistry().get_asset_by_name(name)
    graph=ArenaEnvGraphSpec.from_yaml(task_yaml)
    assets=instantiate_assets_from_spec(graph,LocalRegistry(),enable_cameras=enable_cameras)
    _ensure_scene_lighting(graph,assets)
    _attach_spatial_relations_to_assets(graph.relations,assets)
    import json
    variant_data=json.loads(Path(variant_config or ROOT/'configs/extensions/doors/open_fridge_variants.json').read_text())
    if variant_data.get('_fixed_source_layout'):
        from isaaclab_arena.relations.relations import IsAnchor,RotateAroundSolution
        fixed=json.loads((ROOT/variant_data['_fixed_source_layout']).read_text())
        pose=fixed['robot_asset_pose']
        assets[graph.embodiment.id].set_initial_pose(Pose(tuple(pose['position']),tuple(pose['rotation'])))
        assets[graph.embodiment.id].relations=[r for r in assets[graph.embodiment.id].relations if not isinstance(r,RotateAroundSolution)]
        assets[graph.embodiment.id].add_relation(IsAnchor())
    objects=[v for k,v in assets.items() if k!=graph.embodiment.id]
    if variant!='base':
        import json, math
        spec=variant_data[variant]
        yaw=math.radians(spec.get('yaw_deg',0))
        objects.append(Object(name=spec['name'], usd_path=str(ROOT/spec['asset']),object_type=ObjectType.BASE if spec['static'] else ObjectType.RIGID, initial_pose=Pose(tuple(spec['position']),(0.,0.,math.sin(yaw/2),math.cos(yaw/2)))))
        if spec.get('support'):
            support=spec['support']
            objects.append(Object(name=support['name'],usd_path=str(ROOT/support['asset']),object_type=ObjectType.BASE,initial_pose=Pose(tuple(support['position']),(0.,0.,0.,1.))))
    task=build_task_from_spec(graph.task,assets)
    task.episode_length_s=120.
    env=IsaacLabArenaEnvironment(name=graph.env_name+'_'+variant,scene=Scene(assets=objects),embodiment=assets[graph.embodiment.id],task=task)
    return env,graph
