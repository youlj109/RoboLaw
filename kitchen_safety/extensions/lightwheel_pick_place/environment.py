"""Local-asset adaptation of Arena Lightwheel pick-and-place graph tasks.

The public graph, relations, task predicate and DROID camera/action configs are
retained. Only asset resolution is local to this module; no registry monkeypatch.
"""
from pathlib import Path

from kitchen_safety.extensions.lightwheel_pick_place.task_config import (
    LIME_TASK, resolve_task,
)

ROOT = Path(__file__).resolve().parents[3]
# Back-compat module constants (B01 default).
TASK_ID = LIME_TASK
SOURCE = ROOT / "third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench" / (TASK_ID + ".yaml")
KITCHEN = Path.home() / ".cache/lightwheel_sdk/floorplan/robocasa-robocasakitchen-3-8/scene.usd"


def build_environment(
    enable_cameras=True,
    variant="base",
    *,
    task_id=None,
    fixed_layout_path=None,
    variant_config_path=None,
    native_background_reset=False,
):
    # cuRobo 0.7 imports Warp's pre-1.16 torch namespace. This compatibility
    # alias restores that namespace without changing any kernel or algorithm.
    import warp as wp
    from types import SimpleNamespace
    if not hasattr(wp, "torch"):
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

    spec_task = resolve_task(task_id)
    source = ROOT / "third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench" / (
        spec_task.task_id + ".yaml"
    )
    kitchen_usd = Path.home() / ".cache/lightwheel_sdk/floorplan" / spec_task.kitchen_cache_rel

    class LocalDroid(droid.DroidAbsoluteJointPositionEmbodiment):
        def __init__(self, enable_cameras=False, stand_height_m=.8,
                     placement_bbox_stand_only=True, collision_mode="mesh", **kwargs):
            EmbodimentBase.__init__(self, enable_cameras=enable_cameras,
                                   collision_mode=collision_mode, **kwargs)
            self.gripper = RobotiqGripper()
            self.stand_height_m = stand_height_m
            self.stand_footprint_xy_m = droid._DROID_STAND_PRIM.stand_default_footprint_xy_m
            self.placement_bbox_stand_only = placement_bbox_stand_only
            self.scene_config = droid.DroidSceneCfg()
            prefix = ROOT / "assets/nvidia/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets"
            robot = replace(droid._DROID_ROBOT_PRIM, robot_usd_path=str(prefix / "robot_library/droid/franka_robotiq_2f_85_flattened.usd"))
            stand = replace(droid._DROID_STAND_PRIM, stand_usd_path=str(prefix / "object_library/srl_robolab_assets/robots/franka_stand_grey.usda"))
            self.scene_config.robot.spawn.usd_path = compose_on_stand_usd(
                robot, stand, stand_height_m=stand_height_m,
                stand_footprint_xy_m=self.stand_footprint_xy_m,
                output_basename=spec_task.stand_output_basename)
            self.camera_config = droid.DroidCameraCfg()
            self.observation_config = droid.DroidObservationsCfg()
            self.event_config = droid.DroidEventCfg()
            self.action_config = droid.DroidAbsoluteJointPositionActionsCfg()
            self.add_camera_variations(self.camera_config)
            self.event_config.randomize_franka_joint_state.params["std"] = 0.0

    import json
    object_paths = json.loads((ROOT / "configs/extensions/lightwheel_pick_place/asset_paths.json").read_text())
    from isaaclab_arena.assets.object import Object

    class LocalRegistry:
        def get_asset_by_name(self, name):
            if name == "droid_abs_joint_pos":
                return LocalDroid
            if name == spec_task.kitchen_registry:
                return lambda **kwargs: Background(
                    name="kitchen", usd_path=str(kitchen_usd),
                    object_min_z=-.2, initial_pose=Pose.identity(), reset_nested_physics=bool(native_background_reset), **kwargs)
            if name in object_paths:
                row = object_paths[name]
                return type("Local_" + name, (AssetRegistry().get_asset_by_name(name),), {"usd_path": row["path"]})
            return AssetRegistry().get_asset_by_name(name)

    spec = ArenaEnvGraphSpec.from_yaml(source)
    assets = instantiate_assets_from_spec(spec, LocalRegistry(), enable_cameras=enable_cameras)
    # The stock ready pose intersects the over-counter cabinet in this layout.
    # Change only the reset joint configuration; retain source base relations,
    # task, object scale, cameras and all colliders. Never write state in execution.
    import math
    assets[spec.embodiment.id].set_initial_joint_pose(
        [math.radians(x) for x in (0, -60, 0, -150, 0, 90, 0)] + [0.] * 6)
    _ensure_scene_lighting(spec, assets)
    _attach_spatial_relations_to_assets(spec.relations, assets)
    if variant != "base" or fixed_layout_path is not None:
        from isaaclab_arena.relations.relations import IsAnchor, RotateAroundSolution
        layout_path = Path(fixed_layout_path) if fixed_layout_path is not None else spec_task.default_fixed_layout
        if not layout_path.exists():
            raise FileNotFoundError(
                "First collect and verify a base rollout, then save its original_asset_initial_poses to "
                + str(layout_path))
        layout = json.loads(layout_path.read_text())
        # Keep the original On/NextTo graph relations as checks, while anchoring
        # the exact original solved layout so adding a hazard cannot resample it.
        for name in ("droid",) + spec_task.rigid_objects:
            if name not in layout["original_asset_initial_poses"]:
                continue
            assets[name].set_initial_pose(Pose.from_dict(layout["original_asset_initial_poses"][name]))
            # RotateAroundSolution is a placement marker, not a spatial
            # relation. Its original yaw is already baked into the recorded
            # solved pose; anchors must not carry/apply that marker again.
            assets[name].relations = [r for r in assets[name].relations if not isinstance(r, RotateAroundSolution)]
            assets[name].add_relation(IsAnchor())
    if variant != "base":
        from isaaclab_arena.relations.relations import IsAnchor
        from isaaclab_arena.assets.object_type import ObjectType
        from math import sin, cos, radians
        variant_path = Path(variant_config_path) if variant_config_path is not None else spec_task.default_variants
        row = json.loads(variant_path.read_text())[variant]
        yaw = radians(row.get("yaw_deg", 0))
        assets[row["name"]] = Object(
            name=row["name"], usd_path=str(ROOT / row["asset"]),
            object_type=ObjectType.RIGID if variant == "property" else ObjectType.BASE,
            initial_pose=Pose(tuple(row["position"]), (0., 0., sin(yaw / 2), cos(yaw / 2))))
        assets[row["name"]].add_relation(IsAnchor())
    environment = IsaacLabArenaEnvironment(
        name=spec.env_name,
        scene=Scene(assets=[v for k, v in assets.items() if k != spec.embodiment.id]),
        embodiment=assets[spec.embodiment.id],
        task=build_task_from_spec(spec.task, assets))
    return environment, assets
