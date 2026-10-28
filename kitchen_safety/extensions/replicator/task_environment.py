"""Independent source-preserving environments for the next Replicator tasks.

The existing mustard environment is deliberately not imported or modified.
This factory currently supports the measured G banana/bowl and G bowl/sink
layouts. A successful build is not a physical-success or collection result.
"""
from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass
from pathlib import Path

from isaaclab_arena.environments.arena_environment_factory import ArenaEnvironmentCfg, ArenaEnvironmentFactory

ROOT = Path(__file__).resolve().parents[3]
TASK_LAYOUTS = {
    "kitchen_bench_replicator_g_shape_banana_bowl": "g_shape_banana_bowl_source_layout.json",
    "kitchen_bench_replicator_g_shape_bowl_sink": "g_shape_bowl_sink_source_layout.json",
}


def resolve_task_layout(task_id, layout_path=None):
    if task_id not in TASK_LAYOUTS:
        raise ValueError(f"Task environment is not implemented: {task_id}")
    path = ROOT / "configs/extensions/replicator" / TASK_LAYOUTS[task_id]
    reference = json.loads(path.read_text())
    layout = json.loads(Path(layout_path).read_text()) if layout_path else copy.deepcopy(reference)
    if {key: value for key, value in layout.items() if key != "obstacle"} != reference:
        raise ValueError("A hazard layout must preserve the measured original task layout exactly")
    return layout


@dataclass
class AssignedTaskCfg(ArenaEnvironmentCfg):
    task_id: str = "kitchen_bench_replicator_g_shape_banana_bowl"
    layout_path: str | None = None
    variant: str = "base"
    episode_length_s: float = 120.0


class AssignedTaskEnvironment(ArenaEnvironmentFactory[AssignedTaskCfg]):
    name = "replicator_assigned_pickplace_safety"
    _legacy_argparse_cfg_type = AssignedTaskCfg

    def build(self, cfg):
        if not math.isfinite(cfg.episode_length_s) or cfg.episode_length_s <= 0:
            raise ValueError("episode_length_s must be finite and positive")
        if cfg.variant not in ("base", "human", "property", "robot"):
            raise ValueError(f"Unsupported safety category: {cfg.variant}")
        if cfg.variant != "base":
            raise NotImplementedError(
                "No reviewed task-specific hazard policy is registered yet for this new task. "
                "Define its original policy values/provenance and validate the layout before enabling variants; "
                "arbitrary nonnegative clearances are not an execution policy.")
        from isaaclab.envs.common import ViewerCfg
        from isaaclab_arena.assets.background import Background
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_reference import ObjectReference
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.relations.relations import IsAnchor, NextTo, On, RotateAroundSolution
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.tasks.pick_and_place_task import PickAndPlaceTask
        from isaaclab_arena.utils.pose import Pose

        from kitchen_safety.extensions.replicator.background_physics import mounted_kitchen
        from kitchen_safety.extensions.replicator.local_droid import LocalDroid
        from scripts.extensions.replicator.audit_task_source_relation import check_layout, source

        layout = resolve_task_layout(cfg.task_id, cfg.layout_path)
        if "obstacle" in layout:
            raise ValueError("Base environment must contain no added hazard")
        audit = check_layout(layout)
        if not audit["initial_source_relations_passed"]:
            raise ValueError(f"Original source relations failed: {audit['checks']}")
        _, original = source(cfg.task_id)
        if original["task"]["composition"] != "atomic" or len(original["task"]["subtasks"]) != 1:
            raise ValueError("This environment adapter only implements atomic source tasks")
        subtask = original["task"]["subtasks"][0]
        if subtask["kind"] != "PickAndPlaceTask":
            raise ValueError("Keep the original task type; this adapter does not implement it")
        if cfg.task_id == "kitchen_bench_replicator_g_shape_banana_bowl":
            from kitchen_safety.extensions.replicator.banana_background_physics import mounted_banana_kitchen
            kitchen_path, physics = mounted_banana_kitchen(
                ROOT / layout["kitchen_usd"], ROOT / "reports/replicator_task_assets", layout=layout)
        else:
            kitchen_path, physics = mounted_kitchen(
                ROOT / layout["kitchen_usd"], ROOT / "reports/replicator_task_assets")
        kitchen = Background(name="kitchen", usd_path=str(kitchen_path), object_min_z=-.2,
                             initial_pose=Pose.identity(), reset_nested_physics=True,
                             **original["background"]["params"])

        def pose(position, yaw_deg=0.):
            yaw = math.radians(yaw_deg)
            return Pose(tuple(position), (0., 0., math.sin(yaw / 2), math.cos(yaw / 2)))

        objects = {name: Object(name=name, usd_path=str(ROOT / item["usd_path"]),
                    object_type=ObjectType.RIGID, initial_pose=pose(item["position"], item["yaw_deg"]))
                   for name, item in layout["objects"].items()}
        references = {ref["id"]: ObjectReference(name=ref["id"], parent_asset=kitchen,
                        prim_path=f"{kitchen.get_prim_path()}/{ref['prim_path']}",
                        object_type=ObjectType.BASE, **ref["params"])
                      for ref in original["object_references"]}
        base = list(layout["robot_position"])
        base[2] += layout["stand_height"]
        robot = LocalDroid(enable_cameras=cfg.enable_cameras, stand_height_m=layout["stand_height"],
            initial_pose=pose(base, layout["robot_yaw_deg"]),
            initial_joint_pose=[math.radians(value) for value in (0, -36, 0, -144, 0, 108, 0)] + [0.] * 6)
        robot.event_config.randomize_franka_joint_state.params["std"] = 0.
        entities = {"kitchen": kitchen, "droid": robot, **objects, **references}
        relations = {"is_anchor": IsAnchor, "next_to": NextTo, "on": On,
                     "rotate_around_solution": RotateAroundSolution}
        for row in original["relations"]:
            args = [entities[row["reference"]]] if "reference" in row else []
            entities[row["subject"]].add_relation(relations[row["kind"]](*args, **row["params"]))
        assets = [kitchen, *objects.values(), *references.values(),
                  self.asset_registry.get_asset_by_name("light")(),
                  self.asset_registry.get_asset_by_name("directional_light")()]
        task_params = dict(subtask["params"])
        for key in ("pick_up_object", "destination_location", "background_scene"):
            task_params[key] = entities[task_params[key]]
        task = PickAndPlaceTask(**task_params, task_description=original["task"]["description"],
                                episode_length_s=cfg.episode_length_s)
        center = layout["objects"][subtask["params"]["pick_up_object"]]["position"]
        def viewer(env_cfg):
            env_cfg.viewer = ViewerCfg(eye=(center[0] - 1.5, center[1] - 1.5, center[2] + 1.5), lookat=tuple(center))
            return env_cfg
        result = IsaacLabArenaEnvironment(name=cfg.task_id + "_safety", embodiment=robot,
            scene=Scene(assets=assets), task=task, env_cfg_callback=viewer)
        result.source_relation_audit = audit
        result.source_task_spec = original
        result.resolved_task_layout = layout
        result.background_physics_provenance = physics
        return result
