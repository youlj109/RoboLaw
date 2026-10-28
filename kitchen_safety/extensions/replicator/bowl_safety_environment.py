"""Append one reviewed hazard to the unchanged original bowl definition."""
from dataclasses import dataclass
import math

from .bowl_safety_policy import CANONICAL, ROOT, load_layout, require


@dataclass
class BowlSafetyCfg:
    task_id: str = 'kitchen_bench_replicator_g_shape_bowl_sink'
    layout_path: str | None = None
    variant: str = 'human'
    enable_cameras: bool = True
    episode_length_s: float = 120.


class BowlSafetyEnvironment:
    def build(self, cfg):
        # All simulator imports stay inside build, after SimulationApp starts.
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from .task_environment import AssignedTaskEnvironment, AssignedTaskCfg
        layout, policy = load_layout(cfg.layout_path, cfg.variant)
        require(cfg.task_id == layout['task_id'], 'Bowl hazard cannot change the original task')
        result = AssignedTaskEnvironment().build(AssignedTaskCfg(task_id=cfg.task_id,
            layout_path=str(CANONICAL), variant='base', enable_cameras=cfg.enable_cameras,
            episode_length_s=cfg.episode_length_s))
        if cfg.variant == 'base':
            result.safety_policy = policy
            return result
        obstacle = layout['obstacle']; name = policy['hazard_name']
        require(name not in result.scene.assets, 'Hazard would overwrite an original scene asset')
        original_names = tuple(result.scene.assets)
        angle = math.radians(obstacle.get('yaw_deg', 0))
        asset = Object(name=name, usd_path=str(ROOT/obstacle['usd_path']),
            object_type=ObjectType.BASE if obstacle['static'] else ObjectType.RIGID,
            initial_pose=Pose(tuple(obstacle['position']), (0., 0., math.sin(angle/2), math.cos(angle/2))))
        result.scene.add_asset(asset)
        require(tuple(result.scene.assets) == original_names+(name,), 'Original scene asset inventory changed')
        result.resolved_task_layout = layout
        result.safety_policy = policy
        result.hazard_definition = dict(name=name, prim_path=asset.get_prim_path(), static=obstacle['static'],
            original_scene_assets=list(original_names), source_asset=obstacle['usd_path'])
        return result
