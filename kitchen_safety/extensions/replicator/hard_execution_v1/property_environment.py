"""Accepted glass-in-bowl layout adapted to the original Arena bowl task.

Only reset initial poses are authored. No simulation pose writes or welds.
Simulator imports are deferred until build(), after SimulationApp starts.
"""
from dataclasses import dataclass
from pathlib import Path
import copy
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = ROOT/'configs/extensions/replicator/hard_execution_v1/property.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_config(path=DEFAULT_CONFIG):
    cfg = json.loads(Path(path).read_text())
    for name, expected in cfg['input_sha256'].items():
        if sha(ROOT/name) != expected:
            raise ValueError('Property execution input changed: '+name)
    return cfg


def execution_layout(config):
    """Keep the source bowl/robot relations and accepted glass position."""
    layout = json.loads((ROOT/config['source_layout']).read_text())
    item = copy.deepcopy(config['glass'])
    layout['objects']['fragile_glass'] = {k:item[k] for k in ('usd_path','position','yaw_deg')}
    return layout


@dataclass
class PropertyCfg:
    config_path: str = str(DEFAULT_CONFIG)
    enable_cameras: bool = True
    episode_length_s: float = 240.


class PropertyEnvironment:
    def build(self, cfg):
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from kitchen_safety.extensions.replicator.task_environment import AssignedTaskEnvironment, AssignedTaskCfg
        config = read_config(cfg.config_path)
        result = AssignedTaskEnvironment().build(AssignedTaskCfg(task_id=config['task_id'],
            layout_path=str(ROOT/config['source_layout']), variant='base',
            enable_cameras=cfg.enable_cameras, episode_length_s=cfg.episode_length_s))
        original = tuple(result.scene.assets)
        glass = config['glass']; angle = math.radians(glass['yaw_deg'])
        result.scene.add_asset(Object(name='fragile_glass', usd_path=str(ROOT/glass['usd_path']),
            object_type=ObjectType.RIGID,
            initial_pose=Pose(tuple(glass['position']), (0.,0.,math.sin(angle/2),math.cos(angle/2)))))
        if tuple(result.scene.assets) != original+('fragile_glass',):
            raise ValueError('Original scene inventory changed while appending glass')
        result.resolved_task_layout = execution_layout(config)
        result.hard_execution_config = config
        result.hard_execution_scope = 'Physical glass relocation precedes unchanged bowl/sink task; no auto success from script phases'
        return result
