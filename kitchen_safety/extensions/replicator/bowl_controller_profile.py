"""Source-bound rim grasp and sink landing shared by base and future variants.

This only computes a task-region target. It never accepts a path, masks an
obstacle, writes simulator state or certifies the pending physical controller.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from .bowl_motion import bowl_rim_grasp, sink_reference_bounds, sink_placement_object_pose

ROOT = Path(__file__).resolve().parents[3]
TASK = 'kitchen_bench_replicator_g_shape_bowl_sink'
GRASP_REVISION = 'observed_close_candidate_25deg_v2'
GRASP_ADJUSTMENT = {
    'rotation_frame': 'world',
    'rotation_axis': [1., 0., 0.],
    'rotation_deg': 25.,
    'translation_world_m': [0., 0., 0.005039543828685633],
}


def require_grasp_profile(profile):
    """One reviewed correction only; never widen the local contact policy."""
    if (profile.get('schema_version') != 2 or profile.get('grasp_revision') != GRASP_REVISION or
        profile.get('grasp_adjustment') != GRASP_ADJUSTMENT or
        profile.get('contact_region_rule') != 'unchanged_predeclared_bowl_rim_geometry' or
        profile.get('baseline_and_variants_share_profile') is not True):
        raise ValueError('Unreviewed bowl grasp correction or changed local contact region')


def read_profile(path=None, layout_path=None):
    path = Path(path) if path is not None else ROOT/'configs/extensions/replicator/g_shape_bowl_sink_controller.json'
    layout_path = Path(layout_path) if layout_path is not None else ROOT/'configs/extensions/replicator/g_shape_bowl_sink_source_layout.json'
    profile, layout = json.loads(path.read_text()), json.loads(layout_path.read_text())
    digest = lambda item: hashlib.sha256(Path(item).read_bytes()).hexdigest()
    if profile['task_id'] != TASK or layout['task_id'] != TASK:
        raise ValueError('Source bowl controller profile belongs to another task')
    bound = {'source_layout_sha256': layout_path,
        'source_bowl_sha256': ROOT/layout['objects']['bowl']['usd_path'],
        'source_kitchen_sha256': ROOT/layout['kitchen_usd']}
    if profile.get('schema_version') == 2:
        require_grasp_profile(profile)
        bound['source_grasp_geometry_sha256'] = ROOT/'kitchen_safety/extensions/replicator/bowl_motion.py'
    elif profile.get('schema_version') != 1:
        raise ValueError('Unknown source bowl controller schema')
    if any(profile[key] != digest(source) for key, source in bound.items()):
        raise ValueError('Source layout or assets changed; a previous CPU candidate cannot be silently reused')
    if profile['release_gap_m'] != .003 or profile['grasp_rule'] != 'source_negative_x_upper_wall_rim':
        raise ValueError('Unreviewed source grasp/release profile')
    if profile['sink_target']['source_reference'] != '/World/envs/env_0/kitchen/Sink/BasinBottom':
        raise ValueError('The original task destination must remain BasinBottom')
    offset = np.asarray(profile['sink_target']['offset_xy_m'], dtype=float)
    if offset.shape != (2,) or not np.isfinite(offset).all():
        raise ValueError('Invalid original sink-region target offset')
    if profile.get('schema_version') == 2:
        for name, expected in profile['evidence'].items():
            if digest(ROOT/name) != expected:
                raise ValueError('Reviewed bowl preparation evidence changed: '+name)
    return profile


def grasp_target(payload_pose, world, profile):
    """Apply the fixed correction to the source target from the live bowl pose.

    This function reads no recording and accepts no hazard-specific correction.
    The region is the predeclared local source-geometry policy, not a box from
    upstream YAML. Runtime contact/attachment/safety gates remain independent.
    """
    from scipy.spatial.transform import Rotation
    require_grasp_profile(profile)
    if (world.payload_name != 'bowl' or world.payload_geometry.provenance['source_sha256'] != profile['source_bowl_sha256'] or
        world.contact_policy.source_support != '/World/envs/env_0/kitchen/CapSurface_02'):
        raise ValueError('Runtime source geometry differs from the reviewed bowl grasp')
    target, report = bowl_rim_grasp(payload_pose, world.payload_geometry, world)
    original = target.copy()
    adjustment = profile['grasp_adjustment']
    rotation = Rotation.from_rotvec(np.deg2rad(adjustment['rotation_deg'])*np.asarray(adjustment['rotation_axis'])).as_matrix()
    target[:3, :3] = rotation@target[:3, :3]
    target[:3, 3] += np.asarray(adjustment['translation_world_m'])
    report.update(unadjusted_source_tcp=original.tolist(), target_tcp_world_m=target[:3, 3].tolist(),
        target_tcp_world=target.tolist(), grasp_revision=profile['grasp_revision'],
        fixed_grasp_adjustment=adjustment.copy(), baseline_and_variants_share_profile=True,
        input_pose='Current measured bowl pose; source rim target recomputed each run, no recorded pose lookup',
        contact_region_origin='Unchanged local predeclared source-geometry policy; not a numerical upstream YAML constraint',
        physical_grasp_verified=False)
    return target, report


def sink_target(payload_pose, world, profile):
    if world.payload_name != 'bowl' or world.contact_policy.destination_name != profile['sink_target']['source_reference']:
        raise ValueError('The runtime world does not match the source bowl/sink profile')
    if world.payload_geometry.provenance['source_sha256'] != profile['source_bowl_sha256']:
        raise ValueError('Runtime payload source differs from the prepared source bowl')
    low, high = sink_reference_bounds(world)
    center = (low[:2]+high[:2])/2 + np.asarray(profile['sink_target']['offset_xy_m'])
    target, report = sink_placement_object_pose(payload_pose, world.payload_geometry, world,
        gap=profile['release_gap_m'], center_xy=center)
    report.update(target_rule=profile['sink_target'], baseline_and_variants_share_profile=True,
        input_pose='Live measured payload orientation; position recomputed from original source floor',
        collision_or_task_success_verified=False)
    return target, report
