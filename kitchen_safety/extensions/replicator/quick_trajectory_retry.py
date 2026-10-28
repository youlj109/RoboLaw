"""One reviewed, fixed-layout grasp candidate per failed scene.

This module changes targets only. It does not run a solver, move a source prim,
change joint starts, mask colliders, or modify payload/gripper collision models.
The caller runs the existing six phases once and stops on the first failure.
"""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RECIPES = ROOT / 'configs/extensions/replicator/trajectory_retry_v1.json'


def load_retry_recipes(path=DEFAULT_RECIPES):
    document = json.loads(Path(path).read_text())
    if document.get('schema_version') != 1 or not (document.get('layout_fixed') is True or
            document.get('layout_fixed') is False and document.get('scene_settings_changed_once') is True):
        raise ValueError('Expected version 1 fixed-layout retry recipes')
    return document['recipes']


def _unit_xy(vector):
    value = np.asarray(vector, float).copy()
    value[2] = 0.
    if np.linalg.norm(value) < 1e-8:
        raise ValueError('Degenerate horizontal candidate direction')
    return value / np.linalg.norm(value)


def _section_segments(geometry, normal, offset):
    """Intersect the retained source triangles with one requested plane."""
    triangles = geometry.vertices[geometry.triangles]
    segments = []
    for triangle in triangles:
        values = triangle @ normal - offset
        points = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if (values[i] <= 0 < values[j]) or (values[j] <= 0 < values[i]):
                points.append(triangle[i] + (triangle[j]-triangle[i]) *
                              (-values[i]/(values[j]-values[i])))
        if len(points) == 2:
            segments.append(points)
    if not segments:
        raise ValueError('Selected grasp plane misses the source collision mesh')
    return np.asarray(segments)


def measured_banana_grasp(geometry, yaw_deg, section_offset_m):
    angle = np.deg2rad(yaw_deg)
    jaw = np.array([np.cos(angle), np.sin(angle), 0.])
    normal = np.array([-np.sin(angle), np.cos(angle), 0.])
    points = _section_segments(geometry, normal, section_offset_m).reshape(-1, 3)
    coordinates = points @ jaw
    width = float(np.ptp(coordinates))
    if not .01 < width < .085 or abs(section_offset_m) > .02:
        raise ValueError('Selected banana midbody section is outside the original gripper width/band')
    center = (coordinates.min()+coordinates.max())/2*jaw + section_offset_m*normal
    center[2] = (points[:, 2].min()+points[:, 2].max())/2
    return center, jaw, dict(rule='Measured source midbody plane; one fixed candidate',
        center_local_m=center.tolist(), section_width_m=width,
        section_offset_m=section_offset_m, jaw_yaw_local_deg=yaw_deg,
        section_plane_normal_local=normal.tolist())


def measured_bowl_rim_grasp(geometry, azimuth_deg):
    """Measure actual inner/outer rim intersections at the chosen azimuth."""
    angle = np.deg2rad(azimuth_deg)
    radial = np.array([np.cos(angle), np.sin(angle), 0.])
    normal = np.array([-np.sin(angle), np.cos(angle), 0.])
    segments = _section_segments(geometry, normal, 0.)
    z = float(geometry.high[2]-.002)
    points = []
    for a, b in segments:
        if (a[2] <= z < b[2]) or (b[2] <= z < a[2]):
            point = a+(b-a)*((z-a[2])/(b[2]-a[2]))
            if point @ radial > 0:
                points.append(point)
    if len(points) < 2:
        raise ValueError('Selected bowl rim does not expose inner and outer source surfaces')
    distances = np.unique(np.round(np.asarray(points) @ radial, 9))
    if len(distances) != 2 or not .001 < np.ptp(distances) < .01:
        raise ValueError('Selected bowl rim is not a measured narrow wall pinch')
    center = radial*distances.mean(); center[2] = z
    return center, radial, dict(rule='Measured upper bowl wall; radial pinch, never diameter grip',
        center_local_m=center.tolist(), wall_thickness_m=float(np.ptp(distances)),
        rim_azimuth_local_deg=azimuth_deg, slice_z_local_m=z,
        inner_outer_radius_m=distances.tolist())


def measured_mustard_grasp(geometry, yaw_deg, z_local_m):
    if not geometry.low[2]+.02 <= z_local_m <= geometry.high[2]-.025:
        raise ValueError('Selected mustard pinch height is outside the bottle body guard')
    points = _section_segments(geometry, np.array([0., 0., 1.]), z_local_m).reshape(-1, 3)
    angle = np.deg2rad(yaw_deg)
    jaw = np.array([np.cos(angle), np.sin(angle), 0.])
    width = float(np.ptp(points @ jaw))
    if not .01 < width < .085:
        raise ValueError('Selected mustard body section does not fit the original gripper')
    center = (points.min(0)+points.max(0))/2; center[2] = z_local_m
    return center, jaw, dict(rule='Measured bottle-body horizontal section; one fixed yaw',
        center_local_m=center.tolist(), section_width_m=width, jaw_yaw_local_deg=yaw_deg)


def actual_sink_rim(world):
    rows = [row for row in world.objects if '/Sink/' in row[0]
            and row[0].rsplit('/', 1)[-1].startswith('Rim')]
    if not rows:
        raise ValueError('No original sink Rim collider; cannot substitute faucet height')
    return max(float(row[3][2]) for row in rows), [row[0] for row in rows]


def apply_retry_targets(plan, layout, world, assignment, recipe):
    """Return a new target plan; all original scene/model inputs stay untouched."""
    if recipe.get('layout_fixed') is not True:
        raise ValueError('Retry must explicitly retain the fixed layout')
    if layout.get('task_id') != recipe['task_id']:
        raise ValueError('Retry task does not match the current scene')
    if recipe['payload_name'] != plan['payload_name']:
        raise ValueError('Retry payload does not match the original task')
    initial = np.asarray(plan['initial_payload_pose'], float)
    old_targets = plan['targets']
    destination = np.asarray(old_targets['lower']) @ np.asarray(plan['attachment'])
    geometry = plan['geometry']
    payload = plan['payload_name']
    grasp_recipe = recipe['grasp']
    if payload == 'banana':
        center, local_jaw, info = measured_banana_grasp(geometry,
            grasp_recipe['jaw_yaw_local_deg'], grasp_recipe['section_offset_m'])
    elif payload == 'bowl':
        center, local_jaw, info = measured_bowl_rim_grasp(geometry,
            grasp_recipe['rim_azimuth_local_deg'])
    else:
        center, local_jaw, info = measured_mustard_grasp(geometry,
            grasp_recipe['jaw_yaw_local_deg'], grasp_recipe['z_local_m'])
    grasp = np.eye(4)
    down = np.array([0., 0., -1.])
    jaw = _unit_xy(initial[:3, :3] @ local_jaw)
    grasp[:3, :3] = np.column_stack((down, jaw, np.cross(down, jaw)))
    grasp[:3, 3] = initial[:3, :3] @ center + initial[:3, 3]
    grasp[2, 3] = max(grasp[2, 3], initial[2, 3]+float(geometry.low[2])+.035)
    grasp[2, 3] += float(grasp_recipe.get('tcp_raise_m', 0.))
    attachment = np.linalg.inv(grasp) @ initial
    lower = destination @ np.linalg.inv(attachment)
    pregrasp = grasp.copy()
    pregrasp[:3, 3] += np.asarray(recipe['pregrasp_offset_world_m'])
    lift = grasp.copy()
    transfer = lower.copy()
    lift[:2, 3] += np.asarray(recipe['lift_offset_world_xy_m'])
    if payload == 'bowl':
        rim, rim_paths = actual_sink_rim(world)
        relative = geometry.vertices @ attachment[:3, :3].T + attachment[:3, 3]
        # The carried bowl's lowest point, rather than its TCP, must clear the
        # actual basin rim. Faucet geometry remains in the unchanged world.
        lowest_relative_z = float((relative @ grasp[:3, :3].T)[:, 2].min())
        carry_z = max(grasp[2, 3]+recipe['lift_height_m'],
                      rim+.04-lowest_relative_z, lower[2, 3]+.08)
        info.update(actual_sink_rim_z_m=rim, rim_source_colliders=rim_paths,
                    carried_bottom_relative_tcp_z_m=lowest_relative_z)
    else:
        carry_z = max(float(old_targets['transfer'][2, 3])+
                      float(recipe['carry_height_delta_m']), grasp[2, 3]+.12,
                      lower[2, 3]+.12)
    lift[2, 3] = carry_z
    transfer[2, 3] = carry_z
    retreat = lower.copy()
    retreat[:3, 3] += np.asarray(recipe['retreat_offset_world_m'])
    if not np.allclose(lower @ attachment, destination, atol=1e-10, rtol=0):
        raise ValueError('Retry changed the original final payload pose')
    updated = dict(plan)
    updated['attachment'] = attachment
    updated['targets'] = dict(zip(('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat'),
                                  (pregrasp, grasp, lift, transfer, lower, retreat)))
    updated['grasp_info'] = {**info, 'source_sha256':geometry.provenance['source_sha256'],
        'nominal_only':True, 'physical_contacts_verified':False,
        'target_tcp_world_m':grasp[:3, 3].tolist()}
    updated['retry'] = dict(schema_version=1, recipe_id=recipe['scene_id'],
        layout_fixed=True, one_candidate_only=True, provenance=deepcopy(recipe['provenance']),
        selected_parameters=deepcopy({key:value for key,value in recipe.items()
            if key not in ('provenance', 'review_note')}),
        original_final_payload_pose=destination.tolist(),
        final_payload_pose_preserved=True, collision_models_and_masks_unchanged=True,
        initial_state_collision_warning=bool(recipe.get('initial_state_collision_warning')),
        scope='Target changes only; no execution, grasp or avoidance-success claim.')
    return updated
