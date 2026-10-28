"""Apply one CPU-reviewed target sequence while preserving the fixed task.

This adapter changes planning goals, never authored poses or collision geometry.
All source and final object transforms are checked before any cuRobo solve.
"""
from copy import deepcopy
import numpy as np

PHASES = ('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat')


def checked_pose(value):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError('Invalid retry target matrix')
    if (not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8, rtol=0) or
            not np.allclose(pose[:3,:3].T @ pose[:3,:3], np.eye(3), atol=1e-7, rtol=0) or
            not np.isclose(np.linalg.det(pose[:3,:3]), 1., atol=1e-7, rtol=0)):
        raise ValueError('Retry target is not a rigid pose')
    return pose.copy()


def apply_retry_targets(plan, layout, world, assignment, recipe):
    adjusted = recipe.get('scene_adjustment') is not None
    if (not (recipe.get('layout_fixed') is True or adjusted and recipe.get('layout_fixed') is False)
            or recipe['task_id'] != layout['task_id']):
        raise ValueError('Retry recipe must match the fixed current layout task')
    if recipe['payload_name'] != plan['payload_name']:
        raise ValueError('Retry payload mismatch')
    raw = recipe['target_poses_world']
    if set(raw) != set(PHASES):
        raise ValueError('Expected exactly the six original task phases')
    targets = {phase: checked_pose(raw[phase]) for phase in PHASES}
    initial = plan['initial_payload_pose']
    attachment = np.linalg.inv(targets['approach']) @ initial
    destination = plan['targets']['lower'] @ plan['attachment']
    original_destination = destination.copy()
    dz = float(recipe.get('goal_vertical_clearance_m', 0.))
    if dz:
        if (plan['payload_name'] != 'bowl' or not 0. < dz <= .03 or
                not recipe.get('goal_correction_evidence')):
            raise ValueError('Only audited bowl release-height correction up to 30 mm is supported')
        destination = destination.copy()
        destination[2,3] += dz
        rim_rows = [row for row in world.objects if '/Sink/' in row[0]
                    and row[0].rsplit('/',1)[-1].startswith('Rim')]
        if not rim_rows:
            raise ValueError('Missing source sink rim for goal correction')
        top = (plan['geometry'].vertices @ destination[:3,:3].T + destination[:3,3])[:,2].max()
        if top >= max(row[3][2] for row in rim_rows) - .02:
            raise ValueError('Corrected bowl goal must remain inside the same basin below the rim')
    if not np.allclose(targets['lower'] @ attachment, destination, atol=1e-7, rtol=0):
        raise ValueError('Retry would change the original final payload pose')
    if recipe.get('nominal_attachment') is not None and not np.allclose(
            attachment, checked_pose(recipe['nominal_attachment']), atol=1e-7, rtol=0):
        raise ValueError('Selected grasp attachment is inconsistent with initial object pose')
    updated = dict(plan)
    supports = recipe.get('source_contact_supports')
    if supports is not None:
        if plan['payload_name'] != 'banana' or not isinstance(supports, list) or not 1 <= len(supports) <= 2:
            raise ValueError('Only the measured banana compound support is allowed')
        points = plan['geometry'].vertices @ initial[:3,:3].T + initial[:3,3]
        low, high = points.min(0), points.max(0)
        objects = {row[0]:row for row in world.objects}
        if plan['source_support'] not in supports or not recipe.get('source_contact_evidence'):
            raise ValueError('Compound support must retain the original support and evidence')
        for name in supports:
            row = objects.get(name)
            if (row is None or not name.startswith('/World/kitchen/') or 'CornerCounter_' not in name or
                    not 0. <= low[2] - row[3][2] <= .005 or
                    not np.all(np.minimum(high[:2],row[3][:2]) > np.maximum(low[:2],row[2][:2]))):
                raise ValueError('Requested source contact is not an adjacent original support under the banana')
        updated['source_supports'] = list(supports)
    updated.update(targets=targets, attachment=attachment,
        grasp_info={**deepcopy(recipe.get('grasp_info', recipe.get('grasp', {}))),
                    'source_sha256':plan['geometry'].provenance['source_sha256'],
                    'nominal_only':True, 'physical_contacts_verified':False,
                    'target_tcp_world_m':targets['approach'][:3,3].tolist()})
    updated['retry'] = dict(schema_version=2, recipe_id=recipe['scene_id'],
        layout_fixed=not adjusted, new_layout_frozen=True, scene_settings_changed_once=adjusted,
        scene_adjustment=deepcopy(recipe.get('scene_adjustment')),
        one_candidate_only=True, provenance=deepcopy(recipe.get('provenance', {})),
        selected_parameters=deepcopy({key:value for key,value in recipe.items()
            if key not in ('provenance','target_poses_world','review_note')}),
        original_final_payload_pose=original_destination.tolist(), final_payload_pose=destination.tolist(),
        final_payload_pose_preserved=not bool(dz), original_task_destination_preserved=True,
        goal_vertical_clearance_m=dz,
        collision_geometry_unchanged=not adjusted, collision_shape_assets_unchanged=True,
        collision_models_and_masks_unchanged=supports is None,
        source_contact_supports=deepcopy(supports), added_hazard_masks=False,
        scope=('One hazard-pose adjustment, frozen before one planning pass; no execution or avoidance certification.'
               if adjusted else 'One selected fixed-layout target sequence; no execution or avoidance certification.'))
    return updated
