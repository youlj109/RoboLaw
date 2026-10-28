"""Unexecuted source-bound transport candidate: retain real lift-end height.

This pure leaf computes a goal from current measured transforms. It never
modifies the body, attachment, source scene, collision permissions or planner.
The complete original MotionGen and source FCL path gates remain mandatory.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
TASK = 'kitchen_bench_replicator_g_shape_banana_bowl'
CANDIDATE = ROOT/'configs/extensions/replicator/g_shape_banana_bowl_transport_height_candidate.json'
BASE_PROFILE = ROOT/'configs/extensions/replicator/g_shape_banana_bowl_controller.json'


def validate_rule(candidate):
    expected = dict(schema_version=1, task_id=TASK, candidate_id='retain_actual_lift_end_height',
        payload_root_z_rule='current_observed_payload_root_z',
        destination_xy_rule='current_original_bowl_root_xy',
        payload_orientation_rule='current_observed_payload_orientation',
        attachment_rule='unchanged_original_close_planning_attachment',
        additional_height_m=0., source_clearance_m=.002,
        full_original_world_and_self_and_source_checks_required=True,
        baseline_and_variants_share_candidate=True)
    if any(candidate.get(key) != value for key, value in expected.items()):
        raise ValueError('Unreviewed banana transport-height rule')
    if set(candidate.get('source_object_sha256', {})) != {'banana', 'bowl'}:
        raise ValueError('Original banana/bowl source identity missing')


def read_candidate(path=CANDIDATE, base_profile_path=BASE_PROFILE):
    path, base_profile_path = Path(path), Path(base_profile_path)
    candidate, base = json.loads(path.read_text()), json.loads(base_profile_path.read_text())
    validate_rule(candidate)
    if (candidate['base_profile_sha256'] != hashlib.sha256(base_profile_path.read_bytes()).hexdigest()
            or base['task_id'] != TASK or candidate['source_object_sha256'] != base['source_object_sha256']
            or candidate['source_layout_sha256'] != base['source_layout_sha256']
            or candidate['source_kitchen_sha256'] != base['source_kitchen_sha256']):
        raise ValueError('Prepared original controller/source changed')
    return candidate


def rigid_transform(value, name):
    value = np.asarray(value, dtype=float)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.array_equal(value[3], [0., 0., 0., 1.])
            or not np.allclose(value[:3,:3].T@value[:3,:3], np.eye(3), atol=1e-8, rtol=0.)
            or not np.isclose(np.linalg.det(value[:3,:3]), 1., atol=1e-8, rtol=0.)):
        raise ValueError('Invalid current rigid transform: '+name)
    return value


def transport_target(actual_payload, actual_bowl, original_close_attachment, *, candidate, source_object_sha256):
    """Use current source poses, with no archived world goal or new attachment.

    Call only after the collector's original real lift/contact checks. This
    function supplies a target, not proof that an object is held or a path safe.
    """
    validate_rule(candidate)
    if source_object_sha256 != candidate['source_object_sha256']:
        raise ValueError('Runtime original task objects differ from prepared candidate')
    payload = rigid_transform(actual_payload, 'payload')
    bowl = rigid_transform(actual_bowl, 'bowl')
    attachment = rigid_transform(original_close_attachment, 'original close attachment')
    desired = payload.copy()
    desired[:2, 3] = bowl[:2, 3]
    target = desired@np.linalg.inv(attachment)
    info = dict(candidate_id=candidate['candidate_id'],
        actual_payload_pose=payload.tolist(), actual_original_bowl_pose=bowl.tolist(),
        desired_payload_pose=desired.tolist(), target_tcp=target.tolist(),
        original_close_planning_attachment=attachment.tolist(),
        retained_payload_root_z_m=float(payload[2, 3]),
        target_root_z_difference_m=0., actual_payload_orientation_preserved=True,
        attachment_rebound=False, source_state_written=False,
        full_original_world_and_self_and_source_checks_required=True,
        physical_transport_verified=False, task_success=False,
        scope='Current real lift-end payload Z/orientation and original bowl XY; original close attachment unchanged. Source-bound finite transport candidate only. All original complete-path and actual-execution safety checks still required.')
    return target, info
