"""Frozen-pose mustard grasp adjustments; CPU arithmetic only.

One selected recipe goes to one cuRobo planning pass and one physical execution.
The source layout and historical final payload pose are never changed here.
"""
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation

PHASES = ('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat')


def checked_pose(value):
    p = np.asarray(value, dtype=float)
    if (p.shape != (4, 4) or not np.isfinite(p).all()
            or not np.allclose(p[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(p[:3, :3].T @ p[:3, :3], np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(p[:3, :3]), 1., atol=1e-7)):
        raise ValueError('Finite rigid homogeneous pose required')
    return p.copy()


def candidates(reference):
    """Small declared CPU-only set, ordered by measured reference similarity.

    Rotating around source bottle Z selects an approach side without moving it.
    The measured reference came from a successful original G episode; transfer
    to these augmented scenes is explicitly unverified.
    """
    source = next(x for x in reference['ordered_transfer_candidates']
                  if x['name'] == 'G_actual_approach_end')
    base = checked_pose(source['bottle_to_tcp_local'])
    result = []
    for yaw in (0., 180., 90., 270.):
        rz = np.eye(4)
        rz[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        for dz in (0., .010):
            p = rz @ base
            p[2, 3] += dz
            result.append(dict(name=f'reference_side{yaw:g}_z{dz:g}',
                bottle_to_tcp_local=p.tolist(), source_reference_name=source['name'],
                rotation_about_bottle_z_deg=yaw, body_local_tcp_raise_m=dz,
                planning_gripper_joint_rad=.37, execution_gripper_joint_rad=.70))
    return result


def make_targets(previous, candidate, *, transport_raise_m=0.):
    """Rebuild six TCP targets from the retained old payload endpoints."""
    if previous['payload_name'] != 'mustard_bottle':
        raise ValueError('Mustard-only adapter')
    old = {p: checked_pose(previous['targets'][p]) for p in PHASES}
    old_a = checked_pose(previous['nominal_attachment'])
    initial = old['approach'] @ old_a
    local = checked_pose(candidate['bottle_to_tcp_local'])
    if not -.059 <= local[2, 3] <= .0199:
        raise ValueError('TCP must remain in the source inset broad body band')
    approach = initial @ local
    a = np.linalg.inv(approach) @ initial
    inv_a = np.linalg.inv(a)
    targets = {p: old[p] @ old_a @ inv_a for p in PHASES}
    targets['approach'] = approach
    targets['pregrasp'] = approach.copy()
    targets['pregrasp'][:3, 3] += old['pregrasp'][:3, 3] - old['approach'][:3, 3]
    # Preserve the previous carry payload poses and zero lateral lift motion.
    targets['lift'][:2, 3] = approach[:2, 3]
    rise = float(transport_raise_m)
    if not 0. <= rise <= .08:
        raise ValueError('Transport rise must be within [0, .08] metres')
    for p in ('lift', 'transfer'):
        targets[p][2, 3] += rise
    destination = old['lower'] @ old_a
    if not np.allclose(targets['lower'] @ a, destination, atol=1e-8, rtol=0):
        raise ValueError('Historical destination payload pose changed')
    for p in PHASES:
        checked_pose(targets[p])
    info = deepcopy(previous.get('grasp_info', {}))
    info.update(round2_rule='measured_G_broad_body_grasp_with_new_closed_position_target',
        selected_candidate=deepcopy(candidate), body_local_tcp_pose=local.tolist(),
        old_body_local_tcp_pose=(np.linalg.inv(initial) @ old['approach']).tolist(),
        transport_raise_m=rise, physical_contacts_verified=False,
        closure_semantics='Execution target .70 rad; .37 rad is only the anticipated bottle-blocked planning state, not a measured result of this round.')
    return dict(target_poses_world={p: targets[p].tolist() for p in PHASES},
        nominal_attachment=a.tolist(), nominal_gripper_joint_rad=float(candidate['execution_gripper_joint_rad']),
        planning_gripper_joint_rad=float(candidate['planning_gripper_joint_rad']),
        initial_payload_pose_world=initial.tolist(), final_payload_pose_world=destination.tolist(),
        hold_orientation_phases=['approach', 'lift', 'transfer', 'lower'], grasp_info=info)
