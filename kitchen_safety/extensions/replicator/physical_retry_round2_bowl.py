"""One selected bowl-grasp target recipe for the second physical retry.

Pure CPU pose arithmetic.  This does not edit layouts, collision masks, success
predicates, drive gains, or execute physics.  Source-contact/IK screening remains
an imperfect planning estimate, never a grasp/avoidance certification.
"""
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation

PHASES = ('pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat')
# Ordered, finite CPU-only candidate declaration. The parent freezes exactly one.
CANDIDATES = tuple(dict(jaw_axis_tilt_deg=tilt, tcp_raise_m=raise_m)
                   for tilt, raise_m in ((25., .005039543828685633),
                       (-25., .005039543828685633), (15., .003), (-15., .003),
                       (25., .003), (-25., .003), (25., .008), (-25., .008)))


def pose_checked(value):
    p=np.asarray(value,dtype=float)
    if p.shape!=(4,4) or not np.isfinite(p).all() or not np.allclose(p[3],[0,0,0,1],atol=1e-8):
        raise ValueError('Finite homogeneous transform required')
    if not np.allclose(p[:3,:3].T@p[:3,:3],np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(p[:3,:3]),1,atol=1e-7):
        raise ValueError('Rigid rotation required')
    return p.copy()


def make_targets(previous, candidate, *, rim_azimuth_delta_deg=0., reverse_jaw=False,
                 destination_translation_world_m=(0.,0.,0.)):
    """Modify current historical goals, preserving the original task geometry.

    Recomputes the attachment and all carry goals from the retained payload
    poses.  The initial lift endpoint has no XY displacement from the grasp;
    a full Cartesian path still needs the parent's actual planner.
    """
    if previous['payload_name']!='bowl':raise ValueError('Bowl-only adapter')
    old={p:pose_checked(previous['targets'][p]) for p in PHASES}
    a0=pose_checked(previous['nominal_attachment']);initial=old['approach']@a0
    grasp=old['approach'].copy()
    az=np.deg2rad(float(rim_azimuth_delta_deg))
    if az:
        r=initial[:3,:3]@Rotation.from_euler('z',az).as_matrix()@initial[:3,:3].T
        grasp[:3,3]=initial[:3,3]+r@(grasp[:3,3]-initial[:3,3]);grasp[:3,:3]=r@grasp[:3,:3]
    if reverse_jaw:grasp[:3,:3]=grasp[:3,:3]@Rotation.from_euler('x',180,degrees=True).as_matrix()
    grasp[:3,:3]=grasp[:3,:3]@Rotation.from_euler('y',float(candidate['jaw_axis_tilt_deg']),degrees=True).as_matrix()
    grasp[2,3]+=float(candidate['tcp_raise_m'])
    a=np.linalg.inv(grasp)@initial
    goal_delta=np.asarray(destination_translation_world_m,dtype=float)
    if goal_delta.shape!=(3,) or not np.isfinite(goal_delta).all() or np.linalg.norm(goal_delta)>.10:
        raise ValueError('Bounded same-basin destination adjustment required')
    goals={p:old[p]@a0@np.linalg.inv(a) for p in PHASES}
    goals['approach']=grasp
    # Retain previous pregrasp approach vector and retreat clearance.
    goals['pregrasp']=grasp.copy();goals['pregrasp'][:3,3]+=old['pregrasp'][:3,3]-old['approach'][:3,3]
    goals['lift'][:2,3]=grasp[:2,3]
    for p in ('lower','transfer','retreat'):goals[p][:3,3]+=goal_delta
    # Transfer and lift retain the old carried-object heights; no extra high
    # swing over a faucet is inserted without a full-world planner check.
    for p in PHASES:pose_checked(goals[p])
    destination=old['lower']@a0;destination[:3,3]+=goal_delta
    if not np.allclose(goals['lower']@a,destination,atol=1e-8):raise ValueError('Bowl destination drift')
    info=deepcopy(previous.get('grasp_info',{}))
    info.update(round2_jaw_axis_tilt_deg=float(candidate['jaw_axis_tilt_deg']),
        round2_tcp_raise_m=float(candidate['tcp_raise_m']),
        round2_rim_azimuth_delta_deg=float(rim_azimuth_delta_deg),round2_reverse_jaw=bool(reverse_jaw),
        target_tcp_world_m=grasp[:3,3].tolist(),nominal_only=True,physical_contacts_verified=False)
    return dict(target_poses_world={p:goals[p].tolist() for p in PHASES},
        nominal_attachment=a.tolist(),initial_payload_pose_world=initial.tolist(),
        final_payload_pose_world=destination.tolist(),grasp_info=info,
        nominal_gripper_joint_rad=float(previous['nominal_gripper_joint_rad']))
