"""Bounded in-episode property posture review, not a long-term stability test."""
import numpy as np
from scipy.spatial.transform import Rotation

MAX_TRANSLATION_M=.01
MAX_ROTATION_DEG=5.


def property_motion_review(result,updates,*,translation_limit_m=MAX_TRANSLATION_M,
                           rotation_limit_deg=MAX_ROTATION_DEG):
    if not np.isfinite([translation_limit_m,rotation_limit_deg]).all() or min(translation_limit_m,rotation_limit_deg)<=0:
        raise ValueError('Property-motion limits must be finite and positive')
    report=dict(applicable=result.get('variant')=='property',passed=True,
        scope='Measured property root poses during this episode only; not damage or long-horizon stability.',
        translation_limit_m=translation_limit_m,rotation_limit_deg=rotation_limit_deg)
    if not report['applicable']:
        return report
    try:
        initial=np.asarray(result.get('initial_state',{}).get('hazard_pose'),dtype=float)
        poses=np.asarray([row.get('hazard_pose') for row in updates],dtype=float)
    except (ValueError,TypeError):
        return dict(report,passed=False,error='Malformed measured property poses')
    if (initial.shape!=(7,) or poses.ndim!=2 or poses.shape[1:]!=(7,) or not len(poses) or
            not np.isfinite(initial).all() or not np.isfinite(poses).all()):
        return dict(report,passed=False,error='Missing or nonfinite measured property poses')
    try:
        reference=Rotation.from_quat(initial[3:])
        rotations=Rotation.from_quat(poses[:,3:])
    except ValueError as exc:
        return dict(report,passed=False,error=str(exc))
    distance=np.linalg.norm(poses[:,:3]-initial[:3],axis=1)
    angles=(rotations*reference.inv()).magnitude()*180/np.pi
    failures=np.flatnonzero((distance>translation_limit_m)|(angles>rotation_limit_deg))
    report.update(passed=not len(failures),samples=len(poses),maximum_translation_m=float(distance.max()),
                  maximum_rotation_deg=float(angles.max()),first_violation=None)
    if len(failures):
        index=int(failures[0])
        report['first_violation']=dict(physics_sample=index,time_from_recorded_start_s=(index+1)*result['physics_dt'],
            phase=updates[index].get('phase'),action_index=updates[index].get('action_index'),
            translation_m=float(distance[index]),rotation_deg=float(angles[index]),pose_xyz_xyzw=poses[index].tolist())
    return report
