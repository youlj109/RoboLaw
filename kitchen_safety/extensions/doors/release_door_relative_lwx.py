"""Rigid door-relative release feedback; caller retains all safety gates."""
import numpy as np
from scipy.spatial.transform import Rotation

def follow_measured_door_pose(reference_tcp, hinge, axis, delta_joint_rad):
    T=np.asarray(reference_tcp,float);h=np.asarray(hinge,float);a=np.asarray(axis,float)
    if T.shape!=(4,4) or h.shape!=(3,) or a.shape!=(3,) or not np.isfinite(T).all() or not np.isfinite(h).all() or not np.isfinite(a).all() or not np.isfinite(delta_joint_rad):
        raise ValueError('Expected finite rigid pose, hinge, axis and measured angle')
    if not np.isclose(np.linalg.norm(a),1.,atol=1e-6):raise ValueError('Door axis must be normalized')
    R=Rotation.from_rotvec(a*float(delta_joint_rad)).as_matrix()
    result=T.copy();result[:3,:3]=R@T[:3,:3];result[:3,3]=h+R@(T[:3,3]-h)
    return result
