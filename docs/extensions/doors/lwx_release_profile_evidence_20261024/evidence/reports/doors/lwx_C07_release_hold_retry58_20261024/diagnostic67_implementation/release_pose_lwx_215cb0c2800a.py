"""Offline release target geometry; this module never writes simulation state."""
import numpy as np
from scipy.spatial.transform import Rotation


def release_pose(start, start_angle, measured_angle, hinge, axis, normal,
                 retreat_m, tilt_deg, fraction, follow_door=False):
    """Retract from a fixed pose, optionally following measured hinge rotation.

    Following rotates the measured initial robot pose rigidly around the source
    hinge before adding the outward/downward displacement. It does not change
    the door state or authorize any contact. Caller must check IK, world and
    self collision and actual physical execution independently.
    """
    if not (0. <= retreat_m <= .04 and -60. <= tilt_deg <= 60. and 0. <= fraction <= 1.):
        raise ValueError('Release parameters outside original bounds')
    target = np.array(start, dtype=float, copy=True)
    normal = np.array(normal, dtype=float, copy=True)
    if follow_door:
        axis = np.asarray(axis, dtype=float)
        norm = np.linalg.norm(axis)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError('Invalid hinge axis')
        delta = Rotation.from_rotvec(axis / norm * (measured_angle - start_angle)).as_matrix()
        hinge = np.asarray(hinge, dtype=float)
        target[:3, :3] = delta @ target[:3, :3]
        target[:3, 3] = hinge + delta @ (target[:3, 3] - hinge)
        normal = delta @ normal
    direction = -normal * np.cos(np.deg2rad(tilt_deg)) + np.array([0., 0., np.sin(np.deg2rad(tilt_deg))])
    if not np.all(np.isfinite(target)) or not np.all(np.isfinite(direction)) or np.linalg.norm(direction) < 1e-12:
        raise ValueError('Invalid release geometry')
    target[:3, 3] += direction / np.linalg.norm(direction) * retreat_m * fraction
    return target
