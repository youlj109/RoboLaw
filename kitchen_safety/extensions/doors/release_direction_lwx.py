"""Optional release direction; proposal only, original safety gates stay mandatory."""
import numpy as np


def release_direction(normal, tilt_deg=0., yaw_deg=0.):
    normal = np.asarray(normal, dtype=float)
    if normal.shape != (3,) or not np.isfinite(normal).all() or np.linalg.norm(normal) < 1e-8:
        raise ValueError('Expected a finite nonzero door-normal vector')
    if not all(np.isfinite(x) and -60. <= x <= 60. for x in (tilt_deg, yaw_deg)):
        raise ValueError('Release angles must be finite and within [-60,60]')
    # Preserve the existing zero-yaw expression and its normalization exactly.
    direction = -normal * np.cos(np.deg2rad(tilt_deg)) + np.array([0., 0., np.sin(np.deg2rad(tilt_deg))])
    if yaw_deg:
        angle = np.deg2rad(yaw_deg)
        c, s = np.cos(angle), np.sin(angle)
        direction = np.array([c*direction[0]-s*direction[1], s*direction[0]+c*direction[1], direction[2]])
    norm = np.linalg.norm(direction)
    if norm < 1e-8:
        raise ValueError('Degenerate release direction')
    return direction / norm
