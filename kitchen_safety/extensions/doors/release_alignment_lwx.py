"""Optional open-finger local-roll alignment before withdrawal; no scene changes."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

def release_roll_pose(start_pose, delta_deg, fraction):
    start = np.asarray(start_pose, dtype=float)
    if start.shape != (4,4) or not np.isfinite(start).all():
        raise ValueError('Expected finite 4x4 starting TCP pose')
    if not math.isfinite(delta_deg) or not -60. <= delta_deg <= 60.:
        raise ValueError('Release roll must be finite within [-60,60]')
    if not math.isfinite(fraction) or not 0. <= fraction <= 1.:
        raise ValueError('Alignment fraction must be finite within [0,1]')
    result = start.copy()
    result[:3,:3] = start[:3,:3] @ Rotation.from_euler('x', delta_deg*fraction, degrees=True).as_matrix()
    return result
