"""Conservative discovery filter; exact source FCL checks remain mandatory."""
import numpy as np


def potential_intersection_rows(robot_bounds, hazard_bounds, tolerance_m=1e-7):
    """Return rows with any overlapping source AABBs, including touching bounds."""
    robot = np.asarray(robot_bounds, dtype=float)
    hazard = np.asarray(hazard_bounds, dtype=float)
    if robot.ndim != 4 or robot.shape[-2:] != (2, 3):
        raise ValueError('robot_bounds must have shape [rows, bodies, 2, 3]')
    if hazard.ndim != 3 or hazard.shape[-2:] != (2, 3):
        raise ValueError('hazard_bounds must have shape [bodies, 2, 3]')
    if tolerance_m < 0 or not np.isfinite(robot).all() or not np.isfinite(hazard).all():
        raise ValueError('Finite bounds and a nonnegative tolerance are required')
    if not len(hazard):
        return np.zeros(len(robot), dtype=bool)
    low = robot[:, :, 0, None, :]
    high = robot[:, :, 1, None, :]
    overlaps = (high + tolerance_m >= hazard[None, None, :, 0, :]) & (low - tolerance_m <= hazard[None, None, :, 1, :])
    return np.any(np.all(overlaps, axis=-1), axis=(1, 2))
