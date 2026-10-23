"""Camera-calibrated explanatory projection, never a 2-D safety metric."""
import numpy as np
from scipy.spatial.transform import Rotation


def camera_points(points, pose):
    points, pose = np.asarray(points, dtype=float), np.asarray(pose, dtype=float)
    if pose.shape != (7,) or not np.isfinite(pose).all() or not np.isfinite(points).all():
        raise ValueError('Nonfinite camera pose or world points')
    if abs(np.linalg.norm(pose[3:]) - 1.) > 1e-4:
        raise ValueError('Camera ROS quaternion must be normalized xyzw')
    return (points - pose[:3]) @ Rotation.from_quat(pose[3:]).as_matrix()


def clip_polygon(points, axis, limit, keep_greater):
    """Sutherland-Hodgman clipping in the original coordinate space."""
    points = np.asarray(points, dtype=float)
    if not len(points):
        return points
    result = []
    for previous, current in zip(np.roll(points, 1, axis=0), points):
        a, b = previous[axis] - limit, current[axis] - limit
        inside_a, inside_b = (a >= 0, b >= 0) if keep_greater else (a <= 0, b <= 0)
        if inside_a != inside_b:
            result.append(previous + (current - previous) * (-a / (b - a)))
        if inside_b:
            result.append(current)
    return np.asarray(result).reshape(-1, points.shape[1])


def project_polygon(camera_polygon, intrinsics, size, near=.01):
    """Clip near plane BEFORE perspective division, then clip screen bounds."""
    points = clip_polygon(camera_polygon, 2, near, True)
    if len(points) < 3:
        return np.empty((0, 2))
    projected = points @ np.asarray(intrinsics).T
    projected = projected[:, :2] / projected[:, 2:3]
    for axis, limit, greater in ((0, 0, True), (1, 0, True),
                                  (0, size[0] - 1, False), (1, size[1] - 1, False)):
        projected = clip_polygon(projected, axis, limit, greater)
        if len(projected) < 3:
            return np.empty((0, 2))
    return projected


def project_points(points, pose, intrinsics, size, near=.01):
    values = camera_points(points, pose)
    valid = values[:, 2] >= near
    pixels = np.zeros((len(values), 2))
    p = values[valid] @ np.asarray(intrinsics).T
    pixels[valid] = p[:, :2] / p[:, 2:3]
    valid &= (pixels[:, 0] >= 0) & (pixels[:, 0] < size[0]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < size[1])
    return pixels, valid
