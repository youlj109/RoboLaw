"""Bottle-local grasp targets derived from the source mesh, without starting Kit.

The broad cross-section band excludes the narrow shoulder/nozzle. Its inset is
a geometric contact region, not a prediction of PhysX contact or grasp success.
The caller must still measure both finger contacts and replan from the measured
attachment. Changing the grasp target does not change any transport height.
"""
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOTTLE_USD = ROOT / 'assets/robolab/objects/hope/mustard_bottle.usd'


def _source_triangles(usd_path):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None or not stage.GetDefaultPrim():
        raise ValueError(f'Cannot open bottle mesh with a default prim: {usd_path}')
    cache = UsdGeom.XformCache()
    inverse_root = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(stage.GetDefaultPrim())).T)
    vertices, triangles = [], []
    for prim in Usd.PrimRange(stage.GetDefaultPrim(), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        transform = inverse_root @ np.asarray(cache.GetLocalToWorldTransform(prim)).T
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        points = points @ transform[:3, :3].T + transform[:3, 3]
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
        cursor = 0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            for index in range(1, count - 1):
                triangles.append(points[indices[[cursor, cursor + index, cursor + index + 1]]])
            cursor += count
        vertices.append(points)
    if not vertices or not triangles:
        raise ValueError(f'No triangulatable bottle mesh: {usd_path}')
    return np.concatenate(vertices), np.asarray(triangles)


def _section(triangles, z):
    points = []
    for first, second in ((0, 1), (1, 2), (2, 0)):
        a, b = triangles[:, first], triangles[:, second]
        crossing = ((a[:, 2] <= z) & (b[:, 2] > z)) | ((b[:, 2] <= z) & (a[:, 2] > z))
        a, b = a[crossing], b[crossing]
        if len(a):
            points.append(a + (b - a) * ((z - a[:, 2]) / (b[:, 2] - a[:, 2]))[:, None])
    if not points:
        return None
    points = np.concatenate(points)
    low, high = points[:, :2].min(0), points[:, :2].max(0)
    return {'z_m': float(z), 'width_xy_m': (high - low).tolist(),
            'center_xy_m': ((low + high) / 2).tolist()}


def bottle_body_geometry(usd_path=DEFAULT_BOTTLE_USD, *, slice_step_m=.001,
                         contact_inset_m=.015):
    """Return bottle-local mesh bounds and an inset broad-body contact band.

    For this mustard asset, a broad slice has at least 90% of the maximum X
    width and 70% of maximum Y width. The longest contiguous broad interval is
    the body; inset each end by 15 mm to avoid the bottom edge and shoulder.
    These descriptive fractions are a grasp heuristic, not physics tolerances.
    """
    if slice_step_m <= 0 or contact_inset_m < 0:
        raise ValueError('Slice step must be positive and contact inset nonnegative')
    vertices, triangles = _source_triangles(usd_path)
    low, high = vertices.min(0), vertices.max(0)
    sections = [_section(triangles, z) for z in np.arange(low[2] + slice_step_m,
                                                        high[2], slice_step_m)]
    sections = [section for section in sections if section is not None]
    widths = np.asarray([section['width_xy_m'] for section in sections])
    thresholds = widths.max(0) * np.array([.90, .70])
    valid = np.all(widths >= thresholds, axis=1)
    runs, start = [], None
    for index, broad in enumerate(valid.tolist() + [False]):
        if broad and start is None:
            start = index
        elif not broad and start is not None:
            runs.append((start, index - 1))
            start = None
    if not runs:
        raise ValueError('No broad bottle-body cross-section interval found')
    first, last = max(runs, key=lambda item: sections[item[1]]['z_m'] - sections[item[0]]['z_m'])
    body = np.array([sections[first]['z_m'], sections[last]['z_m']])
    contact = body + np.array([contact_inset_m, -contact_inset_m])
    if contact[0] >= contact[1]:
        raise ValueError('Contact inset leaves no usable body interval')
    reference_z = float(np.clip(0., contact[0], contact[1]))
    reference_section = _section(triangles, reference_z)
    return {
        'asset': str(usd_path),
        'frame': 'Bottle rigid-body/default-prim local coordinates, metres, quaternion convention xyzw',
        'mesh_bounds_m': [low.tolist(), high.tolist()],
        'body_z_range_m': body.tolist(),
        'safe_contact_z_range_m': contact.tolist(),
        'contact_band_meaning': 'Inset broad mesh body only; actual bilateral PhysX contacts and scene clearance must be measured.',
        'width_thresholds_xy_m': thresholds.tolist(),
        'width_threshold_fractions_xy': [.90, .70],
        'slice_step_m': float(slice_step_m), 'contact_inset_m': float(contact_inset_m),
        'default_tcp_point_bottle_local_m': [*reference_section['center_xy_m'], reference_z],
        'default_tcp_meaning': 'TCP target within the broad body. TCP is not a finger-pad contact point; measure both contacts after closure.',
        'sections': sections,
    }


def body_grasp_tcp_target(bottle_pose_xyz_xyzw, local_z=0., *, geometry=None,
                          local_xy_offset=(0., 0.)):
    """Transform a body-local TCP target with the observed bottle pose.

    Local z=0 puts this asset's target near the middle of the bottle body. At
    the previous 45-degree grasp, the pad-reference midpoint was about 8 mm
    below TCP; this function does not assume that offset is a true contact.
    ``local_xy_offset`` shifts the TCP from the section centre in the bottle
    frame, for example to clear the gripper palm while retaining body contact.
    It does not change the allowed contact band or guarantee a physical grasp.
    """
    geometry = bottle_body_geometry() if geometry is None else geometry
    pose = np.asarray(bottle_pose_xyz_xyzw, dtype=float)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError('Bottle pose must be finite xyz + quaternion_xyzw')
    xy_offset = np.asarray(local_xy_offset, dtype=float)
    if xy_offset.shape != (2,) or not np.isfinite(xy_offset).all():
        raise ValueError('Bottle-local XY offset must be a finite two-vector')
    low, high = geometry['safe_contact_z_range_m']
    if not np.isfinite(local_z) or not low <= local_z <= high:
        raise ValueError(f'Local TCP z={local_z} is outside the body band [{low}, {high}]')
    point = np.asarray(geometry['default_tcp_point_bottle_local_m'], dtype=float).copy()
    point[:2] += xy_offset
    point[2] = local_z
    return pose[:3] + Rotation.from_quat(pose[3:]).apply(point)
