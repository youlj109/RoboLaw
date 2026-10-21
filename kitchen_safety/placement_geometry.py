"""Measured bottle-to-bowl lowering targets, without starting a simulator.

Targets describe the authored source triangles. They neither predict cooked
PhysX contact nor replace planning, execution-clearance, or task checks. The
caller must reach the placement column first and keep the measured orientation
during this local vertical motion.
"""
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.grasp_geometry import DEFAULT_BOTTLE_USD, _source_triangles


DEFAULT_BOWL_USD = Path(__file__).resolve().parents[1] / 'assets/robolab/objects/ycb/bowl.usd'


@lru_cache(maxsize=8)
def _meshes(bottle_usd, bowl_usd):
    bottle_vertices, _ = _source_triangles(bottle_usd)
    _, bowl_triangles = _source_triangles(bowl_usd)
    # Never mutate the cached source geometry while transforming a live pose.
    bottle_vertices.setflags(write=False)
    bowl_triangles.setflags(write=False)
    return bottle_vertices, bowl_triangles


def _pose(value, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (7,) or not np.isfinite(value).all():
        raise ValueError(f'{name} must contain finite xyz + quaternion_xyzw')
    if np.linalg.norm(value[3:]) < 1e-8:
        raise ValueError(f'{name} has a zero quaternion')
    return value[:3], Rotation.from_quat(value[3:]).as_matrix()


def bottle_bowl_lower_target(
    bottle_pose_xyz_xyzw, bowl_pose_xyz_xyzw, tcp_xyz, *,
    nominal_gap_m=.003, bottom_band_m=.008, max_lowering_m=.25,
    bottle_usd=DEFAULT_BOTTLE_USD, bowl_usd=DEFAULT_BOWL_USD,
):
    """Return a vertical TCP correction from the two current physical poses.

    Cast vertical rays through the bottle's lowest ``bottom_band_m`` of source
    vertices. For each ray, take the highest bowl surface at that XY, then
    lower until the first source vertex would be ``nominal_gap_m`` above it.
    Both meshes are transformed to world coordinates, so bowl tilt is included.
    The TCP XY and bottle orientation are retained. The function never changes
    the simulator, attachment, collision margins, or success requirements.

    The bottom band is a support-target heuristic, not an exact swept-volume
    test. A candidate still needs the normal robot/payload path checks and an
    opening/withdrawal check. A small authored gap can already be physical
    contact with a cooked collider. Inspect the physical state before release.
    """
    bottle_position, bottle_rotation = _pose(bottle_pose_xyz_xyzw, 'Bottle pose')
    bowl_position, bowl_rotation = _pose(bowl_pose_xyz_xyzw, 'Bowl pose')
    tcp_xyz = np.asarray(tcp_xyz, dtype=float)
    if tcp_xyz.shape != (3,) or not np.isfinite(tcp_xyz).all():
        raise ValueError('TCP must contain three finite world coordinates')
    if not np.isfinite([nominal_gap_m, bottom_band_m, max_lowering_m]).all():
        raise ValueError('Gap, bottom band, and maximum lowering must be finite')
    if nominal_gap_m < 0 or bottom_band_m <= 0 or max_lowering_m <= 0:
        raise ValueError('Gap must be nonnegative; bottom band and lowering limit must be positive')
    if bottle_rotation[2, 2] <= 0 or bowl_rotation[2, 2] <= 0:
        raise ValueError('Bottle and bowl must face upward for vertical placement')

    source_vertices, source_triangles = _meshes(str(bottle_usd), str(bowl_usd))
    vertices = source_vertices @ bottle_rotation.T + bottle_position
    triangles = source_triangles @ bowl_rotation.T + bowl_position
    minimum_z = float(vertices[:, 2].min())
    bottom = vertices[vertices[:, 2] <= minimum_z + bottom_band_m]
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    first, second = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2]
    determinant = first[:, 0] * second[:, 1] - second[:, 0] * first[:, 1]
    valid_area = np.abs(determinant) > 1e-12
    xy_low, xy_high = triangles[:, :, :2].min(1), triangles[:, :, :2].max(1)
    maximum_shift = -float('inf')
    critical_point = None
    critical_support_z = None
    for point in bottom:
        possible = valid_area & np.all(point[:2] >= xy_low - 1e-8, axis=1) & np.all(point[:2] <= xy_high + 1e-8, axis=1)
        offset = point[:2] - a[possible, :2]
        denom = determinant[possible]
        edge_one, edge_two = first[possible], second[possible]
        u = (offset[:, 0] * edge_two[:, 1] - edge_two[:, 0] * offset[:, 1]) / denom
        v = (edge_one[:, 0] * offset[:, 1] - offset[:, 0] * edge_one[:, 1]) / denom
        inside = (u >= -1e-7) & (v >= -1e-7) & (u + v <= 1.0000001)
        if not inside.any():
            raise ValueError(f'Bottle lower footprint has no bowl support at world XY {point[:2].tolist()}')
        base, one, two = a[possible, 2], b[possible, 2], c[possible, 2]
        support_z = float((base + u * (one - base) + v * (two - base))[inside].max())
        shift = support_z - float(point[2]) + nominal_gap_m
        if shift > maximum_shift:
            maximum_shift = shift
            critical_point = point.copy()
            critical_support_z = support_z
    if maximum_shift > 1e-6:
        raise ValueError(f'Bottle is already below the requested source-mesh gap; required upward motion {maximum_shift:.6f} m')
    if maximum_shift < -max_lowering_m:
        raise ValueError(f'Requested lowering {-maximum_shift:.6f} m exceeds {max_lowering_m:.6f} m')
    target = tcp_xyz.copy()
    target[2] += maximum_shift
    return {
        'lower_target_xyz_world_m': target.tolist(),
        'vertical_delta_m': float(maximum_shift),
        'nominal_gap_m': float(nominal_gap_m),
        'bottom_band_m': float(bottom_band_m),
        'source_bottle_minimum_world_z_m': minimum_z,
        'minimum_vertical_gap_before_m': float(nominal_gap_m - maximum_shift),
        'critical_bottle_vertex_world_m': critical_point.tolist(),
        'critical_bowl_surface_world_z_m': critical_support_z,
        'bottom_vertices_checked': len(bottom),
        'bottle_tilt_deg': float(np.rad2deg(np.arccos(np.clip(bottle_rotation[2, 2], -1, 1)))),
        'bowl_tilt_deg': float(np.rad2deg(np.arccos(np.clip(bowl_rotation[2, 2], -1, 1)))),
        'semantics': 'Nominal source-triangle support gap from measured poses; physical support and collision checks remain required',
    }


def release_lowering_target(tcp_xyz, bottle_pose_xyz_xyzw, bowl_pose_xyz_xyzw, **kwargs):
    """Collector interface: return ``(target_xyz, source-geometry diagnostics)``."""
    diagnostics = bottle_bowl_lower_target(bottle_pose_xyz_xyzw, bowl_pose_xyz_xyzw, tcp_xyz, **kwargs)
    return np.asarray(diagnostics['lower_target_xyz_world_m'], dtype=float), diagnostics
