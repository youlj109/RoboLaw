"""CPU contracts for a read-only PhysX cooking snapshot, never a safety gate.

The installed PhysX demo transforms returned vertices by the collision prim's
local-to-world transform. This module retains the raw planes and vertex order;
it does not equate a cooking request with a running native shape or relax a
source-triangle clearance. Native shape-slot ordering remains unverified.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


TASK = "kitchen_bench_replicator_g_shape_bowl_sink"
BOWL_ROOT = "/World/envs/env_0/bowl"
BOWL_COLLIDER = BOWL_ROOT + "/obj_000013_Mesh"
FLOOR_COLLIDER = "/World/envs/env_0/kitchen/Sink/BasinBottom"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def finite_array(value, shape=None):
    result = np.asarray(value, dtype=float)
    require(np.isfinite(result).all() and (shape is None or result.shape == shape), "Nonfinite or malformed numeric snapshot")
    return result


def transform_points(points, matrix):
    """Column-vector 4x4 matrix; transpose Gf row-vector matrices at the caller."""
    points, matrix = finite_array(points), finite_array(matrix, (4, 4))
    require(points.ndim == 2 and points.shape[1] == 3 and len(points), "Missing 3D vertices")
    require(np.array_equal(matrix[3], [0, 0, 0, 1]) and abs(np.linalg.det(matrix[:3, :3])) > 1e-15,
            "Require invertible affine collision transform")
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def validate_convex(raw, local_to_world):
    """Validate copied callback topology without inventing a physical tolerance."""
    vertices = finite_array(raw["vertices_local"])
    require(vertices.ndim == 2 and vertices.shape[1:] == (3,) and len(vertices) >= 4, "Invalid convex vertices")
    raw_indices = np.asarray(raw["indices"])
    require(raw_indices.ndim == 1 and raw_indices.dtype.kind in "iu" and len(raw_indices), "Convex indices must be integers")
    indices = raw_indices.astype(np.int64)
    require(indices.min() >= 0 and indices.max() < len(vertices), "Convex index outside vertices")
    require(raw["polygons"], "Missing convex polygons")
    covered = np.zeros(len(indices), dtype=np.int64)
    triangles, residuals = [], []
    for polygon in raw["polygons"]:
        start, count = polygon["index_base"], polygon["num_vertices"]
        require(type(start) is int and type(count) is int and start >= 0 and count >= 3 and start+count <= len(indices),
                "Invalid polygon range")
        covered[start:start+count] += 1
        face = indices[start:start+count]
        require(len(set(face)) == count, "Repeated vertex in convex polygon")
        plane = finite_array(polygon["plane_local"], (4,))
        normal_norm = float(np.linalg.norm(plane[:3]))
        require(normal_norm > 0, "Zero plane normal")
        # Numeric diagnostic only: API planes retained verbatim, no safety gate.
        residuals.append(float(np.max(np.abs(vertices[face] @ plane[:3] + plane[3])) / normal_norm))
        triangles.extend([[int(face[0]), int(face[i]), int(face[i+1])] for i in range(1, count-1)])
    require(np.all(covered == 1), "Polygon indices must be a disjoint exhaustive partition")
    world = transform_points(vertices, local_to_world)
    return dict(**raw, geometry_sha256=digest(raw), triangles=triangles,
        vertices_world=world.tolist(), bounds_local=[vertices.min(0).tolist(), vertices.max(0).tolist()],
        bounds_world=[world.min(0).tolist(), world.max(0).tolist()],
        plane_vertex_max_residual_local_units=max(residuals),
        plane_residual_scope="Diagnostic only; no contact/safety threshold inferred")


def serialize_convex(convex, local_to_world, callback_index):
    def vector(value, n):
        return [float(value[i]) for i in range(n)]
    raw = dict(vertices_local=[vector(vertex, 3) for vertex in convex.vertices],
        indices=[int(index) for index in convex.indices],
        polygons=[dict(index_base=int(face.index_base), num_vertices=int(face.num_vertices),
                       plane_local=vector(face.plane, 4)) for face in convex.polygons])
    return dict(callback_index=int(callback_index), **validate_convex(raw, local_to_world))


def representation(result_name, convexes, local_to_world):
    """Preserve unsupported boundingCube requests explicitly; no proxy fallback."""
    if result_name != "RESULT_VALID":
        require(not convexes, "Nonvalid result unexpectedly supplied convexes")
        return dict(result=result_name, available=False, hull_count=0, hulls=[],
            runtime_shape_equivalence_verified=False)
    require(convexes, "RESULT_VALID without geometry")
    hulls = [serialize_convex(item, local_to_world, i) for i, item in enumerate(convexes)]
    return dict(result=result_name, available=True, hull_count=len(hulls), hulls=hulls,
        unordered_geometry_sha256=digest(sorted(item["geometry_sha256"] for item in hulls)),
        callback_order_scope="Installed demo says convex decomposition order is not reproducible; never use index as a native shape ID",
        runtime_shape_equivalence_verified=False)


def native_inventory(paths, count, max_shapes, offsets, source_colliders, hull_count=None):
    """Single-body max_shapes gives a count; it still supplies no slot identity."""
    require(type(count) is int and type(max_shapes) is int and count >= 1 and max_shapes >= 1,
            "Invalid native view inventory")
    require(len(paths) == count and len(set(paths)) == count, "Native body path/count mismatch")
    arrays = {}
    for key in ("rest_offsets", "contact_offsets"):
        arrays[key] = finite_array(offsets[key], (count, max_shapes)).tolist()
    one = count == 1
    native_count = max_shapes if one else None
    return dict(native_body_paths=paths, body_count=count, max_shapes=max_shapes, **arrays,
        actual_shape_count=native_count,
        actual_shape_count_basis=("RigidBodyView.max_shapes is documented as the maximum number of shapes among the bodies; this view contains exactly one body"
            if one else "No per-body shape count exposed; max_shapes cannot identify padding per body"),
        authored_collision_prim_paths=list(source_colliders), cooking_hull_count=hull_count,
        count_matches_cooking=(native_count == hull_count if one and hull_count is not None else None),
        native_shape_slot_to_hull=None,
        native_shape_slot_mapping_verified=False,
        runtime_shape_equivalence_verified=False,
        scope="Count consistency is not a geometric or slot-order equality proof; all offset columns retained without assigning hull IDs")


def read_declaration(path, root):
    value = json.loads(Path(path).read_text())
    require(value["schema_version"] == 1 and value["task_id"] == TASK, "Wrong snapshot declaration")
    require(value["accepted"] is False and value["diagnostic_only"] is True, "Snapshot cannot be accepted data")
    require(value["targets"] == {"bowl": BOWL_COLLIDER, "floor": FLOOR_COLLIDER}, "Only the original bowl/floor are declared")
    require(value["maximum_update_calls"] == 300 and value["maximum_wait_seconds"] == 30,
            "Unreviewed cooking wait bounds")
    require(value["maximum_control_actions"] == 0 and value["explicit_simulation_steps"] == 0,
            "Snapshot must not run a physical experiment")
    for relative, expected in value["source_sha256"].items():
        require(sha(Path(root)/relative) == expected, "Snapshot source changed: " + relative)
    return value
