"""Read static USD collision shapes without replacing round primitives by boxes.

Used only by independent counterfactual audits. Existing conservative planner
and execution geometry stays unchanged. Poses come from the actual exported
stage; a moving object must first be reconstructed at its measured rigid pose.
"""
import fcl
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics


def static_collision_shapes(stage, root_path):
    root = stage.GetPrimAtPath(root_path)
    if not root: raise ValueError(f'Unknown collision root: {root_path}')
    cache = UsdGeom.XformCache(); rows = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(): continue
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim)).T
        position = matrix[:3, 3]; scale = np.linalg.norm(matrix[:3, :3], axis=0)
        rotation = matrix[:3, :3] / scale
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or np.linalg.det(rotation) < 0:
            raise ValueError(f'Sheared/reflected collider is unsupported: {prim.GetPath()}')
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim); vertices = np.asarray(mesh.GetPointsAttr().Get()) @ matrix[:3, :3].T + position
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()); faces = []; offset = 0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                poly = indices[offset:offset + count]; offset += count
                faces.extend([[poly[0], poly[i], poly[i + 1]] for i in range(1, count - 1)])
            model = fcl.BVHModel(); model.beginModel(len(vertices), len(faces)); model.addSubModel(vertices, np.asarray(faces, dtype=np.int32)); model.endModel()
            shape = fcl.CollisionObject(model); low = vertices.min(0); high = vertices.max(0)
        else:
            if prim.IsA(UsdGeom.Cube):
                lengths = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) * scale
                model = fcl.Box(*lengths); half = np.abs(rotation) @ (lengths / 2)
            elif prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Capsule):
                schema = UsdGeom.Cylinder(prim) if prim.IsA(UsdGeom.Cylinder) else UsdGeom.Capsule(prim)
                axis = {'X': 0, 'Y': 1, 'Z': 2}[str(schema.GetAxisAttr().Get())]
                perpendicular = [i for i in range(3) if i != axis]
                if not np.isclose(scale[perpendicular[0]], scale[perpendicular[1]], atol=1e-8, rtol=1e-7):
                    raise ValueError(f'Elliptical round primitive unsupported: {prim.GetPath()}')
                radius = float(schema.GetRadiusAttr().Get()) * scale[perpendicular[0]]
                height = float(schema.GetHeightAttr().Get()) * scale[axis]
                direction = rotation[:, axis].copy()
                align = np.eye(3) if axis == 2 else Rotation.from_euler('y' if axis == 0 else 'x', 90 if axis == 0 else -90, degrees=True).as_matrix()
                rotation = rotation @ align
                if prim.IsA(UsdGeom.Cylinder):
                    model = fcl.Cylinder(radius, height)
                    half = radius * np.sqrt(np.maximum(0., 1 - direction**2)) + height / 2 * np.abs(direction)
                else:
                    if not np.allclose(scale, scale[0], atol=1e-8, rtol=1e-7): raise ValueError('Anisotropically scaled capsule is unsupported')
                    model = fcl.Capsule(radius, height); half = radius + height / 2 * np.abs(direction)
            elif prim.IsA(UsdGeom.Sphere):
                if not np.allclose(scale, scale[0], atol=1e-8, rtol=1e-7): raise ValueError('Ellipsoid is unsupported')
                radius = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get()) * scale[0]
                model = fcl.Sphere(radius); half = np.full(3, radius)
            else: raise ValueError(f'Unsupported collider: {prim.GetPath()} ({prim.GetTypeName()})')
            shape = fcl.CollisionObject(model, fcl.Transform(rotation, position)); low = position - half; high = position + half
        rows.append((str(prim.GetPath()), shape, low, high))
    if not rows: raise ValueError(f'No enabled collision geometry: {root_path}')
    return rows


def minimum_distance(robot_shapes, obstacle_shapes, required_m):
    """Exact narrow-phase minimum, using AABBs only for safe pruning."""
    lo = np.array([s[2] for s in robot_shapes]); hi = np.array([s[3] for s in robot_shapes])
    other_lo = np.array([s[2] for s in obstacle_shapes]); other_hi = np.array([s[3] for s in obstacle_shapes])
    bounds = np.linalg.norm(np.maximum(np.maximum(lo[:, None] - other_hi, other_lo - hi[:, None]), 0), axis=2)
    pairs = sorted((float(bounds[i, j]), i, j) for i, j in np.ndindex(bounds.shape))
    best = None
    for bound, i, j in pairs:
        if best is not None and bound > best['distance_m']: break
        left, right = robot_shapes[i], obstacle_shapes[j]
        raw = float(fcl.distance(left[1], right[1], fcl.DistanceRequest(), fcl.DistanceResult()))
        hit = raw < 0 or (raw == 0 and bool(fcl.collide(left[1], right[1], fcl.CollisionRequest(), fcl.CollisionResult())))
        distance = max(0., raw)
        if best is None or distance <= best['distance_m']:
            best = {'valid': bool(not hit and distance >= required_m), 'distance_m': distance, 'required_m': required_m,
                'slack_m': distance - required_m, 'robot_part': left[0], 'obstacle': right[0], 'intersecting': bool(hit)}
            if hit: break
    if best is None: raise ValueError('Empty shape input')
    return best
