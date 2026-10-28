"""CPU source collision surfaces and real support rays for an assigned A layout.

This is an initial geometry gate only, never an execution certificate.  The
original task relation validators are also required.  Outputs are immutable.
"""
import argparse
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation
import fcl

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from kitchen_safety.geometry_audit import static_collision_shapes, minimum_distance
from scripts.extensions.replicator.audit_task_source_relation import source, check_layout, STAND
RAY_BOUNDS = {}
REQUIRED_M = .002


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def triangles(stage, root):
    cache = UsdGeom.XformCache()
    rows = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        mesh = UsdGeom.Mesh(prim)
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim)).T
        verts = np.asarray(mesh.GetPointsAttr().Get()) @ matrix[:3, :3].T + matrix[:3, 3]
        indices, offset, faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()), 0, []
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face = indices[offset:offset + count]
            offset += count
            faces.extend([[face[0], face[j], face[j + 1]] for j in range(1, count - 1)])
        rows.append((str(prim.GetPath()), verts, np.asarray(faces, dtype=np.int32)))
    return rows


def pose_rows(stage, root, position, yaw, subset=None):
    initial = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(root))).T
    # Retain authored root scale, replace only its world rigid pose.
    u, _, vh = np.linalg.svd(initial[:3, :3])
    target = np.eye(4)
    target[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix() @ ((u @ vh).T @ initial[:3, :3])
    target[:3, 3] = position
    delta = target @ np.linalg.inv(initial)
    rotation, translation = delta[:3, :3], delta[:3, 3]
    result = []
    for name, obj, lo, hi in static_collision_shapes(stage, subset or root):
        obj.setTransform(fcl.Transform(rotation @ np.asarray(obj.getRotation()), rotation @ np.asarray(obj.getTranslation()) + translation))
        center, radius = rotation @ ((lo + hi) / 2) + translation, np.abs(rotation) @ ((hi - lo) / 2)
        result.append((name, obj, center - radius, center + radius))
    verts = np.concatenate([v for _, v, _ in triangles(stage, subset or root)])
    return result, verts @ rotation.T + translation


def ray_z(meshes, point):
    hits = []
    for name, verts, faces in meshes:
        key = id(verts)
        if key not in RAY_BOUNDS:
            RAY_BOUNDS[key] = (verts[:, :2].min(0), verts[:, :2].max(0))
        low_xy, high_xy = RAY_BOUNDS[key]
        if np.any(point[:2] < low_xy - 1e-10) or np.any(point[:2] > high_xy + 1e-10):
            continue
        tri = verts[faces]
        lo, hi = tri[:, :, :2].min(1), tri[:, :, :2].max(1)
        tri = tri[np.all(lo <= point[:2] + 1e-10, 1) & np.all(hi >= point[:2] - 1e-10, 1)]
        if not len(tri):
            continue
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        v0, v1, v2 = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2], point[:2] - a[:, :2]
        den = v0[:, 0] * v1[:, 1] - v0[:, 1] * v1[:, 0]
        ok = np.abs(den) > 1e-14
        aa = np.zeros(len(tri)); bb = aa.copy()
        aa[ok] = (v2[ok, 0] * v1[ok, 1] - v2[ok, 1] * v1[ok, 0]) / den[ok]
        bb[ok] = (v0[ok, 0] * v2[ok, 1] - v0[ok, 1] * v2[ok, 0]) / den[ok]
        z = a[:, 2] + aa * (b[:, 2] - a[:, 2]) + bb * (c[:, 2] - a[:, 2])
        eligible = ok & (aa >= -1e-9) & (bb >= -1e-9) & (aa + bb <= 1 + 1e-9) & (z <= point[2] + .01)
        if eligible.any():
            top = float(z[eligible].max())
            hits.append(dict(collider=name, surface_z_m=top, vertical_gap_m=float(point[2] - top)))
    return max(hits, key=lambda x: x['surface_z_m']) if hits else None


def diagnose(config):
    RAY_BOUNDS.clear()
    config = Path(config)
    layout = json.loads(config.read_text())
    relations = check_layout(layout)
    entry, spec = source(layout['task_id'])
    kitchen = Usd.Stage.Open(str(ROOT / layout['kitchen_usd']))
    kitchen_root = str(kitchen.GetDefaultPrim().GetPath())
    shapes = static_collision_shapes(kitchen, kitchen_root)
    meshes = triangles(kitchen, kitchen_root)
    rows, objects, stages = {}, {}, [kitchen]
    refs = {row['id']: kitchen_root + '/' + row['prim_path'] for row in spec['object_references']}
    for name, cfg in layout['objects'].items():
        stage = Usd.Stage.Open(str(ROOT / cfg['usd_path'])); stages.append(stage)
        obj, verts = pose_rows(stage, str(stage.GetDefaultPrim().GetPath()), cfg['position'], cfg['yaw_deg'])
        objects[name] = obj
        support = next(row['reference'] for row in spec['relations'] if row['kind'] == 'on' and row['subject'] == name)
        support_meshes = [row for row in meshes if row[0] == refs[support] or row[0].startswith(refs[support] + '/')]
        lowest = verts[verts[:, 2] <= verts[:, 2].min() + .002]
        # All near-bottom source mesh vertices, not AABB corners or a convex hull.
        rays = [dict(point=point.tolist(), declared_support_hit=ray_z(support_meshes, point),
                     any_kitchen_mesh_hit=ray_z(meshes, point)) for point in lowest]
        rows[name] = dict(source_mesh_world_bounds=[verts.min(0).tolist(), verts.max(0).tolist()],
            declared_support=refs[support], declared_support_collision_mesh_count=len(support_meshes),
            source_surface_minimum_to_kitchen=minimum_distance(obj, shapes, REQUIRED_M),
            near_bottom_vertex_band_m=.002, sampled_vertices=len(lowest), rays=rays,
            declared_support_hit_count=sum(x['declared_support_hit'] is not None for x in rays))
    pair = [dict(left=a, right=b, source_surface_distance=minimum_distance(objects[a], objects[b], REQUIRED_M))
            for i, a in enumerate(objects) for b in list(objects)[i+1:]]
    stand_stage = Usd.Stage.Open(str(ROOT / STAND)); stages.append(stand_stage)
    position = np.asarray(layout['robot_position']) + [0, 0, layout['stand_height']]
    stand, _ = pose_rows(stand_stage, '/panda', position, layout['robot_yaw_deg'], '/panda/panda_link0/stand_instanceable')
    floor = refs['floor']
    nonfloor = [row for row in shapes if not (row[0] == floor or row[0].startswith(floor + '/'))]
    stand_check = minimum_distance(stand, nonfloor, REQUIRED_M)
    stand_objects = {name: minimum_distance(stand, obj, REQUIRED_M) for name, obj in objects.items()}
    dependencies = {}
    for stage in stages:
        for layer in stage.GetUsedLayers():
            path = Path(layer.realPath)
            if path.is_file():
                dependencies[str(path)] = sha(path)
    result = dict(task_id=layout['task_id'], layout=layout, layout_sha256=sha(config),
        source_yaml=entry['source_yaml'], source_yaml_sha256=entry['source_sha256'],
        geometry_layer_sha256=dependencies, objects=rows, object_pairs=pair,
        stand_to_nonfloor_kitchen=stand_check, stand_to_task_objects=stand_objects,
        original_source_relations=relations,
        required_clearance_m=REQUIRED_M,
        scope='Read-only authored collision triangle surfaces and supported analytic primitives; no hull/AABB substitution for narrow phase. AABBs only prune FCL pairs. Ray heights use actual triangles. Triangle shells do not prove solid-containment freedom or cooked PhysX SDF/contact correctness. Lowest-vertex rays are diagnostic, not a continuous support-contact or stability certificate. No IK, full-robot initial configuration collision, physical collection or task success has been validated.',
        execution_ready=False,
        code_snapshots={str(Path(__file__).relative_to(ROOT)): Path(__file__).read_text(),
                        'kitchen_safety/geometry_audit.py': (ROOT/'kitchen_safety/geometry_audit.py').read_text(),
                        'scripts/extensions/replicator/audit_task_source_relation.py': (ROOT/'scripts/extensions/replicator/audit_task_source_relation.py').read_text()})
    # No epsilon is applied to the existing 2 mm clearance gate.  The 5 mm
    # maximum ray gap is the unchanged original On placement height band.
    checks = dict(original_relations=relations['initial_source_relations_passed'],
        stand_source_clearance=stand_check['valid'],
        objects_source_clearance=all(row['source_surface_minimum_to_kitchen']['valid'] for row in rows.values()),
        task_object_pair_clearance=all(row['source_surface_distance']['valid'] for row in pair),
        stand_object_clearance=all(row['valid'] for row in stand_objects.values()),
        declared_real_surface_below_every_near_bottom_vertex=all(
            row['sampled_vertices'] > 0 and all(ray['declared_support_hit'] is not None
            and 0 <= ray['declared_support_hit']['vertical_gap_m'] <= .005 + row['near_bottom_vertex_band_m']
            for ray in row['rays']) for row in rows.values()))
    result.update(checks=checks, initial_geometry_passed=all(checks.values()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report = diagnose(args.layout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(output=str(args.output), checks=report['checks'], execution_ready=False)), flush=True)
    return int(not report['initial_geometry_passed'])


if __name__ == '__main__':
    raise SystemExit(main())
