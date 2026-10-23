"""Task-independent CPU mesh freeze and offline shared gDist query execution."""
import hashlib
import itertools
from pathlib import Path
import numpy as np
from kitchen_safety.task_contract import digest


def authored_surface(prim,cache):
    """Preserve a zero-thickness authored Cube as its exact planar surface.

    Some Lightwheel panels intentionally have one zero scale axis. A collapsed
    cube's affine image is exactly the rectangle covered by its surviving face
    triangles. No thickness is invented and no bbox is used as a distance.
    Other singular/unsupported shapes remain explicit extraction errors.
    """
    from kitchen_safety.hs_geometry import surface_mesh
    try:return surface_mesh(prim,cache)
    except ValueError as exc:
        if str(exc)!='Invalid or degenerate collision shape transform' or prim.GetTypeName()!='Cube':raise
        from pxr import UsdGeom
        from kitchen_safety.fcl_distance import primitive_mesh
        mat=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        if not np.isfinite(mat).all() or not np.allclose(mat[3],[0,0,0,1]):raise
        singular=np.linalg.svd(mat[:3,:3],compute_uv=False)
        if singular[-1]>1e-15 or singular[-2]<=1e-12:raise
        size=float(UsdGeom.Cube(prim).GetSizeAttr().Get())
        if not np.isfinite(size) or size<=0:raise
        vertices,faces,_=primitive_mesh('Cube',size=size)
        vertices=vertices@mat[:3,:3].T+mat[:3,3];faces=np.asarray(faces,dtype=np.int32)
        areas=np.linalg.norm(np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]]),axis=1)
        surviving=faces[areas>0]
        if not len(surviving):raise
        return vertices,surviving,[dict(path=str(prim.GetPath()),method='exact rank-2 affine image of authored Cube',
            zero_thickness=True,discarded_zero_area_faces=int(np.sum(areas==0)),
            discarded_faces_subset_of_surviving_planar_rectangle=True,
            singular_values=singular.tolist(),physics_cooked_shape_equivalence='not asserted')]


def freeze_stage(stage, env_root, directory):
    """Freeze authored enabled collision surfaces; no GPU allocation/query.

    Static vertices are world-space; dynamic vertices include authored affine
    scaling but are local to the rigid actor's initial rotation/translation.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    root = stage.GetPrimAtPath(env_root)
    if not root:
        raise ValueError('Missing environment '+env_root)
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    inventory, joints, errors = [], [], []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            joints.append(dict(path=path, type=prim.GetTypeName(),
                body0=[str(x) for x in joint.GetBody0Rel().GetTargets()],
                body1=[str(x) for x in joint.GetBody1Rel().GetTargets()],
                enabled=joint.GetJointEnabledAttr().Get() is not False,
                attributes={a.GetName(): str(a.Get()) for a in prim.GetAttributes()
                    if a.GetName().startswith(('physics:', 'drive:'))},
                authored_angular_unit='degree; PhysX telemetry is radians'))
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        try:
            vertices, faces, approximation = authored_surface(prim, cache)
            owner = prim
            while owner and owner != root and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                owner = owner.GetParent()
            actor = str(owner.GetPath()) if owner and owner != root else None
            if actor and UsdPhysics.RigidBodyAPI(owner).GetRigidBodyEnabledAttr().Get() is False:
                actor = None
            if actor:
                authored = np.asarray(cache.GetLocalToWorldTransform(owner)).T
                u, _, vt = np.linalg.svd(authored[:3,:3]); rotation = u @ vt
                if np.linalg.det(rotation) < 0:
                    u[:,-1] *= -1; rotation = u @ vt
                vertices = (vertices-authored[:3,3]) @ rotation
            vertices = np.ascontiguousarray(vertices, dtype=np.float64)
            faces = np.ascontiguousarray(faces, dtype=np.int32)
            if vertices.ndim != 2 or faces.ndim != 2 or vertices.shape[1] != 3 or faces.shape[1] != 3 or not len(faces):
                raise ValueError('Invalid triangle arrays')
            if not np.isfinite(vertices).all() or faces.min() < 0 or faces.max() >= len(vertices):
                raise ValueError('Nonfinite mesh or invalid indices')
            key = hashlib.sha256(vertices.tobytes()+faces.tobytes()).hexdigest()
            path_out = directory/(key+'.npz')
            if not path_out.exists():
                with path_out.open('xb') as stream:
                    np.savez_compressed(stream, vertices=vertices, faces=faces)
            inventory.append(dict(path=path, actor=actor, dynamic=actor is not None,
                kinematic=bool(UsdPhysics.RigidBodyAPI(owner).GetKinematicEnabledAttr().Get()) if actor else False,
                fingerprint=key, vertices=len(vertices), triangles=len(faces), approximation=approximation))
        except Exception as exc:
            errors.append(dict(path=path, error=repr(exc)))
    result = dict(schema_version='robolaw-task-mesh-inventory-1', env_root=env_root,
        colliders=inventory, joints=joints, errors=errors, complete=bool(inventory) and not errors,
        mesh_frame='dynamic actor-local rigid frame with affine baked; static world',
        units='m', quaternion_order='xyzw', online_geometry_queries=0,
        representation='authored collision triangles; not cooked convex hulls or signed solid distance')
    result['inventory_sha256'] = digest(result)
    return result


class TaskMeshQueries:
    """Persistent models and one HS/PT/RS GeometryBatch per source-time frame."""
    def __init__(self, inventory, scope, directory, backend):
        from kitchen_safety.fcl_distance import GeometryBatch
        if backend.BACKEND_ID != 'gdist':
            raise ValueError('New task pipeline requires frozen gDist; no backend fallback')
        if scope['inventory_sha256'] != digest(inventory):
            raise ValueError('Scope/inventory identity differs')
        copied = dict(scope); identity = copied.pop('scope_sha256')
        if digest(copied) != identity:
            raise ValueError('Scope hash differs')
        self.scope, self.backend, self.Batch = scope, backend, GeometryBatch
        self.entries, models = {}, {}
        needed = set(scope['rs']['collider_paths'])
        for rel in scope['relations']:
            if rel['query_distance']:
                needed.update(rel['source_colliders']); needed.update(rel['target_colliders'])
        for source in inventory['colliders']:
            if source['path'] not in needed:
                continue
            key = source['fingerprint']
            if key not in models:
                with np.load(Path(directory)/(key+'.npz'), allow_pickle=False) as file:
                    vertices = np.ascontiguousarray(file['vertices'], dtype=np.float64)
                    faces = np.ascontiguousarray(file['faces'], dtype=np.int32)
                if hashlib.sha256(vertices.tobytes()+faces.tobytes()).hexdigest() != key:
                    raise ValueError('Frozen mesh hash differs: '+key)
                model = backend.BVHModel(); model.beginModel(); model.addSubModel(vertices, faces); model.endModel()
                models[key] = model
            model = models[key]
            self.entries[source['path']] = dict(source, model=model, object=backend.CollisionObject(model),
                corners=np.asarray(list(itertools.product(*zip(model.vertices.min(0),model.vertices.max(0))))), last_pose=None)
        if set(self.entries) != needed:
            raise ValueError('Query scope references missing collider geometry')

    def frame(self, actor_states_world, rs_pairs=(), *, operation_id=None, phase=None):
        """rs_pairs are exact audited body pairs, after structural exclusions."""
        from kitchen_safety.hs_geometry import transform
        batch = self.Batch(self.backend)
        errors, requests, collisions = {}, {}, {}
        omissions,effective_targets={},{}
        for path, entry in self.entries.items():
            try:
                pose = transform(actor_states_world[entry['actor']]) if entry['dynamic'] else np.eye(4)
                if entry['last_pose'] is None or not np.array_equal(pose, entry['last_pose']):
                    entry['object'].setTransform(self.backend.Transform(pose[:3,:3], pose[:3,3]))
                    corners = entry['corners'] @ pose[:3,:3].T + pose[:3,3]
                    entry['lo'], entry['hi'] = corners.min(0), corners.max(0)
                    entry['last_pose'] = pose.copy()
            except Exception as exc:
                errors[path] = repr(exc)
        def get(paths):
            if any(p in errors for p in paths):
                raise ValueError(str({p: errors[p] for p in paths if p in errors}))
            return [self.entries[p] for p in paths]
        for relation in self.scope['relations']:
            if not relation['query_distance']:
                continue  # no omitted distance record, not even a null/zero placeholder
            rid = relation['relation_id']
            selected=set()
            declarations=[]
            for declaration in relation.get('operation_distance_omissions',[]):
                if operation_id==declaration['operation_id'] and phase in declaration['allowed_phases']:
                    selected.update(declaration['target_colliders']);declarations.append(declaration['declaration_sha256'])
            targets=sorted(set(relation['target_colliders'])-selected)
            effective_targets[rid]=targets
            if selected:
                omissions[rid]=dict(operation_id=operation_id,phase=phase,source=relation['source'],
                    target_colliders=sorted(selected),fully_omitted=not targets,declaration_sha256=declarations,
                    query_distance=False,persist_distance=False,force_and_impulse_retained=True)
            if not targets:continue  # no null/zero distance or GPU query for a fully omitted relation
            try:
                requests[rid] = batch.minimum(get(relation['source_colliders']), get(targets))
            except Exception as exc:
                requests[rid] = dict(error=repr(exc))
        robot = set(self.scope['rs']['collider_paths'])
        owners = {}
        for path in robot:
            owners.setdefault(self.entries[path]['actor'], []).append(path)
        for left, right in rs_pairs:
            if left not in owners or right not in owners or left == right:
                raise ValueError('Unbound RS body pair')
            key = (left, right)
            try:
                aa, bb = get(owners[left]), get(owners[right])
                collisions[key] = [batch.collision(a,b) for a in aa for b in bb
                    if np.all(a['lo'] <= b['hi']) and np.all(b['lo'] <= a['hi'])]
            except Exception as exc:
                collisions[key] = [dict(error=repr(exc))]
        batch.execute()
        result = {}
        for rid, request in requests.items():
            try:
                result[rid] = dict(self.Batch.result(request), complete=True)
            except Exception as exc:
                result[rid] = dict(distance_m=None, complete=False, error=repr(exc))
            result[rid]['effective_target_colliders']=effective_targets[rid]
        self_results = []
        for pair, requests in collisions.items():
            missing = [r['error'] for r in requests if r.get('error')]
            positive = any(r.get('value') is True for r in requests)
            self_results.append(dict(pair=list(pair), intersection=True if positive else None if missing else False,
                complete=not missing, errors=missing))
        return dict(relations=result, distance_omissions=omissions,self_collision=self_results, batch_statistics=dict(batch.stats),
            scope_sha256=self.scope['scope_sha256'], geometry_backend=self.backend.METRIC)
