"""Independent HS distance observer over authored USD collision meshes.

FCL narrow phase measures mesh surfaces; AABBs only prune impossible minima.
This does not claim equivalence to PhysX cooked convex hulls or injury sensing.
Runtime poses are world metres and xyzw, as in this Arena installation.
"""
import hashlib
import itertools
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.fcl_distance import pair_distance, primitive_mesh, triangulate_polygon

VERSION = 'arena-hs-fcl-surface-4-affine'
SOURCE = 'FCL separation of USD collision surfaces (polygon/primitive surface tessellation); world_m; xyzw'
FIELDS = ('d_robot_h_min_gt_m', 'd_ee_h_min_gt_m', 'd_obj_h_min_gt_m')


def transform(pose):
    p = np.asarray(pose, dtype=float)
    if p.ndim != 1 or len(p) < 7 or not np.isfinite(p[:7]).all():
        raise ValueError('Missing/nonfinite xyz+xyzw pose')
    if abs(np.linalg.norm(p[3:7]) - 1) > 1e-3:
        raise ValueError('Invalid quaternion norm')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(p[3:7]).as_matrix()
    t[:3, 3] = p[:3]
    return t


def bounds_distance(a, b):
    return float(np.linalg.norm(np.maximum(0, np.maximum(a['lo']-b['hi'], b['lo']-a['hi']))))


def surface_mesh(prim, cache):
    """Resolve collision Xform descendants and tessellate authored faces."""
    from pxr import Usd, UsdGeom
    pieces = []
    approximation = []
    candidates = ([prim] if prim.IsA(UsdGeom.Boundable) else
                  [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
                   if p.GetTypeName() in ('Mesh','Cube','Sphere','Capsule','Cylinder')])
    if not candidates:
        raise ValueError('Collision declaration has no supported shape descendants')
    for p in candidates:
        if not p.IsLoaded() or p.GetPrimIndex().localErrors:
            raise ValueError('Unloaded or composition-error collision shape')
        if p.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(p)
            vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
            counts = list(mesh.GetFaceVertexCountsAttr().Get())
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
            if sum(counts) != len(indices) or any(n < 3 for n in counts):
                raise ValueError('Malformed polygon topology')
            if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(indices)
                    or not np.isfinite(vertices).all() or indices.min() < 0 or indices.max() >= len(vertices)):
                raise ValueError('Invalid authored mesh vertices/indices')
            faces = []
            offset = 0
            for n in counts:
                face = indices[offset:offset+n]
                offset += n
                faces.extend(triangulate_polygon(vertices, face))
            approximation.append(dict(path=str(p.GetPath()),method='authored polygon ear-clipping tessellation',
                                      nontriangle_faces=sum(n != 3 for n in counts)))
        elif p.GetTypeName() in ('Cube','Sphere','Capsule'):
            kind = p.GetTypeName()
            schema = getattr(UsdGeom, kind)(p)
            size = float(schema.GetSizeAttr().Get()) if kind == 'Cube' else None
            radius = float(schema.GetRadiusAttr().Get()) if kind != 'Cube' else None
            height = float(schema.GetHeightAttr().Get()) if kind == 'Capsule' else None
            axis = str(schema.GetAxisAttr().Get()) if kind == 'Capsule' else 'Z'
            if any(not np.isfinite(v) or v <= 0 for v in (size,radius,height) if v is not None):
                raise ValueError('Invalid primitive dimensions')
            vertices,faces,method = primitive_mesh(kind,radius=radius,height=height,size=size,axis=axis)
            approximation.append(dict(path=str(p.GetPath()),method=method))
        elif p.IsA(UsdGeom.Cylinder):
            cylinder = UsdGeom.Cylinder(p)
            radius, height = float(cylinder.GetRadiusAttr().Get()), float(cylinder.GetHeightAttr().Get())
            if not np.isfinite([radius,height]).all() or radius <= 0 or height <= 0:
                raise ValueError('Invalid cylinder dimensions')
            n = 128
            r = radius/np.cos(np.pi/n)
            angle = np.arange(n)*2*np.pi/n
            ring = np.c_[r*np.cos(angle),r*np.sin(angle)]
            vertices = np.concatenate([np.c_[ring,np.full(n,z)] for z in (-height/2,height/2)])
            axis = str(cylinder.GetAxisAttr().Get())
            if axis == 'X': vertices = vertices[:,[2,0,1]]
            elif axis == 'Y': vertices = vertices[:,[1,2,0]]
            elif axis != 'Z': raise ValueError('Invalid cylinder axis')
            faces = []
            for j in range(n):
                k = (j+1)%n
                faces.extend([(j,k,n+j),(k,n+k,n+j)])
            faces.extend((0,j+1,j) for j in range(1,n-1))
            faces.extend((n,n+j,n+j+1) for j in range(1,n-1))
            scale = np.linalg.norm(np.asarray(cache.GetLocalToWorldTransform(p)).T[:3,:3],ord=2)
            approximation.append(dict(path=str(p.GetPath()),method='circumscribed 128-sided cylinder',
                                      max_surface_error_world_m=float((r-radius)*scale)))
        else:
            raise ValueError('Unsupported collider type '+p.GetTypeName())
        faces = np.asarray(faces, dtype=np.int32).reshape(-1,3)
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(faces) or
                not np.isfinite(vertices).all() or faces.min() < 0 or faces.max() >= len(vertices)):
            raise ValueError('Invalid collision mesh')
        mat = np.asarray(cache.GetLocalToWorldTransform(p)).T
        if (mat.shape != (4,4) or not np.isfinite(mat).all()
                or not np.allclose(mat[3], [0,0,0,1])
                or np.linalg.svd(mat[:3,:3],compute_uv=False)[-1] < 1e-12):
            raise ValueError('Invalid or degenerate collision shape transform')
        vertices = vertices @ mat[:3,:3].T + mat[:3,3]
        pieces.append((vertices,faces))
    vertices, faces, offset = [], [], 0
    for v, f in pieces:
        vertices.append(v); faces.append(f+offset); offset += len(v)
    return np.concatenate(vertices),np.concatenate(faces),approximation


class HumanDistanceGeometry:
    def __init__(self, stage, env_root, body_names, target_ids=('mustard_bottle',),
                 human_ids=('person',), ee_names=None, fixed_body_names=('panda_link0',),
                 excluded_pairs=()):
        import fcl
        from pxr import Usd, UsdGeom, UsdPhysics
        self.fcl = fcl
        self.root = env_root.rstrip('/')
        self.entries = []
        self.errors = []
        body_names = set(body_names)
        self.fixed_body_names = set(fixed_body_names)
        self.ee_names = set(ee_names if ee_names is not None else
                            (n for n in body_names if not n.startswith('panda_link')))
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        root = stage.GetPrimAtPath(self.root)
        if not root:
            raise ValueError('Missing environment root '+self.root)
        inventory = []
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            relative = path[len(self.root)+1:]
            top = relative.split('/')[0]
            group = ('robot' if top == 'Robot' else 'human' if top in human_ids
                     else 'object' if top in target_ids else None)
            if group is None or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                continue
            inventory.append(path)
            try:
                vertices, faces, approximation = surface_mesh(prim, cache)
                ancestor = prim
                while ancestor and ancestor != root and not ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                    ancestor = ancestor.GetParent()
                actor = str(ancestor.GetPath()) if ancestor and ancestor != root else None
                if actor and not UsdPhysics.RigidBodyAPI(ancestor).GetRigidBodyEnabledAttr().Get():
                    actor = None
                local = np.eye(4)
                if actor is not None:
                    # Runtime rigid poses contain no scale. Bake authored scale
                    # into the geometry, stripping only rotation and translation.
                    actor_matrix = np.asarray(cache.GetLocalToWorldTransform(ancestor)).T
                    u, _, vt = np.linalg.svd(actor_matrix[:3,:3])
                    rotation = u @ vt
                    if np.linalg.det(rotation) < 0:
                        u[:,-1] *= -1
                        rotation = u @ vt
                    rigid = np.eye(4)
                    rigid[:3,:3],rigid[:3,3] = rotation,actor_matrix[:3,3]
                    local = np.linalg.inv(rigid)
                vertices = vertices @ local[:3, :3].T + local[:3, 3]
                if actor:
                    actor_key = actor.rsplit('/', 1)[-1] if group == 'robot' else actor[len(self.root)+1:]
                    if group == 'robot' and actor_key not in body_names:
                        raise ValueError('Robot collider actor absent from runtime body inventory')
                else:
                    actor_key = None
                model = fcl.BVHModel()
                model.beginModel(len(vertices), len(faces))
                model.addSubModel(vertices, faces)
                model.endModel()
                lo, hi = vertices.min(0), vertices.max(0)
                corners = np.asarray(list(itertools.product(*zip(lo, hi))))
                fingerprint = hashlib.sha256(vertices.tobytes()+faces.tobytes()).hexdigest()
                entry = dict(path=path, group=group, actor=actor_key, corners=corners,
                             model=model, object=fcl.CollisionObject(model), fingerprint=fingerprint,
                             vertices=len(vertices), triangles=len(faces), approximation=approximation)
                self.entries.append(entry)
            except Exception as exc:
                self.errors.append({'path':path, 'group':group, 'error':str(exc)})
        for group in ['robot'] + (['human'] if human_ids else []) + (['object'] if target_ids else []):
            if not any(e['group'] == group for e in self.entries):
                self.errors.append(dict(group=group, error='Empty collision geometry group'))
        if not any(e['group'] == 'robot' and e['actor'] in self.ee_names for e in self.entries):
            self.errors.append(dict(group='robot', error='Empty end-effector geometry'))
        self.excluded_pairs = {}
        declarations = []
        by_path = {e['path']:e for e in self.entries}
        for declaration in excluded_pairs:
            paths = [declaration['robot_collider'], declaration['human_collider']]
            if any(p.startswith('/') or any(x in p.split('/') for x in ('','..','.'))
                   or any(c in p for c in '*?[]') for p in paths):
                raise ValueError('HS exemptions require exact environment-relative collider paths')
            a, b = [self.root+'/'+p for p in paths]
            matched = a in by_path and b in by_path
            if matched:
                if (by_path[a]['group']!='robot' or by_path[a]['actor'] not in self.fixed_body_names
                        or by_path[b]['group']!='human'):
                    raise ValueError('HS exemption must bind a fixed robot collider and a human collider')
                self.excluded_pairs[(a,b)] = dict(declaration)
            declarations.append(dict(declaration, matched=matched,
                                     resolved_robot_collider=a, resolved_human_collider=b))
        self.metadata = dict(version=VERSION, source=SOURCE, coordinate_frame='world_m',
            quaternion_order='xyzw', ee_body_names=sorted(self.ee_names),
            fixed_body_names=sorted(self.fixed_body_names),
            scoped_distance_policy='HS excludes only explicitly bound collider pairs; raw minima and exclusions retained',
            excluded_pairs=declarations,
            target_ids=list(target_ids), human_ids=list(human_ids),
            inventoried_paths=inventory, complete=not self.errors, errors=self.errors,
            geometry=[{k:e[k] for k in ('path','group','actor','fingerprint','vertices','triangles','approximation')}
                      for e in self.entries],
            contact_semantics='Mesh intersection is not a PhysX contact event',
            geometry_semantics='Tessellated authored collision surfaces; cylinder error recorded; not PhysX cooked convex hulls')

    def update_objects(self, body_poses, object_poses):
        groups = {name:[] for name in ('robot','ee','object','human','fixed','moving')}
        for entry in self.entries:
            actor = entry['actor']
            if actor is None:
                t = np.eye(4)
            else:
                poses = body_poses if entry['group'] == 'robot' else object_poses
                if actor not in poses:
                    raise ValueError('Unmeasured rigid actor '+actor)
                t = transform(poses[actor])
            entry['object'].setTransform(self.fcl.Transform(t[:3, :3], t[:3, 3]))
            corners = entry['corners'] @ t[:3, :3].T + t[:3, 3]
            entry['lo'], entry['hi'] = corners.min(0), corners.max(0)
            groups[entry['group']].append(entry)
            if entry['group'] == 'robot' and actor in self.ee_names:
                groups['ee'].append(entry)
            if entry['group'] == 'robot':
                groups['fixed' if actor in self.fixed_body_names else 'moving'].append(entry)
        return groups

    def sample(self, body_poses, object_poses):
        if self.errors:
            return self.unknown(self.errors)
        try:
            groups = self.update_objects(body_poses, object_poses)
            # Cache exact pairs shared between robot-wide and EE minima.
            cache = {}
            results = {key:self.minimum(groups[key], groups['human'], cache)
                       for key in ('robot','ee','object','fixed','moving')}
            raw_results = {key:self.minimum(groups[key], groups['human'], cache, apply_exemptions=False)
                           for key in ('robot','fixed')}
            by_path = {e['path']:e for e in self.entries}
            exempted = [dict(declaration=declaration,
                            measurement=self.minimum([by_path[a]],[by_path[b]],cache,apply_exemptions=False))
                        for (a,b),declaration in self.excluded_pairs.items()]
            return dict(version=VERSION, source=SOURCE, complete=True, errors=[],
                        features=dict(zip(FIELDS, (results[k]['distance_m'] for k in ('robot','ee','object')))),
                        scoped_features=dict(d_fixed_h_min_gt_m=results['fixed']['distance_m'],
                                             d_moving_h_min_gt_m=results['moving']['distance_m']),
                        scene_placement=dict(fixed_human_mesh_intersection=results['fixed']['mesh_intersection'],
                            valid=not results['fixed']['mesh_intersection'],
                            evidence=results['fixed'],
                            raw_fixed_human_mesh_intersection=raw_results['fixed']['mesh_intersection'],
                            raw_geometry_valid=not raw_results['fixed']['mesh_intersection'],
                            raw_evidence=raw_results['fixed'],
                            semantics='valid concerns non-exempt pairs; raw_geometry_valid preserves actual overlap'),
                        nearest=results, raw_nearest=raw_results, exempted_pairs=exempted)
        except Exception as exc:
            return self.unknown([dict(error=str(exc))])

    @staticmethod
    def unknown(errors):
        return dict(version=VERSION, source=SOURCE, complete=False, errors=errors,
                    features=dict.fromkeys(FIELDS), nearest={})

    def excluded_contact(self, collider0, collider1):
        return ((collider0,collider1) in self.excluded_pairs or
                (collider1,collider0) in self.excluded_pairs)

    def filter_contact_pairs(self, pairs):
        """Filter exact collider contacts, not whole rigid bodies or humans."""
        applicable, exempted = [], []
        for pair in pairs:
            records = pair.get('contacts')
            if not records:
                # Missing collider identity cannot authorize an exemption.
                applicable.append(pair)
                continue
            kept = []
            for record in records:
                a,b = record.get('collider0'),record.get('collider1')
                if self.excluded_contact(a,b):
                    exempted.append(dict(collider0=a,collider1=b,points=record.get('points',[])))
                else:
                    kept.append(record)
            if kept:
                # HS consumes the pair as a contact flag only. Do not attach
                # aggregate force/impulse values which included excluded points.
                applicable.append(dict(bodyA=pair['bodyA'],bodyB=pair['bodyB'],contacts=kept))
        return applicable, exempted

    def minimum(self, left, right, cache, apply_exemptions=True):
        if not left or not right:
            raise ValueError('Missing distance pair geometry')
        candidates = sorted(((bounds_distance(a,b), i,j,a,b)
                             for i,a in enumerate(left) for j,b in enumerate(right)
                             if not (apply_exemptions and (a['path'],b['path']) in self.excluded_pairs)),
                            key=lambda row:row[:3])
        if not candidates:
            raise ValueError('No non-exempt distance pairs in declared HS scope')
        best = None
        for lower, _, _, a, b in candidates:
            if best is not None and lower > best['distance_m'] + 1e-12:
                continue
            key = (a['path'],b['path'])
            if key not in cache:
                measured = pair_distance(self.fcl, a, b)
                cache[key] = dict(measured, robot_or_object_path=a['path'],
                    human_path=b['path'], body=a['actor'])
            candidate = cache[key]
            if best is None or candidate['distance_m'] < best['distance_m']:
                best = candidate
        return best
