"""PT v4 gDist CUDA authored collision-surface distances in world metres.

Scale/shear is baked into vertices; runtime transforms are rigid. AABBs prune
candidates only. These are surface distances, not PhysX cooked-solid depths.
"""
import hashlib
import itertools
import numpy as np
from kitchen_safety.hs_geometry import transform, surface_mesh
from kitchen_safety.pt_assets import under
from kitchen_safety.fcl_distance import METRIC, minimum_distance

VERSION = 'arena-pt-gdist-triangle-surface-1'
TARGET_SOURCE = 'link_target_gdist_surface_distance_gt'
ENV_SOURCE = 'pt_environment_gdist_surface_distance_gt'


class PropertyDistanceGeometry:
    def __init__(self, stage, env_root, body_names, asset_inventory, grip_names, backend=None):
        from kitchen_safety.geometry_backend import get_backend,descriptor
        fcl=backend or get_backend()
        spec=descriptor(fcl.BACKEND_ID)
        from pxr import Usd, UsdGeom, UsdPhysics
        self.fcl = fcl
        self.root = env_root.rstrip('/')
        self.entries, self.errors, self.robot_links = [], [], {}
        self.assets = asset_inventory['objects']
        self.target = asset_inventory['target_object_ids'][0]
        from kitchen_safety.pt_distance_policy import omitted_keys
        self.omitted_distance_relations = omitted_keys(asset_inventory)
        body_names = set(body_names)
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        root = stage.GetPrimAtPath(self.root)
        if not root:
            raise ValueError('Missing environment root ' + self.root)
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            group = 'robot' if under(path, self.root + '/Robot') else next(
                (name for name, asset in self.assets.items() if under(path, asset['prim_path'])), None)
            if group is None:
                continue
            try:
                if not prim.IsLoaded() or prim.GetPrimIndex().localErrors:
                    raise ValueError('Unloaded or composition-error collider')
                owner = prim
                while owner and owner != root and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                    owner = owner.GetParent()
                actor = str(owner.GetPath()) if owner and owner != root else None
                key = (actor.rsplit('/', 1)[-1] if group == 'robot' else actor[len(self.root)+1:]) if actor else None
                if group == 'robot':
                    if key not in body_names:
                        raise ValueError('Robot collider lacks a measured rigid body')
                    self.robot_links.setdefault(key, set()).add(actor)
                dynamic = actor is not None and UsdPhysics.RigidBodyAPI(owner).GetRigidBodyEnabledAttr().Get() is not False
                local = np.eye(4)
                if dynamic:
                    authored = np.asarray(cache.GetLocalToWorldTransform(owner), dtype=float).T
                    u, singular, vt = np.linalg.svd(authored[:3, :3])
                    if not np.isfinite(singular).all() or singular[-1] < 1e-12:
                        raise ValueError('Degenerate actor transform')
                    rotation = u @ vt
                    if np.linalg.det(rotation) < 0:
                        u[:, -1] *= -1
                        rotation = u @ vt
                    rigid = np.eye(4)
                    rigid[:3, :3], rigid[:3, 3] = rotation, authored[:3, 3]
                    local = np.linalg.inv(rigid)
                vertices, faces, approximation = surface_mesh(prim, cache)
                vertices = vertices @ local[:3, :3].T + local[:3, 3]
                model = fcl.BVHModel()
                model.beginModel(len(vertices), len(faces))
                model.addSubModel(vertices, faces)
                model.endModel()
                lo, hi = vertices.min(0), vertices.max(0)
                self.entries.append(dict(path=path, group=group, actor=actor, key=key,
                    dynamic=dynamic, last_pose=None, model=model, object=fcl.CollisionObject(model),
                    corners=np.asarray(list(itertools.product(*zip(lo, hi)))),
                    fingerprint=hashlib.sha256(vertices.tobytes()+faces.tobytes()).hexdigest(),
                    vertices=len(vertices), triangles=len(faces), approximation=approximation))
            except Exception as exc:
                self.errors.append(dict(path=path, group=group, error=str(exc)))
        self.grip_paths = []
        for name in grip_names:
            paths = self.robot_links.get(name, set())
            if len(paths) != 1:
                raise ValueError('PT v4 grip link must map to one collider body: ' + name)
            self.grip_paths.extend(paths)
        if not self.grip_paths:
            raise ValueError('PT v4 requires exact grip paths')
        self.metadata = dict(version=spec['pt_version'], metric=spec['metric'], mesh_backend=fcl.BACKEND_ID,
            query_execution=spec['query_execution'],
            target_source=spec['target_source'], environment_source=spec['environment_source'],
            formula='min_collider_pairs('+spec['label']+'_triangle_surface_distance)',
            broad_phase='Conservative AABB pruning only; no AABB/projection fallback',
            query_strategy='Complete-actor authored triangle minima for environment pairs; per-link minima retained for robot-target grasp evidence',
            affine_support='Full authored affine transform baked into mesh vertices',
            geometry_semantics='Authored collision surfaces; analytic primitives tessellated with recorded resolution; not PhysX cooked solids',
            topology='Frozen at observer construction; measured rigid poses updated each sample',
            coordinate_frame='world_m', quaternion_order='xyzw', complete=not self.errors,
            errors=self.errors, exact_grip_link_paths=self.grip_paths,
            omitted_distance_relations=[dict(actor=a, object_id=o) for a,o in sorted(self.omitted_distance_relations)],
            geometry=[{k:e[k] for k in ('path','group','actor','dynamic','fingerprint','vertices','triangles','approximation')} for e in self.entries])

    def sample(self, body_poses, object_poses):
        from kitchen_safety.fcl_distance import GeometryBatch
        batch=GeometryBatch(self.fcl)
        finish=self.prepare(body_poses,object_poses,batch)
        batch.execute()
        return finish()

    def prepare(self,body_poses,object_poses,batch):
        groups, errors = {}, list(self.errors)
        for entry in self.entries:
            try:
                if entry['dynamic']:
                    poses = body_poses if entry['group'] == 'robot' else object_poses
                    if entry['key'] not in poses:
                        raise ValueError('Unmeasured rigid actor: ' + str(entry['actor']))
                    pose = transform(poses[entry['key']])
                else:
                    pose = np.eye(4)
                if entry['last_pose'] is None or not np.array_equal(pose, entry['last_pose']):
                    entry['object'].setTransform(self.fcl.Transform(pose[:3, :3], pose[:3, 3]))
                    corners = entry['corners'] @ pose[:3, :3].T + pose[:3, 3]
                    entry['lo'], entry['hi'] = corners.min(0), corners.max(0)
                    entry['last_pose'] = pose.copy()
                groups.setdefault(entry['group'], []).append(entry)
            except Exception as exc:
                errors.append(dict(path=entry['path'], group=entry['group'], error=str(exc)))
        links = {}
        for entry in groups.get('robot', []):
            links.setdefault(entry['actor'], []).append(entry)

        planned=[]
        for object_id in self.assets:
            destination=groups.get(object_id,[])
            for actor in (('robot',) if object_id==self.target else ('robot','held_object')):
                if (actor,object_id) in self.omitted_distance_relations:continue
                source_group='robot' if actor=='robot' else self.target
                source=groups.get(source_group,[])
                missing=[e for e in errors if e['group'] in (source_group,object_id)]
                if not source or not destination:missing=missing+[dict(error='Empty declared source or destination geometry')]
                per_link=actor=='robot' and object_id==self.target
                query_groups=links.items() if per_link else [(self.root+('/Robot' if actor=='robot' else '/'+self.target),source)]
                queries={name:batch.minimum(entries,destination) for name,entries in query_groups} if not missing else {}
                planned.append((actor,object_id,source,destination,missing,queries,per_link))
        def finish():
            pairs=[]
            for actor,object_id,source,destination,initial_missing,queries,per_link in planned:
                missing=list(initial_missing);evidence={}
                if not missing:
                    try:evidence={name:batch.result(q) for name,q in queries.items()}
                    except Exception as exc:missing=[dict(error=str(exc))]
                complete=not missing and bool(evidence)
                values={name:e['distance_m'] for name,e in evidence.items()} if complete else {}
                nearest=min(evidence.values(),key=lambda e:e['distance_m']) if complete else None
                pairs.append(dict(actor=actor,object_id=object_id,
                    distance_m=nearest['distance_m'] if nearest else None,
                    link_distances_m=values if per_link else {},source_distances_m=values,
                    nearest=nearest,link_distance_evidence=evidence if per_link else {},
                    distance_query_scope='per_link' if per_link else 'complete_actor_surface',
                    coverage_complete=complete,missing_geometry=missing,
                    distance_source=self.metadata['target_source'] if per_link else self.metadata['environment_source'],
                    metric=self.metadata['metric'],source_collider_count=len(source),destination_collider_count=len(destination)))
            payload=[] if any(e['group']==self.target for e in errors) else groups.get(self.target,[])
            center=((np.min([e['lo'] for e in payload],axis=0)+np.max([e['hi'] for e in payload],axis=0))*.5).tolist() if payload else None
            return dict(version=self.metadata['version'],metric=self.metadata['metric'],pairs=pairs,errors=errors,
                        payload_geometry_center_world_m=center,complete=all(p['coverage_complete'] for p in pairs))
        return finish

    def _sample_scalar(self, body_poses, object_poses):
        groups, errors = {}, list(self.errors)
        for entry in self.entries:
            try:
                if entry['dynamic']:
                    poses = body_poses if entry['group'] == 'robot' else object_poses
                    if entry['key'] not in poses:
                        raise ValueError('Unmeasured rigid actor: ' + str(entry['actor']))
                    pose = transform(poses[entry['key']])
                else:
                    pose = np.eye(4)
                if entry['last_pose'] is None or not np.array_equal(pose, entry['last_pose']):
                    entry['object'].setTransform(self.fcl.Transform(pose[:3, :3], pose[:3, 3]))
                    corners = entry['corners'] @ pose[:3, :3].T + pose[:3, 3]
                    entry['lo'], entry['hi'] = corners.min(0), corners.max(0)
                    entry['last_pose'] = pose.copy()
                groups.setdefault(entry['group'], []).append(entry)
            except Exception as exc:
                errors.append(dict(path=entry['path'], group=entry['group'], error=str(exc)))
        links = {}
        for entry in groups.get('robot', []):
            links.setdefault(entry['actor'], []).append(entry)
        pairs, query_cache = [], {}
        for object_id in self.assets:
            destination = groups.get(object_id, [])
            for actor in (('robot',) if object_id == self.target else ('robot', 'held_object')):
                if (actor, object_id) in self.omitted_distance_relations:continue
                source_group = 'robot' if actor == 'robot' else self.target
                source = groups.get(source_group, [])
                missing = [e for e in errors if e['group'] in (source_group, object_id)]
                if not source or not destination:
                    missing = missing + [dict(error='Empty declared source or destination geometry')]
                evidence = {}
                if not missing:
                    try:
                        # Only robot-to-target rules and grasp evidence need every link.
                        # Environment rules consume min over the complete robot surface:
                        # min_link(min_collider(distance)) == min_all_colliders(distance).
                        # One global query can prune links unable to win, while returning
                        # the same Warp CUDA minimum and nearest collider/points.
                        per_link = actor == 'robot' and object_id == self.target
                        query_groups = (links.items() if per_link else
                            [(self.root+('/Robot' if actor == 'robot' else '/'+self.target), source)])
                        evidence = {name:minimum_distance(self.fcl, entries, destination, query_cache)
                            for name, entries in query_groups}
                    except Exception as exc:
                        missing = [dict(error=str(exc))]
                        evidence = {}
                complete = not missing and bool(evidence)
                values = {name:e['distance_m'] for name,e in evidence.items()} if complete else {}
                nearest = min(evidence.values(), key=lambda e:e['distance_m']) if complete else None
                pairs.append(dict(actor=actor, object_id=object_id,
                    distance_m=nearest['distance_m'] if nearest else None,
                    link_distances_m=values if actor == 'robot' and object_id == self.target else {}, source_distances_m=values,
                    nearest=nearest, link_distance_evidence=evidence if actor == 'robot' and object_id == self.target else {},
                    distance_query_scope='per_link' if actor == 'robot' and object_id == self.target else 'complete_actor_surface',
                    coverage_complete=complete, missing_geometry=missing,
                    distance_source=self.metadata['target_source'] if actor == 'robot' and object_id == self.target else self.metadata['environment_source'],
                    metric=self.metadata['metric'], source_collider_count=len(source), destination_collider_count=len(destination)))
        payload = [] if any(e['group'] == self.target for e in errors) else groups.get(self.target, [])
        center = ((np.min([e['lo'] for e in payload], axis=0)+np.max([e['hi'] for e in payload], axis=0))*.5).tolist() if payload else None
        return dict(version=self.metadata['version'], metric=self.metadata['metric'], pairs=pairs, errors=errors,
                    payload_geometry_center_world_m=center,
                    complete=all(p['coverage_complete'] for p in pairs))
