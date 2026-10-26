"""Conservative per-source-collider convex Minkowski outer-sphere proxy, task-local and opt-in."""
import itertools
import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection

POLICY = 'source_convex_outer_euclidean_margin26_v1'

def rounded_outer_aabb(lo, hi, margin, source_vertices=None):
    """Contains AABB plus a Euclidean margin ball; no clearance is reduced.

    Every tangent halfspace contains the entire radius-margin ball. Their
    intersection is an OUTER approximation, and Minkowski addition preserves
    containment. Additional 1um protects floating-point triangulation.
    """
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    margin = float(margin)
    if lo.shape != (3,) or hi.shape != (3,) or np.any(hi < lo) or margin <= 0:
        raise ValueError('Expected valid AABB and strictly positive margin')
    normals = np.array([v for v in itertools.product([-1.,0.,1.],repeat=3) if any(v)])
    normals /= np.linalg.norm(normals,axis=1)[:,None]
    radius = margin + 1e-6
    poly = HalfspaceIntersection(np.c_[normals,np.full(26,-radius)],np.zeros(3)).intersections
    corners = np.array(list(itertools.product(*zip(lo,hi)))) if source_vertices is None else np.asarray(source_vertices,float)
    vertices = (corners[:,None,:]+poly[None,:,:]).reshape(-1,3)
    hull = ConvexHull(vertices)
    faces = hull.simplices.copy()
    for i,f in enumerate(faces):
        tri = vertices[f]
        if np.dot(np.cross(tri[1]-tri[0],tri[2]-tri[0]),hull.equations[i,:3]) < 0:
            faces[i] = f[[0,2,1]]
    # Compact indices without changing the convex hull.
    used = np.unique(faces); remap = np.full(len(vertices),-1); remap[used] = np.arange(len(used))
    return vertices[used],remap[faces]

def replace_glass_boxes(planner):
    """Opt-in installation; keeps source FCL/PhysX and sphere model intact."""
    from curobo.geom.types import Mesh,WorldConfig
    if planner.use_cuda_graph:
        raise ValueError('This opt-in policy currently requires use_cuda_graph=False')
    checker=planner.motion_gen.world_coll_checker
    old=checker.world_model
    names={name for name,*_ in planner.world.objects if planner.world._matches(name,'glass')}
    meshes=list(old.mesh or [])
    boxes=[box for box in (old.cuboid or []) if box.name not in names]
    records={}
    for name,ob,lo,hi in planner.world.objects:
        if name not in names:continue
        margin=planner.world._margin(name,'planning')+planner.hazard_extra_padding_m
        vertices,faces=rounded_outer_aabb(lo,hi,margin,planner.world.meshes[name][0] if name in planner.world.meshes else None)
        vertices=vertices@planner.T_B_W[:3,:3].T+planner.T_B_W[:3,3]
        meshes.append(Mesh(name=name,pose=[0.,0.,0.,1.,0.,0.,0.],vertices=vertices.tolist(),faces=faces.tolist()))
        records[name]={'margin_m':margin,'vertices':len(vertices),'faces':len(faces),'reference_rotation':np.array(ob.getRotation()),'reference_translation':np.array(ob.getTranslation())}
    if not names:return {}
    planner.motion_gen.update_world(WorldConfig(mesh=meshes,cuboid=boxes))
    planner._dynamic_box_names -= names
    planner.collision_geometry['door_local_glass_proxy_policy']=POLICY
    planner.collision_geometry['door_local_glass_proxy_margin_m']={n:r['margin_m'] for n,r in records.items()}
    return records



def replace_person_meshes(planner, extra_margin_m):
    """Conservative optional person-only convex outer offset, never a source edit.

    This extra planning buffer helps the sphere optimizer account for the
    independently checked full gripper sweep. Original FCL planning/execution
    margins and source PhysX colliders remain mandatory and unchanged.
    """
    import hashlib
    from curobo.geom.types import Mesh, WorldConfig
    extra_margin_m = float(extra_margin_m)
    if not np.isfinite(extra_margin_m) or not 0. < extra_margin_m <= .03:
        raise ValueError('Person extra planning offset must be in (0,.03]m')
    if planner.use_cuda_graph:
        raise ValueError('Optional person proxy requires use_cuda_graph=False')
    checker = planner.motion_gen.world_coll_checker
    old = checker.world_model
    names = {name for name,*_ in planner.world.objects if planner.world._matches(name,'person')}
    if not names:
        return {}
    meshes = [mesh for mesh in (old.mesh or []) if mesh.name not in names]
    boxes = [box for box in (old.cuboid or []) if box.name not in names]
    records = {}
    for name,ob,lo,hi in planner.world.objects:
        if name not in names:
            continue
        source = planner.world.meshes.get(name)
        if source is None:
            raise ValueError('Person source mesh missing: '+name)
        vertices,faces = rounded_outer_aabb(lo,hi,extra_margin_m,source[0])
        vertices = vertices @ planner.T_B_W[:3,:3].T + planner.T_B_W[:3,3]
        # Warp caches mesh buffers by name; a new geometry needs a new key.
        gpu_name=name+'__doors_person_outer_'+hashlib.sha256(vertices.tobytes()+faces.tobytes()).hexdigest()[:12]
        meshes.append(Mesh(name=gpu_name,pose=[0.,0.,0.,1.,0.,0.,0.],vertices=vertices.tolist(),faces=faces.tolist()))
        records[name] = {'gpu_name':gpu_name,'extra_margin_m':extra_margin_m,'vertices':len(vertices),'faces':len(faces),
                         'reference_rotation':np.array(ob.getRotation()),'reference_translation':np.array(ob.getTranslation())}
    planner.motion_gen.update_world(WorldConfig(mesh=meshes,cuboid=boxes))
    planner.obstacle_names=[records[n]['gpu_name'] if n in records else n for n in planner.obstacle_names]
    planner.collision_geometry['door_local_person_proxy_gpu_names']={n:r['gpu_name'] for n,r in records.items()}
    planner.collision_geometry['door_local_person_proxy_policy']='per_source_collider_convex_outer_extra_v1'
    planner.collision_geometry['door_local_person_proxy_extra_m']=extra_margin_m
    planner.collision_geometry['door_local_person_proxy_scope']='Extra planner buffer only; original source FCL and PhysX unchanged; conservative convex hulls not exact cooked shapes.'
    return records
