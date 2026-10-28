"""Read-only, per-render-frame source geometry audit for the walking human demo.

The moving person's bounds come from the actual evaluated points at *every*
frame, never from a single authored USD extent. Floor support has a separate
check so a shoe resting on the floor is not rejected by the obstacle margin.
No USD layer is edited, and no simulation, planning or rendering is performed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

for _option in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_option] = "4"
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
import fcl
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from kitchen_safety.geometry_audit import minimum_distance

HUMAN = "/World/human/Asset"
DOOR = "/World/kitchen/Asset/fridge_main_group/Refrigerator032_fridge_door"
ROBOT = "/World/robot"
KITCHEN = "/World/kitchen"
FLOOR = "/World/kitchen/Asset/floor_room/geometry"
CUBE_FACES = np.asarray([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                         [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]], dtype=np.int32)
SCOPE = "Discrete actual source geometry at every rendered frame, including the walking human's evaluated full meshes. No native physics, policy, grasp, continuous swept collision or physical foot-contact acceptance."
AFFINE_CYLINDER_AUDITS = {}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative(path):
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(Path(path))


def enabled(prim, time):
    return prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(time) is not False


def visible(prim, time):
    imageable = UsdGeom.Imageable(prim)
    return imageable.ComputeVisibility(time) != UsdGeom.Tokens.invisible and imageable.ComputePurpose() not in (UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy)


def descendants(stage, path):
    root = stage.GetPrimAtPath(path)
    if not root:
        raise ValueError(f"Missing audit root: {path}")
    return Usd.PrimRange(root, Usd.TraverseInstanceProxies())


def world_vertices(prim, time, cache):
    if prim.IsA(UsdGeom.Mesh):
        vertices = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(time), dtype=float)
    elif prim.IsA(UsdGeom.Cube):
        half = float(UsdGeom.Cube(prim).GetSizeAttr().Get(time)) / 2.
        vertices = np.asarray([[x,y,z] for x in (-half,half) for y in (-half,half) for z in (-half,half)])
    else:
        raise ValueError(f"Not a source mesh/Cube: {prim.GetPath()}")
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        raise ValueError(f"Invalid source points: {prim.GetPath()}")
    matrix = np.asarray(cache.GetLocalToWorldTransform(prim))
    return vertices @ matrix[:3,:3] + matrix[3,:3]


def mesh_faces(prim, time):
    if prim.IsA(UsdGeom.Cube):
        return CUBE_FACES
    mesh = UsdGeom.Mesh(prim)
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(time), dtype=np.int32)
    holes = set(mesh.GetHoleIndicesAttr().Get(time) or [])
    faces, offset = [], 0
    for face_number, count in enumerate(mesh.GetFaceVertexCountsAttr().Get(time)):
        poly = indices[offset:offset+count]
        if face_number not in holes:
            faces.extend((poly[0],poly[index],poly[index+1]) for index in range(1,count-1))
        offset += count
    return np.asarray(faces, dtype=np.int32)


def surface_row(prim, time, cache, vertices=None):
    vertices = world_vertices(prim,time,cache) if vertices is None else vertices
    faces = mesh_faces(prim,time)
    model = fcl.BVHModel()
    model.beginModel(len(vertices),len(faces)); model.addSubModel(vertices,faces); model.endModel()
    return (str(prim.GetPath()),fcl.CollisionObject(model),vertices.min(0),vertices.max(0))


def round_row(prim, time, cache):
    """Preserve analytical round primitives; never replace them with AABBs."""
    matrix = np.asarray(cache.GetLocalToWorldTransform(prim)).T
    center = matrix[:3,3]
    scale = np.linalg.norm(matrix[:3,:3],axis=0)
    rotation = matrix[:3,:3] / scale
    if not np.allclose(rotation.T @ rotation,np.eye(3),atol=1e-7) or np.linalg.det(rotation)<0:
        if prim.IsA(UsdGeom.Cylinder):
            return affine_cylinder_row(prim,time,cache)
        raise ValueError(f"Sheared/reflected round collider requires explicit audit support: {prim.GetPath()}")
    if prim.IsA(UsdGeom.Sphere):
        if not np.allclose(scale,scale[0],atol=1e-8,rtol=1e-7):
            raise ValueError(f"Ellipsoid is unsupported: {prim.GetPath()}")
        radius = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get(time))*scale[0]
        model, half = fcl.Sphere(radius), np.full(3,radius)
    elif prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Capsule):
        schema = UsdGeom.Cylinder(prim) if prim.IsA(UsdGeom.Cylinder) else UsdGeom.Capsule(prim)
        axis = "XYZ".index(str(schema.GetAxisAttr().Get(time)))
        perpendicular = [i for i in range(3) if i != axis]
        if not np.isclose(scale[perpendicular[0]],scale[perpendicular[1]],atol=1e-8,rtol=1e-7):
            if prim.IsA(UsdGeom.Cylinder):
                return affine_cylinder_row(prim,time,cache)
            raise ValueError(f"Elliptical round collider is unsupported: {prim.GetPath()}")
        radius = float(schema.GetRadiusAttr().Get(time))*scale[perpendicular[0]]
        height = float(schema.GetHeightAttr().Get(time))*scale[axis]
        direction = rotation[:,axis].copy()
        align = np.eye(3) if axis==2 else Rotation.from_euler("y" if axis==0 else "x",90 if axis==0 else -90,degrees=True).as_matrix()
        rotation = rotation @ align
        if prim.IsA(UsdGeom.Cylinder):
            model = fcl.Cylinder(radius,height)
            half = radius*np.sqrt(np.maximum(0.,1.-direction**2))+height/2.*np.abs(direction)
        else:
            if not np.allclose(scale,scale[0],atol=1e-8,rtol=1e-7):
                raise ValueError(f"Anisotropic capsule is unsupported: {prim.GetPath()}")
            model = fcl.Capsule(radius,height)
            half = radius+height/2.*np.abs(direction)
    else:
        raise ValueError(f"Unsupported enabled collision geometry: {prim.GetPath()} ({prim.GetTypeName()})")
    return (str(prim.GetPath()),fcl.CollisionObject(model,fcl.Transform(rotation,center)),center-half,center+half)


def affine_cylinder_row(prim, time, cache):
    """Conservative tangent 256-gon for a source cylinder with affine shear.

    The source circle is contained in the polygon before and after the actual
    affine transform. Its maximum radial enlargement is bounded and reported;
    this is not silently treated as an exact analytical cylinder or an AABB.
    """
    schema=UsdGeom.Cylinder(prim);segments=256
    axis="XYZ".index(str(schema.GetAxisAttr().Get(time)))
    perpendicular=[i for i in range(3) if i!=axis]
    radius=float(schema.GetRadiusAttr().Get(time));height=float(schema.GetHeightAttr().Get(time))
    inflation=1./np.cos(np.pi/segments)
    angles=(np.arange(segments)+.5)*2.*np.pi/segments
    vertices=np.zeros((segments*2,3))
    for ring,z in enumerate((-height/2.,height/2.)):
        sl=slice(ring*segments,(ring+1)*segments)
        vertices[sl,axis]=z
        vertices[sl,perpendicular[0]]=radius*inflation*np.cos(angles)
        vertices[sl,perpendicular[1]]=radius*inflation*np.sin(angles)
    faces=[]
    for index in range(segments):
        nxt=(index+1)%segments
        faces.extend(((index,nxt,nxt+segments),(index,nxt+segments,index+segments)))
    faces.extend((0,index+1,index) for index in range(1,segments-1))
    faces.extend((segments,segments+index,segments+index+1) for index in range(1,segments-1))
    matrix=np.asarray(cache.GetLocalToWorldTransform(prim))
    vertices=vertices@matrix[:3,:3]+matrix[3,:3]
    model=fcl.BVHModel();model.beginModel(len(vertices),len(faces));model.addSubModel(vertices,np.asarray(faces,dtype=np.int32));model.endModel()
    error=radius*(inflation-1.)*float(np.linalg.norm(matrix[:3,:3],ord=2))
    AFFINE_CYLINDER_AUDITS[str(prim.GetPath())]={"segments":segments,"circumscribed":True,"maximum_world_radial_enlargement_m":error}
    return (str(prim.GetPath()),fcl.CollisionObject(model),vertices.min(0),vertices.max(0))


def source_row(prim, time, cache):
    return surface_row(prim,time,cache) if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube) else round_row(prim,time,cache)


def source_bounds(prim, time, cache):
    if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube):
        vertices = world_vertices(prim,time,cache)
        return vertices.min(0),vertices.max(0)
    # Analytic affine AABBs are only a conservative broad phase, including
    # sheared cylinders outside the walking person's possible motion region.
    matrix = np.asarray(cache.GetLocalToWorldTransform(prim))
    linear, center = matrix[:3,:3], matrix[3,:3]
    if prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Capsule):
        schema = UsdGeom.Cylinder(prim) if prim.IsA(UsdGeom.Cylinder) else UsdGeom.Capsule(prim)
        axis = "XYZ".index(str(schema.GetAxisAttr().Get(time)))
        perpendicular = [i for i in range(3) if i!=axis]
        radius, height = float(schema.GetRadiusAttr().Get(time)),float(schema.GetHeightAttr().Get(time))
        radial = np.linalg.norm(linear[perpendicular],axis=0) if prim.IsA(UsdGeom.Cylinder) else np.linalg.norm(linear,axis=0)
        half = radius*radial + height/2.*np.abs(linear[axis])
    elif prim.IsA(UsdGeom.Sphere):
        half = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get(time))*np.linalg.norm(linear,axis=0)
    else:
        raise ValueError(f"Cannot safely bound collision geometry: {prim.GetPath()}")
    return center-half,center+half


def human_meshes(stage, time):
    meshes = [p for p in descendants(stage,HUMAN) if p.IsA(UsdGeom.Mesh) and visible(p,time)]
    if not meshes or any(not enabled(p,time) for p in meshes):
        raise ValueError("Every visible full-person source mesh must retain an enabled collision API")
    return meshes


def assert_static(stage, path, excluded_prefix=None):
    for prim in descendants(stage,path):
        if excluded_prefix and str(prim.GetPath()).startswith(excluded_prefix):
            continue
        for attr in prim.GetAttributes():
            if attr.GetNumTimeSamples():
                raise ValueError(f"Expected stationary geometry outside door: {attr.GetPath()}")


def distance(left, right, margin):
    if not right:
        return {"valid":True,"intersecting":False,"distance_m":None,"required_m":margin,
                "scope":"No static kitchen collider survived conservative full-motion AABB screening."}
    return minimum_distance(left,right,margin)


def ground_geometry(stage, time, cache):
    floor = stage.GetPrimAtPath(FLOOR)
    if not floor or not floor.IsA(UsdGeom.Cube) or not enabled(floor,time):
        raise ValueError("Expected enabled original room floor Cube")
    vertices = world_vertices(floor,time,cache)
    top_z = float(vertices[:,2].max())
    top = vertices[np.abs(vertices[:,2]-top_z)<1e-6]
    if len(top)!=4:
        raise ValueError("Floor must have a planar horizontal top face for support audit")
    return top_z,ConvexHull(top[:,:2]).equations,top.tolist()


def ground_check(mesh_vertices, floor_z, polygon_equations, tolerance):
    rows=[];shoe_rows=[]
    for path,vertices in mesh_vertices.items():
        is_shoe="shoe" in path.lower()
        low=float(vertices[:,2].min())
        sole=vertices[vertices[:,2]<=low+.012]
        on_floor_polygon=bool(np.all(sole[:,:2]@polygon_equations[:,:2].T+polygon_equations[:,2]<=1e-6))
        row={"mesh":path,"shoe":is_shoe,"minimum_z_m":low,"clearance_to_floor_top_m":low-floor_z,
             "penetration_below_floor_top_m":max(0.,floor_z-low),"low_vertices_over_actual_floor_polygon":on_floor_polygon}
        rows.append(row)
        if is_shoe:shoe_rows.append(row)
    if not shoe_rows:
        raise ValueError("Cannot identify full-person shoe mesh for floor-support audit")
    max_penetration=max(row["penetration_below_floor_top_m"] for row in rows)
    return {"valid":bool(max_penetration<=tolerance and all(row["low_vertices_over_actual_floor_polygon"] for row in shoe_rows)),
            "method":"Actual time-evaluated full mesh vertex Z vs actual original horizontal floor top; shoe sole XY checked against its true convex top polygon.",
            "shoe_contact_allowed":True,"allowed_numerical_penetration_m":tolerance,
            "maximum_penetration_m":max_penetration,"minimum_shoe_clearance_m":min(row["clearance_to_floor_top_m"] for row in shoe_rows),
            "any_shoe_within_12mm_of_floor":any(abs(row["clearance_to_floor_top_m"])<=.012 for row in shoe_rows),
            "meshes":rows,"native_foot_contact_or_gait_stability_verified":False}


def run(scene, timeline_path, output, pause_frame=72, future_frame=167, margin=.002, ground_tolerance=.002):
    AFFINE_CYLINDER_AUDITS.clear()
    scene,timeline_path,output=map(Path,(scene,timeline_path,output))
    scene_hash,timeline_hash=sha(scene),sha(timeline_path)
    timeline=json.loads(timeline_path.read_text());stage=Usd.Stage.Open(str(scene.resolve()))
    if not stage or stage.GetCompositionErrors():
        raise ValueError("Cannot audit incomplete USD composition")
    input_layers={layer.realPath:sha(layer.realPath) for layer in stage.GetUsedLayers() if layer.realPath}
    frames=int(timeline["frames"]);fps=float(timeline["fps"])
    if not 0<=pause_frame<frames or not 0<=future_frame<frames:
        raise ValueError("Counterfactual frames lie outside timeline")
    times=[stage.GetStartTimeCode()+i*stage.GetTimeCodesPerSecond()/fps for i in range(frames)]
    assert_static(stage,ROBOT);assert_static(stage,KITCHEN,DOOR)
    first_time=Usd.TimeCode(times[0]);first_cache=UsdGeom.XformCache(first_time)
    robot=[]
    for prim in descendants(stage,ROBOT):
        if not prim.IsA(UsdGeom.Gprim):continue
        if enabled(prim,first_time) or (prim.IsA(UsdGeom.Mesh) and visible(prim,first_time)):
            robot.append(source_row(prim,first_time,first_cache))
    if not robot:raise ValueError("No original robot geometry")
    floor_z,floor_polygon,floor_top=ground_geometry(stage,first_time,first_cache)
    bounds=[]
    for index,time in enumerate(times):
        code=Usd.TimeCode(time);cache=UsdGeom.XformCache(code)
        values=[world_vertices(p,code,cache) for p in human_meshes(stage,code)]
        low=np.min([v.min(0) for v in values],axis=0);high=np.max([v.max(0) for v in values],axis=0)
        bounds.append({"frame":index,"low":low.tolist(),"high":high.tolist()})
    full_low=np.min([row["low"] for row in bounds],axis=0);full_high=np.max([row["high"] for row in bounds],axis=0)
    screening_margin=margin+.003;kitchen=[];candidate_paths=[];screened=0
    for prim in descendants(stage,KITCHEN):
        path=str(prim.GetPath())
        if path.startswith(DOOR) or path==FLOOR or not prim.IsA(UsdGeom.Gprim) or not enabled(prim,first_time):continue
        screened+=1;low,high=source_bounds(prim,first_time,first_cache)
        separation=float(np.linalg.norm(np.maximum(np.maximum(low-full_high,full_low-high),0.)))
        if separation<=screening_margin:
            kitchen.append(source_row(prim,first_time,first_cache));candidate_paths.append(path)
    samples_by_frame={int(row["frame"]):row for row in timeline.get("samples",[])}
    rows=[];pause_human=None
    for index,time in enumerate(times):
        code=Usd.TimeCode(time);cache=UsdGeom.XformCache(code)
        mesh_vertices={str(p.GetPath()):world_vertices(p,code,cache) for p in human_meshes(stage,code)}
        human=[surface_row(stage.GetPrimAtPath(path),code,cache,vertices) for path,vertices in mesh_vertices.items()]
        door=[source_row(p,code,cache) for p in descendants(stage,DOOR) if p.IsA(UsdGeom.Gprim) and enabled(p,code)]
        if not door:raise ValueError("No original door collision geometry")
        sample=samples_by_frame.get(index,{})
        phase=sample.get("phase")
        if phase is None:
            phase=next((p.get("phase",p.get("label_zh")) for p in timeline.get("phases",[]) if p["start_seconds"]<=index/fps<p["end_seconds"]),"unspecified")
        row={"frame":index,"time_seconds":index/fps,"usd_time_code":time,"phase":phase,
             "human_actual_world_bounds":bounds[index],"human_to_current_door":distance(human,door,margin),
             "human_to_robot":distance(human,robot,margin),"door_to_stationary_robot":distance(door,robot,margin),
             "human_to_surrounding_kitchen":distance(human,kitchen,margin),
             "ground_support":ground_check(mesh_vertices,floor_z,floor_polygon,ground_tolerance)}
        if "door_angle_deg" in sample:row["door_angle_deg"]=sample["door_angle_deg"]
        rows.append(row)
        if index==pause_frame:pause_human=human
    future_code=Usd.TimeCode(times[future_frame]);future_cache=UsdGeom.XformCache(future_code)
    future_door=[source_row(p,future_code,future_cache) for p in descendants(stage,DOOR) if p.IsA(UsdGeom.Gprim) and enabled(p,future_code)]
    future=distance(pause_human,future_door,margin)
    predicates={"current_door_and_human_clear_at_sampled_frames":all(r["human_to_current_door"]["valid"] for r in rows),
                "human_clear_of_stationary_robot":all(r["human_to_robot"]["valid"] for r in rows),
                "human_clear_of_surrounding_kitchen":all(r["human_to_surrounding_kitchen"]["valid"] for r in rows),
                "door_clear_of_stationary_robot":all(r["door_to_stationary_robot"]["valid"] for r in rows),
                "ground_support_geometry_within_tolerance":all(r["ground_support"]["valid"] for r in rows),
                "future_door_would_intersect_pause_human":bool(future["intersecting"])}
    failures={name:[r["frame"] for r in rows if not r[name]["valid"]] for name in ("human_to_current_door","human_to_robot","human_to_surrounding_kitchen","door_to_stationary_robot","ground_support")}
    report={"scope":SCOPE,"scene":relative(scene),"scene_sha256":scene_hash,"timeline":relative(timeline_path),"timeline_sha256":timeline_hash,
            "input_layers_sha256":{relative(path):value for path,value in input_layers.items()},
            "sample_count":frames,"fps":fps,"all_render_frames_checked":True,"required_obstacle_clearance_m":margin,
            "human_motion_bounds_method":"Union of actual world-transformed full source mesh vertices independently evaluated at every rendered frame; authored USD extents are not used.",
            "human_full_motion_bounds":{"low":full_low.tolist(),"high":full_high.tolist()},
            "kitchen_broad_phase":{"static_colliders_screened":screened,"margin_m":screening_margin,"narrow_phase_candidates":candidate_paths,"floor_checked_separately":FLOOR},
            "source_geometry_method":"Exact world-transformed original Mesh/Cube triangle surfaces preserving affine scale/shear; analytical round primitives when their transform supports exact representation; sheared/elliptical source cylinders use conservative circumscribed 256-gon surfaces with explicitly bounded enlargement. AABBs only prune pairs, never replace narrow-phase geometry.",
            "affine_source_cylinder_approximations":dict(AFFINE_CYLINDER_AUDITS),
            "original_floor_top_vertices_m":floor_top,"shoe_support_policy":{"touching_original_floor_allowed":True,"maximum_numerical_penetration_m":ground_tolerance,"deeper_penetration_fails":True,"feet_above_floor_during_swing_are_not_penetration":True},
            "samples":rows,"counterfactual":{"pause_human_frame":pause_frame,"future_door_frame":future_frame,"result":future},
            "checks":predicates,"failed_frames":failures,"all_discrete_geometry_checks_passed":all(predicates.values()),
            "frames_without_shoe_within_12mm_of_floor":[r["frame"] for r in rows if not r["ground_support"]["any_shoe_within_12mm_of_floor"]],
            "no_simulation_started":True,"no_robot_trajectory_generated":True,"native_physics_verified":False,
            "limitations":["Discrete source-surface checks are not continuous swept-volume proof or PhysX cooked-contact acceptance.","The 2 mm floor tolerance is a numerical geometry allowance, not verified physical support; gait stability is not evaluated.","Mesh surface queries do not supply a general closed-volume containment proof."]}
    if sha(scene)!=scene_hash or sha(timeline_path)!=timeline_hash or any(sha(path)!=value for path,value in input_layers.items()):
        raise RuntimeError("Source scene/timeline/referenced USD changed during audit; no stale report written")
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n")
    print(json.dumps({"scene":relative(scene),"sample_count":frames,"checks":predicates,"failed_frames":failures,"counterfactual":report["counterfactual"]},indent=2,ensure_ascii=False))
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene",type=Path,default=ROOT/"assets/extensions/doors/hard_demo_v2/scene.usda")
    parser.add_argument("--timeline",type=Path,default=ROOT/"reports/doors_hard_demo_v2/timeline.json")
    parser.add_argument("--output",type=Path,default=ROOT/"reports/doors_hard_demo_v2/geometry.json")
    parser.add_argument("--pause-frame",type=int,default=72)
    parser.add_argument("--future-frame",type=int,default=167)
    parser.add_argument("--margin",type=float,default=.002)
    parser.add_argument("--ground-tolerance",type=float,default=.002)
    args=parser.parse_args()
    run(args.scene,args.timeline,args.output,args.pause_frame,args.future_frame,args.margin,args.ground_tolerance)
