#!/usr/bin/env python3
"""CPU continuous source-domain exclusion and conservative gripper reach bound.

Uses inward subsets of verified watertight convex SOURCE triangle meshes.
It never uses an outer AABB/hull proxy to declare a source collision. Fixed
source yaw and every allowed source base Z are included. Result scope is exact
source geometry, not a reconstruction of PhysX cooked/contact-offset shapes.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import numpy as np
import trimesh
from pxr import Usd,UsdGeom,UsdPhysics
from scipy.spatial import ConvexHull,cKDTree
from shapely.geometry import Polygon,MultiPoint,Point,box,mapping
from shapely.ops import unary_union,nearest_points
from shapely.affinity import translate
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry,code_evidence
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.search_l_island_bowl_domain import source_position_domain,reach_ball
from scripts.extensions.replicator.source_domain_rounding import float32_outer_domain
from scripts.extensions.replicator.audit_source_layout_geometry import triangles,STAND,sha

INSET_M=1e-5


def convex_source_prism(name,vertices,faces,zlow,zhigh):
    """XY core whose full [zlow,zhigh] prism lies strictly inside source solid."""
    mesh=trimesh.Trimesh(vertices,faces,process=True);mesh.fix_normals()
    merge_error=float(cKDTree(np.asarray(mesh.vertices)).query(vertices)[0].max())
    hull=ConvexHull(vertices)
    if not mesh.is_watertight or not mesh.is_convex or merge_error>1e-12:
        raise ValueError('Cannot certify source convex solid: '+name)
    if abs(mesh.volume-hull.volume)>1e-9 or abs(mesh.area-hull.area)>1e-8:
        raise ValueError('Source triangles differ from convex boundary: '+name)
    low,high=vertices.min(0),vertices.max(0)
    polygon=np.array([[low[0],low[1]],[high[0],low[1]],[high[0],high[1]],[low[0],high[1]]])
    for plane in hull.equations:
        normal=plane[:2]
        limit=-plane[3]-max(plane[2]*zlow,plane[2]*zhigh)-INSET_M
        output=[]
        for p,q in zip(polygon,np.roll(polygon,-1,axis=0)):
            a=float(p@normal-limit);b=float(q@normal-limit)
            if a<=0:output.append(p)
            if (a<0 and b>0) or (a>0 and b<0):output.append(p+(q-p)*(a/(a-b)))
        polygon=np.asarray(output)
        if len(polygon)<3:raise ValueError('Empty guaranteed source interior prism: '+name)
    polygon=Polygon(polygon).convex_hull
    xy=np.asarray(polygon.exterior.coords)[:-1]
    corners=np.r_[np.c_[xy,np.full(len(xy),zlow)],np.c_[xy,np.full(len(xy),zhigh)]]
    maximum=float((corners@hull.equations[:,:3].T+hull.equations[:,3]).max())
    if maximum>-INSET_M+1e-12:raise ValueError('Prism containment failed: '+name)
    return polygon,dict(path=name,original_vertices=len(vertices),original_triangles=len(faces),
        merged_vertices=len(mesh.vertices),source_watertight=bool(mesh.is_watertight),source_convex=bool(mesh.is_convex),
        exact_merge_max_error_m=merge_error,source_volume_m3=float(mesh.volume),source_area_m2=float(mesh.area),
        hull_volume_m3=float(hull.volume),hull_area_m2=float(hull.area),interior_plane_max_signed_m=maximum,
        inward_proof_inset_m=INSET_M,z_interval_m=[zlow,zhigh],inner_xy_polygon=mapping(polygon),
        representation='Verified closed convex original triangle boundary; convex hull planes describe the same solid. Inner prism is a strict subset, never an outer collision proxy.')


def all_gripper_reach_bound(world,robot):
    kin=world.kin
    segment_lengths=[float(np.linalg.norm((kin.chain[i][1]@kin.chain[i+1][0])[:3,3])) for i in range(6)]
    tail=kin.chain[6][1].copy()
    for a,b,_ in kin.chain[7:]:tail=tail@a@b
    chains={world.gripper_root:(sum(segment_lengths),tail,[])};pending=world.gripper_joints.copy()
    while pending:
        count=len(pending)
        for joint in pending.copy():
            name,parent,child,a,b,axis=joint
            if parent not in chains:continue
            length,transform,path=chains[parent];transform=transform@a
            if axis:
                segment=float(np.linalg.norm(transform[:3,3]))
                chains[child]=(length+segment,b,path+[dict(joint=name,axis=axis,translation_length_m=segment)])
            else:chains[child]=(length,transform@b,path+[dict(joint=name,axis=None)])
            pending.remove(joint)
        if count==len(pending):raise ValueError('Disconnected source gripper joint tree')
    cache=UsdGeom.XformCache();parts=[]
    for name,body,_,inverse,*_ in robot.parts:
        if body not in chains:continue
        prim=robot.stage.GetPrimAtPath(name)
        if not prim.IsA(UsdGeom.Mesh):raise ValueError('Unhandled source gripper collider in reach proof')
        vertices=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        local=inverse@np.asarray(cache.GetLocalToWorldTransform(prim)).T
        vertices=vertices@local[:3,:3].T+local[:3,3]
        length,tail,path=chains[body]
        points=vertices@tail[:3,:3].T+tail[:3,3]
        terminal=float(np.linalg.norm(points,axis=1).max())
        parts.append(dict(path=name,body=body,source_vertices=len(vertices),joint_path=path,
            shoulder_to_last_joint_length_bound_m=length,last_joint_to_farthest_source_vertex_m=terminal,
            radius_upper_bound_m=length+terminal))
    if not parts:raise ValueError('No source gripper geometry')
    return dict(radius_upper_bound_m=max(row['radius_upper_bound_m'] for row in parts),parts=parts,
        arm_joint_center_segment_lengths_m=segment_lengths,
        method='Source FK joint-frame translation lengths plus terminal distance to every original enabled gripper collision vertex. Triangle inequality permits all revolute angles independently, even outside their limits and mimic coupling, so the ball is an outer bound on every feasible gripper surface/convex-hull point. No sampled workspace is used as an impossibility proof.')


def all_robot_reach_bound(world,robot):
    """Independent source joint-graph bound including arm/base/link8 colliders."""
    from kitchen_safety.planning import transform
    def local(joint,side):
        quat=getattr(joint,'GetLocalRot'+side+'Attr')().Get()
        return transform(getattr(joint,'GetLocalPos'+side+'Attr')().Get(),[*quat.GetImaginary(),quat.GetReal()])
    chains={'/panda/panda_link0':(0.,np.linalg.inv(world.kin.chain[0][0]),[])}
    pending=[];excluded=[]
    for prim in robot.stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):continue
        joint=UsdPhysics.Joint(prim);parents=joint.GetBody0Rel().GetTargets();children=joint.GetBody1Rel().GetTargets()
        if not parents or not children:
            if str(prim.GetPath())!='/panda/rootJoint':raise ValueError('Unexpected world attachment in source robot')
            excluded.append(str(prim.GetPath()));continue
        if len(parents)!=1 or len(children)!=1:raise ValueError('Ambiguous source joint tree')
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.FixedJoint)):
            raise ValueError('Unsupported robot joint in source reach bound')
        pending.append((str(prim.GetPath()),str(parents[0]),str(children[0]),local(joint,'0'),np.linalg.inv(local(joint,'1')),prim.IsA(UsdPhysics.RevoluteJoint)))
    while pending:
        count=len(pending)
        for item in pending.copy():
            name,parent,child,a,b,revolute=item
            if parent not in chains:continue
            length,tail,path=chains[parent];joint_frame=tail@a
            segment=float(np.linalg.norm(joint_frame[:3,3])) if revolute else 0.
            if child in chains:raise ValueError('Source joint graph is not a tree')
            chains[child]=(length+segment,b if revolute else joint_frame@b,
                path+[dict(joint=name,revolute=bool(revolute),segment_length_m=segment)])
            pending.remove(item)
        if len(pending)==count:raise ValueError('Disconnected source joint graph')
    cache=UsdGeom.XformCache();bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],useExtentsHint=False)
    parts=[]
    for name,body,_,inverse,*_ in robot.parts:
        if body not in chains:raise ValueError('Unbounded source collision body '+body)
        prim=robot.stage.GetPrimAtPath(name)
        if prim.IsA(UsdGeom.Mesh):
            source=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
            matrix=inverse@np.asarray(cache.GetLocalToWorldTransform(prim)).T
            vertices=source@matrix[:3,:3].T+matrix[:3,3]
            representation='Every actual source collision-mesh vertex'
        else:
            bound=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            lo,hi=np.asarray(bound.GetMin()),np.asarray(bound.GetMax())
            vertices=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
            vertices=vertices@inverse[:3,:3].T+inverse[:3,3]
            representation='Source primitive world bounding-box corners, a conservative outer enclosure for reach only'
        length,tail,path=chains[body]
        points=vertices@tail[:3,:3].T+tail[:3,3]
        terminal=float(np.linalg.norm(points,axis=1).max())
        parts.append(dict(path=name,body=body,representation=representation,source_points=len(vertices),
            joint_path=path,joint_center_length_bound_m=length,terminal_radius_bound_m=terminal,
            radius_upper_bound_m=length+terminal))
    if len(parts)!=len(robot.parts):raise ValueError('Incomplete source collision inventory')
    return dict(radius_upper_bound_m=max(row['radius_upper_bound_m'] for row in parts),parts=parts,
        collision_parts_checked=len(parts),excluded_world_attachments=excluded,
        method='Complete original robot joint graph rooted at first-arm shoulder. Triangle inequality at every unrestricted revolute joint, exact transforms at fixed joints, and every source collision vertex or primitive outer bound. Covers arm, base, palm and all fingers; limits/mimic constraints only shrink the set. Fixed stand is checked separately by a vertical separation bound.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-report-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    config=ROOT/'configs/extensions/replicator/l_island_bowl_sink_source_layout.json'
    layout=json.loads(config.read_text());scene=CandidateSearch(layout)
    base0=np.asarray(layout['robot_position'])+[0,0,layout['stand_height']]
    domain=float32_outer_domain(scene,'droid');bowl_domain=float32_outer_domain(scene,'bowl')
    low,high=np.asarray(domain['origin_xyz_lower']),np.asarray(domain['origin_xyz_upper'])
    z_world=[.109,.111]
    stand_mesh=triangles(Usd.Stage.Open(str(ROOT/STAND)),'/panda/panda_link0/stand_instanceable')
    if len(stand_mesh)!=1:raise ValueError('Proof requires the observed single source stand collider')
    name,_,faces=stand_mesh[0]
    stand_vertices=scene.shapes['droid'].vertices-base0
    stand,stand_info=convex_source_prism(name,stand_vertices,faces,z_world[0]-high[2],z_world[1]-low[2])
    # The same inner stand XY core occupies every world Z in z_world for EVERY
    # allowed base Z, including the closure of the original half-open interval.
    constraints=[];forbidden=[]
    for name,vertices,faces in scene.meshes:
        if '/Frame/Bottom' not in name or not any(s in name for s in ('Base_north','IslandBase')):continue
        core,info=convex_source_prism(name,vertices,faces,*z_world)
        a=np.asarray(core.exterior.coords)[:-1];b=np.asarray(stand.exterior.coords)[:-1]
        forbidden_poly=MultiPoint((a[:,None,:]-b[None,:,:]).reshape(-1,2)).convex_hull
        constraints.append(dict(source=info,forbidden_base_xy=mapping(forbidden_poly)))
        forbidden.append(forbidden_poly)
    if len(forbidden)!=8:raise ValueError('Source blocker inventory changed')
    complete_domain=box(low[0],low[1],high[0],high[1])
    blocked=unary_union(forbidden);remaining=complete_domain.difference(blocked)
    if remaining.is_empty:raise ValueError('Unexpected empty superset; retain source evidence rather than guess')
    world=CollisionWorld(Usd.Stage.CreateInMemory(),DroidKinematics(base0,layout['robot_yaw_deg']))
    robot=SourceRobotGeometry(world);contact=all_robot_reach_bound(world,robot)
    shoulder_offset=(world.kin.base@world.kin.chain[0][0])[:3,3]-base0
    possible_shoulder_xy=translate(remaining,xoff=shoulder_offset[0],yoff=shoulder_offset[1])
    # Original relation boxes constrain the root origin. The contact envelope
    # must independently contain actual COLLISION vertices, including geometry
    # that an Arena render/placement bbox could omit.
    bowl_collision=scene.shapes['bowl'].vertices-np.asarray(layout['objects']['bowl']['position'])
    bowl_collision_low,bowl_collision_high=bowl_collision.min(0),bowl_collision.max(0)
    blo=np.array(bowl_domain['origin_xyz_lower'])+bowl_collision_low
    bhi=np.array(bowl_domain['origin_xyz_upper'])+bowl_collision_high
    bowl_xy=box(blo[0],blo[1],bhi[0],bhi[1])
    xy_distance=float(possible_shoulder_xy.distance(bowl_xy));near_a,near_b=nearest_points(possible_shoulder_xy,bowl_xy)
    shoulder_z=[float(low[2]+shoulder_offset[2]),float(high[2]+shoulder_offset[2])]
    z_distance=max(float(blo[2]-shoulder_z[1]),float(shoulder_z[0]-bhi[2]),0.)
    separation=float(np.hypot(xy_distance,z_distance));gap=separation-contact['radius_upper_bound_m']
    stand_z_upper=float(high[2]+stand_vertices[:,2].max())
    stand_bowl_z_gap=float(blo[2]-stand_z_upper)
    if stand_bowl_z_gap<=0:raise ValueError('Fixed stand not proved vertically separate from every legal bowl')
    # This is the original default initialization profile, not every yaw that
    # an independently customized placer might admit.
    import ast
    placer=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/object_placer_params.py'
    definitions=[n for n in ast.walk(ast.parse(placer.read_text())) if isinstance(n,ast.AnnAssign)
                 and isinstance(n.target,ast.Name) and n.target.id=='random_yaw_init']
    if len(definitions)!=1 or ast.literal_eval(definitions[0].value) is not False:
        raise ValueError('Original default yaw profile changed')
    if layout['objects']['bowl']['yaw_deg']!=0 or any(row['subject']=='bowl' and row['kind']=='rotate_around_solution' for row in scene.spec['relations']):
        raise ValueError('This certificate requires default unmarked bowl yaw zero')
    grid_path=args.domain_report_dir/'stand_grid.json';grid=json.loads(grid_path.read_text())
    contradictions=[row['grid_index'] for row in grid['rows'] if blocked.covers(Point(row['base_origin_xyz'][:2])) and row['passed']]
    if contradictions:raise ValueError('Continuous source exclusion contradicts narrow phase witnesses: '+str(contradictions))
    files=[Path(__file__),ROOT/'scripts/extensions/replicator/search_l_island_bowl_domain.py',ROOT/'scripts/extensions/replicator/audit_initial_robot_geometry.py',
        ROOT/'scripts/extensions/replicator/search_source_layout_candidates.py',ROOT/'scripts/extensions/replicator/audit_source_layout_geometry.py',
        ROOT/'scripts/extensions/replicator/audit_task_source_relation.py',ROOT/'scripts/extensions/replicator/variant_geometry.py',
        ROOT/'kitchen_safety/planning.py',ROOT/'kitchen_safety/extensions/replicator/collision_planner.py',
        ROOT/'scripts/extensions/replicator/source_domain_rounding.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/validation/pre_physics.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/relation_loss_strategies.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/relations.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/utils/bounding_box.py',placer,
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/placement_candidate_generator.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/environments/arena_env_builder.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/object_placer.py',
        ROOT/'student_skills/robolaw-replicator/references/assignment.json',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena_environments/kitchen_bench/kitchen_bench_replicator_l_island_bowl_sink.yaml']
    report=dict(task_id=layout['task_id'],layout_path=str(config),layout_sha256=sha(config),
        source_domain=domain,source_bowl_domain=bowl_domain,required_collision_clearance_m=.002,
        proof_inset_m=INSET_M,proof_inset_direction='Shrink source interiors; excluded base set is a subset of actual penetration positions. This strengthens the impossibility proof conservatively and never reduces the execution2mm gate.',
        common_world_z_slab_m=z_world,stand_source_core=stand_info,source_blockers=constraints,
        continuous_base_domain_xy=mapping(complete_domain),guaranteed_source_overlap_xy=mapping(blocked.intersection(complete_domain)),
        remaining_base_xy_superset=mapping(remaining),remaining_area_m2=float(remaining.area),
        shoulder_z_interval_m=shoulder_z,all_legal_bowl_source_bounds_union=[blo.tolist(),bhi.tolist()],
        actual_source_bowl_collision_local_bounds=[bowl_collision_low.tolist(),bowl_collision_high.tolist()],
        actual_source_bowl_collision_vertices=len(bowl_collision),
        minimum_xy_distance_lower_bound_m=xy_distance,minimum_z_distance_lower_bound_m=z_distance,
        minimum_shoulder_to_any_legal_bowl_source_point_m=separation,
        nearest_xy_points=dict(remaining_shoulder=list(near_a.coords)[0],possible_bowl=list(near_b.coords)[0]),
        default_yaw_profile=dict(random_yaw_init=False,bowl_yaw_deg=0,robot_yaw_deg=layout['robot_yaw_deg'],
            scope='Pinned original builder default ObjectPlacerParams and original marker/generator handling; no custom placer/yaw override.'),
        robot_reach=contact,gripper_reach=all_gripper_reach_bound(world,robot),tcp_reach=reach_ball(world.kin),
        positive_separation_after_all_robot_reach_m=gap,
        fixed_stand_max_world_z_m=stand_z_upper,fixed_stand_to_all_legal_bowls_vertical_gap_m=stand_bowl_z_gap,
        source_geometry_no_robot_contact_certificate=bool(gap>0.),
        source_geometry_no_gripper_contact_certificate=bool(gap>0.),
        arbitrary_physics_task_infeasibility_proved=False,
        sampled_grid_cross_check=dict(path=str(grid_path),sha256=sha(grid_path),points=len(grid['rows']),contradictions=contradictions),
        source_robot=robot.provenance,
        geometry_layer_sha256={str(Path(layer.realPath)):sha(layer.realPath) for stage in scene.stages for layer in stage.GetUsedLayers() if Path(layer.realPath).is_file()},
        execution_ready=False,
        scope='Original default fixed-yaw initialization with actual float32-validator outer position domains and all source On allowed base/bowl heights. Only guaranteed penetration using strict inner prisms of verified closed convex SOURCE triangle meshes is excluded; other obstacles are omitted, making remaining positions a superset. Bowl AABB union is an outer bound used solely to LOWER-bound contact distance. Every source arm/gripper/base collider reach is UPPER-bounded with unrestricted revolute joints; fixed stand is vertically separate. Positive gaps rule out direct source-robot contact with an initially legal stationary bowl from any nonpenetrating source base in this default initialization profile. This is not arbitrary dynamic task infeasibility: physically moved bowls, transmitted forces, PhysX cooking/contact offsets or custom yaw profiles are outside the certificate. Static initial-geometry witnesses remain preserved.',
        **code_evidence(files))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),remaining_area_m2=report['remaining_area_m2'],
        minimum_gripper_to_bowl_gap_m=gap,no_source_gripper_contact=gap>0,grid_contradictions=len(contradictions))),flush=True)
    return int(not report['source_geometry_no_gripper_contact_certificate'])


if __name__=='__main__':raise SystemExit(main())
