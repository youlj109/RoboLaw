"""Independent bowl/CapSurface_02 departure policy, with original gates retained.

Scoped from the reviewed banana algorithm; the frozen banana module is not
imported or mutated. The measured region is the original bowl upper wall/rim.

Only a measured, loaded, source-supported bowl may depart vertically from its
original CapSurface_02. Robot/world, payload/other-world, full self/joint
constraints and every independent FCL sample remain mandatory. No destination
or physical collision filter is changed. An unbound diagnostic cannot execute.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json

import numpy as np
from scipy.spatial.transform import Rotation

from .bowl_collision import BowlContactPolicy
from .bowl_motion import bowl_rim_geometry
from .task_plan_diagnostics import constraint_components


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def rigid(value, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError(name+" must be a finite 4x4 rigid transform")
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-9, rtol=0.) or
        not np.allclose(value[:3,:3].T@value[:3,:3], np.eye(3), atol=1e-6, rtol=0.) or
        abs(np.linalg.det(value[:3,:3])-1.) > 1e-6):
        raise ValueError(name+" is not rigid")
    return value


def source_support(world):
    policy = BowlContactPolicy()
    if (world.payload_name != policy.payload_name or world.contact_policy.source_support != policy.source_support or
        world.contact_policy.support_band_m != policy.support_band_m or world.default_clearance != .002):
        raise ValueError("No departure exception for altered task identity, support or safety policy")
    supports = [row for row in world.objects if row[0] == policy.source_support]
    if len(supports) != 1:
        raise ValueError("Original source support must have one exact collision identity")
    return supports[0]


def payload_bounds(world, pose):
    vertices = world.payload_geometry.vertices@pose[:3,:3].T+pose[:3,3]
    return vertices, vertices.min(0), vertices.max(0)


def support_geometry_evidence(world):
    """Bind source mesh topology/vertices and actual/reference transforms."""
    return collider_geometry_evidence(world, source_support(world)[0])


def collider_geometry_evidence(world, name):
    """Exact immutable mesh/transform fingerprint; grants no pair exception."""
    rows = [row for row in world.objects if row[0] == name]
    if len(rows) != 1:
        raise ValueError("Expected one exact declared collider")
    name,obstacle,low,high = rows[0]
    if name not in world.meshes:
        raise ValueError("The declared source support must retain its exact original mesh")
    index = next(i for i,row in enumerate(world.objects) if row[0]==name)
    rotation,translation,reference_low,reference_high,reference_mesh = world._object_reference[index]
    if reference_mesh is None or name not in world._root_reference_transforms:
        raise ValueError("Missing source support mesh/reference transform provenance")
    current_vertices,current_faces = world.meshes[name]
    reference_vertices,reference_faces = reference_mesh
    arrays = dict(current_vertices=(current_vertices,"<f8"),current_faces=(current_faces,"<i8"),
        reference_vertices=(reference_vertices,"<f8"),reference_faces=(reference_faces,"<i8"),
        current_fcl_rotation=(obstacle.getRotation(),"<f8"),current_fcl_translation=(obstacle.getTranslation(),"<f8"),
        reference_fcl_rotation=(rotation,"<f8"),reference_fcl_translation=(translation,"<f8"),
        authored_root_transform=(world._root_reference_transforms[name],"<f8"),
        current_bounds=(np.r_[low,high],"<f8"),reference_bounds=(np.r_[reference_low,reference_high],"<f8"))
    entries = {}
    for key,(value,dtype) in arrays.items():
        data = np.ascontiguousarray(value,dtype=dtype)
        if not np.isfinite(data).all():
            raise ValueError("Nonfinite support geometry cannot be bound")
        entries[key] = dict(shape=list(data.shape),dtype=dtype,sha256=hashlib.sha256(data.tobytes()).hexdigest())
    if not np.array_equal(current_faces,reference_faces):
        raise ValueError("Source support topology differs from authored reference")
    actual_rotation = np.asarray(obstacle.getRotation())@np.asarray(rotation).T
    actual_translation = np.asarray(obstacle.getTranslation())-actual_rotation@np.asarray(translation)
    expected_vertices = np.asarray(reference_vertices)@actual_rotation.T+actual_translation
    if (not np.allclose(current_vertices,expected_vertices,atol=1e-9,rtol=0.) or
        not np.allclose(np.asarray(current_vertices).min(0),low,atol=1e-9,rtol=0.) or
        not np.allclose(np.asarray(current_vertices).max(0),high,atol=1e-9,rtol=0.)):
        raise ValueError("Support current mesh, original FCL transform and bounds disagree")
    record = dict(exact_collider=name,arrays=entries,
        meaning="Exact current/authored world mesh vertices+triangles, FCL current/reference transforms, authored root and bounds; same bounds alone are insufficient")
    return dict(**record,geometry_sha256=digest(record))


@dataclass(frozen=True)
class SourceDepartureContext:
    physics_step: int
    arm_q: tuple
    six_finger_q: tuple
    measured_tcp: tuple
    measured_payload: tuple
    measured_attachment: tuple
    support_bounds: tuple
    source_asset_sha256: str
    source_support_geometry_sha256: str
    raw_contact_window_sha256: str
    raw_contact_steps: tuple
    contact_region: tuple

    def evidence(self):
        result = asdict(self)
        return dict(**result, evidence_sha256=digest(result),
            physical_claim="Caller-supplied observed state plus verified raw contact window; independent recording/provenance audit remains mandatory")


def bind_observed_departure(world, kin, *, physics_step, arm_q, measured_tcp, measured_payload, raw_contact_rows):
    """Bind one actual close→lift query; never synthesize or write an attachment.

    The caller passes the latest recorder state and ObjectGraspContacts.records.
    Ten consecutive close substeps, both loaded original upper-rim contacts, the
    measured six finger angles and inv(actual TCP)@actual object are verified.
    """
    support = source_support(world)
    tcp, payload = rigid(measured_tcp, "measured TCP"), rigid(measured_payload, "measured payload")
    attachment = rigid(world.attachment, "existing measured planning attachment")
    measured_attachment = np.linalg.inv(tcp)@payload
    if not np.allclose(attachment, measured_attachment, atol=1e-9, rtol=0.):
        raise ValueError("Planning attachment differs from the measured TCP/object relation")
    if not np.allclose(world.payload_pose, payload, atol=1e-9, rtol=0.):
        raise ValueError("Collision world does not contain the latest observed payload pose")
    q = np.asarray(arm_q, float)
    fingers = np.asarray(world.gripper_joint_positions, float)
    if q.shape != (7,) or fingers.shape != (6,) or not np.isfinite(np.r_[q, fingers]).all():
        raise ValueError("Require actual seven arm and six finger joint observations")
    fk = kin.fk(q)
    if (np.linalg.norm(fk[:3,3]-tcp[:3,3]) > .002 or
        Rotation.from_matrix(fk[:3,:3]@tcp[:3,:3].T).magnitude() > np.deg2rad(2.)):
        raise ValueError("Observed TCP disagrees with source FK beyond original gates")
    if abs(float(world.gripper_joint_position)-fingers[0]) > 1e-12:
        raise ValueError("cuRobo measured finger_joint and actual six-finger state disagree")
    _, low, high = payload_bounds(world, payload)
    if not (np.all(low[:2] >= support[2][:2]) and np.all(high[:2] <= support[3][:2]) and
            abs(low[2]-support[3][2]) <= world.contact_policy.support_band_m):
        raise ValueError("Observed bowl is not near its exact source-support contact band")
    rows = [row for row in raw_contact_rows if row["phase"] == "close"][-10:]
    expected_steps = list(range(int(physics_step)-9, int(physics_step)+1))
    if len(rows) != 10 or [row["physics_step"] for row in rows] != expected_steps:
        raise ValueError("Departure requires ten consecutive latest physical close substeps")
    region = np.asarray(bowl_rim_geometry(world.payload_geometry)["contact_region_local_m"])
    physics_dt = None
    for row in rows:
        dt = float(row["physics_dt_s"])
        if row["object_name"] != "bowl" or not np.isfinite(dt) or dt <= 0.:
            raise ValueError("Contact window has wrong object or invalid physical time step")
        if physics_dt is not None and dt != physics_dt:
            raise ValueError("Contact window changed physical time step")
        physics_dt = dt
        for name in ("left_inner_finger", "right_inner_finger"):
            item = row["fingers"][name]
            points = np.asarray(item["points_object_local_m"], float).reshape(-1, 3)
            forces, gaps = np.asarray(item["normal_forces_n"], float), np.asarray(item["separations_m"], float)
            if (len(points) != item["count"] or len(forces) != len(points) or len(gaps) != len(points) or
                len(points) >= 4096 or not all(np.isfinite(value).all() for value in (points, forces, gaps))):
                raise ValueError("Incomplete/invalid raw upper-rim finger contacts")
            loaded = forces > .01
            if (not loaded.any() or not np.all(points[loaded] >= region[0]) or
                not np.all(points[loaded] <= region[1]) or not np.all(gaps[loaded] <= .00025)):
                raise ValueError("Original bilateral loaded upper-rim-contact gate failed")
    return SourceDepartureContext(physics_step=int(physics_step), arm_q=tuple(q), six_finger_q=tuple(fingers),
        measured_tcp=tuple(tcp.ravel()), measured_payload=tuple(payload.ravel()), measured_attachment=tuple(attachment.ravel()),
        support_bounds=tuple(np.r_[support[2], support[3]]), source_asset_sha256=world.payload_geometry.provenance["source_sha256"],
        source_support_geometry_sha256=support_geometry_evidence(world)["geometry_sha256"],
        raw_contact_window_sha256=digest(rows), raw_contact_steps=tuple(expected_steps),
        contact_region=tuple(region.ravel()))


def binding_check(world, positions, context):
    if not isinstance(context, SourceDepartureContext):
        return dict(passed=False, reason="No actual observed source-departure binding; diagnostic only")
    support = source_support(world)
    support_evidence = support_geometry_evidence(world)
    checks = dict(query_starts_at_measured_q=np.array_equal(np.asarray(positions)[0], context.arm_q),
        actual_fingers_unchanged=np.array_equal(world.gripper_joint_positions, context.six_finger_q),
        measured_payload_unchanged=np.allclose(np.asarray(world.payload_pose).ravel(), context.measured_payload, atol=1e-12, rtol=0.),
        attachment_unchanged=np.allclose(np.asarray(world.attachment).ravel(), context.measured_attachment, atol=1e-12, rtol=0.),
        source_support_unchanged=np.allclose(np.r_[support[2],support[3]],context.support_bounds,atol=1e-12,rtol=0.),
        source_asset_unchanged=world.payload_geometry.provenance["source_sha256"]==context.source_asset_sha256,
        exact_support_geometry_unchanged=support_evidence["geometry_sha256"]==context.source_support_geometry_sha256,
        phase_is_lift=world.phase=="lift")
    return dict(passed=all(checks.values()), checks=checks, evidence=context.evidence(),support_geometry=support_evidence)


def source_departure_geometry(world, kin, positions):
    """Replace only P×S's proxy test by original source geometry/contact rules."""
    support = source_support(world)
    attachment = rigid(world.attachment, "planning attachment")
    q = np.asarray(positions, float)
    if q.ndim != 2 or q.shape[1] != 7 or len(q) < 2 or not np.isfinite(q).all():
        raise ValueError("Expected at least two finite seven-joint departure samples")
    tcp = np.array([kin.fk(item) for item in q])
    xy = np.linalg.norm(tcp[:,:2,3]-tcp[0,:2,3], axis=1)
    rotation = Rotation.from_matrix(tcp[:,:3,:3]@tcp[0,:3,:3].T).magnitude()
    # Same original vertical-column bounds, plus no downward departure. This
    # policy is deliberately limited to the collector's 0.15 m lift profile.
    path_checks = dict(phase_is_lift=world.phase=="lift", xy_column_5mm=bool(xy.max() <= .005),
        orientation_2deg=bool(rotation.max() <= np.deg2rad(2.)),
        no_downward_tcp=bool(np.all(np.diff(tcp[:,2,3]) >= -1e-6)),
        original_lift_150mm=bool(abs((tcp[-1,2,3]-tcp[0,2,3])-.15) <= .002),
        height_interval=bool(tcp[:,2,3].min() >= tcp[0,2,3]-.002 and tcp[:,2,3].max() <= tcp[0,2,3]+.152))
    rows, previous = [], None
    for index, pose_tcp in enumerate(tcp):
        pose = pose_tcp@attachment
        vertices, low, high = payload_bounds(world, pose)
        shape = world.fcl.CollisionObject(world.payload_model, world.fcl.Transform(pose[:3,:3],pose[:3,3]))
        overlap = bool(world.fcl.collide(shape,support[1],world.fcl.CollisionRequest(),world.fcl.CollisionResult()))
        raw = float(world.fcl.distance(shape,support[1],world.fcl.DistanceRequest(),world.fcl.DistanceResult()))
        if not np.isfinite(raw):
            raise ValueError("Nonfinite source support distance")
        normal_clearance = not overlap and raw >= world.default_clearance
        contact_geometry = bool(np.all(low[:2] >= support[2][:2]) and np.all(high[:2] <= support[3][:2])
            and low[2] >= support[3][2]-world.contact_policy.support_band_m)
        upward_source = previous is None or bool(np.all(vertices[:,2] >= previous[:,2]-1e-6))
        previous = vertices
        # Source geometry rather than enclosing spheres handles this one pair.
        # Source departure must remain upward even after true contact ends.
        passed = bool(contact_geometry and upward_source and all(path_checks.values()))
        rows.append(dict(sample=index, passed=passed, pair=["payload/bowl",support[0]],
            source_distance_m=max(0.,raw), source_intersecting=bool(overlap or raw < 0),
            normal_source_clearance_passed=bool(normal_clearance),
            necessary_source_contact_used=not normal_clearance,
            original_support_band_m=world.contact_policy.support_band_m,
            bounds=[low.tolist(),high.tolist()], source_contact_geometry_passed=contact_geometry,
            no_downward_source_vertices=upward_source))
    return dict(passed=all(path_checks.values()) and all(row["passed"] for row in rows), path_checks=path_checks, rows=rows,
        scope="Only original bowl/CapSurface_02 source convex geometry with existing 6 mm support contact band and full XY containment; other robot/payload/world pairs are never exempt")


def partition_original_world(planner, state, original_components):
    """Exhaustive disjoint partition with exact original CUDA cost functions."""
    return partition_exact_world_pair(planner, state, original_components, source_support(planner.world)[0])


def partition_exact_world_pair(planner, state, original_components, support):
    """Retain all original costs while partitioning one explicit source pair.

    This low-level evidence function grants no permission to execute. Its
    caller must separately validate the task identity, phase, live binding and
    exact source geometry. The only allowed roles are this task's two supports.
    """
    allowed = {planner.world.contact_policy.source_support, planner.world.contact_policy.destination_name}
    if support not in allowed:
        raise ValueError("No partition policy for another object")
    cost = planner.motion_gen.ik_solver.rollout_fn.primitive_collision_constraint
    checker = cost.world_coll_checker
    config = planner.motion_gen.ik_solver.rollout_fn.dynamics_model.robot_model.kinematics_config
    reverse = {value:key for key,value in config.link_name_to_idx_map.items()}
    labels = [reverse[int(index)] for index in config.link_sphere_idx_map.detach().cpu().numpy()]
    payload_ids = [i for i,label in enumerate(labels) if label=="attached_object"]
    robot_ids = [i for i,label in enumerate(labels) if label!="attached_object"]
    if not payload_ids or not robot_ids or sorted(robot_ids+payload_ids) != list(range(len(labels))):
        raise ValueError("Original robot/payload sphere partition is not exhaustive")
    masks = planner.candidate_contact_mask["object_switches"]
    if list(masks) != ["bowl"]:
        raise ValueError("Departure query must retain every world object except its duplicate carried bowl")
    enabled = [name for name in planner.obstacle_names if not planner._ignored(name,masks)]
    if support not in enabled or len(enabled)!=len(set(enabled)):
        raise ValueError("Missing or ambiguous original support collider")
    rows = {}
    groups = (("robot_all_world",payload_ids,enabled),
              ("payload_non_support",robot_ids,[name for name in enabled if name!=support]),
              ("payload_source_support",robot_ids,[support]))
    try:
        for name,inactive,obstacles in groups:
            query = state.robot_spheres.clone()
            query[...,inactive,3] = -100.
            active = set(obstacles)
            for obstacle in planner.obstacle_names:
                checker.enable_obstacle(obstacle,enable=obstacle in active)
            values = cost.forward(query.contiguous(),env_query_idx=None).detach().cpu().numpy()
            values = values.reshape(len(values),-1).max(axis=1)
            if not np.isfinite(values).all():
                raise ValueError("Nonfinite original partition cost")
            rows[name] = dict(costs=values.tolist(),maximum_cost=float(values.max()),
                nonzero_samples=np.flatnonzero(values!=0.).tolist(),enabled_obstacles=obstacles,
                active_sphere_ids=robot_ids if name=="robot_all_world" else payload_ids)
    finally:
        for obstacle in planner.obstacle_names:
            checker.enable_obstacle(obstacle,enable=obstacle in enabled)
    union = np.logical_or.reduce([np.asarray(row["costs"])!=0. for row in rows.values()])
    original_bad = np.asarray(original_components["components"]["world_collision"]["costs"])!=0.
    if not np.array_equal(union,original_bad):
        raise ValueError("Original full world and disjoint pair partition disagree")
    return dict(rows=rows, original_zero_nonzero_equivalent=True, disjoint_exhaustive=True,
        original_sphere_world_pairs=len(labels)*len(enabled),
        noncontact_passed=not rows["robot_all_world"]["nonzero_samples"] and not rows["payload_non_support"]["nonzero_samples"],
        exact_source_pair=["attached_object",support], physical_collision_filter_changes=False)


def audit_source_departure(planner, positions, context=None):
    """Return facts for diagnostics; only a bound fully checked query is valid."""
    world = planner.world
    binding = binding_check(world,positions,context)
    geometry = source_departure_geometry(world,planner.kin,positions)
    original,state = constraint_components(planner,positions)
    partition = partition_original_world(planner,state,original)
    fcl_rows = []
    for index,q in enumerate(positions):
        valid = world.valid(q)
        fcl_rows.append(dict(sample=index,valid=valid,measurement=deepcopy(world.last_clearance)))
    mandatory = dict(joint_zero=not original["components"]["joint_bound"]["nonzero_samples"],
        self_zero=not original["components"]["self_collision"]["nonzero_samples"],
        original_noncontact_spheres=partition["noncontact_passed"], source_geometry=geometry["passed"],
        all_source_fcl=all(row["valid"] for row in fcl_rows))
    return dict(valid=bool(binding["passed"] and all(mandatory.values())),
        candidate_geometry_valid=all(mandatory.values()), actual_state_binding=binding,
        mandatory_checks=mandatory, original_components=original, original_world_pair_partition=partition,
        source_support_geometry=geometry, all_source_fcl=dict(checked=len(fcl_rows),rows=fcl_rows),
        scope="Single exact source-support pair refinement only; unbound candidates cannot authorize execution; dense and every actual physical-step independent audits remain required")
