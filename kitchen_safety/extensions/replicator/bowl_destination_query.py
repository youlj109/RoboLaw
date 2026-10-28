"""One measured bowl/BasinBottom lowering query, with original margins retained.

Only the carried payload's enclosing spheres against the original sink floor
are refined to source triangles. Every robot/floor, payload/drain and other
world sphere pair, full self/joint constraints and source FCL remain mandatory.
This is pre-release lowering to the fixed 3 mm gap: even this selected pair
retains the original 2 mm noncontact clearance. No physical contact exemption,
collision filter, state write, fabricated contact or destination change exists.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .bowl_collision import BowlContactPolicy
from .bowl_motion import bowl_rim_geometry
from .bowl_controller_profile import sink_target
from .bowl_source_support_query import (
    digest, rigid, payload_bounds, collider_geometry_evidence, partition_exact_world_pair,
)
from .task_plan_diagnostics import constraint_components


def destination_floor(world):
    policy = BowlContactPolicy()
    if (world.payload_name != "bowl" or world.contact_policy.destination_name != policy.destination_name or
        world.contact_policy.source_support != policy.source_support or world.default_clearance != .002 or
        world.contact_policy.support_band_m != policy.support_band_m):
        raise ValueError("No lowering refinement for changed task roles or safety policy")
    rows = [row for row in world.objects if row[0] == policy.destination_name]
    if len(rows) != 1:
        raise ValueError("The original sink destination must be one exact BasinBottom collider")
    return rows[0]


def selected_target(world, payload, profile):
    if (profile["task_id"] != "kitchen_bench_replicator_g_shape_bowl_sink" or
        profile["sink_target"]["offset_xy_m"] != [.05, -.1] or profile["release_gap_m"] != .003 or
        profile["grasp_rule"] != "source_negative_x_upper_wall_rim"):
        raise ValueError("Retain the reviewed source-sink landing profile")
    pose = np.r_[payload[:3, 3], Rotation.from_matrix(payload[:3, :3]).as_quat()]
    return sink_target(pose, world, profile)


@dataclass(frozen=True)
class SinkLoweringContext:
    physics_step: int
    arm_q: tuple
    six_finger_q: tuple
    measured_tcp: tuple
    measured_payload: tuple
    measured_attachment: tuple
    target_payload: tuple
    source_asset_sha256: str
    exact_floor_geometry_sha256: str
    controller_profile_sha256: str
    raw_contact_window_sha256: str
    raw_contact_steps: tuple
    contact_region: tuple

    def evidence(self):
        result = asdict(self)
        return dict(**result, evidence_sha256=digest(result),
            physical_claim="Caller-supplied actual transport-end observations + complete bilateral rim contact window; independent recording/provenance audit remains required")


def bind_observed_lowering(world, kin, profile, *, physics_step, arm_q, measured_tcp, measured_payload, raw_contact_rows):
    destination_floor(world)
    tcp, payload = rigid(measured_tcp, "measured TCP"), rigid(measured_payload, "measured bowl")
    attachment = rigid(world.attachment, "measured lowering attachment")
    if not np.allclose(attachment, np.linalg.inv(tcp)@payload, atol=1e-9, rtol=0.):
        raise ValueError("Lowering attachment must be re-observed from current TCP and bowl")
    if not np.allclose(world.payload_pose, payload, atol=1e-9, rtol=0.):
        raise ValueError("World bowl pose differs from current observation")
    q, fingers = np.asarray(arm_q, float), np.asarray(world.gripper_joint_positions, float)
    if q.shape != (7,) or fingers.shape != (6,) or not np.isfinite(np.r_[q, fingers]).all():
        raise ValueError("Require actual seven arm and six finger joint observations")
    fk = kin.fk(q)
    if (np.linalg.norm(fk[:3, 3]-tcp[:3, 3]) > .002 or
        Rotation.from_matrix(fk[:3, :3]@tcp[:3, :3].T).magnitude() > np.deg2rad(2.) or
        abs(float(world.gripper_joint_position)-fingers[0]) > 1e-12):
        raise ValueError("Observed source FK/finger identity differs beyond original gates")
    target, _ = selected_target(world, payload, profile)
    if np.linalg.norm(payload[:2, 3]-target[:2, 3]) > .005 or payload[2, 3] <= target[2, 3]:
        raise ValueError("Observed carried bowl is not above the selected original sink landing column")
    rows = [row for row in raw_contact_rows if row["phase"] == "transport"][-10:]
    expected = list(range(int(physics_step)-9, int(physics_step)+1))
    if len(rows) != 10 or [row["physics_step"] for row in rows] != expected:
        raise ValueError("Lowering requires ten consecutive latest actual transport substeps")
    region = np.asarray(bowl_rim_geometry(world.payload_geometry)["contact_region_local_m"])
    dt = None
    for row in rows:
        current_dt = float(row["physics_dt_s"])
        if row["object_name"] != "bowl" or not np.isfinite(current_dt) or current_dt <= 0. or (dt is not None and current_dt != dt):
            raise ValueError("Transport contact identity/time base differs")
        dt = current_dt
        for name in ("left_inner_finger", "right_inner_finger"):
            item = row["fingers"][name]
            points = np.asarray(item["points_object_local_m"], float).reshape(-1, 3)
            forces, gaps = np.asarray(item["normal_forces_n"], float), np.asarray(item["separations_m"], float)
            if (len(points) != item["count"] or len(forces) != len(points) or len(gaps) != len(points) or
                len(points) >= 4096 or not all(np.isfinite(v).all() for v in (points, forces, gaps))):
                raise ValueError("Incomplete raw transport finger contacts")
            loaded = forces > .01
            if (not loaded.any() or not np.all(points[loaded] >= region[0]) or
                not np.all(points[loaded] <= region[1]) or not np.all(gaps[loaded] <= .00025)):
                raise ValueError("Latest transport lacks simultaneous original loaded upper-rim contacts")
    return SinkLoweringContext(physics_step=int(physics_step), arm_q=tuple(q), six_finger_q=tuple(fingers),
        measured_tcp=tuple(tcp.ravel()), measured_payload=tuple(payload.ravel()), measured_attachment=tuple(attachment.ravel()),
        target_payload=tuple(target.ravel()), source_asset_sha256=world.payload_geometry.provenance["source_sha256"],
        exact_floor_geometry_sha256=collider_geometry_evidence(world, destination_floor(world)[0])["geometry_sha256"],
        controller_profile_sha256=digest(profile), raw_contact_window_sha256=digest(rows),
        raw_contact_steps=tuple(expected), contact_region=tuple(region.ravel()))


def lowering_binding_check(world, positions, profile, context):
    if not isinstance(context, SinkLoweringContext):
        return dict(passed=False, reason="No actual transport/contact/attachment binding; diagnostic only")
    floor = destination_floor(world)
    geometry = collider_geometry_evidence(world, floor[0])
    target, _ = selected_target(world, rigid(world.payload_pose, "payload"), profile)
    checks = dict(phase_is_lower=world.phase == "lower",
        query_starts_at_actual_q=np.array_equal(np.asarray(positions)[0], context.arm_q),
        actual_fingers_unchanged=np.array_equal(world.gripper_joint_positions, context.six_finger_q),
        actual_bowl_unchanged=np.allclose(np.asarray(world.payload_pose).ravel(), context.measured_payload, atol=1e-12, rtol=0.),
        measured_attachment_unchanged=np.allclose(np.asarray(world.attachment).ravel(), context.measured_attachment, atol=1e-12, rtol=0.),
        selected_target_unchanged=np.allclose(target.ravel(), context.target_payload, atol=1e-12, rtol=0.),
        original_payload_unchanged=world.payload_geometry.provenance["source_sha256"] == context.source_asset_sha256,
        exact_floor_geometry_unchanged=geometry["geometry_sha256"] == context.exact_floor_geometry_sha256,
        controller_profile_unchanged=digest(profile) == context.controller_profile_sha256)
    return dict(passed=all(checks.values()), checks=checks, evidence=context.evidence(), exact_floor_geometry=geometry)


def lowering_source_geometry(world, kin, positions, profile):
    """Original source triangle/floor at >=2 mm; no contact allowance is used."""
    floor = destination_floor(world)
    attachment = rigid(world.attachment, "planning attachment")
    q = np.asarray(positions, float)
    if q.ndim != 2 or q.shape[1] != 7 or len(q) < 2 or not np.isfinite(q).all():
        raise ValueError("Require at least two finite seven-joint lowering samples")
    tcp = np.array([kin.fk(row) for row in q])
    source_pose = rigid(world.payload_pose, "observed or explicitly hypothetical initial bowl")
    target, target_info = selected_target(world, source_pose, profile)
    target_tcp = target@np.linalg.inv(attachment)
    xy = np.linalg.norm(tcp[:, :2, 3]-tcp[0, :2, 3], axis=1)
    rotation = Rotation.from_matrix(tcp[:, :3, :3]@tcp[0, :3, :3].T).magnitude()
    checks = dict(phase_is_lower=world.phase == "lower", xy_column_5mm=bool(xy.max() <= .005),
        orientation_2deg=bool(rotation.max() <= np.deg2rad(2.)),
        target_column_5mm=bool(np.linalg.norm(source_pose[:2, 3]-target[:2, 3]) <= .005),
        moves_down=bool(tcp[-1, 2, 3] < tcp[0, 2, 3]),
        no_upward_tcp=bool(np.all(np.diff(tcp[:, 2, 3]) <= 1e-6)),
        requested_3mm_landing_height=bool(abs(tcp[-1, 2, 3]-target_tcp[2, 3]) <= .002),
        bounded_height=bool(tcp[:, 2, 3].max() <= tcp[0, 2, 3]+.002 and tcp[:, 2, 3].min() >= target_tcp[2, 3]-.002))
    rows, previous = [], None
    for index, frame in enumerate(tcp):
        pose = frame@attachment
        vertices, low, high = payload_bounds(world, pose)
        shape = world.fcl.CollisionObject(world.payload_triangle_model, world.fcl.Transform(pose[:3, :3], pose[:3, 3]))
        raw = float(world.fcl.distance(shape, floor[1], world.fcl.DistanceRequest(), world.fcl.DistanceResult()))
        overlap = bool(world.fcl.collide(shape, floor[1], world.fcl.CollisionRequest(), world.fcl.CollisionResult()))
        if not np.isfinite(raw):
            raise ValueError("Nonfinite exact source bowl/floor distance")
        source_clearance = bool(not overlap and raw >= world.default_clearance)
        full_xy = bool(np.all(low[:2] >= floor[2][:2]) and np.all(high[:2] <= floor[3][:2]))
        downward = previous is None or bool(np.all(vertices[:, 2] <= previous[:, 2]+1e-6))
        previous = vertices
        rows.append(dict(sample=index, passed=source_clearance and full_xy and downward,
            pair=["payload/bowl", floor[0]], source_triangle_distance_m=max(0., raw),
            source_intersecting=bool(overlap or raw < 0.), required_clearance_m=world.default_clearance,
            source_clearance_passed=source_clearance, full_bowl_xy_inside_original_floor=full_xy,
            no_upward_source_vertices=downward, source_bottom_gap_m=float(low[2]-floor[3][2]),
            source_contact_allowance_used=False, bounds=[low.tolist(), high.tolist()]))
    return dict(passed=all(checks.values()) and all(row["passed"] for row in rows), path_checks=checks, rows=rows,
        selected_original_target=target_info, exact_floor_geometry=collider_geometry_evidence(world, floor[0]),
        scope="Only payload/original BasinBottom proxy refined to original source triangles, retaining >=2 mm before release; every robot-floor and payload-other pair remains in original conservative queries")


def audit_sink_lowering(planner, positions, profile, context=None):
    world = planner.world
    binding = lowering_binding_check(world, positions, profile, context)
    geometry = lowering_source_geometry(world, planner.kin, positions, profile)
    original, state = constraint_components(planner, positions)
    partition = partition_exact_world_pair(planner, state, original, destination_floor(world)[0])
    fcl = []
    for index, q in enumerate(positions):
        valid = world.valid(q)
        fcl.append(dict(sample=index, valid=valid, measurement=deepcopy(world.last_clearance)))
    mandatory = dict(joint_zero=not original["components"]["joint_bound"]["nonzero_samples"],
        self_zero=not original["components"]["self_collision"]["nonzero_samples"],
        original_robot_floor_and_payload_other_spheres=partition["noncontact_passed"],
        exact_source_floor_2mm=geometry["passed"], all_source_fcl=all(row["valid"] for row in fcl))
    return dict(valid=bool(binding["passed"] and all(mandatory.values())),
        candidate_geometry_valid=all(mandatory.values()), actual_state_binding=binding,
        mandatory_checks=mandatory, original_components=original, original_world_pair_partition=partition,
        destination_geometry=geometry, all_source_fcl=dict(checked=len(fcl), rows=fcl),
        scope="Single original bowl/BasinBottom sphere refinement only; unbound candidates cannot execute, and physical release/support/task/independent gates remain required")
