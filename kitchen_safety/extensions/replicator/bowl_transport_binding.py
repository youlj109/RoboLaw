"""Observed lift-end planning attachment; no physical weld or stability claim.

The original contact region, forces, clearances and sink target are retained.
All close-to-lift relative motion remains evidence. Passing the latest ten
loaded contact samples does not imply a rigid grasp or future safe transport.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib

import numpy as np
from scipy.spatial.transform import Rotation

from .bowl_controller_profile import require_grasp_profile, sink_target
from .bowl_motion import bowl_rim_geometry
from .bowl_source_support_query import digest, rigid, source_support
from .bowl_destination_query import destination_floor
from .task_motion import pose_matrix


def payload_geometry_digest(world):
    arrays = {}
    for name, values, dtype in (("vertices", world.payload_geometry.vertices, "<f8"),
                               ("triangles", world.payload_geometry.triangles, "<i8")):
        data = np.ascontiguousarray(values, dtype=dtype)
        if not np.isfinite(data).all():
            raise ValueError("Nonfinite original bowl geometry")
        arrays[name] = dict(shape=list(data.shape), dtype=dtype,
                           sha256=hashlib.sha256(data.tobytes()).hexdigest())
    return digest(dict(source=world.payload_geometry.provenance, arrays=arrays))


def lift_motion_evidence(physics_rows, payload_index):
    """Retain every observed relation from the last close through end of lift."""
    closes = [i for i, row in enumerate(physics_rows) if row["phase"] == "close"]
    if not closes:
        raise ValueError("Missing actual close origin for lift motion evidence")
    rows = physics_rows[closes[-1]:]
    if len(rows) < 11 or any(row["phase"] != "lift" for row in rows[1:]):
        raise ValueError("Require complete close-end to current lift-end observations")
    steps = [int(row["physics_step"]) for row in rows]
    if any(b-a != 1 for a, b in zip(steps, steps[1:])):
        raise ValueError("Missing actual physics substep in lift history")
    relations = []
    for row in rows:
        tcp = rigid(pose_matrix(row["tcp_pose_w"]), "observed TCP")
        payload = rigid(pose_matrix(row["task_object_states"][payload_index, :7]), "observed bowl")
        relations.append(np.linalg.inv(tcp)@payload)
    reference = relations[0]
    translation = [float(np.linalg.norm(value[:3, 3]-reference[:3, 3])) for value in relations]
    rotation = [float(np.rad2deg(Rotation.from_matrix(value[:3, :3]@reference[:3, :3].T).magnitude()))
                for value in relations]
    samples = [dict(physics_step=step, measured_tcp_to_bowl=relation.tolist(),
                    translation_from_close_m=distance, rotation_from_close_deg=angle)
               for step, relation, distance, angle in zip(steps, relations, translation, rotation)]
    return dict(samples=samples, first_physics_step=steps[0], last_physics_step=steps[-1],
                sample_count=len(samples), samples_sha256=digest(samples),
                final_translation_from_close_m=translation[-1], final_rotation_from_close_deg=rotation[-1],
                maximum_translation_from_close_m=max(translation), maximum_rotation_from_close_deg=max(rotation),
                rigid_grasp_verified=False,
                meaning="All actual close-to-lift substeps; re-observing planning geometry does not erase slip")


@dataclass(frozen=True)
class TransportBinding:
    physics_step: int
    arm_q: tuple
    six_finger_q: tuple
    measured_tcp: tuple
    measured_payload: tuple
    previous_attachment: tuple
    measured_attachment: tuple
    source_geometry_sha256: str
    controller_profile_sha256: str
    contact_region: tuple
    raw_contact_window_sha256: str
    raw_contact_steps: tuple
    lift_motion_sha256: str
    actual_source_fcl: dict
    source_fk: dict

    def evidence(self):
        value = asdict(self)
        return dict(**value, evidence_sha256=digest(value), physical_state_written=False,
                    rigid_grasp_verified=False, future_transport_verified=False,
                    scope="Latest ten loaded near-rim observations and actual full source FCL/FK; fresh full-path CUDA/FCL remains mandatory")


def observe_transport_binding(world, kin, profile, *, physics_step, arm_q, measured_tcp,
                              measured_payload, raw_contact_rows, lift_motion):
    """Validate observations and return a new relation without changing world."""
    require_grasp_profile(profile)
    source_support(world)
    destination_floor(world)
    if (profile['sink_target']['offset_xy_m'] != [.05, -.1] or profile['release_gap_m'] != .003 or
        profile['task_id'] != 'kitchen_bench_replicator_g_shape_bowl_sink'):
        raise ValueError("Keep the original reviewed source sink landing and task")
    if world.phase != "lift" or world.payload_geometry.provenance["source_sha256"] != profile["source_bowl_sha256"]:
        raise ValueError("Transport rebinding requires actual original bowl at lift end")
    tcp, payload = rigid(measured_tcp, "measured TCP"), rigid(measured_payload, "measured bowl")
    previous = rigid(world.attachment, "previous measured close attachment")
    if not np.allclose(world.payload_pose, payload, atol=1e-12, rtol=0.):
        raise ValueError("World bowl pose is stale")
    q, fingers = np.asarray(arm_q, float), np.asarray(world.gripper_joint_positions, float)
    if q.shape != (7,) or fingers.shape != (6,) or not np.isfinite(np.r_[q, fingers]).all():
        raise ValueError("Require real seven arm and six independent finger observations")
    fk = kin.fk(q)
    fk_error = dict(position_error_m=float(np.linalg.norm(fk[:3, 3]-tcp[:3, 3])),
                    rotation_error_deg=float(np.rad2deg(Rotation.from_matrix(fk[:3, :3]@tcp[:3, :3].T).magnitude())))
    if fk_error["position_error_m"] > .002 or fk_error["rotation_error_deg"] > 2. or abs(world.gripper_joint_position-fingers[0]) > 1e-12:
        raise ValueError("Original source FK/actual finger identity gate failed")
    region = np.asarray(bowl_rim_geometry(world.payload_geometry)["contact_region_local_m"])
    rows = raw_contact_rows[-10:]
    expected = list(range(int(physics_step)-9, int(physics_step)+1))
    if len(rows) != 10 or [row["physics_step"] for row in rows] != expected:
        raise ValueError("Transport requires ten consecutive latest physics contacts")
    dt = None
    for row in rows:
        current_dt = float(row["physics_dt_s"])
        if (row["phase"] != "lift" or row["object_name"] != "bowl" or
            not np.isfinite(current_dt) or current_dt <= 0. or (dt is not None and current_dt != dt)):
            raise ValueError("Lift contact phase/identity/time base differs")
        dt = current_dt
        for name in ("left_inner_finger", "right_inner_finger"):
            item = row["fingers"][name]
            points = np.asarray(item["points_object_local_m"], float).reshape(-1, 3)
            forces, gaps = np.asarray(item["normal_forces_n"], float), np.asarray(item["separations_m"], float)
            if (len(points) != item["count"] or len(forces) != len(points) or len(gaps) != len(points) or
                len(points) >= 4096 or not all(np.isfinite(value).all() for value in (points, forces, gaps))):
                raise ValueError("Incomplete raw lift contacts")
            loaded = forces > .01
            if (not loaded.any() or not np.all(points[loaded] >= region[0]) or
                not np.all(points[loaded] <= region[1]) or not np.all(gaps[loaded] <= .00025)):
                raise ValueError("Original bilateral loaded/near upper-rim gate failed")
    attachment = np.linalg.inv(tcp)@payload
    samples = lift_motion["samples"]
    if (lift_motion["last_physics_step"] != int(physics_step) or len(samples) != lift_motion["sample_count"] or
        [row['physics_step'] for row in samples] != list(range(lift_motion['first_physics_step'], int(physics_step)+1)) or
        digest(samples) != lift_motion["samples_sha256"] or
        not np.allclose(samples[0]["measured_tcp_to_bowl"], previous, atol=1e-9, rtol=0.) or
        not np.allclose(samples[-1]["measured_tcp_to_bowl"], attachment, atol=1e-9, rtol=0.)):
        raise ValueError("Lift drift evidence does not bind old and current actual attachments")
    # Explicit actual object pose prevents the old planning relation from being
    # mistaken for current physical geometry in this independent source check.
    actual_valid = world.valid(q, mode="execution", gripper_q=fingers, object_pose=payload)
    fcl = dict(passed=bool(actual_valid), measurement=deepcopy(world.last_clearance),
               mode="execution", actual_six_fingers=True, actual_payload_pose=True)
    if not actual_valid:
        raise ValueError("Actual lift-end source FCL rejected transport binding: "+str(fcl))
    return TransportBinding(int(physics_step), tuple(q), tuple(fingers), tuple(tcp.ravel()), tuple(payload.ravel()),
        tuple(previous.ravel()), tuple(attachment.ravel()), payload_geometry_digest(world), digest(profile),
        tuple(region.ravel()), digest(rows), tuple(expected), lift_motion["samples_sha256"], fcl, fk_error)


def apply_transport_binding(world, profile, context):
    """Change only planning geometry after matching the actual observed state."""
    if not isinstance(context, TransportBinding):
        raise ValueError("No observed transport attachment binding")
    if (world.phase != "lift" or digest(profile) != context.controller_profile_sha256 or
        payload_geometry_digest(world) != context.source_geometry_sha256 or
        not np.array_equal(world.gripper_joint_positions, context.six_finger_q) or
        not np.allclose(np.asarray(world.payload_pose).ravel(), context.measured_payload, atol=1e-12, rtol=0.) or
        not np.allclose(np.asarray(world.attachment).ravel(), context.previous_attachment, atol=1e-12, rtol=0.)):
        raise ValueError("State changed after observed transport binding")
    world.attachment = np.asarray(context.measured_attachment).reshape(4, 4).copy()
    return context.evidence()


def transport_binding_check(world, positions, profile, context):
    if not isinstance(context, TransportBinding):
        return dict(passed=False, reason="No observed lift-end transport binding")
    source_support(world)
    destination_floor(world)
    q = np.asarray(positions)
    checks = dict(phase_is_transport=world.phase == "transport",
        actual_q_start=q.ndim == 2 and q.shape[1:] == (7,) and len(q) > 0 and np.array_equal(q[0], context.arm_q),
        actual_six_fingers=np.array_equal(world.gripper_joint_positions, context.six_finger_q),
        current_payload=np.allclose(np.asarray(world.payload_pose).ravel(), context.measured_payload, atol=1e-12, rtol=0.),
        fresh_attachment=np.allclose(np.asarray(world.attachment).ravel(), context.measured_attachment, atol=1e-12, rtol=0.),
        exact_source_payload=payload_geometry_digest(world) == context.source_geometry_sha256,
        fixed_controller_profile=digest(profile) == context.controller_profile_sha256)
    return dict(passed=all(checks.values()), checks=checks, evidence=context.evidence())


def transport_target(world, profile, context):
    """Original selected landing, with current orientation and new attachment."""
    if not transport_binding_check(world, [context.arm_q], profile, context)["passed"]:
        raise ValueError("Cannot compute transport target from stale observations")
    payload = np.asarray(context.measured_payload).reshape(4, 4)
    pose = np.r_[payload[:3, 3], Rotation.from_matrix(payload[:3, :3]).as_quat()]
    landing, info = sink_target(pose, world, profile)
    desired = landing.copy()
    desired[2, 3] = max(payload[2, 3], landing[2, 3]+.20)
    return desired@np.linalg.inv(world.attachment), dict(original_sink_landing=info,
        desired_source_bowl_pose=desired.tolist(), newly_observed_attachment=context.evidence(),
        rigid_grasp_verified=False, previous_plan_reused=False)
