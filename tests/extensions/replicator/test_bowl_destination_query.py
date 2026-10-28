"""CPU source-floor safety and observation-binding regressions, no physics."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.bowl_controller_profile import read_profile
from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_geometry
from kitchen_safety.extensions.replicator.bowl_destination_query import (
    bind_observed_lowering, lowering_binding_check, lowering_source_geometry,
)
from scripts.extensions.replicator.audit_bowl_controller_cpu import build_world

ROOT = Path(__file__).resolve().parents[3]


def fixture():
    # Synthetic unit-test observations exercise rejection/identity rules. They
    # are not saved as real contacts or supplied to any physical collector.
    profile = read_profile()
    layout = json.loads((ROOT/"configs/extensions/replicator/g_shape_bowl_sink_source_layout.json").read_text())
    world, kin, _ = build_world(layout)
    with np.load(ROOT/"reports/replicator_preparation/g_bowl_sink_placement_cpu01/selected_candidate.npz", allow_pickle=False) as raw:
        path, attachment = raw["lower"].copy(), raw["hypothetical_attachment"].copy()
    world.set_phase("lower")
    world.gripper_joint_position = .75
    world.gripper_joint_positions = np.array([1., 1., -1., 1., -1., -1.])*.75
    world.attachment = attachment
    tcp = kin.fk(path[0])
    world.payload_pose = tcp@attachment
    point = bowl_rim_geometry(world.payload_geometry)["center_local_m"]
    item = dict(count=1, points_object_local_m=[point], normal_forces_n=[1.], separations_m=[0.])
    rows = [dict(phase="transport", physics_step=i, object_name="bowl", physics_dt_s=1/120,
        fingers={name: deepcopy(item) for name in ("left_inner_finger", "right_inner_finger")}) for i in range(10)]
    observed = dict(physics_step=9, arm_q=path[0], measured_tcp=tcp,
        measured_payload=world.payload_pose.copy(), raw_contact_rows=rows)
    return world, kin, profile, path, observed


def test_original_source_triangles_retain_2mm_and_unbound_never_passes():
    world, kin, profile, path, observed = fixture()
    report = lowering_source_geometry(world, kin, path, profile)
    assert report["passed"]
    assert min(row["source_triangle_distance_m"] for row in report["rows"]) >= .002
    assert all(row["required_clearance_m"] == .002 and not row["source_contact_allowance_used"] for row in report["rows"])
    assert not lowering_binding_check(world, path, profile, None)["passed"]
    context = bind_observed_lowering(world, kin, profile, **observed)
    assert lowering_binding_check(world, path, profile, context)["passed"]
    world.gripper_joint_positions[3] += .001
    assert not lowering_binding_check(world, path, profile, context)["passed"]


@pytest.mark.parametrize("bad", ["center_fallback", "old_close_contacts", "unloaded_finger", "fabricated_attachment", "missing_substep", "outside_rim", "changed_floor"])
def test_lowering_binding_rejects_invalid_profile_or_actual_evidence(bad):
    world, kin, profile, path, observed = fixture()
    if bad == "center_fallback": profile["sink_target"]["offset_xy_m"] = [0., 0.]
    if bad == "old_close_contacts":
        for row in observed["raw_contact_rows"]: row["phase"] = "close"
    if bad == "unloaded_finger": observed["raw_contact_rows"][-1]["fingers"]["right_inner_finger"]["normal_forces_n"] = [0.]
    if bad == "fabricated_attachment": world.attachment[2, 3] += .001
    if bad == "missing_substep": observed["raw_contact_rows"][5]["physics_step"] = 3
    if bad == "outside_rim": observed["raw_contact_rows"][-1]["fingers"]["left_inner_finger"]["points_object_local_m"] = [[0., 0., 0.]]
    if bad == "changed_floor":
        from kitchen_safety.extensions.replicator.bowl_collision import BowlContactPolicy
        world.contact_policy = BowlContactPolicy(destination_name="/arbitrary_sink")
    with pytest.raises(ValueError):
        bind_observed_lowering(world, kin, profile, **observed)


def test_subclearance_floor_cannot_pass_triangle_refinement():
    world, kin, profile, path, _ = fixture()
    unsafe = path.copy()
    pose = kin.fk(path[-1]); pose[2, 3] -= .0025
    unsafe[-1] = kin.ik(pose, path[-1])
    report = lowering_source_geometry(world, kin, unsafe, profile)
    assert not report["passed"]
    assert not report["rows"][-1]["source_clearance_passed"]
    assert report["rows"][-1]["source_triangle_distance_m"] < .002


def test_floor_with_same_bounds_but_changed_topology_invalidates_live_binding():
    world, kin, profile, path, observed = fixture()
    context = bind_observed_lowering(world, kin, profile, **observed)
    faces = world.meshes[world.contact_policy.destination_name][1]
    faces[0] = faces[0][::-1]
    assert not lowering_binding_check(world, path, profile, context)["passed"]
