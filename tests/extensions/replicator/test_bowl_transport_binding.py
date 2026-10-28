"""Real-recording regression and fail-closed transport tests; CPU only."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.bowl_transport_binding import (
    observe_transport_binding, apply_transport_binding, transport_binding_check, transport_target,
)
from kitchen_safety.extensions.replicator.bowl_transport_curobo import BowlTransportCuroboPlanner
from scripts.extensions.replicator.diagnose_bowl_transport import recorded_inputs

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def recording():
    return recorded_inputs(ROOT/"datasets/replicator/g_shape_bowl_sink/base/attempt_02")


@pytest.fixture
def observed(recording):
    world, kin, profile, observations, _, _ = recording
    observations = deepcopy(observations)
    world.set_phase("lift")
    world.attachment = np.asarray(observations["lift_motion"]["samples"][0]["measured_tcp_to_bowl"]).copy()
    world.payload_pose = observations["measured_payload"].copy()
    names = [recording[4]["joint_names"].index(name) for name in world.GRIPPER_JOINT_NAMES]
    index = np.flatnonzero(recording[5]["phase"] == "lift")[-1]
    world.gripper_joint_positions = recording[5]["q"][index, names].copy()
    world.gripper_joint_position = float(world.gripper_joint_positions[0])
    return world, kin, deepcopy(profile), observations


def test_real_lift_slip_is_preserved_and_only_planning_attachment_changes(observed):
    world, kin, profile, raw = observed
    old, actual = world.attachment.copy(), world.payload_pose.copy()
    context = observe_transport_binding(world, kin, profile, **raw)
    np.testing.assert_array_equal(world.attachment, old)
    motion = raw["lift_motion"]
    assert motion["sample_count"] == 409
    assert 36.44 < motion["final_rotation_from_close_deg"] < 36.45
    assert .0506 < motion["final_translation_from_close_m"] < .0507
    assert not context.evidence()["rigid_grasp_verified"]
    apply_transport_binding(world, profile, context)
    np.testing.assert_array_equal(world.payload_pose, actual)
    assert not np.allclose(world.attachment, old)
    world.set_phase("transport")
    assert transport_binding_check(world, [context.arm_q], profile, context)["passed"]
    target, info = transport_target(world, profile, context)
    np.testing.assert_allclose(target[:3, :3], raw["measured_tcp"][:3, :3], atol=1e-12, rtol=0.)
    assert profile["sink_target"]["offset_xy_m"] == [.05, -.1]
    assert info["previous_plan_reused"] is False


@pytest.mark.parametrize("bad", ["old_close_contacts", "missing_step", "unloaded", "gap", "outside_region",
    "wrong_object", "stale_tcp", "wrong_six_identity", "stale_object", "modified_history", "old_attachment"])
def test_invalid_real_evidence_cannot_rebind(observed, bad):
    world, kin, profile, raw = observed
    item = raw["raw_contact_rows"][-1]["fingers"]["left_inner_finger"]
    if bad == "old_close_contacts": raw["raw_contact_rows"][-1]["phase"] = "close"
    if bad == "missing_step": raw["raw_contact_rows"][-2]["physics_step"] -= 1
    if bad == "unloaded": item["normal_forces_n"] = [0.]*item["count"]
    if bad == "gap": item["separations_m"] = [.001]*item["count"]
    if bad == "outside_region": item["points_object_local_m"] = [[0., 0., 0.]]*item["count"]
    if bad == "wrong_object": raw["raw_contact_rows"][-1]["object_name"] = "banana"
    if bad == "stale_tcp": raw["measured_tcp"][0, 3] += .01
    if bad == "wrong_six_identity": world.gripper_joint_position += .01
    if bad == "stale_object": world.payload_pose[0, 3] += .001
    if bad == "modified_history": raw["lift_motion"]["samples"][1]["rotation_from_close_deg"] += 1.
    if bad == "old_attachment": world.attachment[0, 3] += .001
    previous = world.attachment.copy()
    with pytest.raises(ValueError):
        observe_transport_binding(world, kin, profile, **raw)
    np.testing.assert_array_equal(world.attachment, previous)


def test_actual_source_collision_is_not_overruled_by_contact_window(observed, monkeypatch):
    world, kin, profile, raw = observed
    monkeypatch.setattr(world, "valid", lambda *a, **k: False)
    with pytest.raises(ValueError, match="source FCL"):
        observe_transport_binding(world, kin, profile, **raw)


@pytest.mark.parametrize("bad", ["joint_start", "finger", "stale_attachment", "profile", "source_geometry"])
def test_stale_query_is_rejected_after_valid_observation(observed, bad):
    world, kin, profile, raw = observed
    context = observe_transport_binding(world, kin, profile, **raw)
    apply_transport_binding(world, profile, context)
    world.set_phase("transport")
    q = np.asarray([context.arm_q]).copy()
    vertices = world.payload_geometry.vertices.copy()
    try:
        if bad == "joint_start": q[0, 0] += 1e-5
        if bad == "finger": world.gripper_joint_positions[3] += 1e-5
        if bad == "stale_attachment": world.attachment = np.asarray(context.previous_attachment).reshape(4, 4).copy()
        if bad == "profile": profile["sink_target"]["offset_xy_m"] = [0., 0.]
        if bad == "source_geometry": world.payload_geometry.vertices[0, 0] += 1e-5
        assert not transport_binding_check(world, q, profile, context)["passed"]
    finally:
        world.payload_geometry.vertices[:] = vertices


def test_actual_six_lock_update_precedes_rebuilt_source_payload_spheres():
    events = []
    class Motion:
        def update_locked_joints(self, locked, robot):
            events.append(("lock", locked.copy()))
        def attach_spheres_to_robot(self, sphere_tensor):
            events.append(("attach", sphere_tensor.copy()))
        def reset(self, **kwargs):
            events.append(("reset", kwargs))
    six = np.array([.60, .61, -.62, .63, -.64, -.65])
    names = tuple("finger"+str(i) for i in range(6))
    world = SimpleNamespace(phase="transport", gripper_joint_positions=six, gripper_joint_position=.60,
        GRIPPER_JOINT_NAMES=names, attachment=np.eye(4), payload_name="bowl",
        payload_geometry=SimpleNamespace(provenance={"source_sha256": "source"},
            sphere_cover=lambda count: (np.array([[.1, .2, .3]]), np.array([.01]), {"kind": "source_cover"})))
    planner = object.__new__(BowlTransportCuroboPlanner)
    planner.world, planner.motion_gen = world, Motion()
    planner.robot_dict = {"kinematics": {"lock_joints": {}, "extra_collision_spheres": {"attached_object": 2}}}
    planner.args, planner.T_ee_tcp = SimpleNamespace(to_device=lambda x: x), np.eye(4)
    planner._last_grip = None
    planner.transport_context = SimpleNamespace(arm_q=np.zeros(7), evidence=lambda: {"evidence_sha256": "binding"})
    planner._transport_check = lambda q: {"passed": True}
    planner._sync_gripper_and_attachment(1, .005)
    assert [event[0] for event in events] == ["lock", "attach"]
    assert events[0][1] == dict(zip(names, six))
    np.testing.assert_allclose(events[1][1][0], [.1, .2, .3, .015], atol=1e-8)
    assert events[1][1][1, 3] == -100.
    assert planner.payload_geometry_report["actual_six_finger_scope"] == "transport only"
    planner._sync_gripper_and_attachment(1, .005)
    assert [event[0] for event in events] == ["lock", "attach", "attach"]


def test_unbound_transport_cannot_prepare_cuda_query():
    planner = object.__new__(BowlTransportCuroboPlanner)
    planner.world = SimpleNamespace(set_phase=lambda phase: None)
    with pytest.raises(ValueError, match="without fresh"):
        planner._prepare_path_query((), 1, "transport")


def test_cuda_failure_and_source_failure_still_preserve_every_dense_sample(monkeypatch):
    import kitchen_safety.extensions.replicator.bowl_transport_curobo as module
    calls = []
    world = SimpleNamespace(set_phase=lambda phase: None, gripper_joint_positions=np.arange(6.)/10,
                            last_clearance={"failure": "source"})
    def valid(q, **kwargs):
        calls.append(q.copy())
        return len(calls) != 2
    world.valid = valid
    def components(planner, samples):
        row = dict(costs=[1.]+[0.]*(len(samples)-1), maximum_cost=1., nonzero_samples=[0])
        return dict(valid=False, samples=len(samples), rejected_samples=[0],
                    components={name: deepcopy(row) for name in ("joint_bound", "self_collision", "world_collision")}), None
    monkeypatch.setattr(module, "constraint_components", components)
    planner = object.__new__(BowlTransportCuroboPlanner)
    planner.world, planner._last_grip = world, .6
    planner._transport_check = lambda path: {"passed": True}
    planner._prepare_path_query = lambda *args: 0.
    planner.candidate_contact_mask, planner.payload_geometry_report = {}, {}
    report = planner.validate_path(np.array([np.zeros(7), np.ones(7)*.04]), grip_binary=1, phase="transport")
    assert not report["valid"]
    assert report["samples"] == report["fcl_samples_checked"] == len(calls) == 3
    assert report["fcl_samples_unchecked"] == 0
    assert report["validation_complete"]
    assert report["curobo_failures"] and report["fcl_failures"]
