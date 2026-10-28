"""CPU failure-mode tests for selective pairs, source targets and full evidence."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld, PayloadGeometry
from kitchen_safety.extensions.replicator.task_contact_policy import BananaContactPolicy
from kitchen_safety.extensions.replicator.task_motion import banana_midbody, banana_bowl_lower_target, top_down_grasp
from kitchen_safety.extensions.replicator.task_recorder import audit_physics

ROOT = Path(__file__).resolve().parents[3]
BANANA = ROOT/"assets/extensions/replicator/source/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/ycb/banana.usd"
BOWL = ROOT/"assets/robolab/objects/ycb/bowl.usd"


def pair_world(labels, obstacle="/World/envs/env_0/banana/mesh", object_z=0., support=False):
    import fcl
    world = PayloadCollisionWorld.__new__(PayloadCollisionWorld)
    world.fcl = fcl
    world.payload_name, world.payload_pose, world.attachment = "banana", np.eye(4), None
    world.payload_pose[2, 3] = object_z
    world.contact_policy, world.phase = BananaContactPolicy(), "approach"
    world.payload_geometry = SimpleNamespace(vertices=np.array([[-.01, -.01, -.01], [.01, .01, .01]]))
    world.set_obstacle_pose = lambda *args: None
    world.default_clearance = .002
    world.clearances, world.execution_clearances = {}, {}
    world._pair_selection_key, world._pair_selection_cache = None, {}
    low, high = (np.array([-.1, -.1, -.1]), np.array([.1, .1, 0.])) if support else (np.full(3, -.01), np.full(3, .01))
    obstacle_shape = fcl.CollisionObject(fcl.Box(*(high-low)), fcl.Transform((high+low)/2))
    world.objects = [(obstacle, obstacle_shape, low, high)]
    world.low, world.high = low[None], high[None]
    center = np.array([0., 0., object_z])
    shapes = [(label, fcl.CollisionObject(fcl.Box(.02, .02, .02), fcl.Transform(center)), center-.01, center+.01)
              for label in labels]
    world.shapes = lambda *args, **kwargs: shapes
    return world


def test_finger_contact_does_not_exempt_arm_palm_or_other_obstacles():
    for label in ("gripper/left_inner_finger", "gripper/right_inner_finger", "finger", "robot_hull/whole_gripper"):
        world = pair_world([label])
        assert world.valid(np.zeros(7))
        world.phase = "pregrasp"
        assert not world.valid(np.zeros(7))
        world = pair_world([label], obstacle="/World/envs/env_0/kitchen/door")
        assert not world.valid(np.zeros(7))
    for label in ("arm", "gripper", "gripper/base_link", "gripper/left_outer_knuckle", "robot_hull/panda_link7"):
        assert not pair_world([label]).valid(np.zeros(7))
    assert not pair_world(["robot_hull/whole_gripper", "gripper/left_inner_finger", "gripper/base_link"]).valid(np.zeros(7))


def test_payload_support_exception_is_exact_pair_and_bounded_depth():
    support = BananaContactPolicy().source_support
    assert pair_world(["payload/banana"], support, object_z=.009, support=True).valid(np.zeros(7))
    assert not pair_world(["payload/banana"], support, object_z=0., support=True).valid(np.zeros(7))
    assert not pair_world(["arm"], support, object_z=.009, support=True).valid(np.zeros(7))
    assert not pair_world(["payload/banana"], support+"_wrong", object_z=.009, support=True).valid(np.zeros(7))


def test_payload_duplicate_is_skipped_without_masking_other_pairs():
    assert pair_world(["payload/banana"]).valid(np.zeros(7))
    assert not pair_world(["payload/banana", "arm"]).valid(np.zeros(7))


def test_cuda_candidate_masks_are_phase_derived_and_explicitly_incomplete():
    policy = BananaContactPolicy()
    assert policy.cuda_candidate_masks("pregrasp", False) == ()
    assert policy.cuda_candidate_masks("approach", False) == ("banana",)
    assert policy.cuda_candidate_masks("lower", True) == ("banana", "bowl")
    assert not policy.finger_pair("gripper/base_link", "close")
    with pytest.raises(ValueError):
        policy.cuda_candidate_masks("unreviewed_phase", False)
    assert "FCL" in policy.report()["cuda_limitation"]


def test_real_banana_midbody_and_vertical_support_target():
    banana, bowl = PayloadGeometry(BANANA), PayloadGeometry(BOWL)
    info = banana_midbody(banana)
    np.testing.assert_allclose(info["center_local_m"], [-.0330336159, 0., 0.], atol=1e-6)
    delta, diagnostic = banana_bowl_lower_target([0, 0, .25, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], banana, bowl)
    assert -.3 < delta < -.1
    assert 0 < diagnostic["overlapping_source_vertices"] < diagnostic["total_source_vertices"]
    with pytest.raises(ValueError, match="No legal downward"):
        banana_bowl_lower_target([1, 0, .25, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], banana, bowl)


def test_actual_source_gripper_preserves_counter_clearance_for_topdown_target():
    import fcl
    world = PayloadCollisionWorld.__new__(PayloadCollisionWorld)
    world.fcl, world.default_clearance, world.contact_policy = fcl, .002, BananaContactPolicy()
    world.gripper_models, world.gripper_sweep = world._gripper_geometry()
    world.objects = [(world.contact_policy.source_support, None, np.array([-3., 1., .855]), np.array([-1., 3., .8949]))]
    target, info = top_down_grasp([-2.26, 2.15, .9152344465, 0, 0, 0, 1], PayloadGeometry(BANANA), world)
    np.testing.assert_allclose(target[:3, 0], [0., 0., -1.])
    np.testing.assert_allclose(target[:3, 1], [1., 0., 0.])
    assert target[2, 3]+info["source_gripper_lowest_relative_tcp_z_m"] >= .8969-1e-10
    # A grossly over-raised target would never allow central-body contact.
    assert info["tcp_raise_for_source_fingers_m"] < .025


def recording():
    n = 7
    meta = dict(physics_dt_s=1., action_dt_s=2., reset_physics_step=10, joint_names=["joint"],
        robot_body_names=["base_link"], contact_body_names=["base_link"], task_object_names=["banana", "bowl"],
        background_paths=["/kitchen/door"], static_anchor_paths=[], task_reference_paths=[], callback_errors=[],
        native_articulation_read_errors=[], background_topology=dict(joint_connected_native_paths=["/kitchen/door"],
            unobserved_joint_connected_bodies=[]))
    data = dict(physics_step=np.arange(10, 10+n), q=np.zeros((n, 1)), dq=np.zeros((n, 1)),
        task_object_states=np.zeros((n, 2, 13)), robot_body_pose_w=np.zeros((n, 1, 7)),
        robot_contact_force=np.zeros((n, 1, 3)), destination_contact_force=np.zeros((n, 1, 1, 3)),
        background_rigid_poses=np.zeros((n, 1, 7)), background_rigid_velocities=np.zeros((n, 1, 6)),
        static_anchor_transforms=np.zeros((n, 0, 4, 4)), task_reference_transforms=np.zeros((n, 0, 4, 4)))
    data["background_rigid_poses"][:, :, 6] = 1.
    actions = [dict(camera_physics_steps=[10+i*2, 10+i*2]) for i in range(3)]
    contacts = [dict(physics_step=step) for step in data["physics_step"]]
    return data, meta, actions, contacts


def test_full_physics_gate_rejects_missing_warmup_contact_or_camera_step():
    data, meta, actions, contacts = recording()
    assert audit_physics(data, meta, actions, contacts)["passed"]
    assert not audit_physics(data, meta, actions, contacts[1:])["passed"]
    data["physics_step"][1] += 1
    assert not audit_physics(data, meta, actions, contacts)["passed"]
    data, meta, actions, contacts = recording()
    actions[0]["camera_physics_steps"][1] += 1
    assert not audit_physics(data, meta, actions, contacts)["passed"]


def test_background_motion_from_reset_cannot_hide_in_settle():
    data, meta, actions, contacts = recording()
    data["background_rigid_poses"][1, 0, 0] = .01
    report = audit_physics(data, meta, actions, contacts)
    assert not report["checks"]["background_stable"]
    assert report["background"][0]["max_translation_m"] == .01
