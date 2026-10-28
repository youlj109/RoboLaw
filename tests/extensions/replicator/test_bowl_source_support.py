"""CPU safety tests for the independent bowl departure adapter."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from kitchen_safety.extensions.replicator.bowl_collision import BowlContactPolicy
from kitchen_safety.extensions.replicator.bowl_motion import bowl_rim_geometry
from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
from kitchen_safety.extensions.replicator.bowl_source_support_query import (
    bind_observed_departure, binding_check, partition_original_world,
)

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def geometry():
    return PayloadGeometry(ROOT/"assets/robolab/objects/ycb/bowl.usd")


def observed_fixture(geometry):
    tcp, payload = np.eye(4), np.eye(4)
    tcp[:3, 3] = [.5, .5, 1.1]
    payload[:3, 3] = [.5, .5, 1.-geometry.low[2]]
    vertices = np.array([[x, y, z] for x in (0., 1.) for y in (0., 1.) for z in (.95, 1.)])
    faces = np.array([[0, 2, 4], [2, 4, 6], [1, 3, 5], [3, 5, 7]])
    policy = BowlContactPolicy()
    obstacle = SimpleNamespace(getRotation=lambda: np.eye(3), getTranslation=lambda: np.zeros(3))
    world = SimpleNamespace(payload_name="bowl", contact_policy=policy, default_clearance=.002,
        objects=[(policy.source_support, obstacle, np.array([0., 0., .95]), np.array([1., 1., 1.]))],
        meshes={policy.source_support: (vertices, faces)},
        _object_reference=[(np.eye(3), np.zeros(3), np.array([0., 0., .95]), np.array([1., 1., 1.]), (vertices, faces))],
        _root_reference_transforms={policy.source_support: np.eye(4)}, payload_geometry=geometry,
        payload_pose=payload, attachment=np.linalg.inv(tcp)@payload,
        gripper_joint_positions=np.array([1, 1, -1, 1, -1, -1])*.75, gripper_joint_position=.75, phase="lift")
    kin = SimpleNamespace(fk=lambda _: tcp)
    point = bowl_rim_geometry(geometry)["center_local_m"]
    item = dict(count=1, points_object_local_m=[point], normal_forces_n=[1.], separations_m=[0.])
    contacts = [dict(phase="close", physics_step=i, object_name="bowl", physics_dt_s=1/120,
        fingers={name: deepcopy(item) for name in ("left_inner_finger", "right_inner_finger")}) for i in range(10)]
    observation = dict(physics_step=9, arm_q=np.zeros(7), measured_tcp=tcp,
        measured_payload=payload, raw_contact_rows=contacts)
    return world, kin, observation


def test_exact_source_bowl_binding_and_same_bounds_shape_change(geometry):
    world, kin, observed = observed_fixture(geometry)
    context = bind_observed_departure(world, kin, **observed)
    assert binding_check(world, np.zeros((2, 7)), context)["passed"]
    assert not binding_check(world, np.zeros((2, 7)), None)["passed"]
    world.meshes[world.contact_policy.source_support][1][0] = [0, 4, 2]
    assert not binding_check(world, np.zeros((2, 7)), context)["passed"]


@pytest.mark.parametrize("bad", ["source_pair", "wrong_payload", "wrong_observed_object", "rim_contact", "gap", "attachment", "missing_steps", "unloaded_finger"])
def test_real_grasp_binding_rejects_substituted_or_incomplete_evidence(geometry, bad):
    world, kin, obs = observed_fixture(geometry)
    contact = obs["raw_contact_rows"][-1]["fingers"]["left_inner_finger"]
    if bad == "source_pair": world.contact_policy = BowlContactPolicy(source_support="/other_counter")
    if bad == "wrong_payload": world.payload_name = "banana"
    if bad == "wrong_observed_object": obs["raw_contact_rows"][-1]["object_name"] = "banana"
    if bad == "rim_contact": contact["points_object_local_m"] = [[0., 0., 0.]]
    if bad == "gap": contact["separations_m"] = [.001]
    if bad == "attachment": world.attachment[2, 3] += .01
    if bad == "missing_steps": obs["raw_contact_rows"][3]["physics_step"] = 2
    if bad == "unloaded_finger": contact["normal_forces_n"] = [0.]
    with pytest.raises(ValueError):
        bind_observed_departure(world, kin, **obs)


def test_partition_keeps_robot_source_and_payload_sink_failures(geometry):
    world, _, _ = observed_fixture(geometry)
    support, sink = world.contact_policy.source_support, world.contact_policy.destination_name
    enabled = {"bowl": False, support: True, sink: True}
    checker = SimpleNamespace(enable_obstacle=lambda name, enable: enabled.__setitem__(name, enable))
    # sample 0: robot/source + payload/source; sample 1: payload/sink.
    pairs = np.array([[[1., 0.], [1., 0.]], [[0., 0.], [0., 1.]]])
    def forward(query, env_query_idx=None):
        active = (query[:, 0, :, 3].numpy() > 0.).astype(float)
        worlds = np.array([enabled[support], enabled[sink]], float)
        return torch.tensor((pairs*active[:, :, None]*worlds[None, None, :]).sum(axis=2)[:, None, :])
    config = SimpleNamespace(link_name_to_idx_map={"robot": 0, "attached_object": 1}, link_sphere_idx_map=torch.tensor([0, 1]))
    rollout = SimpleNamespace(primitive_collision_constraint=SimpleNamespace(world_coll_checker=checker, forward=forward),
        dynamics_model=SimpleNamespace(robot_model=SimpleNamespace(kinematics_config=config)))
    planner = SimpleNamespace(world=world, motion_gen=SimpleNamespace(ik_solver=SimpleNamespace(rollout_fn=rollout)),
        candidate_contact_mask={"object_switches": ["bowl"]}, obstacle_names=list(enabled), _ignored=lambda name, masks: name in masks)
    state = SimpleNamespace(robot_spheres=torch.ones((2, 1, 2, 4)))
    original = dict(components={"world_collision": dict(costs=[2., 1.])})
    report = partition_original_world(planner, state, original)
    assert report["disjoint_exhaustive"] and report["original_zero_nonzero_equivalent"]
    assert report["rows"]["robot_all_world"]["nonzero_samples"] == [0]
    assert report["rows"]["payload_non_support"]["nonzero_samples"] == [1]
    assert not report["noncontact_passed"]
    assert enabled == {"bowl": False, support: True, sink: True}


def test_destination_never_masked_and_caller_masks_rejected(monkeypatch, geometry):
    from kitchen_safety.extensions.replicator.bowl_source_support_curobo import BowlSourceSupportCuroboPlanner
    from kitchen_safety.curobo_backend import CuroboPlanner
    world, _, _ = observed_fixture(geometry)
    world.set_phase = lambda phase: setattr(world, "phase", phase)
    planner = BowlSourceSupportCuroboPlanner.__new__(BowlSourceSupportCuroboPlanner)
    planner.world = world
    calls = []
    monkeypatch.setattr(CuroboPlanner, "_prepare_path_query", lambda self, masks, grip, phase: calls.append((masks, grip, phase)))
    planner._prepare_path_query((), 1, "lower")
    assert calls == [(("bowl",), 1, "lower")]
    assert planner.candidate_contact_mask["original_sink_destination_retained"]
    with pytest.raises(ValueError, match="caller obstacle masks"):
        planner._prepare_path_query((world.contact_policy.source_support,), 1, "lift")
    with pytest.raises(ValueError, match="grip_binary=1"):
        planner._prepare_path_query((), 0, "lift")


def test_missing_binding_cannot_execute_candidate_before_cuda():
    from kitchen_safety.extensions.replicator.bowl_source_support_curobo import BowlSourceSupportCuroboPlanner
    class Kin:
        def fk(self, q):
            pose = np.eye(4); pose[:3, 3] = q[:3]; return pose
        def ik(self, pose, previous, attempts=1):
            q = previous.copy(); q[:3] = pose[:3, 3]; return q
    planner = BowlSourceSupportCuroboPlanner.__new__(BowlSourceSupportCuroboPlanner)
    planner.kin, planner.dt, planner.max_joint_speed = Kin(), 1/15, .4
    start = np.zeros(7); start[2] = 1.
    goal = planner.kin.fk(start); goal[2, 3] += .15
    with pytest.raises(RuntimeError, match="actual measured-state/raw-contact binding"):
        planner._cartesian_vertical(start, goal, "lift")
    assert len(planner.last_cartesian_candidate["positions"]) > 2
