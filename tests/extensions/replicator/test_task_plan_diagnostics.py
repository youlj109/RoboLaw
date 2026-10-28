"""CPU checks: candidate parity and rejected-path evidence, no GPU/Kit."""
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from kitchen_safety.curobo_backend import CuroboPlanner
from kitchen_safety.extensions.replicator.task_curobo import PayloadCuroboPlanner
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate, constraint_components


class LinearKinematics:
    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        return pose

    def ik(self, pose, previous, attempts=1):
        q = previous.copy()
        q[:3] = pose[:3, 3]
        return q


def planner_fixture():
    planner = PayloadCuroboPlanner.__new__(PayloadCuroboPlanner)
    planner.kin = LinearKinematics()
    planner.dt, planner.max_joint_speed = 1/15, .4
    planner._last_grip, planner.collision_geometry = 0., {}
    planner.world = SimpleNamespace(attachment=None)
    planner.args = SimpleNamespace(to_device=lambda value: torch.tensor(value, dtype=torch.float32))
    planner.motion_gen = SimpleNamespace(ik_solver=SimpleNamespace(check_valid=lambda q: torch.ones(len(q), dtype=torch.bool)))
    return planner


def test_candidate_exactly_preserves_public_cartesian_sampling_and_retiming():
    planner = planner_fixture()
    start = np.array([.2, .1, 1.1, 0., 0., 0., 0.])
    goal = planner.kin.fk(start)
    goal[2, 3] -= .125
    public = CuroboPlanner._cartesian_vertical(planner, start, goal, "approach")
    local = planner._cartesian_vertical(start, goal, "approach")
    assert np.array_equal(local["positions"], public["positions"])
    assert np.array_equal(local["effective_goal_pose"], public["effective_goal_pose"])
    assert np.array_equal(planner.last_cartesian_candidate["positions"], public["positions"])
    assert local["metadata"]["start_joint_error_rad"] == 0.


def test_rejected_candidate_is_kept_and_never_returned(monkeypatch):
    import kitchen_safety.extensions.replicator.task_plan_diagnostics as diagnostics
    planner = planner_fixture()
    planner.motion_gen.ik_solver.check_valid = lambda q: torch.zeros(len(q), dtype=torch.bool)
    monkeypatch.setattr(diagnostics, "constraint_components", lambda *args: (
        dict(components={"world_collision": dict(nonzero_samples=[3])}), None))
    start = np.array([.2, .1, 1.1, 0., 0., 0., 0.])
    goal = planner.kin.fk(start)
    goal[2, 3] -= .125
    with pytest.raises(RuntimeError, match="world_collision"):
        planner._cartesian_vertical(start, goal, "approach")
    assert len(planner.last_cartesian_candidate["positions"]) > 2
    assert planner.last_constraint_diagnostics["components"]["world_collision"]["nonzero_samples"] == [3]


def test_local_column_gate_remains_enforced():
    start = np.array([.2, .1, 1.1, 0., 0., 0., 0.])
    kin = LinearKinematics()
    goal = kin.fk(start)
    goal[0, 3] += .01
    with pytest.raises(RuntimeError, match="vertical column"):
        local_cartesian_candidate(kin, start, goal, "approach", 1/15, .4)


def test_constraint_diagnostic_does_not_accept_disagreement_with_original_check():
    planner = planner_fixture()
    state = SimpleNamespace(state_seq=object(), robot_spheres=object())
    zero = lambda *args, **kwargs: torch.zeros((2, 1))
    rollout = SimpleNamespace(start_state=None,
        dynamics_model=SimpleNamespace(forward=lambda *args: state),
        constraint_fn=lambda *args, **kwargs: SimpleNamespace(feasible=torch.zeros((2, 1), dtype=torch.bool)),
        bound_constraint=SimpleNamespace(forward=zero),
        primitive_collision_constraint=SimpleNamespace(enabled=True, forward=zero),
        robot_self_collision_constraint=SimpleNamespace(enabled=True, forward=zero))
    planner.motion_gen.ik_solver.rollout_fn = rollout
    with pytest.raises(RuntimeError, match="differs from check_valid"):
        constraint_components(planner, np.zeros((2, 7)))


def test_original_buffered_gripper_spheres_are_retained_in_target_envelope():
    from kitchen_safety.extensions.replicator.task_robot_spheres import open_gripper_minimum_z
    rotation = np.array([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]])
    minimum, report = open_gripper_minimum_z(rotation)
    assert minimum < -.029
    assert report["collision_sphere_buffer_m"] == .005
    assert report["lowest"]["link"] in ("left_inner_finger", "right_inner_finger")
    assert set(report["source_sha256"]) == {"config", "urdf", "calibration"}


def test_positive_subnormal_fcl_distance_cannot_hide_zero_margin_intersection():
    from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld
    world = PayloadCollisionWorld.__new__(PayloadCollisionWorld)
    world.fcl = SimpleNamespace(distance=lambda *args: 6e-323, collide=lambda *args: 1,
        DistanceRequest=lambda: None, DistanceResult=lambda: None, CollisionRequest=lambda: None, CollisionResult=lambda: None)
    result = world._exact_measurement((0., "payload/banana", "bowl", object(), object(), 0.))
    assert result["intersecting"]
    assert not result["valid"]
