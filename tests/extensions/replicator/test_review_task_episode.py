"""Meaningful negative boundaries for the independent task-aware reviewer (CPU)."""
from copy import deepcopy
import json

import numpy as np
import pytest

from scripts.extensions.replicator.review_task_episode import (
    TASKS, FINGERS, contact_window, raw_destination, sequence_audit, prefix_audit,
    timeline_audit, bound_sidecar)


def contacts_fixture(n=10):
    data = dict(physics_step=np.arange(n), phase=np.full(n, "close", dtype="U24"), action_index=np.zeros(n, dtype=int),
        task_object_states=np.tile([0., 0., .1, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.], (n, 1, 1)))
    meta = dict(task_object_names=["banana"], physics_dt_s=1/120)
    contacts = []
    for i in range(n):
        finger = dict(count=1, points_world_m=[[0., 0., .1]], points_environment_m=[[0., 0., .1]],
            points_object_local_m=[[0., 0., 0.]], normals_world=[[0., 0., 1.]], normals_object_local=[[0., 0., 1.]],
            normal_forces_n=[.02], separations_m=[0.])
        contacts.append(dict(object_name="banana", physics_step=i, phase="close", action_index=0,
            physics_dt_s=1/120, fingers={name: deepcopy(finger) for name in FINGERS}))
    return data, meta, contacts


def check_contact(data, meta, contacts):
    return contact_window(data, meta, contacts, "banana", [[-.01]*3, [.01]*3], "close")


def test_ten_actual_loaded_steps_pass_but_nine_or_unilateral_do_not():
    data, meta, contacts = contacts_fixture()
    assert check_contact(data, meta, contacts)["passed"]
    contacts[-1]["fingers"][FINGERS[0]]["normal_forces_n"] = [.01]
    assert not check_contact(data, meta, contacts)["passed"]
    data, meta, contacts = contacts_fixture(9)
    assert not check_contact(data, meta, contacts)["passed"]


def test_negative_contact_force_and_out_of_region_loaded_point_fail():
    data, meta, contacts = contacts_fixture()
    contacts[-1]["fingers"][FINGERS[0]]["normal_forces_n"] = [-2.]
    assert not check_contact(data, meta, contacts)["passed"]
    data, meta, contacts = contacts_fixture()
    finger = contacts[-1]["fingers"][FINGERS[0]]
    finger["points_world_m"][0][0] = finger["points_object_local_m"][0][0] = .02
    assert not check_contact(data, meta, contacts)["passed"]


def test_contact_row_identity_or_object_frame_tampering_raises():
    data, meta, contacts = contacts_fixture()
    contacts[-1]["physics_step"] -= 1
    with pytest.raises(ValueError, match="identity"):
        check_contact(data, meta, contacts)
    data, meta, contacts = contacts_fixture()
    contacts[-1]["fingers"][FINGERS[0]]["points_object_local_m"] = [[0., .002, 0.]]
    with pytest.raises(ValueError, match="frame mismatch"):
        check_contact(data, meta, contacts)


def test_final_original_support_and_strict_speed_boundaries():
    state = np.array([0., 0., .01, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.])
    destination = np.array([0., 0., 0., 0., 0., 0., 1.])
    def run(force):
        return raw_destination(state, np.array(force), np.zeros(3), destination, [[-.1, -.1, 0.], [.1, .1, .2]])
    assert run([0., 0., .1])["passed"]
    assert not run([0., 0., np.nextafter(.1, 0.)])["passed"]
    assert not run([0., 0., -.1])["passed"]
    state[7] = .003
    assert not run([0., 0., .1])["passed"]
    state[7] = np.nextafter(.003, 0.)
    assert run([0., 0., .1])["passed"]
    state[0] = .100001
    assert not run([0., 0., .1])["passed"]


def test_closed_fingers_with_unexecuted_lift_cannot_be_task_success():
    data, meta, _ = contacts_fixture(12)
    meta.update(task_id=next(iter(TASKS)), joint_names=["finger_joint"])
    data["phase"][:2] = "initial_settle"
    data["q"] = np.zeros((12, 1))
    trajectory = dict(actions=np.zeros((2, 8)), phase=np.full(2, "close"))
    report = sequence_audit(data, meta, trajectory, dict(original_task_success=True),
        dict(passed=True, steps=[dict(physics_step=2)]), dict(passed=False))
    assert not report["passed"]
    assert report["lift_physics_samples"] == 0
    assert report["maximum_actual_lift_m"] is None
    assert not report["checks"]["physical_close_lift_release_order"]


def test_startup_summary_cannot_hide_missing_or_changed_prefix(tmp_path):
    data = dict(phase=np.array(["reset", "warmup", "initial_settle", "close"]), physics_step=np.arange(4), q=np.zeros((4, 1)))
    meta = dict(task_object_names=["banana"], physics_dt_s=3.)
    startup = dict(task_object_names=["banana"], first_physics_step=0, last_physics_step=2, samples=3, continuous=True, duration_s=6.)
    (tmp_path/"startup_audit.json").write_text(json.dumps(startup))
    np.savez(tmp_path/"startup_physics_states.npz", **{k: v[:3] for k,v in data.items()})
    assert prefix_audit(tmp_path, meta, data)["passed"]
    data["q"][1, 0] = .1
    with pytest.raises(ValueError, match="differs"):
        prefix_audit(tmp_path, meta, data)


def test_timeline_rejects_action_phase_shift():
    data = dict(q=np.zeros((3, 1)), phase=np.array(["reset", "close", "close"]), action_index=np.array([-1, 0, 0]))
    meta = dict(physics_dt_s=.01, action_dt_s=.02)
    trajectory = dict(actions=np.zeros((1, 8)), phase=np.array(["close"]), before_physics_step=np.array([0]),
        after_physics_step=np.array([2]), camera_physics_steps=np.array([[0, 0]]))
    actions = [dict(action=[0.]*8, phase="close", before_physics_step=0, after_physics_step=2, camera_physics_steps=[0, 0])]
    assert timeline_audit(data, meta, trajectory, actions)["passed"]
    data["phase"][-1] = "lift"
    with pytest.raises(ValueError, match="substep phase"):
        timeline_audit(data, meta, trajectory, actions)


def test_green_self_sidecar_without_input_binding_cannot_pass(tmp_path):
    for name in ("physics_states.npz", "physics_metadata.json", "trajectory.npz"):
        (tmp_path/name).write_text("raw")
    path = tmp_path/"self.json"
    path.write_text(json.dumps(dict(passed=True, kind="self-collision", input_sha256={})))
    with pytest.raises(ValueError, match="identity"):
        bound_sidecar(path, tmp_path, "self-collision", dict(q=np.zeros((1, 13))), dict())
