"""CPU tests for independent task evidence audits; no Kit or GPU."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace, ModuleType

import h5py
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scripts.extensions.replicator.task_audit_sidecars import (
    ROOT, cameras_audit, fixed_audit, load_episode, self_collision_audit, sha, source_audit, source_camera_config)


def source_states():
    layout = json.loads((ROOT/"configs/extensions/replicator/g_shape_banana_bowl_source_layout.json").read_text())
    meta = dict(source_layout=layout, robot_body_names=["panda_link0"], task_object_names=list(layout["objects"]),
        source_hashes={name: sha(ROOT/item["usd_path"]) for name, item in layout["objects"].items()}, background_paths=[])
    base = np.r_[np.asarray(layout["robot_position"])+[0., 0., layout["stand_height"]],
                 Rotation.from_euler("z", layout["robot_yaw_deg"], degrees=True).as_quat()]
    objects = np.stack([np.r_[item["position"], Rotation.from_euler("z", item["yaw_deg"], degrees=True).as_quat(), np.zeros(6)]
                        for item in layout["objects"].values()])
    data = dict(robot_body_pose_w=np.tile(base, (2, 1, 1)), task_object_states=np.tile(objects, (2, 1, 1)),
        static_anchor_transforms=np.zeros((2, 0, 4, 4)), task_reference_transforms=np.zeros((2, 0, 4, 4)),
        physics_step=np.arange(2), background_rigid_poses=np.zeros((2, 0, 7)))
    return meta, data


def test_actual_base_drift_and_source_object_change_cannot_pass_relations_only():
    metadata, data = source_states()
    assert source_audit(metadata, data)["passed"]
    data["robot_body_pose_w"][1, 0, 0] += .001
    assert not source_audit(metadata, data)["passed"]
    metadata, data = source_states()
    data["task_object_states"][0, 0, 0] += .01
    assert not source_audit(metadata, data)["passed"]


def test_real_source_fixed_stand_in_full_g_kitchen_is_checked_without_kit():
    metadata, data = source_states()
    report = fixed_audit(metadata, data)
    assert report["passed"], report["failures"]
    assert report["checked_samples"] == 2
    # The source collision cube starts 54.56 mm above the floor, although the
    # visible stand and source relation bounds reach it. No support exemption
    # is needed for these actual colliders.
    assert report["allowed_exact_stand_floor_support_pairs"] == 0
    # Move the physical banana onto the actual stand: it is not a source layout
    # change, and the per-sample fixed-object pair must catch it independently.
    data["task_object_states"][1, 0, :3] = [metadata["source_layout"]["robot_position"][0], 1.2, .795]
    invalid = fixed_audit(metadata, data)
    assert not invalid["passed"]
    assert any("banana" in row["obstacle"] for row in invalid["failures"])
    # The former relation-valid x=-2.1 layout intersects west cabinets. Keep
    # this regression independently of the corrected canonical candidate.
    metadata, data = source_states()
    metadata["source_layout"]["robot_position"][0] = -2.1
    data["robot_body_pose_w"][:, 0, 0] = -2.1
    invalid = fixed_audit(metadata, data)
    assert not invalid["passed"]
    assert any("Base_west_02" in row["obstacle"] for row in invalid["failures"])


def camera_episode(tmp_path):
    from kitchen_safety.dataset import CAMERA_NAMES, RGB_ALIGNMENT, camera_poses_from_body_states
    source = source_camera_config()
    metadata = dict(robot_body_names=["panda_link0", "base_link"], action_dt_s=2.)
    body = np.zeros((5, 2, 7))
    body[:, :, 6] = 1.
    body[:, 1, 0] = np.arange(5)*.01
    data = dict(physics_step=np.arange(5), robot_body_pose_w=body)
    trajectory = dict(before_physics_step=np.array([0, 2]), camera_physics_steps=np.array([[0, 0], [2, 2]]))
    recorded = {}
    for name in CAMERA_NAMES:
        config = source[name]
        spawn = config["spawn"]
        recorded[name] = dict(prim_path=config["prim_path"], width=config["width"], height=config["height"],
            intrinsics=[[config["width"]*spawn["focal_length"]/spawn["horizontal_aperture"], 0., 640.],
                [0., config["height"]*spawn["focal_length"]/spawn["vertical_aperture"], 360.], [0., 0., 1.]],
            offset=dict(position=config["offset"]["pos"], quaternion_xyzw=config["offset"]["rot"], convention="opengl"))
    (tmp_path/"cameras.json").write_text(json.dumps(recorded))
    expected = camera_poses_from_body_states(body[[0, 2]], metadata["robot_body_names"], recorded)
    with h5py.File(tmp_path/"images.h5", "w") as images:
        images.attrs.update(frames=2, dt=2., alignment=RGB_ALIGNMENT)
        for name in CAMERA_NAMES:
            group = images.create_group(name)
            group.create_dataset("jpeg", data=np.zeros((2, 1), dtype=np.uint8))
            for field in ("position_world", "quaternion_world_ros_xyzw"):
                group.create_dataset(field, data=expected[name][field])
    return metadata, data, trajectory


def test_camera_source_mount_and_actual_wrist_pose_not_stale(tmp_path):
    metadata, data, trajectory = camera_episode(tmp_path)
    assert cameras_audit(tmp_path, metadata, data, trajectory)["passed"]
    with h5py.File(tmp_path/"images.h5", "r+") as images:
        images["wrist_camera/position_world"][1] = images["wrist_camera/position_world"][0]
    report = cameras_audit(tmp_path, metadata, data, trajectory)
    assert not report["passed"]
    assert report["cameras"]["wrist_camera"]["position_max_error_m"] == pytest.approx(.02)


def test_camera_capture_must_not_advance_physics(tmp_path):
    metadata, data, trajectory = camera_episode(tmp_path)
    trajectory["camera_physics_steps"][1, 1] += 1
    assert not cameras_audit(tmp_path, metadata, data, trajectory)["passed"]


def test_actual_six_finger_angles_are_not_replaced_by_mimic_or_clamp(monkeypatch):
    import kitchen_safety.curobo_backend as backend
    import kitchen_safety.planning as planning
    import yaml
    robot_dict = yaml.safe_load((ROOT/"assets/curobo/droid/franka_robotiq_2f_85_zero_curobo.yml").read_text())["robot_cfg"]
    robot_dict["kinematics"]["urdf_path"] = str(ROOT/"assets/curobo/droid/urdf/franka_robotiq_2f_85_zero.urdf")
    arm_names = [f"panda_joint{i}" for i in range(1, 8)]
    fingers = list(robot_dict["kinematics"]["lock_joints"])
    updates, arms = [], []
    def augmented(state):
        arms.append(state.position.copy())
        return SimpleNamespace(robot_spheres=state.position)
    rollout = SimpleNamespace(_get_augmented_state=augmented, robot_self_collision_constraint=SimpleNamespace(
        enabled=True, forward=lambda _: SimpleNamespace(max=lambda: SimpleNamespace(item=lambda: 0.))))
    fake = SimpleNamespace(robot_dict=robot_dict, args=SimpleNamespace(to_device=lambda value: value),
        motion_gen=SimpleNamespace(ik_solver=SimpleNamespace(joint_names=arm_names, rollout_fn=rollout),
            update_locked_joints=lambda values, _: updates.append(values)))
    monkeypatch.setattr(backend, "CuroboPlanner", lambda *args, **kwargs: fake)
    monkeypatch.setattr(planning, "DroidKinematics", lambda *args: None)
    stub = ModuleType("curobo.types.state")
    stub.JointState = SimpleNamespace
    monkeypatch.setitem(sys.modules, "curobo.types.state", stub)
    names = arm_names+fingers
    values = np.r_[np.array([0., 0., 0., -2., 0., 1., 0.]), np.arange(6)*.01]
    data = dict(q=np.stack([values, values.copy()]), physics_step=np.array([7, 8]), phase=np.array(["close", "close"]))
    data["q"][1, names.index("finger_joint")] = -.000003
    meta = dict(source_layout=dict(robot_position=[0., 0., 0.], stand_height=.8, robot_yaw_deg=0.),
                joint_names=names, action_dt_s=1/15)
    report = self_collision_audit(meta, data)
    assert report["passed"]
    assert len(updates) == 2
    assert updates[1]["finger_joint"] == -.000003
    for index, name in enumerate(fingers):
        assert updates[0][name] == values[7+index]
    assert report["maximum_joint_limit_excess_rad"] == pytest.approx(.000003)
    assert not report["mimic_angles_substituted"]
    np.testing.assert_array_equal(arms[0][0], values[:7])
