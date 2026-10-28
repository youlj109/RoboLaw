"""CPU geometry/contact/adapter-interface checks; no SimulationApp or CUDA."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry, PayloadCollisionWorld
from kitchen_safety.extensions.replicator.task_contacts import ObjectGraspContacts
from kitchen_safety.extensions.replicator.task_curobo import PayloadCuroboPlanner

ROOT = Path(__file__).resolve().parents[3]
ASSETS = [ROOT / "assets/extensions/replicator/source/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/ycb/banana.usd",
          ROOT / "assets/robolab/objects/ycb/bowl.usd"]


@pytest.mark.parametrize("asset", ASSETS)
def test_payload_cover_contains_source_surface_and_solid_interior(asset):
    geometry = PayloadGeometry(asset)
    centers, radii, _ = geometry.sphere_cover(100)
    assert 1 <= len(centers) <= 100
    # Includes all original vertices and deterministic convex combinations,
    # avoiding a surface-only test that would miss interior coverage holes.
    random = np.random.default_rng(42)
    indices = random.integers(len(geometry.vertices), size=(2000, 4))
    weight = random.dirichlet(np.ones(4), size=2000)
    interior = np.einsum("ni,nij->nj", weight, geometry.vertices[indices])
    points = np.concatenate([geometry.vertices, interior])
    gaps = np.linalg.norm(points[:, None] - centers[None], axis=2) - radii
    assert np.max(np.min(gaps, axis=1)) <= 1e-8


def test_fcl_payload_uses_rotated_source_bounds_instead_of_bottle_box():
    fcl = pytest.importorskip("fcl")
    world = PayloadCollisionWorld.__new__(PayloadCollisionWorld)
    world.fcl, world.payload_name = fcl, "banana"
    world.payload_geometry = PayloadGeometry(ASSETS[0])
    world.payload_model, world.payload_hull_vertices = world._convex(world.payload_geometry.vertices)
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", [40, 10, 80], degrees=True).as_matrix()
    pose[:3, 3] = [1.2, -.4, .95]
    shapes = []
    world._append_bottle_shape(shapes, np.zeros(7), pose)
    expected = world.payload_geometry.vertices @ pose[:3, :3].T + pose[:3, 3]
    assert len(shapes) == 1 and shapes[0][0] == "payload/banana"
    np.testing.assert_allclose(shapes[0][2], expected.min(0), atol=1e-9)
    np.testing.assert_allclose(shapes[0][3], expected.max(0), atol=1e-9)


class ContactView:
    def __init__(self, point, force=.2):
        self.point, self.force = np.asarray(point), force

    def get_contact_data(self, dt):
        return (np.array([self.force]), self.point[None], np.array([[1., 0., 0.]]),
                np.array([0.]), np.array([1]), np.array([0]))


def contacts(point, env_origin=None):
    patterns = []
    def create(roots, *, filter_patterns, max_contact_data_count):
        patterns.append(filter_patterns)
        return ContactView(point)
    recorder = ObjectGraspContacts(SimpleNamespace(create_rigid_contact_view=create),
        1 / 120, "banana", env_origin=env_origin)
    assert patterns == [[["/World/envs/env_0/banana"]]] * 2
    return recorder


def test_contacts_use_actual_object_frame_without_double_subtracting_environment_origin():
    pose = np.array([10., 20., .9, *Rotation.from_euler("z", 90, degrees=True).as_quat()])
    local = np.array([-.033, .01, .002])
    point = pose[:3] + Rotation.from_quat(pose[3:]).apply(local)
    recorder = contacts(point, [10., 20., 0.])
    row = recorder.sample("lift", 5, pose, physics_step=41)
    for finger in recorder.fingers:
        np.testing.assert_allclose(row["fingers"][finger]["points_object_local_m"], local[None], atol=1e-12)
    assert row["physics_step"] == 41


def test_contacts_do_not_merge_alternating_fingers_or_pass_partial_window():
    recorder = contacts([-.033, 0., 0.])
    pose = [0, 0, 0, 0, 0, 0, 1]
    recorder.views[recorder.fingers[0]].force = 0.
    recorder.sample("lift", 1, pose, physics_step=1)
    summary = recorder.summarize([-.06, -.04, -.025], [0., .04, .025], phase="lift", last_steps=2)
    assert not summary["both_fingers_loaded"] and not summary["both_fingers_near"]
    recorder.views[recorder.fingers[0]].force = .2
    recorder.views[recorder.fingers[1]].force = 0.
    recorder.sample("lift", 1, pose, physics_step=2)
    summary = recorder.summarize([-.06, -.04, -.025], [0., .04, .025], phase="lift", last_steps=2)
    assert not summary["both_fingers_loaded"]


def test_contacts_require_loaded_points_inside_declared_midbody_region():
    recorder = contacts([-.033, .07, 0.])
    recorder.sample("lift", 1, [0, 0, 0, 0, 0, 0, 1], physics_step=1)
    result = recorder.summarize([-.06, -.04, -.025], [0., .04, .025], phase="lift", last_steps=1)
    assert not result["both_fingers_loaded"]


def test_curobo_attachment_requires_measured_gripper_and_uses_source_spheres():
    planner = PayloadCuroboPlanner.__new__(PayloadCuroboPlanner)
    planner.world = SimpleNamespace(attachment=np.eye(4), payload_geometry=PayloadGeometry(ASSETS[0]),
        payload_name="banana", GRIPPER_JOINT_NAMES=PayloadCollisionWorld.GRIPPER_JOINT_NAMES)
    with pytest.raises(ValueError, match="actual measured"):
        planner._sync_gripper_and_attachment(1)
    planner.world.gripper_joint_position = .5
    planner.robot_dict = {"kinematics": {"lock_joints": {}, "extra_collision_spheres": {"attached_object": 100}}}
    planner._last_grip, planner.T_ee_tcp = None, np.eye(4)
    planner.args = SimpleNamespace(to_device=lambda value: value)
    saved = {}
    planner.motion_gen = SimpleNamespace(update_locked_joints=lambda locked, robot: saved.update(locked=locked),
        attach_spheres_to_robot=lambda sphere_tensor: saved.update(spheres=sphere_tensor))
    planner._sync_gripper_and_attachment(1, .005)
    assert saved["locked"]["finger_joint"] == .5
    assert saved["locked"]["left_inner_finger_joint"] == -.5
    centers, radii, _ = planner.world.payload_geometry.sphere_cover()
    np.testing.assert_allclose(saved["spheres"][:len(centers), :3], centers, atol=1e-7)
    np.testing.assert_allclose(saved["spheres"][:len(centers), 3], radii + .005, atol=1e-7)
    assert np.all(saved["spheres"][len(centers):, 3] < 0)


@pytest.mark.parametrize("mask", [("mustard_bottle",), ("banana",), ("bowl",)])
def test_old_whole_object_masks_fail_before_any_cuda_or_geometry_query(mask):
    planner = PayloadCuroboPlanner.__new__(PayloadCuroboPlanner)
    with pytest.raises(ValueError, match="selective contact policy"):
        planner._prepare_path_query(mask, 0, "approach")
    world = PayloadCollisionWorld.__new__(PayloadCollisionWorld)
    with pytest.raises(ValueError, match="Whole-object masks"):
        world._pairs(np.zeros(7), mask, "planning", None, None, None)


@pytest.mark.parametrize("variant", ["human", "property", "robot"])
def test_undefined_hazard_policy_rejected_before_simulator_import(variant):
    pytest.importorskip("isaaclab_arena.environments.arena_environment_factory")
    from kitchen_safety.extensions.replicator.task_environment import AssignedTaskEnvironment, AssignedTaskCfg
    with pytest.raises(NotImplementedError, match="No reviewed task-specific hazard policy"):
        AssignedTaskEnvironment().build(AssignedTaskCfg(variant=variant))
