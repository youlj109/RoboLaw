"""Startup evidence must expose the previously invisible warmup and substeps."""
import copy
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from kitchen_safety.extensions.replicator.startup_recorder import StartupRecorder, audit_startup


def evidence():
    n = 56
    def poses(shape):
        values = np.zeros(shape)
        values[..., 6] = 1.
        return values
    data = dict(physics_step=np.arange(100, 100 + n), q=np.zeros((n, 13)), dq=np.zeros((n, 13)),
                mustard=poses((n, 13)), bowl=poses((n, 13)), hazard_state=poses((n, 13)),
                has_hazard_state=np.ones(n, dtype=bool), hazard_pose=poses((n, 7)),
                has_hazard_pose=np.ones(n, dtype=bool),
                background_rigid_poses=poses((n, 2, 7)), background_rigid_velocities=np.zeros((n, 2, 6)),
                robot_body_pose_w=poses((n, 1, 7)), static_anchor_transforms=np.empty((n, 0, 4, 4)),
                startup_stage=np.asarray(["reset_initial"] + ["warmup"] * 8 + ["initial_settle"] * (n - 9)))
    metadata = dict(reset_physics_step=100, world_ready_physics_step=155, physics_dt_s=.1,
                    joint_names=["j" + str(i) for i in range(13)], robot_body_names=["robot"],
                    background_paths=["jointed", "free"], static_anchor_paths=[], dynamic_hazard=True,
                    hazard_kind="dynamic", callback_errors=[], background_topology=dict(
                        joint_connected_native_paths=["jointed"], unobserved_joint_connected_bodies=[],
                        static_anchor_paths=[], animated_static_anchors=[]))
    return data, metadata


class StartupRecorderTest(unittest.TestCase):
    def test_contiguous_prefix_passes_but_gap_or_missing_reset_cannot_be_inferred(self):
        data, metadata = evidence()
        self.assertTrue(audit_startup(data, metadata)["passed"])
        data["physics_step"][3] += 1
        report = audit_startup(data, metadata)
        self.assertFalse(report["passed"])
        self.assertTrue(report["coverage"]["gaps"])
        data, metadata = evidence()
        data["startup_stage"][0] = "warmup"
        self.assertFalse(audit_startup(data, metadata)["passed"])

    def test_early_dynamic_hazard_excursion_cannot_hide_after_settling(self):
        data, metadata = evidence()
        data["hazard_state"][3, 0] = .003
        self.assertFalse(audit_startup(data, metadata)["checks"]["dynamic_hazard_stable_from_reset"])
        data, metadata = evidence()
        data["has_hazard_state"][2] = False
        with self.assertRaises(ValueError):
            audit_startup(data, metadata)

    def test_only_free_background_gets_five_second_settling_window(self):
        data, metadata = evidence()
        data["background_rigid_poses"][3, 1, 0] = .01
        self.assertTrue(audit_startup(data, metadata)["passed"])
        data["background_rigid_poses"][3, 0, 0] = .01
        self.assertFalse(audit_startup(data, metadata)["checks"]["background_stable"])
        data, metadata = evidence()
        data["background_rigid_poses"][52, 1, 0] = .003
        self.assertFalse(audit_startup(data, metadata)["passed"])

    def test_missing_topology_nonfinite_fields_or_unreached_world_rejected(self):
        data, metadata = evidence()
        metadata["background_topology"]["unobserved_joint_connected_bodies"] = ["missing_door"]
        self.assertFalse(audit_startup(data, metadata)["passed"])
        data, metadata = evidence()
        metadata["world_ready_physics_step"] = None
        self.assertFalse(audit_startup(data, metadata)["passed"])
        data["dq"][2, 1] = np.nan
        with self.assertRaises(ValueError):
            audit_startup(data, metadata)

    def test_observer_does_not_step_or_change_original_callback_count(self):
        from pxr import Usd
        data, metadata = evidence()
        sim = SimpleNamespace(count=100, stage=Usd.Stage.CreateInMemory())
        sim.get_physics_step_count = lambda: sim.count
        robot = SimpleNamespace(joint_names=metadata["joint_names"], body_names=["robot"], data=SimpleNamespace(
            joint_pos=data["q"][:1], joint_vel=data["dq"][:1],
            body_pos_w=np.zeros((1, 1, 3)), body_quat_w=np.asarray([[[0., 0., 0., 1.]]])))
        view = SimpleNamespace(prim_paths=["jointed", "free"],
                               get_transforms=lambda: data["background_rigid_poses"][0],
                               get_velocities=lambda: data["background_rigid_velocities"][0])
        calls = []
        class Scene:
            def __init__(self):
                self.items = dict(robot=robot, mustard_bottle=SimpleNamespace(data=SimpleNamespace(root_state_w=data["mustard"][:1])),
                                  bowl=SimpleNamespace(data=SimpleNamespace(root_state_w=data["bowl"][:1])),
                                  robot_contacts=SimpleNamespace(_physics_sim_view=SimpleNamespace(create_rigid_body_view=lambda _: view)))
            def __getitem__(self, key):
                return self.items[key]
            def update(self, dt):
                calls.append(dt)
        scene = Scene()
        env = SimpleNamespace(scene=scene, sim=sim, physics_dt=.1, step_dt=.8)
        with tempfile.TemporaryDirectory() as tmp, patch(
                "kitchen_safety.extensions.replicator.startup_recorder.background_topology",
                return_value=copy.deepcopy(metadata["background_topology"])):
            recorder = StartupRecorder(env, {}, tmp, lambda: ("settle", 0))
            self.assertEqual(sim.count, 100)
            recorder.set_stage("warmup", 0)
            scene.update(0.)  # A no-physics refresh cannot masquerade as a substep.
            for _ in range(55):
                sim.count += 1
                scene.update(.1)
            self.assertEqual(len(calls), 56)
            self.assertEqual(len(recorder.rows), 56)
            world = SimpleNamespace(objects=[], _matches=lambda *_: False, attachment=None,
                                    last_collision=None, last_clearance=None, sync_background=lambda *_: None,
                                    valid=lambda *args, **kwargs: True)
            report = recorder.finish(world_ready=True, world=world, arm_ids=np.arange(7))
            self.assertTrue(report["passed"])
            self.assertEqual(sim.count, 155)
            self.assertIs(recorder.finish(), report)
            scene.update(.1)
            self.assertEqual(len(recorder.rows), 56)


if __name__ == "__main__":
    unittest.main()
