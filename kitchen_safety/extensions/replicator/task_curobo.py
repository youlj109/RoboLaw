"""cuRobo adapter with task-source payload spheres and measured gripper state."""
from copy import deepcopy
import time

import numpy as np

from kitchen_safety.curobo_backend import CuroboPlanner


class PayloadCuroboPlanner(CuroboPlanner):
    def _cartesian_vertical(self, q_arm7, target, phase):
        # Same public algorithm and gates, with the candidate retained before
        # check_valid can reject it. This is never a collision bypass/fallback.
        from .task_plan_diagnostics import local_cartesian_candidate, constraint_components
        started = time.perf_counter()
        candidate = local_cartesian_candidate(self.kin, q_arm7, target, phase, self.dt, self.max_joint_speed)
        self.last_cartesian_candidate = candidate
        feasible = self.motion_gen.ik_solver.check_valid(self.args.to_device(candidate["positions"]))
        if not bool(feasible.all()):
            self.last_constraint_diagnostics, _ = constraint_components(self, candidate["positions"])
            rejected = {name: row["nonzero_samples"] for name, row in self.last_constraint_diagnostics["components"].items()
                        if row["nonzero_samples"]}
            raise RuntimeError("cuRobo rejected local Cartesian constraints: " + str(rejected))
        self.last_plan = dict(backend="curobo", method="cartesian_ik", solver="usd_scipy_ik", status="SUCCESS",
            collision_geometry=self.collision_geometry, planning_seconds=time.perf_counter()-started,
            samples=len(candidate["positions"]), dt=self.dt, gripper_joint_position=self._last_grip,
            attached_payload=self.world.attachment is not None, vertical_motion_phase=phase,
            vertical_lowering=phase == "lower", orientation_mode="hold_current", **candidate["metadata"])
        return dict(positions=candidate["positions"], dt=self.dt, metadata=self.last_plan,
                    effective_goal_pose=candidate["effective_goal_pose"])

    def _prepare_path_query(self, ignore_names, grip_binary, phase):
        if ignore_names:
            raise ValueError(
                f"Caller masks {tuple(ignore_names)} are forbidden; selective contact policy derives candidate masks")
        self.world.set_phase(phase)
        candidate_masks = self.world.contact_policy.cuda_candidate_masks(phase, self.world.attachment is not None)
        self.candidate_contact_mask = dict(phase=phase, object_switches=list(candidate_masks),
            full_fcl_refinement_required=True, physical_collision_filter_changes=False)
        return super()._prepare_path_query(candidate_masks, grip_binary, phase)

    def _sync_gripper_and_attachment(self, grip_binary, object_padding_m=0.):
        measured = getattr(self.world, "gripper_joint_position", None)
        if grip_binary and measured is None:
            raise ValueError("Closed-gripper planning requires the actual measured finger_joint; no bottle fallback")
        grip = float(measured if measured is not None else 0.)
        if not np.isfinite(grip) or grip < -1e-5 or grip > np.pi / 4 + 1e-5:
            raise ValueError("Measured gripper joint is outside the original physical range")
        if self._last_grip is None or abs(grip - self._last_grip) > .002:
            locked = dict(self.robot_dict["kinematics"]["lock_joints"])
            for name, sign in zip(self.world.GRIPPER_JOINT_NAMES, (1., 1., -1., 1., -1., -1.)):
                locked[name] = sign * grip
            self.motion_gen.update_locked_joints(locked, deepcopy(self.robot_dict))
            self._last_grip = grip
        if self.world.attachment is None:
            self.motion_gen.detach_spheres_from_robot()
            return
        if not np.isfinite(object_padding_m) or object_padding_m < 0:
            raise ValueError("Payload padding must be finite and nonnegative")
        if not hasattr(self, "_payload_cover"):
            maximum = self.robot_dict["kinematics"]["extra_collision_spheres"]["attached_object"]
            self._payload_cover = self.world.payload_geometry.sphere_cover(maximum)
        centers, radii, provenance = self._payload_cover
        pose = self.T_ee_tcp @ self.world.attachment
        transformed = centers @ pose[:3, :3].T + pose[:3, 3]
        maximum = self.robot_dict["kinematics"]["extra_collision_spheres"]["attached_object"]
        spheres = np.zeros((maximum, 4), dtype=np.float32)
        spheres[:, 3] = -100.
        spheres[:len(centers), :3] = transformed
        spheres[:len(centers), 3] = radii + object_padding_m
        self.motion_gen.attach_spheres_to_robot(sphere_tensor=self.args.to_device(spheres))
        self.payload_geometry_report = dict(provenance, object_name=self.world.payload_name,
            source=self.world.payload_geometry.provenance, padding_m=float(object_padding_m))
