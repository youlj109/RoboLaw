"""Fresh observed bowl transport, retaining original CUDA and source FCL gates.

Only transport uses all six measured independent finger angles. All other
phases retain BowlSourceSupportCuroboPlanner behavior. No query changes world
clearances, physical contact, source-support refinement or sink-floor policy.
"""
from copy import deepcopy
import time

import numpy as np

from .bowl_source_support_curobo import BowlSourceSupportCuroboPlanner
from .bowl_transport_binding import TransportBinding, transport_binding_check
from .task_plan_diagnostics import constraint_components


class BowlTransportCuroboPlanner(BowlSourceSupportCuroboPlanner):
    transport_context = None
    transport_profile = None

    def bind_transport(self, profile, context):
        if not isinstance(context, TransportBinding):
            raise ValueError("Transport requires the measured lift-end binding")
        self.transport_profile, self.transport_context = deepcopy(profile), context
        return self._transport_check([context.arm_q])

    def _transport_check(self, positions):
        report = transport_binding_check(self.world, positions, self.transport_profile, self.transport_context)
        if not report["passed"]:
            raise ValueError("Observed transport binding changed: "+str(report))
        return report

    def _sync_gripper_and_attachment(self, grip_binary, object_padding_m=0.):
        if self.world.phase != "transport":
            return super()._sync_gripper_and_attachment(grip_binary, object_padding_m)
        if grip_binary != 1 or self.world.attachment is None:
            raise ValueError("Observed bowl transport requires a closed attached planning query")
        self._transport_check([self.transport_context.arm_q] if self.transport_context else [])
        fingers = np.asarray(self.world.gripper_joint_positions, float)
        # update_locked_joints replaces kinematic collision data, including
        # attached-sphere storage. Therefore update first, then call the parent
        # attachment builder again. Its coupled-finger branch is skipped only
        # after the real six values have actually been installed below.
        signature = tuple(map(float, fingers))
        previous = getattr(self, "_transport_six_signature", None)
        if previous != signature:
            locked = deepcopy(self.robot_dict["kinematics"]["lock_joints"])
            locked.update(dict(zip(self.world.GRIPPER_JOINT_NAMES, signature)))
            self.motion_gen.update_locked_joints(locked, deepcopy(self.robot_dict))
            if previous is not None:
                self.motion_gen.reset(reset_seed=False)
            self._transport_six_signature = signature
        self._last_grip = float(fingers[0])
        super()._sync_gripper_and_attachment(grip_binary, object_padding_m)
        self.payload_geometry_report.update(measured_six_finger_joints=dict(zip(self.world.GRIPPER_JOINT_NAMES, signature)),
            measured_tcp_to_payload=self.world.attachment.tolist(),
            attachment_binding_sha256=self.transport_context.evidence()["evidence_sha256"],
            actual_six_finger_scope="transport only", rigid_grasp_verified=False)

    def _prepare_path_query(self, ignore_names, grip_binary, phase):
        if phase == "transport":
            self.world.set_phase(phase)
            if self.transport_context is None:
                raise ValueError("Cannot query transport without fresh observation binding")
            self._transport_check([self.transport_context.arm_q])
        return super()._prepare_path_query(ignore_names, grip_binary, phase)

    def plan_pose(self, q_arm7, target, ignore_names=(), grip_binary=0, phase=None):
        if phase == "transport":
            self.world.set_phase(phase)
            self._transport_check(np.asarray(q_arm7)[None])
        return super().plan_pose(q_arm7, target, ignore_names, grip_binary, phase)

    def validate_path(self, positions, ignore_names=(), grip_binary=0, phase=None, resolution=.02):
        if phase != "transport":
            return super().validate_path(positions, ignore_names, grip_binary, phase, resolution)
        started = time.perf_counter()
        path = self._path_array(positions)
        if not np.isfinite(resolution) or resolution <= 0.:
            raise ValueError("Require positive finite dense path resolution")
        self.world.set_phase(phase)
        binding = self._transport_check(path)
        padding = self._prepare_path_query(ignore_names, grip_binary, phase)
        samples, edges, fractions = [path[0]], [0], [0.]
        for edge, (a, b) in enumerate(zip(path[:-1], path[1:])):
            count = max(2, int(np.ceil(np.max(np.abs(b-a))/resolution))+1)
            for fraction in np.linspace(0., 1., count)[1:]:
                samples.append(a+(b-a)*fraction)
                edges.append(edge)
                fractions.append(float(fraction))
        samples = np.asarray(samples)
        self.last_transport_dense_samples = samples.copy()
        original, _ = constraint_components(self, samples)
        rows = []
        # Check every original dense source-geometry sample even if CUDA has
        # already rejected the candidate. This is independent diagnostic
        # evidence; neither side can overrule a failure on the other side.
        for index, q in enumerate(samples):
            valid = self.world.valid(q, gripper_q=self.world.gripper_joint_positions)
            rows.append(dict(sample=index, valid=bool(valid), measurement=deepcopy(self.world.last_clearance)))
        components_zero = all(not row["nonzero_samples"] for row in original["components"].values())
        report = dict(valid=original["valid"] and components_zero and all(row["valid"] for row in rows),
            samples=len(samples), input_waypoints=len(path), resolution_rad=float(resolution),
            curobo_failures=[dict(sample=i, edge=edges[i], fraction=fractions[i]) for i in original["rejected_samples"]],
            fcl_failures=[dict(sample=row["sample"], edge=edges[row["sample"]], fraction=fractions[row["sample"]],
                measurement=row["measurement"]) for row in rows if not row["valid"]],
            curobo_samples_checked=len(samples), fcl_samples_checked=len(samples), fcl_samples_unchecked=0,
            validation_complete=True, original_cuda_components=original, all_source_fcl=dict(rows=rows, checked=len(rows)),
            actual_transport_binding=binding, transport_object_padding_m=padding,
            gripper_joint_position=self._last_grip, actual_six_finger_joints=list(self.world.gripper_joint_positions),
            candidate_contact_mask=self.candidate_contact_mask,
            payload_geometry=self.payload_geometry_report, physical_state_written=False, previous_plan_reused=False,
            sample_order="Every original dense sample: unchanged CUDA joint/self/world plus independent full source FCL",
            validation_seconds=time.perf_counter()-started)
        self.last_transport_validation = report
        return report
