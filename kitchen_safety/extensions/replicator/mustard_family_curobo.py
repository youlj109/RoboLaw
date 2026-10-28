"""Isolated collector planner for one actual source-bound family mustard departure contact.

All other phases delegate unchanged to PayloadCuroboPlanner. There is no bowl,
destination or arbitrary object-mask exception. This module is unvalidated on
GPU until independently diagnosed; importing it never starts CUDA or physics.
"""
import time

import numpy as np

from .task_curobo import PayloadCuroboPlanner
from .task_plan_diagnostics import local_cartesian_candidate
from .mustard_family_source_support import bind_observed_departure, audit_source_departure


class MustardSourceSupportPlanner(PayloadCuroboPlanner):
    def __init__(self, world, kin, *, preparation, **kwargs):
        from .mustard_family_source_support import source_support
        world._mustard_departure_preparation = preparation
        source_support(world)
        super().__init__(world, kin, **kwargs)

    departure_context = None

    def _prepare_path_query(self,ignore_names,grip_binary,phase):
        if phase == "lift" and grip_binary != 1:
            raise ValueError("Source departure requires the original closed-gripper command grip_binary=1")
        return super()._prepare_path_query(ignore_names,grip_binary,phase)

    def bind_source_departure(self, **observations):
        self.departure_context = bind_observed_departure(self.world,self.kin,**observations)
        return self.departure_context.evidence()

    def _cartesian_vertical(self,q_arm7,target,phase):
        if phase != "lift":
            return super()._cartesian_vertical(q_arm7,target,phase)
        started = time.perf_counter()
        candidate = local_cartesian_candidate(self.kin,q_arm7,target,phase,self.dt,self.max_joint_speed)
        self.last_cartesian_candidate = candidate
        if self.departure_context is None:
            raise RuntimeError("Source lift requires actual measured-state/raw-contact binding")
        report = audit_source_departure(self,candidate["positions"],self.departure_context)
        self.last_source_support_validation = report
        self.last_constraint_diagnostics = report["original_components"]
        if not report["valid"]:
            raise RuntimeError("Source-pair departure rejected: "+str(report["mandatory_checks"])+
                "; actual_state_binding="+str(report["actual_state_binding"]["passed"]))
        self.last_plan = dict(backend="curobo",method="cartesian_ik_source_support_pair",solver="usd_scipy_ik",status="SUCCESS",
            collision_geometry=self.collision_geometry,planning_seconds=time.perf_counter()-started,
            samples=len(candidate["positions"]),dt=self.dt,gripper_joint_position=self._last_grip,
            attached_payload=True,vertical_motion_phase=phase,vertical_lowering=False,orientation_mode="hold_current",
            source_support_validation=report,**candidate["metadata"])
        return dict(positions=candidate["positions"],dt=self.dt,metadata=self.last_plan,effective_goal_pose=candidate["effective_goal_pose"])

    def validate_path(self,positions,ignore_names=(),grip_binary=0,phase=None,resolution=.02):
        if phase != "lift":
            return super().validate_path(positions,ignore_names,grip_binary,phase,resolution)
        started = time.perf_counter()
        path = self._path_array(positions)
        if not np.isfinite(resolution) or resolution <= 0.:
            raise ValueError("Require positive finite path resolution")
        padding = self._prepare_path_query(ignore_names,grip_binary,phase)
        samples,edges,fractions = [path[0]],[0],[0.]
        for edge,(a,b) in enumerate(zip(path[:-1],path[1:])):
            count = max(2,int(np.ceil(np.max(np.abs(b-a))/resolution))+1)
            for fraction in np.linspace(0.,1.,count)[1:]:
                samples.append(a+(b-a)*fraction)
                edges.append(edge)
                fractions.append(float(fraction))
        samples = np.asarray(samples)
        report = audit_source_departure(self,samples,self.departure_context)
        self.last_source_support_validation = report
        self.last_constraint_diagnostics = report["original_components"]
        original = report["original_components"]["components"]
        parts = report["original_world_pair_partition"]["rows"]
        rejected = sorted(set(original["joint_bound"]["nonzero_samples"]+original["self_collision"]["nonzero_samples"]+
            parts["robot_all_world"]["nonzero_samples"]+parts["payload_non_support"]["nonzero_samples"]))
        fcl_failures = [dict(sample=row["sample"],edge=edges[row["sample"]],fraction=fractions[row["sample"]],
            measurement=row["measurement"]) for row in report["all_source_fcl"]["rows"] if not row["valid"]]
        return dict(valid=report["valid"],samples=len(samples),input_waypoints=len(path),resolution_rad=float(resolution),
            curobo_failures=[dict(sample=i,edge=edges[i],fraction=fractions[i]) for i in rejected],fcl_failures=fcl_failures,
            original_curobo_world_rejections=original["world_collision"]["nonzero_samples"],
            curobo_samples_checked=len(samples),fcl_samples_checked=len(samples),fcl_samples_unchecked=0,
            validation_complete=True,early_rejection=None,transport_object_padding_m=padding,
            gripper_joint_position=self._last_grip,source_support_validation=report,
            sample_order="All original dense samples; original CUDA partition and all independent source FCL",
            validation_seconds=time.perf_counter()-started)
