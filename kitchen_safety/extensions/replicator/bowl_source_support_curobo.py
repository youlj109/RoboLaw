"""Bowl planner with two explicitly bound, separately checked support pairs.

Source departure and pre-release lowering refine only the corresponding
payload/support sphere pair. Every robot/world and payload/other-world query,
self/joint gate and source FCL remains mandatory. The original BasinBottom
never becomes a whole-object mask. GPU diagnostics and physical trials remain
pending; importing this module never starts CUDA or physics.
"""
from copy import deepcopy
import time

import numpy as np

from .task_curobo import PayloadCuroboPlanner
from .task_plan_diagnostics import local_cartesian_candidate
from kitchen_safety.curobo_backend import CuroboPlanner
from .bowl_source_support_query import bind_observed_departure, audit_source_departure
from .bowl_destination_query import bind_observed_lowering, audit_sink_lowering


class BowlSourceSupportCuroboPlanner(PayloadCuroboPlanner):
    departure_context = None
    lowering_context = None
    lowering_profile = None

    def _prepare_path_query(self,ignore_names,grip_binary,phase):
        if phase in ("lift", "lower") and grip_binary != 1:
            raise ValueError("Source departure requires the original closed-gripper command grip_binary=1")
        if ignore_names:
            raise ValueError("Bowl controller forbids caller obstacle masks")
        if self.world.payload_name != "bowl":
            raise ValueError("This independent planner only supports the original bowl payload")
        self.world.set_phase(phase)
        # Duplicate carried/source object only; BasinBottom, drain and every
        # wall remain in the original CUDA query. Full source FCL still checks
        # every individual robot/payload pair involving the bowl.
        masks = tuple(name for name in self.world.contact_policy.cuda_candidate_masks(
            phase, self.world.attachment is not None) if name == "bowl")
        self.candidate_contact_mask = dict(phase=phase, object_switches=list(masks),
            full_fcl_refinement_required=True, physical_collision_filter_changes=False,
            original_sink_destination_retained=True)
        return CuroboPlanner._prepare_path_query(self,masks,grip_binary,phase)

    def bind_source_departure(self, **observations):
        self.departure_context = bind_observed_departure(self.world,self.kin,**observations)
        return self.departure_context.evidence()

    def bind_sink_lowering(self, profile, **observations):
        self.lowering_profile = deepcopy(profile)
        self.lowering_context = bind_observed_lowering(self.world,self.kin,self.lowering_profile,**observations)
        return self.lowering_context.evidence()

    def _pair_audit(self, positions, phase):
        if phase == "lift":
            report = audit_source_departure(self,positions,self.departure_context)
            self.last_source_support_validation = report
        elif phase == "lower":
            if self.lowering_profile is None:
                raise RuntimeError("Sink lowering requires its explicitly bound controller profile")
            report = audit_sink_lowering(self,positions,self.lowering_profile,self.lowering_context)
            self.last_destination_validation = report
        else:
            raise ValueError("No support-pair refinement in phase "+str(phase))
        self.last_constraint_diagnostics = report["original_components"]
        return report

    def _cartesian_vertical(self,q_arm7,target,phase):
        if phase not in ("lift", "lower"):
            return super()._cartesian_vertical(q_arm7,target,phase)
        started = time.perf_counter()
        candidate = local_cartesian_candidate(self.kin,q_arm7,target,phase,self.dt,self.max_joint_speed)
        self.last_cartesian_candidate = candidate
        context = self.departure_context if phase == "lift" else self.lowering_context
        if context is None:
            raise RuntimeError("Support query requires actual measured-state/raw-contact binding")
        report = self._pair_audit(candidate["positions"],phase)
        if not report["valid"]:
            raise RuntimeError("Bound support-pair query rejected: "+str(report["mandatory_checks"])+
                "; actual_state_binding="+str(report["actual_state_binding"]["passed"]))
        self.last_plan = dict(backend="curobo",method="cartesian_ik_bound_support_pair",solver="usd_scipy_ik",status="SUCCESS",
            collision_geometry=self.collision_geometry,planning_seconds=time.perf_counter()-started,
            samples=len(candidate["positions"]),dt=self.dt,gripper_joint_position=self._last_grip,
            attached_payload=True,vertical_motion_phase=phase,vertical_lowering=phase=="lower",orientation_mode="hold_current",
            support_pair_validation=report,**candidate["metadata"])
        return dict(positions=candidate["positions"],dt=self.dt,metadata=self.last_plan,effective_goal_pose=candidate["effective_goal_pose"])

    def validate_path(self,positions,ignore_names=(),grip_binary=0,phase=None,resolution=.02):
        if phase not in ("lift", "lower"):
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
        report = self._pair_audit(samples,phase)
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
            gripper_joint_position=self._last_grip,support_pair_validation=report,
            sample_order="All original dense samples; original CUDA partition and all independent source FCL",
            validation_seconds=time.perf_counter()-started)
