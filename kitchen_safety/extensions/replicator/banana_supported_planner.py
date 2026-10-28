"""Source-lift planner with explicit cabinet-clearing, bowl-enabled retreat."""
import time
from kitchen_safety.extensions.replicator.source_support_curobo import SourceSupportCuroboPlanner
from kitchen_safety.curobo_backend import CuroboPlanner

class SupportedBananaPlanner(SourceSupportCuroboPlanner):
    def _prepare_path_query(self,ignore_names,grip_binary,phase):
        if phase!='retreat':return super()._prepare_path_query(ignore_names,grip_binary,phase)
        if ignore_names or grip_binary!=0 or self.world.attachment is not None:
            raise ValueError('Retreat requires open command, no caller masks and free actual banana')
        self.world.set_phase(phase)
        if self.world.planning_certificate is None:raise ValueError('Retreat certificate must precede CUDA query')
        # Original necessary finger/banana candidate mask remains, with complete
        # source FCL refinement. Unlike the old candidate policy, bowl stays on.
        masks=('banana',)
        self.candidate_contact_mask=dict(phase=phase,object_switches=list(masks),
            full_fcl_refinement_required=True,physical_collision_filter_changes=False,
            original_destination_cuda_enabled=True)
        return CuroboPlanner._prepare_path_query(self,masks,grip_binary,phase)

    def _cartesian_vertical(self,q_arm7,target,phase):
        if phase!='retreat':
            return super()._cartesian_vertical(q_arm7,target,phase)
        from .banana_retreat_motion import retreat_candidates,cartesian_retreat_candidate
        from .task_plan_diagnostics import constraint_components
        started=time.perf_counter()
        # Fixed shortest candidate selected by the retained six-candidate CPU
        # and CUDA probe, not a relaxed collision gate or hidden target change.
        description=retreat_candidates(self.kin,q_arm7,target)[0]
        if description['id']!='out08_lift04':
            raise ValueError('Unexpected declared retreat selection')
        candidate=cartesian_retreat_candidate(self.kin,q_arm7,description,self.dt,self.max_joint_speed)
        self.last_cartesian_candidate=candidate
        feasible=self.motion_gen.ik_solver.check_valid(self.args.to_device(candidate['positions']))
        if not bool(feasible.all()):
            self.last_constraint_diagnostics,_=constraint_components(self,candidate['positions'])
            rejected={name:row['nonzero_samples'] for name,row in self.last_constraint_diagnostics['components'].items()
                      if row['nonzero_samples']}
            raise RuntimeError('cuRobo rejected explicit cabinet-clearing retreat: '+str(rejected))
        self.last_plan=dict(backend='curobo',method='cartesian_ik_three_segment_retreat',solver='usd_scipy_ik',
            status='SUCCESS',collision_geometry=self.collision_geometry,planning_seconds=time.perf_counter()-started,
            samples=len(candidate['positions']),dt=self.dt,gripper_joint_position=self._last_grip,
            attached_payload=False,orientation_mode='hold_current',original_destination_cuda_enabled=True,
            selection_rule='Fixed out08_lift04; shortest outward displacement and initial lift among six original-gate-passing recorded-state candidates',
            selection_evidence='reports/replicator_safety_upgrade/g_banana_retreat_search_gpu01/diagnostic.json',
            **candidate['metadata'])
        # The caller still performs dense original CUDA + full source-FCL
        # validation and then checks every actual physics substep before success.
        return dict(positions=candidate['positions'],dt=self.dt,metadata=self.last_plan,
                    effective_goal_pose=candidate['effective_goal_pose'])
