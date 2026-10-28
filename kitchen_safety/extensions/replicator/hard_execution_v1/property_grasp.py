"""Native side-grasp trial, with original cuRobo/self and source-part checks."""
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.task_curobo import PayloadCuroboPlanner
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from kitchen_safety.extensions.replicator.task_plan_diagnostics import constraint_components
from .property_geometry import PropertyGlassWorld,grasp_target,straight_candidate,upright_transport_candidate,ENV_ROOT
from .property_environment import ROOT


def finger_force_window(records,region,minimum_force_n,maximum_force_n):
    """Summarize observed normal loads; this is not a glass fracture model."""
    low,high=np.asarray(region[0]),np.asarray(region[1]);steps=[]
    for record in records:
        fingers={}
        for name,entry in record['fingers'].items():
            force=np.asarray(entry['normal_forces_n'],dtype=float).reshape(-1)
            separation=np.asarray(entry['separations_m'],dtype=float).reshape(-1)
            points=np.asarray(entry['points_object_local_m'],dtype=float).reshape(-1,3)
            inside=np.all((points>=low)&(points<=high),axis=1)
            valid=(force>.01)&(separation<=.00025)&inside
            fingers[name]=dict(total_normal_force_n=float(force.sum()),
                loaded_near_wall_normal_force_n=float(force[valid].sum()),
                meets_minimum=bool(force[valid].sum()>=minimum_force_n),
                within_maximum=bool(force.sum()<=maximum_force_n))
        steps.append(dict(physics_step=record['physics_step'],fingers=fingers,
            bilateral_minimum=bool(len(fingers)==2 and all(v['meets_minimum'] for v in fingers.values())),
            within_maximum=bool(all(v['within_maximum'] for v in fingers.values()))))
    return dict(minimum_per_finger_n=minimum_force_n,maximum_per_finger_n=maximum_force_n if np.isfinite(maximum_force_n) else None,
        sampled_steps=len(steps),steps=steps,
        all_bilateral_minimum=bool(steps and all(v['bilateral_minimum'] for v in steps)),
        all_within_maximum=bool(all(v['within_maximum'] for v in steps)),
        scope='Actual contact normal forces on original declared glass wall region; not friction-cone or fracture proof')


class PropertyPlanner(PayloadCuroboPlanner):
    def plan_pose(self,q_arm7,target,ignore_names=(),grip_binary=0,phase=None):
        if phase=='transport' and getattr(self.world,'upright_transport',False):
            self._prepare_path_query(ignore_names,grip_binary,phase)
            candidate=upright_transport_candidate(self.kin,q_arm7,target,self.world.attachment,dt=self.dt,speed=self.max_joint_speed,
                tilt_limit_deg=getattr(self.world,'carry_tilt_limit_deg',3.))
            self.last_cartesian_candidate=candidate
            feasible=self.motion_gen.ik_solver.check_valid(self.args.to_device(candidate['positions']))
            if not bool(feasible.all()):
                self.last_constraint_diagnostics,_=constraint_components(self,candidate['positions'])
                raise RuntimeError('Original cuRobo constraints reject upright property transport')
            return dict(positions=candidate['positions'],dt=self.dt,effective_goal_pose=target,
                metadata=dict(candidate['metadata'],backend='curobo',method='source_ik_upright_transport',phase=phase))
        return super().plan_pose(q_arm7,target,ignore_names=ignore_names,grip_binary=grip_binary,phase=phase)

    @staticmethod
    def _ignored(name,ignore_names):
        # The upstream helper expects short object names and prepends '/'.
        # An exact source-collider absolute identity must not become '//World'.
        # These are candidate-only switches; full source FCL still checks every
        # robot/payload pair and actual physics substep with its original policy.
        absolute=tuple(value for value in ignore_names if value.startswith('/'))
        short=tuple(value for value in ignore_names if not value.startswith('/'))
        return name in absolute or PayloadCuroboPlanner._ignored(name,short)

    def _cartesian_vertical(self,q_arm7,target,phase):
        reorient=phase=='lower' and getattr(self.world,'upright_transfer_targets',False)
        candidate=(upright_transport_candidate(self.kin,q_arm7,target,self.world.attachment,dt=self.dt,speed=self.max_joint_speed,
                tilt_limit_deg=getattr(self.world,'placement_tilt_limit_deg',3.))
            if reorient else straight_candidate(self.kin,q_arm7,target,dt=self.dt,speed=self.max_joint_speed))
        self.last_cartesian_candidate=candidate
        feasible=self.motion_gen.ik_solver.check_valid(self.args.to_device(candidate['positions']))
        if not bool(feasible.all()):
            self.last_constraint_diagnostics,_=constraint_components(self,candidate['positions'])
            raise RuntimeError('Original cuRobo constraints reject property Cartesian segment')
        metadata=dict(candidate['metadata'],backend='curobo',method='source_ik_cup_pose_slerp' if reorient else 'source_ik_straight',phase=phase)
        return dict(positions=candidate['positions'],dt=self.dt,metadata=metadata,effective_goal_pose=target)


def run(args,envdef,e,recorder,contacts,supports,step,result,actions):
    from kitchen_safety.planning import DroidKinematics
    from scripts.extensions.replicator.hard_execution_v1.property_collect import write
    config=envdef.hard_execution_config;layout=envdef.resolved_task_layout;robot=e.scene['robot']
    controller=config.get('glass_controller',{})
    minimum_force=float(controller.get('minimum_normal_force_per_finger_n',0.))
    maximum_force=float(controller.get('maximum_normal_force_per_finger_n',float('inf')))
    arm_ids=[robot.joint_names.index(f'panda_joint{i}') for i in range(1,8)]
    finger_ids=[robot.joint_names.index(n) for n in PropertyGlassWorld.GRIPPER_JOINT_NAMES]
    gi=recorder.object_names.index('fragile_glass');bi=recorder.object_names.index('bowl')
    kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
    world=PropertyGlassWorld(e.sim.stage,kin,glass_usd=ROOT/config['glass']['usd_path'],env_root=ENV_ROOT)
    result['glass_plans']=[];result['glass_geometry']=world.payload_geometry.provenance
    before=recorder.on_sample;support_certificates=[]
    def sync(row):
        world.set_phase(row['phase'] if row['phase'] in ('pregrasp','approach','close','lift','transport','lower','release','retreat') else 'initial_settle')
        world.gripper_joint_positions=row['q'][finger_ids].copy();world.gripper_joint_position=float(world.gripper_joint_positions[0])
        world.payload_pose=pose_matrix(row['task_object_states'][gi,:7])
        world.sync_background(recorder.background_paths,row['background_rigid_poses'])
        world.set_obstacle_pose('bowl',row['task_object_states'][bi,:7]);world.set_obstacle_pose('fragile_glass',row['task_object_states'][gi,:7])
        world.observe_native_bowl_support(row,supports[-1],gi,bi)
    def observed(row):
        before(row);sync(row)
        if row['phase']=='close' and np.isfinite(maximum_force):
            force=finger_force_window(contacts.records[-1:],config['grasp_candidates']['contact_region_local_m'],minimum_force,maximum_force)
            if not force['all_within_maximum']:
                result['glass_grip_force_failure']=dict(phase=row['phase'],physics_step=row['physics_step'],evidence=force)
                raise RuntimeError('Glass close exceeded declared per-finger normal force bound')
        support_certificates.append(dict(phase=row['phase'],**world.native_support_certificate))
        if not world.valid(row['q'][arm_ids],mode='execution',gripper_q=row['q'][finger_ids],object_pose=row['task_object_states'][gi,:7]):
            result['glass_safety_failure']=dict(physics_step=row['physics_step'],phase=row['phase'],**world.last_clearance,
                native_bowl_support_certificate=world.native_support_certificate,
                same_step_finger_contacts=contacts.records[-1])
            raise RuntimeError('Actual property source geometry rejected: '+str(world.last_clearance))
    recorder.on_sample=observed;sync(recorder.rows[-1])
    try:
        actual=recorder.rows[-1];fk=kin.fk(actual['q'][arm_ids]);measured=pose_matrix(actual['tcp_pose_w'])
        if np.linalg.norm(fk[:3,3]-measured[:3,3])>.002 or Rotation.from_matrix(fk[:3,:3]@measured[:3,:3].T).magnitude()>np.deg2rad(2):
            raise RuntimeError('Observed TCP differs from source FK')
        target=grasp_target(actual['task_object_states'][gi,:7],.15,0,'horizontal')
        palm_retreat=float(config['grasp_candidates'].get('tcp_retreat_from_center_m',0.))
        if not np.isfinite(palm_retreat) or not 0<=palm_retreat<=.018:raise ValueError('Glass TCP retreat must be within tested 0–18 mm candidates')
        target[:3,3]-=target[:3,0]*palm_retreat
        pre=target.copy();pre[:3,3]-=target[:3,0]*.10
        result['glass_grasp_target_tcp']=target
        result['glass_tcp_retreat_from_center_m']=palm_retreat
        def move(goal,phase,grip=0.):
            recorder.phase=phase;sync(recorder.rows[-1]);current=recorder.rows[-1]
            planner=PropertyPlanner(world,kin,dt=e.step_dt,seed=args.seed,max_joint_speed=.3)
            record=dict(phase=phase,target_tcp=goal,start_physics_step=current['physics_step'],
                actual_gripper_joint_positions_rad=current['q'][finger_ids].copy(),
                actual_gripper_joint_names=list(world.GRIPPER_JOINT_NAMES));result['glass_plans'].append(record)
            try:
                plan=planner.plan_pose(current['q'][arm_ids],goal,grip_binary=grip,phase=phase)
                validation=planner.validate_path(plan['positions'],grip_binary=grip,phase=phase,resolution=.02)
                record.update(metadata=plan['metadata'],validation=validation,candidate_contact_mask=planner.candidate_contact_mask)
                np.savez_compressed(args.output/(phase+'_glass_plan.npz'),positions=plan['positions'],target_tcp=goal)
                if not validation['valid']:raise RuntimeError('Property path rejected by independent full-world checks')
                for command in plan['positions'][1:]:step(command,grip,phase)
                q=plan['positions'][-1]
                for _ in range(max(1,int(np.ceil(.5/e.step_dt)))):step(q,grip,phase)
                return q
            except Exception as error:
                record.update(error=repr(error),constraint_diagnostics=getattr(planner,'last_constraint_diagnostics',None),
                    candidate_contact_mask=getattr(planner,'candidate_contact_mask',None))
                raise
        q=move(pre,'pregrasp');q=move(target,'approach')
        controller=config.get('glass_controller');grip_term=e.action_manager.get_term('gripper_action')
        if controller:
            grip_term.configure_glass(True,rate_rad_s=controller['target_rate_rad_s'])
            close_seconds=float(controller['maximum_close_seconds'])
        else:close_seconds=2.
        for _ in range(int(np.ceil(close_seconds/e.step_dt))):
            step(q,1.,'close')
            summary=contacts.summarize(*config['grasp_candidates']['contact_region_local_m'],phase='close',last_steps=10)
            recent=[r for r in recorder.rows if r['phase']=='close'][-10:]
            close_contacts=[r for r in contacts.records if r['phase']=='close'][-10:]
            force=finger_force_window(close_contacts,config['grasp_candidates']['contact_region_local_m'],minimum_force,maximum_force)
            stable=bool(len(recent)==10 and all(np.linalg.norm(r['task_object_states'][gi,7:10])<=.02 and
                np.linalg.norm(r['task_object_states'][gi,10:13])<=.20 for r in recent))
            if controller and summary['both_fingers_loaded'] and summary['both_fingers_near'] and stable and force['all_bilateral_minimum']:
                grip_term.hold_verified_grasp()
                result['glass_rate_limited_close']=dict(end_physics_step=recorder.rows[-1]['physics_step'],
                    held_target_rad=grip_term.processed_actions.detach().cpu().numpy(),stable_last_10_steps=True,
                    linear_speed_limit_m_s=.02,angular_speed_limit_rad_s=.20,normal_force_evidence=force)
                break
        summary=contacts.summarize(*config['grasp_candidates']['contact_region_local_m'],phase='close',last_steps=10)
        result['glass_close_contacts']=summary
        result['glass_bilateral_grasp_observed']=bool(summary['both_fingers_loaded'] and summary['both_fingers_near'])
        if not result['glass_bilateral_grasp_observed']:raise RuntimeError('No simultaneous bilateral loaded source glass-body contacts')
        if controller and not result.get('glass_rate_limited_close'):raise RuntimeError('Glass grasp did not become stable within bounded 10-second close')
        if args.stage=='glass-grasp':
            # A diagnostic grasp cannot count as cup relocation or source task success.
            for _ in range(int(np.ceil(1./e.step_dt))):step(q,0.,'release')
            result['glass_grasp_trial_complete']=True
        else:
            recorder.on_sample=before
            from .property_transfer import run as transfer
            transfer(args,envdef,e,recorder,contacts,supports,step,result,kin)
            if args.stage=='execute':
                if not result.get('glass_relocated_and_stable'):raise RuntimeError('Bowl task prohibited before stable glass release')
                if controller:
                    grip_term.configure_glass(False)
                    result['original_bowl_binary_action_restored_at_physics_step']=recorder.rows[-1]['physics_step']
                from .property_bowl import run as bowl
                bowl(args,envdef,e,recorder,step,actions,result)
    finally:
        recorder.on_sample=before
        write(args.output/'glass_source_support_refinements.json',world.source_support_refinements)
        write(args.output/'glass_native_support_certificates.json',support_certificates)
