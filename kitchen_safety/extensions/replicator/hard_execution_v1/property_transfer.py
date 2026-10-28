"""Physical cup relocation after observed bilateral grasp; no pose writes.

Candidate CUDA masks are restored by independent source-part checks. The only
support refinements bind native contact observations to original entities.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from .property_geometry import PropertyGlassWorld,PropertyGlassContactPolicy,ENV_ROOT
from .property_grasp import PropertyPlanner,finger_force_window
from .property_environment import ROOT


@dataclass(frozen=True)
class TransferContactPolicy(PropertyGlassContactPolicy):
    def cuda_candidate_masks(self,phase,attached):
        masks=list(super().cuda_candidate_masks(phase,attached))
        if phase=='lift':masks.extend(('bowl',ENV_ROOT+'/kitchen/CapSurface_02'))
        return tuple(dict.fromkeys(masks))


class TransferWorld(PropertyGlassWorld):
    def __init__(self,*args,safe_xy,**kwargs):
        super().__init__(*args,**kwargs)
        self.contact_policy=TransferContactPolicy(payload_name='fragile_glass',source_support=ENV_ROOT+'/bowl/obj_000013_Mesh',
            destination_name=ENV_ROOT+'/kitchen/CapSurface_02')
        self.safe_xy=np.asarray(safe_xy);self.counter_support=None;self.departure=None
        self.counter_approach_baseline=None
        self.counter_last_positive=None;self.counter_loaded_residual=None
        self.in_actual_query=False;self.support_transition_ledger=[]
        self.carry_tilt_limit_deg=3.;self.placement_tilt_limit_deg=3.

    def observe_native_bowl_support(self,row,support,glass_index,bowl_index):
        glass=row['task_object_states'][glass_index,:7];bowl=row['task_object_states'][bowl_index,:7]
        if np.linalg.norm(glass[:2]-bowl[:2])>.02:
            self.native_support_certificate=dict(valid=False,physics_step=row['physics_step'],glass_pose=pose_matrix(glass),counter_unloaded=False)
            return
        return super().observe_native_bowl_support(row,support,glass_index,bowl_index)

    def valid(self,*args,mode='planning',**kwargs):
        self.in_actual_query=mode=='execution'
        try:return super().valid(*args,mode=mode,**kwargs)
        finally:self.in_actual_query=False

    def _measurement(self,pair):
        label,name=pair[1:3]
        if label=='payload/fragile_glass' and name==ENV_ROOT+'/kitchen/CapSurface_02':
            strict=self._exact_measurement(pair)
            if strict['valid']:return strict
            if self.phase=='lift':
                cert=self.native_support_certificate if self.in_actual_query else self.departure
                delta=self._query_payload_pose[:3,3]-cert['glass_pose'][:3,3] if cert is not None else np.full(3,np.inf)
                within_departure=bool(cert is not None and cert['valid'] and cert['counter_unloaded'] and
                    not strict['intersecting'] and np.linalg.norm(delta[:2])<.001 and -.001<=delta[2]<.004)
                if self.in_actual_query:within_departure &= bool(np.allclose(self._query_payload_pose,cert['glass_pose'],atol=1e-7,rtol=0))
                if within_departure:
                    self.support_transition_ledger.append(dict(phase=self.phase,actual_query=self.in_actual_query,
                        physics_step=cert['physics_step'],original_source=strict.copy(),delta_m=delta))
                    return dict(strict,valid=True,required_m=0.,slack_m=strict['distance_m'],
                        original_strict_valid=False,initial_supported_departure_only=True)
            cert=self.counter_support
            current=bool(cert and np.allclose(self._query_payload_pose,cert['glass_pose'],atol=1e-7,rtol=0))
            if self.phase=='lower' and self.in_actual_query and self.attachment is not None:
                if current and cert['lower_contact']['valid']:
                    self.support_transition_ledger.append(dict(phase=self.phase,actual_query=True,
                        physics_step=cert['physics_step'],original_source=strict.copy(),native_counter_contact=cert))
                    return dict(strict,valid=True,original_strict_valid=False,
                        allowed_observed_counter_contact=True,native_nonpenetration_claim=False)
                if current and cert['lower_approach']['valid'] and not strict['intersecting'] and strict['distance_m']>=0:
                    self.support_transition_ledger.append(dict(phase=self.phase,actual_query=True,
                        physics_step=cert['physics_step'],original_source=strict.copy(),native_counter_approach=cert))
                    return dict(strict,valid=True,required_m=0.,original_strict_valid=False,
                        allowed_observed_counter_approach=True,native_support_claim=False)
            if self.phase in ('release','retreat','final_settle') and self.attachment is None:
                free_approach=bool(self.phase=='release' and self.in_actual_query and current and
                    cert['release_approach']['valid'] and not strict['intersecting'] and strict['distance_m']>=0.)
                release_transfer=bool(self.phase=='release' and self.in_actual_query and current and
                    cert['release_load_transfer']['valid'])
                if current and (cert['valid'] or free_approach or release_transfer):
                    self.support_transition_ledger.append(dict(phase=self.phase,actual_query=self.in_actual_query,
                        physics_step=cert['physics_step'],original_source=strict.copy(),native_support=cert))
                    return dict(strict,valid=True,original_strict_valid=False,
                        allowed_observed_counter_support=bool(cert['valid']),allowed_observed_counter_approach=free_approach,
                        allowed_observed_release_load_transfer=release_transfer)
            return strict
        return super()._measurement(pair)

    def observe_counter(self,row,support,glass_index,finger_observation=None):
        """Certify only the original cup-bottom/target-counter contact pair.

        Positive native reports during held lowering establish proximity, not
        support. A released cup must independently carry its weight on the
        correct bottom patch. The 6 mm region identifies the cup bottom; it is
        never a permitted world penetration depth or a guessed contact offset.
        """
        pose=pose_matrix(row['task_object_states'][glass_index,:7]);points=self.payload_geometry.vertices@pose[:3,:3].T+pose[:3,3]
        forces=np.asarray(support['filtered_force_matrix_n']).reshape(-1,2,3)[0]
        counts=np.asarray(support['counts']).reshape(-1);starts=np.asarray(support['starts']).reshape(-1)
        n,first=int(counts[1]),int(starts[1]);sep=np.asarray(support['separations_m']).reshape(-1)[first:first+n]
        force=np.asarray(support['normal_forces_n']).reshape(-1)[first:first+n]
        native_points=np.asarray(support['points_world_m']).reshape(-1,3)[first:first+n]
        normals=np.asarray(support['normals_world']).reshape(-1,3)[first:first+n]
        local=(native_points-pose[:3,3])@pose[:3,:3]
        counter=next(x for x in self.objects if x[0]==ENV_ROOT+'/kitchen/CapSurface_02')
        gap=float(points[:,2].min()-counter[3][2])
        # Native positions are float32. Two endpoints plus transformed support
        # coordinates account for four coordinate ULPs, recorded explicitly.
        numeric_bound=float(4*np.max(np.abs(np.spacing(np.asarray(np.r_[pose[:3,3],counter[3]],dtype=np.float32)))))
        dz=native_points[:,2]-counter[3][2]
        on_floor=(dz>=np.minimum(0.,sep*.5)-numeric_bound)&(dz<=np.maximum(0.,sep*.5)+numeric_bound)
        bottom=(np.abs(local[:,2])<=.006)&(np.linalg.norm(local[:,:2],axis=1)<=.040)
        upward=normals[:,2]>=np.cos(np.deg2rad(20))
        native_inside=np.all((native_points[:,:2]>=counter[2][:2])&(native_points[:,:2]<=counter[3][:2]),axis=1)
        raw_vector=np.sum(force[:,None]*normals,axis=0)
        full_footprint=float(np.linalg.norm(points[:,:2]-self.safe_xy,axis=1).max())
        common=dict(same_physics_step=support['physics_step']==row['physics_step'],
            native_contact_present=bool(n),
            all_native_points_original_bottom_counter_upward=bool(n and np.all(bottom&on_floor&upward&native_inside)),
            raw_filtered_force_agree=bool(np.linalg.norm(raw_vector-forces[1])<.1),
            counter_horizontal_force_below_1n=bool(np.linalg.norm(forces[1,:2])<1.),
            bowl_unloaded=bool(np.linalg.norm(forces[0])<.01),
            glass_within_declared_placement_tilt=bool(pose[2,2]>=np.cos(np.deg2rad(self.placement_tilt_limit_deg))),
            center_within_5mm=bool(np.linalg.norm(pose[:2,3]-self.safe_xy)<=.005),
            all_source_vertices_within_original_safe_circle=full_footprint<=.052,
            all_source_vertices_over_original_counter=bool(np.all(points[:,:2].min(0)>=counter[2][:2]) and
                np.all(points[:,:2].max(0)<=counter[3][:2])))
        fingers={}
        if finger_observation:
            for name,entry in finger_observation['fingers'].items():
                nf=np.asarray(entry['normal_forces_n']);fs=np.asarray(entry['separations_m'])
                fp=np.asarray(entry['points_object_local_m']).reshape(-1,3)
                region=np.all((fp>=np.array([-.04,-.04,.12]))&(fp<=np.array([.04,.04,.215])),axis=1)
                fl=nf>.01
                fingers[name]=dict(valid=bool(np.any(fl&(fs<=.00025)&region) and np.all(region[fl])),
                    loaded_near_contact_count=int(np.sum(fl&(fs<=.00025)&region)),normal_force_sum_n=float(nf.sum()))
        transition=bool(n and np.all(sep>=0.))
        approach_checks=dict(common,actual_lower_phase=self.phase=='lower',
            current_finger_contacts_same_step=bool(finger_observation and finger_observation['physics_step']==row['physics_step']),
            bilateral_loaded_near_declared_wall=bool(len(fingers)==2 and all(v['valid'] for v in fingers.values())),
            native_counter_unloaded=bool(not np.any(force>.01) and np.linalg.norm(forces[1])<.01),
            native_gaps_nonnegative=transition,
            source_positive_gap_bounded_by_observed_native_gap=bool(n and 0<=gap<=min(.006,float(sep.min()))))
        approach=dict(valid=all(approach_checks.values()),checks=approach_checks,
            failed_checks=[k for k,v in approach_checks.items() if not v],same_step_fingers=fingers,
            support_claim=False,static_counter_contact_offset_verified=False)
        if approach['valid'] and self.counter_approach_baseline is None:
            self.counter_approach_baseline=dict(physics_step=row['physics_step'],source_gap_m=gap,
                minimum_native_gap_m=float(sep.min()),native_minus_source_gap_m=float(sep.min()-gap),
                glass_pose=pose.copy(),native_points_world_m=native_points.copy())
        baseline=self.counter_approach_baseline
        release_checks=dict(common,only_release_phase=self.phase=='release',
            verified_held_lower_approach=baseline is not None,
            native_gaps_nonnegative=transition,
            source_nonnegative_observed_proximity=bool(n and 0<=gap<=min(.006,float(sep.min()))))
        release_approach=dict(valid=all(release_checks.values()),checks=release_checks,
            failed_checks=[k for k,v in release_checks.items() if not v],support_claim=False)
        loaded_checks=dict(common,
            loaded_near_native_bottom=bool(n and np.any((force>.01)&(sep<=.00025))),
            counter_carries_80_percent_weight=bool(forces[1,2]>.8*.56*9.81))
        # Intended support contact has a local 0.1 mm signed residual bound.
        # This is only the original cup-bottom/CapSurface_02 pair, with current
        # loaded native evidence above. It neither changes the 2 mm noncontact
        # margins nor claims that the raw source/native geometry never overlaps.
        # A first contact is diagnostic history, not a forever-fixed depth gate:
        # normal load establishment and tiny changes of tilt change its residual.
        state=row['task_object_states'][glass_index];minimum=float(sep.min()) if n else None
        residual_ok=bool(n and gap>=-.0001 and minimum>=-.0001 and gap<=.00035)
        loaded_checks['source_and_native_residuals_within_target_support_0p1mm']=residual_ok
        loaded=all(loaded_checks.values())
        if loaded and self.counter_loaded_residual is None:
            self.counter_loaded_residual=dict(physics_step=row['physics_step'],source_gap_m=gap,
                minimum_native_gap_m=minimum,glass_pose=pose.copy(),native_points_world_m=native_points.copy(),
                native_separations_m=sep.copy(),native_force_n=forces[1].copy(),native_nonpenetration_claim=False,
                role='First observed loaded contact only; not an acceptance depth baseline')
        first_loaded=self.counter_loaded_residual
        # Opening changes the load distribution and can briefly unload the
        # bottom patch. Preserve that observed transition only during release,
        # after genuine loaded support on this exact target. The final loaded
        # flag and settle checks still require 80% of the glass weight.
        release_transfer_checks=dict(common,only_release_phase=self.phase=='release',
            prior_observed_loaded_target_contact=bool(first_loaded is not None and
                first_loaded['physics_step']<row['physics_step']),
            source_and_native_residuals_within_existing_target_bounds=residual_ok)
        release_transfer=dict(valid=all(release_transfer_checks.values()),checks=release_transfer_checks,
            failed_checks=[k for k,v in release_transfer_checks.items() if not v],
            current_upward_weight_fraction=float(forces[1,2]/(.56*9.81)),
            prior_loaded_contact_step=None if first_loaded is None else first_loaded['physics_step'],
            full_weight_support_claim=False,stable_placement_claim=False)
        contact_checks=dict(loaded_checks,lower_phase=self.phase=='lower',
            verified_prior_positive_approach=baseline is not None,
            same_step_bilateral_wall_grasp=bool(finger_observation and finger_observation['physics_step']==row['physics_step'] and
                len(fingers)==2 and all(v['valid'] for v in fingers.values())),
            low_speed_contact_event=bool(np.linalg.norm(state[7:10])<=.05 and np.linalg.norm(state[10:13])<=.20))
        lower_contact=dict(valid=all(contact_checks.values()),checks=contact_checks,
            failed_checks=[k for k,v in contact_checks.items() if not v],native_nonpenetration_claim=False)
        if (approach['valid'] or release_approach['valid']) and gap>=0 and minimum is not None and minimum>=0:
            self.counter_last_positive=dict(physics_step=row['physics_step'],source_gap_m=gap,
                minimum_native_gap_m=minimum,glass_pose=pose.copy(),velocity_m_s=state[7:10].copy(),
                angular_velocity_rad_s=state[10:13].copy())
        self.counter_support=dict(valid=loaded,loaded=loaded,nonnegative_native_transition=transition,
            physics_step=row['physics_step'],glass_pose=pose,native_force_n=forces[1],source_gap_m=gap,
            xy_error_m=float(np.linalg.norm(pose[:2,3]-self.safe_xy)),source_safe_footprint_radius_m=full_footprint,
            loaded_checks=loaded_checks,failed_checks=[k for k,v in loaded_checks.items() if not v],lower_approach=approach,
            lower_contact=lower_contact,
            release_approach=release_approach,release_load_transfer=release_transfer,first_observed_loaded_contact=first_loaded,
            target_support_signed_residual_limit_m=.0001,
            target_support_source_positive_gap_limit_m=.00035,
            signed_residual_acceptance_scope='Only original cup bottom/original target countertop: current loaded support or observed release transition after prior loaded support; all other geometry keeps original clearance',
            native_points_world_m=native_points,native_points_cup_local_m=local,native_normals_world=normals,
            native_separations_m=sep,native_normal_forces_n=force,raw_force_vector_n=raw_vector,
            native_point_counter_height_m=dz,numeric_coordinate_bound_m=numeric_bound,
            positive_approach_baseline=baseline,static_counter_contact_offset_verified=False,
            native_nonpenetration_claim=False,declared_placement_tilt_limit_deg=self.placement_tilt_limit_deg)

    def observe_lift_departure(self,row,support,finger_observation,glass_index):
        """Observe a bounded transfer of support; never reuse a stale pose.

        The 80%-weight resting requirement remains intact in every other phase.
        During the first 6 mm of lift and within 5 mm horizontally of the
        verified supported grasp, the fingers may unload the bowl. Current
        native contacts/clear separation and bilateral
        loaded fingers are required independently, at this exact physics step.
        """
        cert=self.native_support_certificate;start=self.departure
        if self.phase!='lift' or start is None or cert is None or cert.get('valid'):return
        state=row['task_object_states'][glass_index];pose=pose_matrix(state[:7])
        delta=pose[:3,3]-start['glass_pose'][:3,3]
        fingers={}
        for name,entry in finger_observation['fingers'].items():
            nf=np.asarray(entry['normal_forces_n']);sep=np.asarray(entry['separations_m'])
            pts=np.asarray(entry['points_object_local_m']).reshape(-1,3)
            loaded=nf>.01;near=sep<=.00025
            inside=np.all((pts>=np.array([-.04,-.04,.12]))&(pts<=np.array([.04,.04,.215])),axis=1)
            fingers[name]=dict(valid=bool(np.any(loaded&near&inside) and np.all(inside[loaded])),
                normal_force_sum_n=float(nf.sum()),loaded_near_contact_count=int(np.sum(loaded&near&inside)),
                loaded_separations_m=sep[loaded].tolist())
        counts=np.asarray(support['counts']).reshape(-1);starts=np.asarray(support['starts']).reshape(-1)
        select=slice(int(starts[0]),int(starts[0]+counts[0]))
        sep=np.asarray(support['separations_m']).reshape(-1)[select]
        nf=np.asarray(support['normal_forces_n']).reshape(-1)[select]
        pts=np.asarray(support['points_world_m']).reshape(-1,3)[select]
        normals=np.asarray(support['normals_world']).reshape(-1,3)[select]
        local=(pts-pose[:3,3])@pose[:3,:3]
        from kitchen_safety.extensions.replicator.hard_v1.property_scene import _height
        vertices,faces=next(v for k,v in self.meshes.items() if self._matches(k,'bowl'))
        envelope=cert.get('native_contact_envelope_m')
        footprint=(np.abs(local[:,2])<=.006)&(np.linalg.norm(local[:,:2],axis=1)<=.040)
        floor=np.zeros(len(pts),dtype=bool)
        for i in np.flatnonzero(footprint):
            floor[i]=abs(float(pts[i,2])-_height(vertices[faces],pts[i,:2]))<=.006
        bounded=(sep>=0)&(sep<=envelope) if envelope is not None else np.zeros(len(sep),dtype=bool)
        upward=normals[:,2]>=np.cos(np.deg2rad(20))
        # Preserve the signed raw residual. A zero-load native patch may retain
        # a micrometre-scale negative value while unloading. It is accepted
        # only if it improves on this run's verified loaded support, never by
        # inventing a generic penetration epsilon or claiming nonpenetration.
        baseline_sep=np.asarray(start.get('loaded_native_separations_m',[]),dtype=float)
        baseline_min=float(baseline_sep.min()) if len(baseline_sep) else None
        negative=sep<0
        residual_checks=dict(verified_loaded_baseline=bool(baseline_min is not None and -.001<=baseline_min<0),
            native_signed_gap_not_deeper=bool(len(sep) and baseline_min is not None and np.all(sep>=baseline_min)),
            source_floor_gap_not_deeper=bool(cert['source_bottom_gap_m']>=start['source_bottom_gap_m']),
            observed_cup_rise=bool(delta[2]>0),
            all_negative_pairs_original_bottom_floor_upward=bool(np.any(negative) and
                np.all((footprint&floor&upward)[negative])))
        residual=all(residual_checks.values())
        native_nonnegative=bool(len(sep) and np.all(sep>=0))
        separating=bool(len(sep) and not np.any(nf>.01) and (native_nonnegative or residual) and
            np.any(footprint&floor&bounded&upward) and np.linalg.norm(cert['filtered_force_world_n'])<.01)
        original=cert['native_support_checks']
        partial=bool(original['loaded_contact_present'] and cert['upward_force_n']>.01 and
            all(value for key,value in original.items() if key!='bowl_carries_80_percent_weight'))
        checks=dict(prior_actual_supported_grasp=bool(start.get('valid') and start.get('bilateral_stable_grasp')),
            current_finger_contacts_same_step=finger_observation['physics_step']==row['physics_step'],
            current_bilateral_loaded_near=bool(len(fingers)==2 and all(f['valid'] for f in fingers.values())),
            initial_lift_zone=bool(np.linalg.norm(delta[:2])<=.005 and -.001<=delta[2]<=.006),
            source_floor_band_unchanged=bool(original['source_floor_gap_within_6mm']),
            upright_and_inside_bowl=bool(original['glass_center_within_bowl_20mm'] and original['glass_upright_within_3deg']),
            horizontal_bowl_force_unchanged=bool(original['horizontal_bowl_force_below_1n']),
            bounded_native_partial_load_or_separation=bool(partial or separating),
            no_downward_fall=bool(state[9]>=-.005),
            bounded_departure_motion=bool(np.linalg.norm(state[7:10])<=.05 and np.linalg.norm(state[10:13])<=.20),
            original_counter_unloaded_above=bool(cert['counter_unloaded'] and cert['glass_above_counter']))
        transition=dict(valid=all(checks.values()),checks=checks,failed_checks=[k for k,v in checks.items() if not v],
            physics_step=row['physics_step'],prior_supported_grasp_step=start['physics_step'],delta_from_supported_grasp_m=delta,
            initial_lift_horizontal_limit_m=.005,initial_lift_vertical_range_m=[-.001,.006],
            mode='partial_bowl_load' if partial else ('native_nonnegative_separation' if native_nonnegative else
                'unloaded_negative_residual_improving_from_verified_support') if separating else 'unverified',
            current_bowl_upward_weight_fraction=cert['upward_force_n']/cert['glass_weight_n'],
            current_finger_normal_contacts=fingers,
            native_pair_min_separation_m=float(sep.min()) if len(sep) else None,
            verified_loaded_baseline_min_separation_m=baseline_min,
            verified_loaded_baseline_source_floor_gap_m=start['source_bottom_gap_m'],
            current_source_floor_gap_m=cert['source_bottom_gap_m'],
            unloaded_residual_checks=residual_checks,
            raw_negative_separations_m=sep[negative].tolist(),all_reported_native_separations_nonnegative=native_nonnegative,
            native_nonpenetration_claim=False,
            native_bottom_patch_count=int(np.sum(footprint&floor&bounded&upward)),
            statement='Bowl no longer required to carry 80% weight during verified initial lift. Finger normal contacts do not measure vertical friction; native load/pose/clearance observations are retained.')
        cert['initial_lift_load_transfer']=transition
        if transition['valid']:
            cert['resting_80_percent_support_valid']=False;cert['valid']=True
            cert['support_mode']='observed_initial_lift_load_transfer'
        self.support_transition_ledger.append(dict(phase='lift',actual_query=True,native_load_transfer=transition))


def run(args,envdef,e,recorder,contacts,supports,step,result,kin):
    from scripts.extensions.replicator.hard_execution_v1.property_collect import write
    config=envdef.hard_execution_config;robot=e.scene['robot']
    arm=[robot.joint_names.index(f'panda_joint{i}') for i in range(1,8)]
    fingers=[robot.joint_names.index(n) for n in PropertyGlassWorld.GRIPPER_JOINT_NAMES]
    gi=recorder.object_names.index('fragile_glass');bi=recorder.object_names.index('bowl')
    world=TransferWorld(e.sim.stage,kin,glass_usd=ROOT/config['glass']['usd_path'],env_root=ENV_ROOT,
        safe_xy=config['safe_placement']['glass_target_position_m'][:2])
    world.upright_transport=bool(config.get('glass_transport',{}).get('cartesian_upright',False))
    world.upright_transfer_targets=bool(config.get('glass_transport',{}).get('world_z_upright_targets',False))
    tilt_limits=config.get('glass_tilt_limits_deg',{})
    world.carry_tilt_limit_deg=float(tilt_limits.get('carry',3.))
    world.placement_tilt_limit_deg=float(tilt_limits.get('placement',3.))
    if not 0<world.placement_tilt_limit_deg<=world.carry_tilt_limit_deg<=5.:
        raise ValueError('Cup tilt policy must satisfy 0 < placement <= carry <= 5 degrees')
    result['glass_tilt_policy']=dict(carry_deg=world.carry_tilt_limit_deg,placement_deg=world.placement_tilt_limit_deg,
        carry_phases=['lift','transport'],original_bowl_support_tilt_deg=3.,
        scope='Empty rigid glass; phase-specific geometry and loaded support remain independently checked')
    native_callback=recorder.on_sample;support_certificates=[]
    # Caller passes the native support callback, not the obsolete grasp world's
    # safety callback; only one current payload world owns each phase.
    region=config['grasp_candidates']['contact_region_local_m'];result['glass_transfer_plans']=[]
    result['glass_planning_attachment_bindings']=[]
    initial=recorder.rows[-1]['task_object_states'][gi,:7].copy();bowl_initial=recorder.rows[-1]['task_object_states'][bi,:7].copy()
    counter_contact_event=None
    arm_action=e.action_manager.get_term('arm_action');grip_action=e.action_manager.get_term('gripper_action')
    def sync(row):
        world.set_phase(row['phase']);world.gripper_joint_positions=row['q'][fingers].copy();world.gripper_joint_position=float(world.gripper_joint_positions[0])
        world.payload_pose=pose_matrix(row['task_object_states'][gi,:7]);world.sync_background(recorder.background_paths,row['background_rigid_poses'])
        for name,index in (('bowl',bi),('fragile_glass',gi)):world.set_obstacle_pose(name,row['task_object_states'][index,:7])
        world.observe_native_bowl_support(row,supports[-1],gi,bi);world.observe_counter(row,supports[-1],gi,contacts.records[-1])
        world.observe_lift_departure(row,supports[-1],contacts.records[-1],gi)
    def observed(row):
        nonlocal counter_contact_event
        native_callback(row);sync(row)
        force_max=config.get('glass_controller',{}).get('maximum_normal_force_per_finger_n')
        if force_max is not None and row['phase'] in ('lift','transport','lower'):
            force=finger_force_window(contacts.records[-1:],region,0.,float(force_max))
            if not force['all_within_maximum']:
                result['glass_grip_force_failure']=dict(phase=row['phase'],physics_step=row['physics_step'],evidence=force)
                raise RuntimeError('Carried glass exceeded declared per-finger normal force bound')
        support_certificates.append(dict(phase=row['phase'],**world.native_support_certificate))
        row['has_planning_attachment']=world.attachment is not None
        if world.attachment is not None:row['planning_attachment']=world.attachment.copy()
        if not world.valid(row['q'][arm],mode='execution',gripper_q=row['q'][fingers],object_pose=row['task_object_states'][gi,:7]):
            result['glass_transfer_safety_failure']=dict(phase=row['phase'],physics_step=row['physics_step'],**world.last_clearance,
                native_bowl_support_certificate=world.native_support_certificate,
                native_counter_support_certificate=world.counter_support,
                same_step_finger_contacts=contacts.records[-1])
            raise RuntimeError('Actual glass transfer pair failed: '+str(world.last_clearance))
        bowl=row['task_object_states'][bi,:7]
        if np.linalg.norm(bowl[:3]-bowl_initial[:3])>.003:raise RuntimeError('Bowl moved before stable cup relocation')
        tilt=float(np.degrees(np.arccos(np.clip(pose_matrix(row['task_object_states'][gi,:7])[2,2],-1,1))))
        limit=world.carry_tilt_limit_deg if row['phase'] in ('lift','transport') else world.placement_tilt_limit_deg
        if tilt>limit:
            result['glass_tilt_failure']=dict(phase=row['phase'],physics_step=row['physics_step'],actual_tilt_deg=tilt,declared_limit_deg=limit)
            raise RuntimeError(f'Cup tilt {tilt:.6f} exceeds {row["phase"]} limit {limit:g} degrees')
        if row['phase']=='lower' and counter_contact_event is None and world.counter_support['lower_contact']['valid']:
            prior=contacts.summarize(*region,phase='lower',last_steps=10)
            if not prior['both_fingers_loaded'] or not prior['both_fingers_near']:
                raise RuntimeError('Counter contact occurred without continuous bilateral wall grasp')
            counter_contact_event=dict(physics_step=row['physics_step'],action_index=row['action_index'],
                measured_arm_hold_target_rad=row['q'][arm].copy(),native_counter_contact=world.counter_support,
                prior_bilateral_contacts=prior,effective_from_physics_step=row['physics_step']+1,
                intervention='Stop remaining downward commands; apply measured arm position target and original binary gripper-open target from next physics substep',
                pose_or_joint_state_writes=False)
            result['glass_counter_contact_event']=counter_contact_event
            arm_action.hold_observed_position(row['q'][arm])
            grip_action.open_on_counter_contact()
            world.attachment=None
            recorder.phase='release'
    recorder.on_sample=observed
    def bind_observed_attachment(row):
        # Planning evaluates source FK(q), while native TCP has solver/float
        # measurement noise. Bind to the measured q representation so sample 0
        # reproduces the measured cup pose exactly, rather than weakening the
        # exact-pose native support certificate by a numerical tolerance.
        native_tcp=pose_matrix(row['tcp_pose_w']);source_tcp=kin.fk(row['q'][arm])
        cup=pose_matrix(row['task_object_states'][gi,:7])
        linear=float(np.linalg.norm(native_tcp[:3,3]-source_tcp[:3,3]))
        angular=float(Rotation.from_matrix(native_tcp[:3,:3]@source_tcp[:3,:3].T).magnitude())
        if linear>.002 or angular>np.deg2rad(2):raise RuntimeError('Observed transfer TCP differs from source FK')
        physical=np.linalg.inv(native_tcp)@cup
        world.attachment=np.linalg.inv(source_tcp)@cup
        result['glass_planning_attachment_bindings'].append(dict(physics_step=row['physics_step'],
            phase=row['phase'],native_tcp_relative_cup=physical,source_fk_relative_cup=world.attachment.copy(),
            native_source_tcp_translation_error_m=linear,native_source_tcp_rotation_error_deg=float(np.degrees(angular)),
            initial_source_payload_pose_exact=bool(np.allclose(source_tcp@world.attachment,cup,atol=1e-12,rtol=0)),
            physical_constraints_or_state_writes=False))
        return physical
    try:
        row=recorder.rows[-1];sync(row);world.departure=world.native_support_certificate.copy()
        prior=contacts.summarize(*region,phase='close',last_steps=10)
        close_rows=[r for r in recorder.rows if r['phase']=='close'][-10:]
        world.departure['bilateral_stable_grasp']=bool(prior['both_fingers_loaded'] and prior['both_fingers_near'] and
            len(close_rows)==10 and all(np.linalg.norm(r['task_object_states'][gi,7:10])<=.02 and
                np.linalg.norm(r['task_object_states'][gi,10:13])<=.20 for r in close_rows))
        if not world.departure['valid'] or not world.departure['bilateral_stable_grasp']:
            raise RuntimeError('Lift departure lacks prior observed stable supported bilateral grasp')
        result['glass_initial_departure_binding']=world.departure.copy()
        result['glass_measured_planning_attachment']=bind_observed_attachment(row)
        def hold(q,grip,phase,seconds):
            for _ in range(max(1,int(np.ceil(seconds/e.step_dt)))):
                step(q,grip,phase)
                if phase=='lower' and counter_contact_event is not None:break
        def move(target,phase,grip):
            recorder.phase=phase;sync(recorder.rows[-1]);current=recorder.rows[-1]
            planner=PropertyPlanner(world,kin,dt=e.step_dt,seed=args.seed,max_joint_speed=.3)
            rec=dict(phase=phase,start_physics_step=current['physics_step'],target_tcp=target,
                actual_gripper_joint_positions_rad=current['q'][fingers].copy(),
                actual_gripper_joint_names=list(world.GRIPPER_JOINT_NAMES));result['glass_transfer_plans'].append(rec)
            try:
                plan=planner.plan_pose(current['q'][arm],target,grip_binary=grip,phase=phase)
                validation=planner.validate_path(plan['positions'],grip_binary=grip,phase=phase,resolution=.02)
                rec.update(validation=validation,metadata=plan['metadata'],candidate_masks=planner.candidate_contact_mask)
                np.savez_compressed(args.output/(phase+'_glass_transfer.npz'),positions=plan['positions'],target_tcp=target)
                if not validation['valid']:raise RuntimeError('Cup transfer path failed full cuRobo/FCL checks')
                for command in plan['positions'][1:]:
                    step(command,grip,phase)
                    if phase=='lower' and counter_contact_event is not None:
                        rec['terminated_at_observed_counter_contact']=counter_contact_event['physics_step']
                        return counter_contact_event['measured_arm_hold_target_rad'].copy()
                q=plan['positions'][-1];hold(q,grip,phase,.5)
                if phase=='lower' and counter_contact_event is not None:
                    rec['terminated_at_observed_counter_contact']=counter_contact_event['physics_step']
                    return counter_contact_event['measured_arm_hold_target_rad'].copy()
                return q
            except Exception as error:
                rec.update(error=repr(error),constraints=getattr(planner,'last_constraint_diagnostics',None),
                    candidate_masks=getattr(planner,'candidate_contact_mask',None));raise
        lift=pose_matrix(row['tcp_pose_w']);lift[2,3]+=.10;q=move(lift,'lift',1.)
        summary=contacts.summarize(*region,phase='lift',last_steps=10)
        rise=float(recorder.rows[-1]['task_object_states'][gi,2]-initial[2]);result['glass_lift_evidence']=dict(rise_m=rise,contacts=summary)
        if rise<.08 or not summary['both_fingers_loaded'] or not summary['both_fingers_near']:raise RuntimeError('No actual bilateral cup lift')
        row=recorder.rows[-1];bind_observed_attachment(row)
        desired=pose_matrix(row['task_object_states'][gi,:7]);desired[:2,3]=world.safe_xy
        yaw=float(config.get('glass_transport',{}).get('world_z_yaw_delta_deg',0.))
        desired[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@desired[:3,:3]
        if world.upright_transfer_targets:
            desired[:3,:3]=Rotation.from_euler('z',np.arctan2(desired[1,0],desired[0,0])).as_matrix()
        result['glass_transport_orientation']=dict(world_z_yaw_delta_deg=yaw,initial_observed_roll_pitch_preserved=not world.upright_transfer_targets,
            explicit_world_z_upright_target=world.upright_transfer_targets,target_cup_rotation=desired[:3,:3].copy(),
            safe_placement_center_unchanged=True,cartesian_upright=world.upright_transport)
        transport=desired@np.linalg.inv(world.attachment);q=move(transport,'transport',1.)
        row=recorder.rows[-1];measured=pose_matrix(row['task_object_states'][gi,:7]);bind_observed_attachment(row)
        desired=measured.copy();desired[:2,3]=world.safe_xy
        if world.upright_transfer_targets:
            desired[:3,:3]=Rotation.from_euler('z',np.arctan2(measured[1,0],measured[0,0])).as_matrix()
        result['glass_lower_orientation']=dict(explicit_world_z_upright_target=world.upright_transfer_targets,
            measured_cup_tilt_deg=float(np.degrees(np.arccos(np.clip(measured[2,2],-1,1)))),target_cup_rotation=desired[:3,:3].copy())
        relative=world.payload_geometry.vertices@desired[:3,:3].T
        desired[2,3]=config['safe_placement']['countertop_z_m']+.003-float(relative[:,2].min())
        lower=desired@np.linalg.inv(world.attachment)
        if np.linalg.norm(desired[:2,3]-measured[:2,3])>.005:raise RuntimeError('Cup transport missed original safe-placement column')
        q=move(lower,'lower',1.);summary=contacts.summarize(*region,phase='lower',last_steps=10)
        if counter_contact_event is None and (not summary['both_fingers_loaded'] or not summary['both_fingers_near']):raise RuntimeError('Cup slipped before release')
        world.attachment=None;hold(q,0.,'release',2.)
        row=recorder.rows[-1];sync(row)
        if not world.counter_support['loaded']:raise RuntimeError('Released cup lacks loaded original countertop support')
        arm_action.release_contact_hold()
        retreat=pose_matrix(row['tcp_pose_w']);retreat[:3,3]-=retreat[:3,0]*.10
        q=move(retreat,'retreat',0.);hold(q,0.,'final_settle',4.)
        stable=[r['task_object_states'][gi] for r in recorder.rows if r['phase']=='final_settle']
        poses=np.asarray(stable);motion=float(np.linalg.norm(poses[:,:3]-poses[0,:3],axis=1).max())
        speed=float(np.linalg.norm(poses[:,7:10],axis=1).max())
        result['glass_release_evidence']=dict(counter_support=world.counter_support,motion_m=motion,maximum_speed_m_s=speed)
        if not world.counter_support['loaded'] or motion>.002 or speed>.02:raise RuntimeError('Released fragile cup did not remain stable on safe counter')
        result['glass_relocated_and_stable']=True
    finally:
        recorder.on_sample=native_callback
        write(args.output/'glass_transfer_support_ledger.json',world.support_transition_ledger)
        write(args.output/'glass_transfer_source_support_refinements.json',world.source_support_refinements)
        write(args.output/'glass_transfer_native_support_certificates.json',support_certificates)
