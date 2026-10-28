"""Private complete banana/bowl support policy; old diagnostics stay immutable.

Only exact measured banana/bowl target support is refined. Source triangle
intersection remains recorded and never becomes a fitted penetration allowance.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.replicator.banana_destination_probe import (
    DiagnosticDestinationWorld as GeometryWorld, DestinationNativeContacts,
    BODY, DESTINATION, DESTINATION_MESH, SOURCE_ASSETS, MAX_CONTACTS,
    original_region, support_continuation_checks, release_boundary, matrix,
)
from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld

ROOT=Path(__file__).resolve().parents[3]
SUPPORT_PHASES=frozenset(('release','retreat','final_settle'))
TARGET_PHASES=SUPPORT_PHASES|{'lower'}
MODULE_PATH=str(Path(__file__).resolve().relative_to(ROOT))

def require(value,message):
    if not value:raise ValueError(message)

def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def read_policy(path):
    v=json.loads(Path(path).read_text())
    expected=dict(schema_version=1,task_id='kitchen_bench_replicator_g_shape_banana_bowl',
        payload=BODY,target=DESTINATION,target_mesh=DESTINATION_MESH,noncontact_clearance_m=.002,
        lower_clearance_m=.002,release_gap_window_m=[.003,.005],maximum_extra_lower_hold_actions=5,
        maximum_release_actions=30,maximum_release_physics_steps=240,release_seconds=2.,
        smooth_opening_ramp_actions=15,additional_zero_target_actions=15,retreat_height_m=.15,
        final_settle_seconds=4.,physics_dt_s=1/120,action_dt_s=1/15,force_threshold_n=.1,
        velocity_threshold_m_s=.003,support_cone_half_angle_rad=float(np.pi/4),placement_consecutive_steps=1,
        contact_capacity=MAX_CONTACTS,original_joint_bound_allowance_rad=1e-5,
        unloaded_transition_rule='complete_native_contacts_all_separations_nonnegative_and_no_downward_aggregate',
        action_interface='droid_arm7_and_gripper1_joint_position_radians_v1')
    require({k:v.get(k) for k in expected}==expected,'Original support/schedule/predicate values changed')
    require(v.get('support_phases')==sorted(SUPPORT_PHASES),'Support scope expanded')
    require(v.get('source_assets')=={k:dict(path=p,sha256=s) for k,(p,s) in SOURCE_ASSETS.items()},'Wrong source geometry')
    require(v.get('retreat_destination_cuda_enabled') is True,'Retreat must retain full bowl CUDA collision')
    for name,expected_sha in v['evidence_sha256'].items():
        require(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected_sha,'Changed policy lineage: '+name)
    return v

def audit_policy_archive(report,binding):
    entries=report['entries'];required={binding['path']:binding['sha256'],binding['module_path']:binding['module_sha256']}
    value=json.loads(entries[binding['path']]['source_text'])
    required.update(value['evidence_sha256'])
    for path,expected in required.items():
        e=entries[path]
        require(e['unchanged'] and e['initial_sha256']==e['checked_sha256']==expected,'Changed/missing policy capture: '+path)
        require(hashlib.sha256(e['source_text'].encode()).hexdigest()==expected,'Absent exact archived input: '+path)
    require(entries[binding['path']]['kind']=='target_support_policy_input','Wrong policy capture role')
    require(entries[binding['module_path']]['kind']=='execution_source','Wrong helper role')
    return dict(passed=True,binding=deepcopy(binding),required_source_sha256=required,accepted=False)

def observed_support_checks(source,native,filtered_force,low,high):
    """Same loaded/transition logic as bowl; actual curved geometry and center."""
    instant=support_continuation_checks(source,native,filtered_force,low,high)
    p=instant['checks'];points=np.asarray(native['points_world_m'],float).reshape(-1,3)
    loads=np.asarray(native['normal_forces_n'],float);normals=np.asarray(native['normals_world'],float).reshape(-1,3)
    gaps=np.asarray(native['separations_m'],float);force=np.asarray(filtered_force,float)
    complete=bool(p['raw_contact_complete'] and type(native['count']) is int and 0<native['count']<MAX_CONTACTS)
    summed=(loads[:,None]*normals).sum(0) if complete else np.zeros(3)
    loaded=bool(complete and p['native_aggregate_force_above_threshold'] and p['native_aggregate_force_in_upward_cone'] and p['original_force_above_threshold'] and p['original_force_in_upward_cone'])
    transition=bool(complete and np.all(gaps>=0.) and force[2]>=0. and summed[2]>=0.)
    checks=dict(exact_original_pair=p['native_exact_identity'],complete_native_contacts=complete,
        observed_support_or_nonpenetrating_transition=loaded or transition,
        source_inventory_consistent=bool(source['intersecting'])==bool(source['contacts']),
        source_intersections_reconstructed=not source['intersecting'] or p['source_contacts_reconstructed_from_triangles'],
        source_intersections_in_original_bowl_region=not source['intersecting'] or p['source_intersections_in_original_bowl_region'],
        native_points_in_original_bowl_region=p['native_points_in_original_bowl_region'],
        original_spawned_center_in_destination_region=p['original_spawned_center_in_destination_region'],
        exact_source_assets=p['exact_source_assets'])
    return dict(passed=all(checks.values()),checks=checks,
        contact_state='loaded_support' if loaded else ('observed_nonpenetrating_contact_transition' if transition else 'rejected'),
        loaded_support=loaded,transition_is_not_support_or_success=True,
        native_force_vector_sum_n=summed.tolist(),original_filtered_force_n=force.tolist(),
        native_separations_m=gaps.tolist(),source_geometry_and_native_detail=instant,
        penetration_tolerance_m=None,whole_banana_containment_required=False,planar_floor_assumed=False,
        scope='Only original banana/bowl pair at actual poses. Original bounds center, reconstructed source intersections, native points in original bowl region. No native/source shape equivalence or per-point force cone.')

def geometry_binding(world):
    return dict(source_asset_sha256=dict(world.probe_source_hashes),
        banana_vertices_sha256=hashlib.sha256(world.payload_geometry.vertices.tobytes()).hexdigest(),
        banana_triangles_sha256=hashlib.sha256(world.payload_geometry.triangles.tobytes()).hexdigest(),
        bowl_vertices_sha256=hashlib.sha256(world.probe_bowl_geometry.vertices.tobytes()).hexdigest(),
        bowl_triangles_sha256=hashlib.sha256(world.probe_bowl_geometry.triangles.tobytes()).hexdigest(),
        actual_spawned_local_bounds=deepcopy(world.probe_source_binding['actual_spawned_local_bounds']))

def certificate_matches(certificate,*,physics_step,payload_pose,bowl_pose,geometry,attachment,native):
    try:
        body={k:v for k,v in certificate.items() if k!='certificate_sha256'}
        return bool(attachment is None and certificate['certificate_sha256']==digest(body) and
            certificate['physics_step']==physics_step and certificate['geometry']==geometry and
            np.array_equal(certificate['payload_pose'],matrix(payload_pose)) and
            np.array_equal(certificate['bowl_pose'],matrix(bowl_pose)) and
            certificate['native_observation']==native and native['physics_step']==physics_step and
            certificate['observed_support']['passed'] is True and
            certificate['observed_support']['contact_state']=='loaded_support')
    except (KeyError,ValueError,TypeError):return False

class SupportedBananaWorld(GeometryWorld):
    """Reuse frozen source geometry only; no diagnostic permissive continuation."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.support_certificates=[];self.planning_certificate=None;self.previous_observation=None
        self._last_observed_step=None

    def observe(self,row,native):
        step=int(row['physics_step'])
        require(native['physics_step']==step and native['phase']==row['phase'] and native['action_index']==row['action_index'],'Actual/native step mapping changed')
        require(self._last_observed_step is None or step==self._last_observed_step+1,'Actual support observations have gap/duplicate')
        if self.probe_row is not None:
            self.previous_observation=dict(physics_step=int(self.probe_row['physics_step']),
                phase=self.probe_row['phase'],native=deepcopy(self.probe_native),
                task_object_states=np.asarray(self.probe_row['task_object_states']).tolist())
        self.probe_row=row;self.probe_native=native;self._last_observed_step=step

    def issue_planning_certificate(self):
        require(self.phase=='retreat' and self.attachment is None and self.probe_row is not None,'Only latest released-object retreat certificate')
        source=self.source_bowl_observation(self.payload_pose)
        force=np.asarray(self.probe_row['destination_contact_force']).reshape(-1,3).sum(0)
        support=observed_support_checks(source,self.probe_native,force,self.probe_low,self.probe_high)
        require(support['passed'] and support['contact_state']=='loaded_support','Latest actual loaded native support required before retreat')
        cert=dict(physics_step=int(self.probe_row['physics_step']),payload_pose=matrix(self.payload_pose).tolist(),
            bowl_pose=matrix(self.probe_bowl_pose).tolist(),geometry=geometry_binding(self),
            native_observation=deepcopy(self.probe_native),source_observation=source,observed_support=support,
            scope='Static observed free banana/bowl pair only. No robot/bowl masking, no q-dependent payload motion.')
        cert['certificate_sha256']=digest(cert);self.planning_certificate=cert
        self.support_certificates.append(deepcopy(cert));return cert

    def _destination_measurement(self):
        obstacle=next(x for x in self.objects if x[0]==DESTINATION_MESH);pose=self._query_payload_pose
        shape=self.fcl.CollisionObject(self.payload_triangle_model,self.fcl.Transform(pose[:3,:3],pose[:3,3]))
        strict=self._exact_measurement((0.,'payload/banana',DESTINATION_MESH,shape,obstacle[1],.002 if self.phase=='lower' else 0.))
        strict.update(contact_pair='payload/destination',noncontact_required_m=.002,representation='Original banana/bowl source triangles; strict observation retained')
        if not self._probe_execution_active:
            if self.phase=='retreat':
                require(self.planning_certificate is not None and self.probe_row is not None,'Missing observed free-object retreat certificate')
                require(certificate_matches(self.planning_certificate,physics_step=int(self.probe_row['physics_step']),
                    payload_pose=pose,bowl_pose=self.probe_bowl_pose,geometry=geometry_binding(self),attachment=self.attachment,native=self.probe_native),'Changed or stale retreat support certificate')
                if not strict['valid']:return dict(strict,valid=True,original_source_strict_valid=False,
                    observed_static_support_certificate=self.planning_certificate['certificate_sha256'])
            return strict
        row=self.probe_row;native=self.probe_native
        require(row is not None and native is not None and native['physics_step']==row['physics_step'],'Missing actual native support observation')
        step=int(row['physics_step'])
        if step in self._probe_measurements:return deepcopy(self._probe_measurements[step])
        source=self.source_bowl_observation(pose)
        record=dict(physics_step=step,phase=self.phase,action_index=int(row['action_index']),source=source,
            original_strict_measurement=deepcopy(strict),allowed_original_support=False)
        answer=deepcopy(strict)
        if self.phase in SUPPORT_PHASES:
            require(self.attachment is None,'Only released free banana may use destination support')
            require(source['center_region']['passed'],'Original spawned bounds center left destination region')
            if self.phase=='release':
                require(self.release_start_step is not None and 1<=step-self.release_start_step<=240,'Release must be exactly declared 240 substeps')
            if not strict['valid']:
                force=np.asarray(row['destination_contact_force']).reshape(-1,3).sum(0)
                support=observed_support_checks(source,native,force,self.probe_low,self.probe_high)
                record['observed_support']=support
                if support['passed']:
                    answer.update(valid=True,original_source_strict_valid=False,allowed_original_support=True)
                    record['allowed_original_support']=True
        if self.previous_observation is not None:record['previous_observation']=deepcopy(self.previous_observation)
        if not strict['valid']:self.strict_failure_ledger.append(deepcopy(record))
        self.probe_records.append(record);self._probe_measurements[step]=deepcopy(answer);return answer

    def _measurement(self,pair):
        if pair[1]=='payload/banana' and pair[2]==DESTINATION_MESH and self.phase in TARGET_PHASES:
            return self._destination_measurement()
        return PayloadCollisionWorld._measurement(self,pair)

    def valid(self,q,ignore=(),*,mode='planning',gripper_q=None,object_pose=None,obstacle_filter=None):
        self._probe_execution_active=mode=='execution'
        try:
            if not PayloadCollisionWorld.valid(self,q,ignore,mode=mode,gripper_q=gripper_q,object_pose=object_pose,obstacle_filter=obstacle_filter):return False
            if self.phase in TARGET_PHASES and (self._probe_execution_active or self.phase=='retreat'):
                value=self._destination_measurement()
                if not value['valid']:
                    self.last_clearance=value;self.last_collision=(value['robot_part'],value['obstacle']);return False
            return True
        finally:self._probe_execution_active=False
