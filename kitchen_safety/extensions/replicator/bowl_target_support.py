"""Original bowl/BasinBottom intentional support, with fresh observed evidence.

This policy does not change physical colliders or any other clearance. Source
triangle intersection is retained as a representation observation for this
necessary contact pair. It is not a millimetre penetration allowance. The
original finite release/retreat/settle schedule and success predicate remain.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from .bowl_collision import BowlCollisionWorld
from .bowl_destination_query import destination_floor
from .bowl_source_support_query import collider_geometry_evidence, digest
from .task_contacts import _array

BODY = '/World/envs/env_0/bowl'
FLOOR = '/World/envs/env_0/kitchen/Sink/BasinBottom'
SUPPORT_PHASES = frozenset(('release', 'retreat', 'final_settle'))
MAX_CONTACTS = 4096
ROOT = Path(__file__).resolve().parents[3]


def require(value, message):
    if not value:
        raise ValueError(message)


def read_policy(path):
    value = json.loads(Path(path).read_text())
    require(value['schema_version'] == 1 and value['task_id'] == 'kitchen_bench_replicator_g_shape_bowl_sink', 'Wrong target support policy')
    require(value['payload'] == BODY and value['target'] == FLOOR, 'Only original intended pair is permitted')
    require(value['noncontact_clearance_m'] == .002 and value['lower_clearance_m'] == .002 and
        value['release_gap_window_m'] == [.003,.005] and value['landing_xy_tolerance_m'] == .005,
        'Original noncontact/lower/landing gates changed')
    require(value['release_seconds'] == 2. and value['retreat_height_m'] == .15 and value['final_settle_seconds'] == 4.,
        'Original fixed control schedule changed')
    require(value['force_threshold_n'] == .1 and value['support_cone_half_angle_rad'] == float(np.pi/4) and
        value['velocity_threshold_m_s'] == .003 and value['placement_consecutive_steps'] == 1,
        'Original task predicate changed')
    require(value['physics_dt_s'] == 1/120 and value['action_dt_s'] == 1/15 and value['contact_capacity'] == MAX_CONTACTS,
        'Original sampling rate/contact capacity changed')
    require(value['unloaded_transition_rule'] == 'complete_native_contacts_all_separations_nonnegative_and_no_downward_aggregate',
        'Unreviewed unloaded contact transition rule')
    for name, expected in value['evidence_sha256'].items():
        require(hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == expected, 'Support policy lineage changed: '+name)
    return value


def unpack_native(data):
    force, point, normal, separation, count, start = map(_array, data)
    require(count.size == start.size == 1, 'Require exact one-body/one-filter contact data')
    n, first = int(count.reshape(-1)[0]), int(start.reshape(-1)[0])
    require(0 <= n and 0 <= first and first+n < MAX_CONTACTS, 'Invalid/full native contact buffer')
    require(point.ndim == 2 and point.shape[1:] == (3,) and normal.shape == point.shape and
        first+n <= len(point) and force.size >= first+n and separation.size >= first+n, 'Incomplete contact buffers')
    s = slice(first, first+n)
    points, normals = point[s], normal[s]
    forces, gaps = force.reshape(-1)[s], separation.reshape(-1)[s]
    require(all(np.isfinite(v).all() for v in (points,normals,forces,gaps)), 'Nonfinite native contact')
    return dict(count=n, buffer_count=count.tolist(), buffer_start=start.tolist(), buffer_capacity=MAX_CONTACTS,
        points_world_m=points.tolist(), normals_world=normals.tolist(), normal_forces_n=forces.tolist(), separations_m=gaps.tolist())


class NativeTargetContacts:
    def __init__(self, physics_view, physics_dt, identity, root_view):
        self.identity = deepcopy(identity)
        require(identity['actual_pick_up_path'] == BODY and identity['actual_destination_path'] == FLOOR and
            identity['sensed_native_paths'] == [BODY] and identity['resolved_filter_prim_paths'] == [FLOOR] and
            identity['native_filter_count'] == 1 and physics_dt == 1/120, 'Wrong native source support identity')
        self.dt = physics_dt
        self.view = physics_view.create_rigid_contact_view([BODY], filter_patterns=[[FLOOR]], max_contact_data_count=MAX_CONTACTS)
        require(self.view.filter_count == 1, 'Wrong destination native filter count')
        self.rows, self.offsets = [], {}
        for name in ('get_rest_offsets','get_contact_offsets'):
            try:
                values = _array(getattr(root_view,name)())
                require(values.size and np.isfinite(values).all(), 'Invalid offset array')
                self.offsets[name] = dict(available=True, values_m=values.tolist(), shape=list(values.shape))
            except Exception as error:
                self.offsets[name] = dict(available=False, reason=repr(error))
        self.offsets['static_floor'] = dict(available=False, reason='No verified native static collider offset getter')

    def sample(self, row):
        value = dict(physics_step=int(row['physics_step']), phase=row['phase'], action_index=int(row['action_index']),
            exact_body=BODY, exact_filter=FLOOR, **unpack_native(self.view.get_contact_data(dt=self.dt)))
        contributions = np.asarray(value['normal_forces_n'])[:,None]*np.asarray(value['normals_world']).reshape(-1,3)
        summed = contributions.sum(0)
        original = np.asarray(row['destination_contact_force']).reshape(-1,3).sum(0)
        require(np.isfinite(original).all(), 'Missing original filtered destination force')
        value.update(native_contact_force_vectors_n=contributions.tolist(), native_normal_force_vector_sum_n=summed.tolist(),
            original_filtered_force_n=original.tolist(), native_minus_original_force_n=(summed-original).tolist(),
            native_original_force_difference_norm_n=float(np.linalg.norm(summed-original)), force_difference_is_diagnostic_only=True)
        self.rows.append(value)
        return value


def observed_support_checks(source, native, filtered_force, low, high):
    """Original aggregate support, source entity bounds, full native records.

The API defines scalar*normal, not universal scalar sign. No per-point +Z cone
is added to the original aggregate-force predicate. Negative separation and
source overlap are retained; no tolerance is fitted from their maxima.
    """
    low, high = np.asarray(low), np.asarray(high)
    force = np.asarray(filtered_force, float)
    points = np.asarray(native['points_world_m'], float).reshape(-1,3)
    normals = np.asarray(native['normals_world'], float).reshape(-1,3)
    loads, gaps = np.asarray(native['normal_forces_n'], float), np.asarray(native['separations_m'], float)
    complete = bool(len(points)==len(normals)==len(loads)==len(gaps)==native['count'] and
        all(np.isfinite(v).all() for v in (points,normals,loads,gaps,force)))
    contributions = loads[:,None]*normals if complete else np.empty((0,3))
    summed = contributions.sum(0)
    def supported(vector):
        size = float(np.linalg.norm(vector))
        return size >= .1 and vector[2]/size >= np.cos(np.pi/4)
    loaded_support = bool(complete and supported(force) and supported(summed))
    # Raw potential contacts with nonnegative native separation can be a
    # legitimate short unloading/bounce transition. They do not prove support
    # or success. Require the original pair, complete native geometry, source
    # spatial bounds, and no downward aggregate; never infer unseen contacts.
    transition = bool(complete and len(points) and np.all(gaps>=0.) and force[2]>=0. and summed[2]>=0.)
    state = 'loaded_support' if loaded_support else ('observed_nonpenetrating_contact_transition' if transition else 'rejected')
    checks = dict(exact_original_pair=native.get('exact_body')==BODY and native.get('exact_filter')==FLOOR,
        complete_native_contacts=complete and len(points)>0,
        observed_support_or_nonpenetrating_transition=loaded_support or transition,
        native_points_inside_original_floor_xy=bool(len(points) and np.all(points[:,:2]>=low[:2]) and np.all(points[:,:2]<=high[:2])),
        native_points_above_original_floor_bottom=bool(len(points) and np.all(points[:,2]>=low[2])),
        all_source_bowl_xy_inside_original_floor=bool(source['full_bowl_xy_inside_floor']),
        source_bowl_above_original_floor_bottom=bool(source['minimum_source_z_m']>=low[2]))
    return dict(passed=all(checks.values()), checks=checks, contact_state=state,
        original_loaded_support=loaded_support, nonloaded_transition_is_not_success=True,
        native_force_vector_sum_n=summed.tolist(),
        original_filtered_force_n=force.tolist(), native_minus_original_force_n=(summed-force).tolist(),
        native_separations_m=gaps.tolist(), original_source_gap_m=source['source_gap_m'],
        penetration_tolerance_m=None, per_point_upward_cone_required=False,
        scope='Permitted original target pair only. Loaded original aggregate support or complete nonnegative-native-separation contact transition, both within source entity bounds. No zero-source-intersection claim or fitted penetration allowance. Speed remains the original final task-success gate.')


def release_boundary(source, plan_complete, xy_error_m, contacts, step):
    checks = dict(plan_complete=bool(plan_complete), original_lower_2mm=not source['intersecting'] and source['distance_m']>=.002,
        declared_gap_window=.003<=source['source_gap_m']<=.005, original_xy_error=bool(xy_error_m<=.005),
        full_source_xy=bool(source['full_bowl_xy_inside_floor']), last_ten_physical_steps=[r['physics_step'] for r in contacts.get('steps',[])]==list(range(step-9,step+1)),
        bilateral_loaded=bool(contacts.get('both_fingers_loaded')), bilateral_near=bool(contacts.get('both_fingers_near')))
    return dict(passed=all(checks.values()),checks=checks,physics_step=int(step),source=source,xy_error_m=float(xy_error_m),contacts=contacts)


def certificate_matches(certificate, *, physics_step, payload_pose, floor_hash, payload_hash, attachment):
    body = {k:v for k,v in certificate.items() if k!='certificate_sha256'}
    return bool(attachment is None and certificate.get('certificate_sha256')==digest(body) and
        certificate['physics_step']==physics_step and certificate['floor_geometry_sha256']==floor_hash and
        certificate['payload_source_sha256']==payload_hash and np.array_equal(certificate['payload_pose'],payload_pose) and
        certificate['observed_support']['passed'] is True and certificate['observed_support']['contact_state']=='loaded_support')


class SupportedDestinationWorld(BowlCollisionWorld):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.floor_binding=collider_geometry_evidence(self,destination_floor(self)[0])
        self.payload_source_sha=self.payload_geometry.provenance['source_sha256']
        self.observation=None;self.previous_observation=None;self.native=None
        self.support_records=[];self.source_intersection_ledger=[];self.support_certificates=[]
        self._execution_active=False;self._measurements={};self.planning_certificate=None

    def observe(self,row,native):
        step=int(row['physics_step'])
        require(native['physics_step']==step,'Native/actual physical step mismatch')
        require(self.observation is None or step==int(self.observation['physics_step'])+1,'Missing consecutive actual support state')
        self.previous_observation=self.observation
        self.observation=row;self.native=native

    def source_floor_observation(self,pose):
        floor=destination_floor(self);binding=collider_geometry_evidence(self,floor[0])
        require(binding['geometry_sha256']==self.floor_binding['geometry_sha256'] and
            self.payload_geometry.provenance['source_sha256']==self.payload_source_sha,'Support source identity changed')
        pose=np.asarray(pose);shape=self.fcl.CollisionObject(self.payload_triangle_model,self.fcl.Transform(pose[:3,:3],pose[:3,3]))
        distance=float(self.fcl.distance(shape,floor[1],self.fcl.DistanceRequest(),self.fcl.DistanceResult()))
        result=self.fcl.CollisionResult();n=int(self.fcl.collide(shape,floor[1],self.fcl.CollisionRequest(num_max_contacts=MAX_CONTACTS,enable_contact=True),result))
        require(np.isfinite(distance) and n<MAX_CONTACTS and len(result.contacts)<MAX_CONTACTS,'Incomplete exact source contact inventory')
        vertices=self.payload_geometry.vertices@pose[:3,:3].T+pose[:3,3];low,high=vertices.min(0),vertices.max(0)
        return dict(distance_m=max(0.,distance),intersecting=bool(n or distance<0),source_gap_m=float(low[2]-floor[3][2]),
            minimum_source_z_m=float(low[2]),full_bowl_xy_inside_floor=bool(np.all(low[:2]>=floor[2][:2]) and np.all(high[:2]<=floor[3][:2])),
            floor_bounds=[floor[2].tolist(),floor[3].tolist()],exact_floor_geometry_sha256=binding['geometry_sha256'],
            # FCL contact.pos is saved only as raw diagnostic, never an exact
            # triangle-intersection-region proof or support acceptance input.
            contacts=[dict(point_world_m=np.asarray(c.pos).tolist(),normal_world=np.asarray(c.normal).tolist(),
                source_triangle=int(c.b1),floor_triangle=int(c.b2),penetration_depth_m=float(c.penetration_depth)) for c in result.contacts])

    def issue_planning_certificate(self):
        require(self.phase=='retreat' and self.attachment is None and self.observation is not None,'Retreat certificate requires observed released bowl')
        source=self.source_floor_observation(self.payload_pose);floor=destination_floor(self)
        force=np.asarray(self.observation['destination_contact_force']).reshape(-1,3).sum(0)
        support=observed_support_checks(source,self.native,force,floor[2],floor[3])
        require(support['passed'] and support['contact_state']=='loaded_support',
            'Latest measured loaded target support does not justify a static retreat certificate')
        cert=dict(physics_step=int(self.observation['physics_step']),payload_pose=np.asarray(self.payload_pose).tolist(),
            floor_geometry_sha256=self.floor_binding['geometry_sha256'],payload_source_sha256=self.payload_source_sha,
            native_observation=deepcopy(self.native),observed_support=support,
            scope='Previously observed fixed free-payload/target pair only; no planning-generated contact or robot collision exemption')
        cert['certificate_sha256']=digest(cert);self.planning_certificate=cert;self.support_certificates.append(deepcopy(cert));return cert

    def _floor_measurement(self):
        pose=self._query_payload_pose;floor=destination_floor(self)
        shape=self.fcl.CollisionObject(self.payload_triangle_model,self.fcl.Transform(pose[:3,:3],pose[:3,3]))
        margin=.002 if self.phase=='lower' else 0.
        strict=self._exact_measurement((0.,'payload/bowl',FLOOR,shape,floor[1],margin))
        strict.update(contact_pair='payload/destination',noncontact_required_m=.002)
        if not self._execution_active:
            if self.phase=='retreat' and not strict['valid']:
                require(self.planning_certificate is not None and self.observation is not None,'Missing observed static support certificate')
                binding=collider_geometry_evidence(self,FLOOR)
                require(certificate_matches(self.planning_certificate,physics_step=int(self.observation['physics_step']),payload_pose=pose,
                    floor_hash=binding['geometry_sha256'],payload_hash=self.payload_source_sha,attachment=self.attachment),'Stale/changed planning support certificate')
                return dict(strict,valid=True,original_source_strict_valid=False,observed_static_support_certificate=self.planning_certificate['certificate_sha256'])
            return strict
        require(self.observation is not None and self.native is not None,'Missing actual native target observation')
        step=int(self.observation['physics_step'])
        if step in self._measurements:return deepcopy(self._measurements[step])
        source=self.source_floor_observation(pose)
        record=dict(physics_step=step,phase=self.phase,source=source,original_strict_measurement=deepcopy(strict),allowed_original_support=False)
        if self.previous_observation is not None:
            from .task_motion import pose_matrix
            previous=pose_matrix(self.previous_observation['task_object_states'][0,:7])
            record.update(previous_physics_step=int(self.previous_observation['physics_step']),previous_source=self.source_floor_observation(previous),
                timing_scope='Both consecutive measured states retained; no undocumented API sampling-time equivalence assumed')
        answer=deepcopy(strict)
        if self.phase in SUPPORT_PHASES:
            require(self.attachment is None,'Expected support is only for released free bowl')
            require(source['full_bowl_xy_inside_floor'] and source['minimum_source_z_m']>=floor[2][2],'Bowl crossed original floor footprint/entity lower boundary')
            if not strict['valid']:
                force=np.asarray(self.observation['destination_contact_force']).reshape(-1,3).sum(0)
                observed=observed_support_checks(source,self.native,force,floor[2],floor[3]);record['observed_support']=observed
                if observed['passed']:
                    answer.update(valid=True,original_source_strict_valid=False,allowed_original_support=True)
                    record['allowed_original_support']=True
                self.source_intersection_ledger.append(deepcopy(record))
        self.support_records.append(record);self._measurements[step]=deepcopy(answer);return answer

    def _measurement(self,pair):
        if pair[1]=='payload/bowl' and pair[2]==FLOOR and self.phase in SUPPORT_PHASES|{'lower'}:
            return self._floor_measurement()
        return super()._measurement(pair)

    def valid(self,q,ignore=(),*,mode='planning',gripper_q=None,object_pose=None,obstacle_filter=None):
        self._execution_active=mode=='execution'
        try:
            if not super().valid(q,ignore,mode=mode,gripper_q=gripper_q,object_pose=object_pose,obstacle_filter=obstacle_filter):return False
            if self._execution_active and self.phase in SUPPORT_PHASES|{'lower'}:
                measurement=self._floor_measurement()
                if not measurement['valid']:
                    self.last_clearance=measurement;self.last_collision=(measurement['robot_part'],measurement['obstacle']);return False
            return True
        finally:self._execution_active=False
