"""Bounded banana/bowl diagnostic only; no production collector imports this.

Actual lower remains >=2 mm. At most 56 release substeps may observe an
original source-triangle rejection with complete spatially bounded observations.
Rim support is allowed by the source task, but no diagnostic can be accepted.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
from .task_collision import PayloadCollisionWorld, PayloadGeometry
from .task_contacts import _array
from .task_motion import pose_matrix

ROOT = Path(__file__).resolve().parents[3]
TASK = "kitchen_bench_replicator_g_shape_banana_bowl"
BODY = "/World/envs/env_0/banana"
DESTINATION = "/World/envs/env_0/bowl"
DESTINATION_MESH = DESTINATION + "/obj_000013_Mesh"
MAX_CONTACTS = 4096
SOURCE_ASSETS = {
    "banana": ("assets/extensions/replicator/source/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/ycb/banana.usd", "a0aee64b0d9f6aae5689c1787e176776696ff38f23d17da0e226da19f0d9b064"),
    "bowl": ("assets/robolab/objects/ycb/bowl.usd", "5f968172a13915a4db8f20d1ba17e794f825153d5f72d26d699252a4ca79c81e"),
}
ORIGINAL_COLLECTOR_SHA256 = "9b105d072423f5cc23039b85c7c181d7b175d417664f663779bb4ef1b8649e95"
LEGACY_DECLARATION = dict(schema_version=1, task_id=TASK, diagnostic_only=True, accepted=False,
    physical_trial_passed=False, task_success=False, noncontact_clearance_m=.002,
    release_gap_window_m=[.003,.005], maximum_extra_lower_hold_actions=5,
    maximum_release_actions=7, maximum_release_physics_steps=56,
    physics_dt_s=1/120, action_dt_s=1/15, payload_body=BODY, release_target=DESTINATION,
    release_mesh=DESTINATION_MESH, force_threshold_n=.1, velocity_threshold_m_s=.003,
    support_cone_half_angle_rad=float(np.pi/4), placement_consecutive_steps=1,
    contact_capacity=MAX_CONTACTS, source_assets={k:dict(path=v[0],sha256=v[1]) for k,v in SOURCE_ASSETS.items()},
    original_collector_sha256=ORIGINAL_COLLECTOR_SHA256,
    continuation_scope="Only exact source banana/bowl intersection during bounded release; strict rejection retained. No source/native penetration tolerance or accepted task output.",
    source_region_rule="Reconstructed triangle intersections and native contact points in actual bowl-local XY footprint and above its lower Z bound; actual banana spawned-bounds center obeys the same original spatial predicate. No full banana containment or flat-floor assumption.",
    native_support_rule="Positive raw normal loads must point in original world +Z cone; original filtered force magnitude/direction must pass. Zero-load points retained without establishing support.",
    surface_location_rule="Triangle-pair clipping only; approximate FCL contact.pos never permits continuation.")
VECTOR_DECLARATION = dict(LEGACY_DECLARATION, schema_version=2,
    native_support_rule="Preserve signed raw scalar forces and normals. Sum scalar-times-normal vectors as documented by the installed native API. This native aggregate and the original filtered force must each satisfy the original magnitude and world +Z cone. No individual normal/scalar sign requirement; zero vectors do not establish support.",
    native_force_agreement_rule="Record native-minus-original force without a numerical agreement gate: the compiled backend aggregation order is unavailable, so no unsupported rounding bound is imposed.")
DECLARATION = dict(VECTOR_DECLARATION, schema_version=3,
    native_support_rule="Every bounded release substep needs complete finite native data for the exact pair. A complete empty contact buffer, zero aggregate, or transient non-upward force is an observation, not missing data or terminal task failure. Preserve all signed scalars, normals and separations; nonempty native points must remain in the original destination region.",
    terminal_predicate_rule="Record the unchanged original predicate every physics step and report its final value after exactly 56 release steps. Do not require intermediate support or rest, and never turn a diagnostic into an accepted task.",
    continuation_scope="Only exact source banana/bowl intersection during bounded release; strict rejection retained. Source intersection clipping and original center region, complete exact native observation and every other original pair remain mandatory. No source/native penetration tolerance or accepted task output.")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_declaration(path):
    value=json.loads(Path(path).read_text())
    if value != DECLARATION:
        raise ValueError("Diagnostic declaration differs from reviewed exact banana rules")
    for name,(path,expected) in SOURCE_ASSETS.items():
        if digest(ROOT/path) != expected:
            raise ValueError("Diagnostic source asset changed: "+name)
    if digest(ROOT/'scripts/extensions/replicator/task_collect.py') != ORIGINAL_COLLECTOR_SHA256:
        raise ValueError("Original collector changed since diagnostic fork")
    return value


def audit_declaration_archive(provenance_report, binding):
    entries=provenance_report['entries']
    key=binding['path']; item=entries[key]
    if (item['kind']!='diagnostic_declaration_input' or
        item['initial_sha256']!=item['checked_sha256'] or
        item['initial_sha256']!=binding['sha256'] or not item['unchanged'] or
        digest_text(item['source_text'])!=binding['sha256'] or
        json.loads(item['source_text']) not in (LEGACY_DECLARATION,VECTOR_DECLARATION,DECLARATION)):
        raise ValueError('Diagnostic declaration archive missing/changed/ambiguous')
    module=entries[binding['module_path']]
    if (binding['module_path']!='kitchen_safety/extensions/replicator/banana_destination_probe.py' or
        module['kind']!='execution_source' or not module['unchanged'] or
        module['initial_sha256']!=module['checked_sha256'] or module['initial_sha256']!=binding['module_sha256'] or
        digest_text(module['source_text'])!=binding['module_sha256']):
        raise ValueError('Diagnostic module archive missing/changed')
    original=entries['scripts/extensions/replicator/task_collect.py']
    if (original['initial_sha256']!=original['checked_sha256'] or
        original['initial_sha256']!=ORIGINAL_COLLECTOR_SHA256):
        raise ValueError('Original frozen collector archive differs')
    return dict(passed=True, binding=deepcopy(binding), kind=item['kind'])


def digest_text(text):
    return hashlib.sha256(text.encode()).hexdigest()


def matrix(value):
    value=np.asarray(value,float)
    if value.shape!=(4,4):return pose_matrix(value)
    if (not np.isfinite(value).all() or not np.array_equal(value[3],[0.,0.,0.,1.]) or
        not np.allclose(value[:3,:3].T@value[:3,:3],np.eye(3),atol=1e-8,rtol=0) or np.linalg.det(value[:3,:3])<=0):
        raise ValueError('Expected a finite rigid pose')
    return value

def unpack_native_contact(data, *, capacity=MAX_CONTACTS):
    """Exact one-body/one-filter raw contact inventory with no truncation."""
    force, point, normal, separation, count, start = map(_array, data)
    if (count.shape!=(1,1) or start.shape!=(1,1) or
        count.dtype.kind not in 'iu' or start.dtype.kind not in 'iu'):
        raise ValueError("Need exactly one banana/bowl raw contact pair")
    n, first = int(count.reshape(-1)[0]), int(start.reshape(-1)[0])
    if n < 0 or first < 0 or first+n >= capacity:
        raise ValueError("Destination raw contact buffer invalid or at declared capacity")
    if (force.ndim!=2 or force.shape[1:]!=(1,) or separation.ndim!=2 or separation.shape[1:]!=(1,) or
        point.ndim != 2 or point.shape[1:] != (3,) or normal.shape != point.shape or
        first+n > len(point) or force.size < first+n or separation.size < first+n):
        raise ValueError("Destination contact arrays are incomplete")
    selected = slice(first, first+n)
    points, normals = point[selected], normal[selected]
    forces, gaps = force.reshape(-1)[selected], separation.reshape(-1)[selected]
    if not all(np.isfinite(v).all() for v in (points, normals, forces, gaps)):
        raise ValueError("Nonfinite destination raw contact")
    return dict(count=n, buffer_count=count.tolist(), buffer_start=start.tolist(),
        buffer_capacity=capacity, points_world_m=points.tolist(), normals_world=normals.tolist(),
        normal_forces_n=forces.tolist(), separations_m=gaps.tolist())


def native_offsets(root_view):
    """Read only supported tensor getters; unavailable is never replaced by 0."""
    report = {}
    for name in ("get_rest_offsets", "get_contact_offsets"):
        try:
            values = _array(getattr(root_view, name)())
            if not values.size or not np.isfinite(values).all():
                raise ValueError("Empty/nonfinite native offset array")
            report[name] = dict(available=True, values_m=values.tolist(), shape=list(values.shape), actual_shape_slots_verified=False,
                interpretation="Capacity-shaped native offset array; actual valid shape IDs/slots were not enumerated. Padding is not actual collision geometry.")
        except Exception as error:
            report[name] = dict(available=False, reason=f"{type(error).__name__}: {error}")
    return report


def plane_cut(triangle, point, normal, epsilon):
    distances = (triangle - point) @ normal
    points = [value for value, distance in zip(triangle, distances) if abs(distance) <= epsilon]
    for i, j in ((0, 1), (1, 2), (2, 0)):
        if distances[i] * distances[j] < 0:
            fraction = distances[i] / (distances[i] - distances[j])
            points.append(triangle[i] + fraction * (triangle[j] - triangle[i]))
    return np.array(points).reshape(-1, 3)


def triangle_intersection(first, second, epsilon=1e-10):
    """Floating point source-surface intersection; epsilon is diagnostic only."""
    first, second = np.asarray(first, float), np.asarray(second, float)
    na, nb = np.cross(first[1] - first[0], first[2] - first[0]), np.cross(second[1] - second[0], second[2] - second[0])
    if np.linalg.norm(na) < 1e-18 or np.linalg.norm(nb) < 1e-18:
        raise ValueError("Degenerate source triangle in reported collision pair")
    na, nb = na / np.linalg.norm(na), nb / np.linalg.norm(nb)
    direction = np.cross(na, nb)
    if np.linalg.norm(direction) < 1e-9:
        if np.max(np.abs((second - first[0]) @ na)) > epsilon:
            return np.empty((0, 3))
        # Coplanar convex triangle clipping; keep exact interpolated 3-D points.
        polygon = list(first)
        for i, j in ((0, 1), (1, 2), (2, 0)):
            inward = np.cross(nb, second[j] - second[i])
            inward /= np.linalg.norm(inward)
            if not polygon:
                break
            old, polygon = np.array(polygon), []
            for a, b in zip(old, np.roll(old, -1, axis=0)):
                da, db = (a - second[i]) @ inward, (b - second[i]) @ inward
                ia, ib = da >= -epsilon, db >= -epsilon
                if ia:
                    polygon.append(a)
                if ia != ib:
                    polygon.append(a + da / (da - db) * (b - a))
        return np.asarray(polygon).reshape(-1, 3)
    direction /= np.linalg.norm(direction)
    a, b = plane_cut(first, second[0], nb, epsilon), plane_cut(second, first[0], na, epsilon)
    if not len(a) or not len(b):
        return np.empty((0, 3))
    aa, bb = a @ direction, b @ direction
    low, high = max(aa.min(), bb.min()), min(aa.max(), bb.max())
    if high < low - epsilon:
        return np.empty((0, 3))
    high = max(low, high)
    return np.array([a[0] + (value - aa[0]) * direction for value in (low, high)])


class DestinationNativeContacts:
    def __init__(self, physics_view, physics_dt, sensor_identity, banana_view, bowl_view):
        self.identity=deepcopy(sensor_identity)
        if (self.identity['actual_pick_up_path']!=BODY or self.identity['actual_destination_path']!=DESTINATION or
            self.identity['sensed_native_paths']!=[BODY] or self.identity['resolved_filter_prim_paths']!=[DESTINATION] or
            self.identity['native_filter_count']!=1 or physics_dt!=DECLARATION['physics_dt_s']):
            raise ValueError('Independent native view must retain original banana/bowl identity and dt')
        self.dt=physics_dt
        self.view=physics_view.create_rigid_contact_view([BODY],filter_patterns=[[DESTINATION]],max_contact_data_count=MAX_CONTACTS)
        if int(self.view.filter_count)!=1 or int(self.view.sensor_count)!=1:
            raise ValueError('Independent native contact view has wrong pair inventory')
        self.offsets=dict(banana=native_offsets(banana_view),bowl=native_offsets(bowl_view))
        self.rows=[]

    def sample(self,row):
        raw=unpack_native_contact(self.view.get_contact_data(dt=self.dt))
        original=np.asarray(row['destination_contact_force']).reshape(-1,3).sum(0)
        if not np.isfinite(original).all():raise ValueError('Nonfinite original filtered force')
        normals=np.asarray(raw['normals_world']).reshape(-1,3);loads=np.asarray(raw['normal_forces_n'])
        summed=(loads[:,None]*normals).sum(0)
        value=dict(raw,physics_step=int(row['physics_step']),phase=str(row['phase']),action_index=int(row['action_index']),
            exact_body=BODY,exact_filter=DESTINATION,native_normal_force_vector_sum_n=summed.tolist(),
            original_filtered_force_n=original.tolist(),native_minus_original_force_n=(summed-original).tolist(),
            native_original_force_difference_norm_n=float(np.linalg.norm(summed-original)),force_difference_is_diagnostic_only=True)
        self.rows.append(value)
        return value


def original_region(points,bowl_pose,low,high):
    points=np.asarray(points,float).reshape(-1,3)
    transform=matrix(bowl_pose)
    local=(points-transform[:3,3])@transform[:3,:3]
    valid=bool(len(local) and np.isfinite(local).all() and np.all(local[:,:2]>=low[:2]) and
        np.all(local[:,:2]<=high[:2]) and np.all(local[:,2]>=low[2]))
    return dict(passed=valid,points_bowl_local_m=local.tolist())


def support_continuation_checks(source,native,filtered_force,low,high):
    """Curved bowl/rim region and native support; never a source-pos permission."""
    low,high,force=np.asarray(low),np.asarray(high),np.asarray(filtered_force)
    points=np.asarray(native['points_world_m'],float).reshape(-1,3)
    normals=np.asarray(native['normals_world'],float).reshape(-1,3)
    loads=np.asarray(native['normal_forces_n'],float);gaps=np.asarray(native['separations_m'],float)
    complete=bool(len(points)==len(normals)==len(loads)==len(gaps)==native['count'] and
        all(np.isfinite(v).all() for v in (points,normals,loads,gaps,force)))
    # Native API defines net pair force as sum(raw_scalar * raw_normal).
    # Either raw factor may be signed. Never replace the scalar with abs(),
    # and do not impose a flat-surface per-point normal cone on a curved bowl.
    contributions=loads[:,None]*normals if complete else np.empty((0,3))
    summed=contributions.sum(0);native_magnitude=float(np.linalg.norm(summed))
    nonzero=np.linalg.norm(contributions,axis=1)>0.
    clipped=[np.asarray(c['intersection_points_world_m']).reshape(-1,3) for c in source['contacts']]
    reconstructed=bool(clipped and all(len(v)>0 and np.isfinite(v).all() for v in clipped))
    sourcepoints=np.concatenate(clipped) if reconstructed else np.empty((0,3))
    sourceregion=original_region(sourcepoints,source['actual_bowl_pose'],low,high)
    nativeregion=original_region(points,source['actual_bowl_pose'],low,high)
    centerrule=original_region([source['actual_spawned_bounds_center_world_m']],source['actual_bowl_pose'],low,high)
    magnitude=float(np.linalg.norm(force))
    checks=dict(native_exact_identity=native.get('exact_body')==BODY and native.get('exact_filter')==DESTINATION,
        raw_contact_present=native['count']>0,raw_contact_complete=complete,
        raw_contact_has_nonzero_force_vector=bool(nonzero.any()),native_aggregate_force_above_threshold=native_magnitude>=.1,
        native_aggregate_force_in_upward_cone=bool(native_magnitude>0. and summed[2]/native_magnitude>=np.cos(np.pi/4)),
        original_force_above_threshold=magnitude>=.1,
        original_force_in_upward_cone=bool(magnitude>0. and force[2]/magnitude>=np.cos(np.pi/4)),
        source_contacts_reconstructed_from_triangles=reconstructed,
        source_intersections_in_original_bowl_region=sourceregion['passed'],native_points_in_original_bowl_region=nativeregion['passed'],
        original_spawned_center_in_destination_region=centerrule['passed'],
        exact_source_assets=source['source_asset_sha256']=={k:v[1] for k,v in SOURCE_ASSETS.items()})
    return dict(passed=all(checks.values()),checks=checks,source_intersection_region=sourceregion,native_contact_region=nativeregion,
        original_center_region=centerrule,native_normal_force_vector_sum_n=summed.tolist(),
        native_signed_contact_force_vectors_n=contributions.tolist(),
        raw_scalar_sign_counts=dict(positive=int((loads>0).sum()),negative=int((loads<0).sum()),zero=int((loads==0).sum())),
        nonzero_force_vector_count=int(nonzero.sum()),
        original_filtered_force_n=force.tolist(),native_minus_original_force_n=(summed-force).tolist(),
        force_difference_is_diagnostic_only=True,scope='Bounded observation only. Rim support allowed; no planar-floor, penetration tolerance, or task acceptance inference.')


def release_boundary(source,*,plan_complete,contact_summary,current_step,reset_step,original_other_checks_passed):
    steps=[x['physics_step'] for x in contact_summary.get('steps',[])]
    checks=dict(original_plan_complete=bool(plan_complete),complete_action_boundary=(current_step-reset_step)%8==0,
        actual_source_noncontact_2mm=bool(not source['intersecting'] and source['distance_m']>=.002),
        current_exact_source_gap_in_window=.003<=source['distance_m']<=.005,
        latest_ten_actual_lower_steps=steps==list(range(current_step-9,current_step+1)),
        original_lower_bilateral_loaded=bool(contact_summary.get('both_fingers_loaded')),
        original_lower_bilateral_near=bool(contact_summary.get('both_fingers_near')),
        original_task_object_role=contact_summary.get('object_name')=='banana' and contact_summary.get('phase')=='lower',
        original_center_in_destination_region=bool(source['center_region']['passed']),
        other_original_checks=bool(original_other_checks_passed))
    return dict(passed=all(checks.values()),checks=checks,physics_step=int(current_step),source=source,contact_summary=contact_summary)


def bounded_transition_checks(source,native,filtered_force,low,high):
    """Diagnostic observation guards, not a contact/safety/success certificate.

    A valid empty native buffer is evidence of zero reported contacts. Neither
    its emptiness nor transient velocity/force can prove the terminal outcome.
    """
    instantaneous=support_continuation_checks(source,native,filtered_force,low,high)
    prior=instantaneous['checks'];count=native['count']
    intersecting=source['intersecting']
    checks=dict(native_exact_identity=prior['native_exact_identity'],
        raw_contact_complete=prior['raw_contact_complete'],
        raw_contact_count_valid=type(count) is int and 0<=count<MAX_CONTACTS,
        source_inventory_consistent=bool(intersecting)==bool(source['contacts']),
        source_intersections_reconstructed=(not intersecting or prior['source_contacts_reconstructed_from_triangles']),
        source_intersections_in_original_region=(not intersecting or prior['source_intersections_in_original_bowl_region']),
        native_points_in_original_region=(count==0 or prior['native_points_in_original_bowl_region']),
        original_spawned_center_in_destination_region=prior['original_spawned_center_in_destination_region'],
        exact_source_assets=prior['exact_source_assets'])
    return dict(passed=all(checks.values()),checks=checks,
        observation_state=('complete_empty_native' if count==0 else
            'reported_contacts_zero_vector_sum' if np.linalg.norm(instantaneous['native_normal_force_vector_sum_n'])==0. else
            'reported_contacts_nonzero_vector_sum'),
        instantaneous_support_diagnostic=instantaneous,
        instantaneous_support_required_for_observation=False,
        native_separations_m=deepcopy(native['separations_m']),
        accepted=False,task_success=False,physical_trial_passed=False,
        scope='At most 56 steps of the exact intended pair. Preserve source rejection and native observations; no cooked-shape equality, penetration tolerance, stable support or production permission is inferred.')


class DiagnosticDestinationWorld(PayloadCollisionWorld):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        if self.payload_name!='banana':raise ValueError('This probe is exact banana/bowl only')
        self.probe_bowl_geometry=PayloadGeometry(ROOT/SOURCE_ASSETS['bowl'][0])
        self.probe_source_hashes={name:digest(ROOT/path) for name,(path,_) in SOURCE_ASSETS.items()}
        if self.probe_source_hashes!={name:v[1] for name,v in SOURCE_ASSETS.items()}:
            raise ValueError('Original source banana/bowl identity changed')
        if self.payload_geometry.provenance['source_sha256']!=SOURCE_ASSETS['banana'][1]:
            raise ValueError('Wrong payload collision model')
        self.probe_low,self.probe_high=self.probe_bowl_geometry.low.copy(),self.probe_bowl_geometry.high.copy()
        self.probe_payload_bounds_center=(self.payload_geometry.low+self.payload_geometry.high)/2
        self.probe_row=self.probe_native=self.probe_bowl_pose=None
        self.probe_records=[];self.strict_failure_ledger=[];self._probe_measurements={}
        self._probe_execution_active=False;self.release_start_step=None
        self.probe_source_binding=dict(source_asset_sha256=self.probe_source_hashes,
            bowl_root=DESTINATION,bowl_mesh=DESTINATION_MESH,payload_root=BODY,
            bowl_local_bounds_m=[self.probe_low.tolist(),self.probe_high.tolist()],
            payload_local_bounds_center_m=self.probe_payload_bounds_center.tolist(),
            source_surface_method='Original triangle-pair intersection reconstruction, never approximate FCL contact.pos')

    def bind_spawned_bounds(self,actual):
        for name,geometry in [('banana',self.payload_geometry),('bowl',self.probe_bowl_geometry)]:
            value=actual[name]
            if not np.allclose([value['min'],value['max']],[geometry.low,geometry.high],atol=1e-12,rtol=0):
                raise ValueError('Actual spawned bounds differ from declared original source geometry')
        self.probe_low=np.asarray(actual['bowl']['min']);self.probe_high=np.asarray(actual['bowl']['max'])
        self.probe_payload_bounds_center=(np.asarray(actual['banana']['min'])+actual['banana']['max'])/2
        self.probe_source_binding['actual_spawned_local_bounds']=deepcopy(actual)

    def source_bowl_observation(self,pose):
        if self.probe_bowl_pose is None:raise ValueError('Missing actual bowl pose for exact destination observation')
        bowlpose=matrix(self.probe_bowl_pose);pose=matrix(pose)
        matched=[x for x in self.objects if self._matches(x[0],'bowl')]
        if len(matched)!=1 or matched[0][0]!=DESTINATION_MESH:raise ValueError('Wrong destination collider inventory')
        obstacle=matched[0];dv,df=self.meshes[DESTINATION_MESH]
        expected=self.probe_bowl_geometry.vertices@bowlpose[:3,:3].T+bowlpose[:3,3]
        if not np.array_equal(df,self.probe_bowl_geometry.triangles) or not np.allclose(dv,expected,atol=1e-10,rtol=0):
            raise ValueError('Actual FCL bowl geometry is not the original source at its observed pose')
        bv=self.payload_geometry.vertices@pose[:3,:3].T+pose[:3,3]
        shape=self.fcl.CollisionObject(self.payload_triangle_model,self.fcl.Transform(pose[:3,:3],pose[:3,3]))
        distance=float(self.fcl.distance(shape,obstacle[1],self.fcl.DistanceRequest(),self.fcl.DistanceResult()))
        cr=self.fcl.CollisionResult();n=int(self.fcl.collide(shape,obstacle[1],self.fcl.CollisionRequest(num_max_contacts=MAX_CONTACTS,enable_contact=True),cr))
        if not np.isfinite(distance) or n>=MAX_CONTACTS or len(cr.contacts)>=MAX_CONTACTS:
            raise ValueError('Incomplete source triangle contact inventory')
        contacts=[]
        for c in cr.contacts:
            bt=bv[self.payload_geometry.triangles[c.b1]];dt=dv[df[c.b2]]
            intersection=triangle_intersection(bt,dt)
            if not len(intersection):raise ValueError('Source triangle pair could not be independently reconstructed')
            contacts.append(dict(banana_triangle_index=int(c.b1),bowl_triangle_index=int(c.b2),
                banana_triangle_world_m=bt.tolist(),bowl_triangle_world_m=dt.tolist(),intersection_points_world_m=intersection.tolist(),
                source_pair_fcl_penetration_depth_m=float(c.penetration_depth),
                fcl_approximate_contact_position_world_m=np.asarray(c.pos).tolist(),fcl_normal_world=np.asarray(c.normal).tolist()))
        center=pose[:3,:3]@self.probe_payload_bounds_center+pose[:3,3]
        return dict(distance_m=max(0.,distance),intersecting=bool(n or distance<0.),contacts=contacts,
            source_asset_sha256=dict(self.probe_source_hashes),actual_banana_pose=pose.tolist(),actual_bowl_pose=bowlpose.tolist(),
            actual_spawned_bounds_center_world_m=center.tolist(),
            center_region=original_region([center],bowlpose,self.probe_low,self.probe_high),
            source_pair_depth_is_not_native_cooked_penetration=True)

    def _destination_measurement(self):
        obstacle=next(x for x in self.objects if x[0]==DESTINATION_MESH)
        pose=self._query_payload_pose
        shape=self.fcl.CollisionObject(self.payload_triangle_model,self.fcl.Transform(pose[:3,:3],pose[:3,3]))
        strict=self._exact_measurement((0.,'payload/banana',DESTINATION_MESH,shape,obstacle[1],.002 if self.phase=='lower' else 0.))
        strict.update(contact_pair='payload/destination',noncontact_required_m=.002,
            representation='Original banana/bowl source triangles; diagnostic lower retains 2 mm')
        if not self._probe_execution_active:return strict
        row=self.probe_row
        if row is None or self.probe_native is None or self.probe_native['physics_step']!=row['physics_step']:
            raise ValueError('Missing aligned native destination data')
        step=int(row['physics_step'])
        if step in self._probe_measurements:return deepcopy(self._probe_measurements[step])
        source=self.source_bowl_observation(pose)
        record=dict(physics_step=step,phase=self.phase,action_index=int(row['action_index']),source=source,
            original_strict_measurement=deepcopy(strict),diagnostic_continuation=False)
        answer=deepcopy(strict)
        if self.phase=='release':
            if self.release_start_step is None or not 1<=step-self.release_start_step<=56:
                raise ValueError('Release exceeds predeclared 56 actual substeps')
            previous_release=[x['physics_step'] for x in self.probe_records if x['phase']=='release']
            expected_step=(previous_release[-1]+1) if previous_release else self.release_start_step+1
            if step!=expected_step:raise ValueError('Bounded release observation has a gap/duplicate')
            force=np.asarray(row['destination_contact_force']).reshape(-1,3).sum(0)
            continuation=bounded_transition_checks(source,self.probe_native,force,self.probe_low,self.probe_high)
            record['bounded_transition']=continuation
            if not continuation['passed']:
                answer.update(valid=False,diagnostic_observation_guard_failed=True)
            if not strict['valid']:
                record['support_continuation']=continuation
                if continuation['passed']:
                    answer.update(valid=True,diagnostic_continuation_only=True,original_strict_valid=False)
                    record['diagnostic_continuation']=True
                self.strict_failure_ledger.append(deepcopy(record))
        elif not strict['valid']:self.strict_failure_ledger.append(deepcopy(record))
        self.probe_records.append(record);self._probe_measurements[step]=deepcopy(answer)
        return answer

    def _measurement(self,pair):
        if pair[1]=='payload/banana' and pair[2]==DESTINATION_MESH and self.phase in ('lower','release'):
            return self._destination_measurement()
        return super()._measurement(pair)

    def valid(self,q,ignore=(),*,mode='planning',gripper_q=None,object_pose=None,obstacle_filter=None):
        self._probe_execution_active=mode=='execution'
        try:
            valid=super().valid(q,ignore,mode=mode,gripper_q=gripper_q,object_pose=object_pose,obstacle_filter=obstacle_filter)
            if not valid:return False
            if self._probe_execution_active and self.phase in ('lower','release'):
                measurement=self._destination_measurement()
                if not measurement['valid']:
                    self.last_clearance=measurement;self.last_collision=(measurement['robot_part'],measurement['obstacle']);return False
            return True
        finally:self._probe_execution_active=False


