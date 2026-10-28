"""Full-step hazard evidence layered on the unchanged original bowl recorder.

No stepping or physical setters. Static poses are measured composed USD poses,
never placeholder rigid states; dynamic poses/velocities are native body reads.
Each actual robot body plus source bowl has an exact hazard contact filter.
"""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation

from .bowl_recorder import BowlPhysicsRecorder
from .bowl_safety_policy import ROOT, require, sha, verify_asset_inputs
from .startup_recorder import array

CAPACITY = 4096


def paths(value):
    if isinstance(value,str):return [value]
    return [p for item in value for p in paths(item)]


def resolved_pair(view,sensor,filtered):
    sensed,filters=paths(view.sensor_paths),paths(view.filter_paths)
    require(view.sensor_count==view.filter_count==1 and sensed==[sensor] and filters==[filtered],
        'Native hazard filter/sensor did not resolve to the exact physical pair')
    return dict(sensor_paths=sensed,filter_paths=filters,sensor_count=int(view.sensor_count),filter_count=int(view.filter_count))


def robot_contact_paths(robot):
    """Use actual articulation links; a rigid-body wildcard can omit those links."""
    native = robot.root_view
    require(native.count == 1 and len(native.link_paths) == 1,
        'Exactly one native robot articulation is required')
    links = list(native.link_paths[0])
    names = list(native.shared_metatype.link_names)
    require(len(links) == len(set(links)) == len(names) == len(robot.body_names)
        and all(p.startswith('/World/envs/env_0/Robot/') for p in links)
        and [p.rsplit('/', 1)[-1] for p in links] == names
        and set(names) == set(robot.body_names),
        'Native articulation link inventory differs from recorded robot bodies')
    return links


def audit_hazard(data, metadata, rows):
    """Independent full coverage/motion/contact gate, including reset prefix.

The original local property check is <2 mm, <=2 degrees including vertical and
upright displacement (audit_avoidance.py). This new candidate predeclares the
root-requested stricter 1-degree rotation/upright gate; it is not source-YAML
semantics and is never fitted to observations.
"""
    steps = np.asarray(data['physics_step']); poses = np.asarray(data['hazard_pose_w'])
    velocities = np.asarray(data['hazard_velocity_w']); local = np.asarray(data['hazard_colliders_local_transforms'])
    count = len(steps); sensors = metadata.get('contact_pair_ids',metadata['native_sensor_paths'])
    finite = all(np.isfinite(x).all() for x in (poses,velocities,local,data['hazard_sensor_normal_forces_w']))
    checks = dict(continuous_reset=bool(count and steps[0]==0 and np.all(np.diff(steps)==1)),
        complete_actual_hazard_state=finite and poses.shape==(count,7) and velocities.shape==(count,6)
            and bool(np.all(data['hazard_observation_valid'])),
        native_rows_all_actual=[r['physics_step'] for r in rows]==steps.tolist(),
        immutable_collider_shape=bool(finite and local.shape==(count,len(metadata['collider_paths']),4,4)
            and np.max(np.abs(local-local[0]))<=1e-6),
        no_animated_hazard=not metadata['animated_transform_paths'],
        original_asset_inputs_unchanged=verify_asset_inputs(metadata['policy']))
    required_filters=metadata['collider_paths'] if metadata['static'] else [metadata['hazard_root']]
    expected_pairs=[(s,f) for s in metadata['native_sensor_paths'] for f in required_filters]
    resolved=metadata.get('native_resolved_contact_pairs',[])
    observed_pairs=[(r['sensor_paths'][0],r['filter_paths'][0]) for r in resolved
        if len(r.get('sensor_paths',[]))==len(r.get('filter_paths',[]))==1 and
        r.get('sensor_count')==r.get('filter_count')==1]
    checks['native_resolved_all_physical_pairs']=bool(observed_pairs==expected_pairs and
        metadata.get('exact_filter_paths')==required_filters and
        [r['pair_id'] for r in resolved]==sensors and len(resolved)==len(expected_pairs))
    failures=[]; max_scalar=0.
    if checks['native_rows_all_actual']:
        for i,row in enumerate(rows):
            if (row['phase']!=str(data['phase'][i]) or row['action_index']!=int(data['action_index'][i]) or
                row['exact_hazard']!=metadata['hazard_root'] or list(row['sensors'])!=sensors):
                failures.append(dict(physics_step=int(steps[i]),reason='sensor/step identity'))
                continue
            for j,name in enumerate(sensors):
                raw=row['sensors'][name]; n=raw['count']
                fields=[np.asarray(raw[key]) for key in ('points_world_m','normals_world','normal_forces_n','separations_m')]
                valid=all(len(x)==n and np.isfinite(x).all() for x in fields)
                counts,starts=np.asarray(raw['buffer_count']),np.asarray(raw['buffer_start'])
                valid=bool(valid and counts.size==starts.size==1 and int(counts.flat[0])==n and
                    int(starts.flat[0])>=0 and int(starts.flat[0])+n<CAPACITY and raw['buffer_capacity']==CAPACITY)
                if valid:
                    norm=fields[1].reshape(-1,3);force=fields[2]
                    summed=(force[:,None]*norm).sum(0)
                    valid=bool(np.array_equal(summed,raw['summed_normal_force_n']) and
                        np.array_equal(summed,data['hazard_sensor_normal_forces_w'][i,j]) and
                        n==data['hazard_sensor_contact_counts'][i,j])
                    if n:max_scalar=max(max_scalar,float(np.max(np.abs(force))))
                if not valid:failures.append(dict(physics_step=int(steps[i]),sensor=name,reason='incomplete/mismatched raw contact'))
    checks['complete_raw_hazard_contacts']=checks['native_rows_all_actual'] and not failures
    checks['no_loaded_robot_or_payload_hazard_contact']=max_scalar==0. and checks['complete_raw_hazard_contacts']
    motion=None
    if checks['complete_actual_hazard_state']:
        translation=np.linalg.norm(poses[:,:3]-poses[0,:3],axis=1)
        angles=np.rad2deg((Rotation.from_quat(poses[:,3:])*Rotation.from_quat(poses[0,3:]).inv()).magnitude())
        vertical=np.abs(poses[:,2]-poses[0,2]); upright=np.rad2deg(np.arccos(np.clip(Rotation.from_quat(poses[:,3:]).as_matrix()[:,2,2],-1.,1.)))
        motion=dict(max_translation_m=float(translation.max()),max_rotation_deg=float(angles.max()),
            max_vertical_m=float(vertical.max()),max_upright_deg=float(upright.max()))
        if metadata['static']:
            obstacle=metadata['obstacle']; authored=np.r_[obstacle['position'],Rotation.from_euler('z',obstacle.get('yaw_deg',0),degrees=True).as_quat()]
            position_error=float(np.linalg.norm(poses[0,:3]-authored[:3]));angle_error=float(np.rad2deg((Rotation.from_quat(poses[0,3:])*Rotation.from_quat(authored[3:]).inv()).magnitude()))
            checks['static_native_identity']=not metadata['enabled_rigid_body_paths'] and not bool(np.any(data['hazard_is_dynamic']))
            checks['static_all_measured_transforms_fixed']=bool(translation.max()<1e-6 and angles.max()<.001 and position_error<1e-6 and angle_error<.001)
            motion.update(reset_authored_position_error_m=position_error,reset_authored_angle_error_deg=angle_error)
        else:
            checks['dynamic_native_identity']=metadata['enabled_rigid_body_paths']==[metadata['hazard_root']] and bool(np.all(data['hazard_is_dynamic']))
            checks['dynamic_property_stationary_from_reset']=bool(translation.max()<.002 and vertical.max()<.002 and angles.max()<=1. and upright.max()<=1.)
    else:
        checks['motion_gate']=False
    return dict(passed=all(checks.values()),checks=checks,physics_samples=count,contact_samples=len(rows),
        failures=failures,motion=motion,max_absolute_contact_scalar_n=max_scalar,
        thresholds=dict(dynamic_translation_m=.002,dynamic_vertical_m=.002,dynamic_rotation_deg=1.,dynamic_upright_deg=1.,
            static_translation_m=1e-6,static_rotation_deg=.001),
        scope='Reset through final physical sample, actual pose/velocity and exact original robot/payload to hazard native contacts. No source collision or causal avoidance claim.')


def unpack_contact(buffers):
    f, p, n, separation, count, start = map(array, buffers)
    require(count.size == start.size == 1, 'Exact single sensor/filter pair required')
    size, first = int(count.flat[0]), int(start.flat[0])
    require(size >= 0 and first >= 0 and first+size < CAPACITY and
        first+size <= min(len(p), len(n), f.size, separation.size), 'Incomplete/full hazard contact buffer')
    s = slice(first, first+size)
    points, normals = p[s].reshape(-1,3), n[s].reshape(-1,3)
    forces, gaps = f.reshape(-1)[s], separation.reshape(-1)[s]
    require(all(np.isfinite(v).all() for v in (points,normals,forces,gaps)), 'Nonfinite hazard contact')
    return dict(count=size, buffer_count=count.tolist(), buffer_start=start.tolist(), buffer_capacity=CAPACITY,
        points_world_m=points.tolist(), normals_world=normals.tolist(), normal_forces_n=forces.tolist(),
        separations_m=gaps.tolist(), summed_normal_force_n=(forces[:,None]*normals).sum(0).tolist())


class HazardObserver:
    def __init__(self, env, envdef):
        from pxr import Usd, UsdGeom, UsdPhysics
        self.e, self.policy = env, envdef.safety_policy
        verify_asset_inputs(self.policy)
        self.obstacle = envdef.resolved_task_layout['obstacle']
        self.root = '/World/envs/env_0/'+self.policy['hazard_name']
        self.stage, self.static, self.dt = env.sim.stage, self.obstacle['static'], float(env.physics_dt)
        root = self.stage.GetPrimAtPath(self.root)
        require(root.IsValid(), 'Actual hazard root missing')
        self.colliders, rigid, animated, layer_files = [], [], [], set()
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                self.colliders.append(str(prim.GetPath()))
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get():
                rigid.append(str(prim.GetPath()))
            animated += [str(a.GetPath()) for a in prim.GetAttributes()
                         if a.GetName().startswith('xformOp:') and a.GetNumTimeSamples()]
            for spec in prim.GetPrimStack():
                if spec.layer.realPath:
                    layer_files.add(Path(spec.layer.realPath).resolve())
        require(self.colliders and not animated, 'Missing colliders or unsupported animated hazard')
        if self.static:
            require(not rigid, 'Declared static hazard has enabled rigid dynamics')
        else:
            require(rigid == [self.root], 'Dynamic hazard must be exactly one original rigid body')
        captured = {str((ROOT/r['path']).resolve()): r['sha256'] for r in self.policy['hazard_asset_files']}
        dependencies = []
        for path in sorted(layer_files):
            require(str(path) in captured and sha(path) == captured[str(path)], 'Uncaptured composed hazard dependency: '+str(path))
            dependencies.append(dict(path=str(path.relative_to(ROOT)), sha256=sha(path)))
        require(str((ROOT/self.obstacle['usd_path']).resolve()) in {str(x) for x in layer_files}, 'Actual composed hazard missing its source layer')
        view = env.scene['robot_contacts']._physics_sim_view
        self.body_view = None if self.static else view.create_rigid_body_view(self.root)
        if self.body_view is not None:
            require(self.body_view.count == 1 and list(self.body_view.prim_paths) == [self.root], 'Wrong dynamic hazard native body')
        robot_paths = robot_contact_paths(env.scene['robot'])
        for path in robot_paths:
            prim = self.stage.GetPrimAtPath(path)
            require(prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI)
                and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get(),
                'Native robot contact link lacks its enabled source rigid body: '+path)
        self.sensors = robot_paths+['/World/envs/env_0/bowl']
        self.filters=self.colliders if self.static else [self.root]
        self.views = {}; native_pairs=[]
        for path in self.sensors:
            for filtered in self.filters:
                contact = view.create_rigid_contact_view([path], filter_patterns=[[filtered]], max_contact_data_count=CAPACITY)
                resolved=resolved_pair(contact,path,filtered)
                key=path+' -> '+filtered
                native_pairs.append(dict(pair_id=key,**resolved,
                    sensor_object_type=str(view.get_object_type(path)),filter_object_type=str(view.get_object_type(filtered)),
                    filter_role='actual static CollisionAPI collider' if self.static else 'original dynamic RigidBody'))
                self.views[key] = contact
        self.pairs=list(self.views)
        self.rows, self.previous_pose, self.initial_local = [], None, None
        self.metadata = dict(schema_version=1, hazard_root=self.root, static=self.static, collider_paths=self.colliders,
            obstacle=self.obstacle,
            enabled_rigid_body_paths=rigid, animated_transform_paths=animated, native_sensor_paths=self.sensors,
            robot_sensor_inventory_source='Actual ArticulationView.link_paths, unique per-link names matched to recorded robot.body_names and enabled USD RigidBodyAPI; contact sensor_paths verified separately',
            exact_filter_paths=self.filters, native_resolved_contact_pairs=native_pairs,contact_pair_ids=self.pairs,
            native_static_filter_scope='All actual enabled CollisionAPI children separately resolved by backend filter_paths; no nonexistent Xform-root filter. ObjectType enum does not document a static-collider member.',
            actual_composed_asset_layers=dependencies,
            policy=self.policy, physics_dt_s=self.dt,
            pose_semantics='Measured composed USD root and child-relative matrices' if self.static else 'Native rigid-body get_transforms xyz+xyzw and get_velocities linear+angular',
            scope='Every reset/physical substep. Original source bowl/task objects unchanged; contact scalar*normal sign preserved.')

    def sample(self, step, phase, action_index):
        from pxr import UsdGeom
        cache = UsdGeom.XformCache()
        matrix = np.asarray(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(self.root))).T
        if self.static:
            pose = np.r_[matrix[:3,3], Rotation.from_matrix(matrix[:3,:3]).as_quat()]
            velocity = np.zeros(6)
            if self.previous_pose is not None:
                velocity[:3] = (pose[:3]-self.previous_pose[:3])/self.dt
                velocity[3:] = (Rotation.from_quat(pose[3:])*Rotation.from_quat(self.previous_pose[3:]).inv()).as_rotvec()/self.dt
        else:
            pose = array(self.body_view.get_transforms())[0]
            velocity = array(self.body_view.get_velocities())[0]
        local = np.array([np.linalg.inv(matrix) @ np.asarray(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(p))).T
                          for p in self.colliders])
        require(pose.shape == (7,) and velocity.shape == (6,) and
            all(np.isfinite(v).all() for v in (pose,velocity,local)), 'Missing/invalid actual hazard state')
        if self.initial_local is None:
            self.initial_local = local.copy()
        require(np.max(np.abs(local-self.initial_local)) <= 1e-6, 'Hazard collider shape changed relative to root')
        contact = {path: unpack_contact(view.get_contact_data(dt=self.dt)) for path,view in self.views.items()}
        value = dict(physics_step=int(step), phase=phase, action_index=int(action_index),
            exact_hazard=self.root, sensors=contact)
        require(not self.rows or step == self.rows[-1]['physics_step']+1, 'Missing hazard physical substep')
        self.rows.append(value); self.previous_pose=pose.copy()
        return dict(hazard_pose_w=pose.copy(), hazard_velocity_w=velocity.copy(),
            hazard_colliders_local_transforms=local, hazard_is_dynamic=not self.static,
            hazard_observation_valid=True,
            hazard_sensor_normal_forces_w=np.array([contact[p]['summed_normal_force_n'] for p in self.pairs]),
            hazard_sensor_contact_counts=np.array([contact[p]['count'] for p in self.pairs], dtype=np.int32))


class BowlSafetyPhysicsRecorder(BowlPhysicsRecorder):
    def __init__(self, env, envdef, contacts):
        self.hazard_observer = HazardObserver(env, envdef)
        super().__init__(env, envdef, contacts)
        self.metadata['hazard_evidence'] = self.hazard_observer.metadata

    def capture(self):
        row = super().capture()
        # Explicit failed placeholders preserve a genuinely observed original
        # physical row if a new sensor read fails. They can never pass audits.
        observer = self.hazard_observer
        row.update(hazard_pose_w=np.full(7,np.nan), hazard_velocity_w=np.full(6,np.nan),
            hazard_colliders_local_transforms=np.full((len(observer.colliders),4,4),np.nan),
            hazard_is_dynamic=not observer.static, hazard_observation_valid=False,
            hazard_sensor_normal_forces_w=np.full((len(observer.pairs),3),np.nan),
            hazard_sensor_contact_counts=np.full(len(observer.pairs),-1,np.int32))
        row.update(observer.sample(row['physics_step'], self.phase, self.action_index))
        return row

    def write(self, directory, actions):
        directory=Path(directory)
        # Archive raw native contacts even when a preceding sample failed.
        (directory/'hazard_contacts.json').write_text(json.dumps(dict(metadata=self.hazard_observer.metadata,
            rows=self.hazard_observer.rows), allow_nan=False)+'\n')
        result=super().write(directory,actions)
        arrays={key:np.asarray([row[key] for row in self.rows]) for key in self.rows[0]}
        result['hazard_recording'] = audit_hazard(arrays,self.hazard_observer.metadata,self.hazard_observer.rows)
        (directory/'hazard_audit.json').write_text(json.dumps(result['hazard_recording'],allow_nan=False,indent=2)+'\n')
        result['passed'] = result['passed'] and result['hazard_recording']['passed']
        return result
