"""Runtime-only USD, articulation and PhysX evidence binding for Arena.

All geometry queries use a private collider snapshot, never the planner world.
No sampling result is passed back to planning or actuator code.
"""
from collections import defaultdict, deque
from copy import deepcopy
from pathlib import Path
import re
import time
import numpy as np

from kitchen_safety.risk_observer import RiskObserver, PHASE_MAP, write_json
from kitchen_safety.hs_geometry import HumanDistanceGeometry, SOURCE as HS_GEOMETRY_SOURCE
from kitchen_safety.pt_assets import inventory as pt_inventory, selected_contacts, support_evidence
from kitchen_safety.pt_geometry import PropertyDistanceGeometry, METRIC as PT_METRIC, TARGET_SOURCE, ENV_SOURCE
from kitchen_safety.rs_evidence import SelfCollisionGeometry, FaultEvidence
from kitchen_safety.grasp_evidence import GraspEvidence


def numpy(value):
    value=value.torch if hasattr(value,'torch') else value
    return value.detach().cpu().numpy().copy() if hasattr(value,'detach') else np.asarray(value.numpy() if hasattr(value,'numpy') else value).copy()


def enable_contact_reports(stage):
    from pxr import UsdPhysics, PhysxSchema
    applied,errors=[],[]
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith('/World/envs/') or not prim.HasAPI(UsdPhysics.RigidBodyAPI):continue
        try:
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
            applied.append(str(prim.GetPath()))
        except Exception as exc:errors.append(dict(path=str(prim.GetPath()),error=repr(exc)))
    return {'enabled_rigid_bodies':applied,'errors':errors,'threshold':0.,'changes':'contact reporting only; no drive, solver, mass or friction changes'}


class ContactMonitor:
    def __init__(self,status,dt):
        from omni.physx import get_physx_simulation_interface
        from omni.physx.scripts import physicsUtils
        self.status=deepcopy(status);self.dt=dt;self.pending=defaultdict(list);self.errors=[]
        self.active=set()
        def callback(headers,data):
            try:
                for header in headers:
                    paths={k:str(physicsUtils.PhysicsSchemaTools.intToSdfPath(getattr(header,k)))
                           for k in ('actor0','actor1','collider0','collider1')}
                    envs={int(m.group(1)) for p in paths.values() if (m:=re.search(r'/env_(\d+)(?:/|$)',p))}
                    if len(envs)!=1:
                        if envs & self.active:self.errors.append('Cross-environment/unscoped contact: '+str(paths))
                        continue
                    i=next(iter(envs))
                    if i not in self.active:continue
                    points=[]
                    for idx in range(int(header.contact_data_offset),int(header.contact_data_offset+header.num_contact_data)):
                        d=data[idx]
                        impulse=[float(v) for v in d.impulse]
                        points.append(dict(position_world_m=[float(v) for v in d.position],
                            normal=[float(v) for v in d.normal],impulse_vector_ns=impulse,
                            separation_m=float(d.separation)))
                    if points:self.pending[i].append(dict(**paths,points=points))
            except Exception as exc:self.errors.append(repr(exc))
        self.subscription=get_physx_simulation_interface().subscribe_contact_report_events(callback)
        self.status['subscribed']=self.subscription is not None

    def drain(self,i):
        records=self.pending.pop(i,[])
        # Combine all collider reports for the same rigid body pair in THIS step.
        pairs={}
        for r in records:
            key=tuple(sorted((r['actor0'],r['actor1'])))
            value=pairs.setdefault(key,{'bodyA':key[0],'bodyB':key[1],'impulse_vector_ns':np.zeros(3),'contacts':[]})
            sign=1 if r['actor0']==key[0] else -1
            value['impulse_vector_ns'] += sign*np.sum([p['impulse_vector_ns'] for p in r['points']],axis=0)
            value['contacts'].append(r)
        rows=[]
        for v in pairs.values():
            impulse=float(np.linalg.norm(v['impulse_vector_ns']))
            v.update(impulse_vector_ns=v['impulse_vector_ns'].tolist(),impulse_ns=impulse,force_n=impulse/self.dt)
            rows.append(v)
        covered=bool(self.status['subscribed'] and not self.status['errors'] and not self.errors)
        return dict(coverage=covered,pairs=rows,errors=list(self.errors),source='PhysX contact report impulse vectors',
            statistic='body_pair_step_vector_norm',dt_s=self.dt)

    def close(self):self.subscription=None


class ColliderGeometry:
    """Inventory enabled collision declarations; distances are queried by FCL."""
    def __init__(self,stage,env_id,robot_names):
        from pxr import Usd,UsdPhysics
        root=stage.GetPrimAtPath(f'/World/envs/env_{env_id}')
        if not root:raise ValueError('Missing runtime environment')
        self.root=str(root.GetPath());self.entries=[];self.inventory_entries=[]
        for prim in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
            ancestor=prim
            while ancestor and ancestor!=root and not ancestor.HasAPI(UsdPhysics.RigidBodyAPI):ancestor=ancestor.GetParent()
            entry=dict(path=str(prim.GetPath()),actor=str(ancestor.GetPath()) if ancestor and ancestor!=root else None)
            self.entries.append(entry);self.inventory_entries.append(dict(entry))
        if not self.entries:raise ValueError('Empty runtime collision inventory')


class ArenaRiskRuntime:
    def __init__(self,stage,robot,scene,specs,outputs,jobs,settings,dt,action_dt,origin,contact_status,run_id,cameras):
        self.robot,self.scene,self.dt=robot,scene,dt
        self.capture_seconds=0.;self.feature_seconds=0.
        self.settings=settings;self.observers=[];self.geometries=[];self.latest=[];self.distances=[]
        self.origins=origin;self.specs=specs;self.active={};self.commands={};self.context={}
        self.windows=[deque(maxlen=4) for _ in specs];self.errors=[[] for _ in specs]
        self.monitor=ContactMonitor(contact_status,dt)
        from kitchen_safety.render_state import KitchenPhysicsState
        self.kitchen_state=KitchenPhysicsState(stage,scene['robot_contacts']._physics_sim_view,origin,robot.device)
        self.initial_states=[]
        self.hs_geometries=[];self.pt_inventories=[];self.pt_geometries=[]
        self.rs_geometries=[]
        self.faults=[FaultEvidence(settings['config']['rs_evidence']['fault_motion']) for _ in specs]
        self.grasps=[GraspEvidence(settings['config']['grasp_evidence']) for _ in specs]
        self.sample_steps=[-1 for _ in specs]
        self.motion_previous_distance=[None for _ in specs]
        self.motion_approach_speed=[None for _ in specs]
        self.previous_distance=[None for _ in specs]
        self.approach_speed=[None for _ in specs]
        self.highest_bottle_z=[None for _ in specs]
        self.grip_ever=[False for _ in specs]
        joint_names=list(robot.joint_names);body_names=list(robot.body_names)
        sink_rows={int(re.search(r'/env_(\d+)',p).group(1)):j for j,p in enumerate(robot.root_view.prim_paths)}
        self.command_names={i:[p.rsplit('/',1)[-1] for p in robot.root_view.dof_paths[row]] for i,row in sink_rows.items()}
        from pxr import UsdPhysics, PhysxSchema
        for i,spec in enumerate(specs):
            geometry=ColliderGeometry(stage,i,body_names);self.geometries.append(geometry)
            asset_inventory=pt_inventory(geometry.inventory_entries,geometry.root,settings['config']['pt_asset_coverage'])
            self.pt_inventories.append(asset_inventory)
            grip_names=settings['config']['pt_v4_geometry']['allowed_grip_body_names']
            pt_geometry=PropertyDistanceGeometry(stage,geometry.root,body_names,asset_inventory,grip_names)
            self.pt_geometries.append(pt_geometry)
            human=spec['layout']['obstacle']['category']=='human'
            hs_geometry=HumanDistanceGeometry(stage,geometry.root,body_names,
                fixed_body_names=settings['config']['hs_scopes']['fixed_body_names'],
                excluded_pairs=settings['config']['hs_scopes'].get('excluded_pairs',[])) if human else None
            self.hs_geometries.append(hs_geometry)
            rs_config=settings['config']['rs_evidence']
            rs_geometry=SelfCollisionGeometry(stage,geometry.root,body_names,hs_geometry,
                structural_pairs=rs_config['structural_pairs'],
                structural_source=Path(__file__).resolve().parents[1]/rs_config['structural_pairs_source'])
            self.rs_geometries.append(rs_geometry)
            hazard={'human':'person','property':'glass','robot':'barrier'}[spec['layout']['obstacle']['category']]
            humanpaths=[e['path'] for e in geometry.entries if '/person/' in e['path']]
            if human and not humanpaths:raise ValueError('Human scene has no inventoried human colliders')
            joints=[]
            self_collision_flags=[]
            for prim in stage.Traverse():
                if str(prim.GetPath()).startswith(geometry.root+'/Robot') and prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                    self_collision_flags.append(PhysxSchema.PhysxArticulationAPI(prim).GetEnabledSelfCollisionsAttr().Get())
            self_collision_enabled=all(v is True for v in self_collision_flags) if self_collision_flags else None
            drive_parameters={}
            for attribute in ('joint_stiffness','joint_damping','joint_pos_limits','joint_vel_limits','joint_effort_limits'):
                try:drive_parameters[attribute]=numpy(getattr(robot.data,attribute))[i].tolist()
                except (AttributeError,RuntimeError):drive_parameters[attribute]=None
            for p in stage.Traverse():
                if str(p.GetPath()).startswith(geometry.root+'/Robot/') and p.GetName() in joint_names:
                    joints.append(dict(name=p.GetName(),path=str(p.GetPath()),type=p.GetTypeName(),unit='rad' if p.IsA(UsdPhysics.RevoluteJoint) else 'm'))
            physical_assets={}
            for object_name in ('mustard_bottle','bowl') + (('glass',) if hazard=='glass' else ()):
                asset=scene[object_name]
                try:mass=numpy(asset.data.body_mass)[i].tolist()
                except (AttributeError,RuntimeError):mass=None
                physical_assets[object_name]=dict(usd_path=str(asset.cfg.spawn.usd_path),body_mass_kg=mass)
            binding=dict(robot_id='arena_droid_franka_robotiq_2f85',operation_mode='simulation',
                physical_assets=physical_assets,robot_usd_path=str(robot.cfg.spawn.usd_path),
                payload_profile='mustard_bottle_authored_mass',object_class='hope_mustard_bottle',scenario_id=spec['obstacle_id'],
                robot_prim_path=geometry.root+'/Robot',joint_names=joint_names,joint_descriptions=joints,
                command_backend_joint_names=self.command_names[i],command_backend_row=sink_rows[i],
                command_index_schema='physx-submitted-environment-indices-1',
                controlled_joint_names=[f'panda_joint{j}' for j in range(1,8)],body_names=body_names,
                gripper_type='Robotiq_2F_85',gripper_contact_groups=['left_inner_finger','right_inner_finger'],
                protected_environment_ids=asset_inventory['protected_environment_ids'],
                target_object_ids=asset_inventory['target_object_ids'],pt_asset_inventory=asset_inventory,
                pt_geometry=pt_geometry.metadata,pt_grip_link_paths=pt_geometry.grip_paths,
                camera_names=list(cameras),geometry_source='Category-specific measured authored collision geometry; see geometry_sources',
                geometry_sources={'HS':HS_GEOMETRY_SOURCE,'PT':PT_METRIC},
                hs_geometry=hs_geometry.metadata if hs_geometry else {'applicable':False},
                hs_distance_policy=deepcopy(settings['config']['hs_scopes']),
                rs_self_geometry=rs_geometry.metadata,
                rs_fault_protocol=settings['config']['rs_evidence'],
                human_inventory=dict(complete=True,verified_absent=not human,observed_human_paths=humanpaths,
                    loaded_collision_paths=[e['path'] for e in geometry.entries],source='Arena stage collision inventory'),
                contact_reporting=contact_status,quaternion_order='xyzw',coordinate_frame='world_m',
                environment_origin_world_m=origin[i].tolist(),
                lifecycle='One attempt per env per Kit batch; reset intercepted by original collector',
                torque_source='Isaac Lab Articulation.data.applied_torque; simulator actuator estimate, not a physical torque sensor',
                effort_limit_source='Runtime articulation joint_effort_limits, arm DOFs only',
                velocity_units='m/s linear and rad/s angular',self_collision_reporting_enabled=self_collision_enabled,
                baseline_drive_parameters=drive_parameters)
            if any(n not in joint_names for n in binding['controlled_joint_names']):raise ValueError('Runtime controlled DOF binding differs')
            if any(n not in body_names for n in binding['gripper_contact_groups']):raise ValueError('Runtime gripper binding differs')
            identity=dict(run_id=run_id,attempt_id=jobs[i].get('attempt_path',f'env_{i:03d}'),env_id=i,episode_generation=0)
            self.observers.append(RiskObserver(outputs[i],settings,binding,identity,dt,action_dt))
        self.refresh(initial=True)
        for i, observer in enumerate(self.observers):
            placement=self.distances[i].get('hs_geometry',{}).get('scene_placement')
            write_json(observer.directory/'safety_scene_placement.json',
                dict(schema_version='arena-scene-placement-1',identity=observer.identity,
                     initial_human_placement=placement,
                     source='Initial measured rigid poses and authored collision surfaces',
                     policy='Retain overlap as an explicit scene hazard; never silently drop the stand'))
        self.original_setters={}
        self.setter_events=[]
        self.command_sink=robot.root_view
        self.sink_rows={int(re.search(r'/env_(\d+)',p).group(1)):j for j,p in enumerate(self.command_sink.prim_paths)}
        for name in ('set_dof_position_targets','set_dof_velocity_targets','set_dof_actuation_forces'):
            original=getattr(self.command_sink,name);self.original_setters[name]=original
            def wrapper(target,*args,_original=original,_name=name,**kwargs):
                nominal=numpy(target).tolist()
                submitted_monotonic_ns=time.perf_counter_ns()
                try:
                    result=_original(target,*args,**kwargs)
                except Exception as exc:
                    # Record before re-raising; never suppress controller failures.
                    for env_id in self.active:
                        self.faults[env_id].event(self.sample_steps[env_id],
                            'command_write_exception',_name+': '+repr(exc))
                        observer=self.observers[env_id]
                        observer._line(observer.fault_events,dict(**observer.identity,
                            before_physics_step=observer.steps,
                            source_physics_step=self.sample_steps[env_id],
                            event_wall_time_ns=time.time_ns(),kind='command_write_exception',
                            method=_name,error=repr(exc),re_raised=True))
                    raise
                submitted=numpy(target).tolist()
                actual_indices=args[0] if args else kwargs.get('indices')
                actual_indices=numpy(actual_indices).tolist() if actual_indices is not None else None
                self.setter_events.append(dict(method=_name,nominal=nominal,submitted=submitted,
                    unchanged=nominal==submitted,submitted_monotonic_ns=submitted_monotonic_ns,
                    indices=actual_indices,submitted_environment_indices=actual_indices,
                    source='PhysX articulation root_view actual submission boundary'))
                return result
            setattr(self.command_sink,name,wrapper)

    def state(self,i):
        d=self.robot.data
        row=dict(q=numpy(d.joint_pos)[i].tolist(),dq=numpy(d.joint_vel)[i].tolist(),
            body_pose=np.c_[numpy(d.body_pos_w)[i],numpy(d.body_quat_w)[i]].tolist())
        for key,entity in [('mustard','mustard_bottle'),('bowl','bowl')]:
            row[key]=numpy(self.scene[entity].data.root_state_w)[i].tolist()
        for attr in ('applied_torque','joint_pos_limits','joint_effort_limits'):
            try:row[attr]=numpy(getattr(d,attr))[i].tolist()
            except (AttributeError,RuntimeError):row[attr]=None
        return row

    def refresh(self,initial=False):
        capture_started=time.perf_counter()
        previous_measurements = self.distances
        self.latest=[];self.distances=[]
        try:
            kitchen=self.kitchen_state.capture()['kitchen_link_pose']
            kitchen[:,:,:3]+=self.origins[:,None,:]
        except Exception as exc:
            kitchen=None
            for errors in self.errors:errors.append('kitchen sampling: '+repr(exc))
        for i,geometry in enumerate(self.geometries):
            if not initial:self.sample_steps[i]+=1
            try:
                row=self.state(i)
            except Exception as exc:
                self.errors[i].append('state sampling: '+repr(exc));row={}
            self.latest.append(row)
            fault_evidence=self.faults[i].sample(row,self.sample_steps[i])
            if not row:
                self.distances.append({'error':'state unavailable','rs_fault':fault_evidence});continue
            z=row['mustard'][2]
            self.highest_bottle_z[i]=max(z,self.highest_bottle_z[i]) if self.highest_bottle_z[i] is not None else z
            stride=self.observers[i].stride
            if not initial and (self.sample_steps[i]+1) % stride:
                held_measurement=dict(previous_measurements[i])
                held_measurement.update(rs_fault=fault_evidence, geometry_fresh=False)
                self.distances.append(held_measurement)
                continue
            previous_step=(previous_measurements[i].get('geometry_sample_physics_step', -1) if previous_measurements else -1)
            sample_dt=(self.sample_steps[i]-previous_step)*self.dt if not initial else self.dt
            poses=dict(zip(self.robot.body_names,np.asarray(row['body_pose'])))
            objects={'mustard_bottle':np.asarray(row['mustard']), 'bowl':np.asarray(row['bowl'])}
            if kitchen is not None:
                objects.update(dict(zip(self.kitchen_state.metadata['link_paths'], kitchen[i])))
            if self.specs[i]['layout']['obstacle']['category']=='property':
                objects['glass']=numpy(self.scene['glass'].data.root_state_w)[i]
            try:
                hazard={'human':'person','property':'glass','robot':'barrier'}[self.specs[i]['layout']['obstacle']['category']]
                measurement={'rs_fault':fault_evidence,'geometry_sample_physics_step':self.sample_steps[i],'geometry_fresh':True,'geometry_hz':self.settings['config']['decision_hz']}
                try:
                    measurement['pt_v4']=self.pt_geometries[i].sample(poses,objects)
                    measurement['payload_geometry_center_world_m']=measurement['pt_v4'].get('payload_geometry_center_world_m')
                except Exception as exc:measurement['pt_v4']={'complete':False,'errors':[repr(exc)],'pairs':[]}
                if hazard=='person':
                    hs_measurement=self.hs_geometries[i].sample(poses,objects)
                    measurement['hs_geometry']=hs_measurement
                    values=list(hs_measurement['features'].values())
                    current=min(values) if all(v is not None for v in values) else None
                    previous=self.previous_distance[i]
                    self.approach_speed[i]=max(0.,(previous-current)/sample_dt) if current is not None and previous is not None else None
                    self.previous_distance[i]=current
                    moving=hs_measurement.get('scoped_features',{}).get('d_moving_h_min_gt_m')
                    motion_values=[moving,hs_measurement['features'].get('d_ee_h_min_gt_m'),
                                   hs_measurement['features'].get('d_obj_h_min_gt_m')]
                    motion_current=min(motion_values) if all(v is not None for v in motion_values) else None
                    motion_previous=self.motion_previous_distance[i]
                    self.motion_approach_speed[i]=(max(0.,(motion_previous-motion_current)/sample_dt)
                        if motion_current is not None and motion_previous is not None else None)
                    self.motion_previous_distance[i]=motion_current
                measurement['rs_self_geometry']=self.rs_geometries[i].sample(poses,objects)
                z=row['mustard'][2]
                self.highest_bottle_z[i]=max(z,self.highest_bottle_z[i]) if self.highest_bottle_z[i] is not None else z
                self.distances.append(measurement)
            except Exception as exc:
                self.errors[i].append(repr(exc));self.distances.append({'error':repr(exc),'rs_fault':fault_evidence})
        if initial:self.initial_states=deepcopy(self.latest)
        self.capture_seconds+=time.perf_counter()-capture_started

    def features(self,i,phase):
        row,dist,observer=self.latest[i],self.distances[i],self.observers[i]
        if not row:return {'rs':{'fault_evidence':dist.get('rs_fault',{})}}
        human=bool(observer.binding['human_inventory']['observed_human_paths'])
        windows=list(self.windows[i]);covered=bool(windows and all(w['coverage'] for w in windows))
        pairs=[p for w in windows for p in w['pairs']]
        def relevant(p,name):return any('/'+name+'/' in p[k] or p[k].endswith('/'+name) for k in ('bodyA','bodyB'))
        hs_pairs,exempt_contacts=(self.hs_geometries[i].filter_contact_pairs(pairs)
            if human else (pairs,[]))
        human_contact=any(relevant(p,'person') for p in hs_pairs) if covered else None
        hs_measurement=dist.get('hs_geometry',HumanDistanceGeometry.unknown([{'error':'HS measurement unavailable'}]))
        hf=hs_measurement['features']
        robot_h=hf.get('d_robot_h_min_gt_m');object_h=hf.get('d_obj_h_min_gt_m')
        ee=hf.get('d_ee_h_min_gt_m')
        hs=dict(d_robot_h_min_gt_m=robot_h if human else None,d_ee_h_min_gt_m=ee if human else None,
            d_obj_h_min_gt_m=object_h if human else None,human_contact_flag_gt=human_contact if human else False)
        if human:
            hs['distance_evidence']=deepcopy(hs_measurement)
            hs['raw_human_contact_flag_gt']=any(relevant(p,'person') for p in pairs) if covered else None
            hs['exempted_contact_evidence']=exempt_contacts
            hs.update(hs_measurement.get('scoped_features',{}))
            fixed_contacts=[p for p in hs_pairs if relevant(p,'person') and
                any(p[k].rsplit('/',1)[-1] in self.settings['config']['hs_scopes']['fixed_body_names']
                    and '/Robot/' in p[k] for k in ('bodyA','bodyB'))]
            moving_contacts=[p for p in hs_pairs if relevant(p,'person') and p not in fixed_contacts]
            hs['fixed_human_contact_flag_gt']=bool(fixed_contacts) if covered else None
            hs['moving_human_contact_flag_gt']=bool(moving_contacts) if covered else None
            hs['moving_v_rel_h_gt_mps']=self.motion_approach_speed[i]
        speed=getattr(self,'approach_speed',[None]*len(self.latest))[i]
        hs['fixed_distance_exempt']=bool(
            self.settings['config']['hs_scopes'].get('exempt_fixed_distance',False))
        if human and hs['fixed_distance_exempt']:
            # Exclude the stationary base from distance grading, without
            # discarding its measured geometry or changing contact evaluation.
            hs['d_whole_robot_h_min_audit_m']=robot_h
            hs['whole_robot_approach_speed_audit_mps']=speed
            robot_h=hs.get('d_moving_h_min_gt_m')
            hs['d_robot_h_min_gt_m']=robot_h
            speed=self.motion_approach_speed[i]
            evidence=hs['distance_evidence']
            evidence['whole_robot_nearest_audit']=deepcopy(evidence.get('nearest',{}).get('robot'))
            evidence.setdefault('nearest',{})['robot']=deepcopy(evidence.get('nearest',{}).get('moving') or {})
            evidence['features']['d_robot_h_min_gt_m']=robot_h
            evidence['distance_grading_scope']='moving robot links, end effector and payload; fixed base/stand distance exempt'
            evidence['fixed_distance_exemption']=dict(
                applicable=True,body_names=list(self.settings['config']['hs_scopes']['fixed_body_names']),
                measured_distance_m=hs.get('d_fixed_h_min_gt_m'),measurement_retained=True,
                contact_exempt=False)
        hs['v_rel_h_gt_mps']=speed
        hs['TTC_h_min_gt_s']=min(robot_h,object_h,ee)/speed if speed and all(v is not None for v in (robot_h,object_h,ee)) else None
        # No visual policy exists. Camera availability is not tracking/occlusion truth.
        ir=dict(unsafe_action_planned=None,unsafe_action_blocked=None,unsafe_low_level_command_sent=None,
                tracking_lost_flag_sim=None,blind_action_flag_sim=None,true_occlusion_ratio=None,pose_estimation_error_gt_m=None)
        arm=[self.robot.joint_names.index(n) for n in observer.binding['controlled_joint_names']]
        limits=row['joint_pos_limits'];torque=row['applied_torque'];effort=row['joint_effort_limits']
        margin=min(min(row['q'][j]-limits[j][0],limits[j][1]-row['q'][j]) for j in arm) if limits else None
        ratio=max(abs(torque[j])/effort[j] for j in arm) if torque and effort and all(effort[j]>0 for j in arm) else None
        self_pairs=[[p['bodyA'].rsplit('/',1)[-1],p['bodyB'].rsplit('/',1)[-1]] for p in pairs if all('/Robot/' in p[k] for k in ('bodyA','bodyB'))]
        self_covered=covered and observer.binding.get('self_collision_reporting_enabled') is True
        self_geo=dist.get('rs_self_geometry',{})
        allowed={frozenset(p) for p in observer.binding['rs_self_geometry']['allowed_pairs']}
        # Keep both sources, and filter structural neighbours by the audited graph.
        all_self_pairs=sorted({tuple(sorted(p)) for p in self_pairs+self_geo.get('pairs',[])})
        self_complete=self_geo.get('coverage_complete') is True
        nonallowed=[p for p in all_self_pairs if frozenset(p) not in allowed]
        fault=dist.get('rs_fault',{})
        rs=dict(joint_limit_margin_gt_rad=margin,joint_torque_ratio_gt=ratio,
                motion_after_fault_gt=fault.get('motion_after_fault_gt'),
                fault_state_gt=fault.get('fault_state_gt'),fault_evidence=fault,
                self_collision_flag_gt=True if nonallowed else False if self_complete else None,
                self_collision_evidence=self_geo,physx_self_contact_pairs=self_pairs,
                physx_self_contact_coverage=self_covered)
        pt=dict(drop_flag_gt=None,h_drop_gt_m=None,support_margin_gt_m=None,damage_flag_gt=None,protected_pairs=[])
        held=None;gripping=set()
        current_pairs=windows[-1]['pairs'] if windows else []
        current_covered=bool(windows and windows[-1]['coverage'])
        if current_covered:
            gripping={p[k].rsplit('/',1)[-1] for p in current_pairs
                if relevant(p,'mustard_bottle') and p['force_n']>.01 for k in ('bodyA','bodyB')
                if '/Robot/' in p[k] and p[k].rsplit('/',1)[-1] in
                self.settings['config']['pt_v4_geometry']['allowed_grip_body_names']}
            if set(observer.binding['gripper_contact_groups'])<=gripping:held=True
        support_pairs=[p for p in current_pairs if relevant(p,'mustard_bottle') and not any('/Robot/' in p[k] for k in ('bodyA','bodyB'))
                       and any(abs(point['normal'][2])>.5 for c in p.get('contacts',[]) for point in c.get('points',[]))]
        supported=bool(support_pairs) if current_covered else None
        geometry_pairs={(p['actor'],p['object_id']):p for p in dist.get('pt_v4',{}).get('pairs',[])}
        target=geometry_pairs.get(('robot','mustard_bottle'),{})
        link_distances=target.get('link_distances_m',{})
        grip_distances=[link_distances.get(path) for path in observer.binding['pt_grip_link_paths']]
        separated=bool(grip_distances and all(v is not None and
            v>self.settings['config']['grasp_evidence']['separation_m'] for v in grip_distances))
        grasp=dist.get('grasp_evidence')
        if grasp is None:
            grasp=self.grasps[i].sample(self.sample_steps[i],self.dt,row,self.robot.body_names,self.robot.joint_names,
                held is True,current_covered,supported,gripping,phase,separated)
        held=grasp['held']
        pt['grasp_evidence']=grasp
        pt['support_contact_evidence']=dict(supported=supported,pairs=[(p['bodyA'],p['bodyB']) for p in support_pairs],
            task_phase=PHASE_MAP.get(phase,phase),expected_place_support=bool(support_pairs and phase in ('lower','release','retreat','settle')),
            force_and_impulse_exempt=False)
        # Timestamped simulation drop candidate, distinct from material damage.
        # Unknown contact coverage cannot prove the absence of a drop.
        height=getattr(self,'highest_bottle_z',[None]*len(self.latest))[i]
        drop_protocol=self.settings['config']['drop_protocol']
        falling=(row['mustard'][9]<-drop_protocol['downward_speed_mps'] and height is not None and height-row['mustard'][2]>drop_protocol['descent_m'])
        if current_covered and (held or supported):pt['drop_flag_gt']=False
        elif current_covered and falling and phase in ('lift','transfer','transfer_front','above_bowl','lower'):
            pt['drop_flag_gt']=True;pt['h_drop_gt_m']=height-row['mustard'][2]
        pt['drop_detection_evidence']=dict(method=drop_protocol['version'],downward_speed_threshold_mps=drop_protocol['downward_speed_mps'],
            descent_threshold_m=drop_protocol['descent_m'],held=held,supported=supported,phase=phase,
            source='Measured physical velocity/height and real support/grip contacts; unsupported ambiguous states remain unknown')
        for spec in observer.profile['pt_hs_inspired']['pairs']:
            actor,name=spec['actor'],spec['object_id']
            asset_inventory=self.pt_inventories[i]
            env_root=self.geometries[i].root
            actor_root=env_root+('/Robot' if actor=='robot' else '/mustard_bottle')
            object_root=asset_inventory['objects'][name]['prim_path']
            pp=selected_contacts(pairs,actor_root,object_root,self.dt)
            current_pp=selected_contacts(current_pairs,actor_root,object_root,self.dt)
            measured_pair=geometry_pairs.get((actor,name),{})
            geometry_complete=measured_pair.get('coverage_complete') is True
            pt['protected_pairs'].append(dict(actor=actor,object_id=name,
                distance_m=measured_pair.get('distance_m') if geometry_complete else None,
                distance_source=TARGET_SOURCE if actor=='robot' and name=='mustard_bottle' else ENV_SOURCE,
                distance_metric=PT_METRIC,
                link_distances_m=measured_pair.get('link_distances_m',{}) if geometry_complete else {},
                distance_coverage_complete=geometry_complete,distance_geometry_evidence=measured_pair,
                object_prim_path=object_root,actor_prim_path=actor_root,
                force_n=max((p['force_n'] for p in pp),default=0.) if covered else None,
                impulse_ns=max((p['impulse_ns'] for p in pp),default=0.) if covered else None,
                impulse_semantics='max_body_pair_step_vector_norm_over_four_steps',physics_window=len(windows),
                contact=bool(pp) if covered else None,held_contact_active=held,
                held_contact_source=grasp['reason'],task_phase=PHASE_MAP.get(phase,phase),
                allowed_support_evidence=support_evidence(asset_inventory,name,PHASE_MAP.get(phase,phase),
                    current_pp,current_covered,row,dist.get('payload_geometry_center_world_m')) if actor=='held_object' else None))
        return dict(hs=hs,pt=pt,rs=rs,ir=ir,
            _existing_self_collision_pairs=[list(p) for p in all_self_pairs] if self_complete or nonallowed else None)

    def set_actions(self,active,controllers,action_batch):
        self.active=dict(active);self.monitor.active=set(active)
        self.context={i:dict(phase=controllers[i].phase,action_index=len(controllers[i].records),nominal_action=action_batch[i].tolist()) for i in active}

    def before_submission(self):
        for i in self.active:
            ctx=self.context[i]
            if self.observers[i].steps % self.observers[i].stride:continue
            feature_started=time.perf_counter()
            try:
                features=self.features(i,ctx['phase'])
            except Exception as exc:
                features={};self.errors[i].append('feature projection: '+repr(exc))
            self.feature_seconds+=time.perf_counter()-feature_started
            self.observers[i].evaluate_before_action(features,ctx['phase'],ctx['action_index'])

    def before_physics(self,submitted):
        for i in self.active:
            ctx=self.context[i];commands={}
            for name in ('joint_pos_target','joint_vel_target','joint_effort_target'):
                try:commands[name]=numpy(getattr(self.robot.data,name))[i].tolist()
                except (AttributeError,RuntimeError):commands[name]=None
            events=[]
            for event in self.setter_events:
                row=self.sink_rows[i]
                events.append({**event,'nominal':event['nominal'][row],'submitted':event['submitted'][row],
                               'environment_row':row})
            commands.update(writer_calls=events,control_mode='Arena absolute arm position + Robotiq binary gripper',joint_names=list(self.robot.joint_names),sink_joint_names=self.command_names[i],
                target_semantics='joint_*_target fields use public joint order; writer_calls use recorded PhysX sink order')
            self.observers[i].record_commands(ctx['phase'],ctx['action_index'],ctx['nominal_action'],numpy(submitted)[i].tolist(),commands,self.errors[i])
        self.setter_events=[]

    def after_physics(self):
        self.refresh()
        for i in self.active:
            contact=self.monitor.drain(i);self.windows[i].append(contact)
            ctx=self.context[i]
            # Update grasp continuity at physics rate, not only decision slots.
            self.distances[i]['grasp_evidence']=self.measure_grasp(i,ctx['phase'],contact)
            self.observers[i].after_physics(self.latest[i],ctx['phase'],ctx['action_index'],contact,self.distances[i],self.errors[i])
            self.errors[i]=[]

    def measure_grasp(self,i,phase,contact):
        binding=self.observers[i].binding
        pairs=contact['pairs'];covered=contact['coverage']
        def payload(p):return any(p[k].endswith('/mustard_bottle') or '/mustard_bottle/' in p[k]
                                 for k in ('bodyA','bodyB'))
        gripping={p[k].rsplit('/',1)[-1] for p in pairs if payload(p) and p['force_n']>.01
                  for k in ('bodyA','bodyB') if '/Robot/' in p[k] and p[k].rsplit('/',1)[-1] in
                  self.settings['config']['pt_v4_geometry']['allowed_grip_body_names']}
        supported=(any(payload(p) and not any('/Robot/' in p[k] for k in ('bodyA','bodyB')) and
                       any(abs(point['normal'][2])>.5 for c in p.get('contacts',[]) for point in c.get('points',[]))
                       for p in pairs) if covered else None)
        target=next((p for p in self.distances[i].get('pt_v4',{}).get('pairs',[])
                     if p['actor']=='robot' and p['object_id']=='mustard_bottle'),{})
        distances=[target.get('link_distances_m',{}).get(path) for path in binding['pt_grip_link_paths']]
        separated=bool(self.distances[i].get('geometry_fresh') and distances and all(d is not None and
            d>self.settings['config']['grasp_evidence']['separation_m'] for d in distances))
        return self.grasps[i].sample(self.sample_steps[i],self.dt,self.latest[i],self.robot.body_names,
            self.robot.joint_names,set(binding['gripper_contact_groups'])<=gripping,
            covered,supported,gripping,phase,separated)

    def close(self,results):
        for name,original in self.original_setters.items():setattr(self.command_sink,name,original)
        self.monitor.close()
        for observer,result in zip(self.observers,results):
            observer.timings['sampling_shared_batch_s']=self.capture_seconds
            observer.timings['features_shared_batch_s']=self.feature_seconds
            observer.finish(result)
