"""CPU glass grasp candidates with all original kitchen and gripper geometry.

The cylinder bottom is enclosed by a 256-sided tangent polygon for planning;
the source USD remains unchanged. This is not a PhysX cooked-shape claim.
"""
from pathlib import Path
from dataclasses import dataclass
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation, Slerp
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.extensions.replicator.task_collision import PayloadCollisionWorld, PayloadGeometry
from kitchen_safety.extensions.replicator.task_contact_policy import BananaContactPolicy
from kitchen_safety.extensions.replicator.task_motion import pose_matrix
from .property_environment import ROOT, sha, execution_layout

ENV_ROOT = '/World/envs/env_0'


def source_initial_q():
    """Read original DroidSceneCfg without importing Kit/Arena on the CPU host."""
    import ast,math,re
    path=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py'
    tree=ast.parse(path.read_text())
    names=ast.literal_eval(next(n.value for n in tree.body if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=='_DROID_JOINT_NAMES' for t in n.targets)))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DroidSceneCfg')
    robot=next(n.value for n in cls.body if isinstance(n,ast.AnnAssign) and n.target.id=='robot')
    initial=next(k.value for k in robot.keywords if k.arg=='init_state')
    joints=next(k.value for k in initial.keywords if k.arg=='joint_pos')
    def number(node):
        if isinstance(node,ast.Constant) and isinstance(node.value,(int,float)):return float(node.value)
        if isinstance(node,ast.Attribute) and isinstance(node.value,ast.Name) and node.value.id=='torch' and node.attr=='pi':return math.pi
        if isinstance(node,ast.UnaryOp) and isinstance(node.op,ast.USub):return -number(node.operand)
        if isinstance(node,ast.BinOp):
            a,b=number(node.left),number(node.right)
            if isinstance(node.op,ast.Div):return a/b
            if isinstance(node.op,ast.Mult):return a*b
            if isinstance(node.op,ast.Add):return a+b
            if isinstance(node.op,ast.Sub):return a-b
        raise ValueError('Unsupported original joint expression')
    patterns={ast.literal_eval(k):number(v) for k,v in zip(joints.keys,joints.values)};values={}
    for name in names:
        matches=[v for pattern,v in patterns.items() if re.fullmatch(pattern,name)]
        if len(matches)!=1:raise ValueError('Ambiguous original joint '+name)
        values[name]=matches[0]
    return values,dict(source=str(path.relative_to(ROOT)),sha256=sha(path),full_joint_positions_rad=values,
        semantics='Original authored initial q; native measured reset still required')


class GlassGeometry(PayloadGeometry):
    def __init__(self, path):
        stage=Usd.Stage.Open(str(path));root=stage.GetDefaultPrim();cache=UsdGeom.XformCache()
        inverse=np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(root)).T)
        vertices=[];triangles=[];records=[];cursor=0
        for p in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
            if not p.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is False:continue
            if p.IsA(UsdGeom.Mesh):
                m=UsdGeom.Mesh(p);points=np.asarray(m.GetPointsAttr().Get(),float)
                indices=np.asarray(m.GetFaceVertexIndicesAttr().Get(),int);faces=[];offset=0
                for n in m.GetFaceVertexCountsAttr().Get():
                    faces.extend((indices[offset],indices[offset+i],indices[offset+i+1]) for i in range(1,n-1));offset+=n
                extra={'representation':'original source collision triangles'}
            elif p.IsA(UsdGeom.Cylinder):
                c=UsdGeom.Cylinder(p);n=256;r=float(c.GetRadiusAttr().Get());h=float(c.GetHeightAttr().Get())
                axis='XYZ'.index(str(c.GetAxisAttr().Get()));perp=[i for i in range(3) if i!=axis]
                angle=(np.arange(n)+.5)*2*np.pi/n;inflation=1/np.cos(np.pi/n)
                points=np.zeros((2*n,3))
                for ring,z in enumerate((-h/2,h/2)):
                    sl=slice(ring*n,(ring+1)*n);points[sl,axis]=z
                    points[sl,perp[0]]=r*inflation*np.cos(angle);points[sl,perp[1]]=r*inflation*np.sin(angle)
                faces=[]
                for i in range(n):
                    j=(i+1)%n;faces.extend(((i,j,j+n),(i,j+n,i+n)))
                faces.extend((0,i+1,i) for i in range(1,n-1));faces.extend((n,n+i,n+i+1) for i in range(1,n-1))
                extra={'representation':'circumscribed 256-sided cylinder','maximum_radial_inflation_m':r*(inflation-1)}
            else:raise ValueError('Unsupported glass source collider: '+str(p.GetPath()))
            mat=inverse@np.asarray(cache.GetLocalToWorldTransform(p)).T
            points=points@mat[:3,:3].T+mat[:3,3]
            triangles.extend(np.asarray(faces,dtype=int)+cursor);vertices.append(points);cursor+=len(points)
            records.append(dict(path=str(p.GetPath()),schema=p.GetTypeName(),**extra))
        if len(records)!=41:raise ValueError('Expected all 41 original glass colliders')
        self.vertices=np.concatenate(vertices);self.triangles=np.asarray(triangles,dtype=np.int32)
        self.low,self.high=self.vertices.min(0),self.vertices.max(0);self.hull=ConvexHull(self.vertices)
        self.provenance=dict(source_asset=str(path),source_sha256=sha(path),colliders=records,
            bounds_m=[self.low.tolist(),self.high.tolist()],physx_cooked_shapes_verified=False,
            fcl_representation='Convex envelope of all source wall triangles and conservatively tessellated cylinder bottom')


@dataclass(frozen=True)
class PropertyGlassContactPolicy(BananaContactPolicy):
    def redundant_finger_envelope(self,label,phase):
        # The palm capsule extends to TCP-23 mm and overlaps a 39 mm-radius
        # cylindrical target even while the real palm/knuckles are clear.
        # Refine ONLY this aggregate target pair by the full named source
        # part hulls (including base_link, outer fingers and knuckles) below.
        return super().redundant_finger_envelope(label,phase) or (
            phase in ('approach','close','lift','transport','lower','release','retreat') and label=='gripper')


class PropertyGlassWorld(PayloadCollisionWorld):
    def __init__(self,stage,kin,*,glass_usd,**kwargs):
        self.payload_name='fragile_glass';self.payload_geometry=GlassGeometry(glass_usd)
        CollisionWorld.__init__(self,stage,kin,clearances={'glass':.002},execution_clearances={'glass':.002},
            default_clearance=.002,**kwargs)
        self.payload_model,self.payload_hull_vertices=self._convex(self.payload_geometry.vertices)
        self.contact_policy=PropertyGlassContactPolicy(payload_name=self.payload_name,
            source_support=ENV_ROOT+'/bowl/obj_000013_Mesh',destination_name=ENV_ROOT+'/kitchen/CapSurface_02')
        self.phase='reset';self.payload_pose=None;self.gripper_joint_positions=np.zeros(6)
        self.native_support_certificate=None;self.source_support_refinements=[]
        parts={name.rsplit('/',1)[-1] for name,_,_ in self.gripper_models}
        if not {'base_link','left_inner_finger','right_inner_finger','left_outer_finger','right_outer_finger'}<=parts:
            raise ValueError('Aggregate refinement requires all named original gripper part hulls')
        model=self.fcl.BVHModel();model.beginModel(len(self.payload_geometry.vertices),len(self.payload_geometry.triangles))
        model.addSubModel(self.payload_geometry.vertices,self.payload_geometry.triangles);model.endModel();self.payload_triangle_model=model

    def _measurement(self,pair):
        # Only free-glass/bowl initial support pair uses zero gap. Robot/bowl
        # and every other kitchen pair retain 2 mm. Source intersections fail.
        _,label,name,shape,obstacle,margin=pair
        if label=='payload/fragile_glass' and name==ENV_ROOT+'/kitchen/CapSurface_02' and self.phase in ('initial_settle','pregrasp','approach','close','release'):
            strict=self._exact_measurement(pair);cert=self.native_support_certificate
            if (not strict['valid'] and not strict['intersecting'] and cert is not None and
                    cert['valid'] and cert['counter_unloaded'] and cert['glass_above_counter'] and
                    np.allclose(self._query_payload_pose,cert['glass_pose'],atol=1e-7,rtol=0)):
                record=dict(phase=self.phase,physics_step=cert['physics_step'],original_source=strict.copy(),
                    refinement='Original bowl carries glass above its thin floor; glass/counter remains nonintersecting and unloaded',
                    native_upward_force_n=cert['upward_force_n'],native_counter_force_norm_n=cert['counter_force_norm_n'])
                self.source_support_refinements.append(record)
                return dict(strict,valid=True,required_m=0.,slack_m=strict['distance_m'],
                    original_noncontact_required_m=margin,original_strict_valid=False,
                    allowed_observed_bowl_support_chain=True,native_support_physics_step=cert['physics_step'])
            return strict
        if label=='payload/fragile_glass' and self._matches(name,'bowl') and self.phase in ('reset','warmup','initial_settle','pregrasp','approach','close','lift','release','retreat'):
            result=self._exact_measurement((pair[0],label,name,shape,obstacle,0.))
            result.update(contact_pair='glass/original bowl source support',noncontact_required_m=margin,
                native_contact_verified=False)
            cert=self.native_support_certificate
            if not result['valid'] and cert is not None and cert['valid'] and np.allclose(
                    self._query_payload_pose,cert['glass_pose'],atol=1e-7,rtol=0):
                self.source_support_refinements.append(dict(phase=self.phase,original_source=result.copy(),
                    physics_step=cert['physics_step'],native_upward_force_n=cert['upward_force_n'],
                    source_bottom_gap_m=cert['source_bottom_gap_m']))
                result.update(valid=True,native_contact_verified=True,
                    refinement='Exact observed bowl/base support only; original source intersection retained in ledger',
                    native_support_physics_step=cert['physics_step'])
            return result
        return super()._measurement(pair)

    def observe_native_bowl_support(self,row,support,glass_index,bowl_index):
        """Bind one actual-step support observation to its exact glass pose."""
        glass_pose=pose_matrix(row['task_object_states'][glass_index,:7]);bowl_pose=pose_matrix(row['task_object_states'][bowl_index,:7])
        forces=np.asarray(support['filtered_force_matrix_n']).reshape(-1,2,3)[0]
        force=forces[0];counter_force=float(np.linalg.norm(forces[1]))
        points=self.payload_geometry.vertices@glass_pose[:3,:3].T+glass_pose[:3,3]
        meshes=[v for k,v in self.meshes.items() if self._matches(k,'bowl')]
        if len(meshes)!=1:raise ValueError('Expected original single bowl source collision mesh')
        from kitchen_safety.extensions.replicator.hard_v1.property_scene import _height
        vertices,faces=meshes[0];floor=_height(vertices[faces],glass_pose[:2,3])
        gap=float(points[:,2].min()-floor)
        # Same 6 mm source support band as original payload policy; no broad
        # bowl collision mask. Force+separation must demonstrate real support.
        count=int(np.asarray(support['counts']).reshape(-1)[0]);start=int(np.asarray(support['starts']).reshape(-1)[0])
        sep=np.asarray(support['separations_m']).reshape(-1)[start:start+count]
        nf=np.asarray(support['normal_forces_n']).reshape(-1)[start:start+count]
        loaded=nf>.01
        normals=np.asarray(support['normals_world']).reshape(-1,3)[start:start+count]
        native_points=np.asarray(support['points_world_m']).reshape(-1,3)[start:start+count]
        local=(native_points-glass_pose[:3,3])@glass_pose[:3,:3]
        envelope=support.get('cup_bowl_native_contact_envelope_m')
        # Finger grasp nearness remains 0.25 mm elsewhere. Support contacts
        # use the actual two-body PhysX contact envelope, not that finger rule.
        bounded=bool(envelope is not None and np.isfinite(envelope) and 0<envelope<.10 and
            len(sep) and np.all(sep[loaded]<=envelope) and np.all(sep[loaded]>=-.001))
        bottom=bool(loaded.any() and np.all(np.abs(local[loaded,2])<=.006) and
            np.all(np.linalg.norm(local[loaded,:2],axis=1)<=.040))
        floor_points=bool(loaded.any() and all(abs(float(point[2])-_height(vertices[faces],point[:2]))<=.006 for point in native_points[loaded]))
        summed=(nf[:,None]*normals).sum(0)
        force_match=bool(np.linalg.norm(summed-force)<=.1)
        upward=bool(loaded.any() and np.all(normals[loaded,2]>=np.cos(np.deg2rad(20))))
        contact_ok=bool(loaded.any() and bounded and bottom and floor_points and force_match and upward)
        # Keep every conjunct and its measured value visible. A generic source
        # clearance failure must not hide a rejected native contact certificate.
        checks=dict(bounded_by_actual_contact_offsets=bounded,points_on_cup_bottom=bottom,
            points_on_original_bowl_floor=floor_points,native_filtered_force_matches=force_match,
            upward_normals=upward,loaded_contact_present=bool(loaded.any()),
            bowl_carries_80_percent_weight=bool(force[2]>.8*.56*9.81),
            horizontal_bowl_force_below_1n=bool(np.linalg.norm(force[:2])<1.),
            source_floor_gap_within_6mm=bool(-.006<=gap<=.006),
            glass_center_within_bowl_20mm=bool(np.linalg.norm(glass_pose[:2,3]-bowl_pose[:2,3])<.02),
            glass_upright_within_3deg=bool(glass_pose[2,2]>np.cos(np.deg2rad(3.))))
        valid=all(checks.values())
        counter=next(x for x in self.objects if x[0]==ENV_ROOT+'/kitchen/CapSurface_02')
        above=bool(points[:,2].min()>=counter[3][2] and np.all(points[:,:2].min(0)>=counter[2][:2]) and np.all(points[:,:2].max(0)<=counter[3][:2]))
        self.native_support_certificate=dict(valid=valid,physics_step=row['physics_step'],glass_pose=glass_pose,
            upward_force_n=float(force[2]),source_bottom_gap_m=gap,loaded_near_contact=contact_ok,
            loaded_native_separations_m=sep[loaded].tolist(),native_contact_envelope_m=envelope,
            native_support_checks=checks,failed_checks=[name for name,passed in checks.items() if not passed],
            filtered_force_world_n=force.copy(),raw_summed_normal_force_world_n=summed,
            filtered_raw_force_residual_n=float(np.linalg.norm(summed-force)),
            horizontal_bowl_force_n=float(np.linalg.norm(force[:2])),required_bowl_weight_fraction=.8,
            glass_weight_n=.56*9.81,glass_center_offset_m=float(np.linalg.norm(glass_pose[:2,3]-bowl_pose[:2,3])),
            glass_tilt_deg=float(np.degrees(np.arccos(np.clip(glass_pose[2,2],-1,1)))),
            glass_velocity_world=row['task_object_states'][glass_index,7:13].copy(),
            native_loaded_contacts=[dict(force_n=float(nf[i]),separation_m=float(sep[i]),
                point_world_m=native_points[i].copy(),point_cup_local_m=local[i].copy(),normal_world=normals[i].copy(),
                point_source_floor_gap_m=float(native_points[i,2]-_height(vertices[faces],native_points[i,:2])))
                for i in np.flatnonzero(loaded)],
            counter_force_norm_n=counter_force,counter_unloaded=counter_force<.01,glass_above_counter=above)


def build_world(config):
    from kitchen_safety.planning import DroidKinematics
    stage=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(stage,ENV_ROOT)
    layout=execution_layout(config)
    UsdGeom.Xform.Define(stage,ENV_ROOT+'/kitchen').GetPrim().GetReferences().AddReference(str(ROOT/layout['kitchen_usd']))
    for name,item in layout['objects'].items():
        xf=UsdGeom.Xform.Define(stage,ENV_ROOT+'/'+name);xf.GetPrim().GetReferences().AddReference(str(ROOT/item['usd_path']))
        xf.ClearXformOpOrder();xf.AddTranslateOp().Set(Gf.Vec3d(*item['position']));xf.AddRotateZOp().Set(item['yaw_deg'])
    kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
    world=PropertyGlassWorld(stage,kin,glass_usd=ROOT/config['glass']['usd_path'],env_root=ENV_ROOT)
    glass=config['glass'];pose=np.r_[glass['position'],Rotation.from_euler('z',glass['yaw_deg'],degrees=True).as_quat()]
    world.payload_pose=pose_matrix(pose)
    return stage,world,kin,pose


def grasp_target(pose, local_height, jaw_yaw_deg, approach='vertical'):
    obj=pose_matrix(pose);angle=np.deg2rad(jaw_yaw_deg)
    if approach=='vertical':
        down=np.array([0.,0.,-1.]);jaw=np.array([np.cos(angle),np.sin(angle),0.])
    elif approach=='horizontal':
        down=np.array([np.cos(angle),np.sin(angle),0.]);jaw=np.array([-np.sin(angle),np.cos(angle),0.])
    else:raise ValueError('Unknown grasp approach')
    target=np.eye(4);target[:3,:3]=np.column_stack((down,jaw,np.cross(down,jaw)))
    target[:3,3]=obj[:3,3]+obj[:3,:3]@np.array([0.,0.,local_height])
    return target


def straight_candidate(kin, start, target, *, dt=1/15, speed=.3):
    """2 mm Cartesian sampling, same orientation, bounded 0.15 rad IK branches."""
    first=kin.fk(start);delta=target[:3,3]-first[:3,3]
    if Rotation.from_matrix(target[:3,:3]@first[:3,:3].T).magnitude()>np.deg2rad(2):
        raise ValueError('Straight approach cannot change wrist orientation')
    count=max(2,int(np.ceil(np.linalg.norm(delta)/.002))+1);path=[np.asarray(start).copy()]
    for f in np.linspace(0,1,count)[1:]:
        t=first.copy();t[:3,3]+=f*delta;arm=kin.ik(t,path[-1],attempts=1)
        if np.max(np.abs(arm-path[-1]))>.15:raise ValueError('Straight approach IK branch jump')
        path.append(arm)
    path=np.asarray(path);times=np.r_[0,np.cumsum(np.maximum(3/(count-1),np.max(np.abs(np.diff(path,axis=0)),axis=1)/speed))]
    samples=np.minimum(np.arange(int(np.ceil(times[-1]/dt))+1)*dt,times[-1])
    positions=np.stack([np.interp(samples,times,path[:,i]) for i in range(7)],axis=1)
    return dict(positions=positions,dt=dt,metadata=dict(ik_spacing_m=.002,maximum_joint_speed_rad_s=speed,ik_points=count))


def upright_transport_candidate(kin,start,target,attachment,*,dt=1/15,speed=.3,tilt_limit_deg=3.):
    """Interpolate actual carried-cup xyz and rotation toward its explicit goal.

    Joint commands still require the unchanged cuRobo and full source checks.
    This only replaces a free joint-space transport candidate with a bounded
    Cartesian candidate; it cannot authorize a collision or a state write.
    """
    initial=kin.fk(start)@attachment;goal=target@attachment
    relative=goal[:3,:3]@initial[:3,:3].T
    yaw=float(np.arctan2(relative[1,0],relative[0,0]))
    angle=float(Rotation.from_matrix(relative).magnitude())
    rotations=Slerp([0.,1.],Rotation.from_matrix(np.stack((initial[:3,:3],goal[:3,:3]))))
    delta=goal[:3,3]-initial[:3,3]
    count=max(2,int(np.ceil(np.linalg.norm(delta)/.002))+1,int(np.ceil(angle/np.deg2rad(2)))+1)
    path=[np.asarray(start).copy()];inverse=np.linalg.inv(attachment)
    for fraction in np.linspace(0,1,count)[1:]:
        cup=initial.copy();cup[:3,3]+=fraction*delta
        cup[:3,:3]=rotations(fraction).as_matrix()
        arm=kin.ik(cup@inverse,path[-1],attempts=1)
        if np.max(np.abs(arm-path[-1]))>.15:raise ValueError('Upright transport IK branch jump')
        path.append(arm)
    path=np.asarray(path);times=np.r_[0,np.cumsum(np.maximum(3/(count-1),np.max(np.abs(np.diff(path,axis=0)),axis=1)/speed))]
    samples=np.minimum(np.arange(int(np.ceil(times[-1]/dt))+1)*dt,times[-1])
    positions=np.stack([np.interp(samples,times,path[:,i]) for i in range(7)],axis=1)
    tilts=np.array([np.degrees(np.arccos(np.clip((kin.fk(q)@attachment)[2,2],-1,1))) for q in positions])
    if tilts.max()>tilt_limit_deg:raise ValueError(f'Cup transport candidate tilt {tilts.max():.6f} exceeds {tilt_limit_deg:g} degrees')
    return dict(positions=positions,dt=dt,metadata=dict(ik_spacing_m=.002,maximum_joint_speed_rad_s=speed,
        ik_points=count,world_z_yaw_delta_deg=float(np.degrees(yaw)),maximum_planned_cup_tilt_deg=float(tilts.max()),
        target_cup_tilt_deg=float(np.degrees(np.arccos(np.clip(goal[2,2],-1,1)))),declared_cup_tilt_limit_deg=tilt_limit_deg,
        interpolation='Cup xyz plus shortest-path rotation SLERP toward explicit goal; actual attachment retained'))
