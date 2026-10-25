"""Read actual PhysX kitchen body/joint states without changing the scene."""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation

class KitchenJointRecorder:
    def __init__(self, stage, sim, root='/World/envs/env_0/kitchen'):
        from pxr import Usd,UsdGeom,UsdPhysics
        self.stage=stage;self.cache=UsdGeom.XformCache();self.physics=sim.physics_manager.get_physics_sim_view()
        source_prims=list(Usd.PrimRange(stage.GetPrimAtPath(root),Usd.TraverseInstanceProxies()))
        rigid_paths=[str(p.GetPath()) for p in source_prims if p.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get() is not False]
        self.body_view=self.physics.create_rigid_body_view(rigid_paths)
        self.body_paths=list(self.body_view.prim_paths)
        if not self.body_paths:raise RuntimeError('No actual PhysX kitchen body states bound; USD transforms alone are insufficient')
        self.body_index={p:i for i,p in enumerate(self.body_paths)}
        self.body_com=self.array(self.body_view.get_coms())[:,:3]
        self.native_views=[];self.native_index={};self.joints=[];self.fixed_count=0
        self.native_bind_error=[]
        articulation_paths=[str(p.GetPath()) for p in source_prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        print('PHYSX_BODY_BIND',self.body_view.count,len(rigid_paths),self.body_paths[:3],flush=True)
        for path in articulation_paths:
            try:
                view=self.physics.create_articulation_view(path)
                if view.count!=1:raise RuntimeError(f'Expected one articulation, got {view.count}')
                paths=list(view.dof_paths[0])
                if not paths:continue
                view_id=len(self.native_views);self.native_views.append(view)
                for i,p in enumerate(paths):self.native_index[str(p)]=(view_id,i)
            except Exception as exc:self.native_bind_error.append({'path':path,'error':str(exc)})
        def endpoint(joint,side):
            rel=getattr(joint,'GetBody'+side+'Rel')().GetTargets()
            target=stage.GetPrimAtPath(rel[0]) if rel else None;body=target
            while body and not body.IsPseudoRoot():
                if body.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(body).GetRigidBodyEnabledAttr().Get() is not False:break
                body=body.GetParent()
            q=getattr(joint,'GetLocalRot'+side+'Attr')().Get()
            local=np.eye(4);local[:3,:3]=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix();local[:3,3]=np.asarray(getattr(joint,'GetLocalPos'+side+'Attr')().Get(),float)
            target_matrix=np.asarray(self.cache.GetLocalToWorldTransform(target)).T if target else np.eye(4)
            anchor=target_matrix@local
            if body and not body.IsPseudoRoot():
                path=str(body.GetPath())
                if path not in self.body_index:raise RuntimeError('Enabled joint body missing from actual PhysX view: '+path)
                matrix=np.asarray(self.cache.GetLocalToWorldTransform(body)).T
                rigid=np.eye(4);rigid[:3,:3]=Rotation.from_matrix(matrix[:3,:3]).as_matrix();rigid[:3,3]=matrix[:3,3]
                local=np.linalg.inv(rigid)@anchor
                return {'body':path,'index':self.body_index[path],'static_matrix':rigid,'local_R':Rotation.from_matrix(local[:3,:3]).as_matrix(),'local_p':local[:3,3]}
            anchor[:3,:3]=Rotation.from_matrix(anchor[:3,:3]).as_matrix()
            return {'body':None,'index':None,'static_matrix':anchor,'local_R':np.eye(3),'local_p':np.zeros(3)}
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root),Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdPhysics.Joint):continue
            j=UsdPhysics.Joint(prim)
            if j.GetJointEnabledAttr().Get() is False:continue
            if prim.IsA(UsdPhysics.FixedJoint):self.fixed_count+=1;continue
            if prim.IsA(UsdPhysics.RevoluteJoint):kind='revolute';axis=str(UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get());unit='rad'
            elif prim.IsA(UsdPhysics.PrismaticJoint):kind='prismatic';axis=str(UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get());unit='m'
            elif prim.GetTypeName()=='PhysicsJoint':
                path=str(prim.GetPath());ea=endpoint(j,'0');eb=endpoint(j,'1')
                for prefix,kind,unit in [('trans','prismatic','m'),('rot','revolute','rad')]:
                    for i,axis_name in enumerate('XYZ'):
                        self.joints.append({'path':path+'/'+prefix+axis_name,'source_joint':path,'component':prefix+axis_name,'kind':kind,'unit':unit,'axis':np.eye(3)[i],'a':ea,'b':eb,'native':None})
                continue
            else:raise RuntimeError('Unsupported enabled source joint requires explicit state reader: '+str(prim.GetPath()))
            path=str(prim.GetPath());self.joints.append({'path':path,'kind':kind,'unit':unit,'axis':np.eye(3)['XYZ'.index(axis)],'a':endpoint(j,'0'),'b':endpoint(j,'1'),'native':self.native_index.get(path)})
        if not self.joints:raise RuntimeError('Source joint inventory empty; do not pass vacuously')
        joint_bodies=set()
        for prim in source_prims:
            if not prim.IsA(UsdPhysics.Joint):continue
            joint=UsdPhysics.Joint(prim)
            for rel in (joint.GetBody0Rel(),joint.GetBody1Rel()):
                for target in rel.GetTargets():
                    body=stage.GetPrimAtPath(target)
                    while body and not body.IsPseudoRoot() and not body.HasAPI(UsdPhysics.RigidBodyAPI):body=body.GetParent()
                    if body and not body.IsPseudoRoot():joint_bodies.add(str(body.GetPath()))
        self.monitored_body_indices=[];self.excluded_marker_paths=[]
        for i,path in enumerate(self.body_paths):
            meaningful=(path in joint_bodies) or any((p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()) or (p.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(p).ComputeVisibility()!='invisible' and UsdGeom.Imageable(p).ComputePurpose() in ('default','render')) for p in Usd.PrimRange(stage.GetPrimAtPath(path),Usd.TraverseInstanceProxies()))
            if meaningful:self.monitored_body_indices.append(i)
            else:self.excluded_marker_paths.append(path)
        self.inventory={'native_articulation_bind_error':self.native_bind_error,'body_paths':self.body_paths,'excluded_nonphysical_marker_paths':self.excluded_marker_paths,'exclusion_rule':'No collision geometry, no renderable geometry, and not a source-joint endpoint; all raw states are retained','body_com_local_m':self.body_com.tolist(),'body_velocity_reference':'World-frame COM linear velocity and angular velocity','native_articulation_properties':[{'path':v.prim_paths[0],'dof_paths':list(v.dof_paths[0]),'friction_coefficients':self.array(v.get_dof_friction_coefficients()).tolist(),'friction_properties':self.array(v.get_dof_friction_properties()).tolist()} for v in self.native_views],'native_articulation_paths':[v.prim_paths[0] for v in self.native_views],'fixed_joint_count':self.fixed_count,'joints':[{'path':j['path'],'source_joint':j.get('source_joint',j['path']),'coordinate_component':j.get('component'),'kind':j['kind'],'unit':j['unit'],'body0':j['a']['body'],'body1':j['b']['body'],'method':('native PhysX articulation q/dq' if j['native'] else ('derived from actual PhysX rigid poses and velocities' if j['a']['index'] is not None or j['b']['index'] is not None else 'both endpoints authored static: no runtime DOF'))} for j in self.joints]}
    @staticmethod
    def array(x):
        if isinstance(x,np.ndarray):return x.copy()
        if hasattr(x,'detach'):return x.detach().cpu().numpy().copy()
        import warp as wp
        return wp.to_torch(x).detach().cpu().numpy().copy()
    def read(self):
        poses=self.array(self.body_view.get_transforms());velocities=self.array(self.body_view.get_velocities())
        native=[(self.array(v.get_dof_positions())[0],self.array(v.get_dof_velocities())[0]) for v in self.native_views]
        def frame(endpoint):
            i=endpoint['index']
            if i is None:T=endpoint['static_matrix'];R=T[:3,:3];p=T[:3,3];v=np.zeros(3);w=np.zeros(3)
            else:R=Rotation.from_quat(poses[i,3:7]).as_matrix();p=poses[i,:3];v=velocities[i,:3];w=velocities[i,3:6]
            arm=R@endpoint['local_p'];com_offset=np.zeros(3) if i is None else R@self.body_com[i]
            return R@endpoint['local_R'],p+arm,v+np.cross(w,arm-com_offset),w
        q=[];dq=[]
        for j in self.joints:
            if j['native']:
                vi,di=j['native'];q.append(float(native[vi][0][di]));dq.append(float(native[vi][1][di]));continue
            Ra,pa,va,wa=frame(j['a']);Rb,pb,vb,wb=frame(j['b']);axis=Ra@j['axis']
            if j.get('component'):
                component='XYZ'.index(j['component'][-1])
                if j['component'].startswith('trans'):
                    q.append(float((Ra.T@(pb-pa))[component]));dq.append(float((Ra.T@(vb-va-np.cross(wa,pb-pa)))[component]))
                else:
                    phi=Rotation.from_matrix(Ra.T@Rb).as_rotvec();theta=float(np.linalg.norm(phi));x,y,z=phi
                    H=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
                    coefficient=(1./12.+theta*theta/720.) if theta<1e-6 else (1.-.5*theta/np.tan(.5*theta))/(theta*theta)
                    derivative=(np.eye(3)-.5*H+coefficient*(H@H))@(Ra.T@(wb-wa))
                    q.append(float(phi[component]));dq.append(float(derivative[component]))
                continue

            if j['kind']=='revolute':q.append(float(Rotation.from_matrix(Ra.T@Rb).as_rotvec()@j['axis']));dq.append(float((wb-wa)@axis))
            else:q.append(float((pb-pa)@axis));dq.append(float((vb-va)@axis+(pb-pa)@np.cross(wa,axis)))
        return {'kitchen_body_pose':poses,'kitchen_body_velocity':velocities,'kitchen_joint_q':np.asarray(q),'kitchen_joint_dq':np.asarray(dq)}
    def summarize(self,rows,settle):
        q=rows['kitchen_joint_q'];dq=rows['kitchen_joint_dq'];body=rows['kitchen_body_pose'];velocity=rows['kitchen_body_velocity'];checks=[]
        for i,j in enumerate(self.joints):
            values=np.unwrap(q[:,i]) if j['kind']=='revolute' else q[:,i];limit=np.deg2rad(1.) if j['kind']=='revolute' else .001
            change=float(np.max(np.abs(values-values[0])));speed=float(np.max(np.abs(dq[settle:,i])))
            checks.append({'path':j['path'],'unit':j['unit'],'max_change':change,'max_speed_after_settling':speed,'position_limit':limit,'speed_limit_per_second':limit,'passed':bool(change<=limit and speed<=limit)})
        raw_translation=np.linalg.norm(body[:,:,:3]-body[0:1,:,:3],axis=-1)
        indices=getattr(self,'monitored_body_indices',list(range(len(self.body_paths))))
        body=body[:,indices]
        translation=np.linalg.norm(body[:,:,:3]-body[0:1,:,:3],axis=-1)
        rotation=np.array([np.rad2deg((Rotation.from_quat(x[:,3:7])*Rotation.from_quat(body[0,:,3:7]).inv()).magnitude()) for x in body])
        finite=all(np.isfinite(x).all() for x in (q,dq,body,velocity));body_ok=bool(translation.max()<=.002 and rotation.max()<=1.)
        return {'passed':bool(finite and body_ok and all(x['passed'] for x in checks)),'method':'Read-only PhysX tensor body states; native articulation DOFs where available, otherwise joint coordinates/rates derived from live rigid endpoint states. No USD-only motion certificate.','joint_count':len({j.get('source_joint',j['path']) for j in self.joints}),'coordinate_count':len(checks),'body_count':len(self.body_paths),'all_values_finite':bool(finite),'body_max_translation_m':float(translation.max()),'body_max_rotation_deg':float(rotation.max()),'body_motion_passed':body_ok,'body_translation_limit_m':.002,'raw_max_translation_including_nonphysical_markers_m':float(raw_translation.max()),'joint_checks':checks,'inventory':self.inventory}
