"""Measured-pose robot task attempts for one-per-task review recordings.

Real DROID joint/gripper commands only. No object teleports, external joint
actuation, risk features or risk queries. Execution and native success remain
separate, including unreachable goals or failed grasp/contact attempts.
"""
from copy import deepcopy
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from kitchen_safety.task_contract import digest
from kitchen_safety.task_cartesian_policy import StageArmKinematics,pose_matrix
from kitchen_safety.task_scope import under

VERSION='robolaw-task-review-policy-1'


def create(*,env,contract,bindings,config):
    import torch
    from pxr import Usd,UsdGeom,UsdPhysics
    if config.get('version')!=VERSION or config.get('task_sha256')!=digest(contract):
        raise ValueError('Explicit task-bound review policy required')
    if set(config)-{'version','task_sha256','joint_speed_rad_s','joint_acceleration_rad_s2','purpose'}:
        raise ValueError('Unknown task review policy field')
    if contract['roles']['robot']['registry_name']!='droid_abs_joint_pos' or env.unwrapped.action_manager.total_action_dim!=8 or abs(env.unwrapped.step_dt-1/30)>1e-10:
        raise ValueError('DROID absolute controls and 30Hz required')
    speed=float(config['joint_speed_rad_s']);accel=float(config['joint_acceleration_rad_s2'])
    if not .05<=speed<=.6 or not .05<=accel<=1.2:raise ValueError('Explicit bounded review motion required')
    stage=env.unwrapped.sim.stage;cache=UsdGeom.XformCache()
    names=['panda_joint'+str(i) for i in range(1,8)]
    indices=[env.unwrapped.scene['robot'].joint_names.index(n) for n in names]
    kin=StageArmKinematics(stage,bindings['robot_root'],names,'base_link',[.131,0,0])
    def bounds(path,state):
        # This command target extent comes from the actual collision vertices,
        # including guide/invisible surfaces, not potentially stale render hints.
        # It is not a risk-distance query or a collision/level calculation.
        from kitchen_safety.task_mesh import authored_surface
        root=stage.GetPrimAtPath(path)
        if not root:raise ValueError('Missing control reference '+path)
        clouds=[]
        for prim in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
            vertices,_,_=authored_surface(prim,cache)
            owner=prim
            while owner and str(owner.GetPath()) not in state['actor_states_world']:owner=owner.GetParent()
            if owner:
                authored=np.asarray(cache.GetLocalToWorldTransform(owner)).T
                u,_,vt=np.linalg.svd(authored[:3,:3]);rotation=u@vt
                if np.linalg.det(rotation)<0:u[:,-1]*=-1;rotation=u@vt
                actual=pose_matrix(state['actor_states_world'][str(owner.GetPath())])
                vertices=(vertices-authored[:3,3])@rotation@actual[:3,:3].T+actual[:3,3]
            clouds.append(vertices)
        if not clouds:raise ValueError('Control reference has no enabled collision surface: '+path)
        vertices=np.concatenate(clouds);return vertices.min(axis=0),vertices.max(axis=0)
    def world(path):return np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T
    def vector_rotation(x):
        x=x/np.linalg.norm(x);y=np.cross([0.,0.,1.],x)
        if np.linalg.norm(y)<.01:y=np.cross([0.,1.,0.],x)
        y/=np.linalg.norm(y);z=np.cross(x,y)
        return np.column_stack([x,y,z])
    def step(phase,position=None,rotations=None,gripper=0,duration=1.,operation_id=None):
        return dict(phase=phase,position=position,rotations=rotations,gripper=gripper,duration=duration,operation_id=operation_id)
    class Policy:
        def __init__(self):self.frames=None;self.index=-1;self.goal=None;self.previous=None;self.tick=0;self.failed=False;self.ending=False
        def plan(self,state):
            base=pose_matrix(state['actor_states_world'][kin.base_actor]);self.base=base
            down=base[:3,:3]@Rotation.from_euler('y',45,degrees=True).as_matrix()
            downward=[down,base[:3,:3]@Rotation.from_euler('y',90,degrees=True).as_matrix()]
            frames=[];geometries=[]
            for op in contract['operations']:
                oid=op['operation_id']
                if op['kind']=='PickAndPlaceTask':
                    aid=op['payload_id'];dest=op['destination_id'];root=bindings['asset_roots'][aid]
                    lo,hi=bounds(root,state);center=(lo+hi)/2;grasp=center.copy();registry=contract['asset_declarations'][aid].get('registry_name','')
                    if 'mustard' in registry:
                        grasp=pose_matrix(state['actor_states_world'][root])[:3,3].copy()
                    elif 'bowl' in registry:
                        toward=base[:2,3]-center[:2];toward/=np.linalg.norm(toward)
                        grasp[:2]+=toward*.42*min(hi[:2]-lo[:2]);grasp[2]=hi[2]-.018
                    elif 'ladle' in registry:
                        # Explicit exploratory handle-side grasp based on this loaded asset extent.
                        extent=hi-lo;axis=int(np.argmax(extent));grasp[axis]=lo[axis]+.24*extent[axis]
                    dl,dh=bounds(bindings['asset_roots'][dest],state);destination=(dl+dh)/2
                    height=grasp[2]-lo[2]
                    target=destination.copy();target[2]=dh[2]+height+.015
                    # Keep the payload centre above the destination when grasping a rim/handle.
                    target[:2]+=grasp[:2]-center[:2]
                    pre=grasp+np.array([0,0,.13]);lift=grasp+np.array([0,0,.20]);above=target+np.array([0,0,.15])
                    for phase,pos,grip,duration in [('pregrasp',pre,0,2.5),('approach',grasp,0,1.8),('grasp',None,1,1.),('lift',lift,1,2.),('transfer',above,1,3.),('lower',target,1,2.),('release',None,0,1.),('retreat',above,0,2.),('settle',None,0,1.5)]:
                        frames.append(step(phase,pos,downward,grip,duration,oid))
                    geometries.append(dict(operation_id=oid,asset_id=aid,source_extent_world_m=[lo.tolist(),hi.tolist()],destination_extent_world_m=[dl.tolist(),dh.tolist()],control_grasp_world_m=grasp.tolist(),source='current loaded geometry for command generation, not a risk distance'))
                else:
                    declaration=next(x for x in bindings['operation_contacts'] if x['operation_id']==oid)
                    boxes=[bounds(p,state) for p in declaration['target_colliders']]
                    lo=np.min([x[0] for x in boxes],axis=0);hi=np.max([x[1] for x in boxes],axis=0);handle=(lo+hi)/2
                    toward=base[:3,3]-handle;toward[2]=0;toward/=np.linalg.norm(toward)
                    orientation=vector_rotation(-toward)
                    rotations=[orientation@Rotation.from_euler('x',a,degrees=True).as_matrix() for a in (0,90,-90,45,-45,180)]
                    asset=bindings['asset_roots'][op['articulation_id']]
                    joints=[p for p in Usd.PrimRange(stage.GetPrimAtPath(asset)) if p.IsA(UsdPhysics.Joint) and p.GetName()==op['joint_name']]
                    if len(joints)!=1:raise ValueError('Commanded joint must bind uniquely')
                    prim=joints[0];joint=UsdPhysics.Joint(prim);parent=joint.GetBody0Rel().GetTargets()
                    parent_frame=world(str(parent[0])) if parent else np.eye(4)
                    q0=joint.GetLocalRot0Attr().Get();local=pose_matrix([*joint.GetLocalPos0Attr().Get(),*q0.GetImaginary(),q0.GetReal()]);frame=parent_frame@local
                    typed=UsdPhysics.PrismaticJoint(prim) if prim.IsA(UsdPhysics.PrismaticJoint) else UsdPhysics.RevoluteJoint(prim)
                    axis=frame[:3,:3]@np.eye(3)['XYZ'.index(str(typed.GetAxisAttr().Get()))]
                    lower=float(typed.GetLowerLimitAttr().Get());upper=float(typed.GetUpperLimitAttr().Get())
                    frames.append(step('pregrasp',handle+toward*.12,rotations,0,2.5,oid))
                    frames.append(step('approach',handle,rotations,0,2.,oid))
                    frames.append(step('operate',None,rotations,1,1.,oid))
                    if op['kind']=='PressButtonTask':
                        delta=(upper-lower)*.8
                        sign=1. if np.dot(axis,toward)<0 else -1.
                        delta=sign*min(abs(delta),.015)
                        for fraction in (.33,.66,1.):frames.append(step('operate',handle+axis*delta*fraction,rotations,1,.65,oid))
                    else:
                        angle=math.radians(upper-lower)
                        if not math.isfinite(angle) or abs(angle)>2*math.pi:angle=math.pi/2
                        fraction=.3 if op['kind']=='OpenDoorTask' else .65
                        delta=angle*fraction
                        for f in np.linspace(1/10,1,10):
                            rotation=Rotation.from_rotvec(axis*delta*f).as_matrix()
                            position=frame[:3,3]+rotation@(handle-frame[:3,3])
                            frames.append(step('operate',position,[rotation@r for r in rotations],1,.5,oid))
                    frames.append(step('release',None,None,0,1.,oid))
                    frames.append(step('retreat',handle+toward*.15,rotations,0,2.,oid))
                    frames.append(step('settle',None,None,0,1.,oid))
                    geometries.append(dict(operation_id=oid,contact_colliders=declaration['target_colliders'],handle_world_m=handle.tolist(),joint_axis_world=axis.tolist(),joint_origin_world=frame[:3,3].tolist(),source='actual task-bound operation surfaces and joint frames for robot commands only'))
            self.frames=frames;self.geometries=geometries
        def solve(self,position,rotations,q):
            limits=kin.limits;best=None
            candidates=[q,np.array([0,-.6,0,-2.1,0,1.6,.8]),np.array([.7,-.7,-.5,-2.2,.2,1.7,.3])]
            for rotation in rotations:
                target=np.eye(4);target[:3,:3]=rotation;target[:3,3]=position
                for seed in candidates:
                    def error(values):
                        actual=kin.fk(values,self.base)
                        return np.r_[actual[:3,3]-position,.18*Rotation.from_matrix(rotation@actual[:3,:3].T).as_rotvec(),.0001*(values-q)]
                    fit=least_squares(error,np.clip(seed,limits[:,0]+1e-6,limits[:,1]-1e-6),bounds=(limits[:,0]+1e-6,limits[:,1]-1e-6),max_nfev=150,gtol=1e-8)
                    actual=kin.fk(fit.x,self.base);p=float(np.linalg.norm(actual[:3,3]-position));a=float(np.linalg.norm(Rotation.from_matrix(rotation@actual[:3,:3].T).as_rotvec()))
                    score=p+.05*a+.0001*np.linalg.norm(fit.x-q)
                    item=(score,fit.x,dict(position_error_m=p,orientation_error_rad=a,target_world_matrix=target.tolist()))
                    if best is None or score<best[0]:best=item
                    if p<.002 and a<.025:return item[1],item[2]
            return best[1],best[2]
        def act(self,state,last_result):
            q=np.asarray(state['robot']['q'])[indices]
            if self.frames is None:self.plan(state)
            base=pose_matrix(state['actor_states_world'][kin.base_actor]);measured=kin.measured_tcp(state)
            calibration=float(np.abs(kin.fk(q,base)-measured).max())
            if calibration>.003:raise RuntimeError('Measured TCP disagrees with actual serial kinematics')
            if self.goal is not None and self.tick>=self.steps:
                self.settling+=1
                if np.abs(q-self.goal).max()<.035:self.goal=None
                elif self.settling>90:
                    self.failed=True;self.goal=None;self.ending=True
            if self.goal is None:
                if self.ending:return None
                self.index+=1
                if self.index>=len(self.frames):return None
                self.frame=self.frames[self.index];self.start=q.copy();self.goal=q.copy();self.tick=0;self.settling=0;self.evidence={}
                if self.frame['position'] is not None:
                    self.goal,self.evidence=self.solve(self.frame['position'],self.frame['rotations'],q)
                    if self.evidence['position_error_m']>.015 or self.evidence['orientation_error_rad']>.15:
                        self.failed=True;self.ending=True
                        self.evidence['controller_failure']='unreachable task target; execute best bounded joint approach then stop'
                duration=max(self.frame['duration'],1.875*float(np.abs(self.goal-q).max())/speed,math.sqrt(5.774*float(np.abs(self.goal-q).max())/accel))
                self.steps=max(1,math.ceil(duration*30))
                print('TASK_REVIEW_FRAME',self.index,self.frame['phase'],'steps',self.steps,'IK',self.evidence.get('position_error_m'),flush=True)
            self.tick+=1;u=min(1,self.tick/self.steps);fraction=10*u**3-15*u**4+6*u**5
            target=self.start+fraction*(self.goal-self.start)
            evidence=dict(source='task-bound measured robot review attempt',policy_version=VERSION,phase_semantics_version='canonical-grasp-phase-1',controller_stage='gripper_close' if self.frame['phase']=='grasp' else self.frame['phase'],config_sha256=digest(config),keyframe_index=self.index,keyframe_step=self.tick,controller_failed=self.failed,kinematics_max_matrix_error=calibration,object_pose_writes=0,external_articulation_actuation=False,task_success_source='native termination only',**self.evidence)
            if self.index==0 and self.tick==1:evidence['control_reference_geometry']=deepcopy(self.geometries)
            return dict(action=torch.tensor([target.tolist()+[self.frame['gripper']]],dtype=torch.float32,device=env.unwrapped.device),phase=self.frame['phase'],operation_id=self.frame['operation_id'],evidence=evidence)
    return Policy()
