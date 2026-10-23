"""Measured-frame Cartesian keyframes for native DROID task controllers.

This generates real joint commands. It does not teleport objects, predict task
success, avoid collisions, or evaluate risk. Each task needs explicit keyframes
and independent full-motion acceptance before production use.
"""
from copy import deepcopy
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from kitchen_safety.task_contract import digest


def pose_matrix(pose):
    p=np.asarray(pose,dtype=float)
    if p.shape[0]<7 or not np.isfinite(p[:7]).all() or abs(np.linalg.norm(p[3:7])-1)>1e-4:
        raise ValueError('Expected measured world metres and xyzw quaternion')
    result=np.eye(4);result[:3,3]=p[:3];result[:3,:3]=Rotation.from_quat(p[3:7]).as_matrix()
    return result


def resolve_target_pose(target,reference,robot_base,schema_version):
    """Resolve one control target, keeping position and orientation frames explicit."""
    if schema_version=='robolaw-cartesian-keyframes-1':
        if set(target)!={'reference','offset_m','rotation_world_xyzw'}:
            raise ValueError('Version 1 requires an explicit world orientation')
        rotation_reference='world';quaternion=target['rotation_world_xyzw']
    elif schema_version=='robolaw-cartesian-keyframes-2':
        if set(target)!={'reference','offset_m','rotation_reference','rotation_xyzw'}:
            raise ValueError('Version 2 requires explicit position and orientation reference frames')
        rotation_reference=target['rotation_reference'];quaternion=target['rotation_xyzw']
    else:raise ValueError('Unknown Cartesian keyframe schema')
    if rotation_reference not in ('world','reference','robot_base'):
        raise ValueError('Unknown target orientation reference')
    offset=np.asarray(target['offset_m'],dtype=float);quaternion=np.asarray(quaternion,dtype=float)
    if offset.shape!=(3,) or quaternion.shape!=(4,) or not np.isfinite(offset).all():
        raise ValueError('Target requires a finite 3D offset and xyzw quaternion')
    local=pose_matrix(np.r_[offset,quaternion]);result=np.eye(4)
    result[:3,3]=reference[:3,3]+reference[:3,:3]@offset
    orientation_frame={'world':np.eye(4),'reference':reference,'robot_base':robot_base}[rotation_reference]
    result[:3,:3]=orientation_frame[:3,:3]@local[:3,:3]
    return result


class StageArmKinematics:
    """Read the actual serial joint frames, axes and limits from this stage."""
    def __init__(self,stage,robot_root,joint_names,tcp_body_name,tcp_offset):
        from pxr import Usd,UsdPhysics
        root=stage.GetPrimAtPath(robot_root)
        if not root:raise ValueError('Missing current robot root')
        joints=[p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]
        by_name={}
        for p in joints:by_name.setdefault(p.GetName(),[]).append(p)
        if any(len(by_name.get(n,[]))!=1 for n in joint_names):raise ValueError('Arm joints must bind uniquely')
        self.names=list(joint_names);self.limits=[];self.chain=[]
        start=UsdPhysics.Joint(by_name[self.names[0]][0]).GetBody0Rel().GetTargets()
        if len(start)!=1:raise ValueError('Arm base is ambiguous')
        self.base_actor=str(start[0]);cursor=self.base_actor;visited=set();moving=[]
        def frame(j,side):
            q=getattr(j,'GetLocalRot'+side+'Attr')().Get();p=getattr(j,'GetLocalPos'+side+'Attr')().Get()
            return pose_matrix([*p,*q.GetImaginary(),q.GetReal()])
        while cursor.rsplit('/',1)[-1]!=tcp_body_name:
            candidates=[]
            for p in joints:
                j=UsdPhysics.Joint(p)
                if list(map(str,j.GetBody0Rel().GetTargets()))==[cursor] and (p.GetName() in self.names or p.IsA(UsdPhysics.FixedJoint)):
                    candidates.append(p)
            if len(candidates)!=1:raise ValueError('TCP chain is missing or ambiguous at '+cursor)
            p=candidates[0];j=UsdPhysics.Joint(p);targets=j.GetBody1Rel().GetTargets()
            if len(targets)!=1 or str(targets[0]) in visited:raise ValueError('Invalid serial robot topology')
            if not j.GetJointEnabledAttr().Get():raise ValueError('Disabled joint in control chain')
            index=None;axis=None
            if p.GetName() in self.names:
                if not p.IsA(UsdPhysics.RevoluteJoint):raise ValueError('This DROID profile requires revolute arm joints')
                r=UsdPhysics.RevoluteJoint(p);index=self.names.index(p.GetName());moving.append(p.GetName())
                axis={'X':[1,0,0],'Y':[0,1,0],'Z':[0,0,1]}[str(r.GetAxisAttr().Get())]
                self.limits.append(np.deg2rad([r.GetLowerLimitAttr().Get(),r.GetUpperLimitAttr().Get()]))
            self.chain.append((frame(j,'0'),np.linalg.inv(frame(j,'1')),index,axis))
            cursor=str(targets[0]);visited.add(cursor)
        if moving!=self.names:raise ValueError('Arm command order differs from stage serial chain')
        self.tcp_actor=cursor;self.offset=np.asarray(tcp_offset,dtype=float);self.limits=np.asarray(self.limits)
        if self.offset.shape!=(3,) or not np.isfinite(self.offset).all() or not np.isfinite(self.limits).all():
            raise ValueError('Invalid explicit TCP or finite joint limits')

    def fk(self,q,base):
        result=base.copy()
        for first,last,index,axis in self.chain:
            rotation=np.eye(4)
            if index is not None:rotation[:3,:3]=Rotation.from_rotvec(np.asarray(axis)*q[index]).as_matrix()
            result=result@first@rotation@last
        result[:3,3]+=result[:3,:3]@self.offset
        return result

    def measured_tcp(self,state):
        result=pose_matrix(state['actor_states_world'][self.tcp_actor])
        result[:3,3]+=result[:3,:3]@self.offset
        return result

    def ik(self,target,seed,base):
        def residual(q):
            actual=self.fk(q,base)
            return np.r_[actual[:3,3]-target[:3,3],.25*Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).as_rotvec()]
        limits=self.limits
        fit=least_squares(lambda q:np.r_[residual(q),.0001*(q-seed)],
            np.clip(seed,limits[:,0]+1e-6,limits[:,1]-1e-6),bounds=(limits[:,0]+1e-6,limits[:,1]-1e-6),max_nfev=240,gtol=1e-9)
        actual=self.fk(fit.x,base)
        position=float(np.linalg.norm(actual[:3,3]-target[:3,3]))
        angle=float(np.linalg.norm(Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).as_rotvec()))
        if position>.002 or angle>.02:raise RuntimeError(f'Keyframe IK failed: position={position}, angle={angle}')
        return fit.x,dict(position_error_m=position,orientation_error_rad=angle)


def create(*,env,contract,bindings,config):
    import torch
    if config.get('task_sha256')!=digest(contract):raise ValueError('Keyframes belong to another task contract')
    if contract['roles']['robot']['registry_name']!='droid_abs_joint_pos':raise ValueError('Explicit DROID action profile required')
    if env.unwrapped.action_manager.total_action_dim!=8 or abs(env.unwrapped.step_dt-1/30)>1e-10:
        raise ValueError('DROID keyframes require eight absolute controls at 30Hz')
    allowed={'schema_version','task_sha256','tcp_body_name','tcp_offset_m','max_joint_speed_rad_s',
        'max_joint_acceleration_rad_s2','tracking_limit_rad','settle_tolerance_rad','settle_timeout_s','keyframes','purpose'}
    schema_version=config.get('schema_version')
    if set(config)-allowed or schema_version not in ('robolaw-cartesian-keyframes-1','robolaw-cartesian-keyframes-2'):
        raise ValueError('Unknown keyframe config fields/version')
    names=['panda_joint'+str(i) for i in range(1,8)];robot=env.unwrapped.scene['robot']
    ids=[robot.joint_names.index(n) for n in names]
    kin=StageArmKinematics(env.unwrapped.sim.stage,bindings['robot_root'],names,config['tcp_body_name'],config['tcp_offset_m'])
    frames=deepcopy(config['keyframes']);operations={o['operation_id'] for o in contract['operations']}
    speed=float(config['max_joint_speed_rad_s']);accel=float(config['max_joint_acceleration_rad_s2'])
    tracking=float(config['tracking_limit_rad']);tolerance=float(config['settle_tolerance_rad']);timeout=float(config['settle_timeout_s'])
    if not frames or not np.isfinite([speed,accel,tracking,tolerance,timeout]).all() or min(speed,accel,tracking,tolerance,timeout)<=0:
        raise ValueError('Explicit positive finite controller timing/limits required')
    for frame in frames:
        if set(frame)-{'kind','phase','operation_id','gripper','duration_s','target'}:raise ValueError('Unknown keyframe fields')
        if frame['kind'] not in ('move','hold') or frame['gripper'] not in (0,1) or not isinstance(frame['phase'],str) or not frame['phase']:
            raise ValueError('Invalid keyframe kind/gripper/phase')
        if frame.get('operation_id') is not None and frame['operation_id'] not in operations:raise ValueError('Unknown operation')
        if not math.isfinite(frame['duration_s']) or frame['duration_s']<=0:raise ValueError('Invalid duration')
        if frame['kind']=='move':
            target=frame['target']
            resolve_target_pose(target,np.eye(4),np.eye(4),schema_version)
            ref=target['reference']
            if ref!='current_tcp':
                if not isinstance(ref,dict) or set(ref)!={'asset_id','actor_relative_path'} or ref['asset_id'] not in bindings['asset_roots']:
                    raise ValueError('Target must bind a declared current task asset')
                path=ref['actor_relative_path']
                if path and (path.startswith('/') or any(x in path.split('/') for x in ('.','..',''))):raise ValueError('Invalid relative actor path')
        elif 'target' in frame:raise ValueError('Hold keyframe cannot have a target')
    class Policy:
        def __init__(self):
            self.index=-1;self.tick=0;self.previous=None;self.goal=None;self.last_evidence={};self.settle=0
        def act(self,state,last_result):
            q=np.asarray(state['robot']['q'])[ids];base=pose_matrix(state['actor_states_world'][kin.base_actor])
            measured=kin.measured_tcp(state);predicted=kin.fk(q,base)
            calibration=float(np.max(np.abs(measured-predicted)))
            if calibration>.003:raise RuntimeError('Loaded robot/TCP kinematics disagree with actual state: '+str(calibration))
            if self.previous is not None and np.max(np.abs(q-self.previous))>tracking:
                raise RuntimeError('Measured arm tracking exceeded explicit controller limit')
            if self.goal is not None and self.tick>=self.steps:
                self.settle+=1
                if np.max(np.abs(q-self.goal))<=tolerance:self.goal=None
                elif self.settle>math.ceil(timeout*30):raise RuntimeError('Keyframe measured settling timed out')
            if self.goal is None:
                self.index+=1
                if self.index==len(frames):return None
                self.frame=frames[self.index];self.start=q.copy();self.goal=q.copy();self.tick=0;self.settle=0
                self.last_evidence={}
                if self.frame['kind']=='move':
                    target=self.frame['target'];ref=target['reference']
                    if ref=='current_tcp':reference=measured
                    else:
                        path=bindings['asset_roots'][ref['asset_id']]
                        if ref['actor_relative_path']:path+='/'+ref['actor_relative_path']
                        reference=pose_matrix(state['actor_states_world'][path])
                    desired=resolve_target_pose(target,reference,base,schema_version)
                    self.goal,ik=kin.ik(desired,q,base)
                    self.last_evidence=dict(target_world_matrix=desired.tolist(),reference_world_matrix=reference.tolist(),ik=ik,
                        rotation_reference=target.get('rotation_reference','world'),robot_base_actor=kin.base_actor)
                delta=float(np.max(np.abs(self.goal-self.start)))
                duration=max(self.frame['duration_s'],1.875*delta/speed,math.sqrt(5.774*delta/accel))
                self.steps=max(1,math.ceil(duration*30))
                print('CARTESIAN_KEYFRAME',self.index,self.frame['phase'],'steps',self.steps,flush=True)
            self.tick+=1;u=min(1.,self.tick/self.steps);fraction=10*u**3-15*u**4+6*u**5
            command=self.start+fraction*(self.goal-self.start);self.previous=command.copy()
            evidence=dict(source='measured-frame Cartesian controller',keyframe_index=self.index,keyframe_step=self.tick,
                config_sha256=digest(config),kinematics_max_matrix_error=calibration,**self.last_evidence,
                controller_collision_avoidance=False,task_success_source='native termination only')
            return dict(action=torch.as_tensor(np.r_[command,self.frame['gripper']][None],dtype=torch.float32,device=env.unwrapped.device),
                phase=self.frame['phase'],operation_id=self.frame.get('operation_id'),evidence=evidence)
    return Policy()
