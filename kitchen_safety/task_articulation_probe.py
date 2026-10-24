"""Explicit external-joint diagnostic actuation, never a robot task policy.

For safety pipeline verification only: preserve physics, capture actual effort
submissions and measured motion. No pose teleport, mesh query or risk feature.
"""
from copy import deepcopy
import math
from kitchen_safety.task_contract import digest
from kitchen_safety.task_scope import under

VERSION='robolaw-articulation-motion-probe-1'


def validate_command(command,contract):
    fields={'version','task_sha256','operation_id','effort_si','effort_limit_si','diagnostic_only'}
    if not isinstance(command,dict) or set(command)!=fields or command['version']!=VERSION or command['diagnostic_only'] is not True:
        raise ValueError('Explicit external-articulation diagnostic command required')
    if command['task_sha256']!=digest(contract):raise ValueError('Diagnostic command belongs to another task')
    ops=[o for o in contract['operations'] if o['operation_id']==command['operation_id'] and 'articulation_id' in o]
    if len(ops)!=1:raise ValueError('Diagnostic requires one declared articulation operation')
    effort,limit=command['effort_si'],command['effort_limit_si']
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (effort,limit)) or limit<=0 or abs(effort)>limit:
        raise ValueError('Diagnostic effort exceeds its declared finite bound')
    return ops[0]


def validate_transition(previous,command):
    """Do not leave an unrecorded nonzero force on the previous joint."""
    if previous is None:return
    old=previous['command']
    if old['effort_si']!=0 and (command is None or command.get('operation_id')!=old['operation_id']):
        raise ValueError('Submit explicit zero effort before leaving the diagnostic joint')


def resolve_command(command,contract,bindings,inventory,views):
    op=validate_command(command,contract);root=bindings['asset_roots'][op['articulation_id']]
    matches=[]
    for view in views:
        if int(view.count)!=1:continue
        for index,path in enumerate(map(str,view.dof_paths[0])):
            if under(path,root) and path.rsplit('/',1)[-1]==op['joint_name']:
                matches.append((view,index,path))
    if len(matches)!=1:raise ValueError('Diagnostic DOF must resolve uniquely within this task asset')
    view,index,path=matches[0]
    joints=[j for j in inventory['joints'] if j['path']==path and j['enabled']]
    if len(joints)!=1 or joints[0]['type'] not in ('PhysicsRevoluteJoint','PhysicsPrismaticJoint'):
        raise ValueError('Diagnostic requires a supported enabled joint with known SI units')
    if under(path,bindings['robot_root']):raise ValueError('External probe cannot command robot joints')
    return dict(view=view,index=index,path=path,root=str(view.prim_paths[0]),
        unit='Nm' if joints[0]['type']=='PhysicsRevoluteJoint' else 'N',command=deepcopy(command))


def submit_command(bound,*,device,physics_step,action_index):
    """Write immediately before physics, then record actual backend readback."""
    import torch
    from kitchen_safety.articulation_capture import array
    import numpy as np
    view=bound['view'];original=view.get_dof_actuation_forces();before=array(original)
    if before.shape!=(1,int(view.max_dofs)) or not all(math.isfinite(float(v)) for v in before.flat):
        raise ValueError('Cannot preserve non-target external actuation channels')
    target=before.copy();target[0,bound['index']]=bound['command']['effort_si']
    # The discovered scene views can use Warp even when the policy uses
    # Torch. Preserve the backend tensor family; this is command transport,
    # unrelated to the offline gDist distance backend.
    if isinstance(original,np.ndarray):
        values=target.astype(np.float32);indices=np.asarray([0],dtype=np.int32)
    elif isinstance(original,torch.Tensor):
        values=torch.as_tensor(target,dtype=torch.float32,device=original.device)
        indices=torch.tensor([0],dtype=torch.int32,device=original.device)
    else:
        import warp as wp
        if not isinstance(original,wp.array):raise ValueError('Unsupported PhysX command tensor frontend')
        values=wp.array(target,dtype=wp.float32,device=original.device)
        indices=wp.array([0],dtype=wp.int32,device=original.device)
    view.set_dof_actuation_forces(values,indices)
    actual=array(view.get_dof_actuation_forces())
    if actual.shape!=target.shape or not (abs(actual-target)<=1e-6).all():raise ValueError('External actuation readback differs')
    return dict(method='set_dof_actuation_forces',command_scope='diagnostic_external_articulation',
        before_physics_step=physics_step,action_index=action_index,root_path=bound['root'],dof_path=bound['path'],
        dof_index=bound['index'],unit=bound['unit'],prior=before.tolist(),nominal=target.tolist(),submitted=actual.tolist(),
        success=True,diagnostic_only=True,operation_execution_claimed=False,command=deepcopy(bound['command']))


def create(*,env,contract,bindings,config):
    import torch
    if set(config)!={'version','task_sha256','operation_id','effort_limit_si','segments'} or config['version']!=VERSION:
        raise ValueError('Explicit diagnostic motion schedule required')
    if not config['segments']:raise ValueError('Empty diagnostic schedule')
    schedule=[]
    for segment in config['segments']:
        if set(segment)!={'actions','effort_si'} or type(segment['actions']) is not int or segment['actions']<=0:
            raise ValueError('Invalid diagnostic segment')
        command=dict(version=VERSION,task_sha256=config['task_sha256'],operation_id=config['operation_id'],
            effort_limit_si=config['effort_limit_si'],effort_si=segment['effort_si'],diagnostic_only=True)
        validate_command(command,contract);schedule.extend([command]*segment['actions'])
    if schedule[-1]['effort_si']!=0:raise ValueError('Probe must end with explicit zero external effort')
    if contract['roles']['robot']['registry_name']!='droid_abs_joint_pos' or env.unwrapped.action_manager.total_action_dim!=8:
        raise ValueError('Diagnostic hold requires declared DROID absolute-joint controls')
    if abs(env.unwrapped.step_dt-1/30)>1e-10:raise ValueError('Diagnostic requires 30Hz commands')
    names=env.unwrapped.scene['robot'].joint_names;indices=[names.index('panda_joint'+str(i)) for i in range(1,8)]
    class Policy:
        def __init__(self):self.index=0
        def act(self,state,last_result):
            if self.index==len(schedule):return None
            command=deepcopy(schedule[self.index]);self.index+=1
            q=[float(state['robot']['q'][i]) for i in indices]
            if not all(math.isfinite(v) for v in q):raise ValueError('Invalid measured robot hold reference')
            return dict(action=torch.tensor([q+[0.]],dtype=torch.float32,device=env.unwrapped.device),phase='operate',
                operation_id=config['operation_id'],diagnostic_actuation=command,
                evidence=dict(source='explicit external articulation diagnostic; robot measured hold',
                    config_sha256=digest(config),command_index=self.index-1,operation_execution_claimed=False,
                    full_task_trajectory=False,diagnostic_external_actuation=True))
    return Policy()
