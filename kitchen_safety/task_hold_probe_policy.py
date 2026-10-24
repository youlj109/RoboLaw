"""Task-bound measured hold diagnostic for raw/offline interface checks."""
import math
from kitchen_safety.task_contract import digest


def create(*,env,contract,bindings,config):
    import torch
    if set(config)!={'version','task_sha256','actions'} or config['version']!='robolaw-measured-hold-probe-1':
        raise ValueError('Explicit measured-hold diagnostic configuration required')
    if config['task_sha256']!=digest(contract) or type(config['actions']) is not int or not 0<config['actions']<=30:
        raise ValueError('Diagnostic task identity or short action count is invalid')
    if contract['roles']['robot']['registry_name']!='droid_abs_joint_pos' or env.unwrapped.action_manager.total_action_dim!=8:
        raise ValueError('Measured hold requires the declared DROID absolute-joint action profile')
    if abs(env.unwrapped.step_dt-1/30)>1e-10:raise ValueError('Diagnostic requires 30Hz commands')
    names=env.unwrapped.scene['robot'].joint_names;indices=[names.index('panda_joint'+str(i)) for i in range(1,8)]
    class Policy:
        def __init__(self):self.index=0
        def act(self,state,last_result):
            if self.index==config['actions']:return None
            q=[float(state['robot']['q'][i]) for i in indices]
            if not all(math.isfinite(v) for v in q):raise ValueError('Measured robot hold reference is invalid')
            index=self.index;self.index+=1
            return dict(action=torch.tensor([q+[0.]],dtype=torch.float32,device=env.unwrapped.device),
                phase='settle',operation_id=None,evidence=dict(source='measured hold diagnostic',config_sha256=digest(config),
                    action_index=index,full_task_trajectory=False,operation_execution_claimed=False))
    return Policy()
