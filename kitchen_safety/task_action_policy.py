"""Explicit action schedule adapter; recorded outputs always come from PhysX.

This is a command source, not a simulated state/replay substitute and not an
automatic pick/place/opening policy. Its scene binding must match this attempt.
"""
from pathlib import Path
import hashlib,json
import numpy as np


def create(*,env,contract,bindings,config):
    import torch
    path=Path(config['action_schedule'])
    data=path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=config['sha256']:raise ValueError('Action schedule hash differs')
    schedule=json.loads(data)
    from kitchen_safety.task_contract import digest
    if schedule['task_sha256']!=digest(contract):raise ValueError('Action schedule belongs to another task')
    if schedule['asset_roots']!=bindings['asset_roots']:raise ValueError('Action schedule asset identity differs')
    if schedule['action_hz']!=30:raise ValueError('Action schedule must use the frozen 30Hz clock')
    class Policy:
        def __init__(self):self.index=0
        def act(self,state,last_result):
            if self.index==len(schedule['commands']):return None
            command=schedule['commands'][self.index];self.index+=1
            action=np.asarray(command['action'],dtype=np.float32)
            if action.shape!=(env.unwrapped.action_manager.total_action_dim,) or not np.isfinite(action).all():
                raise ValueError('Invalid actual action dimensions/values')
            return dict(action=torch.as_tensor(action[None],device=env.unwrapped.device),phase=command['phase'],
                operation_id=command.get('operation_id'),evidence=dict(source='explicit command schedule',index=self.index-1,
                    schedule_sha256=config['sha256'],state_authority='subsequent measured PhysX packets only'))
    return Policy()
