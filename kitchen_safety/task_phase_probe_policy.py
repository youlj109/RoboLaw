"""Measured hold commands for operation-scope diagnostics, never task execution."""
from copy import deepcopy
import math
from kitchen_safety.task_contract import digest


def validate_contexts(contract, config):
    if set(config) != {'schema_version', 'task_sha256', 'contexts'} or config['schema_version'] != 'robolaw-phase-scope-probe-1':
        raise ValueError('Explicit phase-scope probe configuration required')
    if config['task_sha256'] != digest(contract):
        raise ValueError('Probe configuration belongs to another task')
    operations = {o['operation_id'] for o in contract['operations'] if 'articulation_id' in o}
    if not operations:
        raise ValueError('This probe requires a declared articulated operation')
    contexts = config['contexts']
    if not isinstance(contexts, list) or not contexts:
        raise ValueError('Nonempty probe contexts required')
    for row in contexts:
        if not isinstance(row, dict) or set(row) != {'phase', 'operation_id'} or row['phase'] not in ('settle', 'operate'):
            raise ValueError('Unknown diagnostic context')
        if row['operation_id'] is not None and row['operation_id'] not in operations:
            raise ValueError('Probe operation is not declared by this task')
    return deepcopy(contexts)


def create(*, env, contract, bindings, config):
    import torch
    contexts = validate_contexts(contract, config)
    if contract['roles']['robot']['registry_name'] != 'droid_abs_joint_pos':
        raise ValueError('Probe action semantics require the declared DROID absolute-joint profile')
    if env.unwrapped.action_manager.total_action_dim != 8 or abs(env.unwrapped.step_dt - 1 / 30) > 1e-10:
        raise ValueError('Probe requires eight absolute controls at 30Hz')
    names = env.unwrapped.scene['robot'].joint_names
    indices = [names.index('panda_joint' + str(i)) for i in range(1, 8)]
    config_hash = digest(config)

    class Policy:
        def __init__(self):
            self.index = 0

        def act(self, state, last_result):
            if self.index == len(contexts):
                return None
            row = contexts[self.index]
            q = [float(state['robot']['q'][i]) for i in indices]
            if not all(math.isfinite(x) for x in q):
                raise ValueError('Measured command reference is not finite')
            index = self.index
            self.index += 1
            return dict(action=torch.as_tensor([q + [0.]], dtype=torch.float32, device=env.unwrapped.device),
                        phase=row['phase'], operation_id=row['operation_id'],
                        evidence=dict(source='current measured joint hold; diagnostic phase annotation only',
                                      config_sha256=config_hash, context_index=index,
                                      operation_execution_claimed=False, full_task_trajectory=False))
    return Policy()
