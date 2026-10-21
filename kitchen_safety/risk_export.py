"""Causal risk labels at original action timestamps; GT labels stay outside observations."""
from pathlib import Path
import numpy as np
from kitchen_safety.risk_reporting import lines, read_json
from kitchen_safety.risk_observer import CATEGORIES


def risk_arrays(source,count,profile_index):
    source=Path(source)
    output={'risk.level':np.full((count,4),-1,dtype=np.int64),
        'risk.valid':np.zeros((count,4),dtype=np.int64),
        'risk.data_complete':np.zeros((count,4),dtype=np.int64),
        'risk.observation_id':np.full(count,-1,dtype=np.int64),
        'risk.source_physics_step':np.full(count,-1,dtype=np.int64),
        'risk.profile_index':np.full(count,-1,dtype=np.int64),
        'result.axes':np.full((count,4),-1,dtype=np.int64)}
    binding=read_json(source/'safety_observer_binding.json')
    if binding is None:return output
    records=list(lines(source/'safety_decisions.jsonl'))
    decimation=round(binding['action_dt_s']/binding['physics_dt_s'])
    by_step={r['decision_physics_step']:r for r in records}
    for action in range(count):
        source_step=action*decimation
        # Use only the decision available at the beginning of this action.
        # Later slots in the same action stay in raw evidence, never shifted back.
        slot=(source_step//binding['decision_stride'])*binding['decision_stride']
        r=by_step.get(slot)
        if r is None:raise ValueError(f'No causal risk slot for action {action}')
        if r['source_physics_step']>=source_step:raise ValueError('Risk input uses future physical state')
        for j,cat in enumerate(CATEGORIES):
            value=r['report']['categories'][cat]
            level=value['level']
            output['risk.level'][action,j]=-1 if level is None else int(level[1:])
            output['risk.valid'][action,j]=int(level is not None)
            output['risk.data_complete'][action,j]=int(value['data_complete'])
        output['risk.observation_id'][action]=r['observation_id']
        output['risk.source_physics_step'][action]=r['source_physics_step']
        output['risk.profile_index'][action]=profile_index
    axes=read_json(source/'result_axes.json',{})
    state=axes.get('obstruction',{}).get('status')
    safe=axes.get('safety',{})
    safety_value=(all(v['known_max']=='L0' for v in safe['risk'].values()) and safe.get('legacy_clearance_passed') is True) if safe.get('risk_complete') else None
    data_axis=axes.get('data_integrity',{})
    values=[axes.get('task',{}).get('legacy_success'),data_axis.get('combined_passed',data_axis.get('legacy_passed')),
            safety_value, True if state=='confirmed' else False if state=='unconfirmed' else None]
    output['result.axes'][:]=[-1 if v is None else int(v) for v in values]
    return output
