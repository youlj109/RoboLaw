import json
from pathlib import Path
import numpy as np
import pytest

from scripts.extensions.replicator import paired_bowl_pregrasp_plans as module
_file=Path(module.__file__)


@pytest.fixture
def inputs():
    p=module.ROOT/'datasets/replicator/g_shape_bowl_sink/base/human_matched_01'
    result=json.loads((p/'result.json').read_text())
    with np.load(p/'physics_states.npz') as data:
        raw={k:data[k] for k in ('physics_step','phase')}
    return result,raw


def test_actual_settled_boundary664(inputs):
    result,data=inputs
    index,number,plan=module.pregrasp_boundary(result,data)
    assert index==664 and number==1
    assert plan['measured_planning_attachment'] is None


@pytest.mark.parametrize('which',['attached','first_pregrasp','not_executed','missing','padding','not_settled'])
def test_pregrasp_identity_cannot_change(inputs,which):
    result,data=inputs
    plan=next(p for p in result['plans'] if p['phase']=='pregrasp')
    if which=='attached':plan['measured_planning_attachment']=np.eye(4).tolist()
    if which=='first_pregrasp':plan['start_physics_step']=665
    if which=='not_executed':plan['status']='failed'
    if which=='missing':result['plans']=[p for p in result['plans'] if p['phase']!='pregrasp']
    if which=='padding':plan['validation']['transport_object_padding_m']=.015
    if which=='not_settled':data['phase'][664]='lift'
    with pytest.raises(ValueError):module.pregrasp_boundary(result,data)


def test_pregrasp_entry_cannot_request_transport_or_close_binary():
    import ast
    tree=ast.parse(_file.read_text())
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
        and n.func.attr in ('plan_pose','validate_path')]
    assert len(calls)==2
    for call in calls:
        keywords={k.arg:ast.literal_eval(k.value) for k in call.keywords}
        assert keywords['phase']=='pregrasp' and keywords['grip_binary']==0
