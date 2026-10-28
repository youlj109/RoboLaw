import math
import pytest
from kitchen_safety.extensions.replicator.banana_release_schedule_candidate import (
    ReleaseSchedule,original_binary_target,SOURCE_CLOSE_RAD,SOURCE_HARD_LIMITS_RAD,
)

@pytest.mark.parametrize('q,dq',[(0.,0.),(.4354921877384186,-2.003696918487549),(.4352908134460449,-1.),(.7851323485374451,0.),(SOURCE_HARD_LIMITS_RAD[1],-8.)])
def test_explicit_target_schedule_preserves_endpoints_duration_and_bound(q,dq):
    s=ReleaseSchedule(q,dq);r=s.describe();a=r['targets_rad']
    assert len(a)==30 and a[0]<=q and a[14:]==[0.]*16
    assert all(0<=y<=x<=q for x,y in zip(a,a[1:]))
    assert r['release_seconds']==2. and r['physics_steps']==240 and r['open_first_applied_physics_ordinal']==113
    assert r['maximum_target_delta_rad']<=r['interpolating_curve_maximum_speed_rad_s']/15+1e-15
    assert r['initial_actual_dq_rad_s']==dq and r['physical_verified'] is False and r['accepted'] is False
    for i in range(30):assert all(s.target_for_physics_ordinal(i*8+j)==a[i] for j in range(1,9))

@pytest.mark.parametrize('kwargs',[{'initial_actual_q_rad':-1e-9},{'initial_actual_q_rad':.786},{'initial_actual_q_rad':float('nan')},{'initial_actual_dq_rad_s':float('inf')},{'release_seconds':3.},{'ramp_seconds':.5},{'source_open_rad':.001},{'source_closed_rad':.7},{'physics_dt_s':1/60},{'action_dt_s':1/30}])
def test_no_silent_state_clamp_or_changed_duration_source_endpoint(kwargs):
    params=dict(initial_actual_q_rad=.435,initial_actual_dq_rad_s=-2.);params.update(kwargs)
    with pytest.raises(ValueError):ReleaseSchedule(**params)

@pytest.mark.parametrize('value',[.25,.5,.75,-1.,2.,float('nan')])
def test_fractional_binary_input_cannot_be_misrepresented_as_smooth(value):
    with pytest.raises(ValueError):original_binary_target(value)

def test_original_commands_map_to_explicit_physical_targets():
    assert original_binary_target(0.)==0. and original_binary_target(1.)==SOURCE_CLOSE_RAD
    # Native old binary mapping is thresholded, so fractional interpolation is not this candidate.
    assert [SOURCE_CLOSE_RAD if x>.5 else 0. for x in [1.,.75,.5,.25,0.]]==[SOURCE_CLOSE_RAD,SOURCE_CLOSE_RAD,0.,0.,0.]

@pytest.mark.parametrize('index',[-1,30,.5,True])
def test_no_hidden_extra_actions(index):
    with pytest.raises(ValueError):ReleaseSchedule(.4,-2).target_at_action(index)

@pytest.mark.parametrize('ordinal',[0,241,.5,True])
def test_no_hidden_extra_physics_steps(ordinal):
    with pytest.raises(ValueError):ReleaseSchedule(.4,-2).target_for_physics_ordinal(ordinal)
