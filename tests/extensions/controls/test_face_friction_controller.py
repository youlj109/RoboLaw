"""Callback contract tests only; these do not simulate or certify contact."""
import numpy as np
import pytest
from kitchen_safety.extensions.controls.temperature_execution import (
    execute_face_friction, face_friction_pose)


def run_controller(policy, advancing=True):
    config={'initial_roll_deg':-90.,'grasp_tcp_world_m':[0.,0.,0.],
            'target_angle_interior_deg':125.,'friction_terminal_policy':policy}
    current={'phase':'pregrasp','angle':0.,'tcp':face_friction_pose(config,0.,0.)}
    steps=[]
    details={}
    def state():
        return {'tcp':current['tcp'],'q':np.zeros(7),
                'target_joint_position':np.deg2rad(current['angle']),
                'predicate':120.<=current['angle']<150.}
    def step(command,grip=1):
        steps.append((current['phase'],grip))
        if current['phase']=='turn' and advancing:current['angle']=126.
    def pose(pose,grip,repeats=1):
        current['tcp']=pose.copy()
        for _ in range(repeats):step(np.zeros(7),grip)
        return np.zeros(7)
    execute_face_friction(config,state,step,lambda phase:current.update(phase=phase),
                          pose,lambda:np.zeros(7),lambda:2.,details)
    return steps,details


@pytest.mark.parametrize('policy,holds',[('hold_then_unload',8),('immediate_unload',0)])
def test_terminal_policy_removes_driving_hold_only_when_declared(policy,holds):
    steps,details=run_controller(policy)
    assert sum(phase=='hold' for phase,_ in steps)==holds
    assert steps[-1]==('retreat',0)
    assert all(grip==0 for phase,grip in steps if phase in ('release','retreat'))
    assert details['completed'] is True


def test_nonadvancing_contact_stops_instead_of_claiming_success():
    with pytest.raises(RuntimeError,match='stalled'):
        run_controller('immediate_unload',advancing=False)
