import numpy as np
import pytest

from kitchen_safety.extensions.controls.planning import plan_control_pose,plan_prepress


class Planner:
    def __init__(self):
        self.kin=self
        self.calls=[]

    def ik(self,target,seed,attempts):
        self.calls.append(('ik',target.copy(),seed.copy(),attempts))
        return seed+.001

    def plan_joint_goal(self,q,goal,**kwargs):
        self.calls.append(('joint',q.copy(),goal.copy(),kwargs))
        return {'positions':np.array([q,goal]),'metadata':{}}

    def plan_pose(self,q,target,**kwargs):
        self.calls.append(('pose',q.copy(),target.copy(),kwargs))
        return {'positions':np.array([q,q]),'metadata':{}}


@pytest.mark.parametrize('grip,phase',[(0,'pregrasp'),(1,'prepress')])
def test_fixed_ik_preserves_goal_policy_and_phase(grip,phase):
    planner=Planner();target=np.eye(4);target[:3,3]=[5.17,-3.22609,1.26588]
    policy={'method':'curobo_joint_goal','ik_branch_seed':[.1]*7}
    plan=plan_control_pose(planner,np.zeros(7),target,policy,grip_binary=grip,phase=phase)
    assert planner.calls[0][0]=='ik'
    assert np.array_equal(planner.calls[0][1],target)
    assert planner.calls[0][3]==1
    assert planner.calls[1][3]=={'grip_binary':grip,'phase':phase}
    assert plan['metadata']['prepress_policy']==policy
    assert plan['metadata']['requested_tcp_pose']==target.tolist()


def test_original_button_default_remains_closed_prepress():
    planner=Planner()
    plan_prepress(planner,np.zeros(7),np.eye(4))
    assert planner.calls[0][0]=='pose'
    assert planner.calls[0][3]=={'grip_binary':1,'phase':'prepress'}
