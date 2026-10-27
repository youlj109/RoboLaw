"""Geometry dispatch tests; real execution always supplies measured joints."""
import numpy as np
import pytest

from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.extensions.controls.collision import ControlsCollisionWorld, held_gripper_pose


def stub(monkeypatch):
    calls=[]
    def shapes(self,q,gripper_q=None,object_pose=None,*,_cache_robot=False):
        calls.append(gripper_q)
        return [('finger',None,None,None),('robot_hull',None,None,None)]
    monkeypatch.setattr(CollisionWorld,'shapes',shapes)
    return ControlsCollisionWorld.__new__(ControlsCollisionWorld),calls


def test_default_retains_unknown_full_stroke_policy(monkeypatch):
    world,calls=stub(monkeypatch)
    result=world.shapes(np.zeros(7))
    assert calls==[None]
    assert [row[0] for row in result]==['robot_hull']


def test_declared_held_pose_is_used_only_without_measured_fingers(monkeypatch):
    world,calls=stub(monkeypatch)
    held=dict(zip(world.GRIPPER_JOINT_NAMES,[.7,.7,-.7,.7,-.7,-.7]))
    world.planning_gripper_joint_positions=held
    world.shapes(np.zeros(7))
    assert calls[-1] is held
    actual={name:0. for name in world.GRIPPER_JOINT_NAMES}
    world.shapes(np.zeros(7),gripper_q=actual)
    assert calls[-1] is actual


def test_extended_measured_joint_vector_is_not_replaced(monkeypatch):
    world,calls=stub(monkeypatch)
    world.planning_gripper_joint_positions={'finger_joint':.7}
    world.shapes(np.zeros(13))
    assert calls[-1] is None


def test_held_pose_capture_preserves_measured_values():
    measured=np.pi/4*np.array([1,1,-1,1,-1,-1])
    measured[2]+=1e-5
    actual=held_gripper_pose(list(CollisionWorld.GRIPPER_JOINT_NAMES),measured,np.zeros(6),np.pi/4)
    assert list(actual.values())==measured.tolist()


@pytest.mark.parametrize('kind',['moving','nan_velocity','wrong_opening'])
def test_unsettled_pose_cannot_remove_full_stroke_proxy(kind):
    measured=np.pi/4*np.array([1,1,-1,1,-1,-1])
    velocity=np.zeros(6)
    if kind=='moving':velocity[0]=.1
    if kind=='nan_velocity':velocity[0]=float('nan')
    if kind=='wrong_opening':measured*=0
    with pytest.raises(ValueError):
        held_gripper_pose(list(CollisionWorld.GRIPPER_JOINT_NAMES),measured,velocity,np.pi/4)
