import numpy as np
import pytest
from kitchen_safety.extensions.controls.execution_contract import root_pose_match


def test_pinned_backend_root_pose_is_xyzw_not_wxyz():
    pose=[1.,2.,3.,0.,0.,np.sqrt(.5),np.sqrt(.5)]
    assert root_pose_match(pose,pose)['passed']
    reordered=pose[:3]+[pose[6]]+pose[3:6]
    assert not root_pose_match(reordered,pose)['passed']


def test_quaternion_sign_does_not_change_root_orientation():
    a=[1.,2.,3.,0.,0.,0.,1.]
    b=[1.,2.,3.,0.,0.,0.,-1.]
    assert root_pose_match(a,b)['passed']


def test_actual_root_position_difference_is_not_silently_ignored():
    pose=[0.,0.,0.,0.,0.,0.,1.]
    assert not root_pose_match([.001,0.,0.,0.,0.,0.,1.],pose)['passed']
    with pytest.raises(ValueError,match='finite'):
        root_pose_match([np.nan]*7,pose)
