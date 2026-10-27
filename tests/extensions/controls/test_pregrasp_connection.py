"""Reference interpolation tests, not contact or physics evidence."""
import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.controls.temperature_execution import (
    pregrasp_connection, temperature_grasp_rotation, knob_turn_pose, knob_retreat_poses, face_friction_pose,
    temperature_pregrasp_pose)


def test_original_pregrasp_connection_keeps_original_rotation():
    start=np.eye(4)
    start[:3,:3]=Rotation.from_euler('x',-90,degrees=True).as_matrix()
    start[:3,3]=[5.17,-3.22609,1.26588]
    original=start.copy()
    poses=list(pregrasp_connection(start,[5.26,-3.22609,1.26588],-90))
    assert np.array_equal(start,original)
    assert all(np.allclose(p[:3,:3],start[:3,:3]) for p in poses)
    assert np.allclose(poses[-1][:3,3],[5.26,-3.22609,1.26588])


def test_auxiliary_reference_aligns_without_jumping_or_changing_grasp_target():
    start=np.eye(4)
    start[:3,:3]=Rotation.from_euler('x',-165,degrees=True).as_matrix()
    start[:3,3]=[5.17,-3.22609,1.32588]
    poses=[start,*pregrasp_connection(start,[5.26,-3.22609,1.26588],-90)]
    for before,after in zip(poses,poses[1:]):
        assert np.linalg.norm(after[:3,3]-before[:3,3])<=.001+1e-12
        angle=Rotation.from_matrix(after[:3,:3]@before[:3,:3].T).magnitude()
        assert angle<=np.deg2rad(1)+1e-12
    assert np.allclose(poses[-1][:3,3],[5.26,-3.22609,1.26588])
    assert np.allclose(poses[-1][:3,:3],Rotation.from_euler('x',-90,degrees=True).as_matrix())


def test_default_turn_retains_original_reference():
    start=temperature_grasp_rotation({'initial_roll_deg':-90})
    pose=knob_turn_pose([5.26,-3.22609,1.26588],start,135)
    assert np.allclose(pose[:3,:3],Rotation.from_euler('x',45,degrees=True).as_matrix())
    assert np.allclose(pose[:3,3],[5.26,-3.22609,1.26588])


def test_tilted_tool_rotates_about_world_knob_axis():
    start=Rotation.from_euler('y',50,degrees=True).as_matrix()
    config={'grasp_rotation_world':start.tolist()}
    pose=knob_turn_pose([1.,2.,3.],temperature_grasp_rotation(config),90)
    world_rotation=Rotation.from_euler('x',90,degrees=True).as_matrix()
    assert np.allclose(pose[:3,:3],world_rotation@start)
    assert not np.allclose(pose[:3,:3],start@world_rotation)
    assert np.array_equal(pose[:3,3],[1.,2.,3.])


def test_invalid_grasp_matrix_cannot_replace_orientation():
    import pytest
    with pytest.raises(ValueError):
        temperature_grasp_rotation({'grasp_rotation_world':np.zeros((3,3)).tolist()})


def test_upward_withdrawal_preserves_orientation_and_step_bound():
    terminal=np.eye(4)
    terminal[:3,3]=[5.26,-3.22609,1.26588]
    poses=[terminal,*knob_retreat_poses(terminal,[-.06,0.,.06])]
    assert np.allclose(poses[-1][:3,3],[5.2,-3.22609,1.32588])
    assert all(np.array_equal(p[:3,:3],terminal[:3,:3]) for p in poses)
    assert max(np.linalg.norm(b[:3,3]-a[:3,3]) for a,b in zip(poses,poses[1:]))<=.002+1e-12


def test_default_withdrawal_is_original_six_centimeters():
    poses=list(knob_retreat_poses(np.eye(4),[-.06,0.,0.]))
    assert len(poses)==30
    assert np.allclose(poses[-1][:3,3],[-.06,0.,0.])


def test_pregrasp_query_uses_the_same_explicit_matrix_as_execution():
    matrix=Rotation.from_euler('y',45,degrees=True).as_matrix()
    config={'initial_roll_deg':-90.,'pregrasp_tcp_world_m':[1.,2.,3.],
            'pregrasp_rotation_world':matrix.tolist()}
    pose=temperature_pregrasp_pose(config)
    assert np.allclose(pose[:3,:3],matrix)
    assert np.array_equal(pose[:3,3],[1.,2.,3.])


def test_pregrasp_query_retains_declared_roll_override():
    pose=temperature_pregrasp_pose({'initial_roll_deg':-90.,'pregrasp_roll_deg':-165.,
                                   'pregrasp_tcp_world_m':[1.,2.,3.]})
    assert np.allclose(pose[:3,:3],Rotation.from_euler('x',-165,degrees=True).as_matrix())


def test_face_reference_moves_about_original_axis_without_turning_tool():
    config={'initial_roll_deg':-90,'grasp_tcp_world_m':[5.26,-3.22609,1.26588]}
    first=face_friction_pose(config,0,.01)
    quarter=face_friction_pose(config,90,.01)
    assert np.array_equal(first[:3,:3],quarter[:3,:3])
    assert np.allclose(first[:3,3],[5.25,-3.20609,1.26588])
    assert np.allclose(quarter[:3,3],[5.25,-3.22609,1.28588])
    assert config['grasp_tcp_world_m']==[5.26,-3.22609,1.26588]


def test_face_reference_rejects_nonfinite_or_excessive_preload_travel():
    import pytest
    config={'initial_roll_deg':-90,'grasp_tcp_world_m':[0,0,0]}
    for angle,advance in ((np.nan,0),(0,np.nan),(0,.026),(0,-.001)):
        with pytest.raises(ValueError):face_friction_pose(config,angle,advance)


def test_declared_axial_backoff_only_withdraws_without_changing_circle():
    config={'initial_roll_deg':-90,'grasp_tcp_world_m':[0,0,0],
            'friction_min_axial_advance_m':-.01}
    a=face_friction_pose(config,30.,0.)
    b=face_friction_pose(config,30.,-.01)
    assert np.allclose(b[:3,3]-a[:3,3],[-.01,0.,0.])
    assert np.array_equal(a[:3,:3],b[:3,:3])
