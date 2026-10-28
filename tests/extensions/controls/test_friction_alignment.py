import numpy as np
import pytest
from kitchen_safety.extensions.controls.temperature_execution import (
    face_friction_connection, face_friction_pose, pregrasp_connection)
from kitchen_safety.extensions.controls.execution_contract import controller_signature


def config():
    return dict(initial_roll_deg=-90., grasp_tcp_world_m=[5.26,-3.22609,1.26588])


def test_default_connection_is_unchanged():
    cfg=config()
    start=face_friction_pose(cfg,0.,0.)
    start[:3,3]=[5.17,-3.27609,1.36588]
    goal=face_friction_pose(cfg,0.,0.)
    assert np.array_equal(list(face_friction_connection(start,cfg)),
        list(pregrasp_connection(start,goal[:3,3],goal[:3,:3])))


def test_opposite_face_contact_keeps_anchor_axis_radius_and_default():
    cfg=config()
    original=face_friction_pose(cfg,0.,0.)
    cfg['friction_start_angle_deg']=180.
    opposite=face_friction_pose(cfg,0.,0.)
    assert np.array_equal(original[:3,:3],opposite[:3,:3])
    assert opposite[0,3]==original[0,3]
    assert opposite[1,3]==pytest.approx(2*cfg['grasp_tcp_world_m'][1]-original[1,3])
    assert opposite[2,3]==pytest.approx(original[2,3])
    cfg['friction_start_angle_deg']=181.
    with pytest.raises(ValueError):face_friction_pose(cfg,0.,0.)


def test_alignment_keeps_original_anchor_and_bounded_pose_steps():
    cfg=config()
    original=dict(cfg)
    cfg['friction_alignment_world_m']=[5.17,-3.22609,1.32588]
    start=face_friction_pose(cfg,0.,0.)
    start[:3,3]=[5.17,-3.27609,1.36588]
    poses=[start,*face_friction_connection(start,cfg)]
    assert any(np.allclose(p[:3,3],cfg['friction_alignment_world_m']) for p in poses)
    assert np.allclose(poses[-1],face_friction_pose(original,0.,0.))
    assert max(np.linalg.norm(b[:3,3]-a[:3,3]) for a,b in zip(poses,poses[1:]))<=.001000001
    assert controller_signature(cfg)!=controller_signature(original)


def test_two_reference_connection_moves_outward_before_entering_known_contact_ray():
    cfg=config()
    cfg['friction_connection_world_m']=[[5.219,-3.27609,1.36588],[5.219,-3.21209,1.28388]]
    start=face_friction_pose(cfg,0.,0.)
    start[:3,3]=[5.17,-3.27609,1.36588]
    poses=[start,*face_friction_connection(start,cfg)]
    first=next(i for i,p in enumerate(poses) if np.allclose(p[:3,3],cfg['friction_connection_world_m'][0]))
    assert all(np.allclose(p[1:3,3],start[1:3,3]) for p in poses[:first+1])
    assert all(any(np.allclose(p[:3,3],point) for p in poses) for point in cfg['friction_connection_world_m'])
    assert max(np.linalg.norm(b[:3,3]-a[:3,3]) for a,b in zip(poses,poses[1:]))<=.001000001
    assert np.allclose(poses[-1],face_friction_pose(config(),0.,0.))
    assert controller_signature(cfg)!=controller_signature(config())
    cfg['friction_alignment_world_m']=[5.17,-3.22609,1.32588]
    with pytest.raises(ValueError):list(face_friction_connection(start,cfg))


def test_close_reference_is_withdrawn_from_original_anchor_not_offset_tcp():
    cfg=config()
    goal=face_friction_pose(cfg,0.,0.)
    cfg['friction_connection_world_m']=[[5.23,goal[1,3],goal[2,3]]]
    start=goal.copy();start[0,3]=5.17
    poses=list(face_friction_connection(start,cfg))
    assert any(np.allclose(p[:3,3],cfg['friction_connection_world_m'][0]) for p in poses)
    assert max(p[0,3] for p in poses)<=goal[0,3]
    cfg['friction_connection_world_m'][0][0]=5.25
    with pytest.raises(ValueError):list(face_friction_connection(start,cfg))


def test_legacy_alignment_keeps_its_original_stricter_reference_bound():
    cfg=config()
    goal=face_friction_pose(cfg,0.,0.)
    cfg['friction_alignment_world_m']=[5.23,goal[1,3],goal[2,3]]
    with pytest.raises(ValueError):list(face_friction_connection(goal,cfg))


@pytest.mark.parametrize('point', [[5.3,-3.22609,1.26588], [5.1,0.,1.3], [float('nan'),0.,0.], [1.,2.]])
def test_invalid_alignment_fails_closed(point):
    cfg=config()
    cfg['friction_alignment_world_m']=point
    with pytest.raises(ValueError):
        list(face_friction_connection(np.eye(4),cfg))
