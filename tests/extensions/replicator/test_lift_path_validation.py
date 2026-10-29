"""Regression for low-altitude lateral dragging and wrist rotation during lift."""
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator import lift_path_validation as gate
from kitchen_safety.extensions.replicator import physical_retry_round2 as planner
from tests.extensions.replicator.test_retry_segment_planning import Kin, FakePlanner

LIMITS=dict(reference_phase='approach',max_xy_deviation_m=.008,max_orientation_deviation_deg=4.,min_z_offset_m=-.002)


def curve():
    poses=np.repeat(np.eye(4)[None],4,axis=0)
    poses[:,2,3]=[0,.01,.06,.152]
    return poses


@pytest.mark.parametrize('failure',['lateral','rotation','drop'])
def test_rejects_defective_curve_even_with_valid_endpoints(failure):
    poses=curve()
    if failure=='lateral': poses[1,0,3]=.041
    if failure=='rotation': poses[1,:3,:3]=Rotation.from_euler('y',14,degrees=True).as_matrix()
    if failure=='drop': poses[1,2,3]=-.003
    report=gate.measure_lift(poses,np.eye(4),LIMITS)
    assert not report['passed']
    assert np.isclose(report['final_z_offset_m'],.152)


def test_rigid_world_translation_and_yaw_preserve_corridor():
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',87,degrees=True).as_matrix();T[:3,3]=[2,-.1,.98]
    report=gate.measure_lift(T@curve(),T,LIMITS)
    assert report['passed'] and report['source_samples']==4
    assert report['max_xy_deviation_m']<1e-12


def test_numpy_angle_matches_source_rotation_without_loading_scipy_in_preflight():
    for degrees in (0.,1e-7,2.103441906,4.,21.214617743,179.999999,180.):
        poses=curve();poses[1,:3,:3]=Rotation.from_rotvec(np.array([1.,2.,3.])/np.sqrt(14)*np.deg2rad(degrees)).as_matrix()
        actual=gate.measure_lift(poses,np.eye(4),LIMITS)['max_orientation_deviation_deg']
        assert np.isclose(actual,degrees,atol=1e-10,rtol=0)
    import subprocess,sys
    completed=subprocess.run([sys.executable,'-c',
        "import sys; from kitchen_safety.extensions.replicator import lift_path_validation; assert not any(x=='scipy' or x.startswith('scipy.') for x in sys.modules)"],capture_output=True,text=True)
    assert completed.returncode==0,completed.stderr


def test_retimed_samples_are_checked_in_addition_to_source_vertices():
    poses=curve();poses[1,1,3]=.009
    data=dict(tcp_world=poses,phase=np.array(['approach','lift','lift','lift']))
    assert gate.array_lift_gate(data,dict(lift_path_validation=LIMITS))['passed'] is False


def test_rejected_solver_path_is_preserved_and_not_retried():
    target=np.eye(4);target[2,3]=.06
    recipe=dict(lift_path_validation=LIMITS,hold_orientation_phases=['lift'],linear_motion_phases=['lift'])
    with TemporaryDirectory() as name,patch.object(planner,'relative',lambda p:str(p)),patch.object(planner,'fk_batch',lambda k,q:np.array([k.fk(x) for x in q])):
        p=FakePlanner(.041)
        path,report=planner.plan_phase(p,Kin(),np.zeros(7),target,'lift',[],recipe,Path(name),8)
        assert path is None and p.calls==1
        assert report['solver_status']=='LIFT_PATH_CONSTRAINT_LIMIT'
        assert report['solver_status_before_gate']=='SUCCESS'
        assert report['lift_path_validation']['passed'] is False
        with np.load(Path(name)/'lift_segment_00.npz') as data:
            assert len(data['q'])==3 and data['tcp_world'][1,1,3]==.041


@pytest.mark.parametrize('bad',[float('nan'),True,-1.,.03])
def test_invalid_limit_cannot_disable_gate(bad):
    limits=deepcopy(LIMITS);limits['max_xy_deviation_m']=bad
    with pytest.raises(ValueError):gate.checked_limits(dict(lift_path_validation=limits))
