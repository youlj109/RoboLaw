import numpy as np
from scipy.spatial.transform import Rotation

from kitchen_safety.extensions.controls.property_motion import property_motion_review


def result():
    return dict(variant='property',physics_dt=1/120,
                initial_state={'hazard_pose':[0.,0.,0.,0.,0.,0.,1.]})


def test_millimeter_settling_is_not_a_topple():
    updates=[dict(hazard_pose=[0.,0.,-.001,0.,0.,0.,1.])]
    assert property_motion_review(result(),updates)['passed']


def test_unforced_setup_topple_is_rejected_and_timed():
    quat=Rotation.from_euler('x',10,degrees=True).as_quat().tolist()
    updates=[dict(phase='setup',action_index=0,hazard_pose=[0.,0.,0.,0.,0.,0.,1.]),
             dict(phase='setup',action_index=0,hazard_pose=[0.,0.,0.,*quat])]
    report=property_motion_review(result(),updates)
    assert not report['passed']
    assert np.isclose(report['maximum_rotation_deg'],10)
    assert report['first_violation']['physics_sample']==1
    assert report['first_violation']['time_from_recorded_start_s']==2/120


def test_missing_property_pose_does_not_pass():
    assert not property_motion_review(result(),[{'hazard_pose':None}])['passed']


def test_non_property_has_no_synthetic_stability_claim():
    report=property_motion_review({'variant':'human'},[])
    assert report['passed'] and not report['applicable']
