"""CPU tests of the explicitly observed original target support policy."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.bowl_target_support import (
    BODY,FLOOR,MAX_CONTACTS,observed_support_checks,certificate_matches,
    read_policy,release_boundary,unpack_native,
)
from kitchen_safety.extensions.replicator.bowl_source_support_query import digest

ROOT=Path(__file__).resolve().parents[3]


def native():
    return dict(count=1,exact_body=BODY,exact_filter=FLOOR,points_world_m=[[.5,.5,1.]],
        normals_world=[[0,0,1]],normal_forces_n=[1.],separations_m=[-.0001])


def source():
    return dict(full_bowl_xy_inside_floor=True,minimum_source_z_m=.9999,source_gap_m=-.0001,
        intersecting=True,distance_m=0.)


def evaluate(s=None,n=None,force=None):
    return observed_support_checks(s or source(),n or native(),force if force is not None else [0,0,1.],[0,0,.9],[1,1,1.])


def test_source_intersection_and_native_separation_are_retained_not_fitted():
    out=evaluate()
    assert out['passed'] and out['penetration_tolerance_m'] is None
    assert out['original_source_gap_m']==-.0001 and out['native_separations_m']==[-.0001]


def test_force_scalar_sign_is_not_a_force_direction_contract():
    n=native();n['normal_forces_n']=[-1.];n['normals_world']=[[0,0,-1.]]
    assert evaluate(n=n)['passed']
    n['normals_world']=[[0,0,1.]]
    assert not evaluate(n=n)['passed']


def test_original_support_uses_aggregate_cone_not_an_added_per_point_cone():
    n=native();n.update(count=2,points_world_m=[[.4,.5,1.],[.6,.5,1.]],
        normals_world=[[.9,0,.435889894],[ -.9,0,.435889894]],normal_forces_n=[1.,1.],separations_m=[0.,0.])
    assert evaluate(n=n)['passed']
    assert evaluate(n=n)['per_point_upward_cone_required'] is False


def test_zero_load_native_near_contact_is_idempotent_transition_not_support():
    n=native();n['normal_forces_n']=[0.];n['separations_m']=[.0002]
    original=deepcopy(n)
    first=evaluate(n=n,force=[0,0,0]);second=evaluate(n=n,force=[0,0,0])
    assert first==second and n==original
    assert first['passed'] and first['contact_state']=='observed_nonpenetrating_contact_transition'
    assert first['original_loaded_support'] is False and first['nonloaded_transition_is_not_success']
    n['separations_m']=[-.000001]
    assert not evaluate(n=n,force=[0,0,0])['passed']


def test_subthreshold_contact_is_not_rejected_by_the_final_success_force_threshold():
    n=native();n['normal_forces_n']=[.05];n['separations_m']=[0.]
    result=evaluate(n=n,force=[0,0,.05])
    assert result['passed'] and result['contact_state']=='observed_nonpenetrating_contact_transition'


@pytest.mark.parametrize('bad',['wrong_body','wrong_floor','missing','unloaded','down_original','down_native',
    'outside_native','native_below_floor','source_outside','source_below_floor','nan','incomplete'])
def test_unobserved_or_wrong_spatial_support_rejected(bad):
    n,s,f=native(),source(),[0,0,1.]
    if bad=='wrong_body':n['exact_body']='/other'
    if bad=='wrong_floor':n['exact_filter']='/other'
    if bad=='missing':n.update(count=0,points_world_m=[],normals_world=[],normal_forces_n=[],separations_m=[])
    if bad=='unloaded':n['normal_forces_n']=[0.]
    if bad=='down_original':f=[0,0,-1.]
    if bad=='down_native':n['normals_world']=[[0,0,-1.]]
    if bad=='outside_native':n['points_world_m']=[[2,.5,1]]
    if bad=='native_below_floor':n['points_world_m']=[[.5,.5,.8]]
    if bad=='source_outside':s['full_bowl_xy_inside_floor']=False
    if bad=='source_below_floor':s['minimum_source_z_m']=.8
    if bad=='nan':n['separations_m']=[float('nan')]
    if bad=='incomplete':n['count']=2
    assert not evaluate(s=s,n=n,force=f)['passed']


def test_native_buffers_are_copied_and_full_buffers_rejected():
    data=[np.array([[1.]]),np.array([[.5,.5,1.]]),np.array([[0.,0.,1.]]),np.array([[0.]]),np.array([[1]]),np.array([[0]])]
    got=unpack_native(data);data[0][0,0]=123.
    assert got['normal_forces_n']==[1.]
    data[4]=np.array([[MAX_CONTACTS]])
    with pytest.raises(ValueError):unpack_native(data)


def test_lower_boundary_keeps_original_two_mm_and_complete_ten_step_grasp():
    s=source();s.update(intersecting=False,distance_m=.0037,source_gap_m=.0037,minimum_source_z_m=1.0037)
    c=dict(steps=[dict(physics_step=i) for i in range(91,101)],both_fingers_loaded=True,both_fingers_near=True)
    assert release_boundary(s,True,.0016,c,100)['passed']
    for key,value in [('distance_m',.00199),('intersecting',True),('source_gap_m',.0029)]:
        bad=dict(s);bad[key]=value
        assert not release_boundary(bad,True,.0016,c,100)['passed']
    assert not release_boundary(s,False,.0016,c,100)['passed']
    assert not release_boundary(s,True,.0051,c,100)['passed']
    c['steps']=c['steps'][1:]
    assert not release_boundary(s,True,.0016,c,100)['passed']


def certificate():
    value=dict(physics_step=100,payload_pose=np.eye(4).tolist(),floor_geometry_sha256='floor',payload_source_sha256='bowl',
        observed_support=dict(passed=True,contact_state='loaded_support'),native_observation=native())
    value['certificate_sha256']=digest(value)
    return value


@pytest.mark.parametrize('bad',['none','stale','payload_pose','floor','payload_source','attached','edited_native','unloaded_transition'])
def test_retreat_certificate_cannot_exempt_a_moving_or_unobserved_pair(bad):
    cert=certificate();kwargs=dict(physics_step=100,payload_pose=np.eye(4),floor_hash='floor',payload_hash='bowl',attachment=None)
    if bad=='stale':kwargs['physics_step']=101
    if bad=='payload_pose':kwargs['payload_pose'][0,3]=.00001
    if bad=='floor':kwargs['floor_hash']='changed'
    if bad=='payload_source':kwargs['payload_hash']='changed'
    if bad=='attached':kwargs['attachment']=np.eye(4)
    if bad=='edited_native':cert['native_observation']['normal_forces_n']=[0.]
    if bad=='unloaded_transition':
        cert['observed_support']['contact_state']='observed_nonpenetrating_contact_transition'
        cert['certificate_sha256']=digest({k:v for k,v in cert.items() if k!='certificate_sha256'})
    assert certificate_matches(cert,**kwargs) is (bad=='none')


def test_real_policy_lineage_and_original_schedule():
    policy=read_policy(ROOT/'configs/extensions/replicator/g_bowl_target_support.json')
    assert [policy[k] for k in ['release_seconds','retreat_height_m','final_settle_seconds']]==[2.,.15,4.]


def test_all_51_retained_source_intersections_have_observed_original_support():
    episode=ROOT/'reports/replicator_execution_20261023/g_bowl_sink/destination_contact_probe_01'
    native_rows={r['physics_step']:r for r in json.loads((episode/'destination_native_contacts.json').read_text())['rows']}
    rows=json.loads((episode/'destination_source_observations.json').read_text())['rows']
    intersections=[r for r in rows if r['phase']=='release' and r['source']['intersecting']]
    assert len(intersections)==51
    for r in intersections:
        s,n=r['source'],native_rows[r['physics_step']]
        assert observed_support_checks(s,n,n['original_filtered_force_n'],*s['floor_bounds'])['passed']
