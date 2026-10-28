"""CPU-only bounded-diagnostic guards; synthetic contacts never physical evidence."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from kitchen_safety.extensions.replicator.banana_destination_probe import (
    BODY,DESTINATION,DESTINATION_MESH,SOURCE_ASSETS,DECLARATION,LEGACY_DECLARATION,ORIGINAL_COLLECTOR_SHA256,DiagnosticDestinationWorld,
    DestinationNativeContacts,read_declaration,release_boundary,support_continuation_checks,
    unpack_native_contact,native_offsets,audit_declaration_archive,digest_text,triangle_intersection,bounded_transition_checks,
)
from kitchen_safety.extensions.replicator.task_motion import pose_matrix

ROOT=Path(__file__).resolve().parents[3]

def native():
    return dict(count=1,exact_body=BODY,exact_filter=DESTINATION,points_world_m=[[.015,.075,.025]],
        normals_world=[[0.,0.,1.]],normal_forces_n=[1.],separations_m=[.001])

def source():
    return dict(actual_bowl_pose=np.eye(4),actual_spawned_bounds_center_world_m=[0.,0.,.04],
        source_asset_sha256={k:v[1] for k,v in SOURCE_ASSETS.items()},
        contacts=[dict(intersection_points_world_m=[[.015,.075,.025],[.016,.075,.025]],
            fcl_approximate_contact_position_world_m=[999.,999.,999.])])

@pytest.mark.parametrize('bad',['none','wrong_body','wrong_target','missing_native','negative_load','zero_load','down_normal',
    'side_normal','down_force','weak_force','native_outside','native_below','source_outside','source_below','no_clipping','wrong_source','center_outside'])
def test_curved_bowl_support_needs_actual_identity_clipped_source_and_signed_native(bad):
    n,s,force=native(),source(),[0.,0.,1.]
    if bad=='wrong_body':n['exact_body']='/other'
    if bad=='wrong_target':n['exact_filter']=DESTINATION_MESH
    if bad=='missing_native':n.update(count=0,points_world_m=[],normals_world=[],normal_forces_n=[],separations_m=[])
    if bad=='negative_load':n['normal_forces_n']=[-1.]
    if bad=='zero_load':n['normal_forces_n']=[0.]
    if bad=='down_normal':n['normals_world']=[[0.,0.,-1.]]
    if bad=='side_normal':n['normals_world']=[[1.,0.,0.]]
    if bad=='down_force':force=[0.,0.,-1.]
    if bad=='weak_force':force=[0.,0.,.099]
    if bad=='native_outside':n['points_world_m']=[[.09,0.,.025]]
    if bad=='native_below':n['points_world_m']=[[0.,0.,-.028]]
    if bad=='source_outside':s['contacts'][0]['intersection_points_world_m']=[[.09,0.,.025]]
    if bad=='source_below':s['contacts'][0]['intersection_points_world_m']=[[0.,0.,-.028]]
    if bad=='no_clipping':s['contacts'][0]['intersection_points_world_m']=[]
    if bad=='wrong_source':s['source_asset_sha256']['bowl']='0'*64
    if bad=='center_outside':s['actual_spawned_bounds_center_world_m']=[.09,0.,.04]
    r=support_continuation_checks(s,n,force,[-.081,-.081,-.0275],[.081,.081,.0275])
    assert r['passed'] is (bad=='none')
    # Approximate FCL.pos is deliberately outside; only reconstructed geometry is used.

def test_zero_load_geometry_kept_without_force_direction_and_nonplanar_bowl_frame():
    n=native();n.update(count=2);n['points_world_m'].append([.01,.07,.025]);n['normals_world'].append([0,0,-1])
    n['normal_forces_n'].append(0.);n['separations_m'].append(-.00001)
    assert support_continuation_checks(source(),n,[0,0,1],[-.081,-.081,-.0275],[.081,.081,.0275])['passed']
    n['normal_forces_n']=[1.,1.]
    assert not support_continuation_checks(source(),n,[0,0,1],[-.081,-.081,-.0275],[.081,.081,.0275])['passed']

@pytest.mark.parametrize('loads,normals,expected',[
    ([-1.],[[0,0,-1]],True),  # signed factors make an upward force, as observed
    ([-1.],[[0,0,1]],False),  # abs(scalar) would falsely turn downward into support
    ([1.,1.],[[.9,0,.4],[-.9,0,.4]],True),  # curved rim: individual normals outside cone
    ([1.,-1.],[[0,0,1],[0,0,1]],False),  # cancellation cannot establish support
    ([1.,1.],[[1,0,0],[0,0,.01]],False),
    ([.099],[[0,0,1]],False),
    ([.1],[[0,0,1]],True),
    ([1.],[[0,0,0]],False),
])
def test_native_signed_vector_aggregate_is_authoritative(loads,normals,expected):
    n=native();n.update(count=len(loads),normal_forces_n=loads,normals_world=normals,
        points_world_m=n['points_world_m']*len(loads),separations_m=n['separations_m']*len(loads))
    review=support_continuation_checks(source(),n,[0,0,1],[-.081,-.081,-.0275],[.081,.081,.0275])
    assert review['passed'] is expected
    assert np.array_equal(review['native_signed_contact_force_vectors_n'],np.array(loads)[:,None]*normals)
    assert review['raw_scalar_sign_counts']['negative']==int((np.array(loads)<0).sum())
    assert review['force_difference_is_diagnostic_only'] is True

def test_actual_probe01_signed_native_is_counterfactual_continuation_only():
    p=ROOT/'reports/replicator_execution_20261023/g_banana/destination_contact_probe_01'
    native_row=json.loads((p/'destination_native_contacts.json').read_text())['rows'][-1]
    saved=json.loads((p/'destination_source_observations.json').read_text())['rows'][-1]
    bounds=json.loads((p/'task_predicate_metadata.json').read_text())['actual_spawned_local_bounds']['bowl']
    original=json.loads((p/'result.json').read_text())
    assert native_row['physics_step']==3925 and saved['diagnostic_continuation'] is False
    review=support_continuation_checks(saved['source'],native_row,native_row['original_filtered_force_n'],bounds['min'],bounds['max'])
    assert review['passed'] and review['raw_scalar_sign_counts']['negative']==2
    assert review['checks']['native_aggregate_force_in_upward_cone']
    assert original['accepted'] is False and original['physical_trial_passed'] is False

@pytest.mark.parametrize('case,expected',[
    ('loaded',True),('complete_empty',True),('complete_zero',True),('side_force',True),('down_force',True),
    ('missing_fields',False),('wrong_identity',False),('incomplete',False),('nonfinite',False),('capacity',False),
    ('native_outside_zero_force',False),('native_below_zero_force',False),('source_outside',False),('source_no_clip',False),
    ('source_inventory_mismatch',False),('center_outside',False),('wrong_hash',False),('nonintersecting_empty',True),
])
def test_bounded_transition_observation_not_instantaneous_rest(case,expected):
    n=native();s=source();s['intersecting']=True;force=[0,0,1]
    if case in ('complete_empty','nonintersecting_empty'):n.update(count=0,points_world_m=[],normals_world=[],normal_forces_n=[],separations_m=[]);force=[0,0,0]
    if case=='nonintersecting_empty':s.update(intersecting=False,contacts=[])
    if case=='complete_zero':n['normal_forces_n']=[0.];force=[0,0,0]
    if case=='side_force':n['normals_world']=[[1,0,0]];force=[1,0,0]
    if case=='down_force':n['normal_forces_n']=[-1.];force=[0,0,-1]
    if case=='missing_fields':n.pop('normals_world')
    if case=='wrong_identity':n['exact_filter']='/another_target'
    if case=='incomplete':n['count']=2
    if case=='nonfinite':n['normal_forces_n']=[float('nan')]
    if case=='capacity':n['count']=4096
    if case=='native_outside_zero_force':n.update(points_world_m=[[.09,0,.025]],normal_forces_n=[0.])
    if case=='native_below_zero_force':n.update(points_world_m=[[0,0,-.028]],normal_forces_n=[0.])
    if case=='source_outside':s['contacts'][0]['intersection_points_world_m']=[[.09,0,.025]]
    if case=='source_no_clip':s['contacts'][0]['intersection_points_world_m']=[]
    if case=='source_inventory_mismatch':s['intersecting']=False
    if case=='center_outside':s['actual_spawned_bounds_center_world_m']=[.09,0,.04]
    if case=='wrong_hash':s['source_asset_sha256']['banana']='0'*64
    if case=='missing_fields':
        with pytest.raises(KeyError):bounded_transition_checks(s,n,force,[-.081,-.081,-.0275],[.081,.081,.0275])
        return
    result=bounded_transition_checks(s,n,force,[-.081,-.081,-.0275],[.081,.081,.0275])
    assert result['passed'] is expected
    assert result['accepted'] is False and result['task_success'] is False and result['physical_trial_passed'] is False
    assert result['instantaneous_support_required_for_observation'] is False

def test_actual_probe02_unloaded_transient_is_observation_not_success():
    p=ROOT/'reports/replicator_execution_20261023/g_banana/destination_contact_probe_02'
    n=json.loads((p/'destination_native_contacts.json').read_text())['rows'][-1]
    s=json.loads((p/'destination_source_observations.json').read_text())['rows'][-1]
    bounds=json.loads((p/'task_predicate_metadata.json').read_text())['actual_spawned_local_bounds']['bowl']
    result=bounded_transition_checks(s['source'],n,n['original_filtered_force_n'],bounds['min'],bounds['max'])
    assert n['physics_step']==3938 and n['count']==92 and not any(n['normal_forces_n'])
    assert result['passed'] and not result['instantaneous_support_diagnostic']['passed']
    assert result['observation_state']=='reported_contacts_zero_vector_sum'
    assert json.loads((p/'result.json').read_text())['accepted'] is False

def test_buffers_missing_offsets_and_declaration_ambiguity(tmp_path):
    data=(np.array([[1.]]),np.array([[0.,0.,0.]]),np.array([[0,0,1]]),np.array([[.001]]),np.array([[1]]),np.array([[0]]))
    assert unpack_native_contact(data)['count']==1
    for i,v in [(4,np.array([[4096]])),(5,np.array([[-1]])),(4,np.array([[1,1]])),(2,np.empty((0,3))),(0,np.array([[np.nan]]))]:
        bad=list(data);bad[i]=v
        with pytest.raises(ValueError):unpack_native_contact(bad)
    assert all(not v['available'] for v in native_offsets(object()).values())
    class View:
        def get_rest_offsets(self):return np.zeros((1,256))
        def get_contact_offsets(self):return np.ones((1,256))*.001
    assert all(v['actual_shape_slots_verified'] is False for v in native_offsets(View()).values())
    p=tmp_path/'declaration.json';p.write_text(json.dumps(DECLARATION));assert read_declaration(p)==DECLARATION
    bad=deepcopy(DECLARATION);bad['maximum_release_physics_steps']=57;p.write_text(json.dumps(bad))
    with pytest.raises(ValueError):read_declaration(p)
    text=json.dumps(DECLARATION);h=digest_text(text);module='kitchen_safety/extensions/replicator/banana_destination_probe.py'
    module_text=(ROOT/module).read_text();mh=digest_text(module_text)
    binding=dict(path='decl',sha256=h,module_path=module,module_sha256=mh)
    report=dict(entries={'decl':dict(kind='diagnostic_declaration_input',initial_sha256=h,checked_sha256=h,unchanged=True,source_text=text)})
    report['entries'][module]=dict(kind='execution_source',initial_sha256=mh,checked_sha256=mh,unchanged=True,source_text=module_text)
    report['entries']['scripts/extensions/replicator/task_collect.py']=dict(initial_sha256=ORIGINAL_COLLECTOR_SHA256,checked_sha256=ORIGINAL_COLLECTOR_SHA256)
    assert audit_declaration_archive(report,binding)['passed']
    legacy=deepcopy(report);legacy_text=json.dumps(LEGACY_DECLARATION);legacy_hash=digest_text(legacy_text)
    legacy['entries']['decl'].update(source_text=legacy_text,initial_sha256=legacy_hash,checked_sha256=legacy_hash)
    assert audit_declaration_archive(legacy,dict(binding,sha256=legacy_hash))['passed']
    wrong=deepcopy(report);wrong['entries'][module]['source_text']+='changed'
    with pytest.raises(ValueError):audit_declaration_archive(wrong,binding)
    report['entries']['decl']['kind']='controller_input'
    with pytest.raises(ValueError):audit_declaration_archive(report,binding)

def test_triangle_clipping_not_approximate_fcl_position():
    a=np.array([[0,0,0],[1,0,0],[0,1,0.]])
    b=np.array([[.2,.2,-1],[.2,.2,1],[.2,.8,0.]])
    points=triangle_intersection(a,b)
    assert len(points)==2 and np.allclose(points[:,0],.2) and np.allclose(points[:,2],0)

@pytest.fixture(scope='module')
def recorded_world():
    from kitchen_safety.extensions.replicator import task_collision
    from scripts.extensions.replicator.search_banana_grasp_stability_cpu import observed_world
    p=ROOT/'datasets/replicator/g_shape_banana_bowl/base/attempt_06'
    original=task_collision.PayloadCollisionWorld
    try:
        task_collision.PayloadCollisionWorld=DiagnosticDestinationWorld
        world,kin,_,_,meta,result,_=observed_world(p)
    finally:task_collision.PayloadCollisionWorld=original
    data=dict(np.load(p/'physics_states.npz',allow_pickle=False))
    world.bind_spawned_bounds(json.loads((p/'task_predicate_metadata.json').read_text())['actual_spawned_local_bounds'])
    return world,meta,data,result,p

def actual(fixture,step):
    world,m,d,_,_=fixture
    arms=[m['joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
    fingers=[m['joint_names'].index(n) for n in world.GRIPPER_JOINT_NAMES]
    row={k:v[step].copy() for k,v in d.items()};row['phase']=str(d['phase'][step]);row['physics_step']=int(d['physics_step'][step])
    world.sync_background(m['background_paths'],d['background_rigid_poses'][step])
    for i,name in enumerate(m['task_object_names']):world.set_obstacle_pose(name,d['task_object_states'][step,i,:7])
    world.probe_bowl_pose=d['task_object_states'][step,1,:7].copy();world.payload_pose=pose_matrix(d['task_object_states'][step,0,:7])
    world.attachment=None;world.gripper_joint_positions=d['q'][step,fingers].copy();world.gripper_joint_position=float(world.gripper_joint_positions[0])
    world.set_phase(row['phase']);world.probe_row=row
    world.probe_native=dict(native(),count=0,points_world_m=[],normals_world=[],normal_forces_n=[],separations_m=[],physics_step=step)
    world._probe_measurements.pop(step,None)
    return world,row,d['q'][step,arms],d['q'][step,fingers],world.payload_pose

def test_actual_plan_endpoint_legal_and_old_hold_violation(recorded_world):
    from kitchen_safety.extensions.replicator.task_contacts import ObjectGraspContacts
    w,m,d,r,p=recorded_world
    plan=np.load(p/'plan_05_lower.npz')['positions'];end=3552+(len(plan)-1)*8
    assert end==3920
    w,row,q,fingers,pose=actual(recorded_world,end)
    passed=w.valid(q,mode='execution',gripper_q=fingers,object_pose=pose);assert passed
    observation=w.source_bowl_observation(pose)
    contacts=object.__new__(ObjectGraspContacts);contacts.records=json.loads((p/'grasp_contacts.json').read_text())[:end+1];contacts.object_name='banana'
    summary=contacts.summarize(*r['grasp_target']['contact_region_local_m'],phase='lower',last_steps=10)
    kwargs=dict(plan_complete=True,contact_summary=summary,current_step=end,reset_step=0,original_other_checks_passed=passed)
    assert release_boundary(observation,**kwargs)['passed']
    for key,value in [('plan_complete',False),('current_step',3921),('original_other_checks_passed',False)]:
        bad=dict(kwargs);bad[key]=value;assert not release_boundary(observation,**bad)['passed']
    for gap in [.0029,.005001]:
        assert not release_boundary(dict(observation,distance_m=gap),**kwargs)['passed']
    w,row,q,fingers,pose=actual(recorded_world,3953)
    assert not w.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
    assert w.last_clearance['required_m']==.002 and 0<w.last_clearance['distance_m']<.002

def test_actual_old_rim_intersection_does_not_infer_missing_native_as_empty(recorded_world):
    w,row,q,fingers,pose=actual(recorded_world,3964);w.release_start_step=3963;w.probe_native=None
    with pytest.raises(ValueError,match='Missing aligned native'):w.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
    assert len(w.source_bowl_observation(pose)['contacts'])==48

def test_synthetic_native_only_declared_pair_and_time_limit(recorded_world):
    w,row,q,fingers,pose=actual(recorded_world,3964);w.release_start_step=3963
    w.probe_records=[]
    observation=w.source_bowl_observation(pose);point=observation['contacts'][0]['intersection_points_world_m'][0]
    w.probe_native=dict(native(),physics_step=3964,points_world_m=[point],normal_forces_n=[1.2187525])
    assert w.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
    assert w.probe_records[-1]['diagnostic_continuation'] and not w.strict_failure_ledger[-1]['original_strict_measurement']['valid']
    w._probe_measurements.pop(3964,None);w.release_start_step=3964-57
    with pytest.raises(ValueError,match='56 actual'):w.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
    called=[];original=w._exact_measurement
    w._exact_measurement=lambda pair:(called.append(pair) or dict(valid=False,required_m=pair[-1]))
    try:
        for a,b in [('robot/gripper',DESTINATION_MESH),('payload/banana','/some_other_object')]:
            answer=w._measurement((0.,a,b,None,None,.002));assert not answer['valid'] and answer['required_m']==.002
        assert len(called)==2
    finally:w._exact_measurement=original

def test_release_physics_gap_rejected_even_with_complete_native_empty(recorded_world):
    w,row,q,fingers,pose=actual(recorded_world,3964);w.release_start_step=3960;w.probe_records=[]
    with pytest.raises(ValueError,match='gap/duplicate'):w.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)


def test_full_world_still_rejects_another_pair_after_diagnostic_contact(recorded_world):
    w,row,q,fingers,pose=actual(recorded_world,3964);w.release_start_step=3963
    w.probe_records=[]
    point=w.source_bowl_observation(pose)['contacts'][0]['intersection_points_world_m'][0]
    w.probe_native=dict(native(),physics_step=3964,points_world_m=[point],normal_forces_n=[1.2187525])
    original_pairs,original_measurement=w._pairs,w._measurement
    added=(0.,'synthetic_other_robot_link','synthetic_other_obstacle',None,None,.002)
    def pairs(*args,**kwargs):
        return sorted([p for p in original_pairs(*args,**kwargs) if p[0]<=0.]+[added],key=lambda p:p[0])
    def measurement(pair):
        if pair is added:return dict(valid=False,required_m=.002,robot_part=added[1],obstacle=added[2],distance_m=0.,intersecting=True)
        return original_measurement(pair)
    w._pairs,w._measurement=pairs,measurement
    try:
        assert not w.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
        assert w.last_clearance['obstacle']=='synthetic_other_obstacle' and w.last_clearance['required_m']==.002
    finally:w._pairs,w._measurement=original_pairs,original_measurement
