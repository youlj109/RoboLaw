"""CPU-only diagnostic guards; synthetic contacts are never physical evidence."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.bowl_destination_probe import (
    BODY, FLOOR, DECLARATION, DiagnosticDestinationWorld, DestinationNativeContacts, native_offsets,
    read_declaration, release_boundary, support_continuation_checks, unpack_native_contact,
)
from kitchen_safety.extensions.replicator.task_motion import pose_matrix

ROOT = Path(__file__).resolve().parents[3]


def contacts():
    return dict(count=1, exact_body=BODY, exact_filter=FLOOR, points_world_m=[[.5,.5,1.]],
        normals_world=[[0.,0.,1.]], normal_forces_n=[1.], separations_m=[-.0001])


def source():
    return dict(contacts=[dict(point_world_m=[.5,.5,1.], normal_world=[0.,0.,-1.])],
        minimum_source_z_m=.9999, full_bowl_xy_inside_floor=True,
        distance_m=0., intersecting=True, source_gap_m=-.0001)


def test_declared_bounds_are_fixed_and_unknown_offsets_never_become_zero(tmp_path):
    p = tmp_path/'declaration.json'
    p.write_text(json.dumps(DECLARATION))
    assert read_declaration(p) == DECLARATION
    bad = deepcopy(DECLARATION); bad['maximum_release_physics_steps'] = 57
    p.write_text(json.dumps(bad))
    with pytest.raises(ValueError): read_declaration(p)
    class Missing: pass
    result = native_offsets(Missing())
    assert all(not v['available'] and 'values_m' not in v for v in result.values())


def test_complete_native_contact_buffers_and_reject_overflow_missing_identity():
    data = (np.array([1.]), np.array([[1.,2.,3.]]), np.array([[0.,0.,1.]]),
            np.array([-.0001]), np.array([[1]]), np.array([[0]]))
    assert unpack_native_contact(data)['count'] == 1
    for index, replacement in [(4, np.array([[4096]])), (5,np.array([[-1]])),
                               (4,np.array([[1,1]])), (2,np.empty((0,3))), (0,np.array([np.nan]))]:
        bad = list(data); bad[index] = replacement
        with pytest.raises(ValueError): unpack_native_contact(bad)


@pytest.mark.parametrize('bad', ['none','wrong_floor','no_points','no_load','down_force','side_normal','down_normal','negative_load',
    'side_source_normal','outside_native','outside_source','below_floor','outside_bowl'])
def test_release_observation_requires_original_support_and_every_region_guard(bad):
    src, native, force = source(), contacts(), [0.,0.,1.]
    if bad == 'wrong_floor': native['exact_filter'] = '/some/other/floor'
    if bad == 'no_points': native['count']=0; native['points_world_m']=[]; native['normals_world']=[]; native['normal_forces_n']=[]
    if bad == 'no_load': native['normal_forces_n']=[0.]
    if bad == 'down_force': force = [0.,0.,-1.]
    if bad == 'side_normal': native['normals_world']=[[1.,0.,0.]]
    if bad == 'down_normal': native['normals_world']=[[0.,0.,-1.]]
    if bad == 'negative_load': native['normal_forces_n']=[-1.]
    if bad == 'side_source_normal': src['contacts'][0]['normal_world']=[1.,0.,0.]
    if bad == 'outside_native': native['points_world_m']=[[2.,2.,1.]]
    if bad == 'outside_source': src['contacts'][0]['point_world_m']=[2.,2.,1.]
    if bad == 'below_floor': src['minimum_source_z_m']=.5
    if bad == 'outside_bowl': src['full_bowl_xy_inside_floor']=False
    report = support_continuation_checks(src,native,force,[0.,0.,.9],[1.,1.,1.])
    assert report['passed'] is (bad == 'none')


def test_unloaded_points_are_retained_but_cannot_establish_force_direction():
    native=contacts()
    native['count']=2
    native['points_world_m'].append([.6,.6,1.])
    native['normals_world'].append([0.,0.,-1.])
    native['normal_forces_n'].append(0.)
    native['separations_m'].append(0.)
    report=support_continuation_checks(source(),native,[0.,0.,1.],[0.,0.,.9],[1.,1.,1.])
    assert report['passed']
    assert report['native_normal_force_vector_sum_n']==[0.,0.,1.]
    assert report['native_contact_force_semantics']['unloaded_contact_count']==1
    native['normal_forces_n']=[0.,0.]
    report=support_continuation_checks(source(),native,[0.,0.,1.],[0.,0.,.9],[1.,1.,1.])
    assert not report['passed']


def test_native_force_difference_is_recorded_without_inventing_tolerance():
    native=contacts();native['normal_forces_n']=[2.]
    report=support_continuation_checks(source(),native,[0.,0.,1.],[0.,0.,.9],[1.,1.,1.])
    assert report['passed']
    assert report['native_minus_original_force_n']==[0.,0.,1.]
    assert report['native_original_force_difference_norm_n']==1.
    assert report['force_difference_is_diagnostic_only']


def test_every_native_observation_records_signed_vector_sum_and_difference():
    class View:
        def get_contact_data(self, *, dt):
            assert dt == 1/120
            return (np.array([[2.]]),np.array([[.5,.5,1.]]),np.array([[0.,0.,1.]]),
                    np.array([[0.]]),np.array([[1]]),np.array([[0]]))
    recorder=object.__new__(DestinationNativeContacts)
    recorder.view=View();recorder.dt=1/120;recorder.rows=[]
    row=recorder.sample(dict(physics_step=1,phase='release',action_index=0,
        destination_contact_force=np.array([[[0.,0.,1.]]])))
    assert row['native_normal_force_vector_sum_n']==[0.,0.,2.]
    assert row['native_minus_original_force_n']==[0.,0.,1.]
    assert len(recorder.rows)==1


def test_release_requires_completed_plan_current_legal_gap_and_complete_contacts():
    summary = dict(both_fingers_loaded=True,both_fingers_near=True,
        steps=[dict(physics_step=i) for i in range(91,101)])
    src = dict(source(), intersecting=False, distance_m=.004, source_gap_m=.004)
    args = dict(plan_complete=True,xy_error_m=.001,contact_summary=summary,current_step=100)
    assert release_boundary(src,**args)['passed']
    for key, value in [('plan_complete',False),('xy_error_m',.00501),('current_step',101)]:
        bad = deepcopy(args);bad[key]=value
        assert not release_boundary(src,**bad)['passed']
    for gap in [.0019,.0025,.005001]:
        bad = dict(src, distance_m=gap, source_gap_m=gap)
        assert not release_boundary(bad,**args)['passed']
    args['contact_summary']['both_fingers_loaded']=False
    assert not release_boundary(src,**args)['passed']


@pytest.fixture(scope='module')
def recorded_world():
    from scripts.extensions.replicator import audit_bowl_controller_cpu as builder
    episode=ROOT/'datasets/replicator/g_shape_bowl_sink/base/attempt_03'
    metadata=json.loads((episode/'physics_metadata.json').read_text())
    with np.load(episode/'physics_states.npz',allow_pickle=False) as raw:
        data={name:raw[name].copy() for name in raw.files}
    original=builder.BowlCollisionWorld
    try:
        builder.BowlCollisionWorld=DiagnosticDestinationWorld
        world,kin,_=builder.build_world(metadata['source_layout'])
    finally:
        builder.BowlCollisionWorld=original
    return world,kin,metadata,data


def actual_state(fixture, step):
    world,kin,meta,data=fixture
    arms=[meta['joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
    fingers=[meta['joint_names'].index(name) for name in world.GRIPPER_JOINT_NAMES]
    payload=meta['task_object_names'].index('bowl')
    row={name:value[step].copy() for name,value in data.items()}
    row['phase']=str(data['phase'][step]); row['physics_step']=int(data['physics_step'][step])
    world.sync_background(meta['background_paths'],data['background_rigid_poses'][step])
    world.payload_pose=pose_matrix(data['task_object_states'][step,payload,:7])
    world.attachment=None
    world.gripper_joint_positions=data['q'][step,fingers].copy()
    world.gripper_joint_position=float(world.gripper_joint_positions[0])
    world.set_phase(row['phase'])
    world.probe_row=row
    # Zero contacts are factual for old lower states (original native force=0).
    # For release test no raw contacts were recorded: preserve unavailable, and
    # test that the diagnostic refuses to promote the known intersection.
    world.probe_native=dict(contacts(), count=0,points_world_m=[],normals_world=[],normal_forces_n=[],
        separations_m=[],physics_step=step)
    return world,row,data['q'][step,arms],data['q'][step,fingers],world.payload_pose


def test_actual_old_lower_now_fails_at_original_2mm(recorded_world):
    world,row,q,fingers,pose=actual_state(recorded_world,3608)
    assert not world.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
    assert world.last_clearance['required_m']==.002
    assert 0. < world.last_clearance['distance_m'] < .002
    assert world.strict_failure_ledger[-1]['phase']=='lower'


def test_actual_old_contact_missing_raw_native_is_rejected(recorded_world):
    world,row,q,fingers,pose=actual_state(recorded_world,3619)
    world.release_start_step=3616
    assert not world.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
    assert world.last_clearance['intersecting']
    assert world.strict_failure_ledger[-1]['support_continuation']['passed'] is False
    assert world.probe_records[-1]['diagnostic_continuation'] is False


def test_synthetic_native_contact_can_only_continue_declared_pair(recorded_world):
    world,row,q,fingers,pose=actual_state(recorded_world,3619)
    world.release_start_step=3616
    world._probe_measurements.pop(3619,None)
    source_contact=world.source_floor_observation(pose)['contacts'][0]
    # Explicitly synthetic native fixture. Old real episode lacks this data;
    # neither this fixture nor its result is exported as runtime evidence.
    world.probe_native=dict(contacts(),physics_step=3619,
        points_world_m=[source_contact['point_world_m']],normal_forces_n=[6.41877])
    assert world.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
    record=world.probe_records[-1]
    assert record['diagnostic_continuation']
    assert not record['original_strict_measurement']['valid']
    assert not world.strict_failure_ledger[-1]['original_strict_measurement']['valid']

    original_pairs,original_measurement=world._pairs,world._measurement
    added=(0.,'synthetic_other_robot_part','synthetic_other_world',None,None,.002)
    def pairs(*args,**kwargs):
        return sorted([p for p in original_pairs(*args,**kwargs) if p[0]<=0.]+[added],key=lambda p:p[0])
    def measurement(pair):
        if pair is added:
            return dict(valid=False,required_m=.002,robot_part=added[1],obstacle=added[2],
                distance_m=0.,intersecting=True)
        return original_measurement(pair)
    world._pairs,world._measurement=pairs,measurement
    try:
        assert not world.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
        assert world.last_clearance['obstacle']=='synthetic_other_world'
        assert world.last_clearance['required_m']==.002
    finally:
        world._pairs,world._measurement=original_pairs,original_measurement


def test_actual_legal_lower_boundary_remains_safe(recorded_world):
    world,row,q,fingers,pose=actual_state(recorded_world,3592)
    assert world.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)
    src=world.source_floor_observation(pose)
    assert .003 <= src['source_gap_m'] <= .005
    assert src['distance_m'] >= .002


def test_time_bound_cannot_be_extended(recorded_world):
    world,row,q,fingers,pose=actual_state(recorded_world,3619)
    world._probe_measurements.pop(3619,None)
    world.release_start_step=3619-57
    with pytest.raises(ValueError,match='56 physical steps'):
        world.valid(q,mode='execution',gripper_q=fingers,object_pose=pose)


def test_other_pair_never_enters_destination_refinement(recorded_world):
    world,*_=recorded_world
    called=[]
    original=world._exact_measurement
    world._exact_measurement=lambda pair: (called.append(pair) or dict(valid=False,required_m=pair[-1]))
    try:
        world.set_phase('release')
        for label,target in [('robot/gripper',FLOOR),('payload/bowl','/World/envs/env_0/kitchen/Sink/Drain')]:
            result=world._measurement((0.,label,target,None,None,.002))
            assert not result['valid'] and result['required_m']==.002
        assert len(called)==2
    finally:
        world._exact_measurement=original
