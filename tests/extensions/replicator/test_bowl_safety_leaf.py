from copy import deepcopy
import json
from pathlib import Path
import tempfile

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.bowl_safety_policy import ROOT,CANONICAL,POLICY_SOURCE,load_layout
from kitchen_safety.extensions.replicator.bowl_safety_recorder import unpack_contact,audit_hazard,resolved_pair,robot_contact_paths


def candidate(variant):
    source=json.loads(CANONICAL.read_text())
    if variant=='human':
        cfg=json.loads(POLICY_SOURCE.read_text())['human']
        obstacle=dict(id='human_default_mpfb',category='human',static=True,usd_path=cfg['asset'],position=[0.,0.,0.],yaw_deg=0.)
    else:
        key='property_vase' if variant=='property' else 'robot_projecting_crossbar'
        obstacle=next(x for x in json.loads((ROOT/f'assets/obstacle_library/{variant}/catalog.json').read_text())['assets']if x['id']==key)
    return dict(source_layout=source,obstacle=obstacle)


def load(value,variant):
    with tempfile.NamedTemporaryFile(mode='w',suffix='.json',dir=ROOT/'reports')as f:
        json.dump(value,f);f.flush();return load_layout(f.name,variant)


@pytest.mark.parametrize('variant',['human','property','robot'])
def test_source_bound_policy(variant):
    layout,policy=load(candidate(variant),variant)
    assert {k:v for k,v in layout.items()if k!='obstacle'}==json.loads(CANONICAL.read_text())
    assert list(layout['objects'])==['bowl']
    assert policy['hazard_asset_files'] and policy['clearances_m']['person']==.04
    if variant=='human':assert any(x['path'].endswith('person.usdc')for x in policy['hazard_asset_files'])


def test_paired_base_uses_exact_original():
    layout,policy=load_layout(CANONICAL,'base')
    assert 'obstacle' not in layout and policy['hazard_asset_files']==[]
    assert policy['transport_object_padding_m_by_variant']['human']==.015


@pytest.mark.parametrize('change',['object','robot','zero_margin','different_asset','dynamic_identity','category'])
def test_reject_changed_source_policy(change):
    c=candidate('property')
    if change=='object':c['source_layout']['objects']['bowl']['position'][0]+=.001
    if change=='robot':c['source_layout']['robot_yaw_deg']+=1.
    if change=='zero_margin':c['planning']={'execution_clearances_m':{'glass':0.}}
    if change=='different_asset':c['obstacle']['usd_path']='assets/safety/glass.usda'
    if change=='dynamic_identity':c['obstacle']['static']=True
    if change=='category':c['obstacle']['category']='human'
    with pytest.raises(ValueError):load(c,'property')


def buffers(force=0.,count=1,start=0):
    return (np.array([[force]]),np.array([[0.,0.,0.]]),np.array([[0.,0.,-1.]]),np.array([[.001]]),np.array([[count]]),np.array([[start]]))


def test_signed_native_vectors_preserved():
    assert unpack_contact(buffers(-1.))['summed_normal_force_n']==[0.,0.,1.]


@pytest.mark.parametrize('count,start',[(2,0),(1,-1),(1,4095)])
def test_truncated_raw_rejected(count,start):
    with pytest.raises(ValueError):unpack_contact(buffers(count=count,start=start))


def synthetic(static=False):
    root='/World/envs/env_0/glass';sensor='/World/envs/env_0/bowl'
    filtered=root+'/mesh' if static else root
    m=dict(static=static,hazard_root=root,collider_paths=[root+'/mesh'],native_sensor_paths=[sensor],
        policy={'asset_inputs':{}},enabled_rigid_body_paths=[]if static else[root],animated_transform_paths=[],
        obstacle={'position':[0.,0.,0.],'yaw_deg':0.})
    m.update(exact_filter_paths=[filtered],contact_pair_ids=[sensor],native_resolved_contact_pairs=[dict(
        pair_id=sensor,sensor_paths=[sensor],filter_paths=[filtered],sensor_count=1,filter_count=1)])
    d=dict(physics_step=np.arange(3),phase=np.array(['reset','warmup','final_settle']),action_index=np.array([-1,0,1]),
        hazard_pose_w=np.tile([0.,0.,0.,0.,0.,0.,1.],(3,1)),hazard_velocity_w=np.zeros((3,6)),
        hazard_colliders_local_transforms=np.tile(np.eye(4),(3,1,1,1)),hazard_is_dynamic=np.full(3,not static),
        hazard_observation_valid=np.ones(3,bool),hazard_sensor_normal_forces_w=np.zeros((3,1,3)),hazard_sensor_contact_counts=np.ones((3,1),int))
    rows=[dict(physics_step=i,phase=str(d['phase'][i]),action_index=int(d['action_index'][i]),exact_hazard=root,
        sensors={sensor:unpack_contact(buffers())})for i in range(3)]
    return d,m,rows


@pytest.mark.parametrize('static',[False,True])
def test_complete_motion_and_contacts(static):
    d,m,r=synthetic(static);assert audit_hazard(d,m,r)['passed']


@pytest.mark.parametrize('change',['missing_step','dynamic_movement','static_movement','tilt','force','missing_contact','fake_static','shape_change','invalid_state'])
def test_hazard_audit_negative(change):
    d,m,r=synthetic(change in ('static_movement','fake_static'))
    if change=='missing_step':r.pop(1)
    if change in ('dynamic_movement','static_movement'):d['hazard_pose_w'][-1,0]=.0021
    if change=='tilt':d['hazard_pose_w'][-1,3:]=[np.sin(.02),0,0,np.cos(.02)]
    if change=='force':r[1]['sensors'][m['native_sensor_paths'][0]]=unpack_contact(buffers(-1.))
    if change=='missing_contact':r[1]['sensors']={}
    if change=='fake_static':m['enabled_rigid_body_paths']=[m['hazard_root']]
    if change=='shape_change':d['hazard_colliders_local_transforms'][-1,0,0,3]=.001
    if change=='invalid_state':d['hazard_observation_valid'][-1]=False
    assert not audit_hazard(d,m,r)['passed']


@pytest.mark.parametrize('filtered',[[],['/wrong'],['/right','/extra']])
def test_static_filter_must_really_resolve(filtered):
    class Fake:
        sensor_count=1;filter_count=1;sensor_paths=['/sensor'];filter_paths=filtered
    with pytest.raises(ValueError):resolved_pair(Fake(),'/sensor','/right')


def test_static_root_filter_does_not_cover_children():
    d,m,r=synthetic(True)
    m['native_resolved_contact_pairs'][0]['filter_paths']=[m['hazard_root']]
    assert not audit_hazard(d,m,r)['passed']


def test_articulation_contact_inventory_uses_native_links():
    from types import SimpleNamespace
    links=['/World/envs/env_0/Robot/panda_link0','/World/envs/env_0/Robot/base_link']
    robot=SimpleNamespace(body_names=['base_link','panda_link0'],root_view=SimpleNamespace(
        count=1,link_paths=[links],shared_metatype=SimpleNamespace(link_names=['panda_link0','base_link'])))
    assert robot_contact_paths(robot)==links
    for changed in ([links[0]], [links[0],links[0]], ['/World/envs/env_1/Robot/panda_link0',links[1]], links[::-1]):
        robot.root_view.link_paths=[changed]
        with pytest.raises(ValueError):robot_contact_paths(robot)
