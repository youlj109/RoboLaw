"""CPU tests for exact retained lift queries and diagnostic-only attribution."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from scripts.extensions.replicator.diagnose_bowl_fixed_lift import (
    ROOT, candidate_path, lift_boundary, obstacle_groups, placement_mode,
)


def retained():
    episode=ROOT/'datasets/replicator/g_shape_bowl_sink/property/property75_02'
    result=json.loads((episode/'result.json').read_text())
    with np.load(episode/'physics_states.npz') as raw:
        data={k:raw[k] for k in ('physics_step','phase','q')}
    return episode,result,data


def test_actual_rejected_46_waypoint_boundary_and_sha():
    episode,result,data=retained()
    i,n,plan=lift_boundary(result,data)
    assert i==2200 and n==3 and data['phase'][i]=='close'
    path,arrays=candidate_path(episode,n,plan,data['q'][i,:7],data['q'][i,7:])
    assert path.name=='rejected_plan_03_lift.npz'
    assert arrays['positions'].shape==(46,7)


@pytest.mark.parametrize('which',['first_lift','stale_close','executed','partial','sha','q','fingers','target','traversal'])
def test_retained_query_cannot_be_changed(which):
    episode,result,data=retained()
    _,_,plan=lift_boundary(result,data)
    q,fingers=data['q'][2200,:7].copy(),data['q'][2200,7:].copy()
    if which=='first_lift': data['phase'][2200]='lift'
    if which=='stale_close':plan['start_physics_step']=2199
    if which=='executed':plan['rejected_candidate_executed']=True
    if which=='partial':plan['observed_candidate_physics_steps']=1
    if which=='sha':plan['rejected_candidate_sha256']='0'*64
    if which=='q':q[0]+=.001
    if which=='fingers':fingers[0]+=.001
    if which=='target':plan['target_tcp'][0][3]+=.001
    if which=='traversal':plan['rejected_candidate_file']='../rejected_plan_03_lift.npz'
    with pytest.raises(ValueError):
        i,n,plan=lift_boundary(result,data)
        candidate_path(episode,n,plan,q,fingers)


def test_new_layout_must_be_explicit_hypothesis():
    old={'obstacle':{'position':[0,0,0]}}
    new={'obstacle':{'position':[1,0,0]}}
    assert placement_mode(old,old,False)=='actual_recorded_hazard'
    assert placement_mode(new,old,True)=='hypothetical_placement_only'
    with pytest.raises(ValueError):placement_mode(new,old,False)
    with pytest.raises(ValueError):placement_mode(old,old,True)


def test_exhaustive_payload_non_support_split():
    hazards,others=obstacle_groups(['source','hazard_a','hazard_b','kitchen'],'source',['hazard_a','hazard_b'])
    assert hazards==['hazard_a','hazard_b'] and others==['kitchen']


@pytest.mark.parametrize('enabled,support,hazards',[
    (['source','kitchen'],'source',[]), (['source','hazard'],'source',['source']),
    (['source','source','hazard'],'source',['hazard']), (['hazard'],'missing',['hazard'])])
def test_cannot_hide_world_objects(enabled,support,hazards):
    with pytest.raises(ValueError):obstacle_groups(enabled,support,hazards)


def test_successful_baseline_lift_uses_recorded_plan():
    episode=ROOT/'datasets/replicator/g_shape_bowl_sink/base/attempt_04'
    result=json.loads((episode/'result.json').read_text())
    with np.load(episode/'physics_states.npz') as raw:
        data={k:raw[k] for k in ('physics_step','phase','q')}
    i,n,plan=lift_boundary(result,data)
    path,arrays=candidate_path(episode,n,plan,data['q'][i,:7],data['q'][i,7:])
    assert i==2200 and path.name=='plan_03_lift.npz'
    assert np.array_equal(arrays['positions'][0],data['q'][i,:7])
