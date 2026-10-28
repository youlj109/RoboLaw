"""Meaningful invariants of the cabinet-clearing free retreat, CPU only."""
from types import SimpleNamespace

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.banana_retreat_motion import (
    cartesian_retreat_candidate, retreat_candidates,
)
from kitchen_safety.extensions.replicator.banana_supported_planner import SupportedBananaPlanner


class LinearKin:
    base = np.eye(4)
    base[1,3] = -1.
    def fk(self,q):
        pose=np.eye(4);pose[:3,3]=np.asarray(q)[:3];return pose
    def ik(self,pose,seed,attempts=1):
        q=np.asarray(seed).copy();q[:3]=pose[:3,3];return q


def fixture():
    kin=LinearKin();start=np.zeros(7);start[2]=1.
    requested=kin.fk(start);requested[2,3]+=.15
    return kin,start,requested


def test_fixed_selected_retreat_preserves_rise_and_does_not_move_task_goal():
    kin,start,requested=fixture();before=requested.copy()
    descriptions=retreat_candidates(kin,start,requested)
    assert [d['id'] for d in descriptions]==['out08_lift04','out08_lift06','out12_lift04','out12_lift06','out16_lift04','out16_lift06']
    path=cartesian_retreat_candidate(kin,start,descriptions[0],1/15,.4)
    assert np.array_equal(requested,before)
    assert np.array_equal(path['positions'][0],start)
    assert np.allclose(path['effective_goal_pose'][:3,3],[0.,-.08,1.15])
    assert np.allclose(kin.fk(path['positions'][-1]),path['effective_goal_pose'])
    assert path['metadata']['task_placement_goal_changed'] is False
    assert path['metadata']['collision_thresholds_changed'] is False
    assert np.max(np.abs(np.diff(path['positions'],axis=0)))/(1/15)<=.4+1e-10
    assert path['poses'][:,2,3].min()>=1.-1e-10
    assert np.all(np.diff(path['poses'][:,2,3])>=-1e-10)
    ids=path['metadata']['retimed_command_segment_ids']
    assert len(ids)==len(path['positions']) and set(ids)=={0,1,2}
    assert np.all(np.diff(ids)>=0)
    assert path['metadata']['segment_names']==['initial_up','outward','final_up']


def test_ambiguous_direction_and_short_rise_are_rejected():
    kin,start,requested=fixture();kin.base=kin.base.copy();kin.base[:2,3]=start[:2]
    with pytest.raises(ValueError,match='ambiguous'):retreat_candidates(kin,start,requested)
    kin,start,requested=fixture();requested[2,3]=1.02
    with pytest.raises(ValueError,match='original upward'):retreat_candidates(kin,start,requested)


def test_ik_branch_jump_is_not_smoothed_away():
    kin,start,requested=fixture();description=retreat_candidates(kin,start,requested)[0]
    kin.ik=lambda pose,seed,attempts=1: np.asarray(seed)+1.
    with pytest.raises(RuntimeError,match='IK branch'):
        cartesian_retreat_candidate(kin,start,description,1/15,.4)


def test_new_planner_retains_cuda_rejection(monkeypatch):
    kin,start,requested=fixture();planner=SupportedBananaPlanner.__new__(SupportedBananaPlanner)
    planner.kin=kin;planner.dt=1/15;planner.max_joint_speed=.4
    planner.args=SimpleNamespace(to_device=lambda x:x)
    planner.motion_gen=SimpleNamespace(ik_solver=SimpleNamespace(check_valid=lambda q:np.zeros(len(q),dtype=bool)))
    import kitchen_safety.extensions.replicator.task_plan_diagnostics as diagnostics
    monkeypatch.setattr(diagnostics,'constraint_components',lambda p,q:({'components':{'world_collision':{'nonzero_samples':[3]}}},None))
    with pytest.raises(RuntimeError,match='cuRobo rejected explicit'):
        planner._cartesian_vertical(start,requested,'retreat')
    assert planner.last_cartesian_candidate['metadata']['candidate_id']=='out08_lift04'


def test_retreat_masks_keep_bowl_and_forbid_attachment(monkeypatch):
    from kitchen_safety.curobo_backend import CuroboPlanner
    planner=SupportedBananaPlanner.__new__(SupportedBananaPlanner)
    planner.world=SimpleNamespace(attachment=None,planning_certificate={'observed':True},set_phase=lambda p:None)
    monkeypatch.setattr(CuroboPlanner,'_prepare_path_query',lambda self,masks,grip,phase:masks)
    assert planner._prepare_path_query((),0,'retreat')==('banana',)
    assert planner.candidate_contact_mask['original_destination_cuda_enabled'] is True
    planner.world.attachment=np.eye(4)
    with pytest.raises(ValueError,match='free actual banana'):
        planner._prepare_path_query((),0,'retreat')
