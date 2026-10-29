"""Reject excessive actual TCP curvature before execution, without retrying."""
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from kitchen_safety.extensions.replicator import physical_retry_round2 as planner

class Kin:
    def fk(self,q):
        pose=np.eye(4);pose[:3,3]=q[:3];return pose

class FakePlanner:
    def __init__(self,offset):self.offset=offset;self.calls=0
    def constrained_plan(self,q,target,*args,**kw):
        self.calls+=1;end=q.copy();end[:3]=target[:3,3];mid=(q+end)/2;mid[1]+=self.offset
        return np.array([q,mid,end]),dict(status='generated',solver_status='SUCCESS',solver_success=True,samples=3)

def test_declared_segments_preserve_vertices_and_rejected_path_without_retry():
    a=np.eye(4);a[0,3]=.02;b=a.copy();b[0,3]=.04
    recipe=dict(phase_waypoints_world={'lift':[a.tolist()]},segment_deviation_limits_m={'lift':.005})
    with tempfile.TemporaryDirectory() as temp,patch.object(planner,'ROOT',Path(temp)),patch.object(planner,'relative',lambda p:str(p)),patch.object(planner,'fk_batch',lambda k,q:np.array([k.fk(x) for x in q])):
        for offset,expected in [(0,True),(.01,False)]:
            folder=Path(temp)/str(offset);folder.mkdir();p=FakePlanner(offset)
            path,detail=planner.plan_phase(p,Kin(),np.zeros(7),b,'lift',[],recipe,folder,8)
            assert (path is not None)==expected
            assert p.calls==(2 if expected else 1)
            assert detail['segments'][0]['deviation_gate_passed']==expected
            assert len(list(folder.glob('*.npz')))==p.calls
            if expected:
                assert len(path)==6
                np.testing.assert_array_equal(path[2],path[3])
                assert detail['segments'][1]['phase_sample_start']==3
            else:
                assert detail['failed_segment_index']==0
                assert detail['solver_success'] is True
                assert detail['solver_status']=='TCP_SEGMENT_DEVIATION_LIMIT'

def test_finite_segment_distance_includes_endpoint_overshoot():
    a=np.eye(4);b=a.copy();b[0,3]=.02
    pose=b.copy();pose[0,3]=.03
    assert np.isclose(planner.segment_deviation(np.array([pose]),a,b),.01)

def test_independent_auditor_rejects_forged_gate_and_saved_fk():
    import json
    import pytest
    from scripts.extensions.replicator import audit_physical_retry_round4 as audit
    target=np.eye(4);target[0,3]=.04
    recipe=dict(segment_deviation_limits_m={'lift':.005})
    class FullPlanner(FakePlanner):
        def constrained_plan(self,*args,**kw):
            path,detail=super().constrained_plan(*args,**kw)
            detail.update(phase='lift',contact_masks=[],nominal_payload_attached=True,vertical_motion_cost_requested=False)
            return path,detail
    with tempfile.TemporaryDirectory() as temp,patch.object(planner,'relative',lambda p:str(p)),patch.object(planner,'fk_batch',lambda k,q:np.array([k.fk(x) for x in q])),patch.object(audit,'ROOT',Path(temp)):
        folder=Path(temp);path,detail=planner.plan_phase(FullPlanner(.01),Kin(),np.zeros(7),target,'lift',[],recipe,folder,8)
        result=dict(status='failed',start_q=[0.]*7,targets={'lift':target.tolist()},phases=[detail],failed_phase='lift',artifacts={})
        row=dict(candidate_planning_result={'path':str(folder/'result.json')})
        checked=audit.audit_declared_segments(result,recipe,row,kin=Kin())
        assert checked['independent_fk_samples']==3
        assert checked['segments'][0]['deviation_gate_passed'] is False
        detail['segments'][0]['deviation_gate_passed']=True
        with pytest.raises(ValueError,match='Hard segment gate'):
            audit.audit_declared_segments(result,recipe,row,kin=Kin())
        detail['segments'][0]['deviation_gate_passed']=False
        artifact=detail['segments'][0]['trajectory'];file=Path(artifact['path'])
        with np.load(file) as saved:q=saved['q'];tcp=saved['tcp_world']
        tcp[1,1,3]=0
        np.savez_compressed(file,q=q,tcp_world=tcp);audit.HASHES.clear();artifact['sha256']=audit.digest(file)
        with pytest.raises(ValueError,match='Saved segment TCP'):
            audit.audit_declared_segments(result,recipe,row,kin=Kin())
