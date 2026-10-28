from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from kitchen_safety.extensions.replicator.task_contact_policy import BananaContactPolicy
from kitchen_safety.extensions.replicator.source_support_query import (
    bind_observed_departure, binding_check, partition_original_world,
)


def observed_fixture():
    vertices = np.array([[x,y,z] for x in (-.04,0.) for y in (-.02,.02) for z in (-.018,.018)])
    geometry = SimpleNamespace(vertices=vertices, triangles=np.array([[0,2,4],[2,4,6],[1,3,5],[3,5,7]]),
        provenance={"source_sha256":"source"})
    tcp,payload = np.eye(4),np.eye(4)
    tcp[:3,3] = [.5,.5,1.05]
    payload[:3,3] = [.5,.5,1.018]
    support_vertices = np.array([[x,y,z] for x in (0.,1.) for y in (0.,1.) for z in (.95,1.)])
    support_faces = np.array([[0,2,4],[2,4,6],[1,3,5],[3,5,7]])
    support_object = SimpleNamespace(getRotation=lambda:np.eye(3),getTranslation=lambda:np.zeros(3))
    world = SimpleNamespace(payload_name="banana",contact_policy=BananaContactPolicy(),default_clearance=.002,
        objects=[(BananaContactPolicy().source_support,support_object,np.array([0.,0.,.95]),np.array([1.,1.,1.]))],
        meshes={BananaContactPolicy().source_support:(support_vertices,support_faces)},
        _object_reference=[(np.eye(3),np.zeros(3),np.array([0.,0.,.95]),np.array([1.,1.,1.]),(support_vertices,support_faces))],
        _root_reference_transforms={BananaContactPolicy().source_support:np.eye(4)},
        payload_geometry=geometry,payload_pose=payload,attachment=np.linalg.inv(tcp)@payload,
        gripper_joint_positions=np.array([1,1,-1,1,-1,-1])*.44,gripper_joint_position=.44,phase="lift")
    kin = SimpleNamespace(fk=lambda _:tcp)
    item = dict(count=1,points_object_local_m=[[-.02,0.,0.]],normal_forces_n=[1.],separations_m=[0.])
    rows = [dict(phase="close",physics_step=i,object_name="banana",physics_dt_s=1/120,
        fingers={name:deepcopy(item) for name in ("left_inner_finger","right_inner_finger")}) for i in range(10)]
    observations = dict(physics_step=9,arm_q=np.zeros(7),measured_tcp=tcp,measured_payload=payload,raw_contact_rows=rows)
    return world,kin,observations


def test_bound_context_requires_actual_state_and_rejects_unbound_diagnostic():
    world,kin,obs = observed_fixture()
    context = bind_observed_departure(world,kin,**obs)
    path = np.zeros((2,7))
    assert binding_check(world,path,context)["passed"]
    assert not binding_check(world,path,None)["passed"]
    changed = path.copy();changed[0,0] = .001
    assert not binding_check(world,changed,context)["passed"]
    world.gripper_joint_positions[2] += .001
    assert not binding_check(world,path,context)["passed"]


@pytest.mark.parametrize("bad",["gapped_steps","one_finger","outside_contact","wrong_attachment","different_support"])
def test_required_physical_evidence_cannot_be_substituted(bad):
    world,kin,obs = observed_fixture()
    if bad == "gapped_steps":obs["raw_contact_rows"][3]["physics_step"] = 2
    if bad == "one_finger":obs["raw_contact_rows"][7]["fingers"]["right_inner_finger"]["normal_forces_n"] = [0.]
    if bad == "outside_contact":obs["raw_contact_rows"][9]["fingers"]["left_inner_finger"]["points_object_local_m"] = [[.02,0.,0.]]
    if bad == "wrong_attachment":world.attachment[0,3] += .001
    if bad == "different_support":world.contact_policy = BananaContactPolicy(source_support="some_other_counter")
    with pytest.raises(ValueError):bind_observed_departure(world,kin,**obs)


def partition_fixture(fail=None):
    world,_,_ = observed_fixture()
    support = world.contact_policy.source_support
    enabled = {"banana":False,support:True,"neighbor":True}
    checker = SimpleNamespace(enable_obstacle=lambda name,enable:enabled.__setitem__(name,enable))
    # sample 0: robot/support, sample 1: payload/neighbor, both samples payload/support.
    pairs = np.array([[[1.,0.],[1.,0.]],[[0.,0.],[1.,1.]]])
    def forward(query,env_query_idx=None):
        if fail == "kernel" and not enabled[support]:raise RuntimeError("diagnostic kernel failure")
        active = (query[:,0,:,3].numpy() > 0.).astype(float)
        worlds = np.array([enabled[support],enabled["neighbor"]],float)
        values = (pairs*active[:,:,None]*worlds[None,None,:]).sum(axis=2)
        return torch.tensor(values[:,None,:])
    config = SimpleNamespace(link_name_to_idx_map={"robot":0,"attached_object":1},link_sphere_idx_map=torch.tensor([0,1]))
    cost = SimpleNamespace(world_coll_checker=checker,forward=forward)
    rollout = SimpleNamespace(primitive_collision_constraint=cost,
        dynamics_model=SimpleNamespace(robot_model=SimpleNamespace(kinematics_config=config)))
    planner = SimpleNamespace(world=world,motion_gen=SimpleNamespace(ik_solver=SimpleNamespace(rollout_fn=rollout)),
        candidate_contact_mask={"object_switches":["banana"]},obstacle_names=list(enabled),_ignored=lambda name,masks:name in masks)
    state = SimpleNamespace(robot_spheres=torch.ones((2,1,2,4)))
    original = dict(components={"world_collision":dict(costs=[2.,2.])})
    return planner,state,original,enabled


def test_partition_keeps_robot_counter_and_payload_neighbor_rejections():
    planner,state,original,enabled = partition_fixture()
    report = partition_original_world(planner,state,original)
    assert report["disjoint_exhaustive"] and report["original_zero_nonzero_equivalent"]
    assert report["rows"]["robot_all_world"]["nonzero_samples"] == [0]
    assert report["rows"]["payload_non_support"]["nonzero_samples"] == [1]
    assert not report["noncontact_passed"]
    assert enabled == {"banana":False,planner.world.contact_policy.source_support:True,"neighbor":True}


def test_partition_restores_original_objects_even_if_query_raises():
    planner,state,original,enabled = partition_fixture("kernel")
    with pytest.raises(RuntimeError,match="kernel failure"):
        partition_original_world(planner,state,original)
    assert enabled == {"banana":False,planner.world.contact_policy.source_support:True,"neighbor":True}


def test_partition_must_equal_original_full_gate():
    planner,state,original,_ = partition_fixture()
    original["components"]["world_collision"]["costs"] = [0.,2.]
    with pytest.raises(ValueError,match="disagree"):
        partition_original_world(planner,state,original)


def test_all_geometry_passes_still_cannot_authorize_an_unbound_candidate(monkeypatch):
    import kitchen_safety.extensions.replicator.source_support_query as module
    world = SimpleNamespace(valid=lambda _:True,last_clearance=None)
    original = dict(components={name:dict(nonzero_samples=[]) for name in
        ("joint_bound","world_collision","self_collision")})
    monkeypatch.setattr(module,"binding_check",lambda *args:dict(passed=False))
    monkeypatch.setattr(module,"source_departure_geometry",lambda *args:dict(passed=True))
    monkeypatch.setattr(module,"constraint_components",lambda *args:(original,None))
    monkeypatch.setattr(module,"partition_original_world",lambda *args:dict(noncontact_passed=True))
    result = module.audit_source_departure(SimpleNamespace(world=world,kin=None),np.zeros((2,7)))
    assert result["candidate_geometry_valid"]
    assert not result["valid"]


def test_runtime_planner_retains_candidate_and_rejects_missing_actual_binding():
    from kitchen_safety.extensions.replicator.source_support_curobo import SourceSupportCuroboPlanner
    class Kinematics:
        def fk(self,q):
            pose = np.eye(4);pose[:3,3] = q[:3];return pose
        def ik(self,pose,previous,attempts=1):
            q = previous.copy();q[:3] = pose[:3,3];return q
    planner = SourceSupportCuroboPlanner.__new__(SourceSupportCuroboPlanner)
    planner.kin = Kinematics();planner.dt = 1/15;planner.max_joint_speed = .4
    start = np.zeros(7);start[2] = 1.
    goal = planner.kin.fk(start);goal[2,3] += .15
    with pytest.raises(RuntimeError,match="actual measured-state/raw-contact binding"):
        planner._cartesian_vertical(start,goal,"lift")
    assert len(planner.last_cartesian_candidate["positions"]) > 2


def test_same_bounds_changed_support_topology_invalidates_binding():
    world,kin,obs = observed_fixture()
    context = bind_observed_departure(world,kin,**obs)
    name = world.contact_policy.source_support
    # Change source and current topology together; the AABB is identical.
    world.meshes[name][1][0] = [0,4,2]
    result = binding_check(world,np.zeros((2,7)),context)
    assert not result["passed"]
    assert not result["checks"]["exact_support_geometry_unchanged"]


@pytest.mark.parametrize("command",[0,.5,float("nan")])
def test_lift_rejects_any_open_or_partial_gripper_command_before_cuda(command):
    from kitchen_safety.extensions.replicator.source_support_curobo import SourceSupportCuroboPlanner
    planner = SourceSupportCuroboPlanner.__new__(SourceSupportCuroboPlanner)
    with pytest.raises(ValueError,match="grip_binary=1"):
        planner._prepare_path_query((),command,"lift")
