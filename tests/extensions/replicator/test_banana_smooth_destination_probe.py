from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
import pytest
from kitchen_safety.extensions.replicator.banana_command_recording import CommandRecorder,JointBounds
from kitchen_safety.extensions.replicator.banana_smooth_destination_probe import DECLARATION,read_declaration,DiagnosticDestinationWorld,PARENT_PATH,PARENT_SHA256,SCHEDULE_PATH,SCHEDULE_SHA256,digest
ROOT=Path(__file__).resolve().parents[3]


def test_declaration_and_frozen_dependencies():
    assert read_declaration(ROOT/'configs/extensions/replicator/g_banana_smooth_destination_contact_probe.json')==DECLARATION
    assert DECLARATION['maximum_release_actions']==30 and DECLARATION['maximum_release_physics_steps']==240
    assert DECLARATION['original_joint_bound_allowance_rad']==1e-5
    assert digest(ROOT/PARENT_PATH)==PARENT_SHA256 and digest(ROOT/SCHEDULE_PATH)==SCHEDULE_SHA256


def test_actual_3927_violation_remains_hard_failure():
    p=ROOT/'reports/replicator_execution_20261023/g_banana/destination_contact_probe_02'
    m=json.loads((p/'physics_metadata.json').read_text());d=dict(np.load(p/'physics_states.npz',allow_pickle=False));audit=JointBounds(m['joint_names'])
    rows=[audit.sample({k:v[i] for k,v in d.items()}) for i in range(len(d['physics_step']))]
    bad=[x for x in rows if not x['passed']]
    assert [x['physics_step'] for x in bad]==[3927]
    assert bad[0]['violations'][0]['actual_q_rad']==-5.342944496078417e-5
    assert bad[0]['allowance_rad']==1e-5 and not bad[0]['measured_q_modified']


@pytest.fixture
def fake():
    names=[f'panda_joint{i}' for i in range(1,8)]+['finger_joint'];raw=np.array([[.1,.2,.3,.4,.5,.6,.7,.435]],dtype=np.float32)
    cfg=SimpleNamespace(scale=1.,offset=0.,use_default_offset=False,clip=None)
    term=type('JointPositionAction',(),{})();term.cfg=cfg;term.action_dim=1;term.processed_actions=raw[:,-1:].copy()
    calls=[];counter=[0]
    def original_step(*args,**kwargs):calls.append('physics');counter[0]+=1
    sim=SimpleNamespace(step=original_step,get_physics_step_count=lambda:counter[0])
    native=SimpleNamespace(shared_metatype=SimpleNamespace(dof_names=names),get_dof_position_targets=lambda:raw.copy())
    robot=SimpleNamespace(joint_names=names,root_view=native,data=SimpleNamespace(joint_pos_target=raw.copy()))
    env=SimpleNamespace(sim=sim,action_manager=SimpleNamespace(action=raw.copy(),get_term=lambda key:term))
    recorder=SimpleNamespace(action_index=0,phase='release');actions=[dict(action=raw[0].copy())]
    return env,robot,recorder,actions,term,calls,counter,raw


def test_pre_step_reads_native_target_and_binds_following_actual_state(fake):
    env,robot,rec,actions,term,calls,count,raw=fake;before=deepcopy(raw)
    trace=CommandRecorder(env,robot,rec,actions);trace.observe(dict(physics_step=0,q=np.zeros(8),dq=np.zeros(8)))
    env.sim.step(render=False)
    assert calls==['physics'] and trace.rows[0]['before_physics_step']==0 and trace.rows[0]['after_physics_step']==1
    q=np.arange(8.)/10;dq=np.ones(8)
    trace.observe(dict(physics_step=1,q=q,dq=dq));report=trace.report()
    assert report['passed'] and np.array_equal(report['rows'][0]['actual_q'],q)
    assert np.array_equal(raw,before) and np.array_equal(robot.data.joint_pos_target,before)
    trace.close();assert env.sim.step is trace.original_step


@pytest.mark.parametrize('failure',['binary_processed','staged','native','raw','nan','wrong_native_order','wrong_action_class','affine','offset','clip'])
def test_no_silent_wrong_interface_or_false_applied_target(fake,failure):
    env,robot,rec,actions,term,calls,count,raw=fake
    if failure=='binary_processed':term.processed_actions[:]=0.
    if failure=='staged':robot.data.joint_pos_target[0,-1]=0.
    if failure=='native':robot.root_view.get_dof_position_targets=lambda:np.zeros_like(raw)
    if failure=='raw':env.action_manager.action[0,-1]=0.
    if failure=='nan':robot.data.joint_pos_target[0,-1]=np.nan
    if failure=='wrong_native_order':robot.root_view.shared_metatype.dof_names=list(reversed(robot.joint_names))
    if failure=='wrong_action_class':term.__class__=type('BinaryJointPositionZeroToOneAction',(),{})
    if failure=='affine':term.cfg.scale=2.
    if failure=='offset':term.cfg.use_default_offset=True
    if failure=='clip':term.cfg.clip={'.*':[0.,1.]}
    with pytest.raises(ValueError):
        trace=CommandRecorder(env,robot,rec,actions);trace.observe(dict(physics_step=0,q=np.zeros(8),dq=np.zeros(8)));env.sim.step()
    assert calls==[]


def test_interval_identity_cannot_skip_actual_steps(fake):
    env,robot,rec,actions,term,calls,count,raw=fake
    trace=CommandRecorder(env,robot,rec,actions);trace.observe(dict(physics_step=0,q=np.zeros(8),dq=np.zeros(8)));env.sim.step()
    with pytest.raises(ValueError,match='matching'):trace.observe(dict(physics_step=2,q=np.zeros(8),dq=np.zeros(8)))
    with pytest.raises(ValueError,match='Unmatched'):env.sim.step()
    assert calls==['physics'] and not trace.report()['passed']


def test_no_physics_without_command_evidence(fake):
    env,robot,rec,actions,term,calls,count,raw=fake
    actions.clear();trace=CommandRecorder(env,robot,rec,actions)
    with pytest.raises(ValueError,match='no recorded command'):env.sim.step()
    assert not calls


@pytest.mark.parametrize('ordinal',[1,240,241])
def test_only_declared_240_release_physics_steps(monkeypatch,ordinal):
    import kitchen_safety.extensions.replicator.banana_smooth_destination_probe as module
    world=object.__new__(DiagnosticDestinationWorld)
    world.objects=[(module.DESTINATION_MESH,object())]
    world._query_payload_pose=np.eye(4);world.payload_triangle_model=None
    world.fcl=SimpleNamespace(CollisionObject=lambda *x:object(),Transform=lambda *x:None)
    strict_args=[]
    def strict(x):strict_args.append(x);return dict(valid=False)
    world._exact_measurement=strict
    world._probe_execution_active=True;world.phase='release';world.release_start_step=100
    world.probe_row=dict(physics_step=100+ordinal,action_index=12,destination_contact_force=np.zeros((1,3)))
    world.probe_native=dict(physics_step=100+ordinal)
    world._probe_measurements={};world.probe_records=[] if ordinal==1 else [dict(phase='release',physics_step=99+ordinal)]
    world.strict_failure_ledger=[];world.source_bowl_observation=lambda pose:dict(source=True)
    world.probe_low=np.zeros(3);world.probe_high=np.ones(3)
    monkeypatch.setattr(module,'bounded_transition_checks',lambda *x:dict(passed=True,scope='56 steps'))
    if ordinal==241:
        with pytest.raises(ValueError,match='240 actual substeps'):world._destination_measurement()
        assert not world.strict_failure_ledger
    else:
        answer=world._destination_measurement()
        assert answer['valid'] and answer['diagnostic_continuation_only']
        assert world.strict_failure_ledger[-1]['original_strict_measurement']['valid'] is False
        assert world.probe_records[-1]['bounded_transition']['scope']=='240 steps'
    assert strict_args[0][-1]==0.


def test_original_all_other_pair_checks_are_inherited():
    from kitchen_safety.extensions.replicator.banana_destination_probe import DiagnosticDestinationWorld as Original
    assert DiagnosticDestinationWorld.valid is Original.valid
    assert DiagnosticDestinationWorld.source_bowl_observation is Original.source_bowl_observation


@pytest.mark.parametrize('bad',['none','role','changed','source_text','binding_module'])
def test_archive_does_not_accept_missing_or_misidentified_code(bad):
    from kitchen_safety.extensions.replicator.banana_smooth_destination_probe import audit_declaration_archive,MODULE_PATH
    config='configs/extensions/replicator/g_banana_smooth_destination_contact_probe.json'
    binding=dict(path=config,sha256=digest(ROOT/config),module_path=MODULE_PATH,module_sha256=digest(ROOT/MODULE_PATH))
    entries={}
    for path in [config,MODULE_PATH,PARENT_PATH,SCHEDULE_PATH]:
        text=(ROOT/path).read_text();h=digest(ROOT/path)
        entries[path]=dict(kind='diagnostic_declaration_input' if path==config else 'execution_source',initial_sha256=h,checked_sha256=h,source_text=text,unchanged=True)
    assert audit_declaration_archive(dict(entries=entries),binding)['passed']
    if bad=='none':del entries[PARENT_PATH]
    elif bad=='role':entries[PARENT_PATH]['kind']='input'
    elif bad=='changed':entries[PARENT_PATH]['checked_sha256']='0'*64
    elif bad=='source_text':entries[PARENT_PATH]['source_text']+=' '
    elif bad=='binding_module':binding['module_path']='wrong.py'
    with pytest.raises((ValueError,KeyError)):audit_declaration_archive(dict(entries=entries),binding)
