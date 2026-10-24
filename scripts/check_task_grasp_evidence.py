"""Verify causal per-payload grasp refinement without physics or GPU queries."""
import argparse
from collections import deque
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def fixture(module,config,payload_ids=('first',)):
    from kitchen_safety.grasp_evidence import GraspEvidence
    f=module.TaskFeatures.__new__(module.TaskFeatures);f.step=-1;f.dt=1/120
    f.config=deepcopy(config);f.windows=deque(maxlen=4);f.payloads={}
    f.scope={'bindings':{'robot_root':'/robot','grip_actor_paths':['/robot/left','/robot/right']},'relations':[]}
    f.model={'hand_actor_path':'/robot/hand','bilateral_actor_paths':['/robot/left','/robot/right'],'opening_joint_name':'finger_joint'}
    f.schema={'robot_joint_names':['finger_joint']}
    state={'actor_states_world':{'/robot/hand':[0.,0.,1.,0.,0.,0.,1.]},'robot':{'q':[.8]}}
    for aid in payload_ids:
        actor='/payload/'+aid
        f.payloads[aid]=dict(actor=actor,colliders=[actor+'/collision'],support_paths=[],local_geometry_center=[0.,0.,0.],
            tracker=GraspEvidence(config['grasp_evidence']),maximum_z=None)
        state['actor_states_world'][actor]=[0.,0.,1.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.]
        for index,grip in enumerate(f.scope['bindings']['grip_actor_paths']):
            f.scope['relations'].append(dict(category='PT',source=grip,target=actor,query_distance=True,relation_id=aid+':'+str(index)))
    return f,state


def sample(f,state,step,*,q=None,bilateral=(),covered=True,geometry=None,phase='transport'):
    f.step=step
    if q is not None:state['robot']['q']=[q]
    f.windows=deque([dict(coverage=covered,pairs=[dict(bodyA=f.payloads[aid]['actor'],bodyB=grip,force_n=1.,contacts=[])
        for aid in bilateral for grip in f.scope['bindings']['grip_actor_paths']])],maxlen=4)
    f._grasp(state,phase,geometry)
    return {aid:deepcopy(p['grasp']) for aid,p in f.payloads.items()}


def geometry_for(f,distance):
    return dict(relations={r['relation_id']:dict(distance_m=distance) for r in f.scope['relations']})


def check(module,config):
    passed=[]
    f,s=fixture(module,config)
    first=sample(f,s,0,bilateral=('first',))
    second=sample(f,s,0,bilateral=('first',))
    assert first==second and first['first']['held'] is True
    passed.append('same_step_contact_confirmation_is_idempotent')
    first=sample(f,s,1,q=.7,bilateral=('first',))
    second=sample(f,s,1,bilateral=('first',),geometry=geometry_for(f,0.))
    assert first['first']['opening_detected'] and second['first']['opening_detected']
    assert first['first']['held'] is None and second['first']['held'] is None
    assert first['first']['reason']==second['first']['reason']=='measured_gripper_opening'
    assert f.payloads['first']['tracker'].anchor is None
    passed.append('opening_not_reconfirmed_by_same_step_residual_contacts')
    third=sample(f,s,1,bilateral=('first',),geometry=geometry_for(f,0.))
    assert third==second
    passed.append('repeated_label_refinement_is_idempotent')
    f,s=fixture(module,config);sample(f,s,0,bilateral=('first',))
    first=sample(f,s,1,bilateral=('first',))
    second=sample(f,s,1,bilateral=('first',))
    assert first==second and second['first']['last_bilateral_age_s']==f.dt
    passed.append('age_uses_previous_physics_step_not_first_call_mutation')
    f,s=fixture(module,config);sample(f,s,0,bilateral=('first',))
    pre=sample(f,s,1)
    assert pre['first']['held'] is True
    refined=sample(f,s,1,geometry=geometry_for(f,.02))
    assert refined['first']['held'] is False and refined['first']['reason']=='separated_gripper'
    assert sample(f,s,1,geometry=geometry_for(f,.02))==refined
    next_step=sample(f,s,2)
    assert next_step['first']['held'] is None
    passed.append('mesh_separation_refinement_commits_to_next_step')
    f,s=fixture(module,config);sample(f,s,0,bilateral=('first',))
    before=sample(f,s,1,covered=False)
    after=sample(f,s,1,covered=False,geometry=geometry_for(f,0.))
    assert before['first']['held'] is None and after['first']['held'] is None
    passed.append('missing_contact_coverage_never_confirms_grasp')
    f,s=fixture(module,config);sample(f,s,0,bilateral=('first',))
    before=sample(f,s,1)
    refined=sample(f,s,1,phase='release')
    assert before['first']['held'] is True and refined['first']['held'] is None
    assert sample(f,s,1,phase='release')==refined
    passed.append('phase_refinement_uses_same_pre_step_anchor')
    f,s=fixture(module,config,('first','second'));sample(f,s,0,bilateral=('first',))
    before=sample(f,s,1,q=.7,bilateral=('first','second'))
    after=sample(f,s,1,bilateral=('first','second'),geometry=geometry_for(f,0.))
    assert before['first']['held'] is None and after['first']['held'] is None
    assert before['second']['held'] is True and after['second']['held'] is True
    assert f.payloads['first']['tracker'].anchor is None and f.payloads['second']['tracker'].anchor is not None
    passed.append('multiple_payload_trackers_are_independent')
    f,s=fixture(module,config);sample(f,s,0,bilateral=('first',))
    for step in range(1,21):
        sample(f,s,step);latest=sample(f,s,step,geometry=geometry_for(f,0.))
    assert latest['first']['held'] is None
    passed.append('repeated_labels_do_not_extend_contact_dropout_budget')
    f,s=fixture(module,config)
    initial=sample(f,s,-1,covered=False)
    assert sample(f,s,-1,covered=False)==initial
    sample(f,s,0,bilateral=('first',))
    last=sample(f,s,1,q=.7,bilateral=('first',),phase='release',geometry=geometry_for(f,0.))
    assert last['first']['held'] is None and last['first']['opening_detected']
    passed.append('initial_and_terminal_source_steps_use_same_contract')
    return passed


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path);p.add_argument('--config',type=Path,default=ROOT/'configs/safety_observer_arena.json')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise ValueError('Preserve existing check output')
    source=args.candidate or ROOT/'kitchen_safety/task_features.py'
    spec=importlib.util.spec_from_file_location('reviewed_task_features',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=dict(passed=True,implementation_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        cases=check(module,json.loads(args.config.read_text())),physics_steps=0,gpu_queries=0,
        historical_outputs_modified=False,scope='Synthetic causal/stateful input cases, not physical grasp-success validation')
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
