"""CPU contract tests. Synthetic fixtures do not certify physical stability."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.bowl_safety_neutral_evidence import (
    ROOT, TASK, DECLARATION, KITCHEN, ROBOT, read_declaration,
    audit_native_inventory, match_episode)
from scripts.extensions.replicator.neutral_bowl_safety_probe import parse_args


def inventory_fixture():
    n = 3
    names = ['panda_link'+str(i) for i in range(9)] + ['base_link'] + [
        s+'_'+p for s in ('left','right') for p in ('outer_knuckle','outer_finger','inner_finger','inner_knuckle')]
    links = [ROBOT+'/'+p for p in names]
    root = KITCHEN+'/Range'
    body = root+'/body'; joint = root+'/hinge'
    item = dict(native_root=root,native_count=1,link_paths=[body],link_names=['body'],
        dof_paths=[joint],dof_names=['hinge'],dof_count=1,link_count=1,limits=[[[-3.,3.]]])
    inv = dict(errors=[],robot_native_paths=links,robot_sensor_paths=links.copy(),robot_enabled_usd_paths=links.copy(),
        robot_native_names=names,articulations=[item],broad_native_roots=[root],authored_articulation_roots=[root],
        authored_root_coverage={root:root},source_joint_paths=[joint],background_enabled_usd_paths=[body],source_joint_rigid_endpoints=[body])
    data = dict(physics_step=np.arange(n),background_articulation_0_q=np.zeros((n,1,1)),
        background_articulation_0_dq=np.zeros((n,1,1)),
        background_articulation_0_link_poses=np.tile([0,0,0,0,0,0,1],(n,1,1,1)),
        background_articulation_0_link_velocities=np.zeros((n,1,1,6)))
    info = dict(native_inventory=inv,robot_body_names=names,background_paths=[root],task_object_names=['bowl'])
    return data,info


def test_full_native_inventory():
    d,m=inventory_fixture();r=audit_native_inventory(d,m)
    assert r['passed'] and r['robot_body_count']==18 and r['articulations'][0]['samples']==3


@pytest.mark.parametrize('bad',['robot_missing','sensor_duplicate','usd_extra_body','background_missing','dof_alias',
    'missing_dq','truncated_q','nan_velocity','extra_root','unmapped_authored','native_error','link_duplicate','native_motion'])
def test_native_missing_or_alias_rejected(bad):
    d,m=inventory_fixture();v=m['native_inventory'];a=v['articulations'][0]
    if bad=='robot_missing':v['robot_native_paths']=v['robot_native_paths'][:-1]
    if bad=='sensor_duplicate':v['robot_sensor_paths'][0]=v['robot_sensor_paths'][1]
    if bad=='usd_extra_body':v['robot_enabled_usd_paths'].append(ROBOT+'/hidden')
    if bad=='background_missing':v['background_enabled_usd_paths'].append(KITCHEN+'/hidden')
    if bad=='dof_alias':a['dof_paths']=[KITCHEN+'/other/hinge']
    if bad=='missing_dq':del d['background_articulation_0_dq']
    if bad=='truncated_q':d['background_articulation_0_q']=d['background_articulation_0_q'][:-1]
    if bad=='nan_velocity':d['background_articulation_0_link_velocities'][1,0,0,0]=np.nan
    if bad=='extra_root':v['broad_native_roots'].append(KITCHEN+'/missed')
    if bad=='unmapped_authored':v['authored_articulation_roots'].append(KITCHEN+'/missed')
    if bad=='native_error':v['errors']=['read failed']
    if bad=='link_duplicate':a['link_paths']*=2
    if bad=='native_motion':d['background_articulation_0_link_poses']=d['background_articulation_0_link_poses'].astype(float);d['background_articulation_0_link_poses'][-1,0,0,0]=.00201
    with pytest.raises((ValueError,KeyError)):audit_native_inventory(d,m)


def match_fixture():
    profile={k:k for k in ('source_sha256','recipe_sha256','overlay_sha256')}
    layout={'task_id':TASK,'obstacle':{'id':'vase','static':False}}
    n=dict(passed=True,kind='matched_hazard_neutral_observation',task_id=TASK,variant='property',seed=42,
        layout=layout,hazard_policy={'variant':'property'},background_physics=profile,
        execution_configuration=dict(simulation_device='cpu',physics_threads=2,torch_threads=2),
        physics_dt=1/120,dt=1/15,observed_duration_seconds=45.)
    m=dict(task_id=TASK,source_layout=deepcopy(layout),hazard_evidence={'policy':deepcopy(n['hazard_policy'])},
        background_physics=deepcopy(profile),physics_dt_s=1/120,action_dt_s=1/15,reset_physics_step=0)
    r=dict(task_id=TASK,variant='property',seed=42,layout=deepcopy(layout),runtime_threads={
        'actual_environment_device':'cpu','live_physics_threads':2,'live_torch_threads':2})
    return n,m,r,np.arange(4713)


def test_match_covers_full_episode():
    n,m,r,s=match_fixture();assert match_episode(n,m,r,s)['passed']


@pytest.mark.parametrize('bad',['duration','layout','variant','base','seed','profile','hazard','threads','missing_step','reset','failed'])
def test_match_rejects_wrong_or_short_evidence(bad):
    n,m,r,s=match_fixture()
    if bad=='duration':s=np.arange(5402)
    if bad=='layout':r['layout']['obstacle']['id']='other'
    if bad=='variant':r['variant']='robot'
    if bad=='base':r['variant']='base'
    if bad=='seed':r['seed']=43
    if bad=='profile':m['background_physics']['overlay_sha256']='other'
    if bad=='hazard':m['hazard_evidence']['policy']['variant']='human'
    if bad=='threads':r['runtime_threads']['live_physics_threads']=1
    if bad=='missing_step':s=np.delete(s,20)
    if bad=='reset':m['reset_physics_step']=1
    if bad=='failed':n['passed']=False
    with pytest.raises(ValueError):match_episode(n,m,r,s)


def test_declaration_unchanged(tmp_path):
    value=read_declaration(DECLARATION)
    for key,change in [('background_rotation_deg',2.),('minimum_duration_seconds',10.),('accepted',True)]:
        candidate={**value,key:change};p=tmp_path/(key+'.json');p.write_text(json.dumps(candidate))
        with pytest.raises(ValueError):read_declaration(p)


@pytest.mark.parametrize('extra',[['--seed','0'],['--device','cuda:0'],['--duration-seconds','44.9'],['--physics-threads','1'],['--torch-threads','1']])
def test_cli_retains_declared_physics(extra):
    with pytest.raises(SystemExit):parse_args(['--variant','property','--layout','x','--hazard-screen','y','--output','z']+extra)


def test_preapp_import_is_not_usd_or_kit():
    command='import sys; import scripts.extensions.replicator.neutral_bowl_safety_probe; import kitchen_safety.extensions.replicator.bowl_safety_neutral_evidence; assert not any(k == "pxr" or k.startswith("pxr.") or k.startswith("omni.usd") for k in sys.modules)'
    subprocess.run([sys.executable,'-c',command],cwd=ROOT,check=True)


def test_real_property_record_retains_failure_and_full_hazard_identity():
    from kitchen_safety.extensions.replicator.bowl_safety_recorder import audit_hazard
    episode=ROOT/'datasets/replicator/g_shape_bowl_sink/property/property75_02'
    if not episode.exists():pytest.skip('Local CPU evidence fixture not installed')
    result=json.loads((episode/'result.json').read_text())
    saved=json.loads((episode/'hazard_contacts.json').read_text())
    with np.load(episode/'physics_states.npz',allow_pickle=False) as arrays:
        data={k:arrays[k] for k in arrays.files if k.startswith('hazard_') or k in ('physics_step','phase','action_index')}
    check=audit_hazard(data,saved['metadata'],saved['rows'])
    assert check['passed'] and check['physics_samples']==2201
    assert result['physical_trial_passed'] is False
    assert len(saved['metadata']['native_sensor_paths'])==19
    assert not audit_hazard(data,saved['metadata'],saved['rows'][:-1])['passed']
    fake=deepcopy(saved['metadata']);fake['static']=True
    assert not audit_hazard(data,fake,saved['rows'])['passed']
