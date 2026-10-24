"""Check the diagnostic hold and task-bound asset placement input contracts."""
import json,tempfile,argparse
from pathlib import Path
from types import SimpleNamespace
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest
from kitchen_safety.task_hold_probe_policy import create
from scripts.probe_risk_asset_library import prepare


def check():
    names=['panda_joint'+str(i) for i in range(1,8)];robot=SimpleNamespace(joint_names=names)
    env=SimpleNamespace(unwrapped=SimpleNamespace(action_manager=SimpleNamespace(total_action_dim=8),step_dt=1/30,scene={'robot':robot},device='cpu'))
    task=dict(roles=dict(robot=dict(id='robot',registry_name='droid_abs_joint_pos')),asset_declarations={'table':{}})
    config=dict(version='robolaw-measured-hold-probe-1',task_sha256=digest(task),actions=3);cases=[]
    policy=create(env=env,contract=task,bindings={},config=config)
    for n in range(3):
        row=policy.act({'robot':{'q':[.1]*7}},None)
        assert row['action'].shape==(1,8) and row['phase']=='settle' and row['operation_id'] is None
        assert row['evidence']['full_task_trajectory'] is False and row['evidence']['action_index']==n
    assert policy.act({'robot':{'q':[.1]*7}},None) is None;cases.append('three_actual_hold_commands')
    for name,delta in [('wrong_task',{'task_sha256':'other'}),('bool_count',{'actions':True}),('zero_count',{'actions':0}),('long_run',{'actions':31}),('extra_field',{'other':1})]:
        rejected=False
        try:create(env=env,contract=task,bindings={},config=dict(config,**delta))
        except ValueError:rejected=True
        assert rejected;cases.append(name)
    policy=create(env=env,contract=task,bindings={},config=config);rejected=False
    try:policy.act({'robot':{'q':[float('nan')]*7}},None)
    except ValueError:rejected=True
    assert rejected;cases.append('invalid_measured_reference')
    with tempfile.TemporaryDirectory(prefix='risk-asset-input-') as directory:
        source=Path(directory)/'fixture.usda';source.write_text('# fixture identity only; no geometry validation claimed\n')
        asset=dict(id='obstacle',category='property',static=False,usd_path=str(source))
        template=dict(task_sha256=digest(task),placement_mode='native_relations',placement_evidence={'purpose':'fixture'},placement_relations=[{'kind':'on','reference':'table','params':{}}])
        binding,policy=prepare({},asset,task,template)
        assert binding['category']=='PT' and binding['static'] is False and policy['actions']==3
        assert binding['asset_id']=='obstacle' and binding['placement_evidence']['asset_usd_sha256'];cases.append('dynamic_asset_identity_preserved')
        other,_=prepare({},dict(asset,category='robot',static=True),task,template)
        assert other['category']=='RS' and other['static'] is True;cases.append('static_robot_obstacle_identity_preserved')
        for name,changes in [('cross_task',{'task_sha256':'other'}),('world_pose',{'position_xyz':[0,0,0]}),('physics_override',{'static':True}),('identity_override',{'asset_id':'other'}),('unknown_reference',{'placement_relations':[{'kind':'on','reference':'other','params':{}}]})]:
            rejected=False
            try:prepare({},asset,task,dict(template,**changes))
            except ValueError:rejected=True
            assert rejected;cases.append(name)
        for name,changes in [('missing_asset',{'usd_path':str(source)+'.missing'}),('unknown_category',{'category':'other'}),('nonboolean_physics',{'static':1})]:
            rejected=False
            try:prepare({},dict(asset,**changes),task,template)
            except ValueError:rejected=True
            assert rejected;cases.append(name)
    return cases


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists();result=dict(passed=True,cases=check(),simulation_steps=0,gpu_queries=0,scope='Input/command contract fixtures; geometry and placement checked separately');a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
