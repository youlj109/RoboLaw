"""Validate diagnostic command binding and actual submission bookkeeping."""
import argparse
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from kitchen_safety.task_contract import digest


def check(module):
    import numpy as np
    import torch,warp as wp
    from kitchen_safety.articulation_capture import array
    contract=dict(operations=[dict(operation_id='op',articulation_id='door',joint_name='hinge')])
    bindings=dict(asset_roots={'door':'/world/door'},robot_root='/world/robot')
    inventory=dict(joints=[dict(path='/world/door/hinge',enabled=True,type='PhysicsRevoluteJoint')])
    command=dict(version=module.VERSION,task_sha256=digest(contract),operation_id='op',effort_si=.2,effort_limit_si=1.,diagnostic_only=True)
    class View:
        count=1;max_dofs=2;dof_paths=[['/world/door/other','/world/door/hinge']];prim_paths=['/world/door']
        def __init__(self,frontend='numpy'):self.values=np.array([[.3,0.]],dtype=np.float32);self.calls=0;self.frontend=frontend
        def get_dof_actuation_forces(self):
            if self.frontend=='torch':return torch.as_tensor(self.values)
            if self.frontend=='warp':return wp.array(self.values,dtype=wp.float32,device='cpu')
            return self.values
        def set_dof_actuation_forces(self,value,indices):
            assert array(indices).tolist()==[0]
            if self.frontend=='warp':assert isinstance(value,wp.array) and isinstance(indices,wp.array)
            elif self.frontend=='torch':assert isinstance(value,torch.Tensor)
            else:assert isinstance(value,np.ndarray)
            self.values=array(value);self.calls+=1
    cases=[]
    for name,change in [('wrong_task',dict(task_sha256='wrong')),('unknown_op',dict(operation_id='other')),
            ('nan',dict(effort_si=float('nan'))),('bool',dict(effort_si=True)),('infinite_limit',dict(effort_limit_si=float('inf'))),
            ('negative_limit',dict(effort_limit_si=-1)),('over_bound',dict(effort_si=2)),('not_diagnostic',dict(diagnostic_only=False)),
            ('extra_field',dict(other=True))]:
        rejected=False
        try:module.validate_command(dict(command,**change),contract)
        except ValueError:rejected=True
        assert rejected,name;cases.append(name)
    for name,views,inv,b in [('ambiguous',[View(),View()],inventory,bindings),('missing',[],inventory,bindings),
            ('disabled',[View()],{'joints':[dict(inventory['joints'][0],enabled=False)]},bindings),
            ('unknown_units',[View()],{'joints':[dict(inventory['joints'][0],type='PhysicsSphericalJoint')]},bindings),
            ('robot_scope',[View()],inventory,dict(bindings,robot_root='/world/door'))]:
        rejected=False
        try:module.resolve_command(command,contract,b,inv,views)
        except ValueError:rejected=True
        assert rejected,name;cases.append(name)
    for frontend in ['numpy','torch','warp']:
        for kind,unit in [('PhysicsRevoluteJoint','Nm'),('PhysicsPrismaticJoint','N')]:
            view=View(frontend);inv={'joints':[dict(inventory['joints'][0],type=kind)]}
            bound=module.resolve_command(command,contract,bindings,inv,[view]);assert bound['unit']==unit
            receipt=module.submit_command(bound,device='cpu',physics_step=3,action_index=0)
            assert view.calls==1 and abs(receipt['submitted'][0][0]-.3)<1e-6 and abs(receipt['submitted'][0][1]-.2)<1e-6
            assert receipt['before_physics_step']==3 and receipt['diagnostic_only'] is True
            assert receipt['command_scope']=='diagnostic_external_articulation'
            cases.append(frontend+'_'+kind+'_submission')
    view=View();view.values[0,0]=float('nan');bound=module.resolve_command(command,contract,bindings,inventory,[view]);rejected=False
    try:module.submit_command(bound,device='cpu',physics_step=0,action_index=0)
    except ValueError:rejected=True
    assert rejected and view.calls==0;cases.append('invalid_other_actuation_not_overwritten')
    for name,target in [('leave_nonzero',None),('switch_nonzero',dict(command,operation_id='other'))]:
        rejected=False
        try:module.validate_transition({'command':command},target)
        except ValueError:rejected=True
        assert rejected;cases.append(name)
    module.validate_transition({'command':dict(command,effort_si=0.)},None);cases.append('leave_zero')
    module.validate_transition({'command':command},dict(command,effort_si=0.));cases.append('submit_zero')
    return cases


def main():
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,default=ROOT/'kitchen_safety/task_articulation_probe.py');p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists()
    spec=importlib.util.spec_from_file_location('probe_candidate',a.candidate);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    result=dict(passed=True,cases=check(m),source_sha256=hashlib.sha256(a.candidate.read_bytes()).hexdigest(),simulation_steps=0,gpu_queries=0)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
