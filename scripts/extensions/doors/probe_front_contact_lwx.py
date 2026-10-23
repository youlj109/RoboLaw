"""Offline contact-approach lookahead for candidate screening; does not change collection policy."""
import argparse, json, hashlib
from pathlib import Path
import numpy as np
from scripts.extensions.doors.paired_planning import plan
from kitchen_safety.extensions.doors.reconstruction import reconstruct
from kitchen_safety.curobo_backend import CuroboPlanner

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--config',type=Path,required=True)
p.add_argument('--variant',choices=['human','property'],default='property')
p.add_argument('--output',type=Path,required=True)
p.add_argument('--baseline',type=Path,default=Path('datasets/doors/lwx_front_20261023/base_01'))
a=p.parse_args()
if a.output.exists():p.error('Refusing to overwrite evidence')
m=json.loads((a.baseline/'metadata.json').read_text());rows=json.loads((a.baseline/'trajectory.json').read_text());m['_start_full_q']=next(r['obs']['q'] for r in rows if r['phase']=='pregrasp')
spec=json.loads(a.config.read_text())[a.variant]
r=plan(np.asarray(m['_start_full_q'][:7]),m,spec,42,m['controller_parameters']['handle_height_candidates_m'])
report={'scope':'Offline IK/collision lookahead only; actual 120Hz contact and physics still required','config':str(a.config),'config_sha256':hashlib.sha256(a.config.read_bytes()).hexdigest(),'pregrasp':r,'approach_samples':[],'approach_passed':False}
if r['success']:
    stage,kin,world=reconstruct(m,spec);world.allow_handle_contact=True
    planner=CuroboPlanner(world,kin,dt=m['dt'],seed=42,max_joint_speed=.5)
    handles=tuple(n.rsplit('/',1)[-1] for n,*_ in world.objects if '/fridge_door_handle' in n)
    q=np.asarray(r['positions'][-1]);target=np.asarray(r['target_pose']);handle=target[:3,3]+[0.,.13,0.]
    for index,alpha in enumerate(np.linspace(0,1,50)[1:]):
        target[:3,3]=handle+[0.,-.13*(1-alpha),0.];command=kin.ik(target,q,attempts=1)
        planner._prepare_path_query(handles,0.,'approach_handle')
        curobo_valid=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(command[None])).all())
        fcl_valid=bool(world.valid(command,mode='execution',gripper_q={n:m['_start_full_q'][m['robot_joint_names'].index(n)] for n in world.GRIPPER_JOINT_NAMES}))
        report['approach_samples'].append({'index':index,'alpha':float(alpha),'q':command.tolist(),'curobo_valid':curobo_valid,'fcl_valid':fcl_valid,'fcl_detail':world.last_clearance})
        if not curobo_valid or not fcl_valid:break
        q=command
    report['approach_passed']=len(report['approach_samples'])==49 and all(x['curobo_valid'] and x['fcl_valid'] for x in report['approach_samples'])
a.output.write_text(json.dumps(report,indent=2)+'\n')
print('CONTACT_PREVIEW',r['success'],r.get('selected_offset_m'),report['approach_passed'],flush=True)
raise SystemExit(0 if report['approach_passed'] else 1)
