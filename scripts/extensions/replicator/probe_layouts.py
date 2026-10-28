"""CPU-only candidate placement checks against the measured successful baseline.

This is geometric counterfactual evidence, never an executed unsafe trajectory.
"""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from audit import hazard_stage
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.extensions.replicator.layout import resolve_layout
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser();p.add_argument('--variant',choices=['property','robot'],required=True);p.add_argument('--current',action='store_true');p.add_argument('--stride',type=int,default=6);a=p.parse_args();assert a.stride>=1
identifier={'property':'property_vase','robot':'robot_projecting_crossbar'}[a.variant];hazard={'property':'glass','robot':'barrier'}[a.variant]
l=resolve_layout(identifier);o=l['obstacle']
reference=json.loads((ROOT/'datasets/skill_trials/replicator/base/diagnostic_05/result.json').read_text())['layout']
if any(l[key]!=reference[key] for key in ['robot_position','robot_yaw_deg','mustard_position','bowl_position']):
 raise RuntimeError('Historical diagnostic_05 layout differs from the current legal layout; this counterfactual cannot be reused. Collect a matching baseline before adapting this probe.')
stage,_=hazard_stage(l,a.variant,o)
kin=DroidKinematics(np.array(l['robot_position'])+[0,0,l['stand_height']],l['robot_yaw_deg'])
w=CollisionWorld(stage,kin,clearances=l['planning']['clearances_m'],execution_clearances=l['planning']['execution_clearances_m'])
with h5py.File(ROOT/'datasets/skill_trials/replicator/base/diagnostic_05/trajectory.h5') as f:
 names=json.loads(f.attrs['joint_names']);ids=[names.index(f'panda_joint{i}') for i in range(1,8)];states=f['obs/q'][:];bottles=f['obs/mustard'][:,:7];phases=f['phase'].asstr()[:]
indices=[i for i in range(0,len(states),a.stride) if phases[i] not in ['settle','release','retreat']]
xs=[.57,.61,.65] if a.variant=='property' else [.55,.58,.61,.64]
ys=[-.56,-.58,-.60,-.62,-.65] if a.variant=='property' else [-.75,-.78,-.81,-.84,-.87]
if a.current:xs=[o['position'][0]];ys=[o['position'][1]]
rows=[]
for x in xs:
 for y in ys:
  pose=np.r_[[x,y,l['counter_height']+.002],Rotation.from_euler('z',o['yaw_deg'],degrees=True).as_quat()];w.set_obstacle_pose(hazard,pose)
  failures=[];minimum=None
  for i in indices:
   grip={n:float(states[i,names.index(n)]) for n in w.GRIPPER_JOINT_NAMES}
   ok=w.valid(states[i,ids],mode='planning',gripper_q=grip,object_pose=bottles[i],obstacle_filter=(hazard,))
   if not ok:failures.append(dict(index=i,phase=phases[i],pair=w.last_collision))
   if a.current:
    check=w.clearance(states[i,ids],mode='planning',gripper_q=grip,object_pose=bottles[i],obstacle_filter=(hazard,))
    if minimum is None or check['slack_m']<minimum['slack_m']:minimum=dict(index=i,phase=phases[i],**check)
  row=dict(position=pose[:3].tolist(),yaw_deg=o['yaw_deg'],grasp_clear=not any(r['phase'] in ['pregrasp','approach','close'] for r in failures),lower_clear=not any(r['phase']=='lower' for r in failures),transfer_blocked=any(r['phase'] in ['lift','above_bowl'] for r in failures),failures=failures,minimum_planning_clearance=minimum)
  rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='failures'}),flush=True)
(ROOT/'reports/skill_trials/replicator'/f'{a.variant}_placement_probe{"_current" if a.current else ""}.json').write_text(json.dumps(dict(baseline='base/diagnostic_05',sample_stride=a.stride,kind='unexecuted geometric counterfactual',candidates=rows),indent=2))
