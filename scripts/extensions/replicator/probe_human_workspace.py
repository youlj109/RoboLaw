"""Test alternative bowl-approach arm poses against the human exclusion zone.

These are IK/geometry probes, never executed trajectories or full-world plans.
"""
import json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from audit import hazard_stage
from kitchen_safety.extensions.replicator.layout import resolve_layout
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3];l=resolve_layout('human_default_mpfb')
reference=json.loads((ROOT/'datasets/skill_trials/replicator/base/diagnostic_05/result.json').read_text())['layout']
if any(l[key]!=reference[key] for key in ['robot_position','robot_yaw_deg','mustard_position','bowl_position']):
 raise RuntimeError('Historical diagnostic_05 differs from the legal current layout; its bowl-target probe is not transferable.')
stage,_=hazard_stage(l,'human',l['obstacle'])
k=DroidKinematics(np.array(l['robot_position'])+[0,0,l['stand_height']],l['robot_yaw_deg']);w=CollisionWorld(stage,k,clearances=l['planning']['clearances_m'])
with h5py.File(ROOT/'datasets/skill_trials/replicator/base/diagnostic_05/trajectory.h5') as f:
 names=json.loads(f.attrs['joint_names']);ids=[names.index(f'panda_joint{i}') for i in range(1,8)];phases=f['phase'].asstr()[:];i=np.flatnonzero(phases=='above_bowl')[-1];q0=f['obs/q'][i,ids];goal=k.fk(q0)
rows=[];found=False
for tilt in [35,55,75]:
 for yaw in range(0,360,30):
  pose=goal.copy();pose[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]])
  try:q=k.ik(pose,q0,attempts=2)
  except RuntimeError:continue
  check=w.clearance(q,mode='planning',obstacle_filter=('person',))
  row=dict(tilt=tilt,yaw=yaw,q=q.tolist(),target=pose.tolist(),fk_error=float(np.linalg.norm(k.fk(q)[:3,3]-pose[:3,3])),human_clearance=check);rows.append(row)
  if not check['valid']:found=True;break
 if found:break
j=dict(scope='Alternative same bowl-approach TCP position; human geometry exclusion only, no execution/full-world/self-collision or task-success claim',human_excludes_reachable_configuration=found,candidates=rows)
(ROOT/'reports/skill_trials/replicator/human_workspace_probe.json').write_text(json.dumps(j,indent=2));print(json.dumps(dict(found=found,candidates=len(rows),last=rows[-1] if rows else None)))
