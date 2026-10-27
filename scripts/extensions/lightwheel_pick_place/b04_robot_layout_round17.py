"""Saved trajectory geometry only: one bounded revised robot obstacle proposal."""
from pathlib import Path
import json,h5py,numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
root=Path('datasets/skill_trials/lightwheel_pick_place');actual=root/'human_lgd_b04_round09_20261027T081735Z';baseline=root/'base_lgd_b04_handle_20261025T194846Z';robot=root/'robot_lgd_b04_round08_20261027T080858Z';r=json.loads((actual/'result.json').read_text());spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world=CollisionWorld(Usd.Stage.Open(str(robot/'scene.usda')),kin,pick_object='green_ladle',destination_object='plate',source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!='green_ladle')+('barrier',));data={}
for label,ep in [('actual',actual),('baseline',baseline)]:
 rr=json.loads((ep/'result.json').read_text());ids=[rr['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)];finger=[rr['robot_joint_names'].index(n) for n in world.GRIPPER_JOINT_NAMES]
 with h5py.File(ep/'trajectory.h5') as h:data[label]=(h['next_obs/q'][:],h['next_obs/green_ladle'][:,:7],ids,finger)
report={'scope':'Geometry proposal, no environment changed or success claim; all actual496 action states, baseline counterfactual sampled at15Hz; later full120Hz and matched-control review mandatory','cases':[]};selected=None
for xyz in ([1.95,-.833,.93],):
 world.set_obstacle_pose('barrier',np.r_[xyz,Rotation.from_euler('z',180,degrees=True).as_quat()]);stats={}
 for label in ('actual','baseline'):
  qs,poses,ids,finger=data[label];rows=[]
  for i in range(len(qs)):
   c=world.clearance(qs[i,ids],('green_ladle',),mode='execution',gripper_q=dict(zip(world.GRIPPER_JOINT_NAMES,qs[i,finger])),object_pose=poses[i],obstacle_filter=('barrier',));rows.append(c)
  stats[label]={'all_safe':all(c['valid'] for c in rows),'minimum':min(rows,key=lambda c:c['slack_m']),'violations':sum(not c['valid'] for c in rows)}
 row={'position':xyz,**stats};report['cases'].append(row);print(row,flush=True)
 if stats['actual']['all_safe'] and not stats['baseline']['all_safe']:selected=xyz;break
report['selected']=selected;Path('/root/autodl-tmp/B04-review/robot_layout_proposal_round17.json').write_text(json.dumps(report,indent=2))
if selected:
 p=Path('configs/extensions/lightwheel_pick_place/variants_ladle_property_round14.json');d=json.loads(p.read_text());d['robot']['position']=selected;Path('configs/extensions/lightwheel_pick_place/variants_ladle_robot_round17.json').write_text(json.dumps(d,indent=2))
