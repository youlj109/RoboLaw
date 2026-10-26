"""Preserve successful native joint endpoints; replan routes at frozen hazards."""
import json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from paired_planning import build_world,load_variants,grip_joints
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
root=Path('datasets/skill_trials/lightwheel_pick_place');base=root/'base_lgd_b03_fixedscene_20261026T080313Z';ref=root/'human_lgd_b03_fixedscene_20261026T092630Z';out=Path('/root/autodl-tmp/B03-review/planning_round_17.json')
if out.exists():raise FileExistsError(out)
r=json.load(open(base/'result.json'));rr=json.load(open(ref/'result.json'));spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);variants=load_variants(Path('configs/extensions/lightwheel_pick_place/variants_bananas_transportreview_20261026.json'));goals={}
with h5py.File(ref/'trajectory.h5') as h:
 ph=h['phase'].asstr()[:]
 for pick,bounds in rr['pick_action_ranges'].items():
  goals[pick]={}
  for phase in ('transfer','lower'):
   i=int(np.flatnonzero((ph==phase)&(np.arange(len(ph))>=bounds['start'])&(np.arange(len(ph))<bounds['end']))[-1]);goals[pick][phase]=h['action'][i,:7].tolist()
report={'scope':'Known native-successful human commanded transfer/lower joint endpoints; change routes only, original grasp and opening policy. No physics, threshold or scene changes.','joint_goals':goals,'cases':{}}
for variant in ('base','robot','property'):
 rows=[]
 for pick in ('banana_1','banana_2'):
  row={'pick':pick,'passed':False}
  try:
   world,stage=build_world(base,kin,variant,variants,task_spec=spec);world.bind_pick(pick)
   with h5py.File(base/'trajectory.h5') as h:
    ph=h['phase'].asstr()[:];bounds=r['pick_action_ranges'][pick];i=int(np.flatnonzero((ph=='lift')&(np.arange(len(ph))>=bounds['start'])&(np.arange(len(ph))<bounds['end']))[-1]);ids=[r['robot_joint_names'].index('panda_joint'+str(j)) for j in range(1,8)];q=h['next_obs/q'][i,ids];pose=h['next_obs/'+pick][i,:7];finger=float(h['next_obs/q'][i,r['robot_joint_names'].index('finger_joint')])
    for n in spec.rigid_objects:
     if n!=pick:world.set_obstacle_pose(n,h['next_obs/'+n][i,:7])
   attachment=np.linalg.inv(kin.fk(q))@transform(pose[:3],pose[3:7]);world.attachment=attachment;world.source_detached=True;world.gripper_joint_position=finger;planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True
   for phase in ('transfer','lower'):
    world.allowed_phase=phase;ignore=(pick,)+(('wooden_bowl',) if phase=='lower' else ());goal=np.asarray(goals[pick][phase]);planner.query_start_q=np.asarray(q).copy();path=planner.plan_joint_goal(q,goal,ignore,1.,phase='transfer');check=planner.validate_path(path['positions'],ignore,1.,phase=phase)
    if not check['valid']:raise RuntimeError(phase+': '+str(check))
    q=path['positions'][-1];row[phase+'_positions']=np.asarray(path['positions']).tolist()
   body=kin.fk(q)@attachment;pose=np.r_[body[:3,3],Rotation.from_matrix(body[:3,:3]).as_quat()];world.attachment=None;world.allowed_phase='release';opening=[world.clearance(q,(pick,'wooden_bowl'),mode='execution',gripper_q=grip_joints(g),object_pose=pose) for g in np.linspace(finger,0,13)]
   if not all(c['valid'] for c in opening):raise RuntimeError('Opening: '+str(min(opening,key=lambda c:c['slack_m'])))
   row['passed']=True
  except Exception as exc:row['error']=str(exc)
  rows.append(row);print('ROUND17',variant,pick,row['passed'],str(row.get('error'))[:200],flush=True)
 report['cases'][variant]=rows;out.write_text(json.dumps(report,indent=2))
report['all_passed']=all(x['passed'] for rows in report['cases'].values() for x in rows);out.write_text(json.dumps(report,indent=2));print('ROUND17_DONE',report['all_passed'],flush=True)
