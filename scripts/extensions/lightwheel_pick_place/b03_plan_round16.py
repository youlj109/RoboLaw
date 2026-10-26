"""One fixed-scene placement correction per scene from saved real lift states."""
import json
from pathlib import Path
import numpy as np,h5py
from scipy.spatial.transform import Rotation
from paired_planning import build_world,grip_joints
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
root=Path('datasets/skill_trials/lightwheel_pick_place');out=Path('/root/autodl-tmp/B03-review/planning_round_16.json')
if out.exists():raise FileExistsError(out)
base='base_lgd_b03_round10_20261026T150435Z';robot='robot_lgd_b03_round10_20261026T152357Z';prop='property_lgd_b03_round10_20261026T154515Z'
refs={}
for pick,episode in [('banana_1','human_lgd_b03_fixedscene_20261026T092630Z'),('banana_2',base)]:
 r=json.load(open(root/episode/'result.json'))
 with h5py.File(root/episode/'trajectory.h5') as h:
  bounds=r['pick_action_ranges'][pick];ph=h['phase'].asstr()[:];i=int(np.flatnonzero((ph=='release')&(np.arange(len(ph))>=bounds['start'])&(np.arange(len(ph))<bounds['end']))[0]);v=h['obs/'+pick][i,:7];b=h['obs/wooden_bowl'][i,:7];refs[pick]={'offset':(v[:3]-b[:3]).tolist(),'quaternion':v[3:7].tolist()}
refs['banana_1']['offset'][2]+=.040
refs['banana_1']['offset'][0]=-.040
refs['banana_1']['offset'][1]=-.025
refs['banana_2']['offset'][0]=.100
refs['banana_2']['offset'][1]=.030
report={'scope':'One goal correction per frozen scene. First release raised20mm from human reference; second uses successful matched baseline pre-release orientation and X=100mm Y=30mm. No physics, environment moves or fallback.','goals':refs,'cases':{}}
for variant,sources in {'base':[base,base],'robot':[robot,robot],'property':[prop,'property_lgd_b03_fixedscene_20261026T104135Z']}.items():
 rows=[]
 for n,source in enumerate(sources,1):
  if n==2:
   rows.append(dict(json.load(open('/root/autodl-tmp/B03-review/planning_round_15.json'))['cases'][variant][1],reused_unchanged_from='planning_round_15.json'));continue
  pick='banana_'+str(n);episode=root/source;r=json.load(open(episode/'result.json'));spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(episode,kin,None,{},task_spec=spec);world.bind_pick(pick);row={'pick':pick,'source':source,'passed':False}
  try:
   with h5py.File(episode/'trajectory.h5') as h:
    ph=h['phase'].asstr()[:];bounds=r['pick_action_ranges'][pick];i=int(np.flatnonzero((ph=='lift')&(np.arange(len(ph))>=bounds['start'])&(np.arange(len(ph))<bounds['end']))[-1]);ids=[r['robot_joint_names'].index('panda_joint'+str(j)) for j in range(1,8)];q=h['next_obs/q'][i,ids];pose=h['next_obs/'+pick][i,:7];dest=h['next_obs/wooden_bowl'][i,:7];finger=float(h['next_obs/q'][i,r['robot_joint_names'].index('finger_joint')])
    for obj in spec.rigid_objects:
     if obj!=pick:world.set_obstacle_pose(obj,h['next_obs/'+obj][i,:7])
   if variant=='property' and n==2:
    cached=json.load(open('/root/autodl-tmp/B03-review/planning_round_04.loaded.json'))['cases']['property']['segments'][-1]['grasp_planning'];q=np.asarray(cached['q_lift_end']);v=kin.fk(q)@np.asarray(cached['attachment_tcp_to_object']);pose=np.r_[v[:3,3],Rotation.from_matrix(v[:3,:3]).as_quat()]
   attachment=np.linalg.inv(kin.fk(q))@transform(pose[:3],pose[3:7]);world.attachment=attachment;world.source_detached=True;world.allowed_phase='transfer';world.gripper_joint_position=finger
   goal=transform(dest[:3]+np.asarray(refs[pick]['offset']),refs[pick]['quaternion'])@np.linalg.inv(attachment)
   planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True
   if n==2:
    endpoint=kin.ik(goal,q,attempts=1);fk=kin.fk(endpoint)
    if np.linalg.norm(fk[:3,3]-goal[:3,3])>.002 or np.rad2deg(Rotation.from_matrix(fk[:3,:3]@goal[:3,:3].T).magnitude())>1:raise RuntimeError('IK misses fixed goal')
    if not world.valid(endpoint,(pick,)):raise RuntimeError('Fixed goal collision: '+str(world.last_clearance))
    planner.query_start_q=np.asarray(q).copy();path=planner.plan_joint_goal(q,endpoint,(pick,),1.,phase='transfer')
   else:path=planner.plan_pose(q,goal,(pick,),1.,phase='transfer')
   check=planner.validate_path(path['positions'],(pick,),1.,phase='transfer')
   if not check['valid']:raise RuntimeError(str(check))
   qend=path['positions'][-1];body=kin.fk(qend)@attachment;bodypose=np.r_[body[:3,3],Rotation.from_matrix(body[:3,:3]).as_quat()]
   world.attachment=None;up=kin.fk(qend).copy();up[2,3]+=.06
   release=planner.plan_cartesian_line(qend,up,(pick,),0.,phase='lift');release_q=np.asarray(release['positions'])
   opening=[world.clearance(qq,(pick,),mode='execution',gripper_q=grip_joints(g),object_pose=bodypose) for qq,g in zip(release_q,np.linspace(finger,0,len(release_q)))]
   row['release_retract_positions']=release_q.tolist()
   if not all(c['valid'] for c in opening):raise RuntimeError('Opening: '+str(min(opening,key=lambda c:c['slack_m'])))
   row.update(passed=True,target=goal.tolist(),positions=np.asarray(path['positions']).tolist(),opening_min=min(opening,key=lambda c:c['slack_m']))
  except Exception as exc:row['error']=str(exc)
  rows.append(row);print('ROUND16',variant,pick,row['passed'],row.get('error'),flush=True)
 report['cases'][variant]=rows;out.write_text(json.dumps(report,indent=2))
report['all_passed']=all(r['passed'] for rows in report['cases'].values() for r in rows);out.write_text(json.dumps(report,indent=2));print('ROUND16_DONE',report['all_passed'],flush=True)
