"""One coherent placement modification per frozen scene; no physics or retries."""
import argparse,json
from pathlib import Path
import numpy as np,h5py
from scipy.spatial.transform import Rotation
from paired_planning import build_world,grip_joints
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
p=argparse.ArgumentParser();p.add_argument('--clearance-z',type=float,default=0.);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
root=Path('datasets/skill_trials/lightwheel_pick_place')
reference_path=root/'human_lgd_b03_fixedscene_20261026T092630Z'
reference=json.load(open(reference_path/'result.json'));assert reference['success'];release_refs={}
with h5py.File(reference_path/'trajectory.h5') as f:
 phases=f['phase'].asstr()[:]
 for name,bounds in reference['pick_action_ranges'].items():
  index=int(np.flatnonzero((phases=='release')&(np.arange(len(phases))>=bounds['start'])&(np.arange(len(phases))<bounds['end']))[0]);body=f['obs/'+name][index,:7];bowl=f['obs/wooden_bowl'][index,:7]
  release_refs[name]={'offset':(body[:3]-bowl[:3]).tolist(),'quaternion':body[3:7].tolist()}

cases={
 'base':['base_lgd_b03_free_20261026T130223Z']*2,
 'robot':['robot_lgd_b03_fixedscene_20261026T113926Z']*2,
 'property':['property_lgd_b03_fixedscene_20261026T104135Z']*2}
report={'scope':'One placement-policy modification per scene: reproduce each successful pre-release body pose, no clamped contact with bowl. Complete candidate consists of transport, vertical lower, planned opening sweep. No PhysX rollout or environment movement. Failed case retained, next case attempted; no retry.','native_success_claimed':False,'reference_stage':'obs at first release action, before opening','release_reference':release_refs,'clearance_z_m':a.clearance_z,'cases':{}}
for variant,sources in cases.items():
 rows=[]
 for number,source in enumerate(sources,1):
  episode=root/source;r=json.load(open(episode/'result.json'));spec=episode_spec(r);pick='banana_'+str(number);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(episode,kin,None,{},task_spec=spec);world.bind_pick(pick)
  row={'pick':pick,'source_real_episode':str(episode),'planning_passed':False}
  try:
   with h5py.File(episode/'trajectory.h5') as h:
    phases=h['phase'].asstr()[:];bounds=r['pick_action_ranges'][pick];hit=np.flatnonzero((phases=='lift')&(np.arange(len(phases))>=bounds['start'])&(np.arange(len(phases))<bounds['end']));i=int(hit[-1]);ids=[r['robot_joint_names'].index('panda_joint'+str(j)) for j in range(1,8)];q=h['next_obs/q'][i,ids];pose=h['next_obs/'+pick][i,:7];dest=h['next_obs/wooden_bowl'][i,:7];finger=float(h['next_obs/q'][i,r['robot_joint_names'].index('finger_joint')])
    for n in spec.rigid_objects:
     if n!=pick:world.set_obstacle_pose(n,h['next_obs/'+n][i,:7])
   tcp=kin.fk(q);attachment=np.linalg.inv(tcp)@transform(pose[:3],pose[3:7]);world.attachment=attachment;world.source_detached=True;world.allowed_phase='transfer';world.gripper_joint_position=finger
   position=dest[:3]+np.array(release_refs[pick]['offset'])+[0.,0.,.045+a.clearance_z];body_q=release_refs[pick]['quaternion'];target=transform(position,body_q)@np.linalg.inv(attachment)
   planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True
   carry=planner.plan_pose(q,target,(pick,),1.,phase='transfer');checked=planner.validate_path(carry['positions'],(pick,),1.,phase='transfer')
   if not checked['valid']:raise RuntimeError('Transport collision check failed: '+str(checked))
   endpoint=planner.kin.fk(carry['positions'][-1]);lower=endpoint.copy();lower[2,3]-=.045
   down=planner.plan_pose(carry['positions'][-1],lower,(pick,),1.,phase='transfer');check_lower=planner.validate_path(down['positions'],(pick,),1.,phase='transfer')
   if not check_lower['valid']:raise RuntimeError('Free-release lower collision check failed: '+str(check_lower))
   qend=down['positions'][-1];object_end=kin.fk(qend)@attachment;object_pose=np.r_[object_end[:3,3],Rotation.from_matrix(object_end[:3,:3]).as_quat()];opening=[]
   for angle in np.linspace(finger,0.,13):
    c=world.clearance(qend,(pick,),mode='execution',gripper_q=grip_joints(angle),object_pose=object_pose);opening.append(c)
   if not all(c['valid'] for c in opening):raise RuntimeError('Planned opening sweep fails: '+str(min(opening,key=lambda c:c['slack_m'])))
   row.update(planning_passed=True,transport_validation=checked,lower_validation=check_lower,opening_minimum=min(opening,key=lambda c:c['slack_m']),transport_positions=np.asarray(carry['positions']).tolist(),lower_positions=np.asarray(down['positions']).tolist())
  except RuntimeError as exc:row['error']=str(exc)
  rows.append(row);print('ROUND_SCENE',variant,pick,row['planning_passed'],row.get('error'),flush=True)
  if not row['planning_passed']:break
 report['cases'][variant]={'planning_passed':len(rows)==2 and all(x['planning_passed'] for x in rows),'segments':rows}
 a.output.write_text(json.dumps(report,indent=2))
report['all_planning_passed']=all(x['planning_passed'] for x in report['cases'].values());a.output.write_text(json.dumps(report,indent=2));print('ROUND_COMPLETE',{k:v['planning_passed'] for k,v in report['cases'].items()},flush=True)
