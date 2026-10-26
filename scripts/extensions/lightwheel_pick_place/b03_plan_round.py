"""One coherent placement modification per frozen scene; no physics or retries."""
import argparse,json
from pathlib import Path
import numpy as np,h5py
from scipy.spatial.transform import Rotation
from paired_planning import build_world,grip_joints
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
p=argparse.ArgumentParser();p.add_argument('--second-release-y',type=float,default=0.);p.add_argument('--second-clearance-z',type=float);p.add_argument('--keep-second-wrist',action='store_true');p.add_argument('--collision-aware-ik',action='store_true');p.add_argument('--reuse-passed',type=Path);p.add_argument('--placement-seed-reference',type=Path);p.add_argument('--direct-second-release',action='store_true');p.add_argument('--grasp-cache',type=Path);p.add_argument('--joint-placement',action='store_true');p.add_argument('--align-second-grasp',action='store_true');p.add_argument('--clearance-z',type=float,default=0.);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
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
 'base':['base_lgd_b03_reference_20261026T134019Z']*2,
 'robot':['robot_lgd_b03_fixedscene_20261026T113926Z']*2,
 'property':['property_lgd_b03_fixedscene_20261026T104135Z']*2}
report={'scope':'One placement-policy modification per scene: reproduce each successful pre-release body pose, no clamped contact with bowl. Complete candidate consists of transport, vertical lower, planned opening sweep. No PhysX rollout or environment movement. Failed case retained, next case attempted; no retry.','native_success_claimed':False,'reference_stage':'obs at first release action, before opening','release_reference':release_refs,'clearance_z_m':a.clearance_z,'align_second_grasp':bool(a.align_second_grasp),'joint_placement':bool(a.joint_placement),'direct_second_release':bool(a.direct_second_release),'second_release_y_m':a.second_release_y,'second_clearance_z_m':a.second_clearance_z,'keep_second_wrist':bool(a.keep_second_wrist),'collision_aware_ik':bool(a.collision_aware_ik),'placement_seed_reference':str(a.placement_seed_reference) if a.placement_seed_reference else None,'cases':{}}
for variant,sources in cases.items():
 if a.reuse_passed:
  prior=json.load(open(a.reuse_passed))['cases'][variant]
  if prior['planning_passed']:
   report['cases'][variant]=dict(prior,reused_unchanged_from=str(a.reuse_passed));print('ROUND_REUSED',variant,flush=True);continue
 rows=[]
 for number,source in enumerate(sources,1):
  if a.align_second_grasp and number==1:
   prior=json.load(open('/root/autodl-tmp/B03-review/planning_round_03.json'))['cases'][variant]['segments'][0]
   rows.append(dict(prior,reused_unchanged_from='planning_round_03.json'));continue
  episode=root/source;r=json.load(open(episode/'result.json'));spec=episode_spec(r);pick='banana_'+str(number);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(episode,kin,None,{},task_spec=spec);world.bind_pick(pick)
  row={'pick':pick,'source_real_episode':str(episode),'planning_passed':False}
  try:
   with h5py.File(episode/'trajectory.h5') as h:
    phases=h['phase'].asstr()[:];bounds=r['pick_action_ranges'][pick];hit=np.flatnonzero((phases=='lift')&(np.arange(len(phases))>=bounds['start'])&(np.arange(len(phases))<bounds['end']));i=int(hit[-1]);ids=[r['robot_joint_names'].index('panda_joint'+str(j)) for j in range(1,8)];q=h['next_obs/q'][i,ids];pose=h['next_obs/'+pick][i,:7];dest=h['next_obs/wooden_bowl'][i,:7];finger=float(h['next_obs/q'][i,r['robot_joint_names'].index('finger_joint')])
    if a.align_second_grasp:
     i=int(np.flatnonzero((phases=='pregrasp')&(np.arange(len(phases))>=bounds['start'])&(np.arange(len(phases))<bounds['end']))[0]);q=h['obs/q'][i,ids];pose=h['obs/'+pick][i,:7];dest=h['obs/wooden_bowl'][i,:7]
     close_index=int(np.flatnonzero((phases=='close')&(np.arange(len(phases))>=bounds['start'])&(np.arange(len(phases))<bounds['end']))[-1]);finger=float(h['next_obs/q'][close_index,r['robot_joint_names'].index('finger_joint')])
    for n in spec.rigid_objects:
     if n!=pick:world.set_obstacle_pose(n,h[('obs/' if a.align_second_grasp else 'next_obs/')+n][i,:7])
   if a.align_second_grasp and not a.grasp_cache:
    from kitchen_safety.extensions.lightwheel_pick_place.b03_strategy import preflight_grasp,collision_mesh_waist,PAD_CALIBRATION_PATH
    model=r['extra_source_support_models'][pick];world.source_support_mode=r['source_support_contact_model']['mode'];world.source_surface_top=model['source_surface_top_m'];world.source_resting_min_z=model['source_resting_min_z_m'];world.source_resting_root=np.asarray(model['source_resting_root_m']);world.source_resting_xy=world.source_resting_root[:2];world.source_detached=False
    object_yaw=float(Rotation.from_quat(pose[3:7]).as_euler('xyz',degrees=True)[2]);rotation=Rotation.from_euler('z',180.+object_yaw,degrees=True).as_matrix()@np.array([[0,-1,0],[0,0,1],[-1,0,0]],float)
    local=collision_mesh_waist(world,y_shift=.011);nominal=(transform(pose[:3],pose[3:7])@np.r_[local,1.])[:3];pad=np.asarray(json.load(open(PAD_CALIBRATION_PATH))['pad_center_tcp_m']);center=nominal+[0.,0.,.0023]-rotation@pad
    gp=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);gp.single_candidate_round=True
    grasp=preflight_grasp(gp,q,rotation,center,np.array([0.,0.,.08]),pose,pick,closed_grip_candidates=(finger,),contact_standoff_m=0.,lift_m=.10)
    q=np.asarray(grasp['q_lift_end']);predicted=kin.fk(q)@np.asarray(grasp['attachment_tcp_to_object']);pose=np.r_[predicted[:3,3],Rotation.from_matrix(predicted[:3,:3]).as_quat()];row['grasp_planning']=grasp;row['grasp_world_yaw_deg']=180.+object_yaw
   if a.grasp_cache:
    cached=json.load(open(a.grasp_cache))['cases'][variant]['segments'][-1]
    grasp=cached['grasp_planning'];q=np.asarray(grasp['q_lift_end']);predicted=kin.fk(q)@np.asarray(grasp['attachment_tcp_to_object']);pose=np.r_[predicted[:3,3],Rotation.from_matrix(predicted[:3,:3]).as_quat()];row['grasp_planning']=grasp;row['reused_grasp_from']=str(a.grasp_cache)
   tcp=kin.fk(q);attachment=np.linalg.inv(tcp)@transform(pose[:3],pose[3:7]);world.attachment=attachment;world.source_detached=True;world.allowed_phase='transfer';world.gripper_joint_position=finger
   position=dest[:3]+np.array(release_refs[pick]['offset'])+[0.,a.second_release_y,(0. if a.direct_second_release else .045)+(a.clearance_z if a.second_clearance_z is None else a.second_clearance_z)];body_q=(pose[3:7] if a.keep_second_wrist else release_refs[pick]['quaternion']);target=transform(position,body_q)@np.linalg.inv(attachment)
   planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True
   def placement_query(start,goal_pose):
    if not a.joint_placement:return planner.plan_pose(start,goal_pose,(pick,),1.,phase='transfer')
    seed=(np.asarray(json.load(open(a.placement_seed_reference))['cases']['property']['segments'][-1]['transport_positions'][-1]) if a.placement_seed_reference else start)
    goal=kin.ik(goal_pose,seed,attempts=1);fk=kin.fk(goal)
    if np.linalg.norm(fk[:3,3]-goal_pose[:3,3])>.002 or np.rad2deg(Rotation.from_matrix(fk[:3,:3]@goal_pose[:3,:3].T).magnitude())>1.:raise RuntimeError('Joint goal missed unchanged TCP pose')
    if not world.valid(goal,(pick,)) and a.collision_aware_ik:
     from scipy.optimize import minimize
     start_goal=goal.copy()
     def equality(v):
      t=kin.fk(v)
      return np.r_[t[:3,3]-goal_pose[:3,3],Rotation.from_matrix(t[:3,:3]@goal_pose[:3,:3].T).as_rotvec()]
     def clearance(v):return world.clearance(v,(pick,),mode='planning')['slack_m']-.001
     opt=minimize(lambda v:float(np.sum((v-start_goal)**2)),goal,method='SLSQP',bounds=[(float(lo)+1e-6,float(hi)-1e-6) for lo,hi in kin.limits],constraints=[{'type':'eq','fun':equality},{'type':'ineq','fun':clearance}],options={'maxiter':60,'ftol':1e-9,'eps':1e-5})
     goal=np.asarray(opt.x);e=equality(goal);row['collision_aware_ik']={'success':bool(opt.success),'iterations':int(opt.nit),'message':str(opt.message),'position_error_m':float(np.linalg.norm(e[:3])),'orientation_error_deg':float(np.rad2deg(np.linalg.norm(e[3:]))),'goal_q':goal.tolist()}
     if np.linalg.norm(e[:3])>.002 or np.rad2deg(np.linalg.norm(e[3:]))>1.:raise RuntimeError('Collision-constrained IK missed unchanged TCP goal: '+str(row['collision_aware_ik']))
    if not world.valid(goal,(pick,)):raise RuntimeError('Unchanged TCP joint goal fails original FCL: '+str(world.last_clearance))
    planner.query_start_q=np.asarray(start).copy()
    return planner.plan_joint_goal(start,goal,(pick,),1.,phase='transfer')
   row['stage']='transport'
   carry=placement_query(q,target);checked=planner.validate_path(carry['positions'],(pick,),1.,phase='transfer')
   if not checked['valid']:raise RuntimeError('Transport collision check failed: '+str(checked))
   endpoint=planner.kin.fk(carry['positions'][-1]);lower=endpoint.copy();lower[2,3]-=(0. if a.direct_second_release else .045)
   row['stage']='lower'
   down=({'positions':np.asarray(carry['positions'][-1:])} if a.direct_second_release else placement_query(carry['positions'][-1],lower));check_lower=planner.validate_path(down['positions'],(pick,),1.,phase='transfer')
   if not check_lower['valid']:raise RuntimeError('Free-release lower collision check failed: '+str(check_lower))
   qend=down['positions'][-1];object_end=kin.fk(qend)@attachment;object_pose=np.r_[object_end[:3,3],Rotation.from_matrix(object_end[:3,:3]).as_quat()];opening=[]
   row['stage']='opening'
   for angle in np.linspace(finger,0.,13):
    c=world.clearance(qend,(pick,),mode='execution',gripper_q=grip_joints(angle),object_pose=object_pose);opening.append(c)
   if not all(c['valid'] for c in opening):raise RuntimeError('Planned opening sweep fails: '+str(min(opening,key=lambda c:c['slack_m'])))
   row.update(planning_passed=True,transport_validation=checked,lower_validation=check_lower,opening_minimum=min(opening,key=lambda c:c['slack_m']),transport_positions=np.asarray(carry['positions']).tolist(),lower_positions=np.asarray(down['positions']).tolist())
  except Exception as exc:row['error']=type(exc).__name__+': '+str(exc)
  rows.append(row);print('ROUND_SCENE',variant,pick,row['planning_passed'],row.get('error'),flush=True)
  if not row['planning_passed']:break
 report['cases'][variant]={'planning_passed':len(rows)==2 and all(x['planning_passed'] for x in rows),'segments':rows}
 a.output.write_text(json.dumps(report,indent=2))
report['all_planning_passed']=all(x['planning_passed'] for x in report['cases'].values());a.output.write_text(json.dumps(report,indent=2));print('ROUND_COMPLETE',{k:v['planning_passed'] for k,v in report['cases'].items()},flush=True)
