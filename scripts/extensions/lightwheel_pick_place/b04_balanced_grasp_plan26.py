"""Calibrate an actual pad-center grasp at native COM and solve one grasp/place compatibility proposal. No PhysX."""
from pathlib import Path
import json,h5py,hashlib,numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation,Slerp
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
root=Path('datasets/skill_trials/lightwheel_pick_place');ep=root/'property_lgd_b04_round16_20261027T095059Z';r=json.loads((ep/'result.json').read_text());spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);ids=[r['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
contacts=json.loads((ep/'contacts.json').read_text());close=[row for row in contacts if row['phase']=='close'][-5:];pads=[]
with h5py.File(ep/'trajectory.h5') as h:
 qs=h['next_obs/q'][:];ph=h['phase'].asstr()[:];i=int(np.flatnonzero(ph=='release')[0]);native_place_q=h['obs/q'][i,ids];native_place=kin.fk(native_place_q);initial_q=h['obs/q'][0];old_grasp_q=qs[close[-1]['action_index'],ids]
 for row in close:
  points=[]
  for f in row['fingers'].values():
   force=np.abs(np.asarray(f['normal_forces_n']));v=np.asarray(f['points_world_m']).reshape(-1,3)[force>.01];points.append(v.mean(0))
  tcp=kin.fk(qs[row['action_index'],ids]);pads.append(tcp[:3,:3].T@(np.mean(points,axis=0)-tcp[:3,3]))
pad=np.mean(pads,axis=0);com=np.asarray(json.loads(Path('/root/autodl-tmp/B04-review/native_mass_properties_20261026.json').read_text())['native_mass_properties']['green_ladle']['com_local_pose'][0][:3]);initial_body=transform(r['initial_lime_pose'][:3],r['initial_lime_pose'][3:7]);pinch=initial_body[:3,:3]@com+initial_body[:3,3];rotations=Rotation.from_matrix(np.stack([kin.fk(old_grasp_q)[:3,:3],native_place[:3,:3]]));slerp=Slerp([0,1],rotations)
world=CollisionWorld(Usd.Stage.Open(str(root/'robot_lgd_b04_round25_20261027T112600Z'/'scene.usda')),kin,pick_object='green_ladle',destination_object='plate',source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!='green_ladle'))
for n,pose in r['initial_object_states'].items():
 if n!='green_ladle':world.set_obstacle_pose(n,pose[:7])
report={'scope':'Native COM plus measured pad-center calibration, explicit legal grasp adaptation, source physics unchanged, no physical trial','source':str(ep),'pad_center_tcp_m':pad.tolist(),'grasp_body_point_m':com.tolist(),'candidates':[],'all_passed':False};rng=np.random.default_rng(43);out=Path('/root/autodl-tmp/B04-review/balanced_grasp_plan26.json')
for fraction in (1.,.7,.4):
 R=slerp([fraction]).as_matrix()[0];center=pinch-R@pad;contact=transform(center,Rotation.from_matrix(R).as_quat());hover=contact.copy();hover[2,3]+=.08
 for seed_index,seed in enumerate([old_grasp_q,initial_q[ids]]+[old_grasp_q+rng.normal(0,.4,7) for _ in range(4)]):
  row={'orientation_fraction':fraction,'ik_seed':seed_index}
  try:
   world.attachment=None;world.source_detached=False;world.allowed_phase='approach';world.gripper_joint_position=0.;world.planning_gripper_q={n:0. for n in world.GRIPPER_JOINT_NAMES}
   grasp=kin.ik(contact,seed,attempts=1);pre=kin.ik(hover,grasp,attempts=1)
   for name,q in [('grasp',grasp),('pregrasp',pre)]:
    c=world.clearance(q,('green_ladle',),mode='execution',gripper_q=world.planning_gripper_q)
    if not c['valid']:raise RuntimeError(name+str(c))
   lifted_pose=contact.copy();lifted_pose[2,3]+=.12;lifted=kin.ik(lifted_pose,grasp,attempts=1);world.gripper_joint_position=float(qs[close[-1]['action_index'],r['robot_joint_names'].index('finger_joint')]);world.planning_gripper_q={n:float(qs[close[-1]['action_index'],r['robot_joint_names'].index(n)]) for n in world.GRIPPER_JOINT_NAMES};world.attachment=np.linalg.inv(contact)@initial_body;world.source_detached=True;world.allowed_phase='lower'
   for dz in (.02,.04,.06):
    goal_pose=native_place.copy();goal_pose[2,3]+=dz;goal=kin.ik(goal_pose,native_place_q,attempts=1);predicted=kin.fk(goal)@world.attachment;dest=np.asarray(r['initial_bowl_pose'][:3]);c=world.clearance(goal,('green_ladle','plate'),mode='execution',gripper_q=world.planning_gripper_q)
    if not c['valid'] or np.linalg.norm(predicted[:2,3]-dest[:2])>.11:continue
    planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True;planner.certify_execution_margin=True;planner.query_start_q=lifted.copy();path=planner.plan_joint_goal(lifted,goal,('green_ladle','plate'),1.,phase='lower');check=planner.validate_path(path['positions'],('green_ladle','plate'),1.,phase='lower')
    if not check['valid']:continue
    calibration={'rotation_world':R.tolist(),'pad_center_tcp_m':pad.tolist(),'body_point_m':com.tolist(),'pregrasp_offset_world_m':[0,0,.08],'source_episode':str(ep),'orientation_fraction':fraction,'policy':'Balance actual pad midpoint at original native COM; select placement-capable wrist orientation'};Path('/root/autodl-tmp/B04-review/balanced_grasp_calibration26.json').write_text(json.dumps(calibration,indent=2));Path('/root/autodl-tmp/B04-review/balanced_grasp_seeds26.json').write_text(json.dumps({'seeds':{'pregrasp':pre.tolist()}}));report.update(all_passed=True,selected=row,grasp=calibration,joint_goals={'green_ladle':{'transfer':goal.tolist(),'lower':goal.tolist()}},goal_body_pose=predicted.tolist(),hover_dz_m=dz,validation=check,positions=np.asarray(path['positions']).tolist());break
   if report['all_passed']:break
   row['error']='No valid whole-path landing goal'
  except RuntimeError as exc:row['error']=str(exc)
  report['candidates'].append(row);out.write_text(json.dumps(report,indent=2));print('CANDIDATE',row,flush=True)
 if report['all_passed']:break
out.write_text(json.dumps(report,indent=2));print('BALANCED_PLAN_PASSED',report['all_passed'],flush=True)
