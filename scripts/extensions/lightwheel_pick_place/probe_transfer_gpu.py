"""Bounded no-Kit cuRobo diagnosis of saved physical transfer-start/IK states."""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--goals',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--positions',type=Path);p.add_argument('--checks-only',action='store_true');p.add_argument('--asset',type=Path);a=p.parse_args()
r=json.loads((a.episode/'result.json').read_text());goals=json.loads(a.goals.read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2])
stage=Usd.Stage.Open(str(a.episode/'scene.usda'))
if a.asset:
 stage.RemovePrim('/World/envs/env_0/barrier');p=UsdGeom.Xform.Define(stage,'/World/envs/env_0/barrier');p.GetPrim().GetReferences().AddReference(str(a.asset.resolve()));p.AddTranslateOp().Set(Gf.Vec3d(*r['variant_spec']['position']))
world=CollisionWorld(stage,kin)
with h5py.File(a.episode/'trajectory.h5') as f:
 q=f['next_obs/q'][-1];home=f['obs/q'][0,:7]
 for name in ('wooden_bowl','broccoli','sweet_potato','red_bell_pepper'):world.set_obstacle_pose(name,f['next_obs/'+name][-1,:7])
world.attachment=np.array(r['planning_attachment_tcp_to_lime']);world.gripper_joint_position=float(q[r['robot_joint_names'].index('finger_joint')]);world.allowed_phase='transfer'
planner=LimePlanner(world,kin,dt=r['dt'],seed=r['seed'],max_joint_speed=.4)
rows=[x for x in goals['rows'] if 'q' in x and x.get('valid')];states=np.array([x['q'] for x in rows]);reports=[]
positions=json.loads(a.positions.read_text()) if a.positions else [[4.1,-.15,.922],[4.0875,-.15,.922],[4.09,-.10,.922]]
for xyz in positions:
 world.set_obstacle_pose('barrier',np.r_[xyz,[0,0,0,1]]);planner.update_dynamic_obstacle('barrier');planner._prepare_path_query(('lime',),1,'transfer')
 checker=planner.motion_gen.world_coll_checker;enabled=[name for name in planner.obstacle_names if not planner._ignored(name,('lime',))]
 def check(names):
  names=set(names)
  for name in planner.obstacle_names:checker.enable_obstacle(name,enable=name in names)
  return planner.motion_gen.ik_solver.check_valid(planner.args.to_device(states)).detach().cpu().numpy().reshape(-1)
 full=check(enabled);without=check([name for name in enabled if '/barrier/' not in name]);self_only=check([])
 for name in planner.obstacle_names:checker.enable_obstacle(name,enable=name in enabled)
 report={'position':xyz,'goal_candidates':len(rows),'full_valid':full.tolist(),'without_barrier_valid':without.tolist(),'self_and_limits_valid':self_only.tolist(),'selected_joint_plan':None}
 good=np.flatnonzero(full)
 for index in ([] if a.checks_only else sorted(good,key=lambda i:np.linalg.norm(states[i]-q[:7]))[:2]):
  try:
   path=planner.plan_joint_goal(q[:7],states[index],('lime',),1,phase='transfer');validation=planner.validate_path(path['positions'],('lime',),1,phase='transfer')
   report['selected_joint_plan']={'candidate_index':int(index),'q_goal':states[index].tolist(),'validation':validation,'positions':path['positions'].tolist(),'metadata':path['metadata']}
   if validation['valid']:break
  except RuntimeError as exc:report.setdefault('joint_plan_failures',[]).append(str(exc))
 world.attachment=None;world.gripper_joint_position=0.;planner._prepare_path_query(('lime',),0,'pregrasp');report['home_full_valid']=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(home[None])).all());world.attachment=np.array(r['planning_attachment_tcp_to_lime']);world.gripper_joint_position=float(q[r['robot_joint_names'].index('finger_joint')])
 reports.append(report);print('POSE',xyz,'FULL_GOALS',int(full.sum()),'NO_BARRIER',int(without.sum()),'SELF',int(self_only.sum()),'HOME',report['home_full_valid'],'PATH',bool(report.get('selected_joint_plan',{} ) and report['selected_joint_plan']['validation']['valid']),flush=True)
result={'source_episode':str(a.episode),'target':goals['target'],'reports':reports,'scope':'Saved genuine transfer start, CPU unchanged-target IK solutions, cuRobo full world/self checks and independently dense FCL paths. No PhysX execution.'};a.output.write_text(json.dumps(result,indent=2))
