"""Plan from a preserved actual failed transfer start, without PhysX replay."""
import argparse,json,hashlib
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from paired_planning import build_world
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.extensions.lightwheel_pick_place.b03_strategy import plan_transfer
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--start-only',action='store_true');p.add_argument('--target-x-delta',type=float,default=0.);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
r=json.load(open(a.episode/'result.json'));spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(a.episode,kin,None,{},task_spec=spec)
with h5py.File(a.episode/'trajectory.h5') as h:
 i=len(h['phase'])-1;active=next(n for n,v in r['pick_action_ranges'].items() if v['start']<=i<v['end']);world.bind_pick(active)
 arm=[r['robot_joint_names'].index('panda_joint'+str(j)) for j in range(1,8)];q=h['next_obs/q'][-1,arm];pose=h['next_obs/'+active][-1,:7];dest=h['next_obs/'+spec.destination_object][-1,:7]
 finger=float(h['next_obs/q'][-1,r['robot_joint_names'].index('finger_joint')])
 for n in spec.rigid_objects:
  if n!=active:world.set_obstacle_pose(n,h['next_obs/'+n][-1,:7])
 if 'next_obs/hazard_pose' in h:world.set_obstacle_pose('glass',h['next_obs/hazard_pose'][-1,:7])
tcp=kin.fk(q);world.attachment=np.linalg.inv(tcp)@transform(pose[:3],pose[3:7]);world.source_detached=True;world.allowed_phase='transfer';world.gripper_joint_position=finger
m=r['task_motion_parameters'];x=m['dual_first_place_offset_x_m'] if active==spec.pick_object else m['dual_second_place_offset_x_m'];target=tcp.copy();target[:3,3]=dest[:3]+[x,0.,spec.transfer_z_m]+(tcp[:3,3]-pose[:3])
target[0,3]+=a.target_x_delta
if a.start_only:
 gq=dict(zip(world.GRIPPER_JOINT_NAMES,[float(h) for h in (finger,finger,-finger,finger,-finger,-finger)]))
 report={'scope':'CPU actual saved start clearance only','episode':str(a.episode),'active_pick':active,'checks':{}}
 for mode in ['planning','execution']:
  report['checks'][mode]=world.clearance(q,(active,),mode=mode,gripper_q=gq,object_pose=pose)
  report['checks'][mode+'_glass']=world.clearance(q,(active,),mode=mode,gripper_q=gq,object_pose=pose,obstacle_filter=('glass',))
 a.output.write_text(json.dumps(report,indent=2));print(json.dumps(report));raise SystemExit(0)
from curobo.types.robot import JointState
import fcl
planner=LimePlanner(world,kin,dt=r['dt'],seed=r['seed'],max_joint_speed=.4);planner.pose_ik_joint_fallback=True;planner.query_start_q=np.array(q);planner._prepare_path_query((active,),1.,'transfer')
state=JointState.from_position(planner.args.to_device(np.asarray(q)[None]),joint_names=['panda_joint'+str(j) for j in range(1,8)])
def query():
 ik=bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(np.asarray(q)[None])).all())
 try:
  mg,status=planner.motion_gen.check_start_state(state);return {'ik_valid':ik,'motiongen_valid':bool(mg),'status':str(status)}
 finally:
  planner.motion_gen.rollout_fn.primitive_collision_constraint.enable_cost();planner.motion_gen.rollout_fn.robot_self_collision_constraint.enable_cost()
report={'scope':'Exact saved start, collision-checker diagnostic only, no physical success claim.','episode':str(a.episode),'with_payload':query()}
planner.motion_gen.detach_object_from_robot();report['without_payload']=query();planner._sync_gripper_and_attachment(1.)
checker=planner.motion_gen.world_coll_checker;names=[n for n in planner.obstacle_names if not planner._ignored(n,(active,))]
for n in names:checker.enable_obstacle(n,enable=False)
report['payload_without_world']=query();planner.motion_gen.detach_object_from_robot();report['robot_without_world']=query();planner._sync_gripper_and_attachment(1.)
for n in names:checker.enable_obstacle(n,enable=True)
if not report['with_payload']['motiongen_valid'] and report['payload_without_world']['motiongen_valid']:
 def invalid(group):
  chosen=set(group)
  for n in names:checker.enable_obstacle(n,enable=n in chosen)
  return not query()['motiongen_valid']
 bad=[]
 def bisect(group):
  if not group or not invalid(group):return
  if len(group)==1:bad.extend(group);return
  m=len(group)//2;bisect(group[:m]);bisect(group[m:])
 bisect(names);report['world_colliders']=bad
 for n in names:checker.enable_obstacle(n,enable=True)
gq=dict(zip(world.GRIPPER_JOINT_NAMES,[finger,finger,-finger,finger,-finger,-finger]));shapes=world.shapes(q,gq,pose);payload=next(o for label,o,*rest in shapes if label=='carried_lime');links=kin.links(q);distances=[]
for body,model,vertices in world.robot_hulls:
 t=links[body];obj=fcl.CollisionObject(model,fcl.Transform(t[:3,:3],t[:3,3]));d=float(fcl.distance(payload,obj,fcl.DistanceRequest(),fcl.DistanceResult()));distances.append({'robot_link':body,'distance_m':d})
report['payload_to_original_robot_hulls']=distances
q=kin.ik(target,q,attempts=12);state=JointState.from_position(planner.args.to_device(np.asarray(q)[None]),joint_names=['panda_joint'+str(j) for j in range(1,8)])
report['goal_with_payload']=query();report['goal_full_fcl']=world.clearance(q,(active,),mode='planning',gripper_q=gq)
for n in names:
 if world._matches(n,'glass'):checker.enable_obstacle(n,enable=False)
report['goal_without_glass']=query()
a.output.write_text(json.dumps(report,indent=2));print('BACKEND_START_DIAGNOSTIC',json.dumps(report),flush=True)
