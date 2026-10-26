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
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--start-only',action='store_true');a=p.parse_args()
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
if a.start_only:
 gq=dict(zip(world.GRIPPER_JOINT_NAMES,[float(h) for h in (finger,finger,-finger,finger,-finger,-finger)]))
 report={'scope':'CPU actual saved start clearance only','episode':str(a.episode),'active_pick':active,'checks':{}}
 for mode in ['planning','execution']:
  report['checks'][mode]=world.clearance(q,(active,),mode=mode,gripper_q=gq,object_pose=pose)
  report['checks'][mode+'_glass']=world.clearance(q,(active,),mode=mode,gripper_q=gq,object_pose=pose,obstacle_filter=('glass',))
 a.output.write_text(json.dumps(report,indent=2));print(json.dumps(report));raise SystemExit(0)
report={'scope':'Preserved real start and payload pose, offline planning/dense geometry only. No physical success claim.','episode':str(a.episode),'active_pick':active,'target':target.tolist(),'passed':False}
try:
 planner=LimePlanner(world,kin,dt=r['dt'],seed=r['seed'],max_joint_speed=.4);planner.pose_ik_joint_fallback=True
 path=plan_transfer(planner,q,target,(active,),1.);check=planner.validate_path(path['positions'],(active,),1.,phase='transfer');report.update(passed=bool(check['valid']),validation=check,metadata=path['metadata']);np.save(a.output.with_suffix('.positions.npy'),path['positions'])
except RuntimeError as e:report['error']=str(e)
a.output.write_text(json.dumps(report,indent=2));print('SAVED_TRANSFER_PROBE',r['variant'],report['passed'],str(report.get('error',''))[:400],flush=True)
