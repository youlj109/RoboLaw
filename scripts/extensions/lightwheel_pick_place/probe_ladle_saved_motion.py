"""No-physics cuRobo check of revised ladle motion from preserved measured states."""
import argparse,json,hashlib
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from paired_planning import build_world
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.extensions.lightwheel_pick_place.b03_strategy import plan_transfer
from kitchen_safety.planning import DroidKinematics,transform
p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
r=json.load(open(a.episode/'result.json'));spec=episode_spec(r)
if spec.pick_object!='green_ladle':raise ValueError('Ladle only')
b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);world,stage=build_world(a.episode,kin,None,{},task_spec=spec)
with h5py.File(a.episode/'trajectory.h5') as f:
 q=f['next_obs/q'][-1,:7];pick=f['next_obs/green_ladle'][-1,:7];dest=f['next_obs/plate'][-1,:7];finger=float(f['next_obs/q'][-1,r['robot_joint_names'].index('finger_joint')]);states={n:f['next_obs/'+n][-1,:7] for n in spec.rigid_objects}
 if 'next_obs/hazard_pose' in f:world.set_obstacle_pose('glass',f['next_obs/hazard_pose'][-1,:7])
for n,pose in states.items():world.set_obstacle_pose(n,pose)
shift=np.r_[spec.lift_offset_xy_m,spec.lift_height_m-float(r.get('lift_m',0.))]
tcp=kin.fk(q);hypothetical_tcp=tcp.copy();hypothetical_tcp[:3,3]+=shift;q=kin.ik(hypothetical_tcp,q,attempts=12);pick[:3]+=shift
world.attachment=np.linalg.inv(kin.fk(q))@transform(pick[:3],pick[3:7]);world.gripper_joint_position=finger;world.source_detached=True;world.allowed_phase='transfer'
goal_pick=transform(dest[:3]+np.r_[spec.place_offset_xy_m,spec.transfer_z_m],pick[3:7]);target=goal_pick@np.linalg.inv(world.attachment)
planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.pose_ik_joint_fallback=True
report={'scope':'Hypothetical revised lift endpoint derived from preserved actual state; cuRobo/self/full dense geometry only, no PhysX step or native task-success claim.','source_episode':str(a.episode),'source_result_sha256':hashlib.sha256((a.episode/'result.json').read_bytes()).hexdigest(),'task_motion':{'lift_height_m':spec.lift_height_m,'lift_offset_xy_m':spec.lift_offset_xy_m,'place_offset_xy_m':spec.place_offset_xy_m},'passed':False}
try:
 path=plan_transfer(planner,q,target,('green_ladle',),1.)
 checked=planner.validate_path(path['positions'],('green_ladle',),1.,phase='transfer')
 report.update(passed=bool(checked['valid']),validation=checked,metadata=path['metadata'],samples=len(path['positions']))
 if report['passed']:np.save(a.output.with_suffix('.positions.npy'),path['positions'])
except RuntimeError as exc:report['error']=str(exc)
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2));print('LADLE_SAVED_MOTION',r['variant'],report['passed'],report.get('samples'),str(report.get('error',''))[:300],flush=True)
