"""One bounded saved-state placement proposal, no simulation or environment changes."""
import json,hashlib
from pathlib import Path
import h5py,numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
root=Path('datasets/skill_trials/lightwheel_pick_place');out=Path('/root/autodl-tmp/B04-review/endpoint_round15_robot.json');assert not out.exists()
seeds=json.loads(Path('/root/autodl-tmp/B04-review/human09_phase_joint_seeds.json').read_text())['seeds'];ref=json.loads(Path('/root/autodl-tmp/B04-review/selected_round04_goals.json').read_text())['goals']['green_ladle']['quaternion'];report={'scope':'Saved-state endpoint screening only, bounded four heights and two measured orientations; no PhysX, no obstacle edits, no success claim','cases':{}}
for variant in ('robot',):
 ep=max(root.glob(variant+'_lgd_b04_round14_*'),key=lambda p:p.name);r=json.loads((ep/'result.json').read_text());spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);ids=[r['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
 with h5py.File(ep/'trajectory.h5') as h:
  q=h['next_obs/q'][-1];pose=h['next_obs/green_ladle'][-1,:7];dest=h['next_obs/plate'][-1,:7];poses={n:h['next_obs/'+n][-1,:7] for n in spec.rigid_objects}
 world=CollisionWorld(Usd.Stage.Open(str(ep/'scene.usda')),kin,pick_object='green_ladle',destination_object='plate',source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!='green_ladle')+('glass',))
 for n,v in poses.items():
  if n!='green_ladle':world.set_obstacle_pose(n,v)
 world.attachment=np.linalg.inv(kin.fk(q[ids]))@transform(pose[:3],pose[3:7]);world.source_detached=True;world.allowed_phase='transfer';world.planning_gripper_q={n:float(q[r['robot_joint_names'].index(n)]) for n in world.GRIPPER_JOINT_NAMES}
 rows=[];selected=None
 for quat in (ref,pose[3:7].tolist()):
  for z in (.08,.10,.06,.12):
   offset=[-.03,0,z];target=transform(dest[:3]+offset,quat)@np.linalg.inv(world.attachment);goal=kin.ik(target,np.asarray(seeds['transfer']),attempts=1);actual=kin.fk(goal);err=float(np.linalg.norm(actual[:3,3]-target[:3,3]));ang=float(Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude());c=world.clearance(goal,('green_ladle',),mode='planning');row={'offset':offset,'quaternion':quat,'position_error_m':err,'angle_error_rad':ang,'clearance':c};rows.append(row)
   if err<.002 and ang<np.deg2rad(1) and c['valid']:
    selected={**row,'joint_goal':goal.tolist()};break
  if selected:break
 report['cases'][variant]={'source':str(ep),'selected':selected,'candidates':rows};out.write_text(json.dumps(report,indent=2));print(variant,selected,flush=True)
 if selected:
  Path('/root/autodl-tmp/B04-review/round15_goals_'+variant+'.json').write_text(json.dumps({'all_passed':True,'goals':{'green_ladle':{'offset':selected['offset'],'quaternion':selected['quaternion']}},'evidence':str(out)},indent=2))
