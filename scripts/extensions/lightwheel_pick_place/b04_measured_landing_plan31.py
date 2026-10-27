"""One saved-state measured-attachment landing correction; no PhysX execution."""
from pathlib import Path
import json,h5py,numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
root=Path('datasets/skill_trials/lightwheel_pick_place');ep=root/'robot_lgd_b04_round27_20261027T122711Z';r=json.loads((ep/'result.json').read_text());spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);ids=[r['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
with h5py.File(ep/'trajectory.h5') as h:
 i=int(np.flatnonzero(h['phase'].asstr()[:]=='release')[0])-31
 q=h['obs/q'][i];states={n:h['obs/'+n][i] for n in r['initial_object_states']}
world=CollisionWorld(Usd.Stage.Open(str(ep/'scene.usda')),kin,pick_object='green_ladle',destination_object='plate',source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!='green_ladle'))
for n,pose in states.items():
 if n!='green_ladle':world.set_obstacle_pose(n,pose[:7])
world.attachment=np.linalg.inv(kin.fk(q[ids]))@transform(states['green_ladle'][:3],states['green_ladle'][3:7]);world.source_detached=True;world.allowed_phase='lower';world.planning_gripper_q={n:float(q[r['robot_joint_names'].index(n)]) for n in world.GRIPPER_JOINT_NAMES};world.gripper_joint_position=world.planning_gripper_q['finger_joint']
reference=json.loads((root/'human_lgd_b04_round09_20261027T081735Z'/'result.json').read_text());quat=reference['placement_body_goals']['green_ladle']['quaternion'];rng=np.random.default_rng(43)
report={'scope':'Correct actual post-transport ladle tilt using measured attachment, native successful body orientation, no physics trial','source_episode':str(ep),'prefix_actions':i,'actual_attachment':world.attachment.tolist(),'all_passed':False,'candidates':[]}
for height in (.08,.10,.12):
 body=transform(states['plate'][:3]+np.array([-.03,0,height]),quat);tcp=body@np.linalg.inv(world.attachment)
 for seed_index,seed in enumerate([q[ids]]+[q[ids]+rng.normal(0,.35,7) for _ in range(5)]):
  row={'height_m':height,'seed':seed_index}
  try:
   goal=kin.ik(tcp,seed,attempts=1);actual=kin.fk(goal)
   if np.linalg.norm(actual[:3,3]-tcp[:3,3])>.001 or Rotation.from_matrix(actual[:3,:3].T@tcp[:3,:3]).magnitude()>.02:raise RuntimeError('IK residual')
   c=world.clearance(goal,('green_ladle','plate'),mode='execution',gripper_q=world.planning_gripper_q)
   if not c['valid']:raise RuntimeError(str(c))
   planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True;planner.certify_execution_margin=True;planner.query_start_q=q[ids].copy()
   path=planner.plan_joint_goal(q[ids],goal,('green_ladle','plate'),1.,phase='lower');check=planner.validate_path(path['positions'],('green_ladle','plate'),1.,phase='lower')
   if not check['valid']:raise RuntimeError(str(check))
   report.update(all_passed=True,selected=row,joint_goal=goal.tolist(),positions=np.asarray(path['positions']).tolist(),body_goal=body.tolist(),validation=check);break
  except RuntimeError as exc:row['error']=str(exc)
  report['candidates'].append(row)
 if report['all_passed']:break
Path('/root/autodl-tmp/B04-review/measured_landing_plan31.json').write_text(json.dumps(report,indent=2));print('MEASURED_LANDING_PLAN',report['all_passed'],report.get('selected'),flush=True)
