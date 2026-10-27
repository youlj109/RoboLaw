"""One fixed constant-wrist goal; bounded redundant IK solver, then one checked joint plan. No PhysX."""
from pathlib import Path
import json,h5py,numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
p=Path('datasets/skill_trials/lightwheel_pick_place/robot_lgd_b04_round20_20261027T105950Z');r=json.loads((p/'result.json').read_text());spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);ids=[r['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
with h5py.File(p/'trajectory.h5') as h:
 q=h['next_obs/q'][-1];pose=h['next_obs/green_ladle'][-1,:7];dest=h['next_obs/plate'][-1,:7];poses={n:h['next_obs/'+n][-1,:7] for n in spec.rigid_objects}
world=CollisionWorld(Usd.Stage.Open(str(p/'scene.usda')),kin,pick_object='green_ladle',destination_object='plate',source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!='green_ladle')+('glass',));world.allowed_phase='lower';world.source_detached=True
for n,v in poses.items():
 if n!='green_ladle':world.set_obstacle_pose(n,v)
world.attachment=np.linalg.inv(kin.fk(q[ids]))@transform(pose[:3],pose[3:7]);world.planning_gripper_q={n:float(q[r['robot_joint_names'].index(n)]) for n in world.GRIPPER_JOINT_NAMES};world.gripper_joint_position=world.planning_gripper_q['finger_joint']
target=kin.fk(q[ids]).copy();target[:3,3]+=dest[:3]+[-.03,0,.06]-pose[:3];reference=np.asarray(json.loads(Path('/root/autodl-tmp/B04-review/human09_phase_joint_seeds.json').read_text())['seeds']['transfer']);rng=np.random.default_rng(43);seeds=[q[ids],reference]+[reference+rng.normal(0,.6,7) for _ in range(14)];report={'scope':'One unchanged measured-wrist Cartesian goal,16 internal IK initializations, no environment edits or physical trials','source':str(p),'target':target.tolist(),'candidates':[],'all_passed':False};goal=None
for i,seed in enumerate(seeds):
 try:
  g=kin.ik(target,seed,attempts=1);fk=kin.fk(g);err=float(np.linalg.norm(fk[:3,3]-target[:3,3]));angle=float(Rotation.from_matrix(fk[:3,:3]@target[:3,:3].T).magnitude());c=world.clearance(g,('green_ladle','plate'),mode='execution',gripper_q=world.planning_gripper_q);row={'index':i,'position_error_m':err,'angle_error_rad':angle,'execution_clearance':c};report['candidates'].append(row)
  if err<.002 and angle<np.deg2rad(1) and c['valid']:goal=g;report['selected_index']=i;break
 except RuntimeError as exc:report['candidates'].append({'index':i,'error':str(exc)})
out=Path('/root/autodl-tmp/B04-review/constant_wrist_saved_plan_round22.json');out.write_text(json.dumps(report,indent=2));print('IK_CANDIDATES',report['candidates'],flush=True)
if goal is not None:
 planner=LimePlanner(world,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True;planner.certify_execution_margin=True;planner.query_start_q=q[ids].copy();path=planner.plan_joint_goal(q[ids],goal,('green_ladle','plate'),1.,phase='lower');check=planner.validate_path(path['positions'],('green_ladle','plate'),1.,phase='lower');report.update(all_passed=bool(check['valid']),validation=check,goal=goal.tolist(),positions=np.asarray(path['positions']).tolist())
 if check['valid']:report['joint_goals']={'green_ladle':{'transfer':goal.tolist(),'lower':goal.tolist()}}
 out.write_text(json.dumps(report,indent=2));print('PATH_PASSED',report['all_passed'],flush=True)
