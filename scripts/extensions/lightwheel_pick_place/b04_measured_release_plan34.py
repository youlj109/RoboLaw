"""Saved real post-landing six-joint withdrawal check, no PhysX."""
import json,h5py,numpy as np
from pathlib import Path
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
p=Path('datasets/skill_trials/lightwheel_pick_place/robot_lgd_b04_round33_20261027T145740Z');r=json.loads((p/'result.json').read_text());b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);spec=episode_spec(r);ids=[r['robot_joint_names'].index('panda_joint'+str(i)) for i in range(1,8)]
with h5py.File(p/'trajectory.h5') as h:q=h['next_obs/q'][-1];states={n:h['next_obs/'+n][-1] for n in r['initial_object_states']}
w=CollisionWorld(Usd.Stage.Open(str(p/'scene.usda')),kin,pick_object='green_ladle',destination_object='plate',source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!='green_ladle'))
for n,pose in states.items():
 if n!='green_ladle':w.set_obstacle_pose(n,pose[:7])
w.attachment=None;w.source_detached=True;w.allowed_phase='release';w.planning_gripper_q={n:float(q[r['robot_joint_names'].index(n)]) for n in w.GRIPPER_JOINT_NAMES};w.gripper_joint_position=w.planning_gripper_q['finger_joint']
start=q[ids];tcp=kin.fk(start);previous=start.copy();path=[]
for i in range(1,21):
 target=tcp.copy();target[2,3]+=.002*i;candidate=kin.ik(target,previous,attempts=1)
 for f in np.linspace(0,1,7)[1:]:path.append(previous+(candidate-previous)*f)
 previous=candidate
planner=LimePlanner(w,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.certify_execution_margin=True;planner.query_start_q=start.copy();check=planner.validate_path(path,('green_ladle','plate'),1.,phase='lift')
diagnostic={}
if not check['valid']:
 bad_index=check.get('curobo_failures',[{'sample':0}])[0]['sample'];bad_q=np.asarray(path[bad_index]);checker=planner.motion_gen.world_coll_checker;names=list(planner.obstacle_names)
 def valid(group):
  selected=set(group)
  for n in names:checker.enable_obstacle(n,enable=n in selected)
  return bool(planner.motion_gen.ik_solver.check_valid(planner.args.to_device(bad_q[None])).all())
 diagnostic={'sample':bad_index,'empty_world_valid':valid([]),'blockers':[]}
 def isolate(group):
  if not group or valid(group):return
  if len(group)==1:diagnostic['blockers'].extend(group);return
  m=len(group)//2;isolate(group[:m]);isolate(group[m:])
 if diagnostic['empty_world_valid']:isolate(names)
 diagnostic['actual_fcl']=w.clearance(bad_q,('green_ladle','plate'),mode='execution',gripper_q=w.planning_gripper_q)
 print('WITHDRAWAL_REJECTION',diagnostic,flush=True)
report={'all_passed':bool(check['valid']),'scope':'Actual post-landing six independent/passive joint positions, no ideal mimic assumption; no PhysX','source_episode':str(p),'measured_gripper_q':w.planning_gripper_q,'validation':check,'rejection_diagnostic':diagnostic,'positions':np.asarray(path).tolist()};Path('/root/autodl-tmp/B04-review/measured_release_plan34.json').write_text(json.dumps(report,indent=2));print('MEASURED_SIX_JOINT_RELEASE_PATH',report['all_passed'],flush=True)
