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
start=q[ids];target=kin.fk(start).copy();target[2,3]+=.04
planner=LimePlanner(w,kin,dt=r['dt'],seed=43,max_joint_speed=.4);planner.single_candidate_round=True;planner.certify_execution_margin=True;planner.query_start_q=start.copy()
try:
 trajectory=planner.plan_pose(start,target,('green_ladle','plate'),1.,phase='transfer');path=trajectory['positions'];check=planner.validate_path(path,('green_ladle','plate'),1.,phase='transfer')
except RuntimeError as exc:
 path=[start];check={'valid':False,'planning_error':str(exc)}
diagnostic={}
report={'all_passed':bool(check['valid']),'scope':'Actual post-landing six independent/passive joint positions, no ideal mimic assumption; no PhysX','source_episode':str(p),'measured_gripper_q':w.planning_gripper_q,'validation':check,'rejection_diagnostic':diagnostic,'target_tcp':target.tolist(),'positions':np.asarray(path).tolist()};Path('/root/autodl-tmp/B04-review/measured_release_plan35.json').write_text(json.dumps(report,indent=2));print('MEASURED_SIX_JOINT_RELEASE_PATH',report['all_passed'],flush=True)
