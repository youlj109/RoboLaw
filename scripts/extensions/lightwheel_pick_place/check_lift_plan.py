"""Read-only cuRobo/FCL lift preflight from the saved genuine closed grasp."""
import json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
root=Path('datasets/skill_trials/lightwheel_pick_place/base_attempt11');r=json.loads((root/'result.json').read_text());b=r['robot_base_pose'];k=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);w=CollisionWorld(Usd.Stage.Open(str(root/'scene.usda')),k)
with h5py.File(root/'trajectory.h5') as f:
 q=f['next_obs/q'][-1];w.attachment=np.array(r['planning_attachment_tcp_to_lime']);w.gripper_joint_position=float(q[7]);w.allowed_phase='lift'
 for n in ('wooden_bowl','broccoli','sweet_potato','red_bell_pepper'):w.set_obstacle_pose(n,f['next_obs/'+n][-1,:7])
p=LimePlanner(w,k,dt=r['dt']);target=k.fk(q[:7]);target[2,3]+=.16
plan=p.plan_pose(q[:7],target,('lime',),1,phase='lift');validation=p.validate_path(plan['positions'],('lime',),1,phase='lift')
report={'phase':'lift','validation':validation,'metadata':plan['metadata'],'positions':plan['positions'].tolist()};Path('reports/skill_trials/lightwheel_pick_place/lift_preflight.json').write_text(json.dumps(report,indent=2));print(json.dumps({'validation':validation,'proxy':p.collision_geometry.get('carried_lime_proxy')},indent=2))
raise SystemExit(0 if validation['valid'] else 2)
