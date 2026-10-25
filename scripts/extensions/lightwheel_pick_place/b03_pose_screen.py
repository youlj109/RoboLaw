"""CPU-only grasp-pose screen; no MotionGen or physical replay."""
import json,itertools,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import resolve_task,BANANA_TASK
root=Path('datasets/skill_trials/lightwheel_pick_place');ep=root/'base_lgd_b03_20261025T032224Z';r=json.load(open(ep/'result.json'));setup=json.load(open(ep/'initial_setup.json'));b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);q=np.asarray(setup['robot_joint_positions'])[:7];spec=resolve_task(BANANA_TASK);world=CollisionWorld(Usd.Stage.Open(str(ep/'scene.usda')),kin,pick_object='banana_2',destination_object='wooden_bowl',source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!='banana_2'))
pose=np.asarray(r['initial_lime_pose'][:7]);center=(transform(pose[:3],pose[3:7])@np.r_[world.object_center,1])[:3]+[0,0,.02];R0=np.array([[0,-1,0],[0,0,1],[-1,0,0]],float);rows=[]
for yaw,tilt,standoff in itertools.product([180,0,-90,90],[-35,-20,0],[.12,.18]):
 R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@R0;T=np.eye(4);T[:3,:3]=R;T[:3,3]=center-standoff*R[:,2];row={'yaw':yaw,'tilt':tilt,'standoff':standoff,'hover':T[:3,3].tolist()}
 try:
  goal=kin.ik(T,q,attempts=3);actual=kin.fk(goal);row.update(ik=True,position_error_m=float(np.linalg.norm(actual[:3,3]-T[:3,3])),orientation_error_deg=float(np.rad2deg(Rotation.from_matrix(T[:3,:3]@actual[:3,:3].T).magnitude())));c=world.clearance(goal,('banana_2',),mode='execution');row['endpoint_clearance']=c;row['q_goal']=goal.tolist()
 except Exception as exc:row.update(ik=False,error=str(exc))
 rows.append(row);print('POSE',yaw,tilt,standoff,row['ik'],row.get('endpoint_clearance',{}).get('valid'),row.get('error'),flush=True)
p=Path('reports/skill_trials/lightwheel_pick_place/b03_cpu_pose_screen_20261025.json');p.write_text(json.dumps({'scope':'Endpoint-only IK and geometry filter; not a path or physical success certificate','rows':rows},indent=2)+'\n')
