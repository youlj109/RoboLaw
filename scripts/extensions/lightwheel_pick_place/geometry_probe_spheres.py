"""Read-only diagnosis of lime grasp sphere extent versus actual open gripper geometry."""
import json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from pxr import Usd
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
root=Path(__file__).resolve().parents[3];r=json.loads((root/'datasets/skill_trials/lightwheel_pick_place/base_attempt06/result.json').read_text());base=r['robot_base_pose']
kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:7]).as_euler('xyz',degrees=True)[2]);s=Usd.Stage.Open(str(root/'reports/skill_trials/lightwheel_pick_place/inspection/stage.usda'));world=CollisionWorld(s,kin)
asset=root/'assets/curobo/droid';cfg=yaml.safe_load((asset/'franka_robotiq_2f_85_zero_curobo.yml').read_text())['robot_cfg']['kinematics'];cal=np.array(json.loads((asset/'tcp_calibration.json').read_text())['T_urdfEE_USDtcp']);joints=ET.parse(asset/'urdf/franka_robotiq_2f_85_zero.urdf').getroot().findall('joint')
rows=[]
for candidate in json.loads((root/'reports/skill_trials/lightwheel_pick_place/cpu_grasp_probe.json').read_text()):
 if 'q' not in candidate:continue
 q=np.array(candidate['q']);tcp=kin.fk(q);frames={'base_link':tcp@np.linalg.inv(cal)}
 pending=joints.copy()
 while pending:
  count=len(pending)
  for j in pending.copy():
   parent=j.find('parent').get('link');child=j.find('child').get('link')
   if parent not in frames:continue
   o=j.find('origin');T=np.eye(4);T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
   frames[child]=frames[parent]@T;pending.remove(j)
  if len(pending)==count:break
 spheres=[]
 for link,T in frames.items():
  for i,sph in enumerate(cfg['collision_spheres'].get(link,[])):
   center=T[:3,:3]@sph['center']+T[:3,3];radius=sph['radius']+.005
   spheres.append({'link':link,'index':i,'center_world_m':center.tolist(),'radius_m':radius,'bottom_z':float(center[2]-radius),'counter_vertical_gap_m':float(center[2]-radius-.92)})
 closest=min(spheres,key=lambda x:x['bottom_z']); shapes=world.shapes(q,gripper_q=dict.fromkeys(world.GRIPPER_JOINT_NAMES,0.));actual=[(name,float(lo[2])) for name,_,lo,hi in shapes if name.startswith('gripper/')]
 rows.append({k:candidate[k] for k in ('tilt','yaw','offset')}|{'tcp_z':float(tcp[2,3]),'closest_sphere':closest,'actual_gripper_lowest':min(actual,key=lambda x:x[1]),'actual_all_gripper_lows':actual})
print(json.dumps(rows,indent=2));(root/'reports/skill_trials/lightwheel_pick_place/geometry_probe_spheres.json').write_text(json.dumps(rows,indent=2))
