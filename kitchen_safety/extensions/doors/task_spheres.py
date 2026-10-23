"""CPU model of the pinned cuRobo initial collision spheres; screening only."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np,yaml
from scipy.spatial.transform import Rotation

def initial_spheres(metadata,kin):
 asset=Path('assets/curobo/droid');cfg=yaml.safe_load((asset/'franka_robotiq_2f_85_zero_curobo.yml').read_text())['robot_cfg']['kinematics'];joints=ET.parse(asset/'urdf/franka_robotiq_2f_85_zero.urdf').getroot().findall('joint')
 angles=dict(zip(metadata['robot_joint_names'],metadata['initial_q']));frames={'panda':kin.base};pending=joints.copy()
 while pending:
  count=len(pending)
  for j in pending.copy():
   parent=j.find('parent').get('link');child=j.find('child').get('link')
   if parent not in frames:continue
   o=j.find('origin');T=np.eye(4);T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
   if j.get('type') not in ['fixed','floating']:T[:3,:3]=T[:3,:3]@Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*angles.get(j.get('name'),0.)).as_matrix()
   frames[child]=frames[parent]@T;pending.remove(j)
  if len(pending)==count:raise RuntimeError('unresolved URDF')
 spheres=[(link,i,T[:3,:3]@s['center']+T[:3,3],s['radius']+.005) for link,T in frames.items() for i,s in enumerate(cfg['collision_spheres'].get(link,[]))]
 return spheres
