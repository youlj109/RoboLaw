"""CPU reconstruction of cuRobo sphere-versus-inflated-AABB initial clearance."""
import json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from scripts.extensions.doors.search_avoidance_layouts import context
ROOT=Path(__file__).resolve().parents[3]
ep,meta,records,specs,w,fixed,shapes=context()
asset=ROOT/'assets/curobo/droid';cfg=yaml.safe_load((asset/'franka_robotiq_2f_85_zero_curobo.yml').read_text())['robot_cfg']['kinematics'];joints=ET.parse(asset/'urdf/franka_robotiq_2f_85_zero.urdf').getroot().findall('joint')
angles=dict(zip(meta['robot_joint_names'],records[0]['obs']['q']));frames={'panda':w.kin.base};pending=joints.copy()
while pending:
 count=len(pending)
 for j in pending.copy():
  parent=j.find('parent').get('link');child=j.find('child').get('link')
  if parent not in frames:continue
  o=j.find('origin');T=np.eye(4);T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
  if j.get('type') not in ['fixed','floating']:
   axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');T[:3,:3]=T[:3,:3]@Rotation.from_rotvec(axis*angles.get(j.get('name'),0)).as_matrix()
  frames[child]=frames[parent]@T;pending.remove(j)
 if len(pending)==count:raise RuntimeError('unresolved URDF links')
spheres=[]
for link,T in frames.items():
 for i,sph in enumerate(cfg['collision_spheres'].get(link,[])):
  spheres.append((link,i,T[:3,:3]@sph['center']+T[:3,3],sph['radius']+.005))
def sphere_clearance(hazards,margin):
 centers=np.array([s[2] for s in spheres]);radii=np.array([s[3] for s in spheres]);lo=np.array([h[2] for h in hazards])-margin;hi=np.array([h[3] for h in hazards])+margin
 delta=np.maximum(np.maximum(lo[:,None]-centers[None],centers[None]-hi[:,None]),0);distances=np.linalg.norm(delta,axis=2)-radii[None];hi_idx,si=np.unravel_index(np.argmin(distances),distances.shape);link,j,center,radius=spheres[si]
 return {'slack_m':float(distances[hi_idx,si]),'link':link,'sphere_index':j,'hazard':hazards[hi_idx][0],'center':center.tolist(),'radius':radius}

if __name__=='__main__':
 rows=[]
 for source in ['property_refined_search.json','property_pedestal_search.json','robot_pedestal_search.json']:
  for i,c in enumerate(json.loads((ROOT/'reports/avoidance_trials/doors'/source).read_text())['candidates']):
   name={'property':'glass','robot':'barrier'}[c['variant']];pos=c['position'];w.set_obstacle_pose(name,[*pos,*Rotation.from_euler('z',c['yaw_deg'],degrees=True).as_quat()]);hazards=[o for o in w.objects if w._matches(o[0],name)]
   rows.append({'source':source,'candidate':i,'position':pos,'yaw':c['yaw_deg'],'baseline_distance':c['baseline_minimum']['distance_m'],'planner_initial':sphere_clearance(hazards,w.clearances[name])})
 rows.sort(key=lambda r:-r['planner_initial']['slack_m']);(ROOT/'reports/avoidance_trials/doors/planner_initial_cpu.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows[:12],indent=2))
