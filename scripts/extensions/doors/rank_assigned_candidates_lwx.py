"""CPU initial cuRobo sphere model screening, independent of physical necessity."""
import argparse,json,itertools,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--variant',required=True);p.add_argument('--search',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
root=Path('.');m=json.loads((a.baseline/'metadata.json').read_text());search=json.loads(a.search.read_text());asset=root/'assets/curobo/droid';cfg=yaml.safe_load((asset/'franka_robotiq_2f_85_zero_curobo.yml').read_text())['robot_cfg']['kinematics'];joints=ET.parse(asset/'urdf/franka_robotiq_2f_85_zero.urdf').getroot().findall('joint');config=json.loads(a.config.read_text());spec=json.loads(Path('configs/extensions/doors/front_variants_lwx_final.json').read_text())[a.variant].copy();spec.pop('support',None);spec['position']=[0,0,0];spec['yaw_deg']=0
if search['candidates']:spec['asset']=search['candidates'][0]['asset']
stage,kin,w,g=reconstruct(m,config,spec);angles=dict(zip(m['robot_joint_names'],m['initial_q']));frames={'panda':kin.base};pending=joints.copy()
while pending:
 count=len(pending)
 for j in pending.copy():
  parent=j.find('parent').get('link');child=j.find('child').get('link')
  if parent not in frames:continue
  o=j.find('origin');T=np.eye(4);T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
  if j.get('type') not in ['fixed','floating']:T[:3,:3]=T[:3,:3]@Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*angles.get(j.get('name'),0.)).as_matrix()
  frames[child]=frames[parent]@T;pending.remove(j)
 if len(pending)==count:raise RuntimeError('unresolved URDF')
spheres=[(link,i,T[:3,:3]@s['center']+T[:3,3],s['radius']+.005) for link,T in frames.items() for i,s in enumerate(cfg['collision_spheres'].get(link,[]))];centers=np.asarray([x[2] for x in spheres]);radii=np.asarray([x[3] for x in spheres]);rows=[]
for index,c in enumerate(search['candidates']):
 w.set_obstacle_pose(spec['name'],[*c['position'],*Rotation.from_euler('z',c['yaw_deg'],degrees=True).as_quat()]);haz=[(n,lo-w.clearances[spec['name']],hi+w.clearances[spec['name']]) for n,_,lo,hi in w.objects if w._matches(n,spec['name'])]
 if c.get('proposed_support'):
  support=c['proposed_support']
  haz.extend(('support',np.asarray(part['low']),np.asarray(part['high'])) for part in support.get('parts',[support]))
 scores=[]
 if a.variant=='human':
  import fcl
  # cuRobo retains human triangles. A whole-mesh AABB is only a broad phase
  # and cannot reject a geometrically clear initial state near a raised arm.
  for n,obj,lo,hi in w.objects:
   if not w._matches(n,spec['name']):continue
   values=[]
   for center,radius in zip(centers,radii):
    sphere=fcl.CollisionObject(fcl.Sphere(float(radius)),fcl.Transform(center))
    raw=float(fcl.distance(sphere,obj,fcl.DistanceRequest(),fcl.DistanceResult()))
    hit=bool(fcl.collide(sphere,obj,fcl.CollisionRequest(),fcl.CollisionResult()))
    values.append((-radius if hit else max(raw,0.))-w.clearances[spec['name']])
   j=int(np.argmin(values));scores.append({'slack_m':float(values[j]),'hazard':n,'link':spheres[j][0],'sphere':spheres[j][1],'geometry':'source human triangle mesh versus source cuRobo sphere including .005m buffer; paired GPU checks remain mandatory'})
 else:
  for n,lo,hi in haz:
   distances=np.linalg.norm(np.maximum(np.maximum(lo-centers,centers-hi),0),axis=1)-radii;j=int(np.argmin(distances));scores.append({'slack_m':float(distances[j]),'hazard':n,'link':spheres[j][0],'sphere':spheres[j][1]})
 rows.append({'candidate':index,'initial_sphere':min(scores,key=lambda s:s['slack_m']),'blocking_actions':len(c['baseline_intersection_actions'])})
rows.sort(key=lambda x:(-x['initial_sphere']['slack_m'],-x['blocking_actions']));
if a.output.exists():raise FileExistsError(a.output)
a.output.write_text(json.dumps(rows,indent=2));print(a.baseline,a.variant,rows[:12])
