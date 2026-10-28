"""CPU diagnosis of the cuRobo inflated-vase rejection; no GPU execution."""
import runpy,xml.etree.ElementTree as ET,yaml,numpy as np,json
from pathlib import Path
from scipy.spatial.transform import Rotation
s=runpy.run_path(str(Path(__file__).with_name('diagnose_property_retreat.py')));w=s['w'];k=s['k'];path=s['path'];hazard=s['hazard'];grip=s['grip'];root=Path(__file__).resolve().parents[3];asset=root/'assets/curobo/droid';cfg=yaml.safe_load((asset/'franka_robotiq_2f_85_zero_curobo.yml').read_text())['robot_cfg']['kinematics'];joints=ET.parse(asset/'urdf/franka_robotiq_2f_85_zero.urdf').getroot().findall('joint');cal=np.array(json.loads((asset/'tcp_calibration.json').read_text())['T_urdfEE_USDtcp']);all_spheres=[];fk_errors=[]
for qi,q in enumerate(path):
 vals={f'panda_joint{i+1}':v for i,v in enumerate(q)}|grip;frames={'panda':k.base};pending=joints.copy()
 while pending:
  old=len(pending)
  for j in pending.copy():
   p=j.find('parent').get('link');c=j.find('child').get('link')
   if p not in frames:continue
   o=j.find('origin');T=np.eye(4)
   if o is not None:T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
   if j.get('type') in ('revolute','continuous'):
    R=np.eye(4);R[:3,:3]=Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*vals.get(j.get('name'),0)).as_matrix();T=T@R
   frames[c]=frames[p]@T;pending.remove(j)
  if len(pending)==old:raise RuntimeError([j.get('name') for j in pending])
 fk_errors.append(float(np.linalg.norm((frames['base_link']@cal)[:3,3]-k.fk(q)[:3,3])))
 spheres=[]
 for link,entries in cfg['collision_spheres'].items():
  if link not in frames:continue
  for si,v in enumerate(entries):
   T=frames[link];spheres.append((link,si,T[:3,:3]@v['center']+T[:3,3],v['radius']+.005))
 all_spheres.append(spheres)
rows=[]
for dy in (0,-.01,-.02,-.04,-.06):
 h=hazard.copy();h[1]+=dy;w.set_obstacle_pose('glass',h);obs=[(n,lo-.06,hi+.06) for n,_,lo,hi in w.objects if w._matches(n,'glass')];minimum=None;bad=[]
 for qi,spheres in enumerate(all_spheres):
  for link,si,c,r in spheres:
   for name,lo,hi in obs:
    gap=float(np.linalg.norm(np.maximum(np.maximum(lo-c,c-hi),0))-r)
    if minimum is None or gap<minimum['gap_m']:minimum={'sample':qi,'link':link,'sphere':si,'obstacle':name,'gap_m':gap}
    if gap<0:bad.append(qi)
 rows.append({'delta_y_m':dy,'invalid_samples':sorted(set(bad)),'minimum':minimum})
report={'scope':'CPU exact URDF collision sphere centres (+5mm buffer) versus per-vase-collider AABBs inflated60mm, matching cuRobo world approximation; not GPU solver/self-collision validation.','maximum_fk_calibration_error_m':max(fk_errors),'candidates':rows};(root/'reports/skill_trials/replicator/replicator_retreat_sphere_review.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
