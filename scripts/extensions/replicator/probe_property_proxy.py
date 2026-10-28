"""CPU URDF sphere/padded-collider diagnosis for baseline non-routing segments."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import yaml,numpy as np
from scipy.spatial.transform import Rotation
from search_avoidance import load_baseline,BASE
from audit import hazard_stage
from kitchen_safety.extensions.replicator.layout import resolve_layout
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.planning import DroidKinematics
ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--report',required=True);a=p.parse_args();l=json.loads(Path(a.layout).read_text());r,d,names,ids=load_baseline(BASE);k=DroidKinematics(np.array(l['robot_position'])+[0,0,l['stand_height']],l['robot_yaw_deg']);stage,_=hazard_stage(l,'property',l['obstacle']);w=CollisionWorld(stage,k);asset=ROOT/'assets/curobo/droid';cfg=yaml.safe_load((asset/'franka_robotiq_2f_85_zero_curobo.yml').read_text())['robot_cfg']['kinematics'];joints=ET.parse(asset/'urdf/franka_robotiq_2f_85_zero.urdf').getroot().findall('joint');cal=np.array(json.loads((asset/'tcp_calibration.json').read_text())['T_urdfEE_USDtcp']);rows=[];fk_errors=[]
oblow=np.array([lo-.06 for _,_,lo,hi in w.objects]);obhigh=np.array([hi+.06 for _,_,lo,hi in w.objects]);obnames=[x[0] for x in w.objects]
for i in range(0,len(d['q']),2):
 phase=d['phases'][i]
 if phase not in ['lift','lower','retreat']:continue
 q=d['q'][i,ids];grip={n:float(d['q'][i,names.index(n)]) for n in w.GRIPPER_JOINT_NAMES};vals={f'panda_joint{j+1}':v for j,v in enumerate(q)}|grip;frames={'panda':k.base};pending=joints.copy()
 while pending:
  old=len(pending)
  for j in pending.copy():
   parent=j.find('parent').get('link');child=j.find('child').get('link')
   if parent not in frames:continue
   o=j.find('origin');T=np.eye(4)
   if o is not None:T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
   if j.get('type') in ('revolute','continuous'):
    A=np.eye(4);A[:3,:3]=Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*vals.get(j.get('name'),0)).as_matrix();T=T@A
   frames[child]=frames[parent]@T;pending.remove(j)
  assert len(pending)<old
 fk_errors.append(float(np.linalg.norm((frames['base_link']@cal)[:3,3]-k.fk(q)[:3,3])));centers=[];radii=[];labels=[]
 for link,entries in cfg['collision_spheres'].items():
  if link not in frames:continue
  for si,v in enumerate(entries):
   T=frames[link];centers.append(T[:3,:3]@v['center']+T[:3,3]);radii.append(v['radius']+.005);labels.append(link+'/'+str(si))
 if phase in ['lift','lower']:
  points=np.array(np.meshgrid(np.linspace(-.0167,.0167,3),np.linspace(-.022,.022,3),np.linspace(-.07,.07,10),indexing='ij')).reshape(3,-1).T;b=d['mustard'][i,:7];centers.extend(points@Rotation.from_quat(b[3:]).as_matrix().T+b[:3]);radii.extend(np.minimum(.019,.077-np.abs(points[:,2])));labels.extend(['attached_bottle']*len(points))
 centers=np.array(centers);radii=np.array(radii);gap=np.linalg.norm(np.maximum(np.maximum(oblow[None]-centers[:,None],centers[:,None]-obhigh[None]),0),axis=-1)-radii[:,None];ix,jx=np.unravel_index(np.argmin(gap),gap.shape);rows.append(dict(index=i,phase=phase,gap_m=float(gap[ix,jx]),sphere=labels[ix],obstacle=obnames[jx]))
report=dict(passed=all(x['gap_m']>=0 for x in rows),layout=a.layout,scope='CPU cuRobo robot URDF spheres +5mm buffer and measured bottle sphere approximation versus vase per-collider AABB padded60mm. Baseline lift/lower/retreat sampled every2 action frames. No CUDA solver/self or continuous-physics claim.',max_fk_error_m=max(fk_errors),minimum=min(rows,key=lambda x:x['gap_m']),phase_minima={phase:min([x for x in rows if x['phase']==phase],key=lambda x:x['gap_m']) for phase in ['lift','lower','retreat']},rows=rows);Path(a.report).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
