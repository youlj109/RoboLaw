"""Calibrate pad midpoint versus opening width from actual loaded contact points and original USD joints."""
import json,hashlib
from pathlib import Path
import numpy as np,h5py,fcl
from scipy.spatial.transform import Rotation
from scipy.optimize import brentq
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.planning import transform
ref=json.load(open('configs/extensions/lightwheel_pick_place/b03_pad_contact_calibration.json'));ep=Path(ref['source_episode']);r=json.load(open(ep/'result.json'));records=json.load(open(ep/'contacts.json'));close=[x for x in records if x['phase']=='close'][-5:]
w=CollisionWorld.__new__(CollisionWorld);w.fcl=fcl;w._gripper_geometry();paths={n:next(p for p in w._gripper_links(dict(zip(w.GRIPPER_JOINT_NAMES,np.zeros(6)))) if p.endswith('/'+n)) for n in ('left_inner_finger','right_inner_finger')};material={n:[] for n in paths};parity=[]
with h5py.File(ep/'trajectory.h5') as h:
 for row in close:
  i=row['action_index'];q=h['next_obs/q'][i];poses=h['next_obs/robot_body_pose_w'][i];tcp=np.array(h['next_obs/tcp'][i]);R=Rotation.from_quat(h['next_obs/tcp_quat_xyzw'][i]).as_matrix();predicted=[]
  links=w._gripper_links({n:float(q[r['robot_joint_names'].index(n)]) for n in w.GRIPPER_JOINT_NAMES})
  for name in paths:
   contact=row['fingers'][name];points=np.array(contact['points_world_m']);force=np.abs(np.array(contact['normal_forces_n']));mask=force>.01
   if not mask.any():raise RuntimeError('Reference must have simultaneous bilateral load')
   point=np.average(points[mask],axis=0,weights=force[mask]);pose=poses[r['robot_body_names'].index(name)];local=Rotation.from_quat(pose[3:7]).inv().apply(point-pose[:3]);material[name].append(local);prediction=(links[paths[name]]@np.r_[local,1])[:3]-[.131,0,0];measured=R.T@(point-tcp);parity.append(float(np.linalg.norm(prediction-measured)))
if max(parity)>.001:raise RuntimeError('Native/USD contact frame parity failed: '+str(parity))
material={n:np.mean(v,axis=0) for n,v in material.items()}
def pads(angle):
 links=w._gripper_links(dict(zip(w.GRIPPER_JOINT_NAMES,np.array([1,1,-1,1,-1,-1])*angle)))
 return np.array([(links[paths[n]]@np.r_[material[n],1])[:3]-[.131,0,0] for n in paths])
def width(angle):return float(abs(np.diff(pads(angle)[:,1])[0]))
rows=[]
for name,opening in [('ladle_handle',.017),('pepsi_can',.063)]:
 angle=brentq(lambda a:width(a)-opening,0.,.8);p=pads(angle);row={'object':name,'target_opening_m':opening,'predicted_joint_rad':float(angle),'pad_center_tcp_m':p.mean(axis=0).tolist(),'scope':'Geometric grasp calibration, not physical task success'};rows.append(row);print(row,flush=True)
report={'scope':'Native loaded pad material points transported through original USD articulated joint frames. No physics, mass, friction, collision or object pose edits. Not task success.','source_episode':str(ep),'source_sha256':ref['source_sha256'],'native_usd_frame_max_error_m':max(parity),'material_points_finger_local_m':{n:p.tolist() for n,p in material.items()},'opening_samples':[{'joint_rad':a,'width_m':width(a),'center_tcp_m':pads(a).mean(axis=0).tolist()} for a in (0.,.2,.4,.6,.78)],'objects':rows}
out=Path('configs/extensions/lightwheel_pick_place/pad_width_calibration_20261026.json')
with out.open('x') as f:json.dump(report,f,indent=2)
