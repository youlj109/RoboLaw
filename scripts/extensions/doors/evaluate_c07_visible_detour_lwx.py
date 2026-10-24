"""Quantify visible TCP detours against unchanged paired baseline; not physical acceptance."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics,transform
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--paired',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
m=json.loads((a.baseline/'metadata.json').read_text());d=json.loads(a.paired.read_text());cams=json.loads((a.baseline/'cameras.json').read_text());kin=DroidKinematics(m['robot_base'],m['robot_yaw_deg']);base=transform(m['robot_base'],Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).as_quat());result={'scope':'Offline TCP geometry and original-camera projection only. Manual video and full physical audits remain required.','paired':str(a.paired),'paired_sha256':hashlib.sha256(a.paired.read_bytes()).hexdigest(),'thresholds':{'min_max_tcp_deviation_m':.10,'min_visible_projection_deviation_px':55.,'max_joint_length_ratio':3.,'max_added_tcp_length_m':.6},'pairs':[]}
def project(points,c):
 pose=base@transform(c['offset']['position'],c['offset']['quaternion_xyzw']);q=(points-pose[:3,3])@pose[:3,:3];z=-q[:,2];k=np.asarray(c['intrinsics']);uv=np.column_stack((k[0,0]*q[:,0]/np.maximum(z,1e-9)+k[0,2],k[1,2]-k[1,1]*q[:,1]/np.maximum(z,1e-9)));inside=(z>.05)&(uv[:,0]>30)&(uv[:,0]<c['width']-30)&(uv[:,1]>30)&(uv[:,1]<c['height']-30);return uv,inside
for pair in d['pairs']:
 r={'seed':pair['seed'],'eligible':False};result['pairs'].append(r)
 if not (pair['off']['success'] and pair['on']['success']):continue
 paths={k:np.asarray([kin.fk(q)[:3,3] for q in pair[k]['positions']]) for k in ['off','on']};off,on=paths['off'],paths['on'];dist=np.linalg.norm(on[:,None,:]-off[None,:,:],axis=2);near=dist.min(1);r['max_tcp_distance_to_baseline_curve_m']=float(near.max());r['on_tcp_world_m']=on.tolist();r['off_tcp_world_m']=off.tolist();r['cameras']={}
 for name,c in cams.items():
  if name=='wrist_camera':continue
  uo,io=project(off,c);un,ine=project(on,c);values=np.linalg.norm(un[:,None,:]-uo[None,io,:],axis=2).min(1) if io.any() else np.zeros(len(on));valid=ine&(near>=.10);r['cameras'][name]={'baseline_visible_fraction':float(io.mean()),'on_visible_fraction':float(ine.mean()),'max_projected_distance_at_10cm_detour_px':float(values[valid].max()) if valid.any() else 0.}
 r['best_visible_projection_deviation_px']=max(x['max_projected_distance_at_10cm_detour_px'] for x in r['cameras'].values());r['joint_length_ratio']=pair['on']['path_metrics']['joint_l2_length_rad']/pair['off']['path_metrics']['joint_l2_length_rad'];r['added_tcp_length_m']=pair['on']['path_metrics']['tcp_length_m']-pair['off']['path_metrics']['tcp_length_m'];r['actual_source_intersection']=any(v.get('intersecting') for v in pair['off']['risk_with_added_geometry']['violations']);r['eligible']=bool(r['actual_source_intersection'] and pair['policy_necessity_supported'] and not pair['target_adaptation'] and r['max_tcp_distance_to_baseline_curve_m']>=.10 and r['best_visible_projection_deviation_px']>=55 and r['joint_length_ratio']<=3 and r['added_tcp_length_m']<=.6)
result['visually_distinct_candidate']=bool(result['pairs']) and all(r['eligible'] for r in result['pairs']);a.output.write_text(json.dumps(result,indent=2)+'\n');print('VISIBLE_DETOUR',result['visually_distinct_candidate'],[(x['seed'],round(x.get('max_tcp_distance_to_baseline_curve_m',0),3),round(x.get('best_visible_projection_deviation_px',0),1)) for x in result['pairs']],flush=True)
