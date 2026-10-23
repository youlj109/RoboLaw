"""Source wrist anchor and bottle bounds relative to the measured robot forward axis."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.doors.reconstruction import reconstruct
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path,required=True)
parser.add_argument('--config',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
if args.output.exists():parser.error('Refusing to overwrite geometry evidence')
m=json.loads((args.baseline/'metadata.json').read_text())
old=json.load(open('configs/extensions/doors/avoidance_variants.json'))
new=json.loads(args.config.read_text())
pose_path=Path('assets/extensions/doors/human_reach_elbow_up/pose.json')
wrist=np.array(json.loads(pose_path.read_text())['actual_wrist_world_asset_m'])
base=np.array(m['robot_base']);yaw=np.deg2rad(m['robot_yaw_deg']);forward=np.array([np.cos(yaw),np.sin(yaw)]);lateral=np.array([-forward[1],forward[0]])
result={'scope':'Static source geometry and declared pose anchor, not physical acceptance or measured live skeleton tracking','robot_base':base.tolist(),'robot_yaw_deg':m['robot_yaw_deg'],'pose_report_sha256':hashlib.sha256(pose_path.read_bytes()).hexdigest(),'config':str(args.config),'config_sha256':hashlib.sha256(args.config.read_bytes()).hexdigest(),'baseline':str(args.baseline),'variants':{}}
for variant in ['human','property']:
 item={}
 for label,config in [('old',old),('new',new)]:
  spec=config[variant];stage,kin,w=reconstruct(m,spec)
  if variant=='human':
   R=Rotation.from_euler('z',spec['baked_yaw_deg']+spec.get('yaw_deg',0),degrees=True).as_matrix()
   anchor=np.asarray(spec['position'])+R@wrist
   verts=np.concatenate([v for n,(v,f) in w.meshes.items() if w._matches(n,spec['name'])])
   nearest=float(np.linalg.norm(verts-anchor,axis=1).min())
  else:
   objs=[o for o in w.objects if w._matches(o[0],spec['name'])];lo=np.min([o[2] for o in objs],axis=0);hi=np.max([o[3] for o in objs],axis=0);anchor=(lo+hi)/2;nearest=None
  item[label]={'anchor_world_m':anchor.tolist(),'forward_m':float((anchor[:2]-base[:2])@forward),'lateral_distance_m':float(abs((anchor[:2]-base[:2])@lateral)),'source_mesh_nearest_vertex_m':nearest}
 result['variants'][variant]=item
args.output.write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
