"""Reconstruct the exact solved source asset transform; never choose a new layout."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom
from kitchen_safety.planning import transform
p=argparse.ArgumentParser();p.add_argument('baseline');p.add_argument('--output',required=True);a=p.parse_args();ep=Path(a.baseline);out=Path(a.output)
if out.exists():raise FileExistsError(out)
m=json.loads((ep/'metadata.json').read_text());r=json.loads((ep/'result.json').read_text());assert r['accepted']
source=Path.home()/'.cache/isaaclab_arena/usd/robot_on_stand/doors_droid_on_stand_0.8_1.08x0.91032.usd';s=Usd.Stage.Open(str(source));base=next(p for p in s.Traverse() if p.GetName()=='panda_link0');local=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(base)).T;T=transform(m['robot_base'],Rotation.from_euler('z',m['robot_yaw_deg'],degrees=True).as_quat())@np.linalg.inv(local)
d={'baseline_episode':a.baseline,'metadata_sha256':hashlib.sha256((ep/'metadata.json').read_bytes()).hexdigest(),'baseline_accepted':r['accepted'],'robot_asset_pose':{'position':T[:3,3].tolist(),'rotation':Rotation.from_matrix(T[:3,:3]).as_quat().tolist()},'expected_robot_base':m['robot_base'],'expected_robot_yaw_deg':m['robot_yaw_deg'],'source_robot_usd_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'scope':'Asset pose reconstructed exactly from original solved source episode actual panda_link0 pose and composed original robot transform; no hand-selected source relocation.'};out.write_text(json.dumps(d,indent=2)+'\n')
