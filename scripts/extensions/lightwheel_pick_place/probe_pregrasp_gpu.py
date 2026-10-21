"""Bounded unchanged-grasp redundant posture comparison; no physical execution."""
import argparse,json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.planner import LimePlanner
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--asset',type=Path,required=True);p.add_argument('--positions',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
r=json.loads((a.baseline/'result.json').read_text());base=r['robot_base_pose'];kin=DroidKinematics(base[:3],Rotation.from_quat(base[3:]).as_euler('xyz',degrees=True)[2]);target=np.array(next(x['target'] for x in r['plans'] if x['phase']=='pregrasp'));rows=[]
with h5py.File(a.baseline/'trajectory.h5') as f:
 q0=f['obs/q'][0,:7];phases=f['phase'].asstr()[:];close_q=f['obs/q'][np.flatnonzero(phases=='close')[0],:7];initial={n:f['obs/'+n][0,:7] for n in ['lime','wooden_bowl','broccoli','sweet_potato','red_bell_pepper']}
for xyz in json.loads(a.positions.read_text()):
 stage=Usd.Stage.Open(str(a.baseline/'scene.usda'));stage.SetEditTarget(stage.GetSessionLayer());x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/barrier');x.GetPrim().GetReferences().AddReference(str(a.asset.resolve()));x.AddTranslateOp().Set(Gf.Vec3d(*xyz));world=CollisionWorld(stage,kin)
 for name,pose in initial.items():world.set_obstacle_pose(name,pose)
 planner=LimePlanner(world,kin,dt=r['dt'],seed=r['seed'],max_joint_speed=.4);row={'position':xyz,'valid':False}
 try:
  path=planner.plan_pose(q0,target,('lime',),0,'pregrasp');end=kin.fk(path['positions'][-1]);end[2,3]-=.125
  column=planner.plan_pose(path['positions'][-1],end,('lime',),0,'approach');qend=column['positions'][-1]
  row.update(valid=True,pregrasp_q=path['positions'][-1].tolist(),approach_q=qend.tolist(),baseline_close_q=close_q.tolist(),q_difference_rad=float(np.linalg.norm(qend-close_q)),approach_pose=kin.fk(qend).tolist(),metadata=path['metadata'])
 except RuntimeError as exc:row['error']=str(exc)
 rows.append(row);print('CANDIDATE',xyz,row.get('q_difference_rad'),row.get('error'),flush=True)
a.output.write_text(json.dumps({'baseline':str(a.baseline),'rows':rows,'scope':'Same seed, home, TCP grasp and source initial poses; compare obstacle-aware redundant arm IK against actually successful baseline grasp.'},indent=2))
