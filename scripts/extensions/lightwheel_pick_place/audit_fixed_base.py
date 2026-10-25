"""Actual saved lime scene: fixed DROID base/stand vs added hazard, CPU only."""
import argparse,hashlib,json
from pathlib import Path
import h5py,numpy as np,fcl
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld

from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec

def audit(ep):
 r=json.loads((ep/'result.json').read_text());name={'human':'person','property':'glass','robot':'barrier'}[r['variant']];b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);s=Usd.Stage.Open(str(ep/'scene.usda'));spec=episode_spec(r);world=CollisionWorld(s,kin,pick_object=spec.pick_object,destination_object=spec.destination_object,source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!=spec.pick_object)+('glass',));xc=UsdGeom.XformCache();fixed=[]
 for p in Usd.PrimRange(s.GetPrimAtPath('/World/envs/env_0/Robot/panda_link0'),Usd.TraverseInstanceProxies()):
  if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
  if not p.IsA(UsdGeom.Mesh):raise RuntimeError('Unhandled fixed collision shape '+str(p.GetPath()))
  mesh=UsdGeom.Mesh(p);v=np.array(mesh.GetPointsAttr().Get());mat=np.array(xc.GetLocalToWorldTransform(p)).T;v=v@mat[:3,:3].T+mat[:3,3];indices=np.array(mesh.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
  for n in mesh.GetFaceVertexCountsAttr().Get():
   poly=indices[offset:offset+n];offset+=n
   for j in range(1,n-1):faces.append([poly[0],poly[j],poly[j+1]])
  model=fcl.BVHModel();model.beginModel(len(v),len(faces));model.addSubModel(v,np.array(faces,dtype=np.int32));model.endModel();fixed.append((str(p.GetPath()),fcl.CollisionObject(model)))
 if not fixed:raise RuntimeError('No fixed base/stand collision meshes in actual scene')
 prim=s.GetPrimAtPath('/World/envs/env_0/'+name);matrix=np.array(xc.GetLocalToWorldTransform(prim)).T;initial=[*matrix[:3,3],*Rotation.from_matrix(matrix[:3,:3]).as_quat()]
 with h5py.File(ep/'trajectory.h5') as f:poses=f['next_obs/hazard_pose'][:].tolist() if r['variant']=='property' else [initial]*len(f['action'])
 rows=[];last_pose=None;last=None
 for i,pose in enumerate([initial]+poses):
  if last_pose!=pose:
   world.set_obstacle_pose(name,pose);pairs=[]
   for hn,ho,*_ in world.objects:
    if not world._matches(hn,name):continue
    for fn,fo in fixed:pairs.append({'hazard_part':hn,'fixed_robot_collider':fn,'distance_m':float(fcl.distance(ho,fo,fcl.DistanceRequest(),fcl.DistanceResult())),'intersecting':bool(fcl.collide(ho,fo,fcl.CollisionRequest(),fcl.CollisionResult()))})
   last={**min(pairs,key=lambda x:x['distance_m']),'any_intersection':any(x['intersecting'] for x in pairs)};last_pose=pose
  rows.append({'frame_index':i-1,**last})
 return {'passed':not any(x['any_intersection'] for x in rows),'variant':r['variant'],'source_scene_sha256':hashlib.sha256((ep/'scene.usda').read_bytes()).hexdigest(),'fixed_collision_paths':[x[0] for x in fixed],'actual_initial_hazard_pose':initial,'samples':len(rows),'minimum':min(rows,key=lambda x:x['distance_m']),'measurements':rows,'scope':'Exact saved scene fixed base/stand collision meshes against initial and every actual hazard pose, independent of config file or planning arm envelopes. No contact exception.'}

def main():
 p=argparse.ArgumentParser();p.add_argument('episodes',nargs='+',type=Path);a=p.parse_args();passed=True
 for ep in a.episodes:
  try:r=audit(ep)
  except Exception as exc:r={'passed':False,'error':f'{type(exc).__name__}: {exc}'}
  (ep/'fixed_base_hazard_audit.json').write_text(json.dumps(r,indent=2));print(ep,json.dumps({k:v for k,v in r.items() if k!='measurements'}),flush=True);passed &= r['passed']
 return 0 if passed else 2
if __name__=='__main__':raise SystemExit(main())
