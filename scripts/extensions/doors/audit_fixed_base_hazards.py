"""CPU supplement: actual hazard poses against exact source fixed base/stand.

The articulated world checker excludes Robot descendants. This separate audit
covers every collision mesh under the composed panda_link0 (including stand),
using the recorded base pose. It does not replay dynamics or change episodes.
"""
import argparse,hashlib,json
from pathlib import Path
import h5py,numpy as np,fcl
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics,transform
from kitchen_safety.extensions.doors.collision import DoorCollisionWorld
ROOT=Path(__file__).resolve().parents[3]
cache=Path.home()/'.cache/isaaclab_arena/usd/robot_on_stand'
GROUPS={'doors':(['human_attempt_02','property_attempt_01','robot_attempt_01'],'doors_droid_on_stand_0.8_1.08x0.91032.usd'),
 'controls':(['human_attempt02','property_attempt01','robot_attempt01'],'controls_droid_original_stand_0.8_1.08x0.91032.usd'),
 'lightwheel_pick_place':([],'lime_droid_original_stand_0.8_1.08x0.91032.usd')}
parser=argparse.ArgumentParser();parser.add_argument('episodes',nargs='*',help='Optional accepted episode paths under datasets/skill_trials/{doors,controls,lightwheel_pick_place}')
args=parser.parse_args()
if args.episodes:
 selected={}
 for raw in args.episodes:
  ep=Path(raw).resolve();group=ep.parent.name
  if group not in GROUPS:raise ValueError('Unsupported group episode: '+raw)
  selected.setdefault(group,[]).append(ep.name)
 GROUPS={g:(names,GROUPS[g][1]) for g,names in selected.items()}
summary={}
for group,(episodes,cachefile) in GROUPS.items():
 source=cache/cachefile;s=Usd.Stage.Open(str(source));xc=UsdGeom.XformCache();base=next(p for p in s.Traverse() if p.GetName()=='panda_link0');basepath=str(base.GetPath());sourcebase=np.array(xc.GetLocalToWorldTransform(base)).T;local_models=[]
 for p in Usd.PrimRange(base,Usd.TraverseInstanceProxies()):
  if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
  if not p.IsA(UsdGeom.Mesh):raise RuntimeError('Unhandled fixed collision primitive '+str(p.GetPath()))
  m=UsdGeom.Mesh(p);v=np.array(m.GetPointsAttr().Get());mat=np.linalg.inv(sourcebase)@np.array(xc.GetLocalToWorldTransform(p)).T;v=v@mat[:3,:3].T+mat[:3,3];indices=np.array(m.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
  for n in m.GetFaceVertexCountsAttr().Get():
   polygon=indices[offset:offset+n];offset+=n
   for j in range(1,n-1):faces.append([polygon[0],polygon[j],polygon[j+1]])
  model=fcl.BVHModel();model.beginModel(len(v),len(faces));model.addSubModel(v,np.array(faces,dtype=np.int32));model.endModel();local_models.append((str(p.GetPath()),model,v))
 if not local_models:raise RuntimeError('No fixed base collision geometry')
 for episode in episodes:
  ep=ROOT/'datasets/skill_trials'/group/episode
  if group=='doors':
   meta=json.loads((ep/'metadata.json').read_text());spec=json.loads((ep/'hazard_clearance.json').read_text())['variant_spec'] if (ep/'hazard_clearance.json').exists() else json.loads((ROOT/'configs/extensions/doors/open_fridge_variants.json').read_text())[meta['variant']];pos=meta['robot_base'];quat=Rotation.from_euler('z',meta['robot_yaw_deg'],degrees=True).as_quat();records=json.loads((ep/'trajectory.json').read_text());poses=[x['next_obs'].get('hazard_pose') for x in records]
  elif group=='controls':
   meta=json.loads((ep/'result.json').read_text());spec=(json.loads((ep/'augmentation_audit.json').read_text())['hazard'] if (ep/'augmentation_audit.json').exists() else json.loads((ROOT/'configs/extensions/controls/left_button_trial.json').read_text())['variants'][meta['variant']]).copy();spec['name']={'human':'person','property':'glass','robot':'barrier'}[meta['variant']];pos=meta['initial_robot_pose'][:3];quat=meta['initial_robot_pose'][3:7];records=json.loads((ep/'records.json').read_text());poses=[x['after'].get('hazard_pose') for x in records]
  else:
   meta=json.loads((ep/'result.json').read_text());old=ep/'fixed_base_hazard_audit.json';spec=(json.loads(old.read_text())['variant_spec'] if old.exists() else json.loads((ROOT/'configs/extensions/lightwheel_pick_place/variants.json').read_text())[meta['variant']]).copy();pos=meta['robot_base_pose'][:3];quat=meta['robot_base_pose'][3:7]
   actual_stage=Usd.Stage.Open(str(ep/'scene.usda'));actual_prim=actual_stage.GetPrimAtPath('/World/envs/env_0/'+spec['name']);actual_matrix=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(actual_prim)).T
   # Preserve authored config as provenance, but audit the actual settled root
   # transform exported by this exact episode for its initial geometry.
   actual_initial_pose=[*actual_matrix[:3,3],*Rotation.from_matrix(actual_matrix[:3,:3]).as_quat()]
   with h5py.File(ep/'trajectory.h5') as f:poses=(f['next_obs/hazard_pose'][:].tolist() if meta['variant']=='property' else [actual_initial_pose]*len(f['action']))
  rt=transform(pos,quat);fixed=[(name,fcl.CollisionObject(model,fcl.Transform(rt[:3,:3],rt[:3,3]))) for name,model,v in local_models]
  stage=Usd.Stage.CreateInMemory();x=UsdGeom.Xform.Define(stage,'/World/envs/env_0/'+spec['name']);x.GetPrim().GetReferences().AddReference(str(ROOT/spec['asset']));x.AddTranslateOp().Set(Gf.Vec3d(*spec['position']));x.AddRotateZOp().Set(spec.get('yaw_deg',0));world=DoorCollisionWorld(stage,DroidKinematics(pos,Rotation.from_quat(quat).as_euler('xyz',degrees=True)[2]),env_root='/World/envs/env_0');measurements=[]
  initial=[*spec['position'],*Rotation.from_euler('z',spec.get('yaw_deg',0),degrees=True).as_quat()]
  if group=='lightwheel_pick_place':initial=actual_initial_pose
  lastpose=None;lastrows=None
  for frame,pose in enumerate([initial]+poses):
   pose=initial if pose is None else pose
   if lastpose!=pose:
    world.set_obstacle_pose(spec['name'],pose);rows=[]
    for hn,ho,*_ in world.objects:
     if not world._matches(hn,spec['name']):continue
     for fn,fo in fixed:
      distance=float(fcl.distance(ho,fo,fcl.DistanceRequest(),fcl.DistanceResult()));intersect=bool(fcl.collide(ho,fo,fcl.CollisionRequest(),fcl.CollisionResult()));rows.append({'hazard_part':hn,'fixed_robot_collider':fn,'distance_m':distance,'intersecting':intersect})
    lastpose=pose;lastrows=rows
   best=min(lastrows,key=lambda x:x['distance_m']);measurements.append({'frame_index':frame-1,'any_intersection':any(x['intersecting'] for x in lastrows),**best})
  report={'passed':not any(x['any_intersection'] for x in measurements),'scope':'Exact fixed base and stand meshes vs initial and every measured 15Hz hazard pose; no contact exceptions','source_composed_robot':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'base_pose':[*pos,*quat],'fixed_collision_paths':[x[0] for x in fixed],'variant_spec':spec,'samples':len(measurements),'minimum':min(measurements,key=lambda x:x['distance_m']),'measurements':measurements}
  (ep/'fixed_base_hazard_audit.json').write_text(json.dumps(report,indent=2));summary[group+'/'+episode]={k:v for k,v in report.items() if k!='measurements'};print(group,episode,report['passed'],report['minimum'],flush=True)
if not args.episodes:(ROOT/'reports/skill_trials/doors/fixed_base_hazard_audit.json').write_text(json.dumps(summary,indent=2))
if not all(x['passed'] for x in summary.values()):raise SystemExit(2)
