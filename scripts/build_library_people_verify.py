"""CPU/FCL placement review of the six library humans against recorded start state.

Run via scripts/arena_python.sh. Does not start Isaac Sim or move any camera.
Checks the unchanged initial robot and bottle, plus 101 samples of direct carried
bottle transport using its actual visible mesh, and original external-camera rays.
These are scene suitability checks, not claims of successful motion planning.
"""
import argparse,json,sys
from pathlib import Path
import fcl,h5py,numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf,Usd,UsdGeom
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from kitchen_safety.collision_planner import CollisionWorld
from kitchen_safety.planning import DroidKinematics
from build_library_people import triangles_of,ray_intersections
LIB=ROOT/'assets/obstacle_library/people'
parser=argparse.ArgumentParser()
parser.add_argument('--include-default',action='store_true',help='Also check the default MPFB corridor human.')
args=parser.parse_args()
EP=ROOT/'datasets/kitchen_auto/obstructed_human_fixed_cameras/human/attempt_0000_seed_42'
def model(v,f):
 g=fcl.BVHModel();g.beginModel(len(v),len(f));g.addSubModel(np.asarray(v,float),np.asarray(f,np.int32));g.endModel();return g
kin=DroidKinematics([2.2,-.93,.8],90.)
with h5py.File(EP/'trajectory.h5') as h:
 names=json.loads(h.attrs['joint_names']);indices=[names.index(f'panda_joint{i}') for i in range(1,8)]
 initial_q=h['obs/q'][0,indices];initial_bottle=h['obs/mustard'][0,:7]
 grip={name:float(h['obs/q'][0,names.index(name)]) for name in CollisionWorld.GRIPPER_JOINT_NAMES}
 phases=h['phase'].asstr()[:];lift=np.flatnonzero(phases=='lift')[-1]
 carried=h['next_obs/mustard'][lift,:7];bowl=h['obs/bowl'][0,:3]
source=Usd.Stage.Open(str(ROOT/'assets/robolab/objects/hope/mustard_bottle.usd'));cache=UsdGeom.XformCache();vv=[];ff=[];offset=0
for prim in Usd.PrimRange(source.GetPseudoRoot(),Usd.TraverseInstanceProxies()):
 if prim.IsA(UsdGeom.Mesh):
  vertices,faces=triangles_of(UsdGeom.Mesh(prim),cache);vv.append(vertices);ff.append(faces+offset);offset+=len(vertices)
bottle_model=model(np.concatenate(vv),np.concatenate(ff))
results=[]
entries=json.loads((LIB/'catalog.json').read_text())['assets']
if args.include_default:
 layout=json.loads((ROOT/'configs/layout.json').read_text())['human']
 pose=json.loads((ROOT/'assets/people/mpfb_corridor/pose.json').read_text())
 entries.append({'id':'human_default_mpfb_corridor','usd_path':layout['asset'],'position':layout['position'],'yaw_deg':layout['yaw_deg'],'keypoints':pose})
for entry in entries:
 stage=Usd.Stage.CreateInMemory();prim=UsdGeom.Xform.Define(stage,'/World/Person');prim.GetPrim().GetReferences().AddReference(str(ROOT/entry['usd_path']))
 prim.AddTranslateOp().Set(Gf.Vec3d(*entry['position']));prim.AddRotateZOp().Set(entry['yaw_deg'])
 world=CollisionWorld(stage,kin,clearances={'person':.04},execution_clearances={'person':.02})
 initial=world.clearance(initial_q,mode='execution',gripper_q=grip,object_pose=initial_bottle,obstacle_filter=('person',))
 meshes=[(name,v,f,fcl.CollisionObject(model(v,f))) for name,(v,f) in world.meshes.items() if '/Person/' in name]
 hits=[];minimum=float('inf')
 for index,u in enumerate(np.linspace(0,1,101)):
  pose=carried.copy();pose[:2]=carried[:2]+u*(bowl[:2]-carried[:2]);obj=fcl.CollisionObject(bottle_model,fcl.Transform(Rotation.from_quat(pose[3:]).as_matrix(),pose[:3]))
  collided=[]
  for name,v,f,person in meshes:
   distance=float(fcl.distance(obj,person,fcl.DistanceRequest(),fcl.DistanceResult()));minimum=min(minimum,max(0.,distance))
   if distance<0 or (distance==0 and fcl.collide(obj,person,fcl.CollisionRequest(),fcl.CollisionResult())):collided.append(name)
  if collided:hits.append({'index':index,'fraction':float(u),'meshes':collided})
 camera=np.array([2.77,-.88,1.46]);rays={}
 R=Rotation.from_euler('z',entry['yaw_deg'],degrees=True).as_matrix()
 targets={'source_bottle':initial_bottle[:3],'bowl':bowl,'wrist':np.asarray(entry['keypoints']['wrist_local'])@R.T+entry['position'],'hand_tip':np.asarray(entry['keypoints']['hand_tip_local'])@R.T+entry['position']}
 for label,target in targets.items():
  delta=target-camera;length=np.linalg.norm(delta);direction=delta/length;rayhits=[]
  for name,v,f,_ in meshes:
   values=ray_intersections(v[f],camera,direction);values=values[values<length-(.035 if label in ('wrist','hand_tip') else .005)]
   if len(values):rayhits.append({'mesh':name,'distance_before_target_m':float(length-values[0])})
  rays[label]={'occluded_by_human':bool(rayhits),'hits':rayhits}
 row={'id':entry['id'],'initial_state_clearance':initial,'visible_bottle_direct_sweep':{'samples':101,'intersections':len(hits),'minimum_distance_m':minimum,'first':hits[0] if hits else None,'last':hits[-1] if hits else None},'fixed_camera2_rays':rays,'initial_pose_valid':initial['valid'],'direct_transport_is_blocked':bool(hits)}
 results.append(row);print(json.dumps(row),flush=True)
report={'reference_episode':str(EP.relative_to(ROOT)),'scope':'Initial recorded robot and bottle envelope clearance; actual visible bottle mesh direct sweep; fixed camera2 point rays. No motion success claim.','passed':all(x['initial_pose_valid'] and x['direct_transport_is_blocked'] for x in results),'assets':results}
(LIB/'placement_validation.json').write_text(json.dumps(report,indent=2)+'\n')
