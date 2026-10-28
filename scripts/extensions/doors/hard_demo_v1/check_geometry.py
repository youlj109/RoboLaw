"""CPU source meshes at every rendered frame; no physics claims."""
import sys,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from pxr import Usd,Sdf,UsdGeom,UsdPhysics
import numpy as np
import fcl
from kitchen_safety.geometry_audit import static_collision_shapes,minimum_distance
from scripts.extensions.doors.hard_demo_v1.build import DOOR,HUMAN,OUT,REPORT,phase

def source_cube_rows(stage,path):
 cache=UsdGeom.XformCache();rows=[]
 faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]],dtype=np.int32)
 for p in Usd.PrimRange(stage.GetPrimAtPath(path),Usd.TraverseInstanceProxies()):
  if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
  if not p.IsA(UsdGeom.Cube):raise ValueError('Expected source fridge Cube collider '+str(p.GetPath()))
  size=float(UsdGeom.Cube(p).GetSizeAttr().Get())/2.;verts=np.array([[x,y,z] for x in (-size,size) for y in (-size,size) for z in (-size,size)])
  mat=np.asarray(cache.GetLocalToWorldTransform(p)).T;verts=verts@mat[:3,:3].T+mat[:3,3]
  model=fcl.BVHModel();model.beginModel(len(verts),len(faces));model.addSubModel(verts,faces);model.endModel();rows.append((str(p.GetPath()),fcl.CollisionObject(model),verts.min(0),verts.max(0)))
 return rows

base=Usd.Stage.Open(str(OUT/'scene.usda'))
session=Sdf.Layer.CreateAnonymous();s=Usd.Stage.Open(base.GetRootLayer(),session);s.SetEditTarget(session)
for p in list(s.Traverse()):
 if p.IsInstance():p.SetInstanceable(False)
for p in Usd.PrimRange(s.GetPrimAtPath('/World/robot')):
 if p.IsA(UsdGeom.Xform) and p.HasAPI(UsdPhysics.CollisionAPI):p.RemoveAPI(UsdPhysics.CollisionAPI)
 if p.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(p).ComputeVisibility()!=UsdGeom.Tokens.invisible:UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
robot=static_collision_shapes(s,'/World/robot')
attrs=[a for p in base.Traverse() for a in p.GetAttributes() if a.GetNumTimeSamples()]

def freeze(frame):
 for a in attrs:s.GetAttributeAtPath(a.GetPath()).Set(a.Get(frame))

# The authored human extent encloses both complete poses, including interpolation.
box=UsdGeom.BBoxCache(Usd.TimeCode(30),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy,UsdGeom.Tokens.guide],False,True)
human_box=box.ComputeWorldBound(base.GetPrimAtPath('/World/human')).ComputeAlignedRange()
human_low=np.asarray(human_box.GetMin());human_high=np.asarray(human_box.GetMax())
nearby=[];screened=0
for p in Usd.PrimRange(base.GetPrimAtPath('/World/kitchen'),Usd.TraverseInstanceProxies()):
 if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() or str(p.GetPath()).startswith(DOOR):continue
 screened+=1;b=box.ComputeWorldBound(p).ComputeAlignedRange();lo=np.asarray(b.GetMin());hi=np.asarray(b.GetMax())
 if np.linalg.norm(np.maximum(np.maximum(lo-human_high,human_low-hi),0))<.005:
  nearby.append(str(p.GetPath()))
static_kitchen=[]
for path in nearby:static_kitchen.extend(source_cube_rows(s,path))
rows=[]
for frame in range(120):
 freeze(frame);human=static_collision_shapes(s,HUMAN);door=source_cube_rows(s,DOOR)
 row={'frame':frame,'phase':phase(frame)[0],'door_angle_deg':phase(frame)[1],
      'human_to_current_door':minimum_distance(human,door,.002),'human_to_robot':minimum_distance(human,robot,.002),
      'door_to_stationary_robot':minimum_distance(door,robot,.002),'human_to_surrounding_kitchen':minimum_distance(human,static_kitchen,.002),'human_lowest_z_m':float(min(x[2][2] for x in human))}
 try:row['human_to_fridge_fixed_body']=minimum_distance(human,source_cube_rows(s,'/World/kitchen/Asset/fridge_main_group/Refrigerator032'),.002)
 except Exception as error:row['fixed_body_query_error']=str(error)
 rows.append(row)
freeze(30);human=static_collision_shapes(s,HUMAN)
op=base.GetPrimAtPath(DOOR).GetAttribute('xformOp:transform:scriptedDoor');s.GetPrimAtPath(DOOR).GetAttribute(op.GetName()).Set(op.Get(119));future=minimum_distance(human,source_cube_rows(s,DOOR),.002)
report={'scope':'Discrete original source geometry checks at every rendered frame; no native physics, policy or continuous swept collision proof','sample_count':len(rows),'all_render_frames_checked':True,'static_kitchen_colliders_aabb_screened':screened,'kitchen_narrow_phase_candidates':nearby,'source_fridge_collider_method':'Exact transformed source Cube surfaces triangulated into 12 triangles, preserving affine scale/shear; no world AABB collision replacement.','samples':rows,'human_at_pause_vs_future_open_door_counterfactual':future,'current_door_and_human_clear_at_sampled_boundaries':all(r['human_to_current_door']['valid'] for r in rows),'human_clear_of_stationary_robot':all(r['human_to_robot']['valid'] for r in rows),'future_door_would_intersect_human':future['intersecting'],'human_clear_of_surrounding_kitchen':all(r['human_to_surrounding_kitchen']['valid'] for r in rows),'door_clear_of_stationary_robot':all(r['door_to_stationary_robot']['valid'] for r in rows),'no_simulation_started':True}
(REPORT/'geometry.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if isinstance(v,(str,int,float,bool))},indent=2))
