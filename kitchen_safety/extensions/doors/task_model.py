"""Resolve each assigned door from its source YAML and USD joint graph."""
from pathlib import Path
import json
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
TASK_IDS={'C01': 'kitchen_bench_lightwheel_open_fridge', 'C02': 'kitchen_bench_lightwheel_open_cabinet_u_shaped_with_island_farmhouse2', 'C03': 'kitchen_bench_lightwheel_open_freezer_g_shaped_large_scandinavian', 'C04': 'kitchen_bench_lightwheel_open_fridge_g_shaped_large_scandinavian', 'C05': 'kitchen_bench_lightwheel_open_fridge_u_shaped_with_island_farmhouse2', 'C06': 'kitchen_bench_lightwheel_open_microwave', 'C07': 'kitchen_bench_lightwheel_open_microwave_g_shaped_large_scandinavian', 'C08': 'kitchen_bench_lightwheel_open_oven_g_shaped_large_scandinavian'}

def assignment(task_id):
 data=json.loads((ROOT/'student_skills/robolaw-doors/references/assignment.json').read_text())
 return next(x for x in data['tasks'] if x['id']==TASK_IDS.get(task_id,task_id))
def resolve(stage,task_id,world=None,prefix='/World/envs/env_0/kitchen'):
 from pxr import Usd,UsdGeom,UsdPhysics
 from kitchen_safety.planning import transform
 task=assignment(task_id);params=task['task']['subtasks'][0]['params'];obj=next(x for x in task['object_references'] if x['id']==params['openable_object'])
 root=stage.GetPrimAtPath(prefix+'/'+obj['prim_path']);prims=list(Usd.PrimRange(root,Usd.TraverseInstanceProxies()))
 joint_name=obj['params']['openable_joint_name'];matches=[p for p in prims if p.GetName()==joint_name and p.IsA(UsdPhysics.RevoluteJoint)]
 if len(matches)!=1:raise ValueError(f'Expected one source revolute joint {joint_name}: {matches}')
 j=UsdPhysics.RevoluteJoint(matches[0])
 def rigid_owner(path):
  prim=stage.GetPrimAtPath(path)
  while prim and not prim.IsPseudoRoot():
   if prim.HasAPI(UsdPhysics.RigidBodyAPI):return str(prim.GetPath())
   prim=prim.GetParent()
  raise ValueError('Joint endpoint has no rigid-body ancestor: '+str(path))
 body0=rigid_owner(j.GetBody0Rel().GetTargets()[0]);body1=rigid_owner(j.GetBody1Rel().GetTargets()[0]);moving={body1}
 fixed=[UsdPhysics.FixedJoint(p) for p in prims if p.IsA(UsdPhysics.FixedJoint)]
 while True:
  before=len(moving)
  for f in fixed:
   ends={rigid_owner(x) for x in [*f.GetBody0Rel().GetTargets(),*f.GetBody1Rel().GetTargets()]}
   if moving&ends:moving.update(ends)
  if len(moving)==before:break
 def belongs(path):return any(path==b or path.startswith(b+'/') for b in moving)
 handles=[str(p.GetPath()) for p in prims if belongs(str(p.GetPath())) and 'handle' in str(p.GetPath()).lower() and p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
 if not handles:raise ValueError('No source handle collider in the joint-connected moving body cluster')
 cache=UsdGeom.XformCache();T0=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body0))).T;r=j.GetLocalRot0Attr().Get();local=transform(j.GetLocalPos0Attr().Get(),[*r.GetImaginary(),r.GetReal()]);hingeT=T0@local
 axis=hingeT[:3,'XYZ'.index(str(j.GetAxisAttr().Get()))];axis=axis/np.linalg.norm(axis)
 data={'task':task['id'],'object_name':obj['id'],'object_path':str(root.GetPath()),'joint_name':joint_name,'joint_path':str(matches[0].GetPath()),'door_path':body1,'moving_bodies':sorted(moving),'handle_colliders':handles,'hinge':hingeT[:3,3].tolist(),'axis':axis.tolist(),'limits_rad':np.deg2rad([j.GetLowerLimitAttr().Get(),j.GetUpperLimitAttr().Get()]).tolist(),'reset_openness':params.get('reset_openness',0.),'openness_threshold':params['openness_threshold']}
 if world is not None:
  shapes=[o for o in world.objects if o[0] in handles]
  if len(shapes)!=len(handles):raise ValueError('Planning world omitted source handle colliders')
  main=max(shapes,key=lambda o:float(np.max(o[3]-o[2])))
  data['handle_main']=main[0];data['handle_center']=((main[2]+main[3])/2).tolist();data['handle_bounds']=[np.min([o[2] for o in shapes],axis=0).tolist(),np.max([o[3] for o in shapes],axis=0).tolist()]
  data['handle_shapes']=[{'path':n,'bounds':[lo.tolist(),hi.tolist()]} for n,_,lo,hi in shapes]
 return data


def door_target_pose(hinge, axis, handle_at_reference, rotation_at_reference, joint_position, reference_position):
 """TCP target from source joint displacement, including nonzero reset coordinates."""
 from scipy.spatial.transform import Rotation
 hinge=np.asarray(hinge,dtype=float);axis=np.asarray(axis,dtype=float)
 norm=np.linalg.norm(axis)
 if not np.isfinite(norm) or norm<1e-12:raise ValueError('Invalid source door axis')
 rr=Rotation.from_rotvec(axis/norm*(float(joint_position)-float(reference_position))).as_matrix()
 target=np.eye(4);target[:3,:3]=rr@np.asarray(rotation_at_reference,dtype=float)
 target[:3,3]=hinge+rr@(np.asarray(handle_at_reference,dtype=float)-hinge)
 return target


def grasp_rotation(approach, roll_deg=0., pitch_deg=0.):
 """Source-facing tool orientation; optional local pitch for handle clearance."""
 from scipy.spatial.transform import Rotation
 approach=np.asarray(approach,dtype=float)
 frame=np.column_stack([approach,np.cross([0.,0.,-1.],approach),[0.,0.,-1.]])
 return frame@Rotation.from_euler('x',roll_deg,degrees=True).as_matrix()@Rotation.from_euler('y',pitch_deg,degrees=True).as_matrix()


def joint_position_from_openness(value, lower, upper):
 """Exact inverse of pinned Arena's sign-dependent normalized joint convention."""
 if not upper>lower:raise ValueError('Joint must have a nonzero range')
 return lower+(upper-lower)*(1.-value if lower<0. else value)


def opening_command(measured, lead_rad, opening_limit, lower):
 """Advance in source opening direction without stepping beyond the chosen limit."""
 return max(measured-lead_rad,opening_limit) if lower<0. else min(measured+lead_rad,opening_limit)
