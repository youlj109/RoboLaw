"""Exact FCL primitives for a counterfactual, avoiding AABB-only necessity claims."""
import numpy as np
from pxr import UsdGeom,Usd
from scipy.spatial.transform import Rotation

def physical_hazards(stage,world,key,root_pose=None):
 import fcl
 from kitchen_safety.planning import transform
 cache=UsdGeom.XformCache();selected=[];delta=np.eye(4)
 if root_pose is not None:
  root=next(p for p in stage.Traverse() if p.GetName()==key)
  delta=transform(root_pose[:3],root_pose[3:])@np.linalg.inv(np.array(cache.GetLocalToWorldTransform(root)).T)
 for name,obj,lo,hi in world.objects:
  if not world._matches(name,key):continue
  prim=stage.GetPrimAtPath(name)
  if prim.IsA(UsdGeom.Mesh):selected.append((name,obj,lo,hi));continue
  T=np.array(cache.GetLocalToWorldTransform(prim)).T;scale=np.linalg.norm(T[:3,:3],axis=0);R=T[:3,:3]/scale
  if prim.IsA(UsdGeom.Cube):g=fcl.Box(*(float(UsdGeom.Cube(prim).GetSizeAttr().Get())*scale))
  elif prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Capsule):
   shape=UsdGeom.Cylinder(prim) if prim.IsA(UsdGeom.Cylinder) else UsdGeom.Capsule(prim);axis='XYZ'.index(str(shape.GetAxisAttr().Get()));perp=[i for i in range(3) if i!=axis]
   if not np.isclose(scale[perp[0]],scale[perp[1]],atol=1e-8):raise ValueError('elliptical primitive not supported')
   radius=float(shape.GetRadiusAttr().Get())*scale[perp[0]];length=float(shape.GetHeightAttr().Get())*scale[axis];g=(fcl.Cylinder if prim.IsA(UsdGeom.Cylinder) else fcl.Capsule)(radius,length)
   local=np.eye(3) if axis==2 else Rotation.from_euler('y' if axis==0 else 'x',90 if axis==0 else -90,degrees=True).as_matrix();R=R@local
  elif prim.IsA(UsdGeom.Sphere):
   if not np.allclose(scale,scale[0]):raise ValueError('ellipsoid primitive unsupported')
   g=fcl.Sphere(float(UsdGeom.Sphere(prim).GetRadiusAttr().Get())*scale[0])
  else:raise ValueError('unsupported '+name)
  obj=fcl.CollisionObject(g,fcl.Transform(delta[:3,:3]@R,delta[:3,:3]@T[:3,3]+delta[:3,3]));selected.append((name,obj,lo,hi))
 return selected

def physical_robot_shapes(shapes):
 return [s for s in shapes if s[0]=='robot_hull' or s[0].startswith('gripper/')]
