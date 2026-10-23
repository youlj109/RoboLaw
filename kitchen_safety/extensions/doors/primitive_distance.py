"""Distance to source solid primitives, including nonuniformly scaled cylinders."""
import numpy as np
from scipy.optimize import brentq

def ellipsoid_solid_distance(point,radii):
 point=np.abs(np.asarray(point));radii=np.asarray(radii)
 if np.sum((point/radii)**2)<=1.:return 0.
 r2=radii**2
 def equation(lam):return np.sum((radii*point/(lam+r2))**2)-1.
 upper=max(float(np.max(radii*point)),1e-12)
 while equation(upper)>0:upper*=2
 lam=brentq(equation,0.,upper,xtol=1e-16)
 closest=r2*point/(lam+r2)
 return float(np.linalg.norm(point-closest))

def source_distance(prim,matrix):
 from pxr import UsdGeom
 A=np.asarray(matrix)[:3,:3];scale=np.linalg.norm(A,axis=0);R=A/scale;t=np.asarray(matrix)[:3,3]
 if not np.allclose(R.T@R,np.eye(3),atol=1e-5):raise ValueError('Sheared primitive unsupported: '+str(prim.GetPath()))
 if prim.IsA(UsdGeom.Cylinder):
  shape=UsdGeom.Cylinder(prim);axis='XYZ'.index(str(shape.GetAxisAttr().Get()));perp=[i for i in range(3) if i!=axis];radii=float(shape.GetRadiusAttr().Get())*scale[perp];half=float(shape.GetHeightAttr().Get())*scale[axis]/2
  def distance(point):
   local=R.T@(np.asarray(point)-t);radial=ellipsoid_solid_distance(local[perp],radii);axial=max(0.,abs(local[axis])-half);return float(np.hypot(radial,axial))
 elif prim.IsA(UsdGeom.Cube):
  half=float(UsdGeom.Cube(prim).GetSizeAttr().Get())*scale/2
  def distance(point):return float(np.linalg.norm(np.maximum(np.abs(R.T@(np.asarray(point)-t))-half,0.)))
 elif prim.IsA(UsdGeom.Sphere):
  radii=float(UsdGeom.Sphere(prim).GetRadiusAttr().Get())*scale
  def distance(point):return ellipsoid_solid_distance(R.T@(np.asarray(point)-t),radii)
 else:raise ValueError('Unsupported source primitive: '+str(prim.GetPath()))
 return distance
