"""Distance to source solid primitives, including nonuniformly scaled cylinders."""
import numpy as np
import itertools
from scipy.optimize import brentq

def affine_box_solid_distance(matrix,half):
 """Exact bounded least-squares distance to an affine image of a source cube."""
 matrix=np.asarray(matrix,dtype=float);A=matrix[:3,:3];t=matrix[:3,3]
 if not np.isfinite(matrix).all() or np.linalg.matrix_rank(A)!=3 or half<=0:
  raise ValueError('Degenerate affine source box')
 inverse=np.linalg.inv(A);cases=[]
 # Every closest point lies in a face, edge or vertex, or inside the solid.
 # Enumerate active local bounds; solve in the free coordinates in world metric.
 for states in itertools.product([-1,0,1],repeat=3):
  fixed=np.flatnonzero(states);free=np.flatnonzero(np.asarray(states)==0)
  if not len(fixed):continue
  offset=A[:,fixed]@(half*np.asarray(states)[fixed])
  basis=A[:,free];solver=np.linalg.pinv(basis) if len(free) else None
  cases.append((offset,basis,solver))
 def distance(point):
  v=np.asarray(point)-t
  if np.all(np.abs(inverse@v)<=half):return 0.
  best=float('inf')
  for offset,basis,solver in cases:
   local=solver@(v-offset) if solver is not None else np.empty(0)
   if np.any(np.abs(local)>half+1e-12):continue
   best=min(best,float(np.linalg.norm(v-offset-basis@local)))
  return best
 return distance

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
 if prim.IsA(UsdGeom.Cube) and not np.allclose(R.T@R,np.eye(3),atol=1e-5):
  return affine_box_solid_distance(matrix,float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2)
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
