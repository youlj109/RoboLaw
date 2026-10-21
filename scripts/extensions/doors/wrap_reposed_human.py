"""Add collision to all six newly posed meshes, retaining original materials."""
from pathlib import Path
import argparse
from pxr import Usd,UsdGeom,UsdPhysics,Sdf
parser=argparse.ArgumentParser();parser.add_argument('--asset-dir',default='assets/extensions/doors/human_reach_high');args=parser.parse_args();p=Path(args.asset_dir);target=p/'collidable.usda'
if target.exists():target.unlink()
s=Usd.Stage.CreateNew(str(target));root=UsdGeom.Xform.Define(s,'/Person');s.SetDefaultPrim(root.GetPrim());root.GetPrim().GetReferences().AddReference('person.usdc');count=0
for q in s.Traverse():
 if q.IsA(UsdGeom.Mesh):
  UsdPhysics.CollisionAPI.Apply(q).CreateCollisionEnabledAttr(True);UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr('none');UsdGeom.Mesh(q).CreateDoubleSidedAttr(True);count+=1
 if 'Principled_BSDF' in str(q.GetPath()) and any(x in str(q.GetPath()) for x in ['short02','eyebrow001','BrownEyes']):q.CreateAttribute('inputs:opacityThreshold',Sdf.ValueTypeNames.Float).Set(.5)
assert count==6,count
UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(s,1);s.GetRootLayer().Save();print('COLLIDERS',count)
