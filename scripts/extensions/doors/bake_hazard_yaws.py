"""Preserve non-cardinal hazard world geometry with Arena-compatible root yaw."""
import json,os
from pathlib import Path
from pxr import Usd,UsdGeom,Gf
ROOT=Path(__file__).resolve().parents[3]
p=ROOT/'configs/extensions/doors/avoidance_variants.json';data=json.loads(p.read_text())
for variant in ['human','robot']:
 spec=data[variant]
 if not spec.get('yaw_deg'):continue
 source=ROOT/spec['asset'];yaw=spec['yaw_deg'];target=ROOT/'assets/extensions/doors'/f'{variant}_yaw{yaw:g}.usda'
 stage=Usd.Stage.CreateNew(str(target));root=UsdGeom.Xform.Define(stage,'/Asset');stage.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
 body=UsdGeom.Xform.Define(stage,'/Asset/Geometry');body.GetPrim().GetReferences().AddReference(os.path.relpath(source,target.parent));body.AddRotateZOp().Set(yaw);stage.GetRootLayer().Save()
 spec['unrotated_asset']=spec['asset'];spec['baked_yaw_deg']=yaw;spec['asset']=str(target.relative_to(ROOT));spec['yaw_deg']=0.
p.write_text(json.dumps(data,indent=2)+'\n')
