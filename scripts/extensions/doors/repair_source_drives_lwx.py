"""Local source drive-schema repair; preserves target doors, limits and mass geometry."""
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,Sdf
import json,hashlib,os
root=Path('/workspace');report=root/'reports/doors/lwx_C02_C03_20261023';rows=[]
for label,name in [('C02','robocasa-robocasakitchen-7-10'),('C03','robocasa-robocasakitchen-9-8')]:
 source=root/'runtime/home/.cache/lightwheel_sdk/floorplan'/name/'scene.usd';dest=root/'assets/extensions/doors'/(label.lower()+'_mechanisms_lwx_01.usda')
 if dest.exists():raise FileExistsError(dest)
 stage=Usd.Stage.CreateNew(str(dest));r=UsdGeom.Xform.Define(stage,'/world');r.GetPrim().GetReferences().AddReference(os.path.relpath(source,dest.parent));stage.SetDefaultPrim(r.GetPrim())
 instances=[str(p.GetPath()) for p in Usd.PrimRange(r.GetPrim(),Usd.TraverseInstanceProxies()) if p.IsInstance()]
 for path in sorted(instances,key=len):stage.OverridePrim(path).SetInstanceable(False)
 changes=[]
 for prim in list(stage.Traverse()):
  if prim.IsA(UsdPhysics.RevoluteJoint) and 'knob' in prim.GetName():
   old=UsdPhysics.DriveAPI(prim,'X')
   if not old:continue
   new=UsdPhysics.DriveAPI.Apply(prim,'angular');attrs={}
   for name in ['Stiffness','Damping','MaxForce','TargetPosition','TargetVelocity','Type']:
    value=getattr(old,'Get'+name+'Attr')().Get()
    if value is not None:getattr(new,'Create'+name+'Attr')(value);attrs[name]=str(value)
   changes.append({'path':str(prim.GetPath()),'change':'Copy authored X drive values into USD revolute angular drive; no new stiffness','source_values':attrs})
  if prim.GetTypeName()=='PhysicsJoint' and prim.GetName()=='handle_joint':
   for axis in ['rotX','rotY']:
    drive=UsdPhysics.DriveAPI(prim,axis)
    if drive and drive.GetMaxForceAttr().Get()==0:
     drive.CreateMaxForceAttr(.5)
     changes.append({'path':str(prim.GetPath()),'axis':axis,'change':'Enable existing finite source spring/damper with 0.5 Nm torque cap (model repair assumption, requires mobility check)','old_maxForce':0,'new_maxForce':.5,'stiffness_unchanged':drive.GetStiffnessAttr().Get(),'damping_unchanged':drive.GetDampingAttr().Get()})
 stage.GetRootLayer().Save();rows.append({'task':label,'source':str(source.relative_to(root)),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'overlay':str(dest.relative_to(root)),'overlay_sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'changes':changes})
(report/'source_drive_repair_01.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows),flush=True)
