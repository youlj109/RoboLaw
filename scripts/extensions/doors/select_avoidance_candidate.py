"""Copy a CPU-proven candidate and author its actual collision pedestal."""
import argparse,json
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,Gf
p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('--index',type=int,required=True);a=p.parse_args();root=Path(__file__).resolve().parents[3]
candidate=json.loads(Path(a.report).read_text())['candidates'][a.index];variant=candidate['variant'];spec=candidate['spec'];lo=candidate['pedestal']['bottom'];hi=candidate['pedestal']['top'];lo[2]=0.0
if spec.get('static'):hi[2]=spec['position'][2]
size=[hi[i]-lo[i] for i in range(3)]
asset=Path(f'assets/extensions/doors/{variant}_floor_pedestal_filtered.usda');target=root/asset
stage=Usd.Stage.CreateNew(str(target));base=UsdGeom.Xform.Define(stage,'/Pedestal');stage.SetDefaultPrim(base.GetPrim());UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
box=UsdGeom.Cube.Define(stage,'/Pedestal/SolidColumn');box.CreateSizeAttr(1.);box.AddTranslateOp().Set(Gf.Vec3d(0,0,size[2]/2));box.AddScaleOp().Set(Gf.Vec3d(*size));box.CreateDisplayColorAttr([(0.19,.23,.28)]);UsdPhysics.CollisionAPI.Apply(box.GetPrim());stage.GetRootLayer().Save()
spec['support']={'name':variant+'_pedestal','asset':str(asset),'position':[spec['position'][0],spec['position'][1],lo[2]],'static':True,'dimensions_m':size}
spec['semantic_anchor']='Anchored floor pedestal; full column, kitchen, stand and door sweep included; old baseline conflicts during free pregrasp.';spec['candidate_evidence']=a.report+' candidate '+str(a.index)
p=root/'configs/extensions/doors/avoidance_variants.json';data=json.loads(p.read_text());data[variant]=spec;p.write_text(json.dumps(data,indent=2)+'\n');print(json.dumps(spec,indent=2))
