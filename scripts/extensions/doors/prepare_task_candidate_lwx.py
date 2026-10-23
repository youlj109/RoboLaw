"""Materialize one immutable source-geometry search candidate and its real support."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--search',type=Path,required=True);p.add_argument('--variant',required=True);p.add_argument('--index',type=int,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
cfg=json.loads(a.config.read_text());spec=json.loads(a.search.read_text())['candidates'][a.index];support=spec.pop('proposed_support',None)
if support:
 low=np.array(support['low']);high=np.array(support['high']);low[2]-=.002;high[2]+=.002;center=(low+high)/2;center[:2]=0.;asset=Path('assets/extensions/doors')/(a.output.stem+'_'+a.variant+'_support.usda')
 if asset.exists():raise FileExistsError(asset)
 stage=Usd.Stage.CreateNew(str(asset));root=UsdGeom.Xform.Define(stage,'/Support');stage.SetDefaultPrim(root.GetPrim());c=UsdGeom.Cube.Define(stage,'/Support/Column');c.CreateSizeAttr(1.);c.AddTranslateOp().Set(Gf.Vec3d(*center));c.AddScaleOp().Set(Gf.Vec3f(*(high-low)));c.CreateDisplayColorAttr([Gf.Vec3f(.28,.30,.32)]);UsdPhysics.CollisionAPI.Apply(c.GetPrim());UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.);stage.GetRootLayer().Save()
 spec['support']={'name':a.variant+'_pedestal','asset':str(asset),'position':[*spec['position'][:2],0.],'static':True,'bounds_world_m':[low.tolist(),high.tolist()],'source_support':support['rests_on']}
if a.variant=='human':
 pose=Path(spec['asset']).parent/'pose.json'
 if not pose.exists():raise FileNotFoundError(pose)
 spec['pose_provenance']=str(pose);spec['pose_provenance_sha256']=hashlib.sha256(pose.read_bytes()).hexdigest();spec['unrotated_asset']=spec['asset'];spec.pop('baked_yaw_deg',None)
spec['semantic_anchor']='Original successful pregrasp actually intersects added source geometry. Fixed baseline task, layout, target and control policy; independent paired planning and physical audits required.';spec['candidate_search']=str(a.search);spec['candidate_search_sha256']=hashlib.sha256(a.search.read_bytes()).hexdigest();spec['candidate_index']=a.index;cfg[a.variant]=spec;cfg['_selection_note']='Discovery candidate only; not task or physical acceptance.';a.output.write_text(json.dumps(cfg,indent=2)+'\n');print(a.output)
