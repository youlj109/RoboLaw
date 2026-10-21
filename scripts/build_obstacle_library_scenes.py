"""Compose each library obstacle with the unchanged shared kitchen task."""
import json
import os
import sys
from pathlib import Path
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kitchen_safety.obstacle_library import load_assets

assets = load_assets()
out = ROOT/'scenes/obstacle_library'
configs = ROOT/'configs/obstacle_library'
out.mkdir(parents=True, exist_ok=True)
configs.mkdir(parents=True, exist_ok=True)
for entry in assets:
    path = out/(entry['id']+'.usda')
    stage = Usd.Stage.CreateNew(str(path))
    stage.GetRootLayer().subLayerPaths = ['../base.usda']
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    name = {'human':'Person', 'property':'Glass', 'robot':'Barrier'}[entry['category']]
    prim = UsdGeom.Xform.Define(stage, '/World/'+name)
    prim.GetPrim().GetReferences().AddReference(os.path.relpath(ROOT/entry['usd_path'], out))
    prim.AddTranslateOp().Set(Gf.Vec3d(*entry['position']))
    prim.AddRotateZOp().Set(entry.get('yaw_deg', 0))
    stage.GetRootLayer().Save()
    config = {'variant':entry['category'], 'obstacle_id':entry['id'],
              'scene':'../../scenes/obstacle_library/'+entry['id']+'.usda',
              'description':entry.get('label',entry['id'])}
    (configs/(entry['id']+'.json')).write_text(json.dumps(config,indent=2,ensure_ascii=False)+'\n')
(ROOT/'configs/obstacle_library.json').write_text(json.dumps({
    'description':'Local obstacle library; shared DROID task and original camera mounts.',
    'assets':assets},indent=2,ensure_ascii=False)+'\n')
print(f'Composed {len(assets)} library scenes')
