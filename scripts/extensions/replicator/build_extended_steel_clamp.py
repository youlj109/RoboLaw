#!/usr/bin/env python3
"""Author a traceable reach650 overlay without changing the original fixture."""
from pathlib import Path
import copy
import hashlib
import json
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
 from pxr import Gf,Usd,UsdGeom
 from kitchen_safety.obstacle_library import get_asset
 from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard
 source=copy.deepcopy(get_asset('robot_steel_clamp'))
 folder=ROOT/'assets/extensions/replicator/steel_clamp_reach650';folder.mkdir(parents=True,exist_ok=True)
 target=folder/'collidable.usda'
 if target.exists():target.unlink()
 stage=Usd.Stage.CreateNew(str(target));UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
 root=UsdGeom.Xform.Define(stage,'/Asset').GetPrim();stage.SetDefaultPrim(root)
 root.GetReferences().AddReference('../../../obstacle_library/robot/steel_clamp.usda')
 root.SetDocumentation('Derived anchored static fixture: 0.65m beam reach; original foot, upright, jaw heights and spacing retained. No real anchor strength/damage certification.')
 overrides={}
 def change(name,attribute,value):
  prim=stage.OverridePrim('/Asset/Fixture/Frame/'+name)
  attr=prim.GetAttribute(attribute)
  if not attr:raise ValueError('Missing inherited source attribute: '+name+'/'+attribute)
  attr.Set(value);overrides.setdefault(str(prim.GetPath()),{})[attribute]=list(value)
 for name,z in [('PinchUpperBeam',1.33),('PinchLowerBeam',.99)]:
  change(name,'xformOp:scale',Gf.Vec3f(.68,.06,.06))
  change(name,'xformOp:translate',Gf.Vec3d(.31,0,z))
 for name,z in [('UpperJawFace',1.298),('LowerJawFace',1.022)]:
  change(name,'xformOp:scale',Gf.Vec3f(.62,.064,.004))
  change(name,'xformOp:translate',Gf.Vec3d(.34,0,z))
 # Appearance-only end caps and yellow side paint follow the longer beams.
 for name,z in [('UpperBeamEnd',1.33),('LowerBeamEnd',.99)]:
  change(name,'xformOp:translate',Gf.Vec3d(.651,0,z))
 for name,z in [('FrontUpperPaint',1.33),('FrontLowerPaint',.99)]:
  change(name,'xformOp:scale',Gf.Vec3f(.674,.0006,.052))
  change(name,'xformOp:translate',Gf.Vec3d(.31,-.0305,z))
 if stage.GetCompositionErrors():raise ValueError(str(stage.GetCompositionErrors()))
 stage.GetRootLayer().Save()
 original=Hazard(source)
 metadata=dict(schema_version=1,asset_type='robot_steel_clamp',asset_variant='reach650',
  source_asset=source['usd_path'],source_asset_sha256=sha(ROOT/source['usd_path']),source_layers_sha256=original.dependencies,
  output_asset=str(target.relative_to(ROOT)),output_asset_sha256=sha(target),
  reach_m=.65,original_reach_m=.33,beam_root_x_m=-.03,beam_length_m=.68,beam_center_x_m=.31,
  jaw_root_x_m=.03,jaw_length_m=.62,jaw_center_x_m=.34,
  unchanged=['AnchoredFoot','Upright','bolt/other source colliders','beam/jaw Z positions and vertical gap','source materials','static floor-anchor semantics'],
  override_attributes=overrides,implementation_path=str(Path(__file__).relative_to(ROOT)),implementation_sha256=sha(__file__),
  scope='Source collider length adaptation only. Native stability, anchor strength, avoidance necessity and task success unverified.')
 (folder/'derivation.json').write_text(json.dumps(metadata,indent=2)+'\n')
 asset=copy.deepcopy(source);asset.update(usd_path=str(target.relative_to(ROOT)),asset_variant='reach650',
  asset_source=dict(id=source['id'],usd_path=source['usd_path'],sha256=sha(ROOT/source['usd_path'])),
  derivation_path=str((folder/'derivation.json').relative_to(ROOT)),derivation_sha256=sha(folder/'derivation.json'),
  derivation_parameters=dict(reach_m=.65,beam_length_m=.68,beam_center_x_m=.31,jaw_length_m=.62,jaw_center_x_m=.34),
  mounting_semantics='Static floor-anchored equipment; source foot/upright/jaw heights preserved; strength not simulated')
 hazard=Hazard(asset)
 asset['source_collision_bounds_m']=[hazard.low.tolist(),hazard.high.tolist()]
 asset['source_collider_count']=len(hazard.rows)
 visual=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy']).ComputeWorldBound(hazard.stage.GetDefaultPrim()).ComputeAlignedRange()
 visual_low=list(visual.GetMin());visual_high=list(visual.GetMax())
 visual_size=[float(b-a) for a,b in zip(visual_low,visual_high)]
 collider_rows={r[0].rsplit('/',1)[-1]:r for r in hazard.rows}
 opening=float(collider_rows['UpperJawFace'][4][2]-collider_rows['LowerJawFace'][5][2])
 asset['source_original_metadata']={key:copy.deepcopy(source[key]) for key in ('dimensions','asset_validation','notes','provenance') if key in source}
 asset['dimensions']=dict(overall_xyz_m=visual_size,opening_z_m=opening,jaw_face_center_separation_m=.276,
     dimension_method='Composed visual AABB; opening is the actual source clear gap between jaw face surfaces')
 asset['asset_validation']=dict(usd_valid=not bool(hazard.stage.GetCompositionErrors()),static=True,colliders=len(hazard.rows),
     visual_bounds_min_m=visual_low,visual_bounds_max_m=visual_high,measured_dimensions_xyz_m=visual_size,
     source_collision_bounds_min_m=hazard.low.tolist(),source_collision_bounds_max_m=hazard.high.tolist(),
     validation_scope='USD composition and authored source geometry only',native_physics_verified=False,task_accepted=False)
 asset['provenance']=dict(type='derived_local_reference',source=source['usd_path'],source_sha256=sha(ROOT/source['usd_path']),
     generator=str(Path(__file__).relative_to(ROOT)),generator_sha256=sha(__file__),derivation=str((folder/'derivation.json').relative_to(ROOT)))
 asset['notes']=['Derived reach650 floor-anchored C-frame; original source asset remains unchanged.',
     'Beam/jaw reaches and their visual end/paint positions are overridden; foot, upright, heights and clear gap are retained.',
     'This derived asset has no native stability, anchor-strength, trajectory-risk or task-success acceptance.']
 output=ROOT/'configs/extensions/replicator/visual_scene_variants_reach650_assets.json'
 output.write_text(json.dumps({'robot_steel_clamp':asset},ensure_ascii=False,indent=2)+'\n')
 print(json.dumps(dict(asset=str(target.relative_to(ROOT)),colliders=len(hazard.rows),bounds=asset['source_collision_bounds_m'],overrides=str(output.relative_to(ROOT)))))
 return 0
if __name__=='__main__':raise SystemExit(main())
