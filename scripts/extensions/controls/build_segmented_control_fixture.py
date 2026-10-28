"""Author contiguous native physics cylinders, without a planner-only geometry override."""
import argparse
import hashlib
import json
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--segments',type=int,default=16)
    args=parser.parse_args()
    if not 2<=args.segments<=64:raise ValueError('Expected2-64 native segments')
    manifest=args.output.with_suffix('.manifest.json')
    if args.output.exists() or manifest.exists():raise FileExistsError(args.output)
    source=Usd.Stage.Open(str(args.source))
    old=source.GetPrimAtPath('/Asset/ProjectingTube')
    tube=UsdGeom.Cylinder(old)
    if not tube or old.GetAttribute('xformOpOrder').Get()!=['xformOp:translate']:
        raise ValueError('Only the declared translated native tube is supported')
    if not UsdPhysics.CollisionAPI(old).GetCollisionEnabledAttr().Get():
        raise ValueError('Source cylinder must have enabled physics collision')
    axis=str(tube.GetAxisAttr().Get());axis_id={'X':0,'Y':1,'Z':2}[axis]
    length=float(tube.GetHeightAttr().Get());radius=float(tube.GetRadiusAttr().Get())
    center=old.GetAttribute('xformOp:translate').Get()
    materials=old.GetRelationship('material:binding').GetTargets()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    layer=Sdf.Layer.CreateNew(str(args.output));layer.TransferContent(source.GetRootLayer())
    stage=Usd.Stage.Open(layer);stage.RemovePrim('/Asset/ProjectingTube')
    parent=UsdGeom.Xform.Define(stage,'/Asset/ProjectingTube')
    parent.AddTranslateOp().Set(center)
    for i in range(args.segments):
        part=UsdGeom.Cylinder.Define(stage,f'/Asset/ProjectingTube/Segment{i:02d}')
        part.GetAxisAttr().Set(axis);part.GetHeightAttr().Set(length/args.segments)
        part.GetRadiusAttr().Set(radius)
        offset=[0.,0.,0.];offset[axis_id]=length*((i+.5)/args.segments-.5)
        part.AddTranslateOp().Set(Gf.Vec3d(*offset))
        half=[radius,radius,radius];half[axis_id]=length/(2*args.segments)
        part.GetExtentAttr().Set([Gf.Vec3f(*[-x for x in half]),Gf.Vec3f(*half)])
        UsdPhysics.CollisionAPI.Apply(part.GetPrim()).GetCollisionEnabledAttr().Set(True)
        part.GetPrim().ApplyAPI('MaterialBindingAPI')
        part.GetPrim().CreateRelationship('material:binding').SetTargets(materials)
    layer.Save()
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    enabled=lambda s:sum(p.HasAPI(UsdPhysics.CollisionAPI) and
        UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() for p in s.Traverse())
    record=dict(source=str(args.source),source_sha256=digest(args.source),output=str(args.output),
        output_sha256=digest(args.output),source_cylinder='/Asset/ProjectingTube',axis=axis,
        radius_m=radius,length_m=length,segments=args.segments,continuous_native_volume=True,
        source_segment_interval_m=[-length/2,length/2],enabled_collision_count_before=enabled(source),
        enabled_collision_count_after=enabled(stage),
        scope='Authored physics collision decomposition. Native cylinders tile the entire original tube '
            'with touching end faces, unchanged radius/material/outer volume and unchanged remaining fixture. '
            'The default original per-collider AABB planner/execution model is retained; no model override. '
            'New image review, physical collection and independent gates are required.')
    manifest.write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
