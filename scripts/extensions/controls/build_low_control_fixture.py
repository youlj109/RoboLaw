"""Generate one lower static fixture using authored native USD collision shapes."""
import argparse
import hashlib
import json
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=Path('assets/obstacle_library/robot/projecting_crossbar.usda'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--height-factor',type=float,default=.174/.28)
    args=parser.parse_args()
    if not .5<=args.height_factor<=1.5:raise ValueError('Fixture height factor outside declared range')
    manifest=args.output.with_suffix('.manifest.json')
    if args.output.exists() or manifest.exists():raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    source=Usd.Stage.Open(str(args.source))
    layer=Sdf.Layer.CreateNew(str(args.output));layer.TransferContent(source.GetRootLayer())
    stage=Usd.Stage.Open(layer)
    edits=[]
    for prim in stage.Traverse():
        translate=prim.GetAttribute('xformOp:translate')
        if translate and translate.HasAuthoredValue():
            value=translate.Get();translate.Set(Gf.Vec3d(value[0],value[1],value[2]*args.height_factor))
            edits.append(str(translate.GetPath()))
        if prim.IsA(UsdGeom.Cube):
            scale=prim.GetAttribute('xformOp:scale')
            if scale and scale.HasAuthoredValue():
                value=scale.Get();scale.Set(Gf.Vec3f(value[0],value[1],value[2]*args.height_factor))
                edits.append(str(scale.GetPath()))
        elif prim.IsA(UsdGeom.Cylinder):
            cylinder=UsdGeom.Cylinder(prim)
            if cylinder.GetAxisAttr().Get()=='Z':
                cylinder.GetHeightAttr().Set(cylinder.GetHeightAttr().Get()*args.height_factor)
                edits.append(str(cylinder.GetHeightAttr().GetPath()))
        elif prim.IsA(UsdGeom.Mesh):
            if prim.GetAttribute('xformOp:scale'):raise ValueError('Mesh with additional authored scale is unsupported')
            points=UsdGeom.Mesh(prim).GetPointsAttr()
            points.Set([Gf.Vec3f(p[0],p[1],p[2]*args.height_factor) for p in points.Get()])
            edits.append(str(points.GetPath()))
    stage.GetDefaultPrim().SetDocumentation('Controlled static protective fixture. Height-adjusted source geometry; '
        'native circular cylinders keep their radius. Not a manufacturer model or accepted scene.')
    layer.Save()
    record=dict(source=str(args.source),source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
        output=str(args.output),output_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        height_factor=args.height_factor,tube_center_local_z_m=.28*args.height_factor,
        native_round_radius_unchanged=True,collisions_disabled=False,edited_attributes=edits,
        scope='One authored static augmentation fixture candidate; no original task geometry/robot changes. '
            'Visual review, support legality, original margins, paired planning and physical execution still required.')
    manifest.write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
