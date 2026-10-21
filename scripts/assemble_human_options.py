"""Make comparable human scene variants and static mesh collision wrappers."""
from pathlib import Path
import json
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf
ROOT=Path(__file__).resolve().parents[1]
layout=json.loads((ROOT/'configs/layout.json').read_text())
for kind in ['nvidia','mpfb']:
    folder=ROOT/'assets/people'/kind
    s=Usd.Stage.CreateNew(str(folder/'collidable.usda'))
    UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1.)
    p=UsdGeom.Xform.Define(s,'/Person').GetPrim();s.SetDefaultPrim(p)
    p.GetReferences().AddReference('person.usdc')
    for child in s.Traverse():
        if child.IsA(UsdGeom.Mesh):
            UsdGeom.Mesh(child).CreateDoubleSidedAttr(True)
            UsdPhysics.CollisionAPI.Apply(child)
            UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr('none')
        if child.IsA(UsdShade.Shader) and UsdShade.Shader(child).GetIdAttr().Get()=='UsdPreviewSurface':
            shader=UsdShade.Shader(child)
            if shader.GetInput('opacity').HasConnectedSource():
                shader.CreateInput('opacityThreshold',Sdf.ValueTypeNames.Float).Set(.5)
    s.GetRootLayer().Save()
    stage=Usd.Stage.CreateNew(str(ROOT/'scenes'/f'human_{kind}.usda'))
    stage.GetRootLayer().subLayerPaths=['base.usda']
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    person=UsdGeom.Xform.Define(stage,'/World/Person')
    person.GetPrim().GetReferences().AddReference(f'../assets/people/{kind}/collidable.usda')
    person.AddTranslateOp().Set(Gf.Vec3d(*layout['human']['position']))
    cam=UsdGeom.Camera.Define(stage,'/World/Portrait');cam.CreateFocalLengthAttr(45.)
    eye=Gf.Vec3d(3.55,-2.9,2.05);target=Gf.Vec3d(2.98,-.85,1.37)
    UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
    stage.GetRootLayer().Save()
