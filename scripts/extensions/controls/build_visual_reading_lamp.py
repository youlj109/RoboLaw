"""Create a recognizable reading lamp for visual review, not damage validation."""
import argparse
import math
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def build(output):
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    root = UsdGeom.Xform.Define(stage, '/Lamp').GetPrim()
    stage.SetDefaultPrim(root)
    root.SetDocumentation('Visual-only revision: weighted reading lamp with an open glass shade '
                          'and visible bulb. No fracture or electrical safety model; not physically validated.')
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(6.)

    def material(name, color, metallic=0., roughness=.35, opacity=1., emission=None):
        mat = UsdShade.Material.Define(stage, '/Lamp/Materials/' + name)
        shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
        shader.CreateIdAttr('UsdPreviewSurface')
        for key, kind, value in [('diffuseColor', Sdf.ValueTypeNames.Color3f, Gf.Vec3f(*color)),
                                 ('metallic', Sdf.ValueTypeNames.Float, metallic),
                                 ('roughness', Sdf.ValueTypeNames.Float, roughness),
                                 ('opacity', Sdf.ValueTypeNames.Float, opacity)]:
            shader.CreateInput(key, kind).Set(value)
        if emission:
            shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emission))
        shader.CreateOutput('surface', Sdf.ValueTypeNames.Token)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        return mat

    black = material('SatinBlackMetal', (.055, .065, .075), .8, .28)
    brass = material('BrushedBrass', (.55, .34, .12), .8, .24)
    glass = material('FrostedGlass', (.60, .79, .85), 0., .18, .65)
    bulb = material('WarmBulb', (.96, .89, .68), 0., .28, emission=(.7, .48, .19))

    def finish(obj, pos, mat):
        prim = obj.GetPrim()
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        UsdPhysics.CollisionAPI.Apply(prim)
        return obj

    def cylinder(name, radius, height, pos, mat, axis='Z'):
        obj = UsdGeom.Cylinder.Define(stage, '/Lamp/' + name)
        obj.CreateRadiusAttr(radius)
        obj.CreateHeightAttr(height)
        obj.CreateAxisAttr(axis)
        return finish(obj, pos, mat)

    def sphere(name, radius, pos, mat):
        obj = UsdGeom.Sphere.Define(stage, '/Lamp/' + name)
        obj.CreateRadiusAttr(radius)
        return finish(obj, pos, mat)

    cylinder('WeightedBase', .16, .035, (0, 0, .0175), black)
    cylinder('BaseCollar', .032, .06, (0, 0, .055), brass)
    cylinder('Upright', .014, 1.16, (0, 0, .615), black)
    cylinder('Arm', .011, .80, (0, .39, 1.185), black, 'Y')
    sphere('ElbowJoint', .022, (0, 0, 1.185), brass)
    cylinder('PendantStem', .010, .09, (0, .78, 1.14), brass)
    cylinder('Socket', .026, .04, (0, .78, 1.085), brass)
    cylinder('BulbNeck', .014, .05, (0, .78, 1.04), bulb)
    sphere('VisibleBulb', .025, (0, .78, .995), bulb)
    cylinder('PowerSwitch', .012, .012, (-.018, 0, .65), brass, 'X')

    # Four rings form a hollow frustum with real openings and a visible thin rim.
    rings = [(.046, 1.10), (.100, .94), (.043, 1.10), (.097, .94)]
    points = []
    n = 64
    for radius, z in rings:
        points.extend([(radius * math.cos(2*math.pi*i/n), .78 + radius*math.sin(2*math.pi*i/n), z)
                       for i in range(n)])
    indices = []
    for a, b in [(0, 1), (3, 2), (2, 0), (1, 3)]:
        for i in range(n):
            j = (i + 1) % n
            indices.extend([a*n+i, a*n+j, b*n+j, b*n+i])
    shade = UsdGeom.Mesh.Define(stage, '/Lamp/OpenGlassShade')
    shade.CreatePointsAttr(points)
    shade.CreateFaceVertexCountsAttr([4] * (n*4))
    shade.CreateFaceVertexIndicesAttr(indices)
    shade.CreateDoubleSidedAttr(True)
    shade.CreateSubdivisionSchemeAttr('none')
    UsdShade.MaterialBindingAPI.Apply(shade.GetPrim()).Bind(glass)
    UsdPhysics.CollisionAPI.Apply(shade.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(shade.GetPrim()).CreateApproximationAttr('convexHull')
    stage.GetRootLayer().Save()
    print(output)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    build(p.parse_args().output)
