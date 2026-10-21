"""Author simple, collidable safety-scene props in metres (Z up)."""
import math
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'assets/safety'

def stage(name, mass=None):
    OUT.mkdir(parents=True, exist_ok=True)
    s = Usd.Stage.CreateNew(str(OUT / f'{name}.usda'))
    UsdGeom.SetStageUpAxis(s, 'Z')
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    p = UsdGeom.Xform.Define(s, '/Asset').GetPrim()
    s.SetDefaultPrim(p)
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
    return s

def material(s, name, color, metallic=0.0, roughness=0.4, opacity=1.0):
    m = UsdShade.Material.Define(s, '/Asset/Materials/'+name)
    sh = UsdShade.Shader.Define(s, m.GetPath().AppendChild('Shader'))
    sh.CreateIdAttr('UsdPreviewSurface')
    for key, typ, val in [('diffuseColor', Sdf.ValueTypeNames.Color3f, color),
                          ('metallic', Sdf.ValueTypeNames.Float, metallic),
                          ('roughness', Sdf.ValueTypeNames.Float, roughness),
                          ('opacity', Sdf.ValueTypeNames.Float, opacity)]:
        sh.CreateInput(key, typ).Set(val)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return m

def shape(s, name, kind, pos, size, mat, collision=True):
    cls = {'box': UsdGeom.Cube, 'sphere': UsdGeom.Sphere, 'cylinder': UsdGeom.Cylinder}[kind]
    obj = cls.Define(s, '/Asset/'+name)
    if kind == 'box': obj.CreateSizeAttr(1.)
    elif kind == 'sphere': obj.CreateRadiusAttr(1.)
    else:
        obj.CreateRadiusAttr(1.)
        obj.CreateHeightAttr(1.)
        obj.CreateAxisAttr('Z')
    x = UsdGeom.Xformable(obj)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision: UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    return obj

def limb(s, name, a, b, radius, mat):
    a,b = Gf.Vec3d(*a),Gf.Vec3d(*b)
    d=b-a
    obj=UsdGeom.Capsule.Define(s,'/Asset/'+name)
    obj.CreateRadiusAttr(radius)
    obj.CreateHeightAttr(max(0.001,d.GetLength()-2*radius))
    obj.CreateAxisAttr('Z')
    x=UsdGeom.Xformable(obj)
    x.AddTranslateOp().Set((a+b)/2)
    x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Rotation(Gf.Vec3d(0,0,1),d.GetNormalized()).GetQuat())
    UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)

def lathe(s, name, profile, mat, n=64, collision=False):
    pts=[]; faces=[]
    for r,z in profile:
        pts.extend([(r*math.cos(i*2*math.pi/n),r*math.sin(i*2*math.pi/n),z) for i in range(n)])
    for j in range(len(profile)-1):
        for i in range(n): faces.extend([j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i])
    obj=UsdGeom.Mesh.Define(s,'/Asset/'+name)
    obj.CreatePointsAttr(pts);obj.CreateFaceVertexCountsAttr([4]*(len(faces)//4));obj.CreateFaceVertexIndicesAttr(faces)
    obj.CreateSubdivisionSchemeAttr('none');obj.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr('convexHull')
    return obj

def build_barrier():
    """A floor-fixed steel C-frame with a visible wrist pinch opening.

    The load-bearing members are the colliders. Paint and bolt heads are
    visual details, so the planner does not need to process every stripe.
    Local +X points into the workspace; the fixture has no moving parts.
    """
    s=stage('barrier')
    steel=material(s,'PowderCoatedSteel',(.16,.20,.23),metallic=.72,roughness=.3)
    edge=material(s,'MachinedSteel',(.48,.53,.56),metallic=.9,roughness=.24)
    yellow=material(s,'SafetyYellow',(.95,.62,.015),roughness=.42)
    black=material(s,'SafetyBlack',(.012,.016,.019),roughness=.48)

    shape(s,'AnchoredFoot','box',(0,0,.02),(.27,.36,.04),steel)
    shape(s,'Upright','box',(0,0,.70),(.06,.06,1.32),steel)
    # Clear opening: X = 0.03..0.33 m, Z = 1.02..1.30 m.
    shape(s,'PinchUpperBeam','box',(.15,0,1.33),(.36,.06,.06),steel)
    shape(s,'PinchLowerBeam','box',(.15,0,.99),(.36,.06,.06),steel)
    # The machined jaw faces make the narrow rigid opening legible in images.
    shape(s,'UpperJawFace','box',(.18,0,1.298),(.30,.064,.004),edge)
    shape(s,'LowerJawFace','box',(.18,0,1.022),(.30,.064,.004),edge)
    for z,label in [(1.33,'Upper'),(.99,'Lower')]:
        shape(s,label+'BeamEnd','box',(.331,0,z),(.002,.06,.06),edge,False)

    def warning_face(name,z,y):
        """Yellow/black diagonal paint, clipped to a beam's rectangular face."""
        x0,x1=-.027,.327
        z0,z1=z-.026,z+.026
        shape(s,name+'Paint','box',((x0+x1)/2,y,z),(x1-x0,.0006,z1-z0),yellow,False)
        # Clip slanted strips x-z in [a,a+width] against the rectangle.
        for index,a in enumerate([x0-z1+i*.068 for i in range(7)]):
            poly=[(x0,z0),(x1,z0),(x1,z1),(x0,z1)]
            for bound,sign in [(a,1.),(a+.031,-1.)]:
                clipped=[]
                for p,q in zip(poly,poly[1:]+poly[:1]):
                    fp=sign*(p[0]-p[1]-bound)
                    fq=sign*(q[0]-q[1]-bound)
                    if fp>=0:clipped.append(p)
                    if (fp>=0)!=(fq>=0):
                        u=fp/(fp-fq)
                        clipped.append((p[0]+u*(q[0]-p[0]),p[1]+u*(q[1]-p[1])))
                poly=clipped
                if not poly:break
            if len(poly)<3:continue
            mesh=UsdGeom.Mesh.Define(s,f'/Asset/{name}Stripe{index}')
            surface_y=y+math.copysign(.0004,y)
            mesh.CreatePointsAttr([(x,surface_y,zz) for x,zz in poly])
            mesh.CreateFaceVertexCountsAttr([len(poly)])
            mesh.CreateFaceVertexIndicesAttr(list(range(len(poly))))
            mesh.CreateSubdivisionSchemeAttr('none')
            mesh.CreateDoubleSidedAttr(True)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(black)

    for side,y in [('Front',-.0305),('Back',.0305)]:
        warning_face(side+'Upper',1.33,y)
        warning_face(side+'Lower',.99,y)
    # Triangular welded support, kept outside the wrist opening.
    points=[(x,y,z) for y in [-.025,.025] for x,z in [(.03,1.10),(.03,1.30),(.13,1.30)]]
    gusset=UsdGeom.Mesh.Define(s,'/Asset/UpperGusset')
    gusset.CreatePointsAttr(points)
    gusset.CreateFaceVertexCountsAttr([3,3,4,4,4])
    gusset.CreateFaceVertexIndicesAttr([0,2,1,3,4,5,0,1,4,3,1,2,5,4,2,0,3,5])
    gusset.CreateSubdivisionSchemeAttr('none')
    UsdShade.MaterialBindingAPI.Apply(gusset.GetPrim()).Bind(steel)
    UsdPhysics.CollisionAPI.Apply(gusset.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(gusset.GetPrim()).CreateApproximationAttr('convexHull')

    def bolt(name,pos,axis='Z'):
        obj=UsdGeom.Cylinder.Define(s,'/Asset/'+name)
        obj.CreateRadiusAttr(.011)
        obj.CreateHeightAttr(.009)
        obj.CreateAxisAttr(axis)
        UsdGeom.Xformable(obj).AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(edge)

    for i,x in enumerate([-.098,.098]):
        for j,y in enumerate([-.14,.14]):bolt(f'AnchorBolt{i}{j}',(x,y,.0445))
    for label,z in [('Upper',1.33),('Lower',.99)]:
        for side,y in [('Front',-.035),('Back',.035)]:
            bolt(label+side+'Fastener',(0,y,z),'Y')
    s.GetRootLayer().Save()

def build_tall_glass():
    """A 22 cm high drinking glass with visible rim and still-water surface."""
    s=stage('tall_glass',.65)
    glass=material(s,'Glass',(.62,.82,.88),roughness=.12,opacity=.32)
    rim=material(s,'GlassRim',(.73,.88,.94),roughness=.16,opacity=.55)
    water=material(s,'Water',(.44,.72,.82),roughness=.08,opacity=.18)
    surface=material(s,'WaterSurface',(.63,.82,.88),roughness=.10,opacity=.28)
    lathe(s,'GlassShell',[(0,0),(.038,0),(.038,.22),(.035,.22),(.034,.008),(0,.008)],glass)
    lathe(s,'GlassRim',[(.0348,.217),(.039,.217),(.039,.22),(.0348,.22),(.0348,.217)],rim)
    # Static visual water; neither a fluid simulator nor a solid liquid collider.
    lathe(s,'WaterBody',[(0,.008),(.0335,.008),(.0335,.13),(0,.13)],water)
    lathe(s,'WaterSurface',[(0,.1302),(.0335,.1302)],surface)
    hidden=material(s,'Collision',(.5,.5,.5))
    base=shape(s,'BaseCollision','cylinder',(0,0,.004),(.038,.038,.008),hidden)
    base.CreateVisibilityAttr('invisible')
    for i in range(16):
        angle=2*math.pi*i/16
        wall=UsdGeom.Cube.Define(s,f'/Asset/WallCollision{i}')
        wall.CreateSizeAttr(1.)
        xf=UsdGeom.Xformable(wall)
        xf.AddTranslateOp().Set(Gf.Vec3d(.0365*math.cos(angle),.0365*math.sin(angle),.114))
        xf.AddRotateZOp().Set(math.degrees(angle)+90)
        xf.AddScaleOp().Set(Gf.Vec3f(.0145,.003,.212))
        UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(wall.GetPrim()).Bind(hidden)
        wall.CreateVisibilityAttr('invisible')
    s.GetRootLayer().Save()

def build():
    s=stage('person')
    skin=material(s,'Skin',(0.58,0.34,0.22),roughness=.65)
    shirt=material(s,'TealShirt',(.025,.27,.30),roughness=.8)
    pants=material(s,'Trousers',(.055,.07,.10),roughness=.85)
    shoe=material(s,'Shoes',(.025,.025,.03),roughness=.8)
    white=material(s,'Eyes',(.9,.9,.85));dark=material(s,'Hair',(.04,.025,.015))
    shape(s,'Torso','sphere',(0,0,1.22),(.19,.12,.30),shirt)
    shape(s,'Pelvis','sphere',(0,0,.91),(.17,.12,.14),pants)
    for label,x in [('Left',-.10),('Right',.10)]:
        limb(s,label+'Thigh',(x,0,.92),(x,.015,.51),.08,pants)
        limb(s,label+'Shin',(x,.015,.51),(x,0,.12),.06,pants)
        shape(s,label+'Shoe','sphere',(x,.045,.065),(.075,.14,.065),shoe)
    limb(s,'Neck',(0,0,1.46),(0,0,1.57),.06,skin)
    shape(s,'Head','sphere',(0,0,1.68),(.105,.10,.135),skin)
    shape(s,'Hair','sphere',(0,-.01,1.755),(.108,.095,.067),dark,False)
    shape(s,'Nose','sphere',(-.075,.065,1.68),(.035,.025,.03),skin,False)
    # Reaching left arm crosses the direct bottle-to-bowl corridor.
    limb(s,'ReachUpperArm',(-.16,0,1.43),(-.35,.28,1.22),.055,shirt)
    limb(s,'ReachForearm',(-.35,.28,1.22),(-.68,.48,1.14),.043,skin)
    shape(s,'ReachPalm','sphere',(-.74,.49,1.14),(.075,.045,.023),skin)
    for i in range(4):
        limb(s,'Finger'+str(i),(-.77,.46+i*.019,1.14),(-.85,.46+i*.019,1.14),.008,skin)
    limb(s,'Thumb',(-.72,.455,1.14),(-.76,.42,1.135),.012,skin)
    limb(s,'RestUpperArm',(.16,0,1.43),(.22,.01,1.14),.055,shirt)
    limb(s,'RestForearm',(.22,.01,1.14),(.20,.08,.91),.043,skin)
    shape(s,'RestHand','sphere',(.20,.08,.88),(.04,.025,.08),skin)
    s.GetRootLayer().Save()

    s=stage('glass',.18)
    glass=material(s,'Glass',(.62,.82,.88),roughness=.12,opacity=.32)
    lathe(s,'GlassShell',[(0,0),(.038,0),(.038,.13),(.035,.13),(.034,.007),(0,.007)],glass)
    # Compound convex walls preserve the open interior.
    hidden=material(s,'Collision',(.5,.5,.5))
    base=shape(s,'BaseCollision','cylinder',(0,0,.0035),(.037,.037,.007),hidden)
    base.CreateVisibilityAttr('invisible')
    for i in range(16):
        a=2*math.pi*i/16
        p=shape(s,f'WallCollision{i}','box',(.036*math.cos(a),.036*math.sin(a),.066),(.0145,.004,.126),hidden)
        UsdGeom.Xformable(p).AddRotateZOp().Set(math.degrees(a)+90)
        p.CreateVisibilityAttr('invisible')
    s.GetRootLayer().Save()
    s=stage('plate',.32);mat=material(s,'Ceramic',(.86,.89,.82),roughness=.22)
    lathe(s,'Plate',[(0,0),(.085,0),(.10,.012),(.095,.018),(.074,.007),(0,.007)],mat,collision=True)
    s.GetRootLayer().Save()
    s=stage('phone',.19);metal=material(s,'Frame',(.12,.13,.15),metallic=.8)
    black=material(s,'Screen',(.008,.018,.025),roughness=.12)
    shape(s,'Body','box',(0,0,.004),(.072,.15,.008),metal)
    shape(s,'Screen','box',(0,0,.0083),(.066,.138,.0006),black,False)
    s.GetRootLayer().Save()
    build_barrier()
    build_tall_glass()

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--only',choices=['all','barrier','tall_glass'],default='all')
    args=parser.parse_args()
    if args.only=='barrier':build_barrier()
    elif args.only=='tall_glass':build_tall_glass()
    else:build()
