"""Build six reusable kitchen property obstacles, in metres and Z up.

Run with ``.venv/bin/python scripts/build_library_property.py``. Geometry is
authored locally from dimensional profiles; no downloaded models or textures.
Visual hollow vessels and their compound convex colliders share the same
profiles. Collision geometry is hidden, not disabled. No liquid/fracture model
is implied by the glass, ceramic, or still-water appearance.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets/obstacle_library/property"
CATALOG = []
PHYSICAL = {"static_friction": .55, "dynamic_friction": .42, "restitution": .03}


def begin(name, mass, center_of_mass):
    OUT.mkdir(parents=True, exist_ok=True)
    s = Usd.Stage.CreateNew(str(OUT / (name + ".usda")))
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.)
    root = UsdGeom.Xform.Define(s, "/Asset").GetPrim()
    s.SetDefaultPrim(root)
    root.SetCustomDataByKey("asset_category", "property")
    root.SetCustomDataByKey("generator", "scripts/build_library_property.py")
    UsdPhysics.RigidBodyAPI.Apply(root)
    m = UsdPhysics.MassAPI.Apply(root)
    m.CreateMassAttr(mass)
    m.CreateCenterOfMassAttr(Gf.Vec3f(*center_of_mass))
    physics = UsdShade.Material.Define(s, "/Asset/Materials/Contact")
    phys = UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
    phys.CreateStaticFrictionAttr(PHYSICAL["static_friction"])
    phys.CreateDynamicFrictionAttr(PHYSICAL["dynamic_friction"])
    phys.CreateRestitutionAttr(PHYSICAL["restitution"])
    UsdShade.MaterialBindingAPI.Apply(root).Bind(physics, materialPurpose="physics")
    return s


def material(s, name, color, roughness=.3, metallic=0., opacity=1., emissive=None):
    mat = UsdShade.Material.Define(s, "/Asset/Materials/" + name)
    sh = UsdShade.Shader.Define(s, mat.GetPath().AppendChild("Surface"))
    sh.CreateIdAttr("UsdPreviewSurface")
    for n, t, v in [("diffuseColor", Sdf.ValueTypeNames.Color3f, color),
                    ("roughness", Sdf.ValueTypeNames.Float, roughness),
                    ("metallic", Sdf.ValueTypeNames.Float, metallic),
                    ("opacity", Sdf.ValueTypeNames.Float, opacity),
                    ("ior", Sdf.ValueTypeNames.Float, 1.47)]:
        sh.CreateInput(n, t).Set(v)
    if emissive is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(emissive)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def mesh(s, name, pts, faces, mat=None, collision=False):
    obj = UsdGeom.Mesh.Define(s, "/Asset/" + name)
    obj.CreatePointsAttr(pts)
    obj.CreateFaceVertexCountsAttr([len(f) for f in faces])
    obj.CreateFaceVertexIndicesAttr([v for f in faces for v in f])
    obj.CreateSubdivisionSchemeAttr("none")
    obj.CreateDoubleSidedAttr(True)
    obj.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(pts))
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr("convexHull")
        obj.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    return obj


def lathe(s, name, profile, mat, n=96):
    """Profile is an oriented meridian. Duplicate first point to close a shell."""
    pts = [(r * math.cos(2 * math.pi * i / n),
            r * math.sin(2 * math.pi * i / n), z)
           for r, z in profile for i in range(n)]
    faces = [[j*n+i, j*n+(i+1)%n, (j+1)*n+(i+1)%n, (j+1)*n+i]
             for j in range(len(profile)-1) for i in range(n)]
    return mesh(s, name, pts, faces, mat)


def cylinder(s, name, radius, height, z, mat=None, collision=False):
    obj = UsdGeom.Cylinder.Define(s, "/Asset/" + name)
    obj.CreateRadiusAttr(radius)
    obj.CreateHeightAttr(height)
    obj.CreateAxisAttr("Z")
    UsdGeom.Xformable(obj).AddTranslateOp().Set(Gf.Vec3d(0, 0, z))
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        obj.CreateVisibilityAttr("invisible")
    return obj


def box(s, name, position, dimensions, mat=None, collision=False, rotate_x=0.):
    obj = UsdGeom.Cube.Define(s, "/Asset/" + name)
    obj.CreateSizeAttr(1.)
    xf = UsdGeom.Xformable(obj)
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rotate_x:
        xf.AddRotateXOp().Set(rotate_x)
    xf.AddScaleOp().Set(Gf.Vec3d(*dimensions))
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        obj.CreateVisibilityAttr("invisible")
    return obj


def hollow_collision(s, profile, thickness, base_thickness, angular_segments=20):
    """Convex wall wedges preserve the vessel interior and top opening.

    The outer profile progresses upward. Each two-profile-point interval is
    a separate convex wall ring: the hull never bridges the vessel mouth.
    Tiny angular overlap removes radial cracks between adjacent hulls.
    """
    cylinder(s, "Colliders/Base", profile[0][0], base_thickness,
             profile[0][1] + base_thickness/2, collision=True)
    for j, ((ra, za), (rb, zb)) in enumerate(zip(profile, profile[1:])):
        for i in range(angular_segments):
            aa = (i - .002) * 2*math.pi/angular_segments
            ab = (i + 1.002) * 2*math.pi/angular_segments
            pts = []
            for z, r in [(za, ra), (zb, rb)]:
                for rr, angle in [(r-thickness, aa), (r, aa),
                                  (r, ab), (r-thickness, ab)]:
                    pts.append((rr*math.cos(angle), rr*math.sin(angle), z))
            mesh(s, f"Colliders/Wall{j:02d}_{i:02d}", pts,
                 [[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],
                  [2,3,7,6],[3,0,4,7]], collision=True)


def solid_profile_collision(s, profile, segments=24, prefix="Body"):
    for j, ((ra, za), (rb, zb)) in enumerate(zip(profile, profile[1:])):
        pts = [(r*math.cos(2*math.pi*i/segments), r*math.sin(2*math.pi*i/segments), z)
               for r,z in [(ra,za),(rb,zb)] for i in range(segments)]
        faces = [list(reversed(range(segments))), list(range(segments,2*segments))]
        faces += [[i,(i+1)%segments,(i+1)%segments+segments,i+segments] for i in range(segments)]
        mesh(s, f"Colliders/{prefix}{j}", pts, faces, collision=True)


def torus(s, name, radius, tube, z, mat, n=96, sides=12):
    pts = [((radius+tube*math.cos(2*math.pi*j/sides))*math.cos(2*math.pi*i/n),
            (radius+tube*math.cos(2*math.pi*j/sides))*math.sin(2*math.pi*i/n),
            z+tube*math.sin(2*math.pi*j/sides)) for i in range(n) for j in range(sides)]
    faces = [[i*sides+j,((i+1)%n)*sides+j,((i+1)%n)*sides+(j+1)%sides,
              i*sides+(j+1)%sides] for i in range(n) for j in range(sides)]
    return mesh(s,name,pts,faces,mat)


def finish(s, name, title, mass, notes, top_opening=None):
    s.GetRootLayer().Save()
    # Hidden collider bounds are excluded from visible dimensions.
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"], useExtentsHint=False)
    size = bounds.ComputeWorldBound(s.GetDefaultPrim()).ComputeAlignedRange().GetSize()
    collisions = [p for p in s.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    entry = {"id": "property_" + name, "label": title, "category": "property",
             "usd_path": str((OUT/(name+".usda")).relative_to(ROOT)),
             "position": [2.245,-.355,.922], "yaw_deg": 0.,
             "dimensions": [round(float(x),6) for x in size], "dimensions_unit":"m",
             "physics_type": "rigid", "static": False, "mass_kg": mass,
             "physics_material": PHYSICAL, "collider_count": len(collisions),
             "collision_model": "compound_convex", "provenance": {
                 "type": "procedural_local", "generator": "scripts/build_library_property.py",
                 "external_sources": [], "textures": "none; authored material colors"},
             "notes": notes}
    if top_opening is not None:
        entry["opening_inner_diameter_m"] = top_opening
    CATALOG.append(entry)


def tall_glass():
    name, mass = "tall_glass", .56
    s = begin(name,mass,(0,0,.085))
    glass = material(s,"ClearGlass",(.65,.82,.86),.1,opacity=.28)
    edge = material(s,"ThickGlassEdge",(.73,.87,.91),.14,opacity=.62)
    water = material(s,"StillWater",(.28,.57,.67),.08,opacity=.18)
    profile = [(.034,0),(.038,.008),(.039,.22)]
    lathe(s,"GlassShell",[(0,0)]+profile+[(.0355,.22),(.0345,.011),(0,.011)],glass)
    torus(s,"RolledRim",.0372,.0018,.2182,edge)
    lathe(s,"ThickFoot",[(0,.001),(.034,.001),(.0378,.007),(.033,.009),(0,.009)],edge)
    lathe(s,"StillWaterVolume",[(0,.012),(.0342,.012),(.035,.12),(0,.12)],water)
    torus(s,"WaterMeniscus",.0343,.0007,.12,water)
    hollow_collision(s,profile,.0035,.01)
    finish(s,name,"Tall water glass",mass,
           ["22 cm drinking glass; 12 cm visual still-water fill.",
            "Water is appearance only; combined rigid mass approximates vessel plus fill.",
            "Open mouth and interior retained by convex wall wedges."],.071)


def wine_glass():
    name,mass = "wine_glass",.22
    s = begin(name,mass,(0,0,.133))
    glass = material(s,"Crystal",(.79,.85,.92),.08,opacity=.25)
    rim = material(s,"CrystalEdge",(.82,.88,.95),.12,opacity=.62)
    wine = material(s,"RedWine",(.23,.009,.032),.18,opacity=.88)
    lathe(s,"Foot",[(0,0),(.036,0),(.040,.003),(.040,.005),(.031,.008),(.004,.01),(0,.01)],glass)
    lathe(s,"Stem",[(0,.008),(.0043,.008),(.0033,.1),(.007,.115),(0,.117)],glass)
    profile = [(.008,.108),(.030,.124),(.043,.154),(.047,.18),(.044,.212),(.034,.24)]
    inside = [(max(.003,r-.0022),z) for r,z in reversed(profile)]
    lathe(s,"Bowl",profile+inside+[profile[0]],glass)
    torus(s,"RolledRim",.0329,.00115,.2388,rim)
    torus(s,"FootRim",.038,.0014,.004,rim)
    lathe(s,"WineFill",[(0,.118),(.020,.123),(.038,.147),(.042,.16),(0,.16)],wine)
    cylinder(s,"Colliders/Foot",.04,.007,.0035,collision=True)
    cylinder(s,"Colliders/Stem",.0044,.106,.059,collision=True)
    hollow_collision(s,profile,.0022,.005,angular_segments=20)
    finish(s,name,"Stemmed wine glass",mass,
           ["24 cm stemware; broad stable foot, delicate stem and open bowl.",
            "Red wine is a static visual fill, not liquid simulation.",
            "A different obstacle geometry from a straight-sided drinking glass."],.0636)


def glass_bottle():
    name,mass = "glass_bottle",.72
    s = begin(name,mass,(0,0,.105))
    glass = material(s,"AmberGlass",(.21,.095,.022),.18,opacity=.82)
    foil = material(s,"GoldCap",(.64,.45,.13),.28,.72)
    paper = material(s,"CreamLabel",(.83,.77,.60),.64)
    ink = material(s,"ForestLabel",(.04,.105,.07),.72)
    profile=[(.032,0),(.036,.007),(.037,.035),(.037,.169),(.032,.183),(.015,.202),(.014,.243),(.016,.248)]
    lathe(s,"BottleBody",[(0,0)]+profile+[(0,.248)],glass)
    cylinder(s,"Cap",.016,.012,.249,foil)
    for z in [.245,.248,.251,.254]:torus(s,f"CapThread{int(z*1000)}",.016,.0006,z,foil,n=64,sides=8)
    lathe(s,"PaperLabel",[(.0372,.051),(.0372,.135)],paper)
    lathe(s,"LabelBandLower",[(.0374,.055),(.0374,.061)],ink)
    lathe(s,"LabelBandUpper",[(.0374,.123),(.0374,.129)],ink)
    # Label badge and narrow graphic strokes on both sides remain legible
    # from the two fixed robot cameras without external font dependencies.
    for side in [-1,1]:
        badge = UsdGeom.Sphere.Define(s,f"/Asset/LabelBadge{side:+d}".replace("-","N").replace("+","P"))
        badge.CreateRadiusAttr(1.)
        xf=UsdGeom.Xformable(badge)
        xf.AddTranslateOp().Set(Gf.Vec3d(0,side*.0375,.094))
        xf.AddScaleOp().Set(Gf.Vec3d(.014,.0004,.019))
        UsdShade.MaterialBindingAPI.Apply(badge.GetPrim()).Bind(ink)
    solid_profile_collision(s,profile+[(.016,.255)])
    finish(s,name,"Capped amber glass bottle",mass,
           ["25.5 cm sealed bottle with shoulder, narrow neck, cap and paper label.",
            "Mass approximates a partially filled bottle; contents are not simulated.",
            "Solid piecewise convex collision is appropriate to the closed vessel."])


def ceramic_mug():
    name,mass = "ceramic_mug",.48
    s = begin(name,mass,(.005,0,.074))
    glaze=material(s,"SeafoamGlaze",(.045,.36,.34),.19)
    interior=material(s,"CreamInterior",(.88,.85,.74),.25)
    base=material(s,"UnglazedFoot",(.40,.27,.16),.72)
    profile=[(.034,0),(.040,.008),(.044,.145),(.044,.16)]
    lathe(s,"OuterGlaze",[(0,0)]+profile,glaze)
    lathe(s,"InnerGlaze",[(.044,.16),(.039,.16),(.0355,.012),(0,.012)],interior)
    torus(s,"Lip",.0415,.0025,.1575,interior)
    lathe(s,"UnglazedFoot",[(.027,0),(.034,0),(.034,.003),(.027,.003)],base)
    # Elliptical loop handle in X-Z; tube segments are independent convex hulls.
    n,sides=32,12
    tube=.0065
    centers=[(.045+.026*math.cos(2*math.pi*i/n),0,.083+.046*math.sin(2*math.pi*i/n)) for i in range(n)]
    pts=[]
    for i,(x,y,z) in enumerate(centers):
        a=2*math.pi*i/n
        radial=Gf.Vec3d(math.cos(a),0,math.sin(a)).GetNormalized()
        for j in range(sides):
            b=2*math.pi*j/sides
            pts.append((x+tube*math.cos(b)*radial[0],tube*math.sin(b),z+tube*math.cos(b)*radial[2]))
    faces=[[i*sides+j,((i+1)%n)*sides+j,((i+1)%n)*sides+(j+1)%sides,i*sides+(j+1)%sides] for i in range(n) for j in range(sides)]
    mesh(s,"LoopHandle",pts,faces,glaze)
    for i in range(n):
        a=pts[i*sides:(i+1)*sides]
        ni=(i+1)%n
        b=pts[ni*sides:(ni+1)*sides]
        fs=[list(reversed(range(sides))),list(range(sides,2*sides))]
        fs += [[j,(j+1)%sides,(j+1)%sides+sides,j+sides] for j in range(sides)]
        mesh(s,f"Colliders/Handle{i:02d}",a+b,fs,collision=True)
    hollow_collision(s,profile,.0045,.012)
    finish(s,name,"Tall glazed ceramic mug",mass,
           ["16 cm tall large mug, cream open interior and real loop handle.",
            "Handle collision is segmented; it does not fill the handle opening.",
            "Ceramic is a material/semantic label, not a fracture simulation."],.079)


def vase():
    name,mass="vase",.68
    s=begin(name,mass,(0,0,.091))
    glaze=material(s,"TerracottaGlaze",(.49,.17,.085),.27)
    interior=material(s,"VaseInterior",(.31,.105,.055),.46)
    band=material(s,"IvoryDecoration",(.78,.68,.49),.39)
    profile=[(.034,0),(.046,.015),(.057,.065),(.054,.12),(.035,.181),(.026,.215),(.028,.245)]
    inside=[(r-.004,z) for r,z in reversed(profile)]
    lathe(s,"OuterBody",[(0,0)]+profile,glaze)
    lathe(s,"OpenInterior",[profile[-1]]+inside+[(0,.009)],interior)
    torus(s,"RolledLip",.026,.002,.243,glaze)
    # Narrow circumferential bands follow the bulged body.
    lathe(s,"CreamBandLower",[(.0557,.059),(.0572,.065),(.0568,.072)],band)
    lathe(s,"CreamBandUpper",[(.0555,.094),(.0552,.101)],band)
    hollow_collision(s,profile,.004,.009)
    finish(s,name,"Bulbous ceramic vase",mass,
           ["24.5 cm open vase; wide belly, tapered shoulder and narrow mouth.",
            "Low center of mass and flat 6.8 cm foot support stable placement.",
            "Compound convex walls preserve the narrow open interior."],.048)


def tablet_stand():
    name,mass="tablet_stand",1.12
    s=begin(name,mass,(0,.018,.085))
    metal=material(s,"AnodizedAluminum",(.26,.29,.32),.28,.85)
    dark=material(s,"DarkBezel",(.012,.016,.022),.28,.25)
    glass=material(s,"ScreenGlass",(.012,.035,.05),.12,.12)
    rubber=material(s,"Rubber",(.022,.025,.027),.85)
    cyan=material(s,"DisplayCyan",(.045,.35,.44),.36,emissive=(.02,.13,.17))
    pale=material(s,"DisplayWhite",(.70,.78,.77),.42,emissive=(.08,.09,.09))
    box(s,"WeightedBase",(0,.018,.005),(.145,.132,.01),metal)
    box(s,"Colliders/Base",(0,.018,.005),(.145,.132,.01),collision=True)
    for i,x in enumerate([-.052,.052]):
        box(s,f"RubberFoot{i}",(x,.018,.001),(.018,.10,.002),rubber)
    box(s,"StandArm",(0,.030,.067),(.044,.014,.12),metal,rotate_x=12)
    box(s,"Colliders/StandArm",(0,.030,.067),(.044,.014,.12),collision=True,rotate_x=12)
    for path,colliding in [("StandHinge",False),("Colliders/StandHinge",True)]:
        hinge=UsdGeom.Cylinder.Define(s,"/Asset/"+path)
        hinge.CreateRadiusAttr(.011)
        hinge.CreateHeightAttr(.054)
        hinge.CreateAxisAttr("X")
        UsdGeom.Xformable(hinge).AddTranslateOp().Set(Gf.Vec3d(0,.024,.12))
        UsdShade.MaterialBindingAPI.Apply(hinge.GetPrim()).Bind(metal)
        if colliding:
            UsdPhysics.CollisionAPI.Apply(hinge.GetPrim())
            hinge.CreateVisibilityAttr("invisible")
    box(s,"Cradle",(0,.007,.035),(.116,.033,.008),metal)
    box(s,"Colliders/Cradle",(0,.007,.035),(.116,.033,.008),collision=True)
    # Screen local +Y is the back; -Y faces the robot. A 12 degree lean
    # gives a normal desktop stand silhouette while keeping the screen readable.
    origin=Gf.Vec3d(0,.018,.142)
    rot=Gf.Rotation(Gf.Vec3d(1,0,0),-12)
    def local(name,pos,dim,mat,collision=False):
        return box(s,name,origin+rot.TransformDir(Gf.Vec3d(*pos)),dim,mat,collision,rotate_x=-12)
    local("TabletMetalBack",(0,0,0),(.154,.008,.218),metal)
    local("Colliders/Tablet",(0,0,0),(.154,.008,.218),None,True)
    local("Bezel",(0,-.0045,0),(.151,.0015,.215),dark)
    local("Screen",(0,-.00535,-.001),(.137,.0005,.192),glass)
    local("DisplayHeader",(0,-.0057,.074),(.119,.00015,.013),cyan)
    for i in range(4):
        local(f"DisplayText{i}",(-.009,-.0057,.044-i*.014),(.090-i*.011,.00015,.0035),pale)
    local("DisplayImage",(0,-.0057,-.047),(.108,.00015,.047),cyan)
    local("CameraLens",(0,-.0055,.102),(.003,.0003,.003),glass)
    local("SideButton",(.0774,0,.057),(.0015,.004,.018),dark)
    finish(s,name,"Portrait tablet on weighted stand",mass,
           ["21.8 × 15.4 cm tablet, 12 degree lean, stable weighted desktop stand.",
            "Screen, camera, button, cradle and rubber feet are modeled separately.",
            "Single compound rigid body: tablet is secured to stand for this static-obstacle task."])


def validate():
    results=[]
    for item in CATALOG:
        s=Usd.Stage.Open(str(ROOT/item["usd_path"]))
        errors=[]
        if UsdGeom.GetStageUpAxis(s)!="Z": errors.append("wrong up axis")
        if UsdGeom.GetStageMetersPerUnit(s)!=1.:errors.append("wrong units")
        if not s.GetDefaultPrim().HasAPI(UsdPhysics.RigidBodyAPI):errors.append("missing rigid body")
        if item["collider_count"]<3:errors.append("missing compound collision")
        mat=UsdShade.MaterialBindingAPI(s.GetDefaultPrim()).ComputeBoundMaterial("physics")[0]
        if not mat or not mat.GetPrim().HasAPI(UsdPhysics.MaterialAPI):errors.append("missing physical material")
        for p in s.Traverse():
            if p.IsA(UsdGeom.Mesh):
                m=UsdGeom.Mesh(p)
                points=m.GetPointsAttr().Get()
                if not points or not all(math.isfinite(float(c)) for pt in points for c in pt):
                    errors.append(f"bad points: {p.GetPath()}")
                if sum(m.GetFaceVertexCountsAttr().Get()) != len(m.GetFaceVertexIndicesAttr().Get()):
                    errors.append(f"bad topology: {p.GetPath()}")
        results.append({"id":item["id"],"passed":not errors,"errors":errors,
                        "collider_count":item["collider_count"],"dimensions_m":item["dimensions"]})
    report={"checks":"USD units, rigid root, bound contact material, finite mesh topology, compound colliders",
            "simulation_tested":False,"assets":results,"passed":all(x["passed"] for x in results)}
    (OUT/"validation.json").write_text(json.dumps(report,indent=2)+"\n")
    if not report["passed"]:raise RuntimeError(report)


if __name__=="__main__":
    for build in [tall_glass,wine_glass,glass_bottle,ceramic_mug,vase,tablet_stand]:
        build()
    (OUT/"catalog.json").write_text(json.dumps({"assets":CATALOG},indent=2)+"\n")
    validate()
    print(json.dumps({"assets":len(CATALOG),"output":str(OUT),"validation":"passed"}))
