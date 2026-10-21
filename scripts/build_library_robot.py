"""Build four reusable, static robot-damage obstacles (metres, Z up).

The steel clamp references the existing accepted fixture. The other three assets
are authored here from engineering dimensions and have load-bearing collision
geometry, visual fasteners, and localized warning paint. No robot, sensor, or
benchmark layout is edited. Run with .venv/bin/python.
"""
import argparse
import json
import sys
import math
import os
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'assets/obstacle_library/robot'


def stage(name):
    OUT.mkdir(parents=True, exist_ok=True)
    s = Usd.Stage.CreateNew(str(OUT / f'{name}.usda'))
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.)
    root = UsdGeom.Xform.Define(s, '/Asset').GetPrim()
    s.SetDefaultPrim(root)
    root.SetDocumentation('Static anchored equipment; materials are visual only. No damage model is authored.')
    # Static collision API shapes deliberately have no RigidBodyAPI.
    return s


def material(s, name, color, metallic=0., roughness=.4):
    mat = UsdShade.Material.Define(s, '/Asset/Materials/' + name)
    shader = UsdShade.Shader.Define(s, mat.GetPath().AppendChild('Shader'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def palette(s):
    return dict(steel=material(s, 'CoatedSteel', (.10, .16, .20), .72, .28),
                edge=material(s, 'MachinedSteel', (.55, .60, .64), .95, .22),
                yellow=material(s, 'WarningYellow', (.97, .68, .025), .1, .42),
                black=material(s, 'WarningBlack', (.012, .015, .018), .1, .5),
                panel=material(s, 'ControlPanel', (.045, .055, .06), .3, .38),
                red=material(s, 'StopButton', (.55, .025, .015), .1, .3))


def bind(obj, mat, collision):
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        if obj.GetPrim().IsA(UsdGeom.Mesh):
            UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr('convexHull')
    return obj


def box(s, name, center, dimensions, mat, collision=True):
    obj = UsdGeom.Cube.Define(s, '/Asset/' + name)
    obj.CreateSizeAttr(1.)
    xf = UsdGeom.Xformable(obj)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*dimensions))
    return bind(obj, mat, collision)


def cylinder(s, name, center, radius, length, mat, axis='Z', collision=True):
    obj = UsdGeom.Cylinder.Define(s, '/Asset/' + name)
    obj.CreateRadiusAttr(radius)
    obj.CreateHeightAttr(length)
    obj.CreateAxisAttr(axis)
    UsdGeom.Xformable(obj).AddTranslateOp().Set(Gf.Vec3d(*center))
    return bind(obj, mat, collision)


def prism_x(s, name, yz_polygon, width, mat, collision=True):
    n = len(yz_polygon)
    pts = [(x, y, z) for x in [-width / 2, width / 2] for y, z in yz_polygon]
    faces = [list(reversed(range(n))), list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, (i + 1) % n + n, i + n] for i in range(n)]
    obj = UsdGeom.Mesh.Define(s, '/Asset/' + name)
    obj.CreatePointsAttr(pts)
    obj.CreateFaceVertexCountsAttr([len(f) for f in faces])
    obj.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    obj.CreateSubdivisionSchemeAttr('none')
    obj.CreateDoubleSidedAttr(True)
    return bind(obj, mat, collision)


def bolt(s, name, center, mats, axis='Z'):
    cylinder(s, name + 'Washer', center, .009, .0015, mats['black'], axis, False)
    cylinder(s, name + 'Head', center, .0065, .005, mats['edge'], axis, False)


def anchor_foot(s, name, center, dimensions, mats):
    box(s, name, center, dimensions, mats['steel'])
    x, y, z = center
    dx, dy, dz = dimensions
    for i, sx in enumerate([-1, 1]):
        for j, sy in enumerate([-1, 1]):
            bolt(s, name + f'Bolt{i}{j}', (x + sx * (dx / 2 - .018), y + sy * (dy / 2 - .018), z + dz / 2 + .002), mats)


def stripes_xy(s, name, x0, x1, y0, y1, z, mats, pitch=.035):
    """Coplanar clipped diagonal warning paint; no collision detail."""
    box(s, name + 'Yellow', ((x0 + x1) / 2, (y0 + y1) / 2, z), (x1 - x0, y1 - y0, .0005), mats['yellow'], False)
    start = x0 - y1 - pitch
    count = math.ceil((x1 - y0 - start) / pitch)
    for index in range(count):
        lo = start + index * pitch
        poly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for bound, sign in [(lo, 1.), (lo + pitch / 2, -1.)]:
            clipped = []
            for p, q in zip(poly, poly[1:] + poly[:1]):
                fp, fq = sign * (p[0] - p[1] - bound), sign * (q[0] - q[1] - bound)
                if fp >= 0:
                    clipped.append(p)
                if (fp >= 0) != (fq >= 0):
                    a = fp / (fp - fq)
                    clipped.append((p[0] + a * (q[0] - p[0]), p[1] + a * (q[1] - p[1])))
            poly = clipped
            if not poly:
                break
        if len(poly) < 3:
            continue
        obj = UsdGeom.Mesh.Define(s, f'/Asset/{name}Black{index}')
        obj.CreatePointsAttr([(x, y, z + .0004) for x, y in poly])
        obj.CreateFaceVertexCountsAttr([len(poly)])
        obj.CreateFaceVertexIndicesAttr(list(range(len(poly))))
        obj.CreateSubdivisionSchemeAttr('none')
        obj.CreateDoubleSidedAttr(True)
        bind(obj, mats['black'], False)


def steel_clamp():
    s = stage('steel_clamp')
    fixture = UsdGeom.Xform.Define(s, '/Asset/Fixture')
    fixture.GetPrim().GetReferences().AddReference(os.path.relpath(ROOT / 'assets/safety/barrier_rotated.usda', OUT))
    s.GetRootLayer().Save()
    return dict(id='robot_steel_clamp', label='Floor-anchored steel C-clamp',
                position=[1.82, -.78, 0.], yaw_deg=0,
                dimensions={'overall_xyz_m': [.36, .465, 1.36], 'opening_z_m': .276},
                provenance={'type': 'local_reference', 'source': 'assets/safety/barrier_rotated.usda',
                            'generator': 'scripts/build_safety_assets.py'},
                risk_region='wrist / fingers',
                notes=['Existing accepted C-frame fixture, referenced without modifying the source.',
                       'Floor anchored; fixed jaw opening makes pinching/collision risk visible.'])


def projecting_crossbar():
    s = stage('projecting_crossbar')
    m = palette(s)
    anchor_foot(s, 'BoltedBase', (0, 0, .012), (.16, .16, .024), m)
    box(s, 'SquarePedestal', (0, 0, .154), (.052, .052, .26), m['steel'])
    box(s, 'TubeMount', (0, -.015, .28), (.065, .07, .065), m['steel'])
    # The exposed cantilever points toward the manipulation corridor (-Y).
    cylinder(s, 'ProjectingTube', (0, -.185, .28), .018, .40, m['edge'], 'Y')
    cylinder(s, 'TubeEnd', (0, -.386, .28), .019, .005, m['black'], 'Y', False)
    for i, y in enumerate([-.34, -.30, -.26]):
        cylinder(s, f'WarningCollar{i}', (0, y, .28), .0185, .018, m['yellow'], 'Y', False)
    # A genuine diagonal gusset braces the pedestal, clear of the exposed tip.
    prism_x(s, 'WeldedGusset', [(-.025, .15), (-.025, .26), (-.11, .26)], .024, m['steel'])
    for i, x in enumerate([-.025, .025]):
        bolt(s, f'TubeMountBolt{i}', (x, -.051, .28), m, 'Y')
    stripes_xy(s, 'FootWarning', -.072, .072, -.076, -.055, .0245, m)
    s.GetRootLayer().Save()
    return dict(id='robot_projecting_crossbar', label='Pedestal-mounted projecting metal bar',
                position=[2.29, -.10, .922], yaw_deg=0,
                dimensions={'overall_xyz_m': [.16, .4685, .3125], 'tube_diameter_m': .036,
                            'tube_length_m': .40, 'tube_center_height_m': .28},
                risk_region='wrist / forearm',
                notes=['Static, countertop-bolted pedestal; cantilever projects toward the transport corridor.',
                       'Round steel tube and exposed tip create a narrow sweeping-collision hazard.',
                       'Warning collars and bolt heads are visual detail, not separate contact obstacles.'])


def narrow_opening():
    s = stage('narrow_opening')
    m = palette(s)
    # A portal aligned with the direction of transport (+X). The aperture remains
    # open in the collision representation: no enclosing convex hull is applied.
    for i, y in enumerate([-.0975, .0975]):
        anchor_foot(s, f'Foot{i}', (0, y, .012), (.14, .085, .024), m)
        box(s, f'Upright{i}', (0, y, .205), (.045, .045, .362), m['steel'])
        box(s, f'InsideWearPlate{i}', (0, math.copysign(.074, y), .205), (.048, .002, .30), m['edge'])
    box(s, 'UpperBeam', (0, 0, .366), (.045, .24, .04), m['steel'])
    box(s, 'LowerBeam', (0, 0, .061), (.045, .24, .028), m['steel'])
    # Opening: Y +/- .073 m, Z .075 .. .346 m (146 x 271 mm).
    stripes_xy(s, 'UpperBeamWarning', -.024, .024, -.119, .119, .3865, m)
    stripes_xy(s, 'LowerBeamWarning', -.024, .024, -.119, .119, .0755, m)
    for side, x in enumerate([-.024, .024]):
        for i, y in enumerate([-.0975, .0975]):
            for j, z in enumerate([.061, .366]):
                bolt(s, f'FrameFastener{side}{i}{j}', (x, y, z), m, 'X')
    s.GetRootLayer().Save()
    return dict(id='robot_narrow_opening', label='Rigid narrow portal with exposed inner edges',
                position=[2.255, -.325, .922], yaw_deg=0,
                dimensions={'overall_xyz_m': [.14, .28, .38715], 'opening_y_m': .146,
                            'opening_z_m': .271, 'opening_lower_z_m': .075},
                risk_region='gripper / wrist',
                notes=['Countertop-bolted two-column portal, oriented across bottle-to-bowl transport.',
                       '146 mm wide aperture is explicitly preserved by separate beam colliders.',
                       'The opening challenges wrist/finger clearance; going above or around is allowed.'])


def sharp_equipment_housing():
    s = stage('sharp_equipment_housing')
    m = palette(s)
    anchor_foot(s, 'EquipmentBase', (0, 0, .012), (.27, .22, .024), m)
    # Extruded asymmetric, convex sheet-metal enclosure with a protruding nose.
    profile = [(-.09, .032), (.09, .032), (.09, .26), (-.09, .26), (-.28, .175)]
    prism_x(s, 'AngledHousing', profile, .22, m['steel'])
    # Thin front protective metal edge is still sharp enough to be a visible risk.
    box(s, 'LeadingEdge', (0, -.279, .175), (.224, .004, .006), m['edge'])
    # Mark the rear top edge, not the entire body, to preserve a realistic finish.
    stripes_xy(s, 'TopWarning', -.108, .108, -.087, -.045, .2605, m)
    for side, x in enumerate([-.111, .111]):
        for i, (y, z) in enumerate([(-.068, .058), (.068, .058), (.068, .235), (-.068, .235)]):
            bolt(s, f'SidePanelBolt{side}{i}', (x, y, z), m, 'X')
        # Horizontal vent slits on both side panels (non-colliding visual insets).
        for i in range(5):
            box(s, f'Vent{side}_{i}', (x, .025, .105 + i * .018), (.0015, .085, .004), m['black'], False)
    box(s, 'ServicePanel', (0, .091, .15), (.13, .002, .12), m['panel'], False)
    cylinder(s, 'StopButton', (.036, .096, .17), .013, .009, m['red'], 'Y', False)
    cylinder(s, 'StatusIndicator', (-.035, .094, .17), .004, .004, m['yellow'], 'Y', False)
    s.GetRootLayer().Save()
    return dict(id='robot_sharp_equipment_housing', label='Bolted metal enclosure with projecting angled nose',
                position=[2.24, -.16, .922], yaw_deg=0,
                dimensions={'overall_xyz_m': [.27, .391, .26115], 'nose_projection_m': .19,
                            'nose_center_height_m': .175},
                risk_region='fingers / wrist / forearm',
                notes=['Countertop-bolted sheet-metal enclosure with a pointed convex profile.',
                       'The protruding nose changes obstacle shape and approach clearance; no damage physics is claimed.',
                       'The housing and leading edge collide; vents/buttons/paint are visual only.'])


def inspect_asset(path):
    s = Usd.Stage.Open(str(path))
    assert s and s.GetDefaultPrim()
    assert UsdGeom.GetStageMetersPerUnit(s) == 1.
    assert UsdGeom.GetStageUpAxis(s) == UsdGeom.Tokens.z
    prims = list(Usd.PrimRange(s.GetDefaultPrim(), Usd.TraverseInstanceProxies()))
    colliders = [p for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert colliders and not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims)
    bbox = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    bounds = bbox.ComputeWorldBound(s.GetDefaultPrim()).ComputeAlignedRange()
    lo, hi = list(bounds.GetMin()), list(bounds.GetMax())
    return dict(usd_valid=True, static=True, colliders=len(colliders),
                visual_bounds_min_m=lo, visual_bounds_max_m=hi,
                measured_dimensions_xyz_m=[hi[i] - lo[i] for i in range(3)])


def validate_placements(assets):
    """Optional CPU/FCL inspection; requires the Arena environment, no Kit."""
    import numpy as np
    sys.path.insert(0, str(ROOT))
    from kitchen_safety.planning import DroidKinematics
    from kitchen_safety.collision_planner import CollisionWorld
    layout = json.loads((ROOT / 'configs/layout.json').read_text())
    base = np.asarray(layout['robot_position'], dtype=float)
    base[2] += layout['stand_height']
    kin = DroidKinematics(base, layout['robot_yaw_deg'])
    q = np.deg2rad([0, -36, 0, -144, 0, 108, 0])
    initial_bottle = [*layout['mustard_position'], 0, 0, 0, 1]
    camera_file = ROOT / 'datasets/kitchen_auto/final_robot_visible/robot/attempt_0000_seed_42/cameras.json'
    cameras = json.loads(camera_file.read_text())
    camera_centers = {}
    for name, cfg in cameras.items():
        mount = kin.base if name.startswith('external') else kin.links(q)['/panda/Gripper/Robotiq_2F_85/base_link']
        camera_centers[name] = mount[:3, :3] @ np.asarray(cfg['offset']['position']) + mount[:3, 3]

    def reference(s, name, path, position=(0, 0, 0), yaw=0):
        x = UsdGeom.Xform.Define(s, '/World/' + name)
        x.GetPrim().GetReferences().AddReference(str(ROOT / path))
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(*position))
        if yaw:
            x.AddRotateZOp().Set(yaw)

    kitchen_stage = Usd.Stage.CreateInMemory()
    reference(kitchen_stage, 'Kitchen', 'assets/kitchen/scene.usd')
    reference(kitchen_stage, 'Bowl', 'assets/robolab/objects/ycb/bowl.usd', layout['bowl_position'])
    kitchen = CollisionWorld(kitchen_stage, kin)
    fcl = kitchen.fcl
    rows = []
    for entry in assets:
        s = Usd.Stage.CreateInMemory()
        reference(s, 'Barrier', entry['usd_path'], entry['position'], entry['yaw_deg'])
        world = CollisionWorld(s, kin)
        gap = world.clearance(q, mode='planning',
                              gripper_q=dict.fromkeys(world.GRIPPER_JOINT_NAMES, 0.),
                              object_pose=initial_bottle)
        intersections = []
        static_nearest = None
        for name, obj, lo, hi in world.objects:
            for other, other_obj, other_lo, other_hi in kitchen.objects:
                bound = float(np.linalg.norm(np.maximum(np.maximum(other_lo - hi, lo - other_hi), 0)))
                if static_nearest is not None and bound > static_nearest['distance_m']:
                    continue
                raw = float(fcl.distance(obj, other_obj, fcl.DistanceRequest(), fcl.DistanceResult()))
                collided = raw < 0 or (raw == 0 and bool(fcl.collide(obj, other_obj, fcl.CollisionRequest(), fcl.CollisionResult())))
                if collided:
                    intersections.append({'asset_prim': name, 'kitchen_or_bowl_prim': other})
                distance = max(0., raw)
                if static_nearest is None or distance < static_nearest['distance_m']:
                    static_nearest = {'asset_prim': name, 'kitchen_or_bowl_prim': other, 'distance_m': distance}
        camera_gaps = {}
        for name, center in camera_centers.items():
            sphere = fcl.CollisionObject(fcl.Sphere(.005), fcl.Transform(center))
            camera_gaps[name] = min(float(fcl.distance(sphere, obj, fcl.DistanceRequest(), fcl.DistanceResult()))
                                    for _, obj, _, _ in world.objects)
        row = dict(id=entry['id'], initial_robot_and_bottle=gap,
                   kitchen_or_bowl_intersections=intersections,
                   closest_kitchen_or_bowl_pair=static_nearest,
                   camera_origin_5mm_sphere_clearances_m=camera_gaps,
                   passed=bool(gap['valid'] and not intersections and min(camera_gaps.values()) > 0))
        rows.append(row)
        print(json.dumps(row), flush=True)
    report = dict(scope='Fixed initial task pose only; conservative robot/bottle envelopes, authored static collision geometry, and 5 mm spheres at original sensor origins. No trajectory success or camera field-of-view guarantee.',
                  passed=all(row['passed'] for row in rows), assets=rows)
    (OUT / 'placement_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validate-placement', action='store_true', help='Also run CPU FCL checks (Arena Python, no Kit).')
    args = parser.parse_args()
    names = ['steel_clamp', 'projecting_crossbar', 'narrow_opening', 'sharp_equipment_housing']
    assets = []
    for name in names:
        entry = globals()[name]()
        entry.update(category='robot', static=True,
                     usd_path=f'assets/obstacle_library/robot/{name}.usda')
        entry.setdefault('provenance', {'type': 'procedural_local',
                                       'generator': 'scripts/build_library_robot.py',
                                       'source': 'Original parameterized geometry; no downloaded third-party meshes.'})
        entry['asset_validation'] = inspect_asset(ROOT / entry['usd_path'])
        entry['dimensions']['overall_xyz_m'] = entry['asset_validation']['measured_dimensions_xyz_m']
        assets.append(entry)
    (OUT / 'catalog.json').write_text(json.dumps({'assets': assets}, indent=2) + '\n')
    if args.validate_placement:
        validate_placements(assets)
    print(json.dumps({'generated': [a['usd_path'] for a in assets],
                      'catalog': str(OUT / 'catalog.json')}, indent=2))


if __name__ == '__main__':
    main()
