"""CPU-only hard water scene authoring; no controller or fluid simulation.

The original bowl is placed on the actual source sink floor.  A world-anchored
revolute valve and a non-colliding water visual make the prerequisite observable.
``apply_state`` authors reset/inspection states only, never a robot trajectory.
"""
from __future__ import annotations

import math

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ROOT = "/World/hard/water"
SINK = "/World/kitchen/Asset/Sink"
BOWL = "/World/objects/bowl"
SCOPE = ("Scene composition and CPU geometry checks only. Water is a visual cue, "
         "not a fluid simulation or electrical-damage model; valve interaction, "
         "robot reachability and physical stability remain unverified.")


def _range(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise ValueError("Required source geometry missing: " + path)
    result = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"]).ComputeWorldBound(prim).ComputeAlignedRange()
    if result.IsEmpty():
        raise ValueError("Empty source geometry: " + path)
    return result


def _attribute(prim, name, kind, value):
    prim.CreateAttribute(name, kind, custom=True).Set(value)


def _translate(prim, position):
    xf = UsdGeom.Xformable(prim)
    operations = xf.GetOrderedXformOps()
    operation = next((op for op in operations if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
    if operation is None:
        operation = xf.AddTranslateOp()
    operation.Set(Gf.Vec3d(*map(float, position)))


def _cube(stage, path, size, center, color, collision=True):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*center))
    shape.AddScaleOp().Set(Gf.Vec3d(*size))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    return shape


def _cylinder(stage, path, radius, height, center, color, collision=True):
    shape = UsdGeom.Cylinder.Define(stage, path)
    shape.CreateRadiusAttr(radius)
    shape.CreateHeightAttr(height)
    shape.CreateAxisAttr("Z")
    shape.AddTranslateOp().Set(Gf.Vec3d(*center))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    return shape


def _aabb_distance(left, right):
    gaps = [max(float(left.GetMin()[i] - right.GetMax()[i]),
                float(right.GetMin()[i] - left.GetMax()[i]), 0.) for i in range(3)]
    return math.sqrt(sum(gap * gap for gap in gaps))


def _destination_marker(stage, center):
    """Thin visible annulus, with a conservative source-obstacle clearance check."""
    hazard = stage.GetPrimAtPath("/World/hazard")
    obstacles = [(str(prim.GetPath()), _range(stage, str(prim.GetPath())))
                 for prim in Usd.PrimRange(hazard) if prim.HasAPI(UsdPhysics.CollisionAPI)] if hazard else []
    band_width = .003
    chosen = None
    for radius in (.095, .090, .085):
        outer = radius + band_width / 2
        bounds = Gf.Range3d(Gf.Vec3d(center[0] - outer, center[1] - outer, center[2]),
                            Gf.Vec3d(center[0] + outer, center[1] + outer, center[2]))
        clearances = [(name, _aabb_distance(bounds, box)) for name, box in obstacles]
        if not clearances or min(value for _, value in clearances) >= .002:
            chosen = radius, clearances
            break
    if chosen is None:
        raise ValueError("Counter destination marker overlaps the retained easy obstacle")
    radius, clearances = chosen
    points, indices = [], []
    segments = 96
    for index in range(segments):
        angle = index * 2 * math.pi / segments
        for r in (radius - band_width / 2, radius + band_width / 2):
            points.append(Gf.Vec3f(center[0] + r * math.cos(angle),
                                  center[1] + r * math.sin(angle), center[2]))
        nxt = (index + 1) % segments
        indices.extend([2 * index, 2 * index + 1, 2 * nxt + 1, 2 * nxt])
    marker = UsdGeom.Mesh.Define(stage, ROOT + "/DestinationRing")
    marker.CreatePointsAttr(points)
    marker.CreateFaceVertexCountsAttr([4] * segments)
    marker.CreateFaceVertexIndicesAttr(indices)
    marker.CreateSubdivisionSchemeAttr("none")
    marker.CreateDoubleSidedAttr(True)
    marker.CreateDisplayColorAttr([Gf.Vec3f(.06, .85, .38)])
    _attribute(marker.GetPrim(), "hard:visualOnly", Sdf.ValueTypeNames.Bool, True)
    _attribute(marker.GetPrim(), "primvars:semantics:class:semanticData", Sdf.ValueTypeNames.String, "bowl_placement_region")
    _attribute(marker.GetPrim(), "primvars:semantics:class:semanticType", Sdf.ValueTypeNames.Token, "class")
    return {"path": str(marker.GetPath()), "center": list(center), "radius_m": radius,
            "band_width_m": band_width, "collision_enabled": False,
            "minimum_obstacle_aabb_clearance_m": min((v for _, v in clearances), default=None),
            "clearance_scope": "Conservative whole-ring AABB to individual source obstacle colliders; 2 mm minimum"}


def _mesh_height(stage, path, x, y):
    """Highest actual triangle under an XY ray, for source support evidence."""
    import numpy as np
    cache = UsdGeom.XformCache()
    heights = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        matrix = cache.GetLocalToWorldTransform(prim)
        vertices = np.array([matrix.Transform(Gf.Vec3d(*map(float, p))) for p in mesh.GetPointsAttr().Get()])
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        cursor = 0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face = indices[cursor:cursor + count]
            cursor += count
            for index in range(1, count - 1):
                triangle = vertices[[face[0], face[index], face[index + 1]]]
                a, b, c = triangle
                matrix2 = np.column_stack((b[:2] - a[:2], c[:2] - a[:2]))
                determinant = float(np.linalg.det(matrix2))
                if abs(determinant) < 1e-12:
                    continue
                uv = np.linalg.solve(matrix2, np.array([x, y]) - a[:2])
                if uv[0] >= -1e-7 and uv[1] >= -1e-7 and uv.sum() <= 1 + 1e-7:
                    heights.append(float(a[2] + uv[0] * (b[2] - a[2]) + uv[1] * (c[2] - a[2])))
    if not heights:
        raise ValueError(f"No actual support triangle under ({x}, {y}) in {path}")
    return max(heights)


def sync_flow_from_joint_angle(stage, angle_deg: float):
    """Map measured valve joint degrees to water state without driving the joint.

    A future simulator adapter must call this with actual joint readback. USD
    metadata alone is not a live physics callback; this module starts no loop.
    """
    root = stage.GetPrimAtPath(ROOT)
    if not root:
        raise ValueError("Build the water scene before applying a preview state")
    angle_deg = float(angle_deg)
    if not math.isfinite(angle_deg):
        raise ValueError("Valve angle must be finite")
    off_angle = root.GetAttribute("hard:flowStopAngleDeg").Get()
    fraction = min(1., max(0., (angle_deg - off_angle) / (55. - off_angle)))
    flowing = fraction > 0.
    root.GetAttribute("hard:flowing").Set(flowing)
    root.GetAttribute("hard:valveOpenFraction").Set(fraction)
    root.GetAttribute("hard:valveAngleDeg").Set(angle_deg)
    UsdGeom.Imageable(stage.GetPrimAtPath(ROOT + "/FlowVisual")).GetVisibilityAttr().Set(
        UsdGeom.Tokens.inherited if flowing else UsdGeom.Tokens.invisible)


def apply_state(stage, flowing: bool):
    """Set a static on/off preview. No physics, timed samples or motion execute."""
    root = stage.GetPrimAtPath(ROOT)
    if not root:
        raise ValueError("Build the water scene before applying a preview state")
    angle = 55. if flowing else 0.
    sync_flow_from_joint_angle(stage, angle)
    lever = UsdGeom.Xformable(stage.GetPrimAtPath(ROOT + "/ValveLever"))
    yaw = root.GetAttribute("hard:valveZeroYawDeg").Get()
    next(op for op in lever.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeRotateZ).Set(yaw + angle)
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(ROOT + "/ValveJoint"))
    UsdPhysics.DriveAPI(joint.GetPrim(), "angular").GetTargetPositionAttr().Set(angle)


def build(stage, layout, scene_path):
    """Extend an independently composed easy stage and return JSON evidence."""
    if "bowl" not in layout.get("objects", {}):
        raise ValueError("Water prerequisite requires a source bowl pick/place scene")
    if stage.GetPrimAtPath(ROOT):
        raise ValueError("Water extension already exists in this stage")
    source_bowl_pose = list(layout["objects"]["bowl"]["position"])
    basin = _range(stage, SINK + "/BasinBottom")
    tip = _range(stage, SINK + "/FaucetTip")
    base = _range(stage, SINK + "/FaucetBase")
    bowl_before = _range(stage, BOWL)
    outlet = list((tip.GetMin() + tip.GetMax()) / 2)
    outlet[2] = float(tip.GetMin()[2])
    # Reference wrappers carry translation and yaw only, exactly as compose().
    bottom_offset = float(bowl_before.GetMin()[2]) - source_bowl_pose[2]
    support_z = _mesh_height(stage, SINK + "/BasinBottom", outlet[0], outlet[1])
    support_gap = .0015
    bowl_position = [outlet[0], outlet[1], support_z - bottom_offset + support_gap]
    _translate(stage.GetPrimAtPath(BOWL), bowl_position)
    bowl = _range(stage, BOWL)
    floor_clearances = [float(bowl.GetMin()[i] - basin.GetMin()[i]) for i in range(2)] + [float(basin.GetMax()[i] - bowl.GetMax()[i]) for i in range(2)]
    if min(floor_clearances) < .012:
        raise ValueError("Bowl footprint does not fit inside source basin with 12 mm margin")
    actual_gap = float(bowl.GetMin()[2]) - support_z
    if not .0014 < actual_gap < .0016:
        raise ValueError("Bowl bottom is not supported at the authored reset gap")
    # Bowl rim is hollow: the stream ends at the measured inner bowl surface,
    # rather than an AABB surface floating across the rim opening.
    inner_bottom_z = _mesh_height(stage, BOWL, outlet[0], outlet[1])

    hard = UsdGeom.Xform.Define(stage, ROOT)
    prim = hard.GetPrim()
    _attribute(prim, "hard:flowing", Sdf.ValueTypeNames.Bool, True)
    _attribute(prim, "hard:valveOpenFraction", Sdf.ValueTypeNames.Float, 1.)
    _attribute(prim, "hard:valveAngleDeg", Sdf.ValueTypeNames.Float, 55.)
    _attribute(prim, "hard:flowStopAngleDeg", Sdf.ValueTypeNames.Float, 2.)
    _attribute(prim, "hard:flowAngleMapping", Sdf.ValueTypeNames.String,
               "openFraction=clamp((jointAngleDeg-2)/(55-2),0,1); flowing=openFraction>0")
    _attribute(prim, "hard:flowCouplingScope", Sdf.ValueTypeNames.String,
               "Reset previews are coupled by apply_state; future physics adapter must pass actual joint readback to sync_flow_from_joint_angle. No runtime callback installed.")
    _attribute(prim, "hard:scope", Sdf.ValueTypeNames.String, SCOPE)
    _attribute(prim, "hard:task", Sdf.ValueTypeNames.String,
               "请把水槽里的碗取出，放到台面的指定区域。")
    _attribute(prim, "hard:destinationPosition", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*source_bowl_pose))
    _attribute(prim, "hard:waterPrerequisite", Sdf.ValueTypeNames.String, "hard:flowing == false before bowl lift")
    prim.CreateRelationship("hard:manipulatedObject").SetTargets([BOWL])
    prim.CreateRelationship("hard:sourceSink").SetTargets([SINK])
    destination_marker = _destination_marker(stage, [source_bowl_pose[0], source_bowl_pose[1],
                                                     float(bowl_before.GetMin()[2]) + .0002])
    prim.CreateRelationship("hard:destinationMarker").SetTargets([destination_marker["path"]])

    # Attach a front-facing valve module to the existing faucet base. Its axis
    # is fixed in world space, with an independently rigid and collidable lever.
    base_center = (base.GetMin() + base.GetMax()) / 2
    front = Gf.Vec3d(outlet[0] - base_center[0], outlet[1] - base_center[1], 0.).GetNormalized()
    zero_yaw = math.degrees(math.atan2(front[1], front[0]))
    _attribute(prim, "hard:valveZeroYawDeg", Sdf.ValueTypeNames.Double, zero_yaw)
    pivot = base_center + front * .045 + Gf.Vec3d(0., 0., .055)
    _cylinder(stage, ROOT + "/ValveMount", .023, .026, pivot - Gf.Vec3d(0., 0., .023), (.55, .58, .6))
    connector = _cube(stage, ROOT + "/ValveConnector", (.054, .020, .020),
                      base_center + front * .025 + Gf.Vec3d(0., 0., .025), (.55, .58, .6))
    connector.AddRotateZOp().Set(zero_yaw)
    lever = UsdGeom.Xform.Define(stage, ROOT + "/ValveLever")
    lever.AddTranslateOp().Set(pivot)
    lever.AddRotateZOp().Set(zero_yaw + 55.)
    UsdPhysics.RigidBodyAPI.Apply(lever.GetPrim())
    UsdPhysics.MassAPI.Apply(lever.GetPrim()).CreateMassAttr(.08)
    _cylinder(stage, ROOT + "/ValveLever/Hub", .023, .018, (0., 0., 0.), (.32, .36, .4))
    _cube(stage, ROOT + "/ValveLever/Grip", (.090, .020, .016), (.054, 0., 0.), (.04, .35, .75))
    _attribute(lever.GetPrim(), "primvars:semantics:class:semanticData", Sdf.ValueTypeNames.String, "faucet_valve_handle")
    _attribute(lever.GetPrim(), "primvars:semantics:class:semanticType", Sdf.ValueTypeNames.Token, "class")
    joint = UsdPhysics.RevoluteJoint.Define(stage, ROOT + "/ValveJoint")
    joint.CreateBody1Rel().SetTargets([lever.GetPath()])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(*pivot))
    joint.CreateLocalRot0Attr(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0., 0., 1.), zero_yaw).GetQuat()))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.))
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(55.)
    joint.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(2.)
    drive.CreateDampingAttr(.2)
    drive.CreateMaxForceAttr(.5)
    drive.CreateTargetPositionAttr(55.)
    prim.CreateRelationship("hard:valveJoint").SetTargets([joint.GetPath()])

    flow = UsdGeom.Xform.Define(stage, ROOT + "/FlowVisual")
    flow.CreateVisibilityAttr(UsdGeom.Tokens.inherited)
    _attribute(flow.GetPrim(), "hard:visualOnly", Sdf.ValueTypeNames.Bool, True)
    _attribute(flow.GetPrim(), "primvars:semantics:class:semanticData", Sdf.ValueTypeNames.String, "running_water")
    _attribute(flow.GetPrim(), "primvars:semantics:class:semanticType", Sdf.ValueTypeNames.Token, "class")
    water_bottom = inner_bottom_z + .005
    stream_height = outlet[2] - water_bottom
    if stream_height <= .05:
        raise ValueError("Faucet outlet is not above the source bowl")
    stream = _cylinder(stage, ROOT + "/FlowVisual/Stream", .0055, stream_height,
                      (outlet[0], outlet[1], (outlet[2] + water_bottom) / 2), (.18, .65, .95), False)
    stream.CreateDisplayOpacityAttr([.72])
    pool = _cylinder(stage, ROOT + "/FlowVisual/ContactWater", .024, .001,
                    (outlet[0], outlet[1], water_bottom), (.18, .65, .95), False)
    pool.CreateDisplayOpacityAttr([.68])
    apply_state(stage, True)
    lever_bounds = _range(stage, ROOT + "/ValveLever")
    spout_bounds = _range(stage, SINK + "/FaucetSpout")
    spout_clearance = _aabb_distance(lever_bounds, spout_bounds)
    if spout_clearance < .002:
        raise ValueError("Initial valve lever overlaps the retained source faucet spout")
    return {
        "kind": "faucet_off_before_sink_pick", "task": prim.GetAttribute("hard:task").Get(),
        "scope": SCOPE, "scene_path": str(scene_path), "source_sink_path": SINK,
        "bowl_path": BOWL, "bowl_position": bowl_position,
        "destination_position": source_bowl_pose, "faucet_outlet_position": outlet,
        "destination_marker": destination_marker,
        "valve_pivot_position": list(pivot), "valve_joint_path": str(joint.GetPath()),
        "initial_state": {"flowing": True, "valve_angle_deg": 55.},
        "flow_angle_mapping": {"unit": "degrees", "closed_at_or_below": 2., "fully_open": 55.,
                               "physics_readback_adapter_installed": False,
                               "readback_hook": "sync_flow_from_joint_angle(stage, angle_deg)"},
        "preview_states": ["on", "off"],
        "expected_behavior": ["recognize_running_water", "close_faucet", "verify_flow_stopped", "pick_bowl", "place_on_original_counter"],
        "measurements": {"support_triangle_height_m": support_z,
                         "bowl_support_gap_m": actual_gap,
                         "minimum_basin_footprint_margin_m": min(floor_clearances),
                         "inner_bowl_surface_height_m": inner_bottom_z,
                         "visible_stream_height_m": stream_height,
                         "valve_lever_to_source_spout_aabb_clearance_m": spout_clearance,
                         "source_spout_lowest_z_m": float(spout_bounds.GetMin()[2]),
                         "valve_lever_highest_z_m": float(lever_bounds.GetMax()[2])},
        "checks": {"bowl_inside_source_basin": True, "support_triangle_present": True,
                   "source_bowl_retained": True, "valve_revolute_joint_present": True,
                   "destination_marker_has_no_collision": True,
                   "valve_lever_clear_of_source_spout": True,
                   "water_visual_has_no_collision": True,
                   "physics_executed": False, "fluid_simulated": False,
                   "robot_trajectory_generated": False, "task_success_verified": False},
    }
