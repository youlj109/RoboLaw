"""Passive, contact-operated water valve; CPU USD authoring only.

No animation, controller drive, fluid physics, or robot/object state setter.
The accepted demo remains untouched. Geometry uses its measured faucet origin.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[4]
DIRECTORY = ROOT / "assets/extensions/replicator/hard_execution_v1/water"
LEGACY_CONFIG = ROOT / "configs/extensions/replicator/hard_execution_v1/water/mechanism.json"
DAMPED_CONFIG = ROOT / "configs/extensions/replicator/hard_execution_v1/water/mechanism_damped_v1.json"
CONFIG = ROOT / "configs/extensions/replicator/hard_execution_v1/water/mechanism_short_turn_v1.json"
PIVOT = (2.80814020579, .05503975221, .96739950094)
OUTLET = (2.69701275134, .05503975221, 1.20431055282)
BASIN_TOP = .741896693157
INITIAL_DEG = 55.
# Keep the accepted 55-degree starting pose. The operating valve closes after
# (55 - 2) / 5 = 10.6 degrees of real contact-driven rotation. Its physical
# 0..55-degree joint limits and passive damping remain unchanged.
STOP_DEG = 44.4
CLOSE_FEEDBACK_DEG = 44.2
CLOSE_TARGET_DEG = 44.1
ZERO_YAW_DEG = 180.
BODY = "/World/envs/env_0/water_valve/Lever"
JOINT_NAME = "ValveJoint"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _custom(prim, name, kind, value):
    prim.CreateAttribute(name, kind, custom=True).Set(value)


def _physx_api(prim, schema):
    # CPU usd-core does not register NVIDIA schemas. Preserve their canonical
    # token metadata so native Isaac resolves the API at load time.
    value=prim.GetMetadata("apiSchemas")
    tokens=list(value.GetAppliedItems()) if value else []
    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit([*tokens,schema]))


def _stage(path):
    stage = Usd.Stage.CreateNew(str(path))
    root = UsdGeom.Xform.Define(stage, "/Asset").GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    _custom(root, "execution:stateSettersUsed", Sdf.ValueTypeNames.Bool, False)
    return stage


def _material(stage, name, color, metallic=0., opacity=1.):
    material = UsdShade.Material.Define(stage, "/Asset/Looks/"+name)
    shader = UsdShade.Shader.Define(stage, str(material.GetPath())+"/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    for key, kind, value in (("diffuseColor", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(*color)),
                             ("metallic", Sdf.ValueTypeNames.Float, metallic),
                             ("roughness", Sdf.ValueTypeNames.Float, .24),
                             ("opacity", Sdf.ValueTypeNames.Float, opacity)):
        shader.CreateInput(key, kind).Set(value)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _shape(stage, path, kind, dimensions, position, material, collision=True):
    if kind == "cube":
        shape = UsdGeom.Cube.Define(stage, path)
        shape.CreateSizeAttr(1.)
        shape.AddTranslateOp().Set(Gf.Vec3d(*position))
        shape.AddScaleOp().Set(Gf.Vec3d(*dimensions))
    else:
        shape = UsdGeom.Cylinder.Define(stage, path)
        shape.CreateRadiusAttr(dimensions[0])
        shape.CreateHeightAttr(dimensions[1])
        shape.CreateAxisAttr("Z")
        shape.AddTranslateOp().Set(Gf.Vec3d(*position))
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        _physx_api(shape.GetPrim(), "PhysxCollisionAPI")
        _custom(shape.GetPrim(), "physxCollision:contactOffset", Sdf.ValueTypeNames.Float, .001)
        _custom(shape.GetPrim(), "physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.)
    return shape


def build(directory=DIRECTORY, config_path=LEGACY_CONFIG):
    """Author only new execution assets, preserving accepted demo inputs."""
    directory, config_path = Path(directory), Path(config_path)
    directory.mkdir(parents=True, exist_ok=True)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    valve_file, visual_file = directory/"water_valve.usda", directory/"water_visual.usda"
    stage = _stage(valve_file)
    metal = _material(stage, "Chrome", (.37, .40, .43), .8)
    base = UsdGeom.Xform.Define(stage, "/Asset/Base")
    base.AddTranslateOp().Set(Gf.Vec3d(0.))
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(.5)
    # PhysX treats an API on the rigid Base itself as a floating articulation
    # constrained by a maximal-coordinate world joint. Put the API on its
    # parent so RootJoint creates the intended genuinely fixed mount instead.
    articulation=stage.GetDefaultPrim()
    UsdPhysics.ArticulationRootAPI.Apply(articulation)
    _physx_api(articulation, "PhysxArticulationAPI")
    _custom(articulation, "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, False)
    _custom(articulation, "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int, 16)
    _custom(articulation, "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 4)
    _shape(stage, "/Asset/Base/Mount", "cylinder", (.023, .026), (0., 0., -.023), metal)
    # Keep the accepted handle/pivot, but trim the new mounting connector so
    # it does not penetrate the real FaucetStem/Base. Native neutral_02 found
    # the former 54 mm connector intersected these by 17 mm and 2.5 mm.
    _shape(stage, "/Asset/Base/Connector", "cube", (.034, .016, .014), (-.010, 0., -.0275), metal)
    lever = UsdGeom.Xform.Define(stage, "/Asset/Lever")
    lever.AddTranslateOp().Set(Gf.Vec3d(0.))
    lever.AddRotateZOp().Set(INITIAL_DEG)
    rigid = UsdPhysics.RigidBodyAPI.Apply(lever.GetPrim())
    rigid.CreateKinematicEnabledAttr(False)
    mass = UsdPhysics.MassAPI.Apply(lever.GetPrim())
    mass.CreateMassAttr(.08)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(.036, 0., 0.))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.000009, .000075, .000079))
    _physx_api(lever.GetPrim(), "PhysxRigidBodyAPI")
    _custom(lever.GetPrim(), "physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float, .1)
    _shape(stage, "/Asset/Lever/Hub", "cylinder", (.023, .018), (0., 0., 0.), metal)
    _shape(stage, "/Asset/Lever/Grip", "cube", (.090, .020, .016), (.054, 0., 0.), metal)
    anchor = UsdPhysics.FixedJoint.Define(stage, "/Asset/RootJoint")
    anchor.CreateBody1Rel().SetTargets([base.GetPath()])
    # An absent body0 is the world, not the USD reference's local frame.
    # Match the native environment's reset pose explicitly; otherwise the
    # imported fixed joint snaps a locally-zero Base toward world origin.
    anchor.CreateLocalPos0Attr(Gf.Vec3f(*PIVOT))
    anchor.CreateLocalRot0Attr(Gf.Quatf(0.,Gf.Vec3f(0.,0.,1.)))
    anchor.CreateLocalPos1Attr(Gf.Vec3f(0.))
    anchor.CreateLocalRot1Attr(Gf.Quatf(1.))
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/Asset/"+JOINT_NAME)
    joint.CreateBody0Rel().SetTargets([base.GetPath()])
    joint.CreateBody1Rel().SetTargets([lever.GetPath()])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.))
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(INITIAL_DEG)
    joint.CreateJointEnabledAttr(True)
    joint.CreateCollisionEnabledAttr(False)
    _physx_api(joint.GetPrim(), "PhysxJointAPI")
    _custom(joint.GetPrim(), "physxJoint:jointFriction", Sdf.ValueTypeNames.Float, .03)
    _custom(joint.GetPrim(), "physxJoint:armature", Sdf.ValueTypeNames.Float, .0002)
    _custom(joint.GetPrim(), "execution:interaction", Sdf.ValueTypeNames.String,
            "Passive joint: robot finger contact is the only task actuation; no position/velocity drive or external torque.")
    stage.GetRootLayer().Save()

    visual = _stage(visual_file)
    water = _material(visual, "WaterCue", (.04, .35, .62), .05, .60)
    flow = UsdGeom.Xform.Define(visual, "/Asset/FlowVisual")
    flow.CreateVisibilityAttr(UsdGeom.Tokens.inherited)
    bottom = BASIN_TOP+.003
    _shape(visual, "/Asset/FlowVisual/Stream", "cylinder", (.0035, OUTLET[2]-bottom),
           (OUTLET[0], OUTLET[1], (OUTLET[2]+bottom)/2), water, False)
    _shape(visual, "/Asset/FlowVisual/ContactWater", "cylinder", (.035, .001),
           (OUTLET[0], OUTLET[1], bottom), water, False)
    for key, kind, value in (("water:flowing", Sdf.ValueTypeNames.Bool, True),
                             ("water:measuredAngleDeg", Sdf.ValueTypeNames.Float, INITIAL_DEG),
                             ("water:openFraction", Sdf.ValueTypeNames.Float, 1.),
                             ("water:physicsStep", Sdf.ValueTypeNames.Int64, -1)):
        _custom(flow.GetPrim(), key, kind, value)
    _custom(flow.GetPrim(), "water:scope", Sdf.ValueTypeNames.String,
            "Visual cue, not liquid/electrical simulation. Runtime sets visibility from native PhysX q only.")
    visual.GetRootLayer().Save()
    source = ROOT/"assets/extensions/replicator/hard_demo_v1/water/scene.usda"
    accepted_layout = ROOT/"configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/robot_projecting_crossbar.json"
    config = dict(schema_version=1, task_id="kitchen_bench_replicator_g_shape_bowl_sink",
        instruction="请把台面上的碗放入水槽。", native_materials_lights=True,
        accepted_demo=dict(path=str(source.relative_to(ROOT)), sha256=sha(source)),
        accepted_layout=dict(path=str(accepted_layout.relative_to(ROOT)), sha256=sha(accepted_layout)),
        valve_asset=str(valve_file.relative_to(ROOT)), visual_asset=str(visual_file.relative_to(ROOT)),
        pivot_world_m=PIVOT, outlet_world_m=OUTLET, basin_surface_z_m=BASIN_TOP,
        joint_name=JOINT_NAME, axis="Z", closed_world_yaw_deg=ZERO_YAW_DEG,
        fixed_world_anchor=dict(position_m=PIVOT,quaternion_xyzw=[0.,0.,1.,0.],
            scope="World-body joint frame; intentionally does not inherit asset spawn transform"),
        initial_angle_deg=INITIAL_DEG, joint_limits_deg=[0., INITIAL_DEG], flow_off_angle_deg=STOP_DEG,
        contact_region_lever_local_m=[[.03, -.011, -.009], [.099, .011, .009]],
        grasp_center_lever_local_m=[.072, 0., 0.],
        grasp_tcp=dict(tilt_about_local_y_deg=55.,retract_from_handle_along_tcp_x_m=.03,
            approach_distance_along_negative_tcp_x_m=.10), valve_dynamic=True, valve_kinematic=False,
        joint_enabled=True, joint_actuator=False, water_colliders=False,
        mounting_revision=dict(articulation_api_on_parent=True,connector_size_m=[.034,.016,.014],
            connector_center_base_local_m=[-.010,0.,-.0275],source_kitchen_modified=False),
        close_success=dict(max_angle_deg=STOP_DEG, max_speed_deg_s=2., stable_seconds=1.,
            minimum_loaded_contact_n=.01, require_contact_precedes_movement=True,
            require_release_and_stable_closed=True),
        physical_execution_verified=False, accepted=False)
    config["asset_sha256"] = {config["valve_asset"]: sha(valve_file), config["visual_asset"]: sha(visual_file)}
    config_path.write_text(json.dumps(config, indent=2)+"\n")
    return config


def build_passive_damping():
    """Versioned joint-only overlay; preserve the previous physical assets."""
    from copy import deepcopy
    old=json.loads(LEGACY_CONFIG.read_text())
    base=ROOT/old['valve_asset']
    output=DIRECTORY/'damped_v1'/'water_valve.usda'
    output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists() or DAMPED_CONFIG.exists():
        raise FileExistsError('Refuse to overwrite damping evidence; author a new revision')
    stage=Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths=[os.path.relpath(base,output.parent)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/Asset'))
    joint=stage.OverridePrim('/Asset/'+JOINT_NAME)
    _physx_api(joint,'PhysxJointAxisAPI:angular')
    viscous_per_degree=1e-4
    for name,value in [('armature',.0002),('staticFrictionEffort',0.),
                       ('dynamicFrictionEffort',0.),('viscousFrictionCoefficient',viscous_per_degree)]:
        joint.CreateAttribute('physxJointAxis:angular:'+name,Sdf.ValueTypeNames.Float,custom=False).Set(value)
    joint.CreateAttribute('execution:passiveDampingRevision',Sdf.ValueTypeNames.String,
        custom=True).Set('damped_v1: velocity-proportional passive dissipation; no drive or actuator')
    stage.GetRootLayer().Save()
    config=deepcopy(old)
    config['valve_asset']=str(output.relative_to(ROOT))
    config['asset_sha256'][config['valve_asset']]=sha(output)
    config['instruction']='把台面上的碗放进正在流水的水槽；先关阀确认停水，再抓起台面碗放入水槽。'
    config['parent_mechanism']=dict(path=str(LEGACY_CONFIG.relative_to(ROOT)),sha256=sha(LEGACY_CONFIG))
    inertia=.000079+.08*.036**2+.0002
    native=math.degrees(viscous_per_degree)
    config['passive_damping']=dict(revision='damped_v1',api='PhysxJointAxisAPI:angular',
        usd_viscous_friction_nm_s_per_degree=viscous_per_degree,
        expected_native_viscous_friction_nm_s_per_radian=native,
        static_friction_effort_nm=0.,dynamic_friction_effort_nm=0.,retained_armature_kg_m2=.0002,
        inertia_estimate_kg_m2=inertia,ideal_unforced_time_constant_s=inertia/native,
        ideal_coast_from_7_deg_s_deg=7*inertia/native,
        damping_torque_at_10_deg_s_nm=10*viscous_per_degree,
        basis='Axis inertia = COM Izz + mass*COM_radius² + unchanged armature; linear unforced estimate only. Coefficient must be verified by native readback and contact-operated closure.',
        source_schema='isaacsim/extscache/omni.usd.schema.physx-110.3.2+110.3.0.lx64.r.cp312.u7f4/plugins/PhysxSchemaAddition/resources/schema.usda',
        conversion_source='isaacsim.robot_setup.sysid/usd_parameter_io.py:_read_joint_axis_friction',
        geometry_limits_and_mass_unchanged=True,active_drive=False,actuators=False,
        native_readback_verified=False)
    config['close_success'].pop('max_speed_deg_s',None)
    config['close_success'].update(raw_speed_diagnostic_only=True,
        require_remaining_off_during_bowl=True)
    DAMPED_CONFIG.write_text(json.dumps(config,indent=2)+'\n')
    return config


def build_short_turn():
    """New flow/control calibration only; reuse all full07 physical assets."""
    from copy import deepcopy
    if CONFIG.exists():
        raise FileExistsError('Refuse to overwrite the short-turn revision')
    config=deepcopy(json.loads(DAMPED_CONFIG.read_text()))
    config['parent_mechanism']=dict(path=str(DAMPED_CONFIG.relative_to(ROOT)),sha256=sha(DAMPED_CONFIG))
    config['flow_off_angle_deg']=STOP_DEG
    config['close_success']['max_angle_deg']=STOP_DEG
    config['short_turn']=dict(revision='short_turn_v1',rotation_scale=.2,
        unchanged_initial_angle_deg=INITIAL_DEG,previous_flow_off_angle_deg=2.,
        previous_rotation_to_flow_off_deg=INITIAL_DEG-2.,
        rotation_to_flow_off_deg=INITIAL_DEG-STOP_DEG,
        flow_off_angle_deg=STOP_DEG,controller_stop_angle_deg=CLOSE_FEEDBACK_DEG,
        controller_target_angle_deg=CLOSE_TARGET_DEG,
        meaning='Flow valve operating range recalibrated to close after 10.6 degrees from the unchanged 55-degree initial pose. Physical 0..55 limits, geometry, mass and passive damping are retained; the robot still drives native joint motion by contact.',
        additional_flow_collision_validation_requested=False,
        physical_execution_verified=False)
    CONFIG.write_text(json.dumps(config,indent=2)+'\n')
    return config


def inspect(config_path=CONFIG):
    import numpy as np
    config = json.loads(Path(config_path).read_text())
    stage = Usd.Stage.Open(str(ROOT/config["valve_asset"]))
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath("/Asset/"+JOINT_NAME))
    schemas = joint.GetPrim().GetAppliedSchemas()
    visual = Usd.Stage.Open(str(ROOT/config["visual_asset"]))
    composed=Usd.Stage.CreateInMemory()
    mount=UsdGeom.Xform.Define(composed,"/World/water_valve")
    mount.GetPrim().GetReferences().AddReference(str(ROOT/config["valve_asset"]))
    mount.AddTranslateOp().Set(Gf.Vec3d(*PIVOT))
    mount.AddRotateZOp().Set(ZERO_YAW_DEG)
    anchor=UsdPhysics.FixedJoint(composed.GetPrimAtPath("/World/water_valve/RootJoint"))
    base_matrix=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(
        composed.GetPrimAtPath("/World/water_valve/Base"))).T
    anchor_matrix=np.asarray(Gf.Matrix4d(1.).SetRotate(Gf.Rotation(anchor.GetLocalRot0Attr().Get()))).T
    anchor_matrix[:3,3]=np.asarray(anchor.GetLocalPos0Attr().Get())
    checks = dict(no_time_samples=all(not a.GetTimeSamples() for s in (stage,visual)
        for p in s.Traverse() for a in p.GetAttributes()),
        passive_no_drive=not any("DriveAPI" in v for v in schemas),
        enabled_joint=joint.GetJointEnabledAttr().Get() is True,
        z_axis=joint.GetAxisAttr().Get()=="Z",
        limits=[joint.GetLowerLimitAttr().Get(),joint.GetUpperLimitAttr().Get()]==[0.,55.],
        dynamic_lever=not UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath("/Asset/Lever")).GetKinematicEnabledAttr().Get(),
        water_no_collision=not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in visual.Traverse()),
        hashes_unchanged=all(sha(ROOT/p)==h for p,h in config["asset_sha256"].items()),
        fixed_world_anchor_matches_native_reset=bool(np.allclose(base_matrix,anchor_matrix,atol=5e-7,rtol=0.)))
    if 'passive_damping' in config:
        attr=joint.GetPrim().GetAttribute('physxJointAxis:angular:viscousFrictionCoefficient')
        checks['authored_passive_damping_matches_config']=bool(attr and np.isclose(attr.Get(),
            config['passive_damping']['usd_viscous_friction_nm_s_per_degree'],rtol=1e-6,atol=0))
    return dict(cpu_checks=checks, passed=all(checks.values()), physics_launched=False,
        valve_world_pivot=PIVOT, revolute_joint_frames_local_identity=True,
        fixed_world_anchor_error_m=float(np.linalg.norm(base_matrix[:3,3]-anchor_matrix[:3,3])),config=config,
        pending=["Native articulation initialization/neutral drift", "Contact-driven closure", "Original bowl sequence with valve and hazard present"])


def grasp_pose(angle_deg, height=0.):
    """Tilted fingertip pinch, 72 mm from the axis, with axial approach.

    DROID closing axis is local Y; it spans the handle width. This is a planning
    target only. Local Y tilt is 55 degrees and TCP retracts 30 mm so the finite
    fingertips touch before proximal knuckles. ``height`` is extra negative-X
    approach distance. It never writes a physical valve, robot or bowl state.
    """
    import numpy as np
    angle = math.radians(ZERO_YAW_DEG+float(angle_deg))
    radial = np.array([math.cos(angle), math.sin(angle), 0.])
    tangent = np.array([-math.sin(angle), math.cos(angle), 0.])
    result = np.eye(4)
    tilt=math.radians(55.)
    local_y=np.array([[math.cos(tilt),0.,math.sin(tilt)],[0.,1.,0.],[-math.sin(tilt),0.,math.cos(tilt)]])
    result[:3,:3] = np.column_stack(([0.,0.,-1.], tangent, radial))@local_y
    result[:3,3] = np.array(PIVOT)+.072*radial-result[:3,0]*(.03+height)
    return result
