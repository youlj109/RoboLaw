"""Mount refrigerator housing and locally improve Kettle solver convergence.

This is a local fixed-installation assumption, not an upstream asset property.
The source refrigerator tips by about 0.4 degrees on its SDF floor contacts;
its un-damped hinges then convert panel gravity into autonomous door opening.
The Kettle's four source rigid bodies use 64 position / 4 velocity iterations,
selected from the retained 16/1, 32/2, 64/4 reset diagnostics. Original world
stability thresholds remain 2 mm / 1 degree. No runtime state writes, collision
filters, mass edits, door drives, or fixed door joints are introduced.
"""
from pathlib import Path
import hashlib
import json

RECIPE = "fixed-refrigerator-housing-kettle-solver64-4-v2"
HOUSING = "/World/Fridge/Fridge_OnlineVisual/Geometry/sm_rc_refrigerator_a01_body_01_obj_00"
MOUNT = "/World/Fridge/RoboLawHousingInstallation"
KETTLE_ROOT = "/World/KettleA01/KettleA01_OnlineVisual/Geometry/"
KETTLE_BODIES = tuple(KETTLE_ROOT + name for name in (
    "sm_kitchenware_kettle_a01_body_obj_01", "sm_kitchenware_kettle_a01_button_obj_00",
    "sm_kitchenware_kettle_a01_lid_obj_01", "sm_kitchenware_kettle_a01_switch_obj_00"))
SOLVER_ATTRIBUTES = {"physxRigidBody:solverPositionIterationCount": 64,
                     "physxRigidBody:solverVelocityIterationCount": 4}
# Reports are local artifacts, not a claim that large data is shipped with Git.
# Retain all predeclared outcomes. Source text hashes bind this evidence choice.
SOLVER_DIAGNOSTICS = (
    (16, 1, False, "83d9d64eed7119cd83fe91716651f4eb0064e05320604bc6b7b334ffe493dff9", "7a11efd8ccad0857e62658e4c01e27f46fadfcd65b2c6236212bdc5ce3eeb0a6"),
    (32, 2, False, "105bf3213762950b4da1e2fa5f05b188f46f1e4e3cb6b9e3377f332a7a21d41b", "78e23bd467a772e0e443dc6f939d28aca5479856dbd3832f98e52547e96b4b96"),
    (64, 4, True, "2cba788e7882f0cd8491e3cf267a8b8b0d347d76cd97b35b015298c576aa89c9", "63d77b90715a228d0edea6c046e4a4212ec9ab6adbcd7c7fcd19de400d5fdbe2"),
)


def validate_local_authoring(stage):
    """Inspect the strong layer, rejecting any extra physics/geometry edit."""
    from pxr import Sdf
    layer = stage.GetRootLayer()
    solver, added_apis, mount_attrs, mount_rels = {}, set(), set(), set()
    expected_mount_attrs = {"physics:localPos0", "physics:localPos1", "physics:localRot0", "physics:localRot1", "physics:collisionEnabled"}
    def inspect(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.AttributeSpec):
            owner = str(spec.owner.path)
            if owner in KETTLE_BODIES and spec.name in SOLVER_ATTRIBUTES and spec.typeName == Sdf.ValueTypeNames.Int:
                solver[str(path)] = int(spec.default)
            elif owner == MOUNT and spec.name in expected_mount_attrs:
                mount_attrs.add(spec.name)
            else:
                raise ValueError("Unexpected local authored attribute: " + str(path))
        elif isinstance(spec, Sdf.RelationshipSpec):
            if str(spec.owner.path) != MOUNT or spec.name not in {"physics:body0", "physics:body1"}:
                raise ValueError("Unexpected local authored relationship: " + str(path))
            mount_rels.add(spec.name)
        elif isinstance(spec, Sdf.PrimSpec):
            allowed = {"subLayers", "subLayerOffsets", "defaultPrim"} if path == Sdf.Path.absoluteRootPath else {"specifier"}
            if str(path) == MOUNT:
                allowed.add("typeName")
                if spec.typeName != "PhysicsFixedJoint":
                    raise ValueError("Housing installation type changed")
            if str(path) in KETTLE_BODIES:
                allowed.add("apiSchemas")
                if not spec.HasInfo("apiSchemas") or spec.GetInfo("apiSchemas").GetAddedOrExplicitItems() != ["PhysxRigidBodyAPI"]:
                    raise ValueError("Unexpected Kettle API schema edit")
                added_apis.add(str(path))
            if set(spec.ListInfoKeys()) - allowed:
                raise ValueError("Unexpected local authored metadata: " + str(path))
    layer.Traverse(Sdf.Path.absoluteRootPath, inspect)
    expected = {path + "." + field: value for path in KETTLE_BODIES for field, value in SOLVER_ATTRIBUTES.items()}
    if solver != expected or added_apis != set(KETTLE_BODIES) or mount_attrs != expected_mount_attrs or mount_rels != {"physics:body0", "physics:body1"}:
        raise ValueError("Incomplete source-preserving physical overlay")
    return dict(passed=True, solver_attributes=solver, physx_api_added_bodies=sorted(added_apis),
                housing_mount=MOUNT, housing_attributes=sorted(mount_attrs), housing_relationships=sorted(mount_rels),
                scope="Strong layer contains only the existing housing installation plus four PhysxRigidBodyAPI additions and eight solver values; all source mass/COM/inertia/joints/collision/xforms remain inherited.")


def mounted_kitchen(source, output_dir):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    source, output_dir = Path(source).resolve(), Path(output_dir)
    source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    recipe_sha = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{RECIPE}_{source_sha[:16]}_{recipe_sha[:12]}.usda"
    metadata = path.with_suffix('.json')
    if path.exists() and metadata.exists():
        info = json.loads(metadata.read_text())
        if hashlib.sha256(path.read_bytes()).hexdigest() != info['overlay_sha256']:
            raise ValueError(f"Modified cached refrigerator installation: {path}")
        if info['recipe'] != RECIPE or info['source_sha256'] != source_sha or info['recipe_sha256'] != recipe_sha:
            raise ValueError("Cached physical profile/source/recipe identity differs")
        validate_local_authoring(Usd.Stage.Open(str(path)))
        return path, info
    stage = Usd.Stage.CreateNew(str(path))
    stage.GetRootLayer().subLayerPaths = [str(source)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    body = stage.GetPrimAtPath(HOUSING)
    if not body or not body.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError(f"Original refrigerator housing is missing: {source}")
    frame = UsdGeom.XformCache().GetLocalToWorldTransform(body)
    rotation = frame.ExtractRotationQuat()
    position = frame.ExtractTranslation()
    # The installation is created before physics initialization at the original
    # authored placement. Reset measurements already contain an initial settling
    # perturbation and are intentionally not used as a new tilted reference.
    mount = UsdPhysics.FixedJoint.Define(stage, MOUNT)
    mount.CreateBody0Rel().SetTargets([])  # stationary world
    mount.CreateBody1Rel().SetTargets([body.GetPath()])
    mount.CreateLocalPos0Attr(Gf.Vec3f(position))
    mount.CreateLocalRot0Attr(Gf.Quatf(rotation))
    mount.CreateLocalPos1Attr(Gf.Vec3f(0))
    mount.CreateLocalRot1Attr(Gf.Quatf(1))
    mount.CreateCollisionEnabledAttr(False)
    source_solver = {}
    for kettle_path in KETTLE_BODIES:
        kettle = stage.GetPrimAtPath(kettle_path)
        if not kettle or not kettle.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError("Missing source Kettle rigid body: " + kettle_path)
        source_solver[kettle_path] = {field: dict(authored=kettle.GetAttribute(field).HasAuthoredValueOpinion(),
                                                value=kettle.GetAttribute(field).Get()) for field in SOLVER_ATTRIBUTES}
        kettle.AddAppliedSchema("PhysxRigidBodyAPI")
        for field, value in SOLVER_ATTRIBUTES.items():
            kettle.CreateAttribute(field, Sdf.ValueTypeNames.Int, custom=False).Set(value)
    authoring = validate_local_authoring(stage)
    stage.GetRootLayer().Save()
    info = {
        'recipe': RECIPE, 'recipe_sha256': recipe_sha,
        'source': str(source), 'source_sha256': source_sha,
        'overlay_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'mount_joint': MOUNT, 'mounted_body': HOUSING,
        'world_anchor_position_m': list(position),
        'world_anchor_quaternion_wxyz': [rotation.GetReal(), *rotation.GetImaginary()],
        'assumption': 'Refrigerator housing is installed and anchored at its original authored kitchen placement; this is a local extension assumption, not an upstream constraint.',
        'preserved': ['All source geometry and collision settings', 'All mass, COM and inertia metadata',
                      'Both refrigerator revolute door joints and their original limits',
                      'Original internal fixed joints', 'Gravity on all bodies',
                      'Robot, mustard, bowl, layout relations and task predicate'],
        'runtime_state_writes': False,
        'local_authoring_validation': authoring,
        'kettle_solver': {
            'position_iterations': 64, 'velocity_iterations': 4, 'bodies': list(KETTLE_BODIES),
            'source_attributes': source_solver,
            'world_stability_thresholds': {'translation_m': .002, 'rotation_deg': 1.},
            'selection_scope': 'One G-mustard base source layout and seed42, 10s/1201 post-reset samples per predeclared pair. 64/4 passed original world gates; 16/1 and 32/2 failed and remain retained. Longer neutral/mobility and new complete samples remain required; no general convergence or batch-success claim.',
            'diagnostics': [dict(position_iterations=pos, velocity_iterations=vel, passed=passed,
                directory=f'reports/replicator_execution_20261023/g_mustard/kettle_solver_{pos}_{vel}_01',
                report_sha256=report_sha, physics_states_sha256=raw_sha)
                for pos, vel, passed, report_sha, raw_sha in SOLVER_DIAGNOSTICS],
        },
    }
    metadata.write_text(json.dumps(info, indent=2) + '\n')
    return path, info
