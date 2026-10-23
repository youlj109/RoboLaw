"""Mount the kitchen's refrigerator housing while retaining its free doors.

This is a local fixed-installation assumption, not an upstream asset property.
The source refrigerator tips by about 0.4 degrees on its SDF floor contacts;
its un-damped hinges then convert panel gravity into autonomous door opening.
Only the housing-to-world mounting constraint is added. No runtime state writes,
collision filters, mass edits, door drives, or fixed door joints are introduced.
"""
from pathlib import Path
import hashlib
import json

RECIPE = "fixed-refrigerator-housing-v1"
HOUSING = "/World/Fridge/Fridge_OnlineVisual/Geometry/sm_rc_refrigerator_a01_body_01_obj_00"
MOUNT = "/World/Fridge/RoboLawHousingInstallation"


def mounted_kitchen(source, output_dir):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

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
    }
    metadata.write_text(json.dumps(info, indent=2) + '\n')
    return path, info
