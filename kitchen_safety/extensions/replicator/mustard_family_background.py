"""Topology-specific background overlays for the four additional mustards.

This file does not alter the existing G recipe. Only original KettleA uses its
already-disclosed 64/4 solver recipe. Kitchens without it (including KettleB)
keep every source solver value. Each new kitchen still requires independent
neutral/mobility physics evidence with the original tolerances.
"""
import hashlib
import json
from pathlib import Path
from .background_physics import HOUSING,MOUNT,KETTLE_BODIES,RECIPE,mounted_kitchen

HOUSING_ONLY_RECIPE='additional-mustard-fixed-refrigerator-housing-only-v1'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inspect_background_profile(source):
    from pxr import Usd,UsdPhysics
    stage=Usd.Stage.Open(str(source))
    if not stage:raise ValueError('Cannot open original background')
    housing=stage.GetPrimAtPath(HOUSING)
    if not housing or not housing.HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Original Fridge housing missing')
    kettle_a={path:bool(stage.GetPrimAtPath(path) and stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI)) for path in KETTLE_BODIES}
    if any(kettle_a.values()) and not all(kettle_a.values()):raise ValueError('Partial KettleA topology must be investigated')
    kettle_inventory=[]
    for prim in stage.Traverse():
        if 'Kettle' in str(prim.GetPath()) and (prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.IsA(UsdPhysics.Joint)):
            kettle_inventory.append(dict(path=str(prim.GetPath()),type=prim.GetTypeName(),rigid_body=prim.HasAPI(UsdPhysics.RigidBodyAPI)))
    return dict(recipe=RECIPE if all(kettle_a.values()) else HOUSING_ONLY_RECIPE,
        topology_applicable=True,source_housing=HOUSING,kettle_a_bodies=kettle_a,kettle_source_inventory=kettle_inventory,
        solver_edit_bodies=list(KETTLE_BODIES) if all(kettle_a.values()) else [],
        recipe_code_sha256=sha(__file__),existing_g_recipe_code_sha256=sha(Path(__file__).with_name('background_physics.py')),
        new_layout_neutral_validated=False,new_layout_mobility_validated=False,
        scope='Housing installation is an explicit local assumption at the original authored pose. Existing KettleA only gets the disclosed 64/4 recipe; absent KettleA and distinct KettleB receive no solver or joint edits. G results do not validate this new layout.')


def validate_housing_only(stage):
    from pxr import Sdf
    layer=stage.GetRootLayer();attrs=set();rels=set()
    required={'physics:localPos0','physics:localPos1','physics:localRot0','physics:localRot1','physics:collisionEnabled'}
    def inspect(path):
        spec=layer.GetObjectAtPath(path)
        if isinstance(spec,Sdf.AttributeSpec):
            if str(spec.owner.path)!=MOUNT or spec.name not in required:raise ValueError('Unexpected local attribute: '+str(path))
            attrs.add(spec.name)
        elif isinstance(spec,Sdf.RelationshipSpec):
            if str(spec.owner.path)!=MOUNT or spec.name not in {'physics:body0','physics:body1'}:raise ValueError('Unexpected local relationship: '+str(path))
            rels.add(spec.name)
        elif isinstance(spec,Sdf.PrimSpec):
            allowed={'subLayers','subLayerOffsets','defaultPrim'} if path==Sdf.Path.absoluteRootPath else {'specifier'}
            if str(path)==MOUNT:
                allowed.add('typeName')
                if spec.typeName!='PhysicsFixedJoint':raise ValueError('Housing installation type changed')
            if set(spec.ListInfoKeys())-allowed:raise ValueError('Unexpected local metadata: '+str(path))
    layer.Traverse(Sdf.Path.absoluteRootPath,inspect)
    if attrs!=required or rels!={'physics:body0','physics:body1'}:raise ValueError('Incomplete housing-only overlay')
    return dict(passed=True,housing_mount=MOUNT,attributes=sorted(attrs),relationships=sorted(rels),
        scope='Only one fixed installation on original refrigerator housing; no source geometry, transforms, joints, mass, collision, drive or solver changes')


def family_mounted_kitchen(source,output_dir):
    from pxr import Gf,Usd,UsdGeom,UsdPhysics
    source=Path(source).resolve();output_dir=Path(output_dir)
    profile=inspect_background_profile(source)
    if profile['recipe']==RECIPE:
        path,info=mounted_kitchen(source,output_dir)
        return path,dict(info,family_recipe_selection=profile)
    output_dir.mkdir(parents=True,exist_ok=True)
    source_hash=sha(source);recipe_hash=sha(__file__)
    path=output_dir/f'{HOUSING_ONLY_RECIPE}_{source_hash[:16]}_{recipe_hash[:12]}.usda'
    metadata=path.with_suffix('.json')
    source_stage=Usd.Stage.Open(str(source))
    layers={str(Path(layer.realPath).resolve()):sha(layer.realPath) for layer in source_stage.GetUsedLayers() if Path(layer.realPath).is_file()}
    if path.exists() or metadata.exists():
        if not path.is_file() or not metadata.is_file():raise ValueError('Incomplete prior overlay; retain failure and choose a new output directory')
        info=json.loads(metadata.read_text())
        if info['overlay_sha256']!=sha(path) or info['source_sha256']!=source_hash or info['recipe_sha256']!=recipe_hash or info['source_layer_sha256']!=layers:
            raise ValueError('Cached overlay/source/recipe changed')
        validate_housing_only(Usd.Stage.Open(str(path)))
        return path,info
    frame=UsdGeom.XformCache().GetLocalToWorldTransform(source_stage.GetPrimAtPath(HOUSING))
    position=frame.ExtractTranslation();rotation=frame.ExtractRotationQuat()
    stage=Usd.Stage.CreateNew(str(path));stage.GetRootLayer().subLayerPaths=[str(source)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    joint=UsdPhysics.FixedJoint.Define(stage,MOUNT)
    joint.CreateBody0Rel().SetTargets([]);joint.CreateBody1Rel().SetTargets([HOUSING])
    joint.CreateLocalPos0Attr(Gf.Vec3f(position));joint.CreateLocalRot0Attr(Gf.Quatf(rotation))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot1Attr(Gf.Quatf(1));joint.CreateCollisionEnabledAttr(False)
    authoring=validate_housing_only(stage);stage.GetRootLayer().Save()
    info=dict(recipe=HOUSING_ONLY_RECIPE,recipe_sha256=recipe_hash,source=str(source),source_sha256=source_hash,
        source_layer_sha256=layers,overlay_sha256=sha(path),mount_joint=MOUNT,mounted_body=HOUSING,
        world_anchor_position_m=list(position),world_anchor_quaternion_wxyz=[rotation.GetReal(),*rotation.GetImaginary()],
        assumption='Fridge housing mounted at original authored pose; a disclosed local installation assumption, not an upstream constraint.',
        preserved=['All source geometry, transforms and collision settings','All source joints, drives, mass, COM and inertia',
                   'Every source solver value, including distinct KettleB','Both original refrigerator door hinges and limits'],
        runtime_state_writes=False,local_authoring_validation=authoring,family_recipe_selection=profile,
        neutral_validated=False,mobility_validated=False,execution_ready=False)
    metadata.write_text(json.dumps(info,indent=2,allow_nan=False)+'\n')
    return path,info
