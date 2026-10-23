"""Declared idealized balance and passive damping of the actual oven knob."""
import hashlib
import json
import math
from pathlib import Path


def balanced_temperature_knob(source,output_dir):
    import numpy as np
    from pxr import Gf,Usd,UsdGeom,UsdPhysics
    source=Path(source).resolve()
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    recipe=digest(__file__)
    directory=Path(output_dir)
    directory.mkdir(parents=True,exist_ok=True)
    output=directory/f'temperature_balanced_v1_{digest(source)[:16]}_{recipe[:12]}.usda'
    manifest=output.with_suffix('.json')
    if output.exists() and manifest.exists():
        info=json.loads(manifest.read_text())
        if info['overlay_sha256']!=digest(output):raise ValueError('Changed temperature knob overlay')
        return output,info
    stage=Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths=[str(source)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/world'))
    body=stage.GetPrimAtPath('/world/oven_right_group/Oven038_knob_temp')
    joint=UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(str(body.GetPath())+'/knob_temp_joint'))
    if not body or not joint:raise ValueError('Missing original oven temperature knob')
    anchor=np.asarray(joint.GetLocalPos1Attr().Get(),dtype=float)
    axis=np.asarray(joint.GetLocalRot1Attr().Get().Transform(Gf.Vec3f(1,0,0)),dtype=float)
    axis/=np.linalg.norm(axis)
    cache=UsdGeom.XformCache()
    inverse=cache.GetLocalToWorldTransform(body).GetInverse()
    volumes=[];centers=[]
    for prim in Usd.PrimRange(body):
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():continue
        transform=cache.GetLocalToWorldTransform(prim)*inverse
        if prim.IsA(UsdGeom.Cube):volume=UsdGeom.Cube(prim).GetSizeAttr().Get()**3
        elif prim.IsA(UsdGeom.Cylinder):
            shape=UsdGeom.Cylinder(prim)
            volume=math.pi*shape.GetRadiusAttr().Get()**2*shape.GetHeightAttr().Get()
        else:raise ValueError('Unsupported temperature knob collider '+str(prim.GetPath()))
        volumes.append(volume*abs(transform.GetDeterminant()))
        centers.append(np.asarray(transform.Transform(Gf.Vec3d(0))))
    centroid=np.average(centers,weights=volumes,axis=0)
    balanced=anchor+axis*np.dot(centroid-anchor,axis)
    mass=UsdPhysics.MassAPI.Apply(body)
    original_com=str(mass.GetCenterOfMassAttr().Get())
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*balanced))
    drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular')
    drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(0.)
    drive.CreateDampingAttr(.05*math.pi/180.)
    drive.CreateTargetVelocityAttr(0.);drive.CreateMaxForceAttr(10000.)
    stage.GetRootLayer().Save()
    info={'recipe':'balanced-temperature-knob-v1','recipe_sha256':recipe,'source':str(source),'source_sha256':digest(source),
        'joint':str(joint.GetPath()),'original_com_attribute':original_com,'source_mass_kg':mass.GetMassAttr().Get(),
        'source_collision_centroid_local_m':centroid.tolist(),'balanced_com_local_m':balanced.tolist(),
        'joint_anchor_local_m':anchor.tolist(),'axis_body_local':axis.tolist(),
        'angular_damping_nm_s_per_rad':.05,'angular_stiffness_nm_per_rad':0.,
        'joint_limits_deg':[joint.GetLowerLimitAttr().Get(),joint.GetUpperLimitAttr().Get()],
        'model':'Idealized balanced passive knob, not manufacturer calibration. Original geometry/mass/limits unchanged. '
                'COM projected onto original hinge axis; zero position stiffness, viscous angular damping. No locks or runtime object state setters.',
        'overlay_sha256':digest(output)}
    manifest.write_text(json.dumps(info,indent=2)+'\n')
    return output,info
