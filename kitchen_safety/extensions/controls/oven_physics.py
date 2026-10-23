"""Explicit passive rack damping; no locks, position servos or state setters."""
import hashlib
import json
from pathlib import Path


def damped_oven_racks(source, output_dir):
    from pxr import Usd, UsdPhysics
    source=Path(source).resolve()
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    recipe=digest(__file__)
    output_dir=Path(output_dir)
    output_dir.mkdir(parents=True,exist_ok=True)
    output=output_dir/f'oven_passive_racks_v1_{digest(source)[:16]}_{recipe[:12]}.usda'
    manifest=output.with_suffix('.json')
    if output.exists() and manifest.exists():
        info=json.loads(manifest.read_text())
        if info['overlay_sha256']!=digest(output):raise ValueError('Changed oven rack overlay')
        return output,info
    stage=Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths=[str(source)]
    stage.SetDefaultPrim(stage.GetPrimAtPath('/world'))
    info={'recipe':'passive-oven-rack-damping-v1','source':str(source),'source_sha256':digest(source),
        'recipe_sha256':recipe,'joints':[],
        'model':'Idealized viscous drawer rail, 1 N s/m. Original X drive is not a USD prismatic linear drive. '
                'Author a correctly named passive linear damper with zero stiffness; retain original mass, '
                'axis, COM, geometry and limits. No positional servo, lock or runtime joint setter. '
                'This is a declared passive model, not manufacturer calibration.'}
    for i in (0,1):
        path=f'/world/oven_right_group/Oven038_rack{i}/rack{i}_joint'
        joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(path))
        if not joint:raise ValueError('Missing source rack '+path)
        old=UsdPhysics.DriveAPI(joint.GetPrim(),'X')
        info['joints'].append({'path':path,'limits_m':[joint.GetLowerLimitAttr().Get(),joint.GetUpperLimitAttr().Get()],
            'original_X_damping':old.GetDampingAttr().Get(),'linear_damping_n_s_per_m':1.,'linear_stiffness_n_per_m':0.})
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(1.)
        drive.CreateTargetVelocityAttr(0.)
        drive.CreateMaxForceAttr(10000.)
    stage.GetRootLayer().Save()
    info['overlay_sha256']=digest(output)
    manifest.write_text(json.dumps(info,indent=2)+'\n')
    return output,info
