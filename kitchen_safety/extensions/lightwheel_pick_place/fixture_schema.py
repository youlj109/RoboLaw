"""Restore authored fixture physics attributes to valid USD schema channels.

No numerical drive/friction values, limits, masses, geometry or DOFs are changed.
"""
from pxr import Usd,UsdPhysics

def canonicalize_drive_channels(stage,root):
    records=[]
    for p in Usd.PrimRange(stage.GetPrimAtPath(root),Usd.TraverseInstanceProxies()):
        if p.IsA(UsdPhysics.RevoluteJoint):axis=str(UsdPhysics.RevoluteJoint(p).GetAxisAttr().Get());canonical='angular'
        elif p.IsA(UsdPhysics.PrismaticJoint):axis=str(UsdPhysics.PrismaticJoint(p).GetAxisAttr().Get());canonical='linear'
        else:continue
        if not p.HasAPI(UsdPhysics.DriveAPI,axis) or p.HasAPI(UsdPhysics.DriveAPI,canonical):continue
        original='drive:'+axis+':';target='drive:'+canonical+':'
        attrs=[a for a in p.GetAttributes() if a.GetName().startswith(original) and a.HasAuthoredValueOpinion()]
        UsdPhysics.DriveAPI.Apply(p,canonical);values={}
        for a in attrs:
            suffix=a.GetName()[len(original):];value=a.Get();p.CreateAttribute(target+suffix,a.GetTypeName(),custom=False).Set(value);values[suffix]=value
        records.append({'joint':str(p.GetPath()),'source_instance':axis,'canonical_instance':canonical,'unchanged_authored_values':values})
    return records

def repair_fixture_schemas(stage,root):
    from pxr import PhysxSchema
    records=canonicalize_drive_channels(stage,root)
    for p in Usd.PrimRange(stage.GetPrimAtPath(root),Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdPhysics.Joint):continue
        friction=p.GetAttribute('physxJoint:jointFriction')
        if friction and friction.HasAuthoredValueOpinion() and not p.HasAPI(PhysxSchema.PhysxJointAPI):
            value=friction.Get();PhysxSchema.PhysxJointAPI.Apply(p)
            if friction.Get()!=value:raise RuntimeError('Friction value changed unexpectedly')
            records.append({'joint':str(p.GetPath()),'schema_applied':'PhysxJointAPI','unchanged_joint_friction':value})
    return records

def install_fixture_schema_repair(scene_cfg):
    original=scene_cfg.kitchen.spawn.func;records=[]
    def spawn(*args,**kwargs):
        prim=original(*args,**kwargs);records.extend(repair_fixture_schemas(prim.GetStage(),str(prim.GetPath())));return prim
    scene_cfg.kitchen.spawn.func=spawn
    return records
