import hashlib

import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from kitchen_safety.extensions.controls.oven_physics import damped_oven_racks


def source(tmp_path):
    path=tmp_path/'source.usda'
    stage=Usd.Stage.CreateNew(str(path))
    root=UsdGeom.Xform.Define(stage,'/world')
    stage.SetDefaultPrim(root.GetPrim())
    for i in (0,1):
        body=UsdGeom.Xform.Define(stage,f'/world/oven_right_group/Oven038_rack{i}')
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(.4)
        joint=UsdPhysics.PrismaticJoint.Define(stage,str(body.GetPath())+f'/rack{i}_joint')
        joint.CreateAxisAttr('X')
        joint.CreateLowerLimitAttr(0.)
        joint.CreateUpperLimitAttr(.245)
        joint.CreateLocalPos1Attr(Gf.Vec3f(.01,.02,.03))
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'X').CreateDampingAttr(1.)
    stage.GetRootLayer().Save()
    return path


def test_passive_damping_does_not_lock_relocate_or_relimit_racks(tmp_path):
    original=source(tmp_path)
    before=hashlib.sha256(original.read_bytes()).hexdigest()
    overlay,info=damped_oven_racks(original,tmp_path/'overlays')
    assert hashlib.sha256(original.read_bytes()).hexdigest()==before
    stage=Usd.Stage.Open(str(overlay))
    for i in (0,1):
        body=stage.GetPrimAtPath(f'/world/oven_right_group/Oven038_rack{i}')
        joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(str(body.GetPath())+f'/rack{i}_joint'))
        assert joint.GetUpperLimitAttr().Get()==pytest.approx(.245)
        assert joint.GetLowerLimitAttr().Get()==0
        assert joint.GetLocalPos1Attr().Get()==Gf.Vec3f(.01,.02,.03)
        assert UsdPhysics.MassAPI(body).GetMassAttr().Get()==pytest.approx(.4)
        drive=UsdPhysics.DriveAPI(joint.GetPrim(),'linear')
        assert drive.GetStiffnessAttr().Get()==0
        assert drive.GetDampingAttr().Get()==1
    assert info['source_sha256']==before


def test_modified_overlay_is_rejected(tmp_path):
    original=source(tmp_path)
    overlay,_=damped_oven_racks(original,tmp_path/'overlays')
    overlay.write_text(overlay.read_text()+'\n')
    with pytest.raises(ValueError,match='Changed oven'):
        damped_oven_racks(original,tmp_path/'overlays')
